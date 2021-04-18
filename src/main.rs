pub mod terrain;
pub mod utils;
pub mod world;

use na::*;
pub use rapier::na;
pub use rapier::parry;
pub use rapier3d_f64 as rapier;

pub type Point3D = Point3<f64>;
pub type Point3F = Point3<f32>;
pub type Vector3D = Vector3<f64>;
pub type Vector3F = Vector3<f32>;

use crate::rapier::dynamics::RigidBodyBuilder;
use crate::rapier::geometry::{ColliderBuilder, Compound, Cuboid, Shape, SharedShape};
use crate::terrain::PlanetTerrain;
use crate::world::PhysicsWorld;

use kiss3d::event::{Action, Event, Key, WindowEvent};
use kiss3d::light::Light;

use kiss3d::window::Window;
use noise::{MultiFractal, Seedable};

use spin_sleep::LoopHelper;

use crate::utils::{generate_grid_texture, generate_planet_chunk_models};
use kiss3d::camera::Camera;
use std::collections::HashSet;
use std::sync::Arc;

fn main() {
    let mut world = PhysicsWorld::default();

    let planet_radius = 637100.0;

    // smooth
    let noise = noise::Fbm::new()
        .set_seed(6557)
        .set_octaves(14)
        .set_frequency(1.0 / 64.0)
        .set_persistence(0.687);

    // jagged
    /*let noise = noise::Fbm::new()
       .set_seed(6557)
       .set_octaves(20)
       .set_frequency(1.0 / 64.0)
       .set_persistence(0.687);
    */

    let terrain = Arc::new(PlanetTerrain::new(planet_radius, 10000.0, 16, noise));

    let resolution = 4;

    let planet = planetmap::parry::Planet::new(terrain.clone(), 256, planet_radius, resolution);

    // planet
    {
        let rb = RigidBodyBuilder::new_static()
            .position(Isometry3::identity())
            .build();

        let rb_handle = world.bodies.insert(rb);

        world.colliders.insert(
            ColliderBuilder::new(SharedShape::new(planet)).build(),
            rb_handle,
            &mut world.bodies,
        );
    }

    // random
    let dir = Vector3D::new(0.435, 0.12, -0.5).normalize();

    let cube_extents = Vector3D::new(0.25, 5.0, 5.0);

    // our test subject
    let (render_origin, subject_handle): (Isometry3<f64>, _) = {
        let shape = Compound::new(vec![(
            Isometry3::identity(),
            SharedShape::new(Cuboid::new(Vector3D::new(
                cube_extents.x,
                cube_extents.y,
                cube_extents.z,
            ))),
        )]);

        let collider_radius = {
            let bs = shape.compute_local_bounding_sphere();
            bs.radius + bs.center.coords.norm()
        };

        let collider = ColliderBuilder::new(SharedShape::new(shape)).build();

        let body_pos = Translation3::from(
            dir * (planet_radius
                /* >1.0 factor for slope tolerance */
                + collider_radius * 1.3
                + terrain.height_at(&Unit::new_normalize(dir.map(|f| f as f32)))),
        )
        .into();

        let rb = RigidBodyBuilder::new_dynamic()
            .position(body_pos)
            .rotation(/* some random rotation */ Vector3D::new(1.5, 0.2, 3.0))
            .build();

        let rb_handle = world.bodies.insert(rb);

        world
            .colliders
            .insert(collider, rb_handle, &mut world.bodies);

        (
            Isometry3::from_parts(
                Translation3::from(
                    dir * (planet_radius
                        + 2.0
                        + terrain.height_at(&Unit::new_normalize(dir.map(|f| f as f32)))),
                ),
                UnitQuaternion::rotation_between_axis(
                    &Vector3D::y_axis(),
                    &Unit::new_normalize(dir),
                )
                .unwrap(),
            ),
            rb_handle,
        )
    };

    let mut window = Window::new("Kiss3d: cube");
    let mut collider = window.add_cube(
        cube_extents.x as f32 * 2.0,
        cube_extents.y as f32 * 2.0,
        cube_extents.z as f32 * 2.0,
    );

    collider.set_color(1.0, 0.0, 0.0);

    let texture = generate_grid_texture(2048, resolution);

    generate_planet_chunk_models(
        &mut window,
        resolution,
        &terrain,
        dir,
        render_origin,
        texture.clone(),
    );

    window.set_light(Light::StickToCamera);

    let mut cam = kiss3d::camera::ArcBall::new(
        convert::<Point3D, Point3F>(
            (dir.cross(&Vector3D::y_axis()).cross(&Vector3D::z_axis())).into(),
        ),
        Point3F::origin(),
    );

    let delta_time = 1.0 / 42.0;
    let mut loop_helper = LoopHelper::builder().build_with_target_rate(1.0 / delta_time as f32);

    let mut keys_down = HashSet::new();

    while window.render_with_camera(&mut cam) {
        loop_helper.loop_sleep();
        loop_helper.loop_start();

        world.step(delta_time);

        collider.set_local_transformation(convert(
            render_origin.inverse() * world.bodies.get(subject_handle).unwrap().position(),
        ));

        {
            let v = Vector3D::new(
                if keys_down.get(&Key::Right).is_some() {
                    1.0
                } else if keys_down.get(&Key::Left).is_some() {
                    -1.0
                } else {
                    0.0
                },
                if keys_down.get(&Key::Up).is_some() {
                    1.0
                } else if keys_down.get(&Key::Down).is_some() {
                    -1.0
                } else {
                    0.0
                },
                0.0,
            );

            let body = world.bodies.get_mut(subject_handle).unwrap();

            body.apply_force(
                render_origin
                    * convert::<_, Isometry3<f64>>(cam.view_transform().inverse())
                    * (v * body.mass() * 20.0),
                true,
            );
        }

        window.events().iter().for_each(|e| match e {
            Event { value, .. } => match value {
                WindowEvent::Key(key, action, _) => match action {
                    Action::Release => {
                        keys_down.remove(&key);
                    }
                    Action::Press => {
                        keys_down.insert(key);
                    }
                },
                _ => (),
            },
        });
    }
}
