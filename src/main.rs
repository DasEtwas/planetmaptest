pub mod terrain;
pub mod world;

use na::*;
pub use rapier::na;
pub use rapier::parry;
pub use rapier3d_f64 as rapier;

pub type Point3D = Point3<f64>;
pub type Vector3D = Vector3<f64>;
pub type Vector3F = Vector3<f32>;

use crate::rapier::dynamics::RigidBodyBuilder;
use crate::rapier::geometry::{ColliderBuilder, SharedShape};
use crate::terrain::PlanetTerrain;
use crate::world::PhysicsWorld;
use noise::{MultiFractal, Seedable};
use std::sync::Arc;

fn main() {
    let mut world = PhysicsWorld::default();

    let radius = 637100.0;

    let planet = planetmap::parry::Planet::new(
        Arc::new(PlanetTerrain::new(
            radius,
            10000.0,
            16,
            noise::Fbm::new()
                .set_seed(6557)
                .set_octaves(14)
                .set_frequency(1.0 / 64.0)
                .set_persistence(0.687),
        )),
        256,
        radius,
        8,
    );

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

    loop {
        world.step(1.0 / 42.0);
    }
}
