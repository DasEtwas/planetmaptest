use crate::rapier::na::{convert, Isometry3, Point2, Point3, Vector2};
use crate::terrain::PlanetTerrain;
use crate::{Point3D, Vector3D, Vector3F};
use image::ImageBuffer;
use kiss3d::context::Texture;
use kiss3d::ncollide3d::procedural::{IndexBuffer, TriMesh};
use kiss3d::resource::TextureManager;
use kiss3d::window::Window;
use planetmap::cubemap::Coords;
use planetmap::parry::Terrain;
use std::rc::Rc;

pub fn generate_planet_chunk_models(
    window: &mut Window,
    resolution: u32,
    terrain: &PlanetTerrain,
    dir: Vector3D,
    render_origin: Isometry3<f64>,
    texture: Rc<Texture>,
) {
    let base = Coords::from_vector(terrain.face_resolution(), &convert(dir));

    let neighbour_count = 7;

    let mut coords = vec![base];

    for _ in 0..neighbour_count {
        for chunk in coords.clone() {
            coords.extend_from_slice(&chunk.neighbors(terrain.face_resolution()));
        }
    }

    // dedup
    {
        let mut i = coords.len() - 1;
        while i != 0 {
            let mut j = i - 1;
            while j != 0 {
                if coords[i] == coords[j] {
                    coords.remove(i);
                    break;
                }
                j -= 1;
            }
            i -= 1;
        }
    }

    let mut heights = vec![0.0; resolution as usize * resolution as usize];

    for chunk_coords in &coords {
        terrain.sample(resolution, &chunk_coords, &mut heights);

        let mut chunk = window.add_trimesh(
            TriMesh::new(
                chunk_coords
                    .samples(terrain.face_resolution(), resolution)
                    .zip(heights.iter())
                    .map(|(dir, height)| {
                        convert(
                            render_origin.inverse()
                                * Point3D::from(
                                    dir.into_inner().map(|x| x as f64)
                                        * (terrain.min_radius() + *height as f64),
                                ),
                        )
                    })
                    .collect(),
                None,
                Some(
                    (0..resolution.pow(2))
                        .map(|i| {
                            let u = (i % resolution) as f32 / (resolution - 1) as f32;
                            let v = (i / resolution) as f32 / (resolution - 1) as f32;
                            Point2::new(u, v)
                        })
                        .collect(),
                ),
                Some(IndexBuffer::Unified(
                    (0..(resolution - 1).pow(2))
                        .flat_map(|quad_index| {
                            let row_index = quad_index / (resolution - 1);

                            vec![0, 1, resolution, 1, resolution + 1, resolution]
                                .into_iter()
                                .map(move |index| {
                                    index + row_index * resolution + quad_index % (resolution - 1)
                                })
                        })
                        .collect::<Vec<u32>>()
                        .chunks(3)
                        .map(|slice| Point3::new(slice[0], slice[1], slice[2]))
                        .collect(),
                )),
            ),
            Vector3F::repeat(1.0),
        );
        chunk.enable_backface_culling(false);
        chunk.set_texture(texture.clone());
    }
}

pub fn generate_grid_texture(texture_resolution: u32, chunk_resolution: u32) -> Rc<Texture> {
    TextureManager::new().add_image(
        image::DynamicImage::ImageRgb8(
            ImageBuffer::from_raw(
                texture_resolution,
                texture_resolution,
                (0..texture_resolution.pow(2))
                    .flat_map(|i| {
                        let u =
                            (i % (texture_resolution)) as f32 / ((texture_resolution) - 1) as f32;
                        let v =
                            (i / (texture_resolution)) as f32 / ((texture_resolution) - 1) as f32;

                        // fade to 0
                        let steepness = 8.0;
                        // line thickness
                        let thickness = 0.04;

                        // sdf to a grid
                        let value = (210.0
                            * (1.0
                                - ((1.0
                                    - Vector2::new(
                                        (1.0 + thickness
                                            - 2.0
                                                * ((u * (chunk_resolution - 1) as f32 - 0.5)
                                                    .rem_euclid(1.0)
                                                    - 0.5)
                                                    .abs())
                                        .min(1.0)
                                        .max(0.0),
                                        (1.0 + thickness
                                            - 2.0
                                                * ((v * (chunk_resolution - 1) as f32 - 0.5)
                                                    .rem_euclid(1.0)
                                                    - 0.5)
                                                    .abs())
                                        .min(1.0)
                                        .max(0.0),
                                    )
                                    .max())
                                    * steepness)
                                    .min(1.0))) as u8;

                        vec![(30 + value) as u8, (40 + value) as u8, (30 + value) as u8]
                    })
                    .collect(),
            )
            .unwrap(),
        ),
        "idk",
    )
}
