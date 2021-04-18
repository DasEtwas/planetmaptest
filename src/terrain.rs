////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2021 DasEtwas - All Rights Reserved                               /
//      Unauthorized copying of this file, via any medium is strictly prohibited   /
//      Proprietary and confidential                                               /
////////////////////////////////////////////////////////////////////////////////////

use crate::na::{convert, Point, Unit};
use crate::{Point3D, Vector3D, Vector3F};
use noise::NoiseFn;
use planetmap::{cubemap::Coords, parry::Terrain};

#[derive(Clone)]
pub struct PlanetTerrain {
    height: f64,
    min_radius: f64,
    depth: u8,
    noise: noise::Fbm,
}

impl PlanetTerrain {
    pub fn new(min_radius: f64, height: f64, depth: u8, noise: noise::Fbm) -> Self {
        PlanetTerrain {
            height,
            min_radius,
            depth,
            noise,
        }
    }

    pub fn max_radius(&self) -> f64 {
        self.min_radius + self.height
    }

    pub fn min_radius(&self) -> f64 {
        self.min_radius
    }

    pub fn depth(&self) -> u8 {
        self.depth
    }

    /// Radial heightmap function
    pub fn height_at(&self, dir: &Unit<Vector3F>) -> f64 {
        let p = Point::from(convert::<_, Vector3D>(dir.into_inner()) * self.min_radius);
        self.sample(p)
    }

    fn sample(&self, p: Point3D) -> f64 {
        let point: [f64; 3] = (p.coords * 5e-5).into();
        let lat = (p.y / self.min_radius as f64).abs();
        let h = ((1.0 + self.noise.get(point)).powi(3) - 1.0) * 1500.0 + (lat - 0.3) * 3000.0;
        h.max(self.min_height() as f64)
            .min(self.max_height() as f64)
    }
}

impl Terrain for PlanetTerrain {
    fn sample(&self, resolution: u32, coords: &Coords, out: &mut [f32]) {
        for (out, sample) in out
            .iter_mut()
            .zip(coords.samples(self.face_resolution(), resolution))
        {
            *out = self.height_at(&sample) as f32;
        }
    }

    fn face_resolution(&self) -> u32 {
        2u32.pow(self.depth as u32)
    }

    fn max_height(&self) -> f32 {
        self.height as f32
    }

    fn min_height(&self) -> f32 {
        0.0
    }
}
