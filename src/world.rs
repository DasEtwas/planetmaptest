use crate::rapier::dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodySet};
use crate::rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use crate::rapier::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use crate::rapier::pipeline::{PhysicsPipeline, QueryPipeline};
use crate::Vector3D;

pub struct PhysicsWorld {
    pub pipeline: PhysicsPipeline,
    pub integration_parameters: IntegrationParameters,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub joints: JointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        PhysicsWorld {
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::with_query_dispatcher(
                planetmap::parry::PlanetDispatcher.chain(DefaultQueryDispatcher),
            ),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            joints: JointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::with_query_dispatcher(
                planetmap::parry::PlanetDispatcher.chain(DefaultQueryDispatcher),
            ),
        }
    }
}

impl PhysicsWorld {
    pub fn step(&mut self, timestep: f64) {
        //TODO inspect use or need of IntegrationParameters::min_ccd_dt
        self.integration_parameters.dt = timestep;

        {
            eprintln!("PRESTEP");

            self.bodies.iter_mut().for_each(|(_handle, body)| {
                if !body.is_static() {
                    let pos = body.position().translation.vector;
                    let vel = *body.linvel();
                    eprintln!(
                        "body pos: [{:>8.3}, {:>8.3}, {:>8.3}], vel: [{:>8.3}, {:>8.3}, {:>8.3}]",
                        pos.x, pos.y, pos.z, vel.x, vel.y, vel.z
                    );
                }
            });

            /*self.colliders.iter().for_each(|(_handle, collider)| {
                let pos = collider.position().translation.vector;
                eprintln!(
                    "collider pos: [{:>8.3}, {:>8.3}, {:>8.3}]",
                    pos.x, pos.y, pos.z,
                );
            });*/
        }

        self.pipeline.step(
            // spaaaaaace
            &Vector3D::zeros(),
            &self.integration_parameters,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }
}
