#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use nalgebra::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

const PLATFORM_HEIGHT: f32 = 2.0;
const PLATFORM_HFTHICKNESS: f32 = 0.2;
const PLATFORM_HFSIDE: f32 = 2.0;
const CYLINDER_HFHEIGHT: f32 = 0.6;
const CYLINDER_RADIUS: f32 = 0.2;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground.
     */
    let ground_size = 10.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the pillars
     */
    let num_pillars: usize = 12;
    let radius = 1.5;
    let platform_top = PLATFORM_HEIGHT + PLATFORM_HFTHICKNESS;
    let pillar_y = platform_top + CYLINDER_HFHEIGHT;
    let bulkhead_y = platform_top + 2.0 * CYLINDER_HFHEIGHT + CYLINDER_RADIUS;
    let sector = 2.0 * std::f32::consts::PI / num_pillars as f32;

    for i in 0..num_pillars {
        let angle = i as f32 * sector;
        let x = angle.cos() * radius;
        let z = angle.sin() * radius;

        let pillar_body = RigidBodyBuilder::new_dynamic()
            .translation(x, pillar_y, z)
            .mass(1.0, true)
            .build();
        let pillar_handle = bodies.insert(pillar_body);
        let pillar_collider =
            ColliderBuilder::round_cylinder(CYLINDER_HFHEIGHT, CYLINDER_RADIUS, 0.02).build();
        colliders.insert(pillar_collider, pillar_handle, &mut bodies);

        if i % 2 == 1 {
            // TODO: avoid recomputing these values.
            let angle0 = (i - 1) as f32 * sector;
            let x0 = angle0.cos() * radius;
            let z0 = angle0.sin() * radius;

            let a = x0 - x;
            let b = z - z0;
            let bulkhead_angle = b.atan2(a);
            println!("bulkhead angle {}", bulkhead_angle);
            let bulkhead_body = RigidBodyBuilder::new_dynamic()
                .translation((x0 + x) / 2.0, bulkhead_y, (z0 + z) / 2.0)
                .rotation(Vector3::y() * bulkhead_angle)
                .mass(1.0, true)
                .build();
            let bulkhead_handle = bodies.insert(bulkhead_body);
            let bulkhead_length = sector;
            let bulkhead_thickness = CYLINDER_RADIUS;
            let bulkhead_width = CYLINDER_RADIUS * 1.1;
            let bulkhead_collider =
                ColliderBuilder::cuboid(bulkhead_length, bulkhead_thickness, bulkhead_width)
                    .build();
            colliders.insert(bulkhead_collider, bulkhead_handle, &mut bodies);
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::new_kinematic()
        .translation(0.0, PLATFORM_HEIGHT, 0.0)
        .build();
    let platform_handle = bodies.insert(platform_body);
    let collider =
        ColliderBuilder::cuboid(PLATFORM_HFSIDE, PLATFORM_HFTHICKNESS, PLATFORM_HFSIDE).build();
    colliders.insert(collider, platform_handle, &mut bodies);

    /*
     * Setup a callback to control the platform.
     */
    let mut count = 0;
    testbed.add_callback(move |_, physics, _, _, time| {
        count += 1;
        if count % 100 > 10 {
            return;
        }

        if let Some(platform) = physics.bodies.get_mut(platform_handle) {
            let mut next_pos = *platform.position();

            let dt = 0.002;
            next_pos.translation.vector.y += (time * 10.0).sin() * dt;
            next_pos.translation.vector.z += time.sin() * 10.0 * dt;

            if next_pos.translation.vector.z >= 2.0 {
                next_pos.translation.vector.z -= dt;
            }
            if next_pos.translation.vector.z <= -2.0 {
                next_pos.translation.vector.z += dt;
            }

            platform.set_next_kinematic_position(next_pos);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_body_color(platform_handle, Point3::new(0.0, 1.0, 0.0));
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(-5.0, 10.0, -10.0), Point3::origin());
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
fn main() {
    let testbed = Testbed::from_builders(0, vec![("Stonehenge", init_world)]);
    testbed.run()
}
