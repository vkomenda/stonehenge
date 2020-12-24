#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use nalgebra::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

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
    let num: usize = 6;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift / 2.0 + 3.04;
    let centerz = shift * num as f32 / 2.0;
    let sector = std::f32::consts::PI * 2.0 / num as f32;
    let radius = 1.0;

    for i in 0..num {
        let angle = i as f32 * sector;
        let x = angle.cos() * radius - centerx + radius;
        let y = centery;
        let z = angle.sin() * radius - centerz - radius;

        let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::round_cylinder(rad * 3.0, rad, rad / 10.0).build();
        colliders.insert(collider, handle, &mut bodies);
    }

    /*
     * Setup a kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::new_kinematic()
        .translation(0.0, 1.5 + 0.8, -10.0 * rad)
        .build();
    let platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad, rad * 10.0).build();
    colliders.insert(collider, platform_handle, &mut bodies);

    /*
     * Setup a callback to control the platform.
     */
    let mut count = 0;
    testbed.add_callback(move |_, physics, _, _, time| {
        count += 1;
        if count % 100 > 50 {
            return;
        }

        if let Some(platform) = physics.bodies.get_mut(platform_handle) {
            let mut next_pos = *platform.position();

            let dt = 0.0016;
            next_pos.translation.vector.y += (time * 5.0).sin() * dt;
            next_pos.translation.vector.z += time.sin() * 5.0 * dt;

            if next_pos.translation.vector.z >= rad * 10.0 {
                next_pos.translation.vector.z -= dt;
            }
            if next_pos.translation.vector.z <= -rad * 10.0 {
                next_pos.translation.vector.z += dt;
            }

            platform.set_next_kinematic_position(next_pos);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(-10.0, 5.0, -10.0), Point3::origin());
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
fn main() {
    let testbed = Testbed::from_builders(0, vec![("Stonehenge", init_world)]);
    testbed.run()
}
