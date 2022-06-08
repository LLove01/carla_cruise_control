import glob
import os
import sys
import numpy as np

from control_node import process_fw, process_r

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def reset(self):
    print("Collision! Respawning ...")
    self.vehicle_bp = self.bp_lib.filter('cybertruck')[0]
    self.spawn_point = self.world.get_map().get_spawn_points()[4]
    self.vehicle = self.world.spawn_actor(
        self.vehicle_bp, self.spawn_point)
    self.actor_list.append(self.vehicle)
    print("Car spawned")
    # FORWARD CAMERA
    fw_cam_bp = self.bp_lib.find('sensor.camera.rgb')
    # change the dimensions of the image
    fw_cam_bp.set_attribute('image_size_x', '800')
    fw_cam_bp.set_attribute('image_size_y', '600')
    fw_cam_bp.set_attribute('fov', '90')
    # Adjust sensor relative to vehicle
    fw_cam_transf = carla.Transform(carla.Location(x=1.5, z=2.4))
    # spawn the sensor and attach to vehicle.
    self.fw_cam = self.world.spawn_actor(
        fw_cam_bp, fw_cam_transf, attach_to=self.vehicle)
    # add sensor to list of actors
    self.actor_list.append(self.fw_cam)
    # do something with this sensor
    self.fw_cam.listen(lambda data: process_fw(data))

    # RIGHT SIDE CAMERA
    r_cam_bp = self.bp_lib.find('sensor.camera.rgb')
    r_cam_bp.set_attribute('image_size_x', '800')
    r_cam_bp.set_attribute('image_size_y', '600')
    r_cam_bp.set_attribute('fov', '90')
    # Adjust sensor relative to vehicle
    r_cam_loc = carla.Location(0, 1.48, 2.4)
    r_cam_rot = carla.Rotation(-62, 90, 0)
    r_cam_transf = carla.Transform(r_cam_loc, r_cam_rot)
    # spawn the sensor and attach to vehicle.
    self.r_cam = self.world.spawn_actor(
        r_cam_bp, r_cam_transf, attach_to=self.vehicle)
    # add sensor to list of actors
    self.actor_list.append(self.r_cam)
    # do something with this sensor
    self.r_cam.listen(lambda data: process_r(data))

    # COLLISION SENSOR
    collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
    self.collision_sensor = self.world.spawn_actor(
        collision_bp, carla.Transform(), attach_to=self.vehicle)
    self.collision_sensor.listen(lambda event: on_collision(event, self))
    self.actor_list.append(self.collision_sensor)


def on_collision(event, self):
    self.client.apply_batch(
        [carla.command.DestroyActor(x) for x in self.actor_list])
    reset(self)
