import glob
import os
import sys
import numpy as np

from control_node import process_image

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
    # spawn_point = carla.Transform(carla.Location(
    #     x=-75.4, y=-1.0, z=15), carla.Rotation(pitch=0, yaw=180, roll=0))
    self.spawn_point = np.random.choice(
        self.world.get_map().get_spawn_points())
    # self.spawn_point = self.world.get_map().get_spawn_points()[7]
    self.vehicle = self.world.spawn_actor(
        self.vehicle_bp, self.spawn_point)
    self.actor_list.append(self.vehicle)
    print("Car spawned")
    # rgb camera
    camera_bp = self.bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_location = carla.Transform(carla.Location(x=1.5, z=2.4))
    self.camera = self.world.spawn_actor(
        camera_bp, camera_location, attach_to=self.vehicle)
    self.camera.listen(lambda image: process_image(image))
    self.actor_list.append(self.camera)
    # collision sensor
    collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
    self.collision_sensor = self.world.spawn_actor(
        collision_bp, carla.Transform(), attach_to=self.vehicle)
    self.collision_sensor.listen(lambda event: on_collision(event, self))
    self.actor_list.append(self.collision_sensor)


def on_collision(event, self):
    self.client.apply_batch(
        [carla.command.DestroyActor(x) for x in self.actor_list])
    self.reset()
