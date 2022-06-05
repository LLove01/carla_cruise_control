import glob
import os
import sys
import numpy as np

from pid_controller import *
# from tf_detection import *
from carla_helpers import *
from lane_detection import *
from radar import _Radar_callback

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def process_image(image):
    # rgb camera returns 4 chanels
    image = np.array(image.raw_data)
    img = image.reshape((600, 800, 4))
    # cutting out alpha chanel
    img = img[:, :, :3]
    # tf object detection
    # objects_img = show_inference(detection_model, img)
    # lanes detection
    lanes_img = find_lanes(img)
    # cv2.imshow("rgb cam", lanes_img)
    cv2.waitKey(50)
    return img


class Cybertruck:
    def __init__(self):
        self.actor_list = []
        self.client = carla.Client('127.0.0.1', 3001)
        self.client.set_timeout(5.0)
        print("connected")
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        # creating and spawning a vehicle
        self.vehicle_bp = self.bp_lib.filter('cybertruck')[0]
        self.spawn_point = self.world.get_map().get_spawn_points()[3]
        # self.spawn_point = np.random.choice(
        # self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(
            self.vehicle_bp, self.spawn_point)
        self.actor_list.append(self.vehicle)
        print("Car spawned")
        self.vehicle_controller = PID_Controller(self.vehicle, args_acceleration={
            'K_P': 1, 'K_D': 0.0, 'K_I': 0.0}, args_direction={'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
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
        # radar
        radar_bp = self.bp_lib.filter('sensor.other.radar')[0]
        radar_bp.set_attribute('horizontal_fov', str(35))
        radar_bp.set_attribute('vertical_fov', str(20))
        radar_bp.set_attribute('points_per_second', str(1500))
        radar_bp.set_attribute('range', str(20))
        radar_location = carla.Transform(carla.Location(x=-0.5, z=1.8))
        self.radar = self.world.spawn_actor(
            radar_bp, radar_location, attach_to=self.vehicle)
        self.radar.listen(lambda radar_data: _Radar_callback(
            radar_data, self.vehicle, self.world))

    def run(self):
        try:
            while True:
                waypoints = self.world.get_map().get_waypoint(self.vehicle.get_location())
                waypoint = np.random.choice(waypoints.next(0.3))
                control_signal = self.vehicle_controller.run_step(50, waypoint)
                self.vehicle.apply_control(control_signal)
        finally:
            print('destroying actors')
            self.client.apply_batch(
                [carla.command.DestroyActor(x) for x in self.actor_list])
            print('done.')

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


if __name__ == '__main__':
    my_tesla = Cybertruck()
    my_tesla.run()
