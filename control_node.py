import glob
import os
import sys
import numpy as np
import time

from pid_controller import *
# from tf_detection import *
from carla_helpers import *
from lane_detection import *

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

lines = []


def process_fw(image):
    global lines
    # rgb camera returns 4 chanels
    image = np.array(image.raw_data)
    img = image.reshape((600, 800, 4))
    # cutting out alpha chanel
    img = img[:, :, :3]
    # tf object detection
    # objects_img = show_inference(detection_model, img)
    # lanes detection
    found_lines, lanes_img = get_lanes(img, 'FW')
    # # refreshing global list
    # lines = found_lines
    # display feed
    cv2.imshow("rgb cam", lanes_img)
    cv2.waitKey(500)
    return img


def process_r(image):
    global lines
    # rgb camera returns 4 chanels
    image = np.array(image.raw_data)
    img = image.reshape((600, 800, 4))
    # cutting out alpha chanel
    img = img[:, :, :3]
    # tf object detection
    # objects_img = show_inference(detection_model, img)
    # lanes detection
    found_lines, lanes_img = get_lanes(img, 'R')
    # refreshing global list
    lines = found_lines
    # display feed
    # cv2.imshow("rgb cam", lanes_img)
    # cv2.waitKey(500)
    return img


class Cybertruck:
    def __init__(self):
        self.actor_list = []
        self.client = carla.Client('127.0.0.1', 3000)
        self.client.set_timeout(5.0)
        print("connected")
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        # creating and spawning a vehicle
        self.vehicle_bp = self.bp_lib.filter('cybertruck')[0]
        self.spawn_point = self.world.get_map().get_spawn_points()[4]
        self.vehicle = self.world.spawn_actor(
            self.vehicle_bp, self.spawn_point)
        self.actor_list.append(self.vehicle)
        print("Car spawned")
        self.vehicle_controller = PID_Controller(self.vehicle, args_acceleration={
            'K_P': 1, 'K_D': 0.0, 'K_I': 0.0}, args_direction={'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
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
        # self.fw_cam.listen(lambda data: process_fw(data))
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
        # collision sensor
        collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(
            collision_bp, carla.Transform(), attach_to=self.vehicle)
        self.collision_sensor.listen(lambda event: on_collision(event, self))
        self.actor_list.append(self.collision_sensor)

    def run(self):
        try:
            while True:
                # waypoints = self.world.get_map().get_waypoint(self.vehicle.get_location())
                # waypoint = np.random.choice(waypoints.next(0.3))
                control_signal = self.vehicle_controller.run_step(
                    30, lines)
                self.vehicle.apply_control(control_signal)
        finally:
            print('destroying actors')
            self.client.apply_batch(
                [carla.command.DestroyActor(x) for x in self.actor_list])
            print('done.')


if __name__ == '__main__':
    my_tesla = Cybertruck()
    my_tesla.run()
