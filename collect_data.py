import glob
import os
import sys


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
import cv2
import h5py

IM_WIDTH = 800
IM_HEIGHT = 600

# collecting data
FW_IMG_LIST = []
L_IMG_LIST = []
R_IMG_LIST = []
SPEED_LIST = []
STEERING_LIST = []
THROTTLE_LIST = []
BRAKE_LIST = []


def process_frame(image, vehicle, cam_label):
    global FW_IMG_LIST, L_IMG_LIST, R_IMG_LIST, STEERING_LIST, THROTTLE_LIST, BRAKE_LIST, SPEED_LIST
    # raw data is a flattened array, cannot be displayed
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    # zanima nas rgb, alfa vrednosti zavrzemo
    i3 = i2[:, :, :3]
    # cv2.imshow("", i3)
    cv2.waitKey(1)
    # normalizacija
    i4 = i3/255.0
    if cam_label == 'FW':
        # forward camera
        FW_IMG_LIST.append(i4)
        # saving controls at current frame
        STEERING_LIST.append(vehicle.get_control().steer)
        THROTTLE_LIST.append(vehicle.get_control().throttle)
        BRAKE_LIST.append(vehicle.get_control().brake)
        # velocity - 3d array (x, y, z)
        velocity = vehicle.get_velocity()
        velocity_arr = np.array([velocity.x, velocity.y, velocity.z])
        SPEED_LIST.append(velocity_arr)
    elif cam_label == 'R':
        # right side camera
        R_IMG_LIST.append(i4)
    else:
        # left side camera
        L_IMG_LIST.append(i4)
    return i4


actor_list = []
try:
    client = carla.Client('127.0.0.1', 3000)
    client.set_timeout(2.0)
    print("connected")
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter('cybertruck')[0]
    print(bp)
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, spawn_point)
    # vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.
    actor_list.append(vehicle)

    # FORWARD CAMERA
    fw_cam_bp = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    fw_cam_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
    fw_cam_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
    fw_cam_bp.set_attribute('fov', '90')
    # Adjust sensor relative to vehicle
    fw_cam_transf = carla.Transform(carla.Location(x=1.5, z=2.4))
    # spawn the sensor and attach to vehicle.
    fw_cam = world.spawn_actor(fw_cam_bp, fw_cam_transf, attach_to=vehicle)
    # add sensor to list of actors
    actor_list.append(fw_cam)
    # do something with this sensor
    fw_cam.listen(lambda data: process_frame(data, vehicle, 'FW'))

    # RIGHT SIDE CAMERA
    r_cam_bp = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    r_cam_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
    r_cam_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
    r_cam_bp.set_attribute('fov', '120')
    # Adjust sensor relative to vehicle
    r_cam_loc = carla.Location(0, 1.48, 2.4)
    r_cam_rot = carla.Rotation(-45, 90, 0)
    r_cam_transf = carla.Transform(r_cam_loc, r_cam_rot)
    # spawn the sensor and attach to vehicle.
    r_cam = world.spawn_actor(r_cam_bp, r_cam_transf, attach_to=vehicle)
    # add sensor to list of actors
    actor_list.append(r_cam)
    # do something with this sensor
    r_cam.listen(lambda data: process_frame(data, vehicle, 'R'))

    # LEFT SIDE CAMERA
    l_cam_bp = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    l_cam_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
    l_cam_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
    l_cam_bp.set_attribute('fov', '120')
    # Adjust sensor relative to vehicle
    l_cam_loc = carla.Location(0, -1.48, 2.4)
    l_cam_rot = carla.Rotation(-45, -90, 0)
    l_cam_transf = carla.Transform(l_cam_loc, l_cam_rot)
    # spawn the sensor and attach to vehicle.
    l_cam = world.spawn_actor(l_cam_bp, l_cam_transf, attach_to=vehicle)
    # add sensor to list of actors
    actor_list.append(l_cam)
    # do something with this sensor
    l_cam.listen(lambda data: process_frame(data, vehicle, 'L'))

    time.sleep(5)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
    # saving data
    with h5py.File('capture_data/data.h5', 'w') as hdf:
        hdf.create_dataset('fw_frames', data=np.array(FW_IMG_LIST))
        hdf.create_dataset('r_frames', data=np.array(R_IMG_LIST))
        hdf.create_dataset('l_frames', data=np.array(L_IMG_LIST))
        hdf.create_dataset('steering', data=np.array(STEERING_LIST))
        hdf.create_dataset('throttle', data=np.array(THROTTLE_LIST))
        hdf.create_dataset('brake', data=np.array(BRAKE_LIST))
        hdf.create_dataset('speed', data=np.array(SPEED_LIST))
