import numpy as np
import h5py
import matplotlib.pyplot as plt

# reading data
with h5py.File('capture_data/data.h5', 'r') as hdf:
    # ls = list(hdf.keys())
    # for dataset in ls:
    #     data = hdf.get(dataset)
    #     as_arr = np.array(data)
    #     print(f'{dataset} shape: {data.shape}')
    # forward
    fw_frames = hdf.get('fw_frames')
    fw_frames_arr = np.array(fw_frames)
    print(fw_frames_arr.shape)
    # right
    r_frames = hdf.get('r_frames')
    r_frames_arr = np.array(r_frames)
    print(r_frames_arr.shape)
    # left
    l_frames = hdf.get('l_frames')
    l_frames_arr = np.array(l_frames)
    print(l_frames_arr.shape)
    # steering
    steering = hdf.get('steering')
    steering_arr = np.array(steering)
    print(steering_arr.shape)
    # throttle
    throttle = hdf.get('throttle')
    throttle_arr = np.array(throttle)
    print(throttle_arr.shape)
    # brake
    brake = hdf.get('brake')
    brake_arr = np.array(brake)
    print(brake_arr.shape)
    # speed vector
    velocity = hdf.get('speed')
    velocity_arr = np.array(velocity)
    velocity_arr = velocity_arr.transpose()

    # displaying
    fig, ax = plt.subplots(3, 4)
    ax[0][0].plot(steering_arr)
    ax[0][0].set_title('Steering')
    ax[0][1].plot(throttle_arr)
    ax[0][1].set_title('Throttle')
    ax[0][2].plot(brake_arr)
    ax[0][2].set_title('Brake')
    ax[0][3].plot(velocity_arr[0])
    ax[0][3].set_title('Velocity x')
    ax[1][0].plot(velocity_arr[1])
    ax[1][0].set_title('Velocity y')
    ax[1][1].plot(velocity_arr[2])
    ax[1][1].set_title('Velocity z')
    ax[1][2].imshow(fw_frames_arr[0])
    ax[1][2].set_title('First frame FW')
    ax[1][3].imshow(fw_frames_arr[-1])
    ax[1][3].set_title('Last frame FW')
    ax[2][0].imshow(l_frames_arr[0])
    ax[2][0].set_title('First frame L')
    ax[2][1].imshow(l_frames_arr[-1])
    ax[2][1].set_title('Last frame L')
    ax[2][2].imshow(r_frames_arr[0])
    ax[2][2].set_title('First frame R')
    ax[2][3].imshow(r_frames_arr[-1])
    ax[2][3].set_title('Last frame R')
    plt.show()
