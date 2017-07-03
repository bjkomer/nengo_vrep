import vrep
import ctypes
import numpy as np

vrep_mode = vrep.simx_opmode_oneshot

# dim = 3
def position(cid, handle):
    err, ret = vrep.simxGetObjectPosition(cid, handle, -1, vrep_mode)
    return ret

# dim = 3
def orientation(cid, handle):
    err, ret = vrep.simxGetObjectOrientation(cid, handle, -1, vrep_mode)
    return ret

# dim = 6
def velocity(cid, handle):
    err, lin, ang = vrep.simxGetObjectVelocity(cid, handle, vrep_mode)
    return [lin[0], lin[1], lin[2], ang[0], ang[1], ang[2]]

# dim = 3
def linear_velocity(cid, handle):
    err, lin, ang = vrep.simxGetObjectVelocity(cid, handle, vrep_mode)
    return lin

# dim = 3
def angular_velocity(cid, handle):
    err, lin, ang = vrep.simxGetObjectVelocity(cid, handle, vrep_mode)
    return ang

# dim = 1
def joint_angle(cid, handle):
    err, ret = vrep.simxGetJointPosition(cid, handle, vrep_mode)
    return [ret]

# dim = 1
def joint_velocity(cid, handle):
    err, ret = vrep.simxGetObjectFloatParameter(cid, handle, 2012, vrep_mode)
    return [ret]

# dim = 1
def prox_dist(cid, handle, default=0):
    err, state, point, handle, normal = vrep.simxReadProximitySensor(cid, handle, vrep_mode)
    if state:
        return [point[2]]
    else:
        return [default]

# dim = 1
def prox_inv_dist(cid, handle, max_val=1):
    # Returns a smaller value the further the detected object is away
    err, state, point, handle, normal = vrep.simxReadProximitySensor(cid, handle, vrep_mode)
    if state:
        return [max_val - point[2]]
    else:
        return [0]

# dim = 1
def prox_bool(cid, handle):
    err, state, point, handle, normal = vrep.simxReadProximitySensor(cid, handle, vrep_mode)
    return [1] if state else [0]

# dim = 1
def float_signal(cid, handle, signal_name):
    err, val = vrep.simxGetFloatSignal(cid, signal_name, vrep_mode)
    return [val]

# dim = 3
def accelerometer(cid, handle):
    err, accel_x = vrep.simxGetFloatSignal(cid, 'accelerometerX', vrep_mode)
    err, accel_y = vrep.simxGetFloatSignal(cid, 'accelerometerY', vrep_mode)
    err, accel_z = vrep.simxGetFloatSignal(cid, 'accelerometerZ', vrep_mode)

    return [accel_x, accel_y, accel_z]

# dim = 3
def gyro(cid, handle):
    err, accel_x = vrep.simxGetFloatSignal(cid, 'gyroX', vrep_mode)
    err, accel_y = vrep.simxGetFloatSignal(cid, 'gyroY', vrep_mode)
    err, accel_z = vrep.simxGetFloatSignal(cid, 'gyroZ', vrep_mode)

    return [accel_x, accel_y, accel_z]

# dim = 1
def integer_signal(cid, handle, signal_name):
    err, val = vrep.simxGetIntegerSignal(cid, signal_name, vrep_mode)
    return [val]

def dvs_vision(cid, handle, height=32, width=32):
    err, res, image = vrep.simxGetVisionSensorImage(cid, handle, 1, vrep_mode)
    # Return zeroes if nothing is in the input buffer
    if err == 1:
        return np.zeros(height * width)
    #return np.array(image).ravel()==0
    return (np.array(image).ravel()==0) + (np.array(image).ravel()==-1)
    #return (np.array(image).ravel()==0) - (np.array(image).ravel()==-1)

# dim = dim_x * dim_y
def dvs(cid, handle=None, signal_name="dataFromThisTimestep", 
        dim_x=32, dim_y=32, offset=0, magnitude=1):
    empty_str=""
    raw_bytes = (ctypes.c_ubyte * len(empty_str)).from_buffer_copy(empty_str) 
    err, data = vrep.simxGetStringSignal(cid, "dataFromThisTimeStep",
                                         vrep.simx_opmode_oneshot)
    err = vrep.simxSetStringSignal(cid, "dataFromThisTimeStep", raw_bytes,
                                   vrep.simx_opmode_oneshot_wait)
    image = np.zeros( (dim_x, dim_y), dtype='uint8' ) + offset
    l = len(data)
    for i in range( int(l/4) ):
      b = list(bytearray(data[i*4:i*4+4]))
      x_coord = b[0]
      y_coord = b[1]
      if x_coord >= 128:
        x_coord -= 128
        polarity = 1
      else:
        #polarity = 0
        polarity = -1
      timestamp = (b[3] * 256) + b[2]
      # Distinguish between different polarities
      #image[127-y_coord][127-x_coord] = polarity * magnitude
      image[dim_y-1-y_coord][dim_x-1-x_coord] = polarity * magnitude
      
      # No Distinction between different polarities
      #image[127-y_coord][127-x_coord] = magnitude

    return image.ravel()
