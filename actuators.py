import vrep
import ctypes

vrep_mode = vrep.simx_opmode_oneshot

# dim = 3
def position(cid, handle, val):
    err = vrep.simxSetObjectPosition(cid, handle, -1, val, vrep_mode)

# dim = 3
def orientation(cid, handle, val):
    err = vrep.simxSetObjectOrientation(cid, handle, -1, val, vrep_mode)

# dim = 1
def joint_torque(cid, handle, val):
    vrep.simxSetJointTargetVelocity(cid, handle, val*100, vrep_mode)
    vrep.simxSetJointForce(cid, handle, abs(val), vrep_mode)

# dim = 1
def joint_velocity(cid, handle, val):
    vrep.simxSetJointTargetVelocity(cid, handle, val, vrep_mode)

# dim = 1
def float_signal(cid, handle, val, signal_name):
    err = vrep.simxSetFloatSignal(cid, signal_name, val, vrep_mode)

# dim = 1
def integer_signal(cid, handle, val, signal_name):
    err = vrep.simxSetIntegerSignal(cid, signal_name, int(val), vrep_mode)

# dim = 4
def quadcopter_rotors(cid, handle, val):
    # This function does not use handle, it just exists in the signature
    # to make it consistent
    motor_values = np.zeros(4)
    for i in range(4):
      motor_values[i] = val[i]
    packedData=vrep.simxPackFloats(motor_values.flatten())
    raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
    err = vrep.simxSetStringSignal(cid, "rotorTargetVelocities",
                                   raw_bytes,
                                   vrep_mode)

