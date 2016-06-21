import numpy as np
import vrep
import ctypes
import math
import nengo

vrep_mode = vrep.simx_opmode_oneshot

class Robot(object):
    def __init__(self, sim_dt=0.05, nengo_dt=0.001, sync=True):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.cid = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
        self.sync = sync

        if self.cid != -1:
            print ('Connected to V-REP remote API server, client id: %s' % self.cid)
            vrep.simxStartSimulation( self.cid, vrep.simx_opmode_oneshot )
            if self.sync:
              vrep.simxSynchronous( self.cid, True )
        else:
            print ('Failed connecting to V-REP remote API server')
            exit(1)
        self.count = 0
        self.sim_dt = sim_dt
        self.nengo_dt = nengo_dt

    def handle_input(self, values):
        raise NotImplemented

    def handle_output(self):
        raise NotImplemented

    def __call__(self, t, values):
        self.count += 1
        if self.count == int(round(self.sim_dt/self.nengo_dt)):
            self.count = 0
            self.handle_input( values )
            if self.sync:
              vrep.simxSynchronousTrigger( self.cid )
        return self.handle_output()

class CustomRobot(Robot):
    """
    Sensors and actuators may be added to this component after it is created
    """

    def __init__(self, sim_dt=0.01, nengo_dt=0.001, sync=True):
        super(CustomRobot, self).__init__(sim_dt, nengo_dt, sync)
        self.sensors = []
        self.actuators = []
        self.size_in = 0
        self.size_out = 0
        
        # Store the output here so it doesn't need to be generated for each
        # Nengo timestep, only for V-REP timesteps
        self.output = []

    def handle_input(self, values):

        count = 0
        for handle, func, dim in self.actuators:
            func(self.cid, handle, values[count:count+dim])
            count += dim

        self.generate_output()

    def generate_output(self):

        ret = []
        for handle, func in self.sensors:
          tmp = func(self.cid, handle)
          for i in tmp:
            ret.append(i)

        self.output = ret
    
    def handle_output(self):
        return self.output

    def add_sensor(self, name, func, dim=1):
        if name is None:
            handle = None
        else:
            err, handle = vrep.simxGetObjectHandle(self.cid, name,
                                                   vrep.simx_opmode_oneshot_wait )
        self.sensors.append([handle, func])
        
        # This is needed so Nengo doesn't error on the first timestep
        self.output.extend([0]*dim)
        self.size_out += dim
    
    def add_actuator(self, name, func, dim=1):
        if name is None:
            handle = None
        else:
            err, handle = vrep.simxGetObjectHandle(self.cid, name,
                                                   vrep.simx_opmode_oneshot_wait )
        self.actuators.append([handle, func, dim])
        self.size_in += dim

    def build_node(self):
        return nengo.Node(self, size_in=self.size_in, size_out=self.size_out)

class Pendulum(Robot):
    def __init__(self):

        super(Pendulum, self).__init__()
        
        err, self.arm_joint = vrep.simxGetObjectHandle(self.cid, "arm_joint",
                                                vrep.simx_opmode_oneshot_wait )
    
    def handle_input(self, values):
        # Set the velocity to some large number with the correct sign,
        # because v-rep is weird like that
        vrep.simxSetJointTargetVelocity(self.cid, self.arm_joint, values[0]*100,
                                        vrep.simx_opmode_oneshot)

        # Apply the desired torques to the joints
        # V-REP is looking for just the absolute value here
        vrep.simxSetJointForce(self.cid, self.arm_joint, abs(values[0]),
                                        vrep.simx_opmode_oneshot)

    def handle_output(self):
        err, arm_ori = vrep.simxGetJointPosition(self.cid, self.arm_joint,
                                                   vrep.simx_opmode_oneshot)
        #2012 is the code for joint velocity
        err, arm_vel = vrep.simxGetObjectFloatParameter(self.cid,
                                                        self.arm_joint, 2012,
                                                        vrep.simx_opmode_oneshot)

        return [arm_ori, arm_vel]


class Arm(Robot):
    def __init__(self, sim_dt=0.05):

        super(Arm, self).__init__(sim_dt)
        err, self.hand = vrep.simxGetObjectHandle(self.cid, "hand_end",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.target = vrep.simxGetObjectHandle(self.cid, "target",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.hand_joint = vrep.simxGetObjectHandle(self.cid, "hand_joint",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.arm_joint = vrep.simxGetObjectHandle(self.cid, "arm_joint",
                                                vrep.simx_opmode_oneshot_wait )
        
    def handle_input(self, values):
        # Set the velocity to some large number with the correct sign,
        # because v-rep is weird like that
        vrep.simxSetJointTargetVelocity(self.cid, self.arm_joint, values[0]*100,
                                        vrep.simx_opmode_oneshot)

        vrep.simxSetJointTargetVelocity(self.cid, self.hand_joint, values[1]*100,
                                        vrep.simx_opmode_oneshot)

        # Apply the desired torques to the joints
        # V-REP is looking for just the absolute value here
        vrep.simxSetJointForce(self.cid, self.arm_joint, abs(values[0]),
                                        vrep.simx_opmode_oneshot)

        vrep.simxSetJointForce(self.cid, self.hand_joint, abs(values[1]),
                                        vrep.simx_opmode_oneshot)

    def handle_output(self):
        # return whatever state information you need to get the error you want
        # this will get you the information for the center of the object. If you
        # want something like the position of the end of an arm, you will need
        # to do some calculations, or just make a dummy object, attach it to
        # the point you want, and get the position of the dummy object
        
        #err, hand_pos = vrep.simxGetObjectPosition(self.cid, self.hand, -1,
        #                                           vrep.simx_opmode_oneshot)
        #err, hand_ori = vrep.simxGetObjectOrientation(self.cid, self.hand, -1,
        #                                           vrep.simx_opmode_oneshot)
        err, hand_ori = vrep.simxGetJointPosition(self.cid, self.hand_joint,
                                                   vrep.simx_opmode_oneshot)
        err, arm_ori = vrep.simxGetJointPosition(self.cid, self.arm_joint,
                                                   vrep.simx_opmode_oneshot)
        err, hand_vel = vrep.simxGetObjectFloatParameter(self.cid, self.hand_joint,
                                                         2012, vrep.simx_opmode_oneshot)
        err, arm_vel = vrep.simxGetObjectFloatParameter(self.cid, self.arm_joint,
                                                        2012, vrep.simx_opmode_oneshot)
        err, hand_pos = vrep.simxGetObjectPosition(self.cid, self.hand, -1,
                                                   vrep.simx_opmode_oneshot)
        err, target_pos = vrep.simxGetObjectPosition(self.cid, self.target, -1,
                                                   vrep.simx_opmode_oneshot)

        return [arm_ori, hand_ori, arm_vel, hand_vel, hand_pos[0], hand_pos[2],
                target_pos[0], target_pos[1]]

def b( num ):
  """ forces magnitude to be 1 or less """
  if abs( num ) > 1.0:
    return math.copysign( 1.0, num )
  else:
    return num

def convert_angles( ang ):
  """ Converts Euler angles from x-y-z to z-x-y convention """
  s1 = math.sin(ang[0])
  s2 = math.sin(ang[1])
  s3 = math.sin(ang[2])
  c1 = math.cos(ang[0])
  c2 = math.cos(ang[1])
  c3 = math.cos(ang[2])
  
  pitch = math.asin( b(c1*c3*s2-s1*s3) )
  cp = math.cos(pitch)
  # just in case
  if cp == 0:
    cp = 0.000001

  yaw = math.asin( b((c1*s3+c3*s1*s2)/cp) ) #flipped
  # Fix for getting the quadrants right
  if c3 < 0 and yaw > 0:
    yaw = math.pi - yaw
  elif c3 < 0 and yaw < 0:
    yaw = -math.pi - yaw
  
  roll = math.asin( b((c3*s1+c1*s2*s3)/cp) ) #flipped
  return [roll, pitch, yaw]

class Quadcopter( Robot ):
    """
    This callable class will return the state of the quadcopter relative to its
    target whenever it is called. It will also accept motor commands which will be
    sent to the quadcopter in V-REP.
    """
    def __init__( self, sim_dt=0.01, max_target_distance=3, noise=False,
                  noise_std=[0,0,0,0,0,0],
                  target_func=None,
                ):

        super(Quadcopter, self).__init__(sim_dt)

        err, self.copter = vrep.simxGetObjectHandle(self.cid, "Quadricopter_base",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.target = vrep.simxGetObjectHandle(self.cid, "Quadricopter_target",
                                                vrep.simx_opmode_oneshot_wait )

        # Reset the motor commands to zero
        packedData=vrep.simxPackFloats([0,0,0,0])
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 

        err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                        raw_bytes,
                                        vrep_mode)

        self.pos = [0,0,0]
        self.pos_err = [0,0,0]
        self.t_pos = [0,0,0]
        self.lin = [0,0,0]
        self.ori = [0,0,0]
        self.ori_err = [0,0,0]
        self.t_ori = [0,0,0]
        self.ang = [0,0,0]

        self.vert_prox_dist = 0
        self.left_prox_dist = 0
        self.right_prox_dist = 0
        
        # Distance reading recorded when nothing is in range
        self.max_vert_dist = 1.5
        self.max_left_dist = 1.0
        self.max_right_dist = 1.0
        
        # Maximum target distance error that can be returned
        self.max_target_distance = max_target_distance
 
        # If noise is being modelled
        self.noise = noise

        # Standard Deviation of the noise for the 4 state variables
        self.noise_std = noise_std
        
        # Overwrite the get_target method if the target is to be controlled by a
        # function instead of by V-REP
        if target_func is not None:
          
          self.step = 0
          self.target_func = target_func

          def get_target():
            self.t_pos, self.t_ori = self.target_func( self.step )
            self.step += 1

          self.get_target = get_target

    def reset( self ):
        err = vrep.simxStopSimulation(self.cid, vrep.simx_opmode_oneshot_wait)
        time.sleep(1)
        self.pos_err = [0,0,0]
        self.ori_err = [0,0,0]
        self.lin = [0,0,0]
        self.ang = [0,0,0]
        self.vert_prox = 0
        self.left_prox = 0
        self.right_prox = 0
        err = vrep.simxStartSimulation(self.cid, vrep.simx_opmode_oneshot_wait)
        if self.sync:
          vrep.simxSynchronous( self.cid, True )
    
    def exit( self ):
        exit(1)

    def get_target( self ):
        err, self.t_ori = vrep.simxGetObjectOrientation(self.cid, self.target, -1,
                                                    vrep_mode )
        err, self.t_pos = vrep.simxGetObjectPosition(self.cid, self.target, -1,
                                                vrep_mode )
        
        # Convert orientations to z-y-x convention
        self.t_ori = convert_angles(self.t_ori)

    def calculate_error( self ):
        # Return the state variables
        err, self.ori = vrep.simxGetObjectOrientation(self.cid, self.copter, -1,
                                                vrep_mode )
        err, self.pos = vrep.simxGetObjectPosition(self.cid, self.copter, -1,
                                            vrep_mode )
        err, self.lin, self.ang = vrep.simxGetObjectVelocity(self.cid, self.copter,
                                                            vrep_mode )
        
        self.ori = convert_angles(self.ori)
        
        # Apply noise to each measurement if required
        if self.noise:
          self.pos += np.random.normal(0,self.noise_std[0],3)
          self.lin += np.random.normal(0,self.noise_std[1],3)
          self.ori += np.random.normal(0,self.noise_std[2],3)
          self.ang += np.random.normal(0,self.noise_std[3],3)
          #TODO: might have to wrap angles here
        
        # Find the error
        self.ori_err = [self.t_ori[0] - self.ori[0], 
                        self.t_ori[1] - self.ori[1],
                        self.t_ori[2] - self.ori[2]]
        cz = math.cos(self.ori[2])
        sz = math.sin(self.ori[2])
        x_err = self.t_pos[0] - self.pos[0]
        y_err = self.t_pos[1] - self.pos[1]
        self.pos_err = [ x_err * cz + y_err * sz, 
                        -x_err * sz + y_err * cz, 
                         self.t_pos[2] - self.pos[2]]
        
        self.lin = [self.lin[0]*cz+self.lin[1]*sz, -self.lin[0]*sz+self.lin[1]*cz, self.lin[2]]
        self.ang = [self.ang[0]*cz+self.ang[1]*sz, -self.ang[0]*sz+self.ang[1]*cz, self.ang[2]]

        for i in range(3):
          if self.ori_err[i] > math.pi:
            self.ori_err[i] -= 2 * math.pi
          elif self.ori_err[i] < -math.pi:
            self.ori_err[i] += 2 * math.pi

    def send_motor_commands( self, values ):

        motor_values = np.zeros(4)
        for i in range(4):
          motor_values[i] = values[i]
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                        raw_bytes,
                                        vrep_mode)
    
    def handle_input( self, values ):
        
        # Send motor commands to V-REP
        self.send_motor_commands( values )

        # Retrieve target location
        self.get_target()

        # Calculate state error
        self.calculate_error()

    def bound( self, value ):
        if abs( value ) > self.max_target_distance:
          return math.copysign( self.max_target_distance, value )
        else:
          return value

    def handle_output( self ):
        l = math.sqrt(self.pos_err[0]**2 + self.pos_err[1]**2)
        bl = self.bound(l)
        r = (bl+.1)/(l+.1)

        return [r*self.pos_err[0], r*self.pos_err[1], self.bound(self.pos_err[2]), 
                self.lin[0], self.lin[1], self.lin[2], 
                self.ori_err[0], self.ori_err[1], self.ori_err[2], 
                self.ang[0], self.ang[1], self.ang[2],
               ]

class SensorQuadcopter( Quadcopter ):
    
    def __init__( self, *args, **kwargs ):

        super(SensorQuadcopter, self).__init__(*args, **kwargs)
        
        err, self.vert_prox = vrep.simxGetObjectHandle(self.cid, "vert_prox",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.left_prox = vrep.simxGetObjectHandle(self.cid, "left_prox",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.right_prox = vrep.simxGetObjectHandle(self.cid, "right_prox",
                                                vrep.simx_opmode_oneshot_wait )

    def read_proximity( self ):

        err, state, point, handle, normal = vrep.simxReadProximitySensor(self.cid, self.vert_prox, vrep_mode)
        if state:
          self.vert_prox_dist = point[2]
        else:
          self.vert_prox_dist = self.max_vert_dist

        err, state, point, handle, normal =\
            vrep.simxReadProximitySensor(self.cid, self.left_prox, vrep_mode)
        if state:
          self.left_prox_dist = point[2]
        else:
          self.left_prox_dist = self.max_left_dist
        
        err, state, point, handle, normal =\
            vrep.simxReadProximitySensor(self.cid, self.right_prox, vrep_mode)
        if state:
          self.right_prox_dist = point[2]
        else:
          self.right_prox_dist = self.max_right_dist
    
    def handle_input( self, values ):
        
        # Send motor commands to V-REP
        self.send_motor_commands( values )

        # Retrieve target location
        self.get_target()

        # Calculate state error
        self.calculate_error()

        # Get proximity sensor readings
        self.read_proximity()

    def handle_output( self ):
        l = math.sqrt(self.pos_err[0]**2 + self.pos_err[1]**2)
        bl = self.bound(l)
        r = (bl+.1)/(l+.1)

        return [r*self.pos_err[0], r*self.pos_err[1], self.bound(self.pos_err[2]), 
                self.lin[0], self.lin[1], self.lin[2], 
                self.ori_err[0], self.ori_err[1], self.ori_err[2], 
                self.ang[0], self.ang[1], self.ang[2],
                self.vert_prox_dist, self.left_prox_dist, self.right_prox_dist,
               ]

class TargetQuadcopter( Quadcopter ):
    
    """ Returns target position as well """

    def __init__( self, *args, **kwargs ):

        super(TargetQuadcopter, self).__init__(*args, **kwargs)
    
    def handle_output( self ):
        l = math.sqrt(self.pos_err[0]**2 + self.pos_err[1]**2)
        bl = self.bound(l)
        r = (bl+.1)/(l+.1)

        return [r*self.pos_err[0], r*self.pos_err[1], self.bound(self.pos_err[2]), 
                self.lin[0], self.lin[1], self.lin[2], 
                self.ori_err[0], self.ori_err[1], self.ori_err[2], 
                self.ang[0], self.ang[1], self.ang[2],
                self.t_pos[0], self.t_pos[1], self.t_pos[2],
                self.t_ori[0], self.t_ori[1], self.t_ori[2],
               ]

class WaypointQuadcopter( Quadcopter ):

    """ Takes the desired target as an input rather than moving to the green circle """

    def __init__( self, sim_dt=0.01, max_target_distance=3, noise=False,
                  noise_std=[0,0,0,0,0,0],
                ):
        
        # Call the superclass of Quadcopter, which is Robot
        super(Quadcopter, self).__init__(sim_dt)

        err, self.copter = vrep.simxGetObjectHandle(self.cid, "Quadricopter_base",
                                                vrep.simx_opmode_oneshot_wait )

        # Reset the motor commands to zero
        packedData=vrep.simxPackFloats([0,0,0,0])
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 

        err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                        raw_bytes,
                                        vrep_mode)

        self.pos = [0,0,0]
        self.pos_err = [0,0,0]
        self.t_pos = [0,0,0]
        self.lin = [0,0,0]
        self.ori = [0,0,0]
        self.ori_err = [0,0,0]
        self.t_ori = [0,0,0]
        self.ang = [0,0,0]
        
        # Maximum target distance error that can be returned
        self.max_target_distance = max_target_distance
 
        # If noise is being modelled
        self.noise = noise

        # Standard Deviation of the noise for the 4 state variables
        self.noise_std = noise_std

    def handle_input( self, values ):
        
        # Send motor commands to V-REP
        self.send_motor_commands( values[:4] )

        # Retrieve target location
        self.t_pos = values[[4,5,6]] 
        self.t_ori = values[[7,8,9]]

        # Calculate state error
        self.calculate_error()
