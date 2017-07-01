# A two-wheeled robot that tries to avoid walls using sensors
# Scene File: ss_pioneer.ttt
import nengo
import sensors
import actuators
from robots import CustomRobot
from functools import partial

model = nengo.Network(label="Pioneer p3dx", seed=13)

# Create a robot object that can have sensors and actuators added to it
# The 'sim_dt' parameter is the dt it is expecting V-REP to be run with
# this can be different than Nengo's dt and the difference is accounted
# for when the two simulators are set to be synchronized
pioneer = CustomRobot(sim_dt=0.05, nengo_dt=0.001, sync=True)

braitenberg = [[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6,],
               [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2,],
              ]

# When adding sensors and actuators, the string names given must match
# the names of the specific sensors and actuators in V-REP
# These names can be found in the Scene Hierarchy pane

# Left and right wheels
pioneer.add_actuator("Pioneer_p3dx_leftMotor", actuators.joint_velocity)
pioneer.add_actuator("Pioneer_p3dx_rightMotor", actuators.joint_velocity)

# Sensor array
for i in range(1,9):
  pioneer.add_sensor("Pioneer_p3dx_ultrasonicSensor" + str(i), 
                     partial(sensors.prox_inv_dist, max_val=1))


model.config[nengo.Ensemble].neuron_type=nengo.LIF()
with model:
  # Create a Node that interfaces Nengo with V-REP
  robot = pioneer.build_node()
  # the above function is just a shortcut for the following line
  #robot = nengo.Node(pioneer, size_in=2, size_out=8)
  
  motor = nengo.Ensemble(n_neurons=100, dimensions=2, radius=3)
  sensors = nengo.Ensemble(n_neurons=400, dimensions=8)
  
  # Create a single slider to control the speed
  speed = nengo.Node([0])

  nengo.Connection(sensors, motor, transform=braitenberg)
  nengo.Connection(speed, motor, transform = [[1],[1]])

  nengo.Connection(motor, robot)
  nengo.Connection(robot, sensors)
