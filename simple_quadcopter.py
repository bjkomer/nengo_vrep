# Example of an adaptive quadcopter controller.
# The quadcopter will follow the green sphere in the simulation
# Scene File: ss_adaptive_quadcopter.ttt
import nengo
import numpy as np
from robots import Quadcopter

k1 = 0.43352026190263104
k2 = 8.0
k3 = 0.5388202808181405
k4 = 6.6
k5 = 2.5995452450850185
k6 = 6.42298200082
k7 = 0.5990281657438163
k8 = 11.5589242985

ak1 = 0.026210965785217845
ak2 = 26.0
ak3 = 0.027614986033826894
ak4 = 21.45

gain_matrix = np.matrix([[  0,  0, k2,  0,  0,-k4,  0,  0,  0,  0,  0,  0],
                         [  0, k1,  0,  0,-k3,  0,-k5,  0,  0, k7,  0,  0],
                         [-k1,  0,  0, k3,  0,  0,  0,-k5,  0,  0, k7,  0],
                         [  0,  0,  0,  0,  0,  0,  0,  0,-k6,  0,  0, k8] 
                        ])

adaptive_filter = np.matrix([[  0,  0, ak2,  0,  0,-ak4,  0,  0,  0,  0,  0,  0],
                             [  0, ak1,  0,  0,-ak3,  0,  0,  0,  0,  0,  0,  0],
                             [-ak1,  0,  0, ak3,  0,  0,  0,  0,  0,  0,  0,  0],
                             [  0,   0,  0,  0,   0,  0,  0,  0,-k6,  0,  0, k8]
                            ])

# Defined to match the alignment of the propellors
rotor_transform = np.matrix([[ 1,-1, 1, 1],
                             [ 1,-1,-1,-1],
                             [ 1, 1,-1, 1],
                             [ 1, 1, 1,-1] ])

control_transform = rotor_transform * gain_matrix
adaptive_transform = -1 * rotor_transform * adaptive_filter

neuron_type = nengo.Direct()

model = nengo.Network(label='V-REP Quadcopter', seed=13)
with model:
    copter = nengo.Node(Quadcopter(sim_dt=0.01), size_in=4, size_out=12)

    # State Error Population
    state = nengo.Ensemble(n_neurons=4800, dimensions=12, 
                           neuron_type=neuron_type, radius=5)

    # Contains the rotor speeds
    motor = nengo.Ensemble(n_neurons=1600, dimensions=4, 
                           neuron_type=neuron_type, radius=7)

    adaptation = nengo.Ensemble(n_neurons=1000, dimensions=12)

    nengo.Connection(state, adaptation, synapse=None)


    adapt_conn = nengo.Connection(adaptation, motor, function=lambda x: [0,0,0,0],
                                  learning_rule_type=nengo.PES(learning_rate=1e-4))
    
    error_conn = nengo.Connection(state, adapt_conn.learning_rule,
                                  transform=adaptive_transform)

    nengo.Connection(state, motor, transform=control_transform)
    nengo.Connection(copter[:12], state, synapse=None)
    nengo.Connection(motor, copter, synapse=0.001)
