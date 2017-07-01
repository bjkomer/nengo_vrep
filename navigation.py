# An example of a robot navigating to specific places in an environment
# Uses an associative memory that links places to coordinates
# Scene File: ss_navigation.ttt
import nengo
from nengo import spa
import numpy as np
from robots import WaypointQuadcopter

# Dimensions of the SPA vocabulary
DIM = 16 #32

# Define the vocabulary of semantic pointers that will be used
location_vocab = spa.Vocabulary(dimensions=DIM)
coordinate_vocab = spa.Vocabulary(dimensions=3)

# Locations that the quadcopter knows about
locations = ['HOME', 'FOREST', 'ROBOT', 'TABLE', 'SAND']
coordinates = [(0,0,.5),(-0.55,2.4,1),(3,-.1,0.75),(2.7,2.5,1.5),(1.65,-1.975,1)]
coordinate_names = ['C'+l for l in locations]

# Generate a semantic pointer for each of these locations in the vocabulary
for location in locations:
    location_vocab.parse(location)

# Generate a vector for each coordinate
for i, coordinate in enumerate(coordinates):
    coordinate_vocab.add(coordinate_names[i], coordinate)

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

model = spa.SPA(label='Navigation Quadcopter', seed=13)
with model:

    model.assoc_mem = spa.AssociativeMemory(input_vocab=location_vocab, 
                                            output_vocab=coordinate_vocab,
                                            input_keys=locations,
                                            output_keys=coordinate_names,
                                            wta_output=True,
                                            threshold_output=True)

    # Destination coordinates: x, y, z
    destination = nengo.Ensemble(n_neurons=300, dimensions=3, radius=5)

    # Input for the location name
    model.location_state = spa.State(DIM, vocab=location_vocab)

    # Output of the associative memory is the coordinates of the destination
    nengo.Connection(model.assoc_mem.output, destination)

    # Send the user-specified input to the associative memory
    nengo.Connection(model.location_state.output, model.assoc_mem.input)

    # Quadcopter node that connects to V-REP
    #  Input: 0-3: rotor velocity, 4-6: target position, 7-9: target orientation
    #  Output: 0-2: position, 3-5: velocity, 6-8: angle, 9-11: angular velocity
    copter = nengo.Node(WaypointQuadcopter(sim_dt=0.01), size_in=10, size_out=12)

    # State Error Population
    state = nengo.Ensemble(n_neurons=4800, dimensions=12, 
                           neuron_type=nengo.Direct(), radius=5)

    # Contains the rotor speeds
    motor = nengo.Ensemble(n_neurons=1600, dimensions=4, 
                           neuron_type=nengo.Direct(), radius=7)

    adaptation = nengo.Ensemble(n_neurons=1000, dimensions=12)

    nengo.Connection(state, adaptation, synapse=None)


    adapt_conn = nengo.Connection(adaptation, motor, function=lambda x: [0,0,0,0],
                                  learning_rule_type=nengo.PES(learning_rate=1e-4))
    
    error_conn = nengo.Connection(state, adapt_conn.learning_rule,
                                  transform=adaptive_transform)

    nengo.Connection(state, motor, transform=control_transform)
    nengo.Connection(copter[:12], state, synapse=None)
    nengo.Connection(motor, copter[0:4], synapse=0.001)

    # Send the desired location to the quadcopter
    nengo.Connection(destination, copter[[4,5,6]])
