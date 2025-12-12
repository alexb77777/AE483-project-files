###################################
# IMPORTS

# Add parent directory to sys.path so that we can import from ae483clients
import sys, os
sys.path.append(os.path.abspath('..'))
from ae483clients import CrazyflieClient, QualisysClient

# Do all other imports
import time
import json


###################################
# PARAMETERS

# -- PROBABLY THE SAME FOR EVERY FLIGHT IN LABS 1-10 --

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/87/2M/E7E7E7E7E7'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_70'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [71, 72, 73, 74]

# -- MAY CHANGE FROM FLIGHT TO FLIGHT --

# Specify whether or not to use the motion capture system
use_mocap = False

# Specify whether or not to use a custom controller
use_controller = True

# Specify whether or not to use a custom observer
use_observer = True

# Specify the variables you want to log at 100 Hz from the drone
variables = [
    # State estimates (still from default observer)
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
]

###################################
# FLIGHT CODE

# Create and start the client that will connect to the drone
drone_client = CrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

# Wait until the client is fully connected to the drone
while not drone_client.is_fully_connected:
    time.sleep(0.1)

# Create and start the client that will connect to the motion capture system
if use_mocap:
    mocap_client = QualisysClient([{'name': marker_deck_name, 'callback': None}])

# Pause before takeoff
drone_client.stop(1.0)

#
#
# --- Graceful takeoff ---
drone_client.move(0.0, 0.0, 0.2, 0.0, 1.0)
drone_client.move_smooth([0., 0., 0.2], [0., 0., 0.5], 0.0, 0.20)
drone_client.move(0.0, 0.0, 0.5, 0.0, 1.0)

# Graceful takeoff
drone_client.move_smooth([0.0, 0.0, 0.5], [1.0, 0.0, 0.5], 0.0, 0.20)

drone_client.move(1.0, 0.1, 0.5, 0.0, 1.0)

drone_client.move_smooth([1.0, 0.1, 0.5], [0.0, 0.1, 0.5], 0.0, 0.20)

# Graceful landing
drone_client.move_smooth([0.0, 0.0, 0.5], [0.0, 0.0, 0.20], 0.0, 0.20)
drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)

# Pause after landing
drone_client.stop(1.0)

# Disconnect from the drone
drone_client.disconnect()

# Disconnect from the motion capture system
if use_mocap:
    mocap_client.close()

# Assemble flight data from both clients
data = {}
data['drone'] = drone_client.data
data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if use_mocap else {}
data['bodies'] = mocap_client.data if use_mocap else {}

# Write flight data to a file
with open('hardware_data.json', 'w') as outfile:
    json.dump(data, outfile, sort_keys=False)