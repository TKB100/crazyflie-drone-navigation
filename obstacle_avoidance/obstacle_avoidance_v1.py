import signal
import logging
import sys
import time
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger 

# Define the default URI for communication with the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

# Flight parameters
DEFAULT_HEIGHT = 1
MIN_DISTANCE = 0.2  
BOX_LIMIT = .5
GOAL_X = .4
BUFFER = 0.05  
VELOCITY = 0.15  
MAX_RUN_TIME = 180

# Position Tracking
position_estimate = [0, 0]  
position_x, position_y = [], []  

# Global flag to stop flight safely
keep_flying = True  


"---------------------------------------------------------------------------"
"""Handles Ctrl+C (SIGINT) to land the drone safely."""
def signal_handler(sig, frame):
    global keep_flying
    print("\nCtrl+C detected! Landing the drone safely...")
    keep_flying = False  # Stop the main loop
"---------------------------------------------------------------------------"


# Register the Ctrl+C signal handler
signal.signal(signal.SIGINT, signal_handler)


"---------------------------------------------------------------------------"
"""Check if an object is too close to the drone."""
def is_close(range):
    return range is not None and range < MIN_DISTANCE
"---------------------------------------------------------------------------"


"---------------------------------------------------------------------------"
""" Updates position estimate and stores data for visualization """
def log_pos_callback(timestamp, data, logconf):
    global position_estimate, position_x, position_y
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_x.append(position_estimate[0])
    position_y.append(position_estimate[1])
"---------------------------------------------------------------------------"


"---------------------------------------------------------------------------"
"""Plots the drone's movement in 2D space."""
def plot_position_data():
    plt.figure(figsize=(8, 6))
    plt.plot(position_x, position_y, marker='o', linestyle='-', markersize=3, label="Drone Path")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Drone Movement Path")
    plt.legend()
    plt.grid(True)
    plt.show()
"---------------------------------------------------------------------------"


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Enable position logging
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
            with Multiranger(scf) as multiranger:
                right = 1
                start_time = time.time()
                try:
                    while keep_flying:
                        elapsed_time = time.time() - start_time  # Calculate elapsed time

                        # **Check if time limit is reached**
                        if elapsed_time >= MAX_RUN_TIME:
                            print("Time limit reached! Landing the drone safely.")
                            motion_commander.land()
                            break

                        velocity_x, velocity_y = VELOCITY, 0.0

                        # **Check if drone has reached the goal**
                        if position_estimate[0] >= GOAL_X:
                            print("Goal reached! Landing.")
                            motion_commander.land()
                            break

                        # **Obstacle Avoidance**
                        if is_close(multiranger.front):
                            print("Object detected in front! Stopping.")
                            motion_commander.stop()
                            time.sleep(0.5)

                            print("Moving backward.")
                            motion_commander.start_linear_motion(-VELOCITY, 0, 0)
                            time.sleep(1)
                            motion_commander.stop()

                            if right == 1:
                                print("Moving right.")
                                motion_commander.start_linear_motion(0, -VELOCITY, 0)
                            else:
                                print("Moving left.")       
                                motion_commander.start_linear_motion(0, VELOCITY, 0)
                            
                            time.sleep(1)
                            motion_commander.stop()

                        # **Boundary Handling**
                        if position_estimate[0] > BOX_LIMIT:
                            print("Hit right boundary, stopping.")
                            motion_commander.stop()
                            time.sleep(0.5)

                            print("Moving left.")
                            motion_commander.start_linear_motion(0, VELOCITY, 0)  # move left
                            time.sleep(1)
                            motion_commander.stop()
                            right = 0 

                            print("Continuing forward.")

                        elif position_estimate[0] < -BOX_LIMIT:
                            print("Hit left boundary, stopping.")
                            motion_commander.stop()
                            time.sleep(0.5)

                            print("Moving right.")
                            motion_commander.start_linear_motion(0, -VELOCITY, 0)  # move right
                            time.sleep(1)
                            motion_commander.stop()
                            right = 1
                            print("Continuing forward.")

                        elif position_estimate[1] > BOX_LIMIT:
                            print("Hit upper boundary, stopping")
                            motion_commander.stop()
                            time.sleep(.5)

                            print("Moving Down")
                            motion_commander.start_linear_motion(VELOCITY , 0, 0)  # move down
                            time.sleep(1)
                            motion_commander.stop()

                        if is_close(multiranger.up):  # If ceiling is detected
                            print("Ceiling detected! Stopping.")
                            keep_flying = False

                        # **Move if no obstacles detected**
                        if not is_close(multiranger.front):
                            motion_commander.start_linear_motion(velocity_x, velocity_y, 0)

                        time.sleep(0.1)

                except KeyboardInterrupt:
                    print("\nEmergency stop triggered (Ctrl+C). Landing now...")
                    motion_commander.land()
                    time.sleep(1)

        logconf.stop()

    # Plot recorded position data after flight
    plot_position_data()
