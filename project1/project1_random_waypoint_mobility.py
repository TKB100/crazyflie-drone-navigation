import logging
import sys
import time
from threading import Event
import matplotlib.pyplot as plt
import random

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

# Project parameters
DEFAULT_HEIGHT = 0.5 
BOX_LIMIT = 0.5     
MAX_VEL = 0.15
MOVE_DURATION = 2
MAX_RUN_TIME = 30

positions = []
waypoints = []

def param_deck_flow(name, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f'Deck {name} is attached!')
    else:
        print(f'Deck {name} is NOT attached!')

def generate_random_waypoint():
    """Generate a random waypoint within the box limits (2D only)"""
    x = random.uniform(-BOX_LIMIT, BOX_LIMIT)
    y = random.uniform(-BOX_LIMIT, BOX_LIMIT)
    return x, y

def log_position(x, y):
    """Log current position"""
    positions.append((x, y))
    print(f"Position: ({x:.3f}, {y:.3f})")

def simple_random_waypoint_mobility(scf):
    """Execute the Random Waypoint Mobility pattern for 30 seconds"""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Starting Random Waypoint Mobility pattern...")
       
        current_pos = (0.0, 0.0)
        log_position(current_pos[0], current_pos[1])
        

        time.sleep(2)
    
        flight_start_time = time.time()
        waypoint_count = 0
        
        while (time.time() - flight_start_time) < MAX_RUN_TIME:
            waypoint_count += 1
            print(f"\n--- Waypoint {waypoint_count} ---")

            waypoint = generate_random_waypoint()
            waypoints.append(waypoint)
            
            print(f"Flying to waypoint: ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
            
            dx = waypoint[0] - current_pos[0]
            dy = waypoint[1] - current_pos[1]
  
            mc.move_distance(dx, dy, 0)
            
            current_pos = waypoint
            log_position(current_pos[0], current_pos[1])
    
            elapsed_time = time.time() - flight_start_time
            remaining_time = MAX_RUN_TIME - elapsed_time
            
            if remaining_time <= 0:
                print("30 seconds completed during flight!")
                break
       
            pause_duration = min(MOVE_DURATION, remaining_time)
            print(f"Arrived at waypoint! Pausing for {pause_duration:.1f} seconds...")
            print(f"Time elapsed: {elapsed_time:.1f}s / {MAX_RUN_TIME}s")
            
            time.sleep(pause_duration)

            if (time.time() - flight_start_time) >= MAX_RUN_TIME:
                print("30 seconds completed during pause!")
                break
        
        total_time = time.time() - flight_start_time
        print(f"Mobility pattern completed! Total flight time: {total_time:.1f} seconds")
        print(f"Visited {waypoint_count} waypoints")
        time.sleep(1)

def plot_path():
    """Create x-y plot of the drone's path"""
    if not positions:
        print("No position data to plot!")
        return

    x_coords = [pos[0] for pos in positions]
    y_coords = [pos[1] for pos in positions]
    
    plt.figure(figsize=(10, 8))

    plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Drone Path')

    if waypoints:
        wp_x = [wp[0] for wp in waypoints]
        wp_y = [wp[1] for wp in waypoints]
        plt.plot(wp_x, wp_y, 'bo', markersize=8, label='Waypoints')

    plt.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')

    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')
 
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Random Waypoint Mobility - Drone Flight Path (2D)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')

    plt.axhline(y=BOX_LIMIT, color='r', linestyle='--', alpha=0.5, label='Boundary')
    plt.axhline(y=-BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
    plt.axvline(x=BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
    plt.axvline(x=-BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
    
    margin = BOX_LIMIT * 0.2
    plt.xlim(-BOX_LIMIT - margin, BOX_LIMIT + margin)
    plt.ylim(-BOX_LIMIT - margin, BOX_LIMIT + margin)
    
    plt.tight_layout()
    plt.show()
    
    plt.savefig('drone_path_2d.png', dpi=300, bbox_inches='tight')
    print("Plot saved as 'drone_path_2d.png'")

if __name__ == '__main__':
    print("Initializing drivers...")
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected!")

        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_deck_flow)
        
        time.sleep(1)  # Allow time for the deck check to complete

        # If no flow deck is detected within 5 seconds, exit the script
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected! Exiting...')
            sys.exit(1)

        # Execute the Random Waypoint Mobility pattern
        try:
            simple_random_waypoint_mobility(scf)
        except Exception as e:
            print(f"Error during flight: {e}")
        
        # Plot the results
        print(f"Flight completed! Total positions logged: {len(positions)}")
        print(f"Total waypoints visited: {len(waypoints)}")
        plot_path()