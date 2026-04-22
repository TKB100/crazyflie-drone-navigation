import logging
import sys
import time
from threading import Event
import matplotlib.pyplot as plt
import random
import math

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
MAX_DUMMY = 2  # Maximum number of dummy waypoints
MAX_RUN_TIME = 30
MAX_DESTINATIONS = 5  # Maximum number of destinations to visit

positions = []
destinations = []  # Store actual destinations
dummies = []  # Store dummy waypoints
flight_active = True  

def param_deck_flow(name, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f'Deck {name} is attached!')
    else:
        print(f'Deck {name} is NOT attached!')

def generate_random_destination():
    """Generate a random destination within the box limits (2D only)"""
    x = random.uniform(-BOX_LIMIT, BOX_LIMIT)
    y = random.uniform(-BOX_LIMIT, BOX_LIMIT)
    return x, y

def generate_dummy_waypoints(current_pos, destination, num_dummies):
    """
    Generate dummy waypoints within a virtual rectangle.
    The rectangle's diagonal spans from current_pos to destination.
    """
    dummy_waypoints = []
    
    # Define the rectangle boundaries
    min_x = min(current_pos[0], destination[0])
    max_x = max(current_pos[0], destination[0])
    min_y = min(current_pos[1], destination[1])
    max_y = max(current_pos[1], destination[1])
    
    # Handle edge case where rectangle has zero width or height
    if abs(max_x - min_x) < 0.01:
        min_x -= 0.05
        max_x += 0.05
    if abs(max_y - min_y) < 0.01:
        min_y -= 0.05
        max_y += 0.05
    
    # Generate random dummy waypoints within the rectangle
    for i in range(num_dummies):
        dummy_x = random.uniform(min_x, max_x)
        dummy_y = random.uniform(min_y, max_y)
        dummy_waypoints.append((dummy_x, dummy_y))
    
    return dummy_waypoints

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def sort_waypoints_by_distance(current_pos, waypoint_list):
    """Sort waypoints by distance from current position (closest first)"""
    if not waypoint_list:
        return []
    return sorted(waypoint_list, key=lambda wp: calculate_distance(current_pos, wp))

def log_position(x, y):
    """Log current position"""
    positions.append((x, y))
    print(f"Position: ({x:.3f}, {y:.3f})")

def check_time_remaining(flight_start_time):
    """Check if there's time remaining in the flight"""
    elapsed = time.time() - flight_start_time
    remaining = MAX_RUN_TIME - elapsed
    return remaining > 0, elapsed, remaining

def privacy_preserving_random_waypoint_mobility(scf):
    """Execute the Privacy-Preserving Random Waypoint Mobility pattern"""
    global flight_active
    
    mc = None
    try:
        mc = MotionCommander(scf, default_height=DEFAULT_HEIGHT)
        mc.take_off(height=DEFAULT_HEIGHT)
        
        print(f"Starting Privacy-Preserving Random Waypoint Mobility pattern...")
        print(f"Will visit {MAX_DESTINATIONS} destinations...")
       
        current_pos = (0.0, 0.0)
        log_position(current_pos[0], current_pos[1])
        
        time.sleep(2)
    
        flight_start_time = time.time()
        destination_count = 0
        
        while flight_active and destination_count < MAX_DESTINATIONS:
            # Check time at the start of each iteration
            has_time, elapsed, remaining = check_time_remaining(flight_start_time)
            if not has_time:
                print(f"\n30 seconds completed! (Total time: {elapsed:.1f}s)")
                break
            
            # Don't start a new destination if we have less than 3 seconds
            if remaining < 3:
                print(f"\nLess than 3 seconds remaining ({remaining:.1f}s), stopping...")
                break
            
            destination_count += 1
            print(f"\n=== Destination {destination_count}/{MAX_DESTINATIONS} ===")

            # Generate a random destination
            destination = generate_random_destination()
            destinations.append(destination)
            print(f"Target destination: ({destination[0]:.3f}, {destination[1]:.3f})")
            
            # Generate random number of dummy waypoints (0 to MAX_DUMMY)
            num_dummies = random.randint(0, MAX_DUMMY)
            print(f"Generating {num_dummies} dummy waypoints...")
            
            dummy_waypoints = generate_dummy_waypoints(current_pos, destination, num_dummies)
            
            # Store dummy waypoints for plotting
            for dummy in dummy_waypoints:
                dummies.append(dummy)
                print(f"  Dummy waypoint: ({dummy[0]:.3f}, {dummy[1]:.3f})")
            
            # Create path: current -> sorted dummies -> destination
            # Sort dummies by distance from current position
            sorted_dummies = sort_waypoints_by_distance(current_pos, dummy_waypoints)
            
            # Build complete path
            path = sorted_dummies + [destination]
            
            print(f"Flight path: Current -> {len(sorted_dummies)} dummies -> Destination")
            
            # Fly through each waypoint in the path
            for idx, waypoint in enumerate(path):
                # Check time before each movement
                has_time, elapsed, remaining = check_time_remaining(flight_start_time)
                if not has_time:
                    print(f"\n30 seconds reached during flight! (Total time: {elapsed:.1f}s)")
                    flight_active = False
                    break
                
                is_destination = (idx == len(path) - 1)
                waypoint_type = "DESTINATION" if is_destination else f"Dummy {idx+1}"
                
                print(f"\n  -> Flying to {waypoint_type}: ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
                
                try:
                    # Calculate movement
                    dx = waypoint[0] - current_pos[0]
                    dy = waypoint[1] - current_pos[1]
                    
                    # Move to waypoint with velocity limit
                    mc.move_distance(dx, dy, 0, velocity=MAX_VEL)
                    
                    # Update current position
                    current_pos = waypoint
                    log_position(current_pos[0], current_pos[1])
                    
                except Exception as e:
                    print(f"Error during movement: {e}")
                    flight_active = False
                    break
            
            if not flight_active:
                break
            
            # Check if we exceeded time after completing path
            has_time, elapsed, remaining = check_time_remaining(flight_start_time)
            
            if not has_time:
                print(f"\n30 seconds completed after reaching destination! (Total time: {elapsed:.1f}s)")
                break
            
            # Check if we've reached the maximum number of destinations
            if destination_count >= MAX_DESTINATIONS:
                print(f"\nReached maximum of {MAX_DESTINATIONS} destinations!")
                break
            
            # Pause at destination (only if we have time left and more destinations to visit)
            if destination_count < MAX_DESTINATIONS:
                pause_duration = min(MOVE_DURATION, remaining)
                print(f"\nArrived at destination! Pausing for {pause_duration:.1f} seconds...")
                print(f"Time elapsed: {elapsed:.1f}s / {MAX_RUN_TIME}s")
                
                time.sleep(pause_duration)

                # Final check after pause
                has_time, elapsed, remaining = check_time_remaining(flight_start_time)
                if not has_time:
                    print(f"\n30 seconds completed during pause! (Total time: {elapsed:.1f}s)")
                    break
    
    except KeyboardInterrupt:
        print("\n\nFlight interrupted by user!")
        flight_active = False
        
    except Exception as e:
        print(f"\nUnexpected error during flight: {e}")
        import traceback
        traceback.print_exc()
        flight_active = False
        
    finally:
        # Always land the drone
        if mc is not None:
            try:
                print("\nLanding drone...")
                mc.land()
                time.sleep(2)
                print("Drone landed successfully!")
            except Exception as e:
                print(f"Error during landing: {e}")
        
        total_time = time.time() - flight_start_time
        print(f"\n=== Flight Summary ===")
        print(f"Total flight time: {total_time:.1f} seconds")
        print(f"Destinations visited: {destination_count}/{MAX_DESTINATIONS}")
        print(f"Total dummy waypoints generated: {len(dummies)}")

def plot_path():
    """Create x-y plot of the drone's path with destinations and dummies"""
    if not positions:
        print("No position data to plot!")
        return

    try:
        x_coords = [pos[0] for pos in positions]
        y_coords = [pos[1] for pos in positions]
        
        plt.figure(figsize=(10, 8))

        # Plot drone path as solid line
        plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Drone Path')

        # Plot destinations as solid blue circles
        if destinations:
            dest_x = [dest[0] for dest in destinations]
            dest_y = [dest[1] for dest in destinations]
            plt.plot(dest_x, dest_y, 'bo', markersize=10, markerfacecolor='blue', 
                    markeredgecolor='blue', label='Destinations')

        # Plot dummy waypoints as empty circles
        if dummies:
            dummy_x = [dummy[0] for dummy in dummies]
            dummy_y = [dummy[1] for dummy in dummies]
            plt.plot(dummy_x, dummy_y, 'o', markersize=8, markerfacecolor='none', 
                    markeredgecolor='black', markeredgewidth=2, label='Dummy Waypoints')

        # Plot start position
        plt.plot(x_coords[0], y_coords[0], 'go', markersize=12, label='Start')

        # Plot end position
        plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='End')
     
        plt.xlabel('X Position (m)', fontsize=12)
        plt.ylabel('Y Position (m)', fontsize=12)
        plt.title('Privacy-Preserving Random Waypoint Mobility - Drone Flight Path (2D)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(loc='best')
        plt.axis('equal')

        # Plot boundary
        plt.axhline(y=BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
        plt.axhline(y=-BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
        plt.axvline(x=BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
        plt.axvline(x=-BOX_LIMIT, color='r', linestyle='--', alpha=0.5)
        
        margin = BOX_LIMIT * 0.2
        plt.xlim(-BOX_LIMIT - margin, BOX_LIMIT + margin)
        plt.ylim(-BOX_LIMIT - margin, BOX_LIMIT + margin)
        
        plt.tight_layout()
        
        # Save the plot
        filename = 'drone_path_privacy_preserving_2d.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved as '{filename}'")
        
        # Show the plot
        plt.show()
        
    except Exception as e:
        print(f"Error creating plot: {e}")
        import traceback
        traceback.print_exc()

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

        # Execute the Privacy-Preserving Random Waypoint Mobility pattern
        try:
            privacy_preserving_random_waypoint_mobility(scf)
            print("\nFlight sequence completed!")
        except Exception as e:
            print(f"\nCritical error: {e}")
            import traceback
            traceback.print_exc()
        
        # Always plot the results, even if flight was interrupted
        print(f"\n=== Data Summary ===")
        print(f"Total positions logged: {len(positions)}")
        print(f"Total destinations: {len(destinations)}")
        print(f"Total dummy waypoints: {len(dummies)}")
        
        if len(positions) > 0:
            plot_path()
        else:
            print("No data to plot!")