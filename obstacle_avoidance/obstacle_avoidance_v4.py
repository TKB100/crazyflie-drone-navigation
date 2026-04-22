import sys
import time
import logging
import signal
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
import matplotlib.pyplot as plt

# Define the default URI for communication with the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# ==================== PARAMETERS ====================
DEFAULT_HEIGHT = 0.5    # Default flight height in meters
BOX_LIMIT = 0.5         # Boundary limits for movement in meters
MAX_VEL = 0.15          # Maximum velocity of the drone
MAX_RUN_TIME = 180      # Maximum run time in seconds
MIN_DISTANCE = 0.2      # Minimum safe distance from obstacles
FINISH_LINE_X = 0.4     # X-coordinate of the finish line (destination)

# ==================== GLOBAL VARIABLES ====================
position_log = []       # List to store (x, y) positions
collision_count = 0     # Counter for collisions
start_time = None       # Track start time
keep_flying = True      # Flag to control flight loop

# ==================== SIGNAL HANDLER ====================
def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully stop the drone."""
    global keep_flying
    print('\nCtrl+C detected! Landing...')
    keep_flying = False

signal.signal(signal.SIGINT, signal_handler)

# ==================== HELPER FUNCTIONS ====================
def is_close(range_value):
    """
    Check if an object is too close to the drone.
    
    Args:
        range_value (float or None): Distance measured by the sensor.
    
    Returns:
        bool: True if the object is closer than the minimum distance, otherwise False.
    """
    if range_value is None:
        return False
    return range_value < MIN_DISTANCE

def within_bounds(x, y):
    """
    Check if the position is within the virtual boundary.
    
    Args:
        x (float): X-coordinate
        y (float): Y-coordinate
    
    Returns:
        bool: True if within bounds, False otherwise
    """
    return abs(x) <= BOX_LIMIT and abs(y) <= BOX_LIMIT

def reached_finish_line(x):
    """
    Check if the drone has crossed the finish line.
    
    Args:
        x (float): Current X-coordinate
    
    Returns:
        bool: True if finish line is reached
    """
    return x >= FINISH_LINE_X

def log_position(motion_commander):
    """
    Log the current position of the drone.
    
    Args:
        motion_commander: MotionCommander instance to get position
    """
    global position_log
    # Get current position estimate
    x = motion_commander._x
    y = motion_commander._y
    position_log.append((x, y))
    return x, y

def plot_path():
    """
    Plot the drone's path using matplotlib.
    Shows the trajectory with intermediate positions marked as blue circles.
    """
    if len(position_log) < 2:
        print("Not enough data to plot.")
        return
    
    # Extract x and y coordinates
    x_coords = [pos[0] for pos in position_log]
    y_coords = [pos[1] for pos in position_log]
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Plot the path as a solid line
    plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Drone Path')
    
    # Mark intermediate positions as empty blue circles
    plt.plot(x_coords, y_coords, 'bo', markerfacecolor='none', 
             markersize=8, markeredgewidth=1.5, label='Positions')
    
    # Mark start and end points
    plt.plot(x_coords[0], y_coords[0], 'go', markersize=12, 
             label='Start', markeredgewidth=2)
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, 
             label='End', markeredgewidth=2)
    
    # Draw the virtual boundary box
    box_x = [-BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT, -BOX_LIMIT]
    box_y = [-BOX_LIMIT, -BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT]
    plt.plot(box_x, box_y, 'k--', linewidth=2, label='Virtual Boundary')
    
    # Draw the finish line
    plt.axvline(x=FINISH_LINE_X, color='g', linestyle='--', 
                linewidth=2, label='Finish Line')
    
    # Labels and title
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title(f'Drone Flight Path - Collisions: {collision_count}', fontsize=14)
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Save and show the plot
    plt.savefig('/mnt/user-data/outputs/drone_path.png', dpi=300, bbox_inches='tight')
    print(f"\nPath plot saved to: drone_path.png")
    plt.show()

def calculate_avoidance_action(multiranger, motion_commander, current_x, current_y):
    """
    Calculate and execute movement based on obstacle detection.
    Implements a basic path planning strategy.
    
    Args:
        multiranger: Multiranger sensor object
        motion_commander: MotionCommander object
        current_x: Current X position
        current_y: Current Y position
    
    Returns:
        bool: True if collision was detected
    """
    global collision_count
    
    collision_detected = False
    
    # Check for obstacles in all directions
    front_blocked = is_close(multiranger.front)
    back_blocked = is_close(multiranger.back)
    left_blocked = is_close(multiranger.left)
    right_blocked = is_close(multiranger.right)
    
    # Movement distance for each action
    MOVE_DISTANCE = 0.1  # meters
    
    # Obstacle avoidance logic
    if front_blocked:
        collision_detected = True
        collision_count += 1
        print(f"Obstacle detected in front! Collision #{collision_count}")
        
        # Move backward to retreat
        motion_commander.back(MOVE_DISTANCE, velocity=MAX_VEL)
        
        # Try to move around the obstacle
        if not right_blocked and within_bounds(current_x, current_y - MOVE_DISTANCE):
            # Prefer moving right
            motion_commander.right(MOVE_DISTANCE, velocity=MAX_VEL)
            print("  -> Moved backward and right")
        elif not left_blocked and within_bounds(current_x, current_y + MOVE_DISTANCE):
            # Otherwise move left
            motion_commander.left(MOVE_DISTANCE, velocity=MAX_VEL)
            print("  -> Moved backward and left")
        else:
            print("  -> Moved backward")
    
    elif back_blocked:
        collision_detected = True
        collision_count += 1
        print(f"Obstacle detected behind! Collision #{collision_count}")
        # Move forward
        motion_commander.forward(MOVE_DISTANCE, velocity=MAX_VEL)
    
    elif left_blocked:
        collision_detected = True
        collision_count += 1
        print(f"Obstacle detected on left! Collision #{collision_count}")
        # Move right
        if within_bounds(current_x, current_y - MOVE_DISTANCE):
            motion_commander.right(MOVE_DISTANCE, velocity=MAX_VEL)
    
    elif right_blocked:
        collision_detected = True
        collision_count += 1
        print(f"Obstacle detected on right! Collision #{collision_count}")
        # Move left
        if within_bounds(current_x, current_y + MOVE_DISTANCE):
            motion_commander.left(MOVE_DISTANCE, velocity=MAX_VEL)
    
    else:
        # No obstacles - move forward toward destination
        if within_bounds(current_x + MOVE_DISTANCE, current_y):
            motion_commander.forward(MOVE_DISTANCE, velocity=MAX_VEL)
    
    return collision_detected

# ==================== MAIN PROGRAM ====================
if __name__ == '__main__':
    print("=" * 60)
    print("SkySensor: Safe Racing without Collisions")
    print("=" * 60)
    print(f"Parameters:")
    print(f"  - Default Height: {DEFAULT_HEIGHT}m")
    print(f"  - Box Limit: ±{BOX_LIMIT}m")
    print(f"  - Max Velocity: {MAX_VEL}m/s")
    print(f"  - Max Run Time: {MAX_RUN_TIME}s")
    print(f"  - Finish Line: X = {FINISH_LINE_X}m")
    print("=" * 60)
    print("\nPress Ctrl+C to stop the drone at any time.\n")
    
    # Initialize the low-level drivers for Crazyflie communication
    cflib.crtp.init_drivers()
    
    # Create a Crazyflie object with a cache for parameters
    cf = Crazyflie(rw_cache='./cache')
    
    try:
        # Establish a synchronous connection to the Crazyflie
        with SyncCrazyflie(URI, cf=cf) as scf:
            print("Connected to Crazyflie!")
            
            # Send an arming request to enable the drone for flight
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)
            print("Drone armed. Taking off...\n")
            
            # Enter motion control mode
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
                print(f"Takeoff complete. Flying at {DEFAULT_HEIGHT}m altitude.")
                print("Starting obstacle avoidance navigation...\n")
                
                # Activate the Multi-ranger deck for distance sensing
                with Multiranger(scf) as multiranger:
                    start_time = time.time()
                    
                    while keep_flying:
                        # Check if maximum run time exceeded
                        elapsed_time = time.time() - start_time
                        if elapsed_time > MAX_RUN_TIME:
                            print(f"\nMaximum run time ({MAX_RUN_TIME}s) reached!")
                            keep_flying = False
                            break
                        
                        # Log current position
                        x, y = log_position(motion_commander)
                        
                        # Check if finish line reached
                        if reached_finish_line(x):
                            print(f"\n🎉 Finish line reached at X = {x:.2f}m!")
                            keep_flying = False
                            break
                        
                        # Check if hand is placed above (emergency stop)
                        if is_close(multiranger.up):
                            print("\nObject detected above! Emergency landing...")
                            keep_flying = False
                            break
                        
                        # Execute movement based on obstacles
                        collision = calculate_avoidance_action(multiranger, motion_commander, x, y)
                        
                        # Short delay before next sensor check
                        time.sleep(0.1)
                
                print("\nLanding...")
            
            print("Landed successfully!")
            
    except Exception as e:
        print(f"\nError occurred: {e}")
    
    finally:
        # Print summary
        print("\n" + "=" * 60)
        print("FLIGHT SUMMARY")
        print("=" * 60)
        print(f"Total flight time: {time.time() - start_time:.2f} seconds")
        print(f"Total positions logged: {len(position_log)}")
        print(f"Total collisions detected: {collision_count}")
        print(f"Final position: X = {position_log[0][0]:.2f}m, Y = {position_log[0][1]:.2f}m")
        print("=" * 60)
        
        # Plot the path
        print("\nGenerating path visualization...")
        plot_path()
        
        print("\n Demo terminated successfully!")