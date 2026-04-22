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
DEFAULT_HEIGHT = 0.3    # Default flight height in meters
BOX_LIMIT = 4.0         # Boundary limits for movement in meters
MAX_VEL = 0.15          # Maximum velocity of the drone
MAX_RUN_TIME = 180      # Maximum run time in seconds
MIN_DISTANCE = 0.2      # Minimum safe distance from obstacles
FINISH_LINE_X = 3.0     # X-coordinate of the finish line (adjusted to be reachable)
SIDE_MOVE_DISTANCE = 0.2  # How far to move sideways when avoiding obstacles

# ==================== GLOBAL VARIABLES ====================
position_log = []       # List to store (x, y) positions
collision_count = 0     # Counter for collisions
start_time = None       # Track start time
keep_flying = True      # Flag to control flight loop
current_x = 0.0         # Current X position estimate
current_y = 0.0         # Current Y position estimate
avoiding_obstacle = False  # Flag to track if currently in avoidance mode

# ==================== SIGNAL HANDLER ====================
def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully stop the drone."""
    global keep_flying
    print('\nCtrl+C detected! Landing...')
    keep_flying = False

signal.signal(signal.SIGINT, signal_handler)

# ==================== HELPER FUNCTIONS ====================
def is_close(range_value):
    """Check if an object is too close to the drone."""
    return range_value is not None and range_value < MIN_DISTANCE

def is_within_boundary(x, y):
    """Check if the position is within the virtual boundary."""
    return abs(x) <= BOX_LIMIT and abs(y) <= BOX_LIMIT

def has_reached_finish_line(x):
    """Check if the drone has crossed the finish line."""
    return x >= FINISH_LINE_X

def update_position(distance, direction):
    """
    Update the estimated position based on movement.
    
    Args:
        distance: Distance moved in meters
        direction: 'forward', 'back', 'left', or 'right'
    """
    global current_x, current_y
    
    if direction == 'forward':
        current_x += distance
    elif direction == 'back':
        current_x -= distance
    elif direction == 'left':
        current_y += distance
    elif direction == 'right':
        current_y -= distance

def log_position():
    """Log the current position."""
    global position_log
    position_log.append((current_x, current_y))
    return current_x, current_y

def plot_path():
    """Plot the drone's flight path using matplotlib."""
    if len(position_log) < 2:
        print("Not enough position data to plot.")
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
             markersize=8, markeredgewidth=1.5, label='Position Markers')
    
    # Mark start and end points
    plt.plot(x_coords[0], y_coords[0], 'go', markersize=12, label='Start')
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='End')
    
    # Draw boundary box
    boundary_x = [-BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT, -BOX_LIMIT]
    boundary_y = [-BOX_LIMIT, -BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT]
    plt.plot(boundary_x, boundary_y, 'k--', linewidth=2, label='Virtual Boundary')
    
    # Draw finish line
    plt.axvline(x=FINISH_LINE_X, color='g', linestyle='--', linewidth=2, label='Finish Line')
    
    # Labels and formatting
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title(f'Drone Flight Path\nCollisions: {collision_count}', fontsize=14, fontweight='bold')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Save and show the plot
    plt.savefig('flight_path.png', dpi=300, bbox_inches='tight')
    print(f"\nFlight path plot saved to: flight_path.png")
    plt.show()
    
    print(f"\n===== Flight Statistics =====")
    print(f"Total Collisions: {collision_count}")
    print(f"Total Position Samples: {len(position_log)}")
    print(f"Final Position: ({x_coords[-1]:.3f}, {y_coords[-1]:.3f})")
    print(f"=============================\n")

def navigate_with_avoidance(motion_commander, multiranger):
    """
    Improved obstacle avoidance with better path planning.
    Returns True if obstacle was detected.
    """
    global collision_count, avoiding_obstacle
    
    obstacle_detected = False
    MOVE_DIST = 0.1  # Distance to move during normal operations
    
    # Check for obstacles in all directions
    front_blocked = is_close(multiranger.front)
    back_blocked = is_close(multiranger.back)
    left_blocked = is_close(multiranger.left)
    right_blocked = is_close(multiranger.right)
    
    if front_blocked:
        obstacle_detected = True
        collision_count += 1
        avoiding_obstacle = True
        
        print(f"🚧 Obstacle in front! Collision #{collision_count}")
        
        # Step 1: Back up more to get clear space
        print("  -> Backing up...")
        motion_commander.back(0.15, velocity=MAX_VEL)
        update_position(0.15, 'back')
        time.sleep(0.2)
        
        # Step 2: Decide which way to go around (prefer right, but check boundaries)
        can_go_right = not right_blocked and is_within_boundary(current_x, current_y - SIDE_MOVE_DISTANCE)
        can_go_left = not left_blocked and is_within_boundary(current_x, current_y + SIDE_MOVE_DISTANCE)
        
        if can_go_right:
            print(f"  -> Moving RIGHT {SIDE_MOVE_DISTANCE}m to go around obstacle")
            motion_commander.right(SIDE_MOVE_DISTANCE, velocity=MAX_VEL)
            update_position(SIDE_MOVE_DISTANCE, 'right')
            time.sleep(0.2)
            
            # Try to move forward alongside the obstacle
            if not is_close(multiranger.front):
                print("  -> Moving forward alongside obstacle")
                motion_commander.forward(0.2, velocity=MAX_VEL)
                update_position(0.2, 'forward')
                time.sleep(0.2)
                
        elif can_go_left:
            print(f"  -> Moving LEFT {SIDE_MOVE_DISTANCE}m to go around obstacle")
            motion_commander.left(SIDE_MOVE_DISTANCE, velocity=MAX_VEL)
            update_position(SIDE_MOVE_DISTANCE, 'left')
            time.sleep(0.2)
            
            # Try to move forward alongside the obstacle
            if not is_close(multiranger.front):
                print("  -> Moving forward alongside obstacle")
                motion_commander.forward(0.2, velocity=MAX_VEL)
                update_position(0.2, 'forward')
                time.sleep(0.2)
        else:
            print("  -> Stuck! Can't go around. Backing up more...")
            motion_commander.back(0.2, velocity=MAX_VEL)
            update_position(0.2, 'back')
            time.sleep(0.2)
    
    elif back_blocked:
        obstacle_detected = True
        collision_count += 1
        print(f"Obstacle behind! Collision #{collision_count} - Moving forward...")
        motion_commander.forward(MOVE_DIST, velocity=MAX_VEL)
        update_position(MOVE_DIST, 'forward')
        time.sleep(0.1)
    
    elif left_blocked:
        obstacle_detected = True
        collision_count += 1
        print(f"Obstacle on left! Collision #{collision_count} - Moving right...")
        if is_within_boundary(current_x, current_y - MOVE_DIST):
            motion_commander.right(MOVE_DIST, velocity=MAX_VEL)
            update_position(MOVE_DIST, 'right')
        time.sleep(0.1)
    
    elif right_blocked:
        obstacle_detected = True
        collision_count += 1
        print(f"Obstacle on right! Collision #{collision_count} - Moving left...")
        if is_within_boundary(current_x, current_y + MOVE_DIST):
            motion_commander.left(MOVE_DIST, velocity=MAX_VEL)
            update_position(MOVE_DIST, 'left')
        time.sleep(0.1)
    
    else:
        # No obstacles detected - clear to proceed
        if avoiding_obstacle:
            print("✓ Path is clear! Resuming forward movement")
            avoiding_obstacle = False
    
    return obstacle_detected

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
    
    # Initialize to avoid UnboundLocalError
    elapsed_time = 0
    
    try:
        # Establish a synchronous connection to the Crazyflie
        with SyncCrazyflie(URI, cf=cf) as scf:
            print("Connected to Crazyflie!")
            
            # Send an arming request to enable the drone for flight
            scf.cf.platform.send_arming_request(True)
            print("Drone armed. Waiting for stabilization...")
            time.sleep(1.0)
            
            # Enter motion control mode
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
                print(f"Taking off to {DEFAULT_HEIGHT}m altitude...")
                time.sleep(2)  # Allow takeoff to complete
                
                # Activate the Multi-ranger deck for distance sensing
                with Multiranger(scf) as multiranger:
                    print("Multi-ranger activated. Starting autonomous navigation...")
                    print(f"Virtual boundary: ±{BOX_LIMIT}m")
                    print(f"Finish line at X = {FINISH_LINE_X}m")
                    print("Press Ctrl-C to land early, or place hand above drone.\n")
                    
                    start_time = time.time()
                    
                    # Reset position to (0, 0) at start
                    current_x = 0.0
                    current_y = 0.0
                    
                    while keep_flying:
                        # Check elapsed time
                        elapsed_time = time.time() - start_time
                        if elapsed_time > MAX_RUN_TIME:
                            print(f"\nMaximum run time ({MAX_RUN_TIME}s) reached. Landing...")
                            break
                        
                        # Check if hand is placed above drone
                        if is_close(multiranger.up):
                            print("\nObject detected above drone. Landing...")
                            break
                        
                        # Log current position
                        pos_x, pos_y = log_position()
                        
                        # Check if finish line is reached
                        if has_reached_finish_line(pos_x):
                            print(f"\n🎉 Finish line reached at X = {pos_x:.3f}m!")
                            break
                        
                        # Check virtual boundary
                        if not is_within_boundary(pos_x, pos_y):
                            print(f"⚠️  Warning: Hit boundary at ({pos_x:.2f}, {pos_y:.2f})")
                            # Move back toward center
                            if abs(pos_x) > BOX_LIMIT:
                                if pos_x > 0:
                                    print("  -> Moving back from X boundary")
                                    motion_commander.back(0.15, velocity=MAX_VEL)
                                    update_position(0.15, 'back')
                                else:
                                    print("  -> Moving forward from X boundary")
                                    motion_commander.forward(0.15, velocity=MAX_VEL)
                                    update_position(0.15, 'forward')
                            if abs(pos_y) > BOX_LIMIT:
                                if pos_y > 0:
                                    print("  -> Moving right from Y boundary")
                                    motion_commander.right(0.15, velocity=MAX_VEL)
                                    update_position(0.15, 'right')
                                else:
                                    print("  -> Moving left from Y boundary")
                                    motion_commander.left(0.15, velocity=MAX_VEL)
                                    update_position(0.15, 'left')
                            time.sleep(0.3)
                            continue
                        
                        # Navigate with improved obstacle avoidance
                        obstacle_detected = navigate_with_avoidance(motion_commander, multiranger)
                        
                        # If no obstacle, move forward toward destination
                        if not obstacle_detected:
                            motion_commander.forward(0.1, velocity=MAX_VEL)
                            update_position(0.1, 'forward')
                        
                        time.sleep(0.1)  # Short delay before next iteration
                    
                    # Land the drone
                    print("\nLanding drone...")
        
        print("\n" + "=" * 60)
        print("MISSION COMPLETE")
        print("=" * 60)
        print(f"Flight duration: {elapsed_time:.2f} seconds")
        print(f"Total collisions: {collision_count}")
        print(f"Total positions logged: {len(position_log)}")
        if position_log:
            print(f"Final position: X = {position_log[-1][0]:.3f}m, Y = {position_log[-1][1]:.3f}m")
        print("=" * 60 + "\n")
        
        # Plot the flight path
        print("Generating flight path visualization...")
        plot_path()
        
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nProgram terminated successfully!")