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
BOX_LIMIT = 0.5         # Boundary limits for movement in meters
MAX_VEL = 0.15          # Maximum velocity of the drone
MAX_RUN_TIME = 180      # Maximum run time in seconds
MIN_DISTANCE = 0.2      # Minimum safe distance from obstacles
FINISH_LINE_X = 1.5     # X-coordinate of the finish line (destination)

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
    """Check if an object is too close to the drone."""
    return range_value is not None and range_value < MIN_DISTANCE

def is_within_boundary(x, y):
    """Check if the position is within the virtual boundary."""
    return abs(x) <= BOX_LIMIT and abs(y) <= BOX_LIMIT

def has_reached_finish_line(x):
    """Check if the drone has crossed the finish line."""
    return x >= FINISH_LINE_X

def log_position(motion_commander):
    """Log the current position and return coordinates."""
    global position_log
    x = motion_commander._x
    y = motion_commander._y
    position_log.append((x, y))
    return x, y

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

def avoid_obstacle(motion_commander, multiranger):
    """
    Implement obstacle avoidance strategy.
    Returns True if collision was detected.
    """
    global collision_count
    
    obstacle_detected = False
    
    # Check for obstacles in all directions
    front_blocked = is_close(multiranger.front)
    back_blocked = is_close(multiranger.back)
    left_blocked = is_close(multiranger.left)
    right_blocked = is_close(multiranger.right)
    
    if front_blocked or back_blocked or left_blocked or right_blocked:
        obstacle_detected = True
        collision_count += 1
        
        # Retreat backward if front is blocked
        if front_blocked:
            print(f"Obstacle in front! Collision #{collision_count} - Retreating...")
            motion_commander.back(0.1, velocity=MAX_VEL)
            time.sleep(0.1)
            
            # Try to move right or left to avoid
            if not right_blocked:
                print("  -> Moving right to avoid")
                motion_commander.right(0.15, velocity=MAX_VEL)
            elif not left_blocked:
                print("  -> Moving left to avoid")
                motion_commander.left(0.15, velocity=MAX_VEL)
            time.sleep(0.1)
        
        # Move forward if back is blocked
        elif back_blocked:
            print(f"Obstacle behind! Collision #{collision_count} - Moving forward...")
            motion_commander.forward(0.1, velocity=MAX_VEL)
            time.sleep(0.1)
        
        # Move right if left is blocked
        if left_blocked:
            print(f"Obstacle on left! Collision #{collision_count} - Moving right...")
            motion_commander.right(0.1, velocity=MAX_VEL)
            time.sleep(0.1)
        
        # Move left if right is blocked
        elif right_blocked:
            print(f"Obstacle on right! Collision #{collision_count} - Moving left...")
            motion_commander.left(0.1, velocity=MAX_VEL)
            time.sleep(0.1)
    
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
                        current_x, current_y = log_position(motion_commander)
                        
                        # Check if finish line is reached
                        if has_reached_finish_line(current_x):
                            print(f"\n🎉 Finish line reached at X = {current_x:.3f}m!")
                            break
                        
                        # Check virtual boundary
                        if not is_within_boundary(current_x, current_y):
                            print(f"Warning: Approaching boundary at ({current_x:.2f}, {current_y:.2f})")
                            # Move back toward center
                            if abs(current_x) > BOX_LIMIT:
                                motion_commander.back(0.1, velocity=MAX_VEL) if current_x > 0 else motion_commander.forward(0.1, velocity=MAX_VEL)
                            if abs(current_y) > BOX_LIMIT:
                                motion_commander.left(0.1, velocity=MAX_VEL) if current_y > 0 else motion_commander.right(0.1, velocity=MAX_VEL)
                            time.sleep(0.2)
                            continue
                        
                        # Check for obstacles and avoid them
                        obstacle_detected = avoid_obstacle(motion_commander, multiranger)
                        
                        # If no obstacle, move forward toward destination
                        if not obstacle_detected:
                            motion_commander.forward(0.1, velocity=MAX_VEL)
                        
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