import logging
import sys
import time
import random
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.5

# Game variables
num_win = 0
num_lose = 0
game_log = []
round_number = 0


def param_deck_flow(name, value_str):
    """Callback to check if required deck is attached"""
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f'Deck {name} is attached!')
    else:
        print(f'Deck {name} is NOT attached!')


def set_led_color(cf, r, g, b):
    """
    Set the LED ring color on the Crazyflie
    r, g, b: RGB values (0-255)
    """
    cf.param.set_value('ring.effect', '7')  # Solid color effect
    cf.param.set_value('ring.solidRed', str(r))
    cf.param.set_value('ring.solidGreen', str(g))
    cf.param.set_value('ring.solidBlue', str(b))


def flash_led(cf, r, g, b, duration=0.5):
    """Flash LED in specified color"""
    set_led_color(cf, r, g, b)
    time.sleep(duration)
    set_led_color(cf, 0, 0, 0)  # Turn off


def get_user_choice():
    """Get user's choice for Rock, Scissors, or Paper"""
    while True:
        choice = input("\nEnter your choice (R for Rock, S for Scissors, P for Paper): ").upper()
        if choice in ['R', 'S', 'P']:
            return choice
        else:
            print("Invalid input! Please enter R, S, or P.")


def get_drone_choice():
    """Generate random choice for the drone"""
    choices = ['R', 'S', 'P']
    return random.choice(choices)


def determine_winner(user_choice, drone_choice):
    """
    Determine the winner of the round
    Returns: 'WIN' if drone wins, 'LOSE' if drone loses, 'TIE' if tie
    """
    if user_choice == drone_choice:
        return 'TIE'
    
    # Drone wins scenarios
    if (drone_choice == 'R' and user_choice == 'S') or \
       (drone_choice == 'S' and user_choice == 'P') or \
       (drone_choice == 'P' and user_choice == 'R'):
        return 'WIN'
    else:
        return 'LOSE'


def get_choice_name(choice):
    """Convert choice letter to full name"""
    names = {'R': 'Rock', 'S': 'Scissors', 'P': 'Paper'}
    return names[choice]


def play_rsp_game(scf):
    """Main game logic with drone control and LED feedback"""
    global num_win, num_lose, game_log, round_number
    
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("\n" + "="*60)
        print("ROCK-SCISSORS-PAPER GAME WITH CRAZYFLIE DRONE")
        print("="*60)
        print(f"Starting altitude: {DEFAULT_HEIGHT}m")
        print("Game rules: First to 2 wins or 2 losses ends the game")
        print("Drone moves UP on win, DOWN on loss, STAYS on tie")
        print("LED: Blue for WIN, Red for LOSS, Green for TIE")
        print("="*60)
        
        # Start with white LED to indicate ready
        set_led_color(scf.cf, 255, 255, 255)
        time.sleep(2)
        set_led_color(scf.cf, 0, 0, 0)
        
        # Game loop
        while num_win < 2 and num_lose < 2:
            round_number += 1
            print(f"\n{'='*60}")
            print(f"ROUND {round_number}")
            print(f"Current Score - Drone Wins: {num_win}, Drone Losses: {num_lose}")
            print(f"{'='*60}")
            
            # Get choices
            user_choice = get_user_choice()
            
            # Flash yellow LED while "thinking"
            flash_led(scf.cf, 255, 255, 0, 0.3)
            
            drone_choice = get_drone_choice()
            
            print(f"\nYou chose: {get_choice_name(user_choice)}")
            print(f"Drone chose: {get_choice_name(drone_choice)}")
            
            # Determine winner
            result = determine_winner(user_choice, drone_choice)
            
            # Execute drone action based on result
            if result == 'WIN':
                print(">>> Drone WINS this round! Moving UP 0.1m")
                num_win += 1
                game_log.append("Drone (W)")
                
                # Blue LED for win
                set_led_color(scf.cf, 0, 0, 255)
                mc.up(0.1)
                time.sleep(2)
                set_led_color(scf.cf, 0, 0, 0)
                
                # Check if drone won the game
                if num_win == 2:
                    print("\n*** DRONE WINS THE GAME! Landing... ***")
                    # Victory flash - blue pulsing
                    for _ in range(3):
                        set_led_color(scf.cf, 0, 0, 255)
                        time.sleep(0.3)
                        set_led_color(scf.cf, 0, 0, 0)
                        time.sleep(0.3)
                    
            elif result == 'LOSE':
                print(">>> Drone LOSES this round! Moving DOWN 0.1m")
                num_lose += 1
                game_log.append("Drone (L)")
                
                # Red LED for loss
                set_led_color(scf.cf, 255, 0, 0)
                mc.down(0.1)
                time.sleep(2)
                set_led_color(scf.cf, 0, 0, 0)
                
                # Check if drone lost the game
                if num_lose == 2:
                    print("\n*** USER WINS THE GAME! Drone landing... ***")
                    # Defeat flash - red pulsing
                    for _ in range(3):
                        set_led_color(scf.cf, 255, 0, 0)
                        time.sleep(0.3)
                        set_led_color(scf.cf, 0, 0, 0)
                        time.sleep(0.3)
                    
            else:  # TIE
                print(">>> It's a TIE! Drone stays at current position")
                game_log.append("Tie")
                
                # Green LED for tie
                set_led_color(scf.cf, 0, 255, 0)
                time.sleep(2)
                set_led_color(scf.cf, 0, 0, 0)
        
        # Game ended, landing with white LED
        print("\n" + "="*60)
        print("GAME OVER - LANDING DRONE")
        print("="*60)
        set_led_color(scf.cf, 255, 255, 255)
        time.sleep(1)
        mc.stop()
        set_led_color(scf.cf, 0, 0, 0)


def display_game_summary():
    """Display the full game log and statistics"""
    print("\n" + "="*60)
    print("GAME SUMMARY")
    print("="*60)
    print(f"\nTotal number of rounds played: {round_number}")
    print(f"\nFinal Score:")
    print(f"  Drone Wins: {num_win}")
    print(f"  Drone Losses: {num_lose}")
    print(f"  Ties: {round_number - num_win - num_lose}")
    
    print(f"\nFull log of outcomes:")
    for i, outcome in enumerate(game_log, 1):
        print(f"  Round {i}: {outcome}")
    
    if num_win == 2:
        print("\nFINAL RESULT: DRONE WINS THE GAME!")
    else:
        print("\nFINAL RESULT: USER WINS THE GAME!")
    
    print("="*60)


if __name__ == '__main__':
    print("="*60)
    print("INITIALIZING CRAZYFLIE DRONE SYSTEM")
    print("="*60)
    
    print("\nInitializing drivers...")
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected successfully!")

        # Add callbacks to check if required decks are attached
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_deck_flow)
        
        time.sleep(1)  # Allow time for the deck check to complete

        # If no flow deck is detected within 5 seconds, exit the script
        if not deck_attached_event.wait(timeout=5):
            print('\n ERROR: No flow deck detected! Exiting for safety...')
            sys.exit(1)

        print("\n✓ Flow deck detected - Safe to fly!")
        print("\nPreparing to launch drone...")
        time.sleep(1)
        
        # Play the game
        try:
            play_rsp_game(scf)
        except KeyboardInterrupt:
            print("\n\n Game interrupted by user!")
        except Exception as e:
            print(f"\n\n Error during game: {e}")
        finally:
            # Always display summary
            display_game_summary()
            print("\nDrone landed safely. Goodbye!")