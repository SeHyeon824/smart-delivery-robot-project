# servo_sweep_test.py
# English comments are provided for clarity.

from gpiozero import Servo
from time import sleep

# --- Settings ---
# Connect the servo signal wire to GPIO 17 (physical pin 11)
SERVO_PIN = 17
# ----------------

# Initialize the Servo object
# For SG90, default pulse widths are usually fine,
# but they can be specified if needed:
# servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo = Servo(SERVO_PIN)

def sweep_test():
    """
    Sweeps the servo motor from its minimum, to middle, to maximum position,
    and then back, to test its full range of motion.
    """
    print("Starting servo sweep test.")
    print("The servo will move from min -> mid -> max -> mid -> min.")

    try:
        while True:
            # Move to minimum position (-90 degrees)
            print("Position: Min")
            servo.min()
            sleep(2)

            # Move to middle position (0 degrees)
            print("Position: Mid")
            servo.mid()
            sleep(2)

            # Move to maximum position (+90 degrees)
            print("Position: Max")
            servo.max()
            sleep(2)

            # Move back to middle position (0 degrees)
            print("Position: Mid")
            servo.mid()
            sleep(2)

    except KeyboardInterrupt:
        # Allows the user to stop the test with Ctrl+C
        print("\nTest stopped by user.")
    finally:
        # Cleanly release GPIO resources
        servo.close()
        print("Servo resources released.")

if __name__ == '__main__':
    sweep_test()
