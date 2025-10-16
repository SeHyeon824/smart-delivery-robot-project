# capture_faces.py
# English comments have been added for clarity.

from picamera2 import Picamera2
from time import sleep
import cv2
import os

# --- Settings ---
SAVE_DIR = "known_faces"  # Directory to save the photos
USER_NAME = "user"        # ?? Use your name in English here (e.g., sehyen)
# ----------------

def main():
    """
    Uses the Raspberry Pi camera to capture and save photos for face recognition training.
    """
    # Create the save directory if it doesn't exist
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
        print(f"Directory '{SAVE_DIR}' was created.")

    # Initialize and configure the camera. 
    # 'main' stream format is RGBA, which OpenCV needs to convert.
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    print("Starting camera... You can start capturing in 2 seconds...")
    sleep(2) # Wait for the camera to stabilize

    count = 1
    print("\n" + "="*55)
    print("  Get ready to capture photos while looking at the camera window.")
    print("  Press 's' on your keyboard to save a photo.")
    print("  Press 'q' on your keyboard to quit the program.")
    print("="*55)

    while True:
        # Capture the current frame from the camera (in RGBA format)
        frame_rgba = picam2.capture_array("main")
        
        # ? Convert the RGBA frame to BGR for correct color display and saving with OpenCV
        frame_bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)
        
        # Display the correctly colored BGR frame in a window
        cv2.imshow("Press 's' to save, 'q' to quit", frame_bgr)
        
        # Wait for a key press for 1ms
        key = cv2.waitKey(1) & 0xFF

        # If the 's' key is pressed
        if key == ord('s'):
            # Set the filename to save (e.g., known_faces/user_1.jpg)
            filename = os.path.join(SAVE_DIR, f"{USER_NAME}_{count}.jpg")
            
            # ? Save the correctly colored BGR frame as an image file
            cv2.imwrite(filename, frame_bgr)
            print(f"? Image saved: [{filename}]")
            count += 1
        
        # If the 'q' key is pressed
        elif key == ord('q'):
            print("Exiting capture program.")
            break

    # Release all resources when the program ends
    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
