# smart_safe_main.py

import cv2
import face_recognition
import pickle
import time
from time import sleep
import os
import socket
from picamera2 import Picamera2
from gpiozero import Servo

# --- Settings ---
KNOWN_ENCODING_PATH = 'my_final_encodings.pkl'
SERVO_PIN = 17
SOCKET_PORT = 8000
TELEGRAM_REQUEST_FLAG = 'request_telegram.flag'
RESPONSE_FLAG = 'response_telegram.flag'
IMAGE_TO_SAVE = "unauthorized.jpg"
UNLOCK_DURATION = 7 # Seconds the safe remains unlocked

def send_sound_command(command):
    """ Sends a sound command to the Xycar PC's speaker server. """
    XYCAR_PC_IP = "192.168.142.50"
    XYCAR_PC_PORT = 9000
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2) # Set a 2-second timeout
            s.connect((XYCAR_PC_IP, XYCAR_PC_PORT))
            s.sendall(command.encode())
            print(f"? Sound command '{command}' sent to Xycar PC.")
    except Exception as e:
        print(f"? [ERROR] Failed to send sound command: {e}")

# --- Initialization ---
def initialize():
    """ Initializes all hardware and models. """
    print("Initializing system...")
    
    if os.path.exists(TELEGRAM_REQUEST_FLAG): os.remove(TELEGRAM_REQUEST_FLAG)
    if os.path.exists(RESPONSE_FLAG): os.remove(RESPONSE_FLAG)

    with open(KNOWN_ENCODING_PATH, 'rb') as f:
        known_encodings = pickle.load(f)
    print(f"- {len(known_encodings)} face encodings loaded successfully.")

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    sleep(2)
    print("- Camera started successfully.")

    servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
    servo.min()
    sleep(1)
    servo.detach()
    print("- Servo motor initialized successfully.")

    return known_encodings, picam2, servo

# --- Core Functions ---
def unlock_safe(servo):
    """ Opens the safe and locks it again after the specified duration. """
    print("?? Unlocking safe...")
    # --- MODIFIED PART ---
    # Changed "UNLOCKED" back to "RECOGNIZED" to match the speaker server.
    send_sound_command("RECOGNIZED") 
    servo.max()
    sleep(1)
    servo.detach()
    
    print(f"Safe will remain unlocked for {UNLOCK_DURATION} seconds.")
    sleep(UNLOCK_DURATION)
    
    print("?? Locking safe again...")
    servo.min()
    sleep(1)
    servo.detach()

def run_face_recognition(known_encodings, picam2, servo):
    """
    New logic: Immediately requests Telegram approval while also trying
    face recognition. The first one to succeed wins.
    """
    print("?? Starting mission: Requesting admin approval and trying face recognition...")
    
    # --- STEP 1: Immediately take a photo and request Telegram approval ---
    initial_frame_rgba = picam2.capture_array("main")
    initial_frame_bgr = cv2.cvtColor(initial_frame_rgba, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(IMAGE_TO_SAVE, initial_frame_bgr)
    
    with open(TELEGRAM_REQUEST_FLAG, "w") as f:
        f.write("send")
    print("?? Telegram approval request sent. Waiting for response (60s) OR face recognition (10s)...")

    mission_start_time = time.time()
    face_rec_timeout = 10
    telegram_timeout = 60

    # --- STEP 2: Unified loop for checking both conditions ---
    while time.time() - mission_start_time < telegram_timeout:
        
        # --- Condition A: Check for Telegram Response (for up to 60 seconds) ---
        if os.path.exists(RESPONSE_FLAG):
            with open(RESPONSE_FLAG, "r") as f:
                result = f.read().strip()
            os.remove(RESPONSE_FLAG)

            if result == "approved":
                print("? Admin approval received!")
                unlock_safe(servo)
                return True # Mission Successful
            else:
                print("? Admin rejection received. Mission terminated.")
                send_sound_command("REJECTED")
                return False # Mission Failed
        
        # --- Condition B: Try Face Recognition (for the first 10 seconds) ---
        if time.time() - mission_start_time < face_rec_timeout:
            frame_rgba = picam2.capture_array("main")
            rgb_frame = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2RGB)
            
            face_locations = face_recognition.face_locations(rgb_frame, model='hog')
            if face_locations:
                face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
                for encoding in face_encodings:
                    matches = face_recognition.compare_faces(known_encodings, encoding, tolerance=0.6)
                    if True in matches:
                        print("? Face recognition successful!")
                        unlock_safe(servo)
                        return True # Mission Successful
        sleep(0.5) # Small delay to prevent high CPU usage

    # --- STEP 3: Handle timeout ---
    print("? No success from face recognition or admin response within the time limit. Mission terminated.")
    send_sound_command("FAIL")
    return False # Mission Failed

# --- Main Execution Loop ---
def main():
    """ Opens a socket server and waits for a signal from the Xycar. """
    known_encodings, picam2, servo = initialize()
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', SOCKET_PORT))
    server_socket.listen(1)
    print(f"?? Waiting for 'goal reached' signal from Xycar on port {SOCKET_PORT}...")

    try:
        while True:
            client_socket, addr = server_socket.accept()
            with client_socket:
                print(f"?? Xycar connected from: {addr}")
                data = client_socket.recv(1024).decode().strip()
                if data == "GOAL_REACHED":
                    print("?? Signal received! Entering mission mode.")
                    send_sound_command("START")
                    
                    run_face_recognition(known_encodings, picam2, servo)
                    
                    print(f"\n?? Mission sequence finished. Shutting down system.")
                    break
                    
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down system...")
    finally:
        picam2.stop()
        servo.close()
        server_socket.close()
        if os.path.exists(TELEGRAM_REQUEST_FLAG): os.remove(TELEGRAM_REQUEST_FLAG)
        if os.path.exists(RESPONSE_FLAG): os.remove(RESPONSE_FLAG)
        print("All devices and sockets have been closed.")

if __name__ == "__main__":
    main()
