# Raspberry Pi Application: Smart Safe & Dual Authentication

This directory contains the core application running on the Raspberry Pi, designed to manage the smart safe mechanism and implement a robust dual-authentication security system for the Smart Delivery Robot.

## 📁 Directory Structure

. ├── capture_faces.py # Utility to capture new user's face images for training ├── create_encodings.py # Generates face encodings (.pkl) from 'known_faces' for recognition ├── known_faces/ # Stores authorized user images for face recognition training │ ├── user_1.jpg │ └── ... ├── my_final_encodings.pkl # Output file: Pre-calculated face encodings for quick recognition ├── smart_safe_main.py # Main application: Handles authentication flow, safe control, and communication ├── telegram_manager.py # Manages communication with the Telegram Bot API for remote approval ├── servo_test.py # Test script for basic servo motor operation └── unauthorized.jpg # Image captured during unauthorized access attempts


## ✨ Key Features

* **1st-Factor: Face Recognition**: Real-time identification of authorized users using a connected camera and `face_recognition` library.
* **2nd-Factor: Telegram Remote Approval**: Secure fallback system; if face recognition fails, an image is sent to an admin via Telegram for remote access approval/rejection.
* **Smart Safe Control**: Direct control over a servo-actuated locking mechanism to secure deliveries.
* **Xycar Integration**: Seamless Socket communication with the Xycar's main PC for "goal reached" signals and status updates.

## 🛠️ Setup & Installation

1.  **Hardware**:
    * Connect a Raspberry Pi Camera Module.
    * Connect a servo motor to a suitable GPIO pin (e.g., BCM 18) for safe locking.

2.  **Dependencies**: Install required Python packages.
    ```bash
    # Ensure you are in the 'raspberry_pi_app' directory on your Raspberry Pi
    sudo apt update
    sudo apt install -y python3-picamera python3-opencv libopenjp2-7-dev libtiff5-dev
    pip install face_recognition RPi.GPIO python-telegram-bot Pillow
    ```

3.  **Telegram Bot Configuration**:
    * Create a new Telegram Bot via BotFather and obtain its **Bot Token**.
    * Find your **Telegram User ID**.
    * Update `telegram_manager.py` with these credentials (`BOT_TOKEN` and `ADMIN_CHAT_ID`).

4.  **Face Database Setup**:
    * Place images of authorized users (e.g., `user_1.jpg`, `user_2.jpg`) into the `known_faces/` directory.
    * Generate face encodings:
        ```bash
        python3 create_encodings.py
        ```
        This will create `my_final_encodings.pkl`.

## 🚀 Usage

To start the authentication and safe control system:

```bash
python3 smart_safe_main.py
