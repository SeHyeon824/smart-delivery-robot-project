# create_encodings.py

import face_recognition
import pickle
import os

# --- Settings ---
KNOWN_FACES_DIR = 'known_faces'
ENCODINGS_FILE = 'my_final_encodings.pkl'
# ----------------

def main():
    """
    Generates face encodings from images in a specified directory and saves them to a file.
    """
    known_face_encodings = []
    print(f"Starting to learn faces from the '{KNOWN_FACES_DIR}' directory...")

    for filename in os.listdir(KNOWN_FACES_DIR):
        # Process only image files
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            try:
                image_path = os.path.join(KNOWN_FACES_DIR, filename)
                image = face_recognition.load_image_file(image_path)
                
                # Extract face encodings from the image
                # This assumes one face per image; it will use the first one found.
                encodings = face_recognition.face_encodings(image)
                
                if encodings:
                    known_face_encodings.append(encodings[0])
                    print(f"- Learning complete for {filename}.")
                else:
                    print(f"?? Could not find a face in {filename}.")
            except Exception as e:
                print(f"? Error processing {filename}: {e}")

    # Save the learned encoding data to a pickle file
    with open(ENCODINGS_FILE, 'wb') as f:
        pickle.dump(known_face_encodings, f)
        
    print(f"\nLearned a total of {len(known_face_encodings)} faces. Saved to '{ENCODINGS_FILE}'.")

if __name__ == '__main__':
    main()
