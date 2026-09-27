#!/usr/bin/env python3
"""
enroll_faces.py — build known_faces encodings for perception_node.

Usage:
  1. Create: seagr_ros2/known_faces/Ranveer/*.jpg  (5-10 clear front-facing photos,
     varied lighting/angle helps)
  2. Run: python3 enroll_faces.py
  3. Produces encodings.pkl next to perception_node.py — delete it any time
     you add more photos and want to rebuild.
"""

import os
import pickle
import face_recognition

KNOWN_FACES_DIR = os.path.join(os.path.dirname(__file__), 'known_faces')
ENCODINGS_PATH = os.path.join(os.path.dirname(__file__), 'encodings.pkl')


def main():
    if not os.path.isdir(KNOWN_FACES_DIR):
        print(f"Missing {KNOWN_FACES_DIR} — create it and add subfolders per person.")
        return

    encodings = {}
    for name in os.listdir(KNOWN_FACES_DIR):
        person_dir = os.path.join(KNOWN_FACES_DIR, name)
        if not os.path.isdir(person_dir):
            continue
        vectors = []
        for fname in os.listdir(person_dir):
            path = os.path.join(person_dir, fname)
            print(f"Processing {path}")
            img = face_recognition.load_image_file(path)
            faces = face_recognition.face_encodings(img)
            if faces:
                vectors.append(faces[0])
            else:
                print(f"  WARNING: no face found in {fname}")
        if vectors:
            encodings[name] = vectors
            print(f"  {name}: {len(vectors)} encodings")

    with open(ENCODINGS_PATH, 'wb') as f:
        pickle.dump(encodings, f)
    print(f"Saved {ENCODINGS_PATH}")


if __name__ == '__main__':
    main()
