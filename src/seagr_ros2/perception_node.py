#!/usr/bin/env python3
"""
perception_node.py — SEAGR Phase 1

Webcam face detection + identity recognition (face_recognition/dlib embeddings)
+ basic emotion heuristic. Publishes UserDetection.

Setup:
  pip install face_recognition opencv-python

  Put training photos here (5-10 clear, front-facing per person):
    seagr_ros2/known_faces/Ranveer/*.jpg

  Run enroll_faces.py once to build encodings.pkl. If encodings.pkl doesn't
  exist yet, this node will build it itself on first launch (slower startup).
"""

import os
import pickle
import cv2
import face_recognition
import rclpy
from rclpy.node import Node
from seagr_ros2_msgs.msg import UserDetection  # adjust if your package import path differs

KNOWN_FACES_DIR = os.path.join(os.path.dirname(__file__), 'known_faces')
ENCODINGS_PATH = os.path.join(os.path.dirname(__file__), 'encodings.pkl')
RECOGNITION_TOLERANCE = 0.5  # lower = stricter match, 0.6 is face_recognition default


def load_or_build_encodings():
    if os.path.exists(ENCODINGS_PATH):
        with open(ENCODINGS_PATH, 'rb') as f:
            return pickle.load(f)

    encodings = {}  # name -> list of 128-d vectors
    if not os.path.isdir(KNOWN_FACES_DIR):
        return encodings

    for name in os.listdir(KNOWN_FACES_DIR):
        person_dir = os.path.join(KNOWN_FACES_DIR, name)
        if not os.path.isdir(person_dir):
            continue
        vectors = []
        for fname in os.listdir(person_dir):
            path = os.path.join(person_dir, fname)
            img = face_recognition.load_image_file(path)
            faces = face_recognition.face_encodings(img)
            if faces:
                vectors.append(faces[0])
        if vectors:
            encodings[name] = vectors

    with open(ENCODINGS_PATH, 'wb') as f:
        pickle.dump(encodings, f)
    return encodings


def estimate_emotion(face_landmarks, face_width):
    """
    Rough heuristic from mouth/eyebrow landmark geometry, normalized by
    face_width so it works regardless of how far you are from the camera
    or how much the detection frame is downscaled. Swap for a real model
    later — these thresholds are hand-tuned starting points, not measured.
    """
    if not face_landmarks or not face_width:
        return "neutral", {}

    try:
        top_lip = face_landmarks.get('top_lip', [])
        bottom_lip = face_landmarks.get('bottom_lip', [])
        left_eyebrow = face_landmarks.get('left_eyebrow', [])
        left_eye = face_landmarks.get('left_eye', [])

        if not (top_lip and bottom_lip and len(top_lip) >= 7):
            return "neutral", {}

        # vertical mouth opening (top-lip center to bottom-lip center)
        mouth_open = abs(top_lip[3][1] - bottom_lip[3][1]) / face_width
        # mouth width
        mouth_width = abs(top_lip[6][0] - top_lip[0][0]) / face_width
        # corner_raise > 0 means corners are higher (smiling) than mouth center
        center_top_y = top_lip[3][1]
        corner_avg_y = (top_lip[0][1] + top_lip[6][1]) / 2
        corner_raise = (center_top_y - corner_avg_y) / face_width

        brow_eye_dist = 0.0
        if left_eyebrow and left_eye:
            brow_eye_dist = abs(left_eyebrow[2][1] - left_eye[1][1]) / face_width

        metrics = {
            "mouth_open": round(mouth_open, 3),
            "mouth_width": round(mouth_width, 3),
            "corner_raise": round(corner_raise, 3),
            "brow_eye_dist": round(brow_eye_dist, 3),
        }

        if corner_raise > 0.015 and mouth_width > 0.28:
            return "happy", metrics
        if mouth_open > 0.16 and brow_eye_dist > 0.14:
            return "surprised", metrics
        if corner_raise < -0.01:
            return "sad", metrics
        if brow_eye_dist < 0.045:
            return "angry", metrics
        return "neutral", metrics
    except (IndexError, KeyError, ZeroDivisionError):
        return "neutral", {}


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.publisher = self.create_publisher(UserDetection, 'user_detection', 10)
        self.encodings = load_or_build_encodings()
        self.get_logger().info(f"Loaded identities: {list(self.encodings.keys())}")

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
        self.timer = self.create_timer(0.2, self.process_frame)  # ~5 Hz

    def process_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        small = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)

        locations = face_recognition.face_locations(rgb_small)
        if not locations:
            cv2.imshow('SEAGR - perception', frame)
            cv2.waitKey(1)
            return

        encodings = face_recognition.face_encodings(rgb_small, locations)
        landmarks_list = face_recognition.face_landmarks(rgb_small, locations)

        for (top, right, bottom, left), enc, landmarks in zip(locations, encodings, landmarks_list):
            badge_id, confidence = self.match_identity(enc)
            face_width = right - left  # in the small-frame's own coordinate space
            emotion, metrics = estimate_emotion(landmarks, face_width)

            msg = UserDetection()
            msg.badge_id = badge_id
            msg.confidence = float(confidence)
            msg.emotion = emotion
            self.publisher.publish(msg)
            self.get_logger().info(
                f"Detected: {badge_id} ({confidence:.2f}) emotion={emotion} {metrics}")

            # scale face box back up to full-res frame (we detected on a 0.25x resize)
            top, right, bottom, left = top * 4, right * 4, bottom * 4, left * 4
            box_color = (0, 200, 0) if badge_id != "unknown" else (0, 0, 200)
            cv2.rectangle(frame, (left, top), (right, bottom), box_color, 2)
            label = f"{badge_id} ({confidence:.2f}) - {emotion}"
            cv2.putText(frame, label, (left, top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        cv2.imshow('SEAGR - perception', frame)
        cv2.waitKey(1)

    def match_identity(self, encoding):
        best_name, best_dist = "unknown", 1.0
        for name, vectors in self.encodings.items():
            distances = face_recognition.face_distance(vectors, encoding)
            if len(distances) == 0:
                continue
            min_dist = min(distances)
            if min_dist < best_dist:
                best_dist = min_dist
                best_name = name

        if best_dist <= RECOGNITION_TOLERANCE:
            return best_name, 1.0 - best_dist
        return "unknown", 1.0 - best_dist

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()