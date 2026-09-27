"""
greeting_logic.py — decides which greeting to perform and at what base
amplitude, given who was detected and their emotion.

Lives inside seagr_ros2/ so decision_node.py can just do a normal package
import instead of relying on an external filesystem path.
"""

# Which greeting each known person gets. Extend as you add more identities.
GREETING_BY_PERSON = {
    "Ranveer": "namaste",
}
DEFAULT_GREETING = "wave"

BASE_AMPLITUDE = 1.0


def greeting_logic(badge_id: str, emotion: str):
    """
    Returns (greeting_type: str, amplitude: float).
    Emotion-based amplitude scaling happens in decision_node.py, so this
    just returns a flat base amplitude — keep it simple here, one place
    owns the emotion modulation.
    """
    greeting_type = GREETING_BY_PERSON.get(badge_id, DEFAULT_GREETING)
    return greeting_type, BASE_AMPLITUDE
