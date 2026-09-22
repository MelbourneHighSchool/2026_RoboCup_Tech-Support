"""Represent controller-owned state in comma-separated logs without extra fields."""

import json
from dataclasses import asdict, is_dataclass
from urllib.parse import quote, unquote


def encode_state(state):
    """Encode a dataclass or JSON-compatible controller state as one CSV token."""
    if is_dataclass(state):
        state = asdict(state)
    return quote(json.dumps(state, separators=(",", ":")), safe="")


def decode_state(token):
    """Decode current state tokens and legacy boolean steering fields for display."""
    try:
        return json.loads(unquote(token))
    except (ValueError, TypeError):
        return {"True": True, "False": False}.get(token)
