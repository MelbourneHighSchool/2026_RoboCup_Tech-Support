# This module allows two bots on the same network to communicate with each other over UDP broadcast.
# This communication is symmetric, which means that both bots can run the same code, and both send and receive messages to each other.

import copy
import json
import secrets
import socket
import threading
import time

DEFAULT_PORT = 5005
DEFAULT_PEER_TIMEOUT = 0.5 # seconds, how long to wait for a message from a peer before considering it timed out (damaged)
DEFAULT_SOCKET_TIMEOUT = 0.05 # seconds, udp_socket timeout
BROADCAST_ADDR = "255.255.255.255"


class Peer:
    """Symmetric UDP broadcast peer for bot-to-bot state sharing."""

    def __init__(self, port: int = DEFAULT_PORT, peer_timeout_s: float = DEFAULT_PEER_TIMEOUT):
        self.port = port
        self.peer_timeout_s = peer_timeout_s
        self.bot_id = secrets.token_hex(8) # Random 8-byte hex string. Used to stop the bot from receiving messages from itself.

        self._socket: socket.socket | None = None
        self._running = False
        self._receive_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last_message: dict | None = None
        self._last_received_at: float | None = None
        self._receive_count = 0

    def start(self) -> None:
        # Don't start if already running
        if self._running:
            return

        # Create a UDP socket
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_socket.settimeout(DEFAULT_SOCKET_TIMEOUT)
        udp_socket.bind(("0.0.0.0", self.port))
        self._socket = udp_socket

        # Start the receive thread
        self._running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def stop(self) -> None:
        # Stop the receive thread
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=0.5)
            self._receive_thread = None

        # Close the socket
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        # Clear the last message and received at timestamp
        with self._lock:
            self._last_message = None
            self._last_received_at = None

        self._running = False

    def send(self, payload: dict) -> None:
        # Don't send if the socket is not open
        if self._socket is None:
            return

        # Create a packet
        packet = dict(payload)
        packet["bot_id"] = self.bot_id
        try:
            # Send the packet to the broadcast address and port
            self._socket.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                (BROADCAST_ADDR, self.port),
            )
        except OSError:
            pass

    def receive(self) -> dict | None:
        # Returns the last received message
        with self._lock:
            if self._last_message is None or self._last_received_at is None:
                # No message received yet
                return None
            if time.monotonic() - self._last_received_at > self.peer_timeout_s:
                # Message received but timed out
                return None
            return self._last_message

    def receive_snapshot(self) -> dict | None:
        """Atomically copy a message with its local monotonic receipt time."""
        with self._lock:
            if self._last_message is None or self._last_received_at is None:
                return None
            if time.monotonic() - self._last_received_at > self.peer_timeout_s:
                return None
            return {"message": copy.deepcopy(self._last_message),
                    "received_at": self._last_received_at}

    def _receive_loop(self) -> None:
        # Receive loop
        while self._running:
            if self._socket is None:
                # Socket is not open yet
                time.sleep(DEFAULT_SOCKET_TIMEOUT)
                continue

            # Try to receive a message
            try:
                packet, _addr = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                if self._running:
                    time.sleep(DEFAULT_SOCKET_TIMEOUT)
                continue

            # Try to parse the message as JSON
            try:
                message = json.loads(packet.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            # Ignore if the message is not a dictionary
            if not isinstance(message, dict):
                continue
            # Ignore if the message is from the same bot
            if message.get("bot_id") == self.bot_id:
                continue

            # If all checks pass, update the last message and received at timestamp, to be fetched by calling receive()
            with self._lock:
                self._last_message = message
                self._last_received_at = time.monotonic()
                self._receive_count += 1

    @property
    def receive_count(self) -> int:
        # Returns the number of messages received
        with self._lock:
            return self._receive_count
