"""Run on both bots with: python -m tests.communication."""

import argparse
import json
import time

from lib.communication import DEFAULT_PEER_TIMEOUT, DEFAULT_PORT, Peer


def main() -> None:
    parser = argparse.ArgumentParser(description="Send and receive UDP test messages between bots.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between sends")
    parser.add_argument("--peer-timeout", type=float, default=DEFAULT_PEER_TIMEOUT)
    parser.add_argument("--message", default="Communication test")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not 0 < args.interval < float("inf"):
        parser.error("--interval must be finite and positive")
    if not 0 < args.peer_timeout < float("inf"):
        parser.error("--peer-timeout must be finite and positive")

    peer = Peer(port=args.port, peer_timeout_s=args.peer_timeout)
    sequence = 0
    received_count = 0
    connected = False
    try:
        peer.start()
        print(f"Bot {peer.bot_id}, UDP port {args.port}. Run this on the other bot too.", flush=True)
        print("Waiting for peer messages; press Ctrl+C to stop.", flush=True)
        while True:
            peer.send({"message": args.message, "sequence": sequence})
            sequence += 1
            count = peer.receive_count
            message = peer.receive()
            if message is not None:
                if count != received_count:
                    print(f"Received: {json.dumps(message, sort_keys=True)}", flush=True)
                    received_count = count
                connected = True
            elif connected:
                print("Peer timed out; waiting for messages.", flush=True)
                connected = False
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping communication test.", flush=True)
    finally:
        peer.stop()


if __name__ == "__main__":
    main()
