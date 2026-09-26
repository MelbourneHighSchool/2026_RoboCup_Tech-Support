"""Measure bot-to-bot UDP round-trip latency; run the same command on both bots."""

import argparse
import statistics
import time

from lib.communication import DEFAULT_PEER_TIMEOUT, DEFAULT_PORT, Peer


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Measure round-trip latency over bot-to-bot UDP broadcast."
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--interval", type=float, default=0.25, help="Seconds between probes")
    parser.add_argument("--count", type=int, default=20, help="Number of probes to send")
    parser.add_argument("--timeout", type=float, default=2.0, help="Seconds to await each reply")
    parser.add_argument("--peer-timeout", type=float, default=DEFAULT_PEER_TIMEOUT)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.interval <= 0 or args.timeout <= 0 or args.peer_timeout <= 0:
        parser.error("interval and timeouts must be positive")
    if args.count <= 0:
        parser.error("--count must be positive")

    peer = Peer(port=args.port, peer_timeout_s=args.peer_timeout)
    sent: dict[int, float] = {}
    completed: set[int] = set()
    handled: set[tuple[str, str, int]] = set()
    rtts_ms: list[float] = []
    next_send = time.monotonic()
    next_sequence = 0
    last_receive_count = 0
    deadline = None

    try:
        peer.start()
        print(
            f"Bot {peer.bot_id}, UDP port {args.port}. Run this script on the other bot too.",
            flush=True,
        )
        while len(completed) < args.count:
            now = time.monotonic()
            if next_sequence < args.count and now >= next_send:
                sent[next_sequence] = now
                peer.send({"latency": "ping", "sequence": next_sequence})
                print(f"Sent probe {next_sequence + 1}/{args.count}", flush=True)
                next_sequence += 1
                next_send = now + args.interval

            count = peer.receive_count
            if count != last_receive_count:
                message = peer.receive()
                last_receive_count = count
                if message is not None:
                    kind = message.get("latency")
                    source = message.get("bot_id", "")
                    sequence = message.get("sequence")
                    if kind in {"ping", "pong"} and isinstance(sequence, int):
                        key = (source, kind, sequence)
                        if key not in handled:
                            handled.add(key)
                            if kind == "ping":
                                peer.send({
                                    "latency": "pong",
                                    "sequence": sequence,
                                    "probe_bot": source,
                                })
                            elif message.get("probe_bot") == peer.bot_id and sequence in sent:
                                rtt_ms = (now - sent[sequence]) * 1000
                                completed.add(sequence)
                                rtts_ms.append(rtt_ms)
                                print(f"Probe {sequence}: {rtt_ms:.2f} ms", flush=True)

            if next_sequence == args.count and deadline is None:
                deadline = time.monotonic() + args.timeout
            if deadline is not None and time.monotonic() >= deadline:
                break
            time.sleep(0.001)

        lost = args.count - len(completed)
        if rtts_ms:
            print(
                f"\nResults: {len(rtts_ms)}/{args.count} replies, {lost} lost; "
                f"min {min(rtts_ms):.2f} ms, median {statistics.median(rtts_ms):.2f} ms, "
                f"mean {statistics.mean(rtts_ms):.2f} ms, max {max(rtts_ms):.2f} ms",
                flush=True,
            )
        else:
            print(f"\nNo replies received ({lost} lost).", flush=True)
    except KeyboardInterrupt:
        print("\nLatency measurement interrupted.", flush=True)
    finally:
        peer.stop()


if __name__ == "__main__":
    main()
