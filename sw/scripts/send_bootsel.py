#!/usr/bin/env python3
"""Send the PicoCart64 BOOTSEL magic sequence over USB CDC (one-shot write)."""

import argparse
import sys

BOOTSEL_MAGIC = b"PC64BOOTSEL!!!!!!!!!!!"


def send_magic(port: str, repeat: int) -> None:
    # Works on Linux/macOS when the device is exposed as a tty (for example /dev/ttyACM0).
    with open(port, "wb", buffering=0) as f:
        for _ in range(repeat):
            written = f.write(BOOTSEL_MAGIC)
            f.flush()
            if written != len(BOOTSEL_MAGIC):
                raise RuntimeError(f"short write: wrote {written} of {len(BOOTSEL_MAGIC)} bytes")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send PicoCart64 BOOTSEL magic sequence")
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0",
                        help="Serial device path (default: /dev/ttyACM0)")
    parser.add_argument("--repeat", type=int, default=1, help="Number of times to send the magic sequence")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.repeat < 1:
        print("--repeat must be >= 1", file=sys.stderr)
        return 2

    try:
        send_magic(args.port, args.repeat)
        print(f"Sent {len(BOOTSEL_MAGIC)}-byte BOOTSEL magic x{args.repeat} to {args.port}")
        return 0
    except Exception as exc:  # pragma: no cover
        print(f"Failed to send BOOTSEL magic: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
