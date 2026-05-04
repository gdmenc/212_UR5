"""CLI wrapper for the pour_bottle task package."""

from __future__ import annotations

import argparse

from .pour_bottle import main


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the pick/pour/place poses without connecting to RTDE.",
    )
    args = parser.parse_args()
    raise SystemExit(main(dry=args.dry))
