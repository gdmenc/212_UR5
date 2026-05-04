"""CLI wrapper for the pick_place_cup task package."""

from __future__ import annotations

import argparse

from .pick_place_cup import main


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print the grasp/place poses without connecting to RTDE.",
    )
    args = parser.parse_args()
    raise SystemExit(main(dry=args.dry))
