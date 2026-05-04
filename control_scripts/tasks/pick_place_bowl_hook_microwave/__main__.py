"""CLI wrapper for the pick_place_bowl_hook_microwave task package."""

from __future__ import annotations

import argparse

from .pick_place_bowl_hook_microwave import main


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dry",
        action="store_true",
        help="Plan and print poses without connecting to RTDE.",
    )
    args = parser.parse_args()
    raise SystemExit(main(dry=args.dry))
