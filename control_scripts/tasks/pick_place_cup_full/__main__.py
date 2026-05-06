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
    parser.add_argument(
        "--mode",
        choices=["real", "sim"],
        default="real",
        help=(
            "Execution mode. 'real' (default) runs on the rig via RTDE. "
            "'sim' plans all transits and visualises them in Drake/Meshcat "
            "without executing on hardware."
        ),
    )
    parser.add_argument(
        "--no-motion-planning",
        action="store_true",
        help=(
            "Disable Drake motion planning entirely. All transits route "
            "through sequential moveL Cartesian primitives — the pre-"
            "planner flow."
        ),
    )
    args = parser.parse_args()
    raise SystemExit(main(
        dry=args.dry,
        mode=args.mode,
        motion_planning=not args.no_motion_planning,
    ))
