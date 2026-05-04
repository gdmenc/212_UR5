"""Redirect — runner has moved to control_scripts.tasks.close_microwave.

    python3.11 -m control_scripts.tasks.close_microwave [--dry]
"""
from ..tasks.close_microwave.__main__ import main

if __name__ == "__main__":
    main()
