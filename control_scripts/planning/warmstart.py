"""Warm-start cache for trajectory optimization.

Trajopt benefits significantly from a good initial guess — a B-spline that
was feasible for a similar problem is usually close enough to feasible in
the new problem that the NLP converges in a handful of iterations instead
of hundreds.

Cache key = (rounded q_start, rounded q_goal, problem_signature).
``problem_signature`` is any Hashable the caller assembles to differentiate
distinct optimisation problems — typically a tuple of a content-derived
scene fingerprint, constraint params (orientation lock, z lock, collision
flags...), and intermediate-waypoint joints. Same signature ↔ effectively
the same NLP, so the prior solution is a reasonable warm-start.

Rounding groups near-duplicate queries together at ~0.06° per joint.

Persistence
-----------
On import, the cache is loaded from ``logs/warmstart_cache.pkl`` (relative
to the project root) if present. On normal Python exit, an ``atexit``
handler flushes the in-memory cache back to disk. Crashes / SIGKILL skip
the flush — call ``flush()`` manually to checkpoint mid-run.

Sharing across machines: commit the pkl to git, sync via Dropbox, etc.
The signature uses a deterministic scene fingerprint (not ``id(plant)``)
so cache entries hit on machines that build the same scene with the same
Drake version. Cross-version Drake updates may invalidate entries — they
quietly miss (no false hits possible) until rebuilt.
"""

import atexit
import pickle
from collections import OrderedDict
from pathlib import Path
from typing import Hashable, Optional, Tuple

import numpy as np


_MAX_ENTRIES = 256
_CACHE: "OrderedDict[Tuple, np.ndarray]" = OrderedDict()
_HITS = 0
_MISSES = 0

# Cache lives next to this file, in control_scripts/planning/, so it's
# colocated with the code that owns it. Override via
# ``warmstart.set_cache_path(...)`` before first store/flush.
_CACHE_PATH = Path(__file__).resolve().parent / "warmstart_cache.pkl"


def _make_key(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    problem_signature: Hashable,
) -> Tuple:
    return (
        tuple(np.round(q_start, 3)),
        tuple(np.round(q_goal, 3)),
        problem_signature,
    )


def lookup(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    problem_signature: Hashable,
) -> Optional[np.ndarray]:
    """Return cached control points for a matching problem, or None."""
    global _HITS, _MISSES
    key = _make_key(q_start, q_goal, problem_signature)
    if key in _CACHE:
        _CACHE.move_to_end(key)  # LRU: mark as recently used
        _HITS += 1
        return _CACHE[key]
    _MISSES += 1
    return None


def store(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    problem_signature: Hashable,
    control_points: np.ndarray,
) -> None:
    """Cache a successful solution. Evicts the oldest entry when full."""
    key = _make_key(q_start, q_goal, problem_signature)
    _CACHE[key] = np.asarray(control_points, dtype=float)
    _CACHE.move_to_end(key)
    if len(_CACHE) > _MAX_ENTRIES:
        _CACHE.popitem(last=False)


def clear() -> None:
    """Wipe the cache. Useful across unrelated tasks or during debugging."""
    global _HITS, _MISSES
    _CACHE.clear()
    _HITS = 0
    _MISSES = 0


def stats() -> dict:
    """Current hit/miss counts and cache size. Cheap to call."""
    return {"size": len(_CACHE), "hits": _HITS, "misses": _MISSES}


def cache_path() -> Path:
    """Where the cache is loaded from / flushed to."""
    return _CACHE_PATH


def set_cache_path(path) -> None:
    """Override the disk cache path. Useful for shared/synced locations."""
    global _CACHE_PATH
    _CACHE_PATH = Path(path)


def load(path: Optional[Path] = None) -> int:
    """Merge entries from a pickle on disk into the in-memory cache.

    Returns the number of entries loaded. Silently no-ops on missing
    or unreadable files (corrupted pickle, schema drift, ...).
    """
    p = Path(path) if path is not None else _CACHE_PATH
    if not p.exists():
        return 0
    try:
        with p.open("rb") as f:
            disk = pickle.load(f)
        if not isinstance(disk, OrderedDict):
            return 0
        for key, value in disk.items():
            _CACHE[key] = np.asarray(value, dtype=float)
            _CACHE.move_to_end(key)
        while len(_CACHE) > _MAX_ENTRIES:
            _CACHE.popitem(last=False)
        return len(disk)
    except Exception as exc:
        print(f"[warmstart] failed to load {p}: {exc}; starting fresh")
        return 0


def flush(path: Optional[Path] = None) -> None:
    """Write the current cache to disk. Auto-called on Python exit."""
    p = Path(path) if path is not None else _CACHE_PATH
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix(p.suffix + ".tmp")
        with tmp.open("wb") as f:
            pickle.dump(_CACHE, f, protocol=pickle.HIGHEST_PROTOCOL)
        tmp.replace(p)  # atomic on POSIX; survives interrupts mid-write
    except Exception as exc:
        print(f"[warmstart] failed to flush to {p}: {exc}")


# Auto-load on import; auto-flush on exit. Process-local, but the file
# on disk is the shared persistent state.
_loaded = load()
if _loaded:
    print(f"[warmstart] loaded {_loaded} cached entries from {_CACHE_PATH}")
atexit.register(flush)


if __name__ == "__main__":
    # Inspection / maintenance CLI:
    #   python3.11 -m control_scripts.planning.warmstart --stats
    #   python3.11 -m control_scripts.planning.warmstart --clear
    #   python3.11 -m control_scripts.planning.warmstart --path
    import argparse

    # Suppress the auto-flush atexit; we manage disk explicitly here so
    # ``--stats`` doesn't accidentally rewrite the file.
    atexit.unregister(flush)

    ap = argparse.ArgumentParser(description="warmstart cache maintenance")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--stats", action="store_true",
                   help="Print cache size, hits, misses, and disk path.")
    g.add_argument("--clear", action="store_true",
                   help="Wipe in-memory cache AND delete the on-disk pkl.")
    g.add_argument("--path", action="store_true",
                   help="Print the cache file path and exit.")
    args = ap.parse_args()

    if args.path:
        print(_CACHE_PATH)
    elif args.stats:
        s = stats()
        on_disk = _CACHE_PATH.exists()
        print(f"path:    {_CACHE_PATH}")
        print(f"on-disk: {'yes' if on_disk else 'no'}"
              f"{f' ({_CACHE_PATH.stat().st_size} bytes)' if on_disk else ''}")
        print(f"in-mem:  size={s['size']}  hits={s['hits']}  misses={s['misses']}")
    elif args.clear:
        clear()
        try:
            _CACHE_PATH.unlink(missing_ok=True)
            print(f"[warmstart] cleared in-memory cache and deleted {_CACHE_PATH}")
        except Exception as exc:
            print(f"[warmstart] cleared in-memory but failed to delete "
                  f"{_CACHE_PATH}: {exc}")
