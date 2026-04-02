from __future__ import annotations


class ArtifactManager:
    """Tracks ground-truth artifact positions and discovery state.

    Artifacts are a semantic overlay — they are NOT written into the occupancy
    grid, so coverage math, pathfinding, and frontier detection are unaffected.
    The operator (and the animation) can see all artifact positions; the robot
    only 'knows' about an artifact once its sensor sweep reveals that cell.
    """

    def __init__(self, positions: list[tuple[int, int]]) -> None:
        self._positions: set[tuple[int, int]] = {(int(x), int(y)) for x, y in positions}
        self._found: set[tuple[int, int]] = set()

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def check_discovery(self, revealed_cells: set[tuple[int, int]]) -> list[tuple[int, int]]:
        """Return newly-discovered artifact positions given the set of cells
        revealed by the robot's sensor this step."""
        newly = revealed_cells & self._positions - self._found
        self._found.update(newly)
        return list(newly)

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def all_found(self) -> bool:
        return self._positions <= self._found

    @property
    def found_count(self) -> int:
        return len(self._found)

    @property
    def total_count(self) -> int:
        return len(self._positions)

    @property
    def positions(self) -> list[tuple[int, int]]:
        """All artifact positions (ground truth — operator view)."""
        return list(self._positions)

    @property
    def found_positions(self) -> list[tuple[int, int]]:
        return list(self._found)

    @property
    def undiscovered_positions(self) -> list[tuple[int, int]]:
        return list(self._positions - self._found)
