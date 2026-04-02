from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from .types import RobotState


class MapAdapter(ABC):
    """Adapter hook for future Gazebo/real map backends."""

    @abstractmethod
    def get_shared_occupancy(self) -> Any:
        raise NotImplementedError


class StateAdapter(ABC):
    """Adapter hook for future Unitree Go2W state ingestion."""

    @abstractmethod
    def get_robot_states(self) -> list[RobotState]:
        raise NotImplementedError


class CommandAdapter(ABC):
    """Adapter hook for future command output integration."""

    @abstractmethod
    def apply_assignments(self, assignments: dict[int, Any]) -> None:
        raise NotImplementedError
