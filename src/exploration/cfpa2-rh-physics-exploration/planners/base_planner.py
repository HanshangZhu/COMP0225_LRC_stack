from __future__ import annotations

from abc import ABC, abstractmethod

from core.types import PlannerInput, PlannerOutput


class BasePlanner(ABC):
    name: str = "base"

    @abstractmethod
    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        raise NotImplementedError
