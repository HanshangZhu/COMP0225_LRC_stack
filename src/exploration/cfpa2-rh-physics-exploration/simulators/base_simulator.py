from __future__ import annotations

from abc import ABC, abstractmethod


class BaseSimulator(ABC):
    @abstractmethod
    def run_episode(self, *args, **kwargs):
        raise NotImplementedError
