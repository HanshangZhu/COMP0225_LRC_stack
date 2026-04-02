from __future__ import annotations

from abc import ABC, abstractmethod

from core.types import PredictorInput, PredictorOutput


class BasePredictor(ABC):
    name: str = "base"

    @abstractmethod
    def predict(self, pred_input: PredictorInput) -> PredictorOutput:
        raise NotImplementedError
