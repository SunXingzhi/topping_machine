"""约束流水线。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .constraints import Constraint, ConstraintResult, PoseContext

VALID_MODES = ("off", "shadow", "enforce")


@dataclass(frozen=True)
class Decision:
    accepted: bool
    all_ok: bool
    results: list[ConstraintResult]


class ConstraintPipeline:
    def __init__(self, constraints: Iterable[Constraint], mode: str = "shadow") -> None:
        if mode not in VALID_MODES:
            raise ValueError(f"mode 必须是 {VALID_MODES} 之一")
        self.constraints = list(constraints)
        self.mode = mode

    def evaluate(self, ctx: PoseContext) -> Decision:
        if self.mode == "off":
            return Decision(accepted=True, all_ok=True, results=[])

        results = [
            constraint.evaluate(ctx)
            for constraint in self.constraints
            if constraint.enabled
        ]
        all_ok = all(result.ok for result in results)
        accepted = all_ok if self.mode == "enforce" else True
        return Decision(accepted=accepted, all_ok=all_ok, results=results)
