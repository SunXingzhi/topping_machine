"""约束接口定义。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class PoseContext:
    """一次位姿检查需要的全部中间量。"""

    P: np.ndarray
    theta: np.ndarray
    B: np.ndarray
    A: np.ndarray
    b_hat: np.ndarray
    ratio: np.ndarray | None = None
    prev_theta: np.ndarray | None = None


@dataclass(frozen=True)
class ConstraintResult:
    name: str
    ok: bool
    margin: float
    code: str
    detail: dict = field(default_factory=dict)


class Constraint(Protocol):
    name: str
    enabled: bool

    def evaluate(self, ctx: PoseContext) -> ConstraintResult:
        ...
