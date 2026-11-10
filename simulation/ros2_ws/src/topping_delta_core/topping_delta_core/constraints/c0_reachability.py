"""C0：几何可达性。"""

from __future__ import annotations

import numpy as np

from ..geometry import DeltaGeometry
from ..kinematics import reachable
from .base import ConstraintResult, PoseContext


class C0Reachability:
    name = "c0_reachability"

    def __init__(self, geom: DeltaGeometry, enabled: bool = True) -> None:
        self.geom = geom
        self.enabled = enabled

    def evaluate(self, ctx: PoseContext) -> ConstraintResult:
        ok = bool(np.all(reachable(ctx.P, self.geom)))
        ratio_max = (
            float(np.max(np.abs(ctx.ratio)))
            if ctx.ratio is not None
            else (0.0 if ok else 2.0)
        )
        margin = max(0.0, min(1.0, 1.0 - ratio_max)) if ok else 0.0
        return ConstraintResult(
            name=self.name,
            ok=ok,
            margin=margin,
            code="C0_REACHABLE" if ok else "C0_UNREACHABLE",
            detail={
                "P": [float(x) for x in np.asarray(ctx.P).reshape(3)],
                "ratio_max": ratio_max,
            },
        )
