"""不启动 ROS 的快速验证命令行。"""

from __future__ import annotations

import argparse
import json
import math

import numpy as np

from .config import load_geometry
from .constraints import C0Reachability, PoseContext
from .geometry import DeltaGeometry
from .kinematics import (
    elbow_points,
    forward_kinematics,
    inverse_kinematics,
    platform_attachment_points,
)
from .pipeline import ConstraintPipeline


def _geometry(args: argparse.Namespace) -> DeltaGeometry:
    return load_geometry(args.config) if args.config else DeltaGeometry()


def _ik_payload(P: np.ndarray, geom: DeltaGeometry) -> dict:
    result = inverse_kinematics(P, geom)
    B, _ = elbow_points(result.theta, geom)
    A = platform_attachment_points(P, geom)
    b_hat = (A - B) / geom.l
    ctx = PoseContext(
        P=P,
        theta=result.theta,
        B=B,
        A=A,
        b_hat=b_hat,
        ratio=result.ratio,
    )
    decision = ConstraintPipeline(
        [C0Reachability(geom)], mode="enforce"
    ).evaluate(ctx)
    return {
        "P_m": [float(x) for x in P],
        "theta_deg": [float(x) for x in np.degrees(result.theta)],
        "reachable": bool(np.all(result.ok)),
        "constraint_accepted": decision.accepted,
        "constraints": [
            {
                "name": item.name,
                "ok": item.ok,
                "margin": item.margin,
                "code": item.code,
            }
            for item in decision.results
        ],
    }


def _cmd_ik(args: argparse.Namespace) -> None:
    geom = _geometry(args)
    print(json.dumps(_ik_payload(np.array([args.x, args.y, args.z]), geom), indent=2))


def _cmd_fk(args: argparse.Namespace) -> None:
    geom = _geometry(args)
    theta = np.radians(np.array(args.theta, dtype=float))
    result = forward_kinematics(theta, geom)
    print(
        json.dumps(
            {
                "theta_deg": list(args.theta),
                "P_mm": [float(x * 1000.0) for x in result.P],
                "ok": bool(np.all(result.ok)),
                "branch": result.branch,
            },
            indent=2,
        )
    )


def _cmd_check(args: argparse.Namespace) -> None:
    geom = _geometry(args)
    print(json.dumps(_ik_payload(np.array([args.x, args.y, args.z]), geom), indent=2))


def _cmd_workspace(args: argparse.Namespace) -> None:
    geom = _geometry(args)
    grid = np.linspace(-math.pi, math.pi, args.n)
    th1, th2, th3 = np.meshgrid(grid, grid, grid, indexing="ij")
    thetas = np.stack([th1.ravel(), th2.ravel(), th3.ravel()], axis=-1)
    result = forward_kinematics(thetas, geom)
    ok = np.asarray(result.ok) & (result.P[..., 2] < 0.0)
    points = result.P[ok]
    np.savetxt(args.out, points, delimiter=",", header="x,y,z", comments="")
    print(
        json.dumps(
            {
                "samples": int(thetas.shape[0]),
                "reachable_below_base": int(points.shape[0]),
                "out": args.out,
            },
            indent=2,
        )
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Delta 运动学快速验证")
    parser.add_argument("--config", help="geometry.yaml 路径", default="")
    sub = parser.add_subparsers(dest="command", required=True)

    p_ik = sub.add_parser("ik", help="逆解")
    p_ik.add_argument("--x", type=float, required=True)
    p_ik.add_argument("--y", type=float, required=True)
    p_ik.add_argument("--z", type=float, required=True)
    p_ik.set_defaults(func=_cmd_ik)

    p_fk = sub.add_parser("fk", help="正解")
    p_fk.add_argument("--theta", type=float, nargs=3, required=True)
    p_fk.set_defaults(func=_cmd_fk)

    p_check = sub.add_parser("check", help="逆解 + C0 检查")
    p_check.add_argument("--x", type=float, required=True)
    p_check.add_argument("--y", type=float, required=True)
    p_check.add_argument("--z", type=float, required=True)
    p_check.set_defaults(func=_cmd_check)

    p_ws = sub.add_parser("workspace", help="关节空间采样工作空间")
    p_ws.add_argument("--n", type=int, default=21)
    p_ws.add_argument("--out", default="/tmp/delta_workspace.csv")
    p_ws.set_defaults(func=_cmd_workspace)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
