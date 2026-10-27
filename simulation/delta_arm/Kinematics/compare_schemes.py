"""
多种臂端关节方案的工作空间对比图

方案定义与参数全部来自 workspace_analysis.SCHEMES，保证与
`python workspace_analysis.py <n> <方案键>` 的结果完全一致。

用法：python compare_schemes.py [n] [方案键 ...]
输出：output/compare_schemes.html
"""

import sys
from pathlib import Path

import numpy as np

import workspace_analysis as wa

DEFAULT_KEYS = ["ball45", "uj110", "uj160"]

# 作业区高度上限：高于此高度的位姿动平台会穿过静平台平面，属于不可用分支（上块），剔除以保证对比口径一致
Z_WORK_MAX = -20.0


def run_scheme(key, n):
    """按方案预设设置参数并扫描工作空间，只保留可作业区"""
    wa.apply_scheme(key)
    _, P, _, counts, total = wa.build_workspace(n=n)
    return P[P[:, 2] <= Z_WORK_MAX], counts, total


def metrics(P):
    if len(P) == 0:
        return dict(n=0, zmin=np.nan, zmax=np.nan, stroke=np.nan, rmax=np.nan)
    z = P[:, 2]
    rad = np.hypot(P[:, 0], P[:, 1])
    return dict(
        n=len(P), zmin=z.min(), zmax=z.max(), stroke=z.max() - z.min(), rmax=rad.max()
    )


def radius_vs_stroke(P, band=5.0, rhos=None):
    """每个半径圆环上还能容纳的 Z 行程 -> (所需行程, 可用最大半径) 曲线点"""
    if rhos is None:
        rhos = np.arange(0.0, 130.0, 5.0)
    z = P[:, 2]
    rad = np.hypot(P[:, 0], P[:, 1])
    pts = []
    for rho in rhos:
        m = np.abs(rad - rho) <= band
        if np.any(m):
            pts.append((z[m].max() - z[m].min(), rho))
    return pts


def build_html(results, n):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(
        rows=1,
        cols=3,
        specs=[[{"type": "scatter3d"}, {"type": "xy"}, {"type": "xy"}]],
        subplot_titles=(
            "3D 工作空间点云",
            "Z 行程 – XY 半径包络",
            "所需行程 → 可用半径",
        ),
        column_widths=[0.44, 0.31, 0.25],
    )

    # 点数多的先画、点数少的后画，避免小方案被大方案盖住
    ordered = sorted(results, key=lambda r: -len(r[1]))

    for label, P, color in ordered:
        if len(P) == 0:
            continue
        rad = np.hypot(P[:, 0], P[:, 1])
        fig.add_trace(
            go.Scatter3d(
                x=P[:, 0],
                y=P[:, 1],
                z=P[:, 2],
                mode="markers",
                marker=dict(size=1.6, color=color, opacity=0.55),
                name=label,
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=rad,
                y=P[:, 2],
                mode="markers",
                marker=dict(size=2.0, color=color, opacity=0.5),
                name=label,
                showlegend=False,
            ),
            row=1,
            col=2,
        )

        curve = radius_vs_stroke(P)
        curve.sort()
        fig.add_trace(
            go.Scatter(
                x=[c[0] for c in curve],
                y=[c[1] for c in curve],
                mode="lines+markers",
                line=dict(color=color, width=4),
                marker=dict(size=5, color=color),
                name=label,
                showlegend=False,
            ),
            row=1,
            col=3,
        )

    lines = [
        "<b>方案对比（n=%d，平台球窝外倾 45°）</b>" % n,
        "<i>已剔除 z > %.0f mm 的不可用上块</i>" % Z_WORK_MAX,
    ]
    for label, P, _ in results:
        m = metrics(P)
        if m["n"]:
            lines.append(
                "%s<br>　点数 %d　Z %+.1f~%+.1f<br>　行程 %.1f mm　半径 %.1f mm"
                % (label, m["n"], m["zmin"], m["zmax"], m["stroke"], m["rmax"])
            )
        else:
            lines.append("%s<br>　无有效点" % label)
    fig.add_annotation(
        text="<br>".join(lines),
        xref="x2 domain",
        yref="y2 domain",
        x=0.02,
        y=0.98,
        showarrow=False,
        align="left",
        font=dict(size=11),
        bgcolor="rgba(255,255,255,0.85)",
        bordercolor="#999",
        borderwidth=1,
    )

    fig.update_layout(
        title="Delta 机械臂：臂端关节方案对比（平行四边形从动臂，平台端球头铰链）",
        margin=dict(l=0, r=0, b=0, t=60),
        legend=dict(x=0.005, y=0.62, bgcolor="rgba(255,255,255,0.85)"),
    )
    fig.update_scenes(
        aspectmode="data",
        xaxis_title="X (mm)",
        yaxis_title="Y (mm)",
        zaxis_title="Z (mm)",
    )
    fig.update_xaxes(title_text="XY 半径 (mm)", row=1, col=2)
    fig.update_yaxes(title_text="Z (mm)", row=1, col=2)
    fig.update_xaxes(title_text="所需 Z 行程 (mm)", row=1, col=3)
    fig.update_yaxes(title_text="可用最大半径 (mm)", row=1, col=3)

    out_dir = Path(__file__).resolve().parent / "output"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / "compare_schemes.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    return out


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 101
    keys = sys.argv[2:] or DEFAULT_KEYS

    print("Delta 工作空间方案对比（n=%d）" % n)
    print("%-24s %8s %9s %9s %8s %8s" % ("方案", "点数", "Zmin", "Zmax", "行程", "半径"))
    results = []
    for key in keys:
        if key not in wa.SCHEMES:
            print("未知方案键：%s（可选：%s）" % (key, ", ".join(wa.SCHEMES)))
            continue
        scheme = wa.SCHEMES[key]
        P, _, _ = run_scheme(key, n)
        m = metrics(P)
        print(
            "%-24s %8d %9.1f %9.1f %8.1f %8.1f"
            % (scheme["label"], m["n"], m["zmin"], m["zmax"], m["stroke"], m["rmax"])
        )
        results.append((scheme["label"], P, scheme["color"]))

    if not results:
        return
    out = build_html(results, n)
    print("\n对比图已输出：%s" % out)


if __name__ == "__main__":
    main()
