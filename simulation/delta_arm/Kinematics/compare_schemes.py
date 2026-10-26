"""
两种臂端关节方案的工作空间对比图

方案 A：臂端球头铰链 + 平台端球头铰链（全球铰）
方案 B：臂端十字万向节 + 平台端球头铰链（万向节双轴钝角）

用法：python compare_schemes.py [n]
输出：output/compare_schemes.html
"""

import sys
from pathlib import Path

import numpy as np

import workspace_analysis as wa

SCHEMES = [
    dict(key="ball45", name="全球铰 45°", joint="ball", cone=45.0, color="#2E86DE"),
    dict(
        key="uj110",
        name="万向节 110° + 球铰 45°",
        joint="ujoint",
        alpha=110.0,
        cone=45.0,
        color="#E67E22",
    ),
    dict(
        key="uj120",
        name="万向节 120° + 球铰 45°",
        joint="ujoint",
        alpha=120.0,
        cone=45.0,
        color="#16A085",
    ),
]


def run_scheme(scheme, n):
    """按方案设置全局参数并扫描工作空间"""
    wa.ARM_JOINT = scheme["joint"]
    wa.MAX_BALL_CONE = np.deg2rad(scheme["cone"])
    if scheme["joint"] == "ujoint":
        wa.UJ_ALPHA_MAX = np.deg2rad(scheme["alpha"])
        wa.UJ_BETA_MAX = np.deg2rad(90.0)
    _, P, _, counts, total = wa.build_workspace(n=n)
    return P, counts, total


def metrics(P):
    if len(P) == 0:
        return dict(n=0, zmin=np.nan, zmax=np.nan, stroke=np.nan, rmax=np.nan)
    z = P[:, 2]
    rad = np.hypot(P[:, 0], P[:, 1])
    return dict(
        n=len(P), zmin=z.min(), zmax=z.max(), stroke=z.max() - z.min(), rmax=rad.max()
    )


def build_html(results, n):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(
        rows=1,
        cols=2,
        specs=[[{"type": "scatter3d"}, {"type": "xy"}]],
        subplot_titles=("3D 工作空间点云对比", "Z 行程 – XY 半径包络对比"),
        column_widths=[0.62, 0.38],
    )

    for name, P, color in results:
        if len(P) == 0:
            continue
        rad = np.hypot(P[:, 0], P[:, 1])
        fig.add_trace(
            go.Scatter3d(
                x=P[:, 0],
                y=P[:, 1],
                z=P[:, 2],
                mode="markers",
                marker=dict(size=1.0, color=color, opacity=0.18),
                name=name,
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=rad,
                y=P[:, 2],
                mode="markers",
                marker=dict(size=1.2, color=color, opacity=0.18),
                name=name,
                showlegend=False,
            ),
            row=1,
            col=2,
        )

    # 指标表（放在 2D 子图左上角）
    lines = ["<b>方案对比（n=%d）</b>" % n]
    for name, P, _ in results:
        m = metrics(P)
        lines.append(
            "%s<br>　点数 %d　Z %+.1f~%+.1f<br>　行程 %.1f mm　半径 %.1f mm"
            % (name, m["n"], m["zmin"], m["zmax"], m["stroke"], m["rmax"])
            if m["n"]
            else "%s<br>　无有效点" % name
        )
    fig.add_annotation(
        text="<br>".join(lines),
        xref="x2 domain",
        yref="y2 domain",
        x=0.02,
        y=0.98,
        showarrow=False,
        align="left",
        font=dict(size=11),
        bgcolor="rgba(255,255,255,0.82)",
        bordercolor="#999",
        borderwidth=1,
    )

    fig.update_layout(
        title="Delta 机械臂：臂端关节方案对比（平行四边形从动臂，平台端球头铰链）",
        margin=dict(l=0, r=0, b=0, t=60),
        legend=dict(x=0.01, y=0.60),
    )
    fig.update_scenes(
        aspectmode="data",
        xaxis_title="X (mm)",
        yaxis_title="Y (mm)",
        zaxis_title="Z (mm)",
    )
    fig.update_xaxes(title_text="XY 半径 (mm)", row=1, col=2)
    fig.update_yaxes(title_text="Z (mm)", row=1, col=2)

    out_dir = Path(__file__).resolve().parent / "output"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / "compare_schemes.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    return out


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 101
    results = []
    print("Delta 工作空间方案对比（n=%d）" % n)
    print("%-24s %8s %9s %9s %8s %8s" % ("方案", "点数", "Zmin", "Zmax", "行程", "半径"))
    for scheme in SCHEMES:
        P, counts, total = run_scheme(scheme, n)
        m = metrics(P)
        print(
            "%-24s %8d %9.1f %9.1f %8.1f %8.1f"
            % (scheme["name"], m["n"], m["zmin"], m["zmax"], m["stroke"], m["rmax"])
        )
        results.append((scheme["name"], P, scheme["color"]))

    out = build_html(results, n)
    print("\n对比图已输出：%s" % out)


if __name__ == "__main__":
    main()
