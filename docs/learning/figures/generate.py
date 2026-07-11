"""Generate all figures for docs/learning/.

Run:  ../../../../../tmp/claude/learning-venv/bin/python generate.py

Saves PNGs into the current directory.
Style: clean matplotlib, large fonts, light grid where useful.
"""

from __future__ import annotations
import os
import shutil
import subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyArrow, FancyArrowPatch, Rectangle, Polygon
from matplotlib.lines import Line2D
from matplotlib import patches as mpatches

plt.rcParams.update({
    "font.size": 12,
    "axes.titlesize": 14,
    "axes.labelsize": 12,
    "figure.dpi": 110,
    "savefig.bbox": "tight",
    "savefig.facecolor": "white",
})

HERE = os.path.dirname(os.path.abspath(__file__))


def save(fig, name: str) -> None:
    path = os.path.join(HERE, name)
    fig.savefig(path)
    plt.close(fig)
    print(f"  → {name}")


# Graphviz-rendered figures. matplotlib cannot reproduce a DOT graph, so these
# PNGs are rendered from their checked-in `.dot` source via graphviz `dot`. They
# are wired here so `generate.py` is the single "reproduce every PNG" entry
# point — no hand-editing, per the never-edit-the-PNG rule. Requires the
# `graphviz` package (the `dot` binary). If `dot` is missing, we warn and skip
# rather than fail the matplotlib figures.
DOT_FIGURES = [
    "22-tracker-stack-alternatives.dot",
]


def render_dot_figures() -> None:
    dot = shutil.which("dot")
    if dot is None:
        print("  ! graphviz `dot` not found — skipping .dot figures "
              "(install graphviz to render): " + ", ".join(DOT_FIGURES))
        return
    for src in DOT_FIGURES:
        src_path = os.path.join(HERE, src)
        out_path = os.path.join(HERE, os.path.splitext(src)[0] + ".png")
        subprocess.run([dot, "-Tpng", src_path, "-o", out_path], check=True)
        print(f"  → {os.path.basename(out_path)} (graphviz)")


# ──────────────────────────────────────────────────────────────────────────────
# 02 probability
# ──────────────────────────────────────────────────────────────────────────────

def fig_gaussian_1d():
    mu, sigma = 0.0, 1.0
    x = np.linspace(-4, 4, 600)
    y = np.exp(-0.5 * ((x - mu) / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi))

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(x, y, color="#1f3a5f", linewidth=2.5)

    bands = [(-3, 3, "#dceaf7", "99.7% (3σ)"),
             (-2, 2, "#9cc3e8", "95% (2σ)"),
             (-1, 1, "#4d8fd1", "68% (1σ)")]
    for lo, hi, color, label in bands:
        mask = (x >= lo) & (x <= hi)
        ax.fill_between(x[mask], 0, y[mask], color=color, label=label, alpha=0.95)

    for k in range(-3, 4):
        ax.axvline(k, color="#888", linewidth=0.6, linestyle=":", zorder=0)
    ax.set_xticks([-3, -2, -1, 0, 1, 2, 3])
    ax.set_xticklabels([r"$\mu-3\sigma$", r"$\mu-2\sigma$", r"$\mu-\sigma$",
                        r"$\mu$", r"$\mu+\sigma$", r"$\mu+2\sigma$", r"$\mu+3\sigma$"])
    ax.set_ylabel(r"$p(x)$")
    ax.set_title("1-D Gaussian with σ confidence bands")
    ax.set_ylim(0, y.max() * 1.15)
    ax.legend(loc="upper right", framealpha=0.95)
    ax.grid(False)
    save(fig, "02-gaussian-1d.png")


def _draw_ellipse(ax, cov, mu=(0, 0), color="#1f3a5f", n_sigma=(1, 2)):
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    for k in n_sigma:
        w, h = 2 * k * np.sqrt(eigvals)
        e = Ellipse(mu, w, h, angle=angle, edgecolor=color, facecolor="none", linewidth=2.0)
        ax.add_patch(e)
    ax.plot(*mu, marker="+", markersize=14, color=color, markeredgewidth=2)


def fig_cov_ellipses():
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    titles_and_cov = [
        ("Round\n(uncorrelated, equal var)", np.array([[1.0, 0.0], [0.0, 1.0]])),
        ("Long & thin\n(bearing-only, range unknown)", np.array([[5.0, 0.0], [0.0, 0.25]])),
        ("Tilted\n(positive correlation)", np.array([[2.0, 1.4], [1.4, 1.2]])),
    ]
    for ax, (title, cov) in zip(axes, titles_and_cov):
        _draw_ellipse(ax, cov, mu=(0, 0))
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        ax.set_aspect("equal")
        ax.set_xlabel(r"$p_x$")
        ax.set_ylabel(r"$p_y$")
        ax.set_title(title)
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.axhline(0, color="#999", linewidth=0.5)
        ax.axvline(0, color="#999", linewidth=0.5)
    fig.suptitle("Covariance shapes (1σ inner, 2σ outer)", y=1.02)
    save(fig, "02-cov-ellipses.png")


def fig_mixture_vs_collapse():
    x = np.linspace(-6, 6, 600)
    mu_a, mu_b = -1.8, 1.8
    s_a = s_b = 0.8
    w_a = w_b = 0.5
    pa = w_a * np.exp(-0.5 * ((x - mu_a) / s_a) ** 2) / (s_a * np.sqrt(2 * np.pi))
    pb = w_b * np.exp(-0.5 * ((x - mu_b) / s_b) ** 2) / (s_b * np.sqrt(2 * np.pi))
    mixture = pa + pb

    mu_c = w_a * mu_a + w_b * mu_b
    var_c = w_a * (s_a ** 2 + (mu_a - mu_c) ** 2) + w_b * (s_b ** 2 + (mu_b - mu_c) ** 2)
    s_c = np.sqrt(var_c)
    pc = np.exp(-0.5 * ((x - mu_c) / s_c) ** 2) / (s_c * np.sqrt(2 * np.pi))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)

    ax = axes[0]
    ax.fill_between(x, 0, mixture, color="#dceaf7", alpha=0.9)
    ax.plot(x, mixture, color="#1f3a5f", linewidth=2.4, label="true mixture")
    ax.axvline(mu_a, color="#aa3333", linestyle="--", linewidth=1, label=r"$\mu_a, \mu_b$")
    ax.axvline(mu_b, color="#aa3333", linestyle="--", linewidth=1)
    ax.set_title("True 50/50 mixture (two humps)")
    ax.set_xlabel(r"$x$")
    ax.set_ylabel(r"$p(x)$")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)

    ax = axes[1]
    ax.fill_between(x, 0, pc, color="#f7dcdc", alpha=0.9)
    ax.plot(x, pc, color="#aa3333", linewidth=2.4, label="moment-matched Gaussian")
    ax.plot(x, mixture, color="#1f3a5f", linewidth=1.5, linestyle=":", label="(original mixture)")
    ax.axvline(mu_c, color="#aa3333", linestyle="--", linewidth=1)
    ax.set_title("Collapsed single Gaussian (wider, no humps)")
    ax.set_xlabel(r"$x$")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)

    save(fig, "02-mixture-vs-collapse.png")


# ──────────────────────────────────────────────────────────────────────────────
# 04 KF
# ──────────────────────────────────────────────────────────────────────────────

def fig_kf_predict_grow():
    fig, ax = plt.subplots(figsize=(9, 5))
    P0 = np.array([[0.6, 0.0], [0.0, 0.6]])
    Q_each = np.array([[1.2, 0.0], [0.0, 0.6]])
    centres = [(0, 0), (3, 0.3), (6, 0.7), (9, 1.0)]
    colors = ["#1f3a5f", "#3d6da8", "#5a98c8", "#85bbdb"]
    labels = ["posterior", "predict 1", "predict 2", "predict 3"]
    P = P0
    for i, (c, col, lab) in enumerate(zip(centres, colors, labels)):
        if i > 0:
            P = P + Q_each
        _draw_ellipse(ax, P, mu=c, color=col, n_sigma=(2,))
        ax.text(c[0], c[1] + 0.3 + 0.7 * np.sqrt(P[1, 1]), lab,
                ha="center", color=col, fontsize=11)
    ax.annotate("", xy=(9.5, 0.3), xytext=(-1.0, -0.2),
                arrowprops=dict(arrowstyle="->", color="#888", linewidth=1.2))
    ax.text(4.5, -1.6, "every predict shifts (along velocity) and grows (adds Q)",
            ha="center", color="#666", style="italic")
    ax.set_xlim(-2, 12)
    ax.set_ylim(-3, 4.5)
    ax.set_aspect("equal")
    ax.set_xlabel(r"$p_x$ (m)")
    ax.set_ylabel(r"$p_y$ (m)")
    ax.set_title("KF predict: ellipse moves AND grows over time")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "04-kf-predict-grow.png")


def fig_kf_predict_update_cycle():
    fig, ax = plt.subplots(figsize=(11, 5))
    truth = np.linspace(0, 10, 60)
    truth_y = 0.05 * truth
    ax.plot(truth, truth_y, color="#888", linestyle=":", linewidth=1.2, label="true trajectory")

    P_post = np.array([[0.3, 0.0], [0.0, 0.3]])
    Q = np.array([[0.45, 0.0], [0.0, 0.2]])
    R = 0.18
    cx = 0.5
    cy = 0.0
    measurements = [(2.0, 0.18), (4.0, 0.05), (6.5, 0.3), (9.0, 0.35)]
    seq_label_done = {"predict": False, "update": False, "measurement": False}

    for i, (zx, zy) in enumerate(measurements):
        # predict: ellipse at predicted position
        cx_pred = zx - 0.4
        cy_pred = cy + 0.5 * (zy - cy)
        P_pred = P_post + Q
        _draw_ellipse(ax, P_pred, mu=(cx_pred, cy_pred), color="#85bbdb", n_sigma=(2,))
        if not seq_label_done["predict"]:
            ax.text(cx_pred, cy_pred + np.sqrt(P_pred[1, 1]) * 2 + 0.15, "predict",
                    ha="center", fontsize=10, color="#3a6da8")
            seq_label_done["predict"] = True

        # measurement
        ax.plot(zx, zy, marker="o", markersize=11, color="#aa3333", zorder=10,
                label="measurement z" if not seq_label_done["measurement"] else None)
        seq_label_done["measurement"] = True

        # posterior (shrunk, moved toward z)
        # simple KF update on x and y separately for visual
        K = P_pred[0, 0] / (P_pred[0, 0] + R)
        cx_post = cx_pred + K * (zx - cx_pred)
        Ky = P_pred[1, 1] / (P_pred[1, 1] + R)
        cy_post = cy_pred + Ky * (zy - cy_pred)
        P_post = np.array([[(1 - K) * P_pred[0, 0], 0], [0, (1 - Ky) * P_pred[1, 1]]])
        _draw_ellipse(ax, P_post, mu=(cx_post, cy_post), color="#1f3a5f", n_sigma=(2,))
        if not seq_label_done["update"]:
            ax.text(cx_post, cy_post - np.sqrt(P_post[1, 1]) * 2 - 0.35, "posterior",
                    ha="center", fontsize=10, color="#1f3a5f")
            seq_label_done["update"] = True
        cx, cy = cx_post, cy_post

    handles = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#aa3333",
               markersize=10, label="measurement z"),
        Line2D([0], [0], color="#85bbdb", linewidth=2, label="predict (grown)"),
        Line2D([0], [0], color="#1f3a5f", linewidth=2, label="posterior (shrunk to z)"),
        Line2D([0], [0], color="#888", linestyle=":", label="true trajectory"),
    ]
    ax.legend(handles=handles, loc="upper left", framealpha=0.95)
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1.5, 2)
    ax.set_aspect("equal")
    ax.set_xlabel(r"$p_x$ (m)")
    ax.set_ylabel(r"$p_y$ (m)")
    ax.set_title("Predict / update cycle: ellipse grows during predict, shrinks during update")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "04-predict-update-cycle.png")


def fig_kalman_gain_geometry():
    fig, ax = plt.subplots(figsize=(9, 5))
    prior = np.array([0.0, 0.0])
    z = np.array([6.0, 0.5])
    ax.plot(*prior, marker="s", markersize=14, color="#1f3a5f", label="prior mean")
    ax.plot(*z, marker="o", markersize=14, color="#aa3333", label="measurement z")
    ax.plot([prior[0], z[0]], [prior[1], z[1]], color="#999", linestyle="--", linewidth=1.4)

    Ks = [0.1, 0.5, 0.9]
    colors_K = ["#85bbdb", "#3a6da8", "#102e54"]
    for K, c in zip(Ks, colors_K):
        post = prior + K * (z - prior)
        ax.plot(*post, marker="o", markersize=12, color=c)
        ax.annotate(f"K = {K}", xy=post, xytext=(post[0], post[1] - 0.5 - K * 0.3),
                    ha="center", color=c, fontsize=11)

    ax.text(prior[0] - 0.3, prior[1] - 0.6, "prior", color="#1f3a5f", fontsize=11)
    ax.text(z[0] + 0.1, z[1] - 0.5, "measurement", color="#aa3333", fontsize=11)
    ax.set_xlim(-1.5, 8)
    ax.set_ylim(-2, 2.5)
    ax.set_aspect("equal")
    ax.set_xlabel(r"$p_x$")
    ax.set_ylabel(r"$p_y$")
    ax.set_title("Kalman gain K places the posterior on the prior→z line")
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper left")
    save(fig, "04-kalman-gain-geometry.png")


# ──────────────────────────────────────────────────────────────────────────────
# 05 EKF
# ──────────────────────────────────────────────────────────────────────────────

def fig_ekf_ruler():
    def h(x):
        return np.sin(x) + 0.4 * x

    def hp(x):
        return np.cos(x) + 0.4

    x = np.linspace(-3, 3, 500)
    y = h(x)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)

    # narrow prior, ruler matches
    ax = axes[0]
    ax.plot(x, y, color="#1f3a5f", linewidth=2.4, label="true $h(x)$")
    mu = 0.6
    s = 0.25
    tan = h(mu) + hp(mu) * (x - mu)
    ax.plot(x, tan, color="#aa3333", linewidth=1.6, linestyle="--", label="EKF ruler (Jacobian at $\\hat{x}$)")
    ax.axvspan(mu - 2 * s, mu + 2 * s, color="#9cc3e8", alpha=0.45, label="prior support (2σ)")
    ax.plot([mu], [h(mu)], "o", color="#1f3a5f", markersize=10)
    ax.set_title("Narrow prior — ruler hugs curve ⇒ EKF accurate")
    ax.legend()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_xlabel("x"); ax.set_ylabel("h(x)")

    # wide prior, ruler bad
    ax = axes[1]
    ax.plot(x, y, color="#1f3a5f", linewidth=2.4, label="true $h(x)$")
    mu = 0.6
    s = 1.4
    tan = h(mu) + hp(mu) * (x - mu)
    ax.plot(x, tan, color="#aa3333", linewidth=1.6, linestyle="--", label="EKF ruler (Jacobian at $\\hat{x}$)")
    ax.axvspan(mu - 2 * s, mu + 2 * s, color="#9cc3e8", alpha=0.45, label="prior support (2σ)")
    ax.plot([mu], [h(mu)], "o", color="#1f3a5f", markersize=10)
    ax.set_title("Wide prior — curve bends inside prior ⇒ EKF biased")
    ax.legend()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_xlabel("x")

    fig.suptitle("EKF linearisation: when the ruler approximation works (left) and fails (right)", y=1.02)
    save(fig, "05-ekf-ruler.png")


def fig_rangebearing_geometry():
    fig, ax = plt.subplots(figsize=(8, 7))
    sensor = (0, 0)
    target = (4, 3)

    ax.plot(*sensor, marker="^", markersize=20, color="#1f3a5f")
    ax.text(sensor[0] - 0.2, sensor[1] - 0.6, "sensor (sx, sy)", color="#1f3a5f")
    ax.plot(*target, marker="o", markersize=14, color="#aa3333")
    ax.text(target[0] + 0.2, target[1] + 0.2, "target (px, py)", color="#aa3333")

    # range line
    ax.plot([sensor[0], target[0]], [sensor[1], target[1]], color="#3a6da8", linewidth=2)
    ax.annotate(r"range $r = \sqrt{dx^2+dy^2}$",
                xy=(target[0] / 2 + 0.4, target[1] / 2 + 0.4),
                color="#3a6da8", fontsize=12, rotation=np.degrees(np.arctan2(target[1], target[0])))

    # dx, dy
    ax.plot([sensor[0], target[0]], [sensor[1], sensor[1]], color="#888", linewidth=1.2, linestyle=":")
    ax.plot([target[0], target[0]], [sensor[1], target[1]], color="#888", linewidth=1.2, linestyle=":")
    ax.text(target[0] / 2, -0.3, r"$dx = p_x - s_x$", color="#666", ha="center")
    ax.text(target[0] + 0.15, target[1] / 2, r"$dy = p_y - s_y$", color="#666")

    # bearing arc
    theta = np.linspace(0, np.arctan2(target[1], target[0]), 50)
    R = 1.1
    ax.plot(R * np.cos(theta), R * np.sin(theta), color="#2d8659", linewidth=1.8)
    ax.text(1.4, 0.45, r"$\beta = \mathrm{atan2}(dy, dx)$", color="#2d8659", fontsize=12)

    ax.axhline(0, color="#bbb", linewidth=0.7)
    ax.axvline(0, color="#bbb", linewidth=0.7)
    ax.set_xlim(-1, 6.5)
    ax.set_ylim(-1.5, 4.5)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Range–bearing geometry, sensor at (sx, sy), target at (px, py)")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "05-rangebearing-geometry.png")


# ──────────────────────────────────────────────────────────────────────────────
# 06 UKF
# ──────────────────────────────────────────────────────────────────────────────

def fig_sigma_points():
    fig, ax = plt.subplots(figsize=(7, 7))
    mu = np.array([0.0, 0.0])
    P = np.array([[2.0, 1.0], [1.0, 1.0]])
    n = 2
    alpha, beta, kappa = 1e-3, 2.0, 0.0
    scale = alpha ** 2 * (n + kappa)
    L = np.linalg.cholesky(scale * P)
    # use a larger scale for visibility
    L_show = np.linalg.cholesky(P)
    points = [mu]
    for i in range(n):
        points.append(mu + L_show[:, i])
        points.append(mu - L_show[:, i])

    _draw_ellipse(ax, P, mu=mu, color="#1f3a5f", n_sigma=(1, 2))
    for i, p in enumerate(points):
        ax.plot(*p, marker="o", markersize=14, color="#aa3333", zorder=5)
        ax.annotate(rf"$\chi_{i}$", xy=p, xytext=(p[0] + 0.18, p[1] + 0.18),
                    color="#aa3333", fontsize=13)
    ax.plot(*mu, marker="+", markersize=18, markeredgewidth=2, color="#1f3a5f")
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal")
    ax.set_title("UKF sigma points on a tilted covariance ellipse (n=2 ⇒ 5 points)")
    ax.set_xlabel(r"$x_1$"); ax.set_ylabel(r"$x_2$")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "06-sigma-points.png")


def fig_ukf_vs_ekf_propagation():
    def h(x):
        return np.sin(1.2 * x) + 0.3 * x

    def hp(x):
        return 1.2 * np.cos(1.2 * x) + 0.3

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    x = np.linspace(-3, 3, 400)
    y = h(x)
    mu = 0.4
    s = 1.0

    # left: function + sigma points
    ax = axes[0]
    ax.plot(x, y, color="#1f3a5f", linewidth=2.4, label="true $h(x)$")
    # tangent
    tan = h(mu) + hp(mu) * (x - mu)
    ax.plot(x, tan, color="#aa3333", linewidth=1.4, linestyle="--", label="EKF ruler at $\\hat{x}$")
    # sigma points 1D
    sigma_xs = [mu - s, mu, mu + s]
    sigma_ys = [h(p) for p in sigma_xs]
    ax.plot(sigma_xs, sigma_ys, "o", color="#2d8659", markersize=12, label="UKF sigma points")
    for sx, sy in zip(sigma_xs, sigma_ys):
        ax.plot([sx, sx], [0, sy], color="#2d8659", linestyle=":", alpha=0.5)
    ax.axvspan(mu - s, mu + s, color="#dceaf7", alpha=0.5, label="prior (1σ)")
    ax.set_xlim(-3, 3); ax.set_ylim(-2.5, 2.5)
    ax.set_xlabel("x"); ax.set_ylabel("h(x)")
    ax.legend(loc="lower right", fontsize=9)
    ax.set_title("Push sigma points through nonlinear h(x)")
    ax.grid(True, linestyle=":", alpha=0.4)

    # right: reconstructed output densities
    ax = axes[1]
    # Monte Carlo for reference
    rng = np.random.default_rng(0)
    samples = mu + s * rng.standard_normal(20000)
    true_h = h(samples)
    ax.hist(true_h, bins=80, density=True, color="#dceaf7", edgecolor="#9cc3e8",
            label="true output distribution (MC)")
    # EKF Gaussian
    mu_ekf = h(mu); s_ekf = abs(hp(mu)) * s
    xx = np.linspace(-3, 3, 400)
    ax.plot(xx, np.exp(-0.5 * ((xx - mu_ekf) / s_ekf) ** 2) / (s_ekf * np.sqrt(2 * np.pi)),
            color="#aa3333", linewidth=2, label="EKF approx (Gaussian via Jacobian)")
    # UKF Gaussian via sigma points
    w0 = 1 / 3; w1 = 1 / 3
    mu_ukf = w0 * sigma_ys[1] + w1 * (sigma_ys[0] + sigma_ys[2])
    var_ukf = w0 * (sigma_ys[1] - mu_ukf) ** 2 + w1 * ((sigma_ys[0] - mu_ukf) ** 2 + (sigma_ys[2] - mu_ukf) ** 2)
    s_ukf = np.sqrt(var_ukf)
    ax.plot(xx, np.exp(-0.5 * ((xx - mu_ukf) / s_ukf) ** 2) / (s_ukf * np.sqrt(2 * np.pi)),
            color="#2d8659", linewidth=2, label="UKF approx (Gaussian via sigma pts)")
    ax.set_xlim(-3, 3)
    ax.set_xlabel("h(x)")
    ax.set_ylabel("density")
    ax.set_title("UKF reconstructs a better Gaussian than EKF")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("EKF vs UKF: linearise vs sample", y=1.02)
    save(fig, "06-ukf-vs-ekf-propagation.png")


# ──────────────────────────────────────────────────────────────────────────────
# 07 PF
# ──────────────────────────────────────────────────────────────────────────────

def fig_pf_bearing_only_evolution():
    rng = np.random.default_rng(1)
    truth = np.array([2.5, 7.0])
    sensor_path = np.array([[0.0, 0.0], [0.0, 2.0], [0.0, 4.0], [0.0, 6.0]])
    sigma_b = np.radians(2.5)

    # initial particles
    N = 1500
    particles = rng.uniform(low=[-5, 0], high=[8, 12], size=(N, 2))
    weights = np.ones(N) / N

    fig, axes = plt.subplots(1, 4, figsize=(15, 5), sharey=True)

    for k, ax in enumerate(axes):
        s = sensor_path[k]
        true_b = np.arctan2(truth[1] - s[1], truth[0] - s[0])
        pred_b = np.arctan2(particles[:, 1] - s[1], particles[:, 0] - s[0])
        r = np.angle(np.exp(1j * (true_b - pred_b)))
        log_w = -0.5 * (r / sigma_b) ** 2
        weights = weights * np.exp(log_w - log_w.max())
        weights /= weights.sum()

        # alpha based on weight
        alpha = np.clip(weights / weights.max(), 0.02, 0.85)
        # color based on weight too
        ax.scatter(particles[:, 0], particles[:, 1], s=6, color="#1f3a5f", alpha=alpha)
        ax.plot(*s, marker="^", markersize=18, color="#2d8659")
        ax.plot(*truth, marker="*", markersize=22, color="#aa3333")

        # bearing line
        end = s + 14 * np.array([np.cos(true_b), np.sin(true_b)])
        ax.plot([s[0], end[0]], [s[1], end[1]], color="#888", linestyle=":", linewidth=1)

        ax.set_xlim(-5, 8); ax.set_ylim(0, 12)
        ax.set_aspect("equal")
        ax.set_title(f"After {k+1} bearing measurement{'s' if k else ''}")
        ax.set_xlabel("east (m)")
        if k == 0: ax.set_ylabel("north (m)")
        ax.grid(True, linestyle=":", alpha=0.4)

        # resample if N_eff drops
        N_eff = 1 / np.sum(weights ** 2)
        if N_eff < N / 2:
            idx = rng.choice(N, N, p=weights)
            particles = particles[idx] + 0.15 * rng.standard_normal((N, 2))
            weights = np.ones(N) / N

    handles = [
        Line2D([0], [0], marker="^", color="w", markerfacecolor="#2d8659", markersize=14, label="sensor"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="#aa3333", markersize=14, label="true target"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#1f3a5f", markersize=8, label="particles (α∝weight)"),
        Line2D([0], [0], color="#888", linestyle=":", label="bearing line"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.05))
    fig.suptitle("Particle filter: bearing-only posterior tightens as the moving sensor accumulates geometry", y=1.02)
    save(fig, "07-pf-bearing-only-evolution.png")


def fig_pf_degeneracy_resample():
    rng = np.random.default_rng(2)
    N = 20
    # before: skewed weights
    w_before = rng.exponential(1.0, N) ** 3
    w_before /= w_before.sum()
    # resample
    idx = rng.choice(N, N, p=w_before)
    unique, counts = np.unique(idx, return_counts=True)
    w_after = np.zeros(N)
    w_after[unique] = counts / N

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)

    ax = axes[0]
    ax.bar(np.arange(N), w_before, color="#1f3a5f")
    ax.set_title("Before resample: weight degeneracy")
    ax.set_xlabel("particle index"); ax.set_ylabel("weight")
    ax.set_ylim(0, 1)
    ax.grid(True, axis="y", linestyle=":", alpha=0.4)

    ax = axes[1]
    ax.bar(np.arange(N), w_after, color="#2d8659")
    ax.set_title(f"After systematic resample: equal weights $1/N$ (heavy particles cloned)")
    ax.set_xlabel("particle index")
    ax.set_ylim(0, 1)
    ax.grid(True, axis="y", linestyle=":", alpha=0.4)

    save(fig, "07-pf-degeneracy-resample.png")


# ──────────────────────────────────────────────────────────────────────────────
# 08 motion models
# ──────────────────────────────────────────────────────────────────────────────

def fig_cv_vs_ct_trajectory():
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True)

    # CV
    ax = axes[0]
    times = np.linspace(0, 8, 9)
    x0 = np.array([0, 0])
    v = np.array([1.0, 0.3])
    P = 0.1 * np.eye(2)
    Q = np.array([[0.12, 0.0], [0.0, 0.08]])
    for t in times:
        c = x0 + v * t
        _draw_ellipse(ax, P + Q * t, mu=c, color="#1f3a5f", n_sigma=(2,))
        ax.plot(*c, marker="o", color="#1f3a5f", markersize=4)
    ax.set_title("Constant Velocity: straight line + uncertainty grows linearly")
    ax.set_aspect("equal"); ax.grid(True, linestyle=":", alpha=0.4)
    ax.set_xlim(-1, 10); ax.set_ylim(-2.5, 5)
    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")

    # CT
    ax = axes[1]
    omega = 0.35
    times = np.linspace(0, 8, 9)
    cx0, cy0 = 0, 0
    v0 = 1.0
    P = 0.1 * np.eye(2)
    Q = np.array([[0.12, 0.0], [0.0, 0.08]])
    for t in times:
        # rotate around origin
        x = (v0 / omega) * np.sin(omega * t)
        y = (v0 / omega) * (1 - np.cos(omega * t))
        _draw_ellipse(ax, P + Q * t, mu=(x, y), color="#aa3333", n_sigma=(2,))
        ax.plot(x, y, marker="o", color="#aa3333", markersize=4)
    ax.set_title("Coordinated Turn (ω≈20°/s): circular arc, same uncertainty model")
    ax.set_aspect("equal"); ax.grid(True, linestyle=":", alpha=0.4)
    ax.set_xlim(-1, 10); ax.set_ylim(-2.5, 5)
    ax.set_xlabel("east (m)")

    save(fig, "08-cv-vs-ct-trajectory.png")


# ──────────────────────────────────────────────────────────────────────────────
# 09 IMM
# ──────────────────────────────────────────────────────────────────────────────

def fig_imm_mode_probs():
    t = np.linspace(0, 60, 600)
    # CV high until turn starts at 18; CT high during 22..38; CV resumes after 42
    def sigm(x): return 1.0 / (1.0 + np.exp(-x))
    cv = sigm((18 - t) * 0.6) + sigm((t - 42) * 0.6)
    ct = sigm((t - 20) * 0.5) * sigm((40 - t) * 0.5)
    ncv = np.full_like(t, 0.04) + 0.06 * sigm((t - 19) * 0.3) * sigm((42 - t) * 0.3)
    # normalise to sum 1
    s = cv + ct + ncv
    cv /= s; ct /= s; ncv /= s

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.stackplot(t, cv, ct, ncv,
                 labels=["μ_CV (Constant Velocity)", "μ_CT (Coordinated Turn)", "μ_nCV (noisy CV)"],
                 colors=["#1f3a5f", "#aa3333", "#cbb46a"], alpha=0.85)
    ax.axvline(20, color="#444", linestyle=":", linewidth=1)
    ax.axvline(40, color="#444", linestyle=":", linewidth=1)
    ax.text(20, 1.03, "vessel starts turn", ha="center", fontsize=11)
    ax.text(40, 1.03, "resumes straight cruise", ha="center", fontsize=11)
    ax.set_ylim(0, 1.08); ax.set_xlim(0, 60)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("mode probability $\\mu_k$")
    ax.set_title("IMM: mode probabilities are driven by the data, not by a flag")
    ax.legend(loc="center right")
    ax.grid(False)
    save(fig, "09-imm-mode-probs.png")


# ──────────────────────────────────────────────────────────────────────────────
# 11 gating
# ──────────────────────────────────────────────────────────────────────────────

def fig_gating_ellipse():
    fig, ax = plt.subplots(figsize=(8.5, 6))
    z_hat = np.array([0.0, 0.0])
    S = np.array([[2.0, 0.8], [0.8, 1.0]])
    _draw_ellipse(ax, S, mu=z_hat, color="#1f3a5f", n_sigma=(2.146,))  # 2.146² ≈ 4.605 = χ²_2 90% quantile
    ax.plot(*z_hat, marker="+", markersize=18, color="#1f3a5f", markeredgewidth=2)
    ax.text(z_hat[0] + 0.1, z_hat[1] + 0.15, r"$\hat{z}$", color="#1f3a5f", fontsize=14)

    measurements = [
        ((0.8, 0.4), "in", "z_1"),
        ((4.5, 2.5), "out", "z_2"),
        ((-3, -2.5), "out", "z_3"),
        ((-0.5, -0.7), "in", "z_4"),
    ]
    for (p, kind, name) in measurements:
        color = "#2d8659" if kind == "in" else "#aa3333"
        ax.plot(*p, marker="o", markersize=14, color=color, zorder=5)
        ax.annotate(name, xy=p, xytext=(p[0] + 0.2, p[1] + 0.2), color=color, fontsize=12)

    handles = [
        Line2D([0], [0], marker="+", color="w", markerfacecolor="#1f3a5f",
               markeredgecolor="#1f3a5f", markersize=14, label=r"predicted $\hat{z}$"),
        Line2D([0], [0], color="#1f3a5f", linewidth=2, label=r"gate boundary $d^2=\gamma$"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2d8659", markersize=10, label="in-gate"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#aa3333", markersize=10, label="out-of-gate"),
    ]
    ax.legend(handles=handles, loc="upper right")
    ax.set_xlim(-5, 6); ax.set_ylim(-4, 4)
    ax.set_aspect("equal")
    ax.set_xlabel("z dim 1"); ax.set_ylabel("z dim 2")
    ax.set_title("Mahalanobis gate around the predicted measurement")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "11-gating-ellipse.png")


# ──────────────────────────────────────────────────────────────────────────────
# 12 JPDA
# ──────────────────────────────────────────────────────────────────────────────

def fig_jpda_soft():
    fig, ax = plt.subplots(figsize=(9, 6))
    t1 = np.array([1.0, 1.0])
    t2 = np.array([4.0, 1.2])
    z = np.array([2.5, 2.4])

    # tracks
    ax.plot(*t1, marker="o", markersize=18, color="#1f3a5f")
    ax.plot(*t2, marker="o", markersize=18, color="#2d8659")
    ax.text(t1[0] - 0.7, t1[1] - 0.6, "track $t_1$", color="#1f3a5f", fontsize=12)
    ax.text(t2[0] + 0.2, t2[1] - 0.6, "track $t_2$", color="#2d8659", fontsize=12)
    # gates
    _draw_ellipse(ax, np.array([[1.4, 0.3], [0.3, 0.9]]), mu=t1, color="#1f3a5f", n_sigma=(2,))
    _draw_ellipse(ax, np.array([[1.4, -0.3], [-0.3, 0.9]]), mu=t2, color="#2d8659", n_sigma=(2,))
    # measurement
    ax.plot(*z, marker="o", markersize=18, color="#aa3333")
    ax.text(z[0] + 0.15, z[1] + 0.2, "measurement $z$", color="#aa3333", fontsize=12)

    # association arrows with beta values
    ax.annotate("", xy=z, xytext=t1,
                arrowprops=dict(arrowstyle="->", color="#1f3a5f", linewidth=2.4))
    ax.annotate("", xy=z, xytext=t2,
                arrowprops=dict(arrowstyle="->", color="#2d8659", linewidth=2.4))
    ax.text((t1[0] + z[0]) / 2 - 0.55, (t1[1] + z[1]) / 2 + 0.2,
            r"$\beta_{1z} = 0.55$", color="#1f3a5f", fontsize=12)
    ax.text((t2[0] + z[0]) / 2 + 0.05, (t2[1] + z[1]) / 2 + 0.2,
            r"$\beta_{2z} = 0.40$", color="#2d8659", fontsize=12)
    ax.text(z[0] - 0.2, z[1] + 0.6, r"$\beta_{0z} = 0.05$ (clutter share)", color="#aa3333", fontsize=11)

    ax.set_xlim(-1, 7); ax.set_ylim(-1, 4.5)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title("JPDA: one ambiguous measurement shared softly between two tracks")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "12-jpda-soft.png")


# ──────────────────────────────────────────────────────────────────────────────
# 13 clutter map
# ──────────────────────────────────────────────────────────────────────────────

def fig_clutter_map():
    rng = np.random.default_rng(3)
    n_bearing = 24
    n_range = 12
    grid = rng.exponential(0.4, (n_range, n_bearing))
    # coast at bearing 90..150
    grid[0:4, 6:11] += 6.0
    # wake behind own-ship (bearing 180)
    grid[1:3, 12:16] += 4.5
    # bird flock
    grid[6:8, 16:19] += 1.2
    grid = np.clip(grid, 0, 8)

    fig, ax = plt.subplots(figsize=(11, 5))
    im = ax.imshow(grid, origin="lower", aspect="auto", cmap="YlOrRd",
                   extent=[0, 360, 0, 8])
    ax.set_xlabel("bearing (deg)"); ax.set_ylabel("range (km)")
    ax.set_title("Adaptive clutter map: λ_C learned per (range, bearing) cell")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label(r"clutter density $\hat{\lambda}_C$ (relative)")
    ax.annotate("coastline\n(high λ_C)", xy=(120, 0.5), xytext=(150, 2.0),
                color="#222", arrowprops=dict(arrowstyle="->"))
    ax.annotate("own-ship wake", xy=(220, 1.0), xytext=(255, 3.0),
                color="#222", arrowprops=dict(arrowstyle="->"))
    ax.annotate("open water\n(low λ_C)", xy=(40, 6.0), xytext=(30, 6.5),
                color="#222")
    save(fig, "13-clutter-map.png")


def fig_compound_k_clutter():
    """Poisson vs compound-K radar clutter over a coverage annulus.

    Left: homogeneous Poisson — the flat clutter every tracker's λ_C term
    assumes. Right: compound-K (gamma texture × Poisson speckle) — spatially
    clumpy, heavy-tailed, the same expected count. A uniform-λ tracker
    over-counts on the right (spikes look like target births). This is why the
    sim battery's clutter scenario uses compound-K (anti-model-matched-optimism).
    """
    rng = np.random.default_rng(7)
    r_min, r_max = 0.3, 8.0  # km
    n_wedges = 24
    lam_mean = 60.0 / n_wedges  # expected clutter points per wedge

    def annulus(n, a_lo, a_hi):
        r = np.sqrt(rng.random(n) * (r_max**2 - r_min**2) + r_min**2)
        a = rng.uniform(a_lo, a_hi, n)
        return r * np.cos(a), r * np.sin(a)

    edges = np.linspace(0, 2 * np.pi, n_wedges + 1)
    # Poisson: uniform intensity per wedge.
    px, py = [], []
    for k in range(n_wedges):
        n = rng.poisson(lam_mean)
        x, y = annulus(n, edges[k], edges[k + 1]); px.append(x); py.append(y)
    px, py = np.concatenate(px), np.concatenate(py)
    # Compound-K: gamma texture (shape nu) modulates each wedge's rate.
    nu = 0.5
    tex = rng.gamma(nu, 1.0 / nu, n_wedges)
    kx, ky = [], []
    for k in range(n_wedges):
        n = rng.poisson(lam_mean * tex[k])
        x, y = annulus(n, edges[k], edges[k + 1]); kx.append(x); ky.append(y)
    kx, ky = np.concatenate(kx), np.concatenate(ky)

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    for ax, (xx, yy, title) in zip(axes, [
            (px, py, f"Homogeneous Poisson\n(n={len(px)}, uniform λ_C)"),
            (kx, ky, f"Compound-K (ν={nu})\n(n={len(kx)}, gamma-modulated λ_C — clumpy)")]):
        ax.scatter(xx, yy, s=10, c="#aa3333", alpha=0.7, edgecolors="none")
        ax.scatter([0], [0], marker="^", s=120, c="#1f3a5f", zorder=5)
        ax.annotate("own-ship", (0, 0), (0.3, 0.6), color="#1f3a5f", fontsize=9)
        circ = plt.Circle((0, 0), r_max, fill=False, ls="--", color="#999")
        ax.add_patch(circ)
        ax.set_xlim(-r_max * 1.1, r_max * 1.1); ax.set_ylim(-r_max * 1.1, r_max * 1.1)
        ax.set_aspect("equal"); ax.set_title(title)
        ax.set_xlabel("east (km)"); ax.set_ylabel("north (km)")
    fig.suptitle("Same expected clutter count, very different spatial structure",
                 fontsize=12)
    save(fig, "13-compound-k-clutter.png")


# ──────────────────────────────────────────────────────────────────────────────
# 14 MHT tree
# ──────────────────────────────────────────────────────────────────────────────

def fig_mht_tree():
    fig, ax = plt.subplots(figsize=(11, 6.5))

    nodes = {
        "root":   (0.5, 0.95, "root\n(birth z₁)\nscore = s₀"),
        "a":      (0.25, 0.65, "z₂ → t\nscore = s_a"),
        "b":      (0.75, 0.65, "z₂ → clutter\nscore = s_b"),
        "aa":     (0.10, 0.30, "z₃ → t\ns_aa"),
        "ab":     (0.35, 0.30, "z₃ → clutter\ns_ab"),
        "ba":     (0.62, 0.30, "z₃ → t\ns_ba"),
        "bb":     (0.90, 0.30, "z₃ → clutter\ns_bb"),
    }
    best = "aa"

    edges = [("root", "a"), ("root", "b"),
             ("a", "aa"), ("a", "ab"),
             ("b", "ba"), ("b", "bb")]
    for src, dst in edges:
        x0, y0, _ = nodes[src]; x1, y1, _ = nodes[dst]
        ax.plot([x0, x1], [y0, y1], color="#888", linewidth=1.5, zorder=1)

    for key, (x, y, label) in nodes.items():
        color = "#2d8659" if key == best else "#1f3a5f"
        size = 1600 if key == best else 1100
        ax.scatter(x, y, s=size, color=color, edgecolor="black", zorder=3)
        ax.text(x, y - 0.08, label, ha="center", fontsize=10, color="black")
    ax.text(0.10, 0.20, "↑ surviving leaf\n(best score)",
            ha="center", color="#2d8659", fontsize=10, style="italic")

    ax.set_xlim(0, 1); ax.set_ylim(0.1, 1.05)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title("MHT track tree — each leaf is a Gaussian + accumulated LLR score")
    for spine in ax.spines.values(): spine.set_visible(False)
    save(fig, "14-mht-tree.png")


# ──────────────────────────────────────────────────────────────────────────────
# 16 NEES / NIS
# ──────────────────────────────────────────────────────────────────────────────

def _chi2_pdf(x, k):
    from math import lgamma
    return np.where(x > 0,
                    np.exp(-x / 2) * x ** (k / 2 - 1) / (2 ** (k / 2) * np.exp(lgamma(k / 2))),
                    0.0)


def fig_nis_regimes():
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    rng = np.random.default_rng(4)
    k = 2

    # consistent
    samples1 = rng.chisquare(k, 800)
    axes[0].hist(samples1, bins=40, density=True, color="#9cc3e8", edgecolor="#3a6da8")
    axes[0].axvline(samples1.mean(), color="#aa3333", linewidth=2, label=f"sample mean = {samples1.mean():.2f}")
    axes[0].axvline(k, color="#1f3a5f", linewidth=2, linestyle="--", label=f"expected = m = {k}")
    axes[0].set_title("Consistent\n$\\bar{\\epsilon}_{NIS} \\approx m$")
    axes[0].legend(fontsize=9)
    axes[0].set_xlabel(r"$\epsilon_{NIS}$"); axes[0].set_ylabel("density")
    axes[0].set_xlim(0, 20)
    axes[0].grid(True, linestyle=":", alpha=0.4)

    # overconfident → samples scaled up
    samples2 = rng.chisquare(k, 800) * 2.5
    axes[1].hist(samples2, bins=40, density=True, color="#f7c2b7", edgecolor="#aa3333")
    axes[1].axvline(samples2.mean(), color="#aa3333", linewidth=2, label=f"sample mean = {samples2.mean():.2f}")
    axes[1].axvline(k, color="#1f3a5f", linewidth=2, linestyle="--", label=f"expected = m = {k}")
    axes[1].set_title("Overconfident filter\n$\\bar{\\epsilon}_{NIS} > m$ — R or Q too small")
    axes[1].legend(fontsize=9)
    axes[1].set_xlabel(r"$\epsilon_{NIS}$")
    axes[1].set_xlim(0, 20)
    axes[1].grid(True, linestyle=":", alpha=0.4)

    # underconfident → samples scaled down
    samples3 = rng.chisquare(k, 800) * 0.35
    axes[2].hist(samples3, bins=40, density=True, color="#cbe7c8", edgecolor="#2d8659")
    axes[2].axvline(samples3.mean(), color="#aa3333", linewidth=2, label=f"sample mean = {samples3.mean():.2f}")
    axes[2].axvline(k, color="#1f3a5f", linewidth=2, linestyle="--", label=f"expected = m = {k}")
    axes[2].set_title("Underconfident filter\n$\\bar{\\epsilon}_{NIS} < m$ — R too big")
    axes[2].legend(fontsize=9)
    axes[2].set_xlabel(r"$\epsilon_{NIS}$")
    axes[2].set_xlim(0, 20)
    axes[2].grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("Three NIS regimes — diagnose the filter from its own innovations", y=1.02)
    save(fig, "16-nis-regimes.png")


def fig_nis_timeseries():
    rng = np.random.default_rng(5)
    N = 600
    k = 2
    samples = rng.chisquare(k, N)
    # one outlier
    samples[200] = 9.0
    cumulative_mean = np.cumsum(samples) / np.arange(1, N + 1)

    fig, ax = plt.subplots(figsize=(12, 5))
    t = np.arange(N)
    ax.scatter(t, samples, s=10, color="#1f3a5f", alpha=0.45, label="per-update $\\epsilon_{NIS}$")
    ax.plot(t, cumulative_mean, color="#aa3333", linewidth=2, label="running mean")
    # chi-squared 95% bands for sample mean
    var = 2 * k
    n_arr = np.arange(1, N + 1)
    upper = k + 1.96 * np.sqrt(var / n_arr)
    lower = k - 1.96 * np.sqrt(var / n_arr)
    ax.fill_between(t, lower, upper, color="#9cc3e8", alpha=0.4, label="95% band for running mean")
    ax.axhline(k, color="#1f3a5f", linestyle="--", linewidth=1.5, label=f"expected = m = {k}")
    ax.set_xlabel("update number")
    ax.set_ylabel(r"$\epsilon_{NIS}$")
    ax.set_title("NIS over a run: a single big outlier is fine; the mean is what matters")
    ax.legend(loc="upper right")
    ax.set_ylim(0, 11)
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "16-nis-timeseries.png")


# ──────────────────────────────────────────────────────────────────────────────
# 18 CPA
# ──────────────────────────────────────────────────────────────────────────────

def fig_cpa_geometry():
    fig, ax = plt.subplots(figsize=(9, 6.5))
    t = np.linspace(0, 80, 200)

    own_p = np.array([0.0, 0.0])
    own_v = np.array([6.0, 0.0])
    tgt_p = np.array([350.0, 200.0])
    tgt_v = np.array([-4.0, -2.0])

    own_traj = own_p + np.outer(t, own_v)
    tgt_traj = tgt_p + np.outer(t, tgt_v)
    # CPA computation
    dp = tgt_p - own_p
    dv = tgt_v - own_v
    tcpa = -np.dot(dp, dv) / np.dot(dv, dv)
    cpa_own = own_p + own_v * tcpa
    cpa_tgt = tgt_p + tgt_v * tcpa
    cpa_dist = np.linalg.norm(cpa_own - cpa_tgt)

    ax.plot(own_traj[:, 0], own_traj[:, 1], color="#1f3a5f", linewidth=2.4, label="own-ship trajectory")
    ax.plot(tgt_traj[:, 0], tgt_traj[:, 1], color="#aa3333", linewidth=2.4, label="target trajectory")
    ax.plot(*own_p, marker="o", color="#1f3a5f", markersize=14)
    ax.plot(*tgt_p, marker="o", color="#aa3333", markersize=14)
    ax.plot(*cpa_own, marker="X", color="#1f3a5f", markersize=14)
    ax.plot(*cpa_tgt, marker="X", color="#aa3333", markersize=14)
    ax.plot([cpa_own[0], cpa_tgt[0]], [cpa_own[1], cpa_tgt[1]],
            color="#2d8659", linewidth=2.4, linestyle="--",
            label=f"CPA distance = {cpa_dist:.0f} m  (at TCPA = {tcpa:.0f} s)")

    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title("CPA: closest point of approach under straight-line extrapolation")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "18-cpa-geometry.png")


def fig_cpa_hysteresis():
    t = np.linspace(0, 120, 600)
    rng = np.random.default_rng(6)
    base = 0.5 + 0.45 * np.sin(0.07 * t - 1.0)
    base[200:380] += 0.15
    p = np.clip(base + 0.06 * rng.standard_normal(600), 0, 1)

    state = "OUT"
    states = []
    for v in p:
        if state == "OUT" and v > 0.7:
            state = "IN"
        elif state == "IN" and v < 0.3:
            state = "OUT"
        states.append(state)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.fill_between(t, 0, 1, where=np.array(states) == "IN",
                    color="#f7c2b7", alpha=0.5, label="INSIDE region (alarm)")
    ax.plot(t, p, color="#1f3a5f", linewidth=2, label=r"$p_{breach}$")
    ax.axhline(0.7, color="#aa3333", linestyle="--", linewidth=1.4, label="enter threshold = 0.7")
    ax.axhline(0.3, color="#2d8659", linestyle="--", linewidth=1.4, label="exit threshold = 0.3")
    # mark first up-crossing of 0.7 after OUT and first down-crossing of 0.3 from IN
    prev = "OUT"
    for ti, st in zip(t, states):
        if prev == "OUT" and st == "IN":
            ax.axvline(ti, color="#aa3333", linestyle=":", linewidth=1)
            ax.text(ti, 1.02, "Entered", ha="center", color="#aa3333", fontsize=10)
        elif prev == "IN" and st == "OUT":
            ax.axvline(ti, color="#2d8659", linestyle=":", linewidth=1)
            ax.text(ti, 1.02, "Exited", ha="center", color="#2d8659", fontsize=10)
        prev = st
    ax.set_xlim(0, 120); ax.set_ylim(0, 1.1)
    ax.set_xlabel("time (s)"); ax.set_ylabel(r"$p_{breach}$")
    ax.set_title("CPA hysteresis: separated enter / exit thresholds prevent event flapping")
    ax.legend(loc="lower right")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "18-cpa-hysteresis.png")


def fig_sensor_bias_convergence():
    """KF posterior shrinking toward a true 2-D bias as paired
    observations arrive. Shows the 1-sigma covariance ellipse and the
    current b_hat after n in {0, 1, 5, 20} observations."""
    rng = np.random.default_rng(42)
    true_b = np.array([3.0, -2.0])
    P = np.eye(2) * 25.0  # initial prior (5 m 1-sigma)
    R_obs = np.eye(2) * (1.0 * 1.0 + 2.0 * 2.0)  # AIS + sensor
    b_hat = np.array([0.0, 0.0])

    snapshots = {0: (b_hat.copy(), P.copy())}
    n_obs = 20
    for i in range(1, n_obs + 1):
        z = true_b + rng.multivariate_normal([0, 0], R_obs)
        S = P + R_obs
        K = P @ np.linalg.inv(S)
        b_hat = b_hat + K @ (z - b_hat)
        P = (np.eye(2) - K) @ P
        if i in (1, 5, 20):
            snapshots[i] = (b_hat.copy(), P.copy())

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(*true_b, marker="x", color="#aa3333", markersize=14,
            markeredgewidth=3, label="true bias")
    colors = ["#cccccc", "#a8b6c8", "#5278a3", "#1f3a5f"]
    for (n, (bh, Pn)), c in zip(snapshots.items(), colors):
        vals, vecs = np.linalg.eigh(Pn)
        order = np.argsort(vals)[::-1]
        vals, vecs = vals[order], vecs[:, order]
        ang = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
        w, h = 2 * np.sqrt(vals)
        ell = Ellipse(bh, w, h, angle=ang,
                              fill=False, edgecolor=c, linewidth=2,
                              label=f"n={n}")
        ax.add_patch(ell)
        ax.plot(*bh, marker="o", color=c, markersize=7)

    ax.set_xlim(-7, 9)
    ax.set_ylim(-9, 7)
    ax.set_aspect("equal")
    ax.set_xlabel("bias_x (m)")
    ax.set_ylabel("bias_y (m)")
    ax.set_title(
        "Per-sensor position-bias KF: posterior shrinks toward true b\n"
        "1-sigma covariance ellipse vs number of paired observations")
    ax.axhline(0, color="#bbbbbb", linewidth=0.7)
    ax.axvline(0, color="#bbbbbb", linewidth=0.7)
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper left")
    save(fig, "21-sensor-bias-convergence.png")


def fig_seeing_the_tracker():
    """Plan view (left) and NIS time series (right) that together explain
    how to read a navtracker Foxglove session.

    Left panel:
      - A track position (navy filled circle) with a small P covariance
        ellipse (navy, 2σ).
      - A larger gate ellipse derived from S = H P H' + R (light blue).
      - One in-gate detection (green) connected to the track by an
        association line.
      - One out-of-gate detection (red).
      - A bearing wedge (green dashed lines) from a sensor in the
        lower-left corner.

    Right panel:
      - Per-update NIS scatter + running mean + 95 % chi-squared band,
        mirroring the explanation in chapter 16.
    """
    import numpy as np
    from matplotlib.patches import Ellipse
    from matplotlib.lines import Line2D

    rng = np.random.default_rng(77)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ── left: plan view ────────────────────────────────────────────────
    ax = axes[0]

    track_pos = np.array([0.0, 0.0])

    # P covariance (small, tilted)
    P2 = np.array([[1.8, 0.6], [0.6, 0.9]])
    # S = P + R adds isotropic measurement noise
    R2 = np.array([[4.5, 0.0], [0.0, 4.5]])
    S2 = P2 + R2
    gamma = 9.21  # chi2 2-DOF 99%

    def eigellipse(ax, cov, mu, scale, edgecolor, linestyle, linewidth, label=None):
        vals, vecs = np.linalg.eigh(cov)
        order = np.argsort(vals)[::-1]
        vals, vecs = vals[order], vecs[:, order]
        angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
        w = 2 * scale * np.sqrt(vals[0])
        h = 2 * scale * np.sqrt(vals[1])
        e = Ellipse(mu, w, h, angle=angle,
                    edgecolor=edgecolor, facecolor="none",
                    linestyle=linestyle, linewidth=linewidth, label=label)
        ax.add_patch(e)

    # Gate ellipse (larger, light blue)
    eigellipse(ax, S2, track_pos, scale=np.sqrt(gamma),
               edgecolor="#85bbdb", linestyle="-", linewidth=2.2,
               label=r"Gate ellipse $\sqrt{\gamma}\,\mathrm{ellipse}(S)$")

    # P covariance ellipse (small, dark navy)
    eigellipse(ax, P2, track_pos, scale=2.0,
               edgecolor="#1f3a5f", linestyle="-", linewidth=2.2,
               label=r"$P$ ellipse (2$\sigma$ position covariance)")

    # Track position marker
    ax.plot(*track_pos, marker="o", markersize=12, color="#1f3a5f", zorder=5)
    ax.text(track_pos[0] + 0.3, track_pos[1] + 0.3, "track", color="#1f3a5f", fontsize=11)

    # In-gate detection (green)
    z_in = np.array([1.5, -0.8])
    ax.plot(*z_in, marker="o", markersize=12, color="#2d8659", zorder=5)
    ax.text(z_in[0] + 0.2, z_in[1] - 0.5, "in-gate\ndetection", color="#2d8659",
            fontsize=10, ha="left")
    # Association line from in-gate detection to track
    ax.annotate("", xy=track_pos, xytext=z_in,
                arrowprops=dict(arrowstyle="-", color="#2d8659", linewidth=1.8,
                                linestyle="solid"))

    # Out-of-gate detection (red)
    z_out = np.array([-5.5, 3.2])
    ax.plot(*z_out, marker="o", markersize=12, color="#aa3333", zorder=5)
    ax.text(z_out[0] - 0.2, z_out[1] + 0.4, "out-of-gate\ndetection",
            color="#aa3333", fontsize=10, ha="right")

    # Bearing wedge from sensor (lower-left)
    sensor = np.array([-7.0, -6.0])
    ax.plot(*sensor, marker="^", markersize=14, color="#2d8659", zorder=5)
    ax.text(sensor[0] + 0.2, sensor[1] - 0.8, "sensor", color="#2d8659", fontsize=10)

    alpha = np.arctan2(track_pos[1] - sensor[1], track_pos[0] - sensor[0])
    sigma_b = np.radians(6.0)
    k = 2.0
    length = 13.0
    for sign in [+1, -1]:
        a_edge = alpha + sign * k * sigma_b
        tip = sensor + length * np.array([np.cos(a_edge), np.sin(a_edge)])
        ax.plot([sensor[0], tip[0]], [sensor[1], tip[1]],
                color="#2d8659", linestyle="--", linewidth=1.6)
    # Fill wedge lightly
    theta_fill = np.linspace(alpha - k * sigma_b, alpha + k * sigma_b, 30)
    xs = [sensor[0]] + list(sensor[0] + length * np.cos(theta_fill)) + [sensor[0]]
    ys = [sensor[1]] + list(sensor[1] + length * np.sin(theta_fill)) + [sensor[1]]
    ax.fill(xs, ys, color="#2d8659", alpha=0.10)

    handles = [
        Line2D([0], [0], color="#1f3a5f", linewidth=2, label=r"$P$ ellipse (2$\sigma$)"),
        Line2D([0], [0], color="#85bbdb", linewidth=2,
               label=r"gate ellipse $\sqrt{\gamma}\cdot\mathrm{ellipse}(S)$"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2d8659",
               markersize=10, label="in-gate detection (association line shown)"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#aa3333",
               markersize=10, label="out-of-gate detection (dropped)"),
        Line2D([0], [0], color="#2d8659", linestyle="--", linewidth=1.5,
               label="bearing wedge (2σ half-angle)"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=9, framealpha=0.95)

    ax.set_xlim(-10, 8)
    ax.set_ylim(-9, 7)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Plan view: covariance ellipse, gate, association line, bearing wedge")
    ax.grid(True, linestyle=":", alpha=0.4)

    # ── right: NIS time series ─────────────────────────────────────────
    ax2 = axes[1]
    N = 300
    k_dim = 2  # measurement dimension
    samples = rng.chisquare(k_dim, N)
    # Insert two moderate spikes
    samples[80] = 8.5
    samples[210] = 9.8
    cumulative_mean = np.cumsum(samples) / np.arange(1, N + 1)

    t = np.arange(N)
    ax2.scatter(t, samples, s=8, color="#1f3a5f", alpha=0.35,
                label=r"per-update $\epsilon_{NIS}$")
    ax2.plot(t, cumulative_mean, color="#aa3333", linewidth=2.0,
             label="running mean")

    n_arr = np.arange(1, N + 1)
    var_nis = 2 * k_dim
    upper = k_dim + 1.96 * np.sqrt(var_nis / n_arr)
    lower = k_dim - 1.96 * np.sqrt(var_nis / n_arr)
    ax2.fill_between(t, lower, upper, color="#9cc3e8", alpha=0.45,
                     label="95 % band for running mean")
    ax2.axhline(k_dim, color="#1f3a5f", linestyle="--", linewidth=1.4,
                label=f"expected $m = {k_dim}$")

    ax2.set_xlabel("update number")
    ax2.set_ylabel(r"$\epsilon_{NIS}$")
    ax2.set_title("NIS time series — running mean stays near m = 2 (consistent filter)")
    ax2.set_ylim(0, 12)
    ax2.legend(loc="upper right", fontsize=9)
    ax2.grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("Seeing the tracker: reading Foxglove output", y=1.01, fontsize=14)
    save(fig, "seeing-the-tracker.png")


def fig_ospa_vs_gospa():
    """OSPA vs GOSPA growth with number of missed targets."""
    c = 20.0
    d = 5.0
    n_miss_range = np.arange(0, 11)
    ospa, gospa = [], []
    for k in n_miss_range:
        n = max(1, 1 + k)
        ospa.append(np.sqrt((d * d + k * c * c) / n))
        gospa.append(np.sqrt(d * d + k * c * c / 2.0))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(n_miss_range, ospa, marker="o", color="#aa3333", linewidth=2,
            markersize=8, label=f"OSPA (c={int(c)} m, p=2)")
    ax.plot(n_miss_range, gospa, marker="s", color="#1f3a5f", linewidth=2,
            markersize=8, label=f"GOSPA (c={int(c)} m, p=alpha=2)")
    ax.axhline(c, color="#aa3333", linestyle=":", linewidth=1.2,
               label=f"OSPA bound = c = {int(c)} m")
    ax.set_xlabel("number of missed truths (1 matched pair, error d=5 m)")
    ax.set_ylabel("metric value (m)")
    ax.set_title("OSPA saturates at c; GOSPA grows — cardinality errors surface")
    ax.set_xticks(n_miss_range)
    ax.set_ylim(0, max(gospa) * 1.05)
    ax.legend(loc="upper left")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "20-ospa-vs-gospa-cardinality.png")


# ──────────────────────────────────────────────────────────────────────────────
# 26 static obstacles
# ──────────────────────────────────────────────────────────────────────────────

def fig_static_obstacle_zones():
    """Hard footprint core + soft keep-clear ring + passing vessel + suppressed phantom."""
    fig, ax = plt.subplots(figsize=(8, 8))

    cx, cy = 0.0, 0.0   # obstacle centre
    r_hard = 1.5         # footprint core radius
    r_soft = 4.0         # keep-clear ring outer radius

    # Soft keep-clear ring (amber, between r_hard and r_soft)
    ring_amber = plt.Circle((cx, cy), r_soft, color="#e8a030", alpha=0.22, zorder=1)
    ax.add_patch(ring_amber)
    ring_amber_edge = plt.Circle((cx, cy), r_soft, fill=False, edgecolor="#e8a030",
                                  linewidth=2.0, linestyle="--", zorder=2)
    ax.add_patch(ring_amber_edge)

    # Hard footprint disc (red)
    disc_red = plt.Circle((cx, cy), r_hard, color="#aa3333", alpha=0.45, zorder=3)
    ax.add_patch(disc_red)
    disc_red_edge = plt.Circle((cx, cy), r_hard, fill=False, edgecolor="#aa3333",
                                linewidth=2.0, zorder=4)
    ax.add_patch(disc_red_edge)

    # Suppressed phantom at obstacle centre
    ax.plot(cx, cy, marker="x", markersize=18, color="#aa3333", markeredgewidth=3.5,
            zorder=6)
    ax.text(cx + 0.15, cy - 0.45, "suppressed\n(birth hard-gated)", fontsize=10,
            color="#aa3333", ha="left", va="top", zorder=7)

    # Green vessel track passing through the ring (left to right)
    track_y = 2.9          # inside soft ring, outside hard core
    track_xs = np.linspace(-6.5, 6.5, 300)

    # Colour track: green inside soft ring, light blue outside
    inside_ring = np.abs(np.sqrt((track_xs - cx) ** 2 + (track_y - cy) ** 2) - 0) <= r_soft
    # Split into inside-ring and outside-ring segments
    for segment_inside in [True, False]:
        mask = inside_ring if segment_inside else ~inside_ring
        # find contiguous blocks
        indices = np.where(mask)[0]
        if len(indices) == 0:
            continue
        color = "#2d8659" if segment_inside else "#85bbdb"
        lw = 2.5
        # plot in one go (gaps will appear naturally)
        xs_seg = track_xs.copy()
        xs_seg[~mask] = np.nan
        ax.plot(xs_seg, np.full_like(xs_seg, track_y), color=color, linewidth=lw,
                zorder=5)

    # Arrow showing direction of travel
    ax.annotate("", xy=(4.5, track_y), xytext=(2.0, track_y),
                arrowprops=dict(arrowstyle="->", color="#2d8659", lw=2.0), zorder=5)

    # Label "still tracked" above the track, inside the ring
    ax.text(0.0, track_y + 0.35, "still tracked\n(soft suppression only)",
            fontsize=10, color="#2d8659", ha="center", va="bottom", zorder=7)

    # Radius annotations
    ax.annotate("", xy=(r_hard, cy), xytext=(cx, cy),
                arrowprops=dict(arrowstyle="-", color="#aa3333", lw=1.5,
                                linestyle="dotted"), zorder=2)
    ax.text(r_hard / 2, -0.3, r"$R_\mathrm{hard}$", fontsize=10, color="#aa3333",
            ha="center", va="top")

    ax.annotate("", xy=(r_soft, cy), xytext=(r_hard, cy),
                arrowprops=dict(arrowstyle="-", color="#e8a030", lw=1.5,
                                linestyle="dotted"), zorder=2)
    ax.text((r_hard + r_soft) / 2, -0.3, r"$R_\mathrm{soft}$", fontsize=10,
            color="#9a7010", ha="center", va="top")

    # Zone labels
    ax.text(cx, cy + r_hard * 0.45, "footprint\ncore", fontsize=9,
            color="#6b1010", ha="center", va="center", fontweight="bold")
    ax.text(cx, r_soft - 0.55, "keep-clear\nbuffer", fontsize=9,
            color="#9a7010", ha="center", va="center")
    ax.text(5.5, -4.5, "clear\nwater", fontsize=9, color="#1f3a5f",
            ha="center", va="center")

    ax.set_xlim(-7, 7)
    ax.set_ylim(-6, 6)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Static-obstacle zones: birth suppression + keep-clear ring")
    ax.grid(True, linestyle=":", alpha=0.35)

    # Legend
    legend_handles = [
        mpatches.Patch(color="#aa3333", alpha=0.5, label=f"Hard core (R_hard): birth dropped"),
        mpatches.Patch(color="#e8a030", alpha=0.35, label=f"Keep-clear buffer: birth weakened"),
        Line2D([0], [0], color="#2d8659", linewidth=2.5, label="Vessel track (still tracked)"),
        Line2D([0], [0], color="#aa3333", linewidth=0, marker="x", markersize=10,
               markeredgewidth=2.5, label="Phantom (suppressed inside core)"),
    ]
    ax.legend(handles=legend_handles, loc="lower left", fontsize=9, framealpha=0.92)

    save(fig, "26-static-obstacle-zones.png")


# ──────────────────────────────────────────────────────────────────────────────
# 25 coastline clutter prior — the signed-distance shoreline ramp c(d)
# ──────────────────────────────────────────────────────────────────────────────

def fig_coastline_ramp():
    """The shoreline clutter prior c(d) vs signed distance to the nearest shore
    edge. c(d) = clamp((W_off - d) / (W_off + W_in), 0, 1). d<0 inland, d>0
    offshore. Marks the inland hard-gate plateau and the offshore taper."""
    W_in = 50.0    # inland half-width: c reaches 1.0 this far inland
    W_off = 50.0   # offshore half-width: c reaches 0.0 this far offshore
    hard_gate = 0.95

    d = np.linspace(-80.0, 80.0, 800)
    c = np.clip((W_off - d) / (W_off + W_in), 0.0, 1.0)

    fig, ax = plt.subplots(figsize=(9, 5))

    # Shade the three regimes.
    ax.axvspan(-80.0, -W_in, color="#aa3333", alpha=0.10)   # inland plateau (c=1)
    ax.axvspan(-W_in, W_off, color="#e8a030", alpha=0.10)   # soft taper band
    ax.axvspan(W_off, 80.0, color="#2d8659", alpha=0.10)    # open water (c=0)

    # The ramp itself.
    ax.plot(d, c, color="#1f3a5f", linewidth=2.6, zorder=5, label=r"$c(d)$")

    # Hard-gate threshold: births with c > 0.95 are dropped. c=0.95 at d=-45 m.
    d_gate = W_off - hard_gate * (W_off + W_in)   # solve c(d)=0.95 → d = -45
    ax.axhline(hard_gate, color="#aa3333", linewidth=1.3, linestyle="--",
               alpha=0.8, zorder=4)
    ax.text(78.0, hard_gate + 0.015, "hard gate  c = 0.95\n(birth dropped above)",
            color="#aa3333", fontsize=9, ha="right", va="bottom")

    # Waterline marker at d=0, c≈0.5.
    ax.plot(0.0, 0.5, marker="o", markersize=8, color="#1f3a5f", zorder=6)
    ax.annotate("waterline\nd = 0,  c ≈ 0.5", xy=(0.0, 0.5), xytext=(14.0, 0.62),
                fontsize=9, color="#1f3a5f",
                arrowprops=dict(arrowstyle="->", color="#1f3a5f", lw=1.2))

    # Region labels.
    ax.text(-65.0, 0.30, "inland hard-gate\nplateau  (c = 1)", color="#8a2020",
            fontsize=9.5, ha="center", va="center", fontweight="bold")
    ax.text(0.0, 0.14, "soft taper\n(birth weakened)", color="#9a7010",
            fontsize=9.5, ha="center", va="center")
    ax.text(65.0, 0.30, "open water\n(c = 0, no effect)", color="#1c6b45",
            fontsize=9.5, ha="center", va="center")

    # Half-width guides.
    for x, lab in [(-W_in, r"$-W_\mathrm{in}$"), (W_off, r"$+W_\mathrm{off}$")]:
        ax.axvline(x, color="#888888", linewidth=1.0, linestyle=":", zorder=2)
        ax.text(x, -0.09, lab, color="#555555", fontsize=10, ha="center",
                va="top")

    ax.axvline(0.0, color="#bbbbbb", linewidth=0.9, zorder=1)
    ax.set_xlim(-80.0, 80.0)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlabel("signed distance to shore  d  (m)     ← inland      offshore →")
    ax.set_ylabel("clutter prior  c(d)")
    ax.set_title("Shoreline clutter prior: the signed-distance ramp  "
                 r"($W_\mathrm{in}=W_\mathrm{off}=50$ m)")
    ax.grid(True, linestyle=":", alpha=0.4)
    save(fig, "25-coastline-ramp.png")


# ──────────────────────────────────────────────────────────────────────────────
# 27 live occupancy grid + EWMA persistence
# ──────────────────────────────────────────────────────────────────────────────

def fig_live_occupancy():
    """Left: a learned occupancy grid (pier = extended, boat = compact, clutter
    = transient). Right: EWMA persistence over scans for a structure cell, a
    passing-vessel cell, and a clutter cell."""
    fig, (axg, axe) = plt.subplots(1, 2, figsize=(13, 5.5),
                                   gridspec_kw={"width_ratios": [1.05, 1.0]})

    # ---- Left panel: occupancy grid heatmap ----------------------------------
    nx, ny = 10, 8
    grid = np.zeros((ny, nx))
    rng = np.random.default_rng(7)
    # faint transient clutter everywhere
    grid += rng.exponential(0.04, (ny, nx))
    # a pier: an extended line of high-persistence cells (row 2, cols 2..6)
    for cx in range(2, 7):
        grid[2, cx] = 0.68 + 0.03 * rng.standard_normal()
    # a lone anchored boat: one hot but COMPACT cell
    grid[5, 8] = 0.72
    grid = np.clip(grid, 0, 1)

    im = axg.imshow(grid, origin="lower", cmap="YlOrRd", vmin=0, vmax=1,
                    extent=[0, nx * 25, 0, ny * 25], aspect="equal")
    cbar = fig.colorbar(im, ax=axg, fraction=0.046, pad=0.04)
    cbar.set_label("persistence  p")

    # Outline the emitted structure (the pier) — extended → suppressed.
    axg.add_patch(Rectangle((2 * 25, 2 * 25), 5 * 25, 1 * 25, fill=False,
                            edgecolor="#1f3a5f", linewidth=2.5, zorder=5))
    axg.annotate("pier: persistent AND extended\n→ uncharted hazard (suppressed)",
                 xy=(4.5 * 25, 3 * 25), xytext=(0.4 * 25, 6.4 * 25),
                 color="#1f3a5f", fontsize=9.5,
                 arrowprops=dict(arrowstyle="->", color="#1f3a5f"))
    # Mark the anchored boat — compact → NOT suppressed.
    axg.add_patch(Rectangle((8 * 25, 5 * 25), 25, 25, fill=False,
                            edgecolor="#2d8659", linewidth=2.5, zorder=5))
    axg.annotate("anchored boat: persistent\nbut COMPACT → not suppressed",
                 xy=(8.5 * 25, 5.5 * 25), xytext=(2.3 * 25, 0.4 * 25),
                 color="#2d8659", fontsize=9.5,
                 arrowprops=dict(arrowstyle="->", color="#2d8659"))
    axg.set_xlabel("East (m)")
    axg.set_ylabel("North (m)")
    axg.set_title("Learned occupancy grid (25 m cells)")

    # ---- Right panel: EWMA persistence over scans ----------------------------
    alpha = 0.3
    bar = 0.5
    n = 40
    scans = np.arange(n)

    def ewma(hits):
        p, out = 0.0, []
        for h in hits:
            p = (1 - alpha) * p + alpha * h   # decay then bump (w in {0,1})
            out.append(p)
        return np.array(out)

    # structure: fed (w=1) almost every scan
    struct_hits = np.ones(n)
    struct_hits[[7, 19, 28]] = 0.0
    p_struct = ewma(struct_hits)
    # passing vessel: fed for scans 5..9 only, then gone (decays)
    vessel_hits = np.zeros(n)
    vessel_hits[5:10] = 1.0
    p_vessel = ewma(vessel_hits)
    # clutter: sporadic single hits, never sustained
    clutter_hits = np.zeros(n)
    clutter_hits[[3, 12, 13, 24, 33]] = 1.0
    p_clutter = ewma(clutter_hits)

    axe.plot(scans, p_struct, color="#aa3333", linewidth=2.5,
             label="structure cell (fed most scans)")
    axe.plot(scans, p_vessel, color="#2d8659", linewidth=2.5,
             label="passing vessel (fed scans 5–9, then gone)")
    axe.plot(scans, p_clutter, color="#85bbdb", linewidth=2.2,
             label="clutter cell (sporadic)")
    axe.axhline(bar, color="#555", linestyle="--", linewidth=1.4)
    axe.text(n - 0.5, bar + 0.02, r"persistence_bar = 0.5", ha="right",
             va="bottom", fontsize=9, color="#333")
    axe.set_xlabel("scan number")
    axe.set_ylabel("EWMA persistence  p")
    axe.set_title(r"Persistence fades old evidence: $p \leftarrow (1-\alpha)p + \alpha w,\ \alpha = 0.3$")
    axe.set_ylim(0, 1.02)
    axe.grid(True, linestyle=":", alpha=0.4)
    axe.legend(loc="center right", fontsize=9, framealpha=0.92)

    fig.subplots_adjust(wspace=0.32)
    save(fig, "27-live-occupancy.png")


def fig_shadow_guard():
    """LOS/shadow guard: a swept cell behind a closer occluder is NOT observed-empty."""
    fig, ax = plt.subplots(figsize=(9, 8))

    sx, sy = 0.0, 0.0  # own-ship / sensor
    half = np.deg2rad(9.0)  # shadow half-angle (occluder extent + pad)
    r_occ = 3.0             # occluder range (car carrier)
    r_cov = 12.0            # coverage sector range (swept)

    # Coverage sector (swept area) — a light disc out to r_cov.
    cov = plt.Circle((sx, sy), r_cov, color="#85bbdb", alpha=0.14, zorder=1)
    ax.add_patch(cov)
    cov_edge = plt.Circle((sx, sy), r_cov, fill=False, edgecolor="#5a8fb0",
                          linewidth=1.5, linestyle="--", zorder=2)
    ax.add_patch(cov_edge)
    ax.text(0.0, r_cov - 0.7, "swept coverage sector\n(observed this scan)",
            fontsize=9, color="#33627d", ha="center", va="top", zorder=6)

    # Shadow wedge behind the occluder (bearing ~ +90° here, i.e. straight up).
    ang0 = np.pi / 2 - half
    ang1 = np.pi / 2 + half
    # wedge polygon from just beyond the occluder out to the coverage edge
    r0, r1 = r_occ + 0.6, r_cov + 1.0
    wedge_x = [r0 * np.cos(ang0), r1 * np.cos(ang0), r1 * np.cos(ang1),
               r0 * np.cos(ang1)]
    wedge_y = [r0 * np.sin(ang0), r1 * np.sin(ang0), r1 * np.sin(ang1),
               r0 * np.sin(ang1)]
    ax.add_patch(mpatches.Polygon(list(zip(wedge_x, wedge_y)), closed=True,
                                  color="#555555", alpha=0.28, zorder=3))
    ax.text(-3.4, 8.6, "SHADOW\n(line of sight blocked →\nnot 'observed empty')",
            fontsize=10, color="#333333", ha="center", va="center", zorder=7,
            fontweight="bold")

    # Occluder cluster (car carrier) — a few closer returns on the bearing.
    occ_x = np.array([-0.5, 0.0, 0.5, -0.25, 0.25]) * 0.9
    occ_y = np.full_like(occ_x, r_occ)
    ax.scatter(occ_x, occ_y, s=90, marker="s", color="#aa3333", zorder=6,
               edgecolor="white", linewidth=0.8)
    ax.text(1.4, r_occ, "closer occluder\n(car carrier — CFAR plots)", fontsize=9,
            color="#aa3333", ha="left", va="center", zorder=7)

    # Shadowed moored-yacht cell (behind the occluder) — HELD.
    yx, yy = 0.0, 8.5
    ax.add_patch(plt.Rectangle((yx - 0.6, yy - 0.6), 1.2, 1.2, color="#2d8659",
                               alpha=0.75, zorder=6))
    ax.text(yx + 1.0, yy, "moored yacht cell:\nswept but SHADOWED →\ndecay SKIPPED (held)",
            fontsize=9, color="#1f6b46", ha="left", va="center", zorder=7)

    # Control cell: swept, in the open (different bearing), no return → decays.
    cxo = r_cov * 0.62 * np.cos(np.deg2rad(20.0))
    cyo = r_cov * 0.62 * np.sin(np.deg2rad(20.0))
    ax.add_patch(plt.Rectangle((cxo - 0.6, cyo - 0.6), 1.2, 1.2, fill=False,
                               edgecolor="#c07a10", linewidth=2.0, linestyle=":",
                               zorder=6))
    ax.text(cxo + 0.9, cyo, "open swept cell,\nno return →\nobserved empty → decays",
            fontsize=9, color="#9a6010", ha="left", va="center", zorder=7)

    # Sight lines from the sensor.
    ax.annotate("", xy=(0.0, r_occ - 0.2), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="->", color="#aa3333", lw=1.8), zorder=5)
    ax.plot([0.0, 0.0], [r_occ + 0.2, yy - 0.7], color="#aa3333", lw=1.2,
            linestyle=":", zorder=4)  # blocked continuation
    ax.annotate("", xy=(cxo * 0.92, cyo * 0.92), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="->", color="#c07a10", lw=1.6), zorder=5)

    # Own-ship marker.
    ax.plot(sx, sy, marker="^", markersize=15, color="#1f3a5f", zorder=8)
    ax.text(sx, sy - 0.8, "own-ship\n(sensor)", fontsize=9, color="#1f3a5f",
            ha="center", va="top", zorder=8)

    ax.set_xlim(-8, 9)
    ax.set_ylim(-2, 14)
    ax.set_aspect("equal")
    ax.set_xlabel("East (rel. m, schematic)")
    ax.set_ylabel("North (rel. m, schematic)")
    ax.set_title("LOS / shadow guard: absence of a return behind an occluder\n"
                 "is a shadow, not vacancy")
    ax.grid(True, linestyle=":", alpha=0.30)
    save(fig, "27-los-shadow-guard.png")


def fig_innovation_gate():
    """#25 Phase 2b: the update-acceptance position-innovation gate.

    Left: geometry — a small innovation is a trusted return; a gross innovation
    is the WRONG return, so accept its position but distrust the implied velocity.
    Right: the real imazu_15 dying-track shape — moderate innovations pump the
    velocity, then one oversized innovation; the gate fires and deweights velocity
    so the estimate cannot fly off."""
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5.6))

    # ---- Panel A: geometry ----
    px, py = 0.0, 0.0  # predicted position (post-predict, pre-update)
    dmax = 4.0         # D_max acceptance radius (schematic; ≈ 400 m in the config)
    circ = plt.Circle((px, py), dmax, fill=False, edgecolor="#5a8fb0",
                      linewidth=1.6, linestyle="--", zorder=2)
    axL.add_patch(circ)
    axL.text(0, dmax + 0.25, "D_max (position-innovation gate)", ha="center",
             fontsize=9, color="#33627d")
    axL.scatter([px], [py], s=90, marker="P", color="#333333", zorder=5)
    axL.text(px + 0.2, py - 0.5, "predicted\nposition", fontsize=9, va="top")

    # trusted (small-innovation) return
    gx, gy = 1.4, 1.0
    axL.scatter([gx], [gy], s=110, marker="o", color="#2d8659", zorder=5,
                edgecolor="white")
    axL.annotate("", xy=(gx, gy), xytext=(px, py),
                 arrowprops=dict(arrowstyle="->", color="#2d8659", lw=1.8))
    axL.text(gx + 0.15, gy, "small innovation\n→ trusted return\n(normal update)",
             fontsize=9, color="#2d8659", va="center")

    # gross (oversized-innovation) return = wrong vessel / clutter
    bx, by = 5.6, -3.0
    axL.scatter([bx], [by], s=130, marker="X", color="#aa3333", zorder=5,
                edgecolor="white")
    axL.annotate("", xy=(bx, by), xytext=(px, py),
                 arrowprops=dict(arrowstyle="->", color="#aa3333", lw=2.2))
    axL.text(bx - 0.3, by - 0.4, "GROSS innovation (> D_max)\n= the wrong return",
             fontsize=9, color="#aa3333", ha="right", va="top", fontweight="bold")

    # ungated consequence: velocity dumped → fly-off
    axL.annotate("", xy=(bx + 2.4, by - 1.3), xytext=(bx, by),
                 arrowprops=dict(arrowstyle="-|>", color="#aa3333", lw=3.0,
                                 linestyle=":"))
    axL.text(bx + 2.5, by - 1.5, "UNGATED: velocity\ndumps → track flies off",
             fontsize=9, color="#aa3333", va="top")
    # gated: accept position, deweight velocity (fat uncertainty blob)
    axL.add_patch(mpatches.Ellipse((bx, by), 2.6, 2.0, color="#5a8fb0",
                                   alpha=0.30, zorder=3))
    axL.text(bx, by + 1.3, "GATED: accept position,\ndeweight velocity\n"
             "(wide → re-learn)", fontsize=9, color="#33627d", ha="center")
    axL.set_xlim(-5, 9.5); axL.set_ylim(-6.5, 5.5); axL.set_aspect("equal")
    axL.set_title("Acceptance gate: a huge position innovation\nis the wrong "
                  "measurement — accept where, not how-fast")
    axL.grid(True, linestyle=":", alpha=0.3)

    # ---- Panel B: the real dying-track shape (imazu_15 id 6) ----
    t = np.array([315, 320, 325, 327, 330, 335, 337, 340, 345, 350, 355, 358, 362])
    innov = np.array([18, 29, 10, 67, 22, 11, 54, 71, 77, 105, 484, 436, 128.])
    speed = np.array([2, 8, 12, 34, 22, 42, 75, 101, 122, 150, 178, 223, 150.])
    axR.plot(t, innov, "-o", color="#aa3333", lw=1.8, ms=4, label="position innovation (m)")
    axR.axhline(400, color="#33627d", ls="--", lw=1.4)
    axR.text(316, 415, "D_max = 400 m", color="#33627d", fontsize=9)
    axR.axvspan(327, 352, color="#e8b04b", alpha=0.18)
    axR.text(339, 300, "moderate innovations\n(CT-mode pumps velocity)",
             ha="center", fontsize=8.5, color="#8a6d1f")
    axR.annotate("gate fires\n(deweight velocity)", xy=(355, 484), xytext=(330, 455),
                 fontsize=9, color="#aa3333", fontweight="bold",
                 arrowprops=dict(arrowstyle="->", color="#aa3333"))
    axR.set_ylim(0, 560)
    axR.set_xlabel("time (s)"); axR.set_ylabel("position innovation (m)")
    axR2 = axR.twinx()
    axR2.plot(t, speed, "-s", color="#2f6f9f", lw=1.4, ms=3, alpha=0.8,
              label="implied speed (m/s)")
    axR2.set_ylabel("implied speed (m/s)", color="#2f6f9f")
    axR.set_title("Runaway onset: a moderate-innovation build-up,\nthen one "
                  "oversized innovation trips the gate")
    axR.grid(True, linestyle=":", alpha=0.3)
    axR.legend(loc="upper left", fontsize=8)
    axR2.legend(loc="center left", fontsize=8)

    save(fig, "11-innovation-gate.png")


def fig_bearing_wedge():
    """Camera-only contact → a bearing wedge from own-ship; handover by suppression."""
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 6.2), sharex=True, sharey=True)

    apex = (0.0, 0.0)
    bearing_deg = 63.0          # true-ish bearing of the camera contact
    half_deg = 9.0              # exaggerated half-width for legibility
    Rmax = 9.3                  # draw length (range is actually UNKNOWN)

    def draw_common(ax, title):
        # Unbounded wedge (drawn to the plot edge; range genuinely unknown).
        wedge = mpatches.Wedge(apex, Rmax, bearing_deg - half_deg,
                               bearing_deg + half_deg, color="#e8a030",
                               alpha=0.28, zorder=1)
        ax.add_patch(wedge)
        wedge_edge = mpatches.Wedge(apex, Rmax, bearing_deg - half_deg,
                                    bearing_deg + half_deg, fill=False,
                                    edgecolor="#e8a030", linewidth=1.8,
                                    linestyle="--", zorder=2)
        ax.add_patch(wedge_edge)
        # Centre bearing line.
        br = np.deg2rad(bearing_deg)
        ax.plot([0, Rmax * np.cos(br)], [0, Rmax * np.sin(br)],
                color="#b5791f", linewidth=1.4, linestyle=":", zorder=3)
        # Own-ship apex.
        ax.plot(*apex, marker="^", markersize=16, color="#26527a", zorder=6)
        ax.text(0.15, -0.55, "own-ship\n(apex)", fontsize=10, color="#26527a",
                ha="left", va="top")
        # "range unknown" arrow along the bearing.
        ax.annotate("", xy=(Rmax * np.cos(br), Rmax * np.sin(br)),
                    xytext=(5.0 * np.cos(br), 5.0 * np.sin(br)),
                    arrowprops=dict(arrowstyle="->", color="#b5791f", lw=1.6),
                    zorder=4)
        ax.text(8.6, 8.9, "range unknown →", fontsize=9.5, color="#b5791f",
                rotation=bearing_deg, ha="center", va="center")
        ax.set_title(title, fontsize=12)
        ax.set_xlim(-2.0, 11.0)
        ax.set_ylim(-2.0, 11.0)
        ax.set_aspect("equal")
        ax.grid(True, linestyle=":", alpha=0.35)
        ax.set_xlabel("east (arb.)")

    # LEFT: the wedge is emitted (no track claims it).
    draw_common(axL, "Camera-only contact → wedge emitted")
    # camera contact marker somewhere on the line (range unknown).
    br = np.deg2rad(bearing_deg)
    cx, cy = 6.5 * np.cos(br), 6.5 * np.sin(br)
    axL.plot(cx, cy, marker="*", markersize=16, color="#7a3fb5", zorder=6)
    axL.text(cx + 0.3, cy, "camera contact\n(bearing only)", fontsize=9.5,
             color="#7a3fb5", ha="left", va="center")
    axL.annotate(f"half-width = 2σ\n(σ = camera ⊕ heading)",
                 xy=(3.2 * np.cos(br + np.deg2rad(half_deg)),
                     3.2 * np.sin(br + np.deg2rad(half_deg))),
                 xytext=(0.4, 8.7), fontsize=9.5, color="#b5791f",
                 arrowprops=dict(arrowstyle="->", color="#b5791f", lw=1.3))
    axL.set_ylabel("north (arb.)")

    # RIGHT: a confirmed track sits on the bearing → wedge SUPPRESSED (handover).
    draw_common(axR, "Track claims the bearing → wedge suppressed (not deleted)")
    # grey-out the wedge to signal suppression.
    axR.add_patch(mpatches.Wedge(apex, Rmax, bearing_deg - half_deg,
                                 bearing_deg + half_deg, color="#999999",
                                 alpha=0.30, zorder=5))
    tx, ty = 5.2 * np.cos(br), 5.2 * np.sin(br)
    axR.plot(tx, ty, marker="o", markersize=12, color="#2d8659", zorder=7)
    axR.text(tx + 0.3, ty - 0.2, "confirmed track\n(better source)", fontsize=9.5,
             color="#2d8659", ha="left", va="top")
    axR.text(2.2, 9.6, "suppressed while claimed —\nreappears if the track leaves\n"
             "(only camera silence deletes it)", fontsize=9.5, color="#555555",
             ha="left", va="top")

    fig.suptitle("Bearing-wedge hazard: 'never invisible' without a range",
                 fontsize=13)
    fig.subplots_adjust(wspace=0.12, top=0.90)
    save(fig, "28-bearing-wedge.png")


# ──────────────────────────────────────────────────────────────────────────────

def _cov_ellipse(ax, mean, cov, n_std, **kw):
    """Draw an n_std covariance ellipse for a 2x2 cov at mean."""
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    w, h = 2.0 * n_std * np.sqrt(vals)
    e = Ellipse(mean, w, h, angle=angle, fill=False, lw=2.2, **kw)
    ax.add_patch(e)
    return e


def fig_t2t_ci_ellipse():
    """Two estimates of one object with different-shaped covariances. Naive
    (independence-assuming) fusion produces a DANGEROUSLY SMALL ellipse if the
    two secretly share information; covariance intersection stays honest."""
    navy, lblue, red, green = "#1f3a5f", "#85bbdb", "#aa3333", "#2d8659"
    P1 = np.array([[9.0, 0.0], [0.0, 1.0]])   # source A: sure in N, unsure in E
    P2 = np.array([[1.0, 0.0], [0.0, 9.0]])   # source B: sure in E, unsure in N
    I1, I2 = np.linalg.inv(P1), np.linalg.inv(P2)
    P_naive = np.linalg.inv(I1 + I2)          # counts both fully -> tiny
    w = 0.5                                    # CI weight (symmetric here)
    P_ci = np.linalg.inv(w * I1 + (1 - w) * I2)
    mean = np.array([0.0, 0.0])

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 5.2))
    for ax in (axL, axR):
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_aspect("equal")
        ax.axhline(0, color="0.85", lw=0.8); ax.axvline(0, color="0.85", lw=0.8)
        ax.set_xlabel("east error (m)"); ax.set_ylabel("north error (m)")

    _cov_ellipse(axL, mean, P1, 2.0, edgecolor=navy, label="estimate A  (P₁)")
    _cov_ellipse(axL, mean, P2, 2.0, edgecolor=lblue, label="estimate B  (P₂)")
    axL.set_title("Two trackers, one object")
    axL.legend(loc="upper right", fontsize=10)

    _cov_ellipse(axR, mean, P1, 2.0, edgecolor=navy, alpha=0.55)
    _cov_ellipse(axR, mean, P2, 2.0, edgecolor=lblue, alpha=0.55)
    _cov_ellipse(axR, mean, P_naive, 2.0, edgecolor=red,
                 label="naive fuse (overconfident)", ls="--")
    _cov_ellipse(axR, mean, P_ci, 2.0, edgecolor=green, label="CI fuse (honest)")
    axR.set_title("Fuse them")
    axR.legend(loc="upper right", fontsize=10)
    fig.suptitle("Covariance intersection: never claim to know more than justified",
                 fontsize=14)
    save(fig, "29-t2t-ci-ellipse.png")


def fig_t2t_double_counting():
    """Shared-newspaper cartoon: two trackers both read the same AIS, so their
    errors are correlated; naive fusion counts that AIS twice."""
    navy, lblue, red, green = "#1f3a5f", "#85bbdb", "#aa3333", "#2d8659"
    fig, ax = plt.subplots(figsize=(10.5, 5.2))
    ax.set_xlim(0, 10); ax.set_ylim(0, 6); ax.axis("off")

    def box(x, y, w, h, text, fc, tc="white"):
        ax.add_patch(Rectangle((x, y), w, h, facecolor=fc, edgecolor="black", lw=1.4))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
                color=tc, fontsize=11, weight="bold")

    box(0.3, 4.2, 1.9, 0.9, "AIS feed", green)         # shared source
    box(0.3, 1.0, 1.9, 0.9, "radar", navy)
    box(4.0, 4.0, 2.0, 1.0, "Tracker A", lblue, tc="black")
    box(4.0, 1.2, 2.0, 1.0, "Tracker B", lblue, tc="black")
    box(7.8, 2.6, 1.9, 1.0, "Fuser", "#c9a13b", tc="black")

    def arrow(x1, y1, x2, y2, color="black", ls="-"):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                     mutation_scale=16, lw=2, color=color, ls=ls))
    # AIS feeds BOTH trackers (the shared stream), radar only A.
    arrow(2.2, 4.7, 4.0, 4.6, green)
    arrow(2.2, 4.5, 4.0, 1.9, green)     # same AIS into B  <-- the double count
    arrow(2.2, 1.4, 4.0, 4.2, navy)
    arrow(6.0, 4.4, 7.8, 3.4)
    arrow(6.0, 1.7, 7.8, 2.9)
    ax.text(3.1, 3.0, "same AIS\ninto both", color=red, fontsize=10,
            ha="center", weight="bold")
    ax.text(4.9, 0.35,
            "A and B share the AIS stream → their errors are CORRELATED.\n"
            "Naive fusion assumes independence and counts that AIS twice "
            "→ overconfident. CI does not.",
            ha="center", fontsize=10.5)
    ax.set_title("Double counting: the shared-newspaper problem", fontsize=14)
    save(fig, "29-t2t-double-counting.png")


# ──────────────────────────────────────────────────────────────────────────────

def main():
    print("Generating figures into", HERE)
    fig_gaussian_1d()
    fig_cov_ellipses()
    fig_mixture_vs_collapse()
    fig_kf_predict_grow()
    fig_kf_predict_update_cycle()
    fig_kalman_gain_geometry()
    fig_ekf_ruler()
    fig_rangebearing_geometry()
    fig_sigma_points()
    fig_ukf_vs_ekf_propagation()
    fig_pf_bearing_only_evolution()
    fig_pf_degeneracy_resample()
    fig_cv_vs_ct_trajectory()
    fig_imm_mode_probs()
    fig_gating_ellipse()
    fig_jpda_soft()
    fig_clutter_map()
    fig_compound_k_clutter()
    fig_mht_tree()
    fig_nis_regimes()
    fig_nis_timeseries()
    fig_cpa_geometry()
    fig_cpa_hysteresis()
    fig_ospa_vs_gospa()
    fig_sensor_bias_convergence()
    fig_seeing_the_tracker()
    fig_coastline_ramp()
    fig_static_obstacle_zones()
    fig_live_occupancy()
    fig_shadow_guard()
    fig_bearing_wedge()
    fig_innovation_gate()
    fig_t2t_ci_ellipse()
    fig_t2t_double_counting()
    render_dot_figures()
    print("done.")


if __name__ == "__main__":
    main()
