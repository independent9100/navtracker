"""Generate all figures for docs/learning/.

Run:  ../../../../../tmp/claude/learning-venv/bin/python generate.py

Saves PNGs into the current directory.
Style: clean matplotlib, large fonts, light grid where useful.
"""

from __future__ import annotations
import os
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
    _draw_ellipse(ax, S, mu=z_hat, color="#1f3a5f", n_sigma=(2.146,))  # χ²_2 99% ≈ 9.21
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
    fig_mht_tree()
    fig_nis_regimes()
    fig_nis_timeseries()
    fig_cpa_geometry()
    fig_cpa_hysteresis()
    fig_ospa_vs_gospa()
    fig_sensor_bias_convergence()
    fig_seeing_the_tracker()
    fig_static_obstacle_zones()
    print("done.")


if __name__ == "__main__":
    main()
