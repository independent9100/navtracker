"""Radar false-plot (clutter) fields.

Two models, selectable per scenario:

* ``poisson``     — homogeneous Poisson point process over the coverage
  annulus. The textbook clutter every tracker's clutter term assumes.
* ``compound_k``  — a *doubly-stochastic* (Cox) process: a gamma-distributed
  texture modulates a local Poisson rate, patch by patch. This is the
  count-level analogue of K-distributed sea clutter (Ward/Watts): a slowly
  varying "texture" (the gamma field) times a fast "speckle" (the Poisson
  draw). The marginal count per patch is negative-binomial — heavier-tailed
  and spatially clumpy, NOT uniform. This is the anti-model-matched-optimism
  lever: a filter whose clutter term assumes flat Poisson should measurably
  under-perform here versus a spatially-varying-λ model, which is exactly what
  the clutter_burst scenario is designed to detect (ticket §5).

All false plots are emitted in the own-ship body frame (range m, bearing deg
relative to bow) so they drop straight into the radar plot CSV alongside real
detections. See docs/learning/ (compound-K subsection) for the intuition.
"""

from __future__ import annotations

import math

import numpy as np


def _annulus_uniform(rng, n, r_min, r_max, az_lo, az_hi):
    """n points uniform in area over an annulus sector (range m, bearing deg)."""
    # area-uniform radius: r = sqrt(U*(r_max^2-r_min^2)+r_min^2)
    u = rng.random(n)
    r = np.sqrt(u * (r_max ** 2 - r_min ** 2) + r_min ** 2)
    az = rng.uniform(az_lo, az_hi, size=n)
    return r, az


def poisson_plots(rng, lam_per_m2, r_min, r_max, scan_dt_s):
    """Homogeneous Poisson clutter over the full 360° coverage annulus for one
    scan. ``lam_per_m2`` is spatial intensity (m^-2) per scan."""
    area = math.pi * (r_max ** 2 - r_min ** 2)
    n = rng.poisson(lam_per_m2 * area)
    if n == 0:
        return np.empty(0), np.empty(0)
    return _annulus_uniform(rng, n, r_min, r_max, 0.0, 360.0)


def compound_k_plots(rng, lam_bar_per_m2, r_min, r_max, shape_nu=0.6, n_patches_az=24):
    """Compound-K (gamma-modulated Poisson) clutter for one scan.

    The coverage annulus is split into ``n_patches_az`` azimuth wedges; each
    wedge draws a gamma texture weight g ~ Gamma(nu, 1/nu) (mean 1, variance
    1/nu). Small ``nu`` => spiky, clumpy clutter. Local count ~ Poisson(
    lam_bar * area_patch * g). Marginal is negative-binomial."""
    az_edges = np.linspace(0.0, 360.0, n_patches_az + 1)
    area_patch = math.pi * (r_max ** 2 - r_min ** 2) / n_patches_az
    rs, azs = [], []
    tex = rng.gamma(shape_nu, 1.0 / shape_nu, size=n_patches_az)
    for k in range(n_patches_az):
        n = rng.poisson(lam_bar_per_m2 * area_patch * tex[k])
        if n == 0:
            continue
        r, az = _annulus_uniform(rng, n, r_min, r_max, az_edges[k], az_edges[k + 1])
        rs.append(r); azs.append(az)
    if not rs:
        return np.empty(0), np.empty(0)
    return np.concatenate(rs), np.concatenate(azs)


def burst_plots(rng, own_e, own_n, own_heading_deg, e_c, n_c, radius_m, count):
    """A spatially localised burst (rain cell / interference / dense return)
    around an absolute ENU point, expressed in the own-ship body frame for the
    given own-ship pose. Returns (range_m, bearing_body_deg)."""
    ang = rng.uniform(0.0, 2 * math.pi, size=count)
    rad = radius_m * np.sqrt(rng.random(count))
    pe = e_c + rad * np.cos(ang)
    pn = n_c + rad * np.sin(ang)
    de, dn = pe - own_e, pn - own_n
    rng_m = np.hypot(de, dn)
    bearing_world = (np.degrees(np.arctan2(de, dn))) % 360.0
    bearing_body = (bearing_world - own_heading_deg) % 360.0
    return rng_m, bearing_body
