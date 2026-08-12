"""WGS-84 geodetic <-> local ENU conversion (pure numpy, no scipy)."""

import numpy as np

_A  = 6_378_137.0
_F  = 1.0 / 298.257_223_563
_E2 = 2.0 * _F - _F * _F


def _llh_to_ecef(lat_r, lon_r, alt):
    N = _A / np.sqrt(1.0 - _E2 * np.sin(lat_r) ** 2)
    x = (N + alt) * np.cos(lat_r) * np.cos(lon_r)
    y = (N + alt) * np.cos(lat_r) * np.sin(lon_r)
    z = (N * (1.0 - _E2) + alt) * np.sin(lat_r)
    return np.array([x, y, z])


def llh_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """LLH (deg,deg,m) -> ENU [m] relative to (lat0,lon0,alt0)."""
    lat_r,  lon_r  = np.radians(lat),  np.radians(lon)
    lat0_r, lon0_r = np.radians(lat0), np.radians(lon0)

    p  = _llh_to_ecef(lat_r,  lon_r,  alt)
    p0 = _llh_to_ecef(lat0_r, lon0_r, alt0)
    dp = p - p0

    sl, cl = np.sin(lat0_r), np.cos(lat0_r)
    so, co = np.sin(lon0_r), np.cos(lon0_r)

    R = np.array([
        [-so,       co,      0.0],
        [-sl * co, -sl * so, cl ],
        [ cl * co,  cl * so, sl ],
    ])
    return R @ dp


def enu_to_llh(e, n, u, lat0, lon0, alt0):
    """ENU [m] -> LLH (lat_deg, lon_deg, alt_m) given reference (lat0,lon0,alt0)."""
    lat0_r, lon0_r = np.radians(lat0), np.radians(lon0)
    p0 = _llh_to_ecef(lat0_r, lon0_r, alt0)

    sl, cl = np.sin(lat0_r), np.cos(lat0_r)
    so, co = np.sin(lon0_r), np.cos(lon0_r)

    R_T = np.array([
        [-so,  -sl * co,  cl * co],
        [ co,  -sl * so,  cl * so],
        [0.0,   cl,       sl     ],
    ])
    p = p0 + R_T @ np.array([e, n, u])

    x, y, z = p
    lon_r = np.arctan2(y, x)
    p_xy  = np.hypot(x, y)

    lat_r = np.arctan2(z, p_xy * (1.0 - _E2))
    for _ in range(5):
        N     = _A / np.sqrt(1.0 - _E2 * np.sin(lat_r) ** 2)
        lat_r = np.arctan2(z + _E2 * N * np.sin(lat_r), p_xy)

    N   = _A / np.sqrt(1.0 - _E2 * np.sin(lat_r) ** 2)
    alt = (p_xy / np.cos(lat_r) - N) if abs(lat_r) < np.radians(89.9) \
          else (z / np.sin(lat_r) - N * (1.0 - _E2))

    return float(np.degrees(lat_r)), float(np.degrees(lon_r)), float(alt)
