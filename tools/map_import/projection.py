# SPDX-License-Identifier: GPL-2.0-only

"""ETRS-TM35FIN forward transverse Mercator on GRS80, Krueger n-series to n^6.

WGS84 geographic coordinates are treated as ETRS89 at this coarse map scale;
no epoch/plate-motion correction is implied. The projection itself is sub-mm
near Finland's central meridian. No pyproj, GDAL or network service is used.
"""

import math


def tm35fin(lat: float, lon: float) -> tuple[float, float]:
    if not (math.isfinite(lat) and math.isfinite(lon) and -80 < lat < 84 and 15 <= lon <= 39):
        raise ValueError("TM35FIN input must be finite and in its Finnish longitude neighborhood")
    a, inv_f, k0 = 6378137.0, 298.257222101, 0.9996
    f = 1 / inv_f
    n = f / (2 - f)
    eccentricity = math.sqrt(f * (2 - f))
    scale = a / (1 + n) * (1 + n**2 / 4 + n**4 / 64 + n**6 / 256)
    alpha = (
        n/2 - 2*n**2/3 + 5*n**3/16 + 41*n**4/180 - 127*n**5/288 + 7891*n**6/37800,
        13*n**2/48 - 3*n**3/5 + 557*n**4/1440 + 281*n**5/630 - 1983433*n**6/1935360,
        61*n**3/240 - 103*n**4/140 + 15061*n**5/26880 + 167603*n**6/181440,
        49561*n**4/161280 - 179*n**5/168 + 6601661*n**6/7257600,
        34729*n**5/80640 - 3418889*n**6/1995840,
        212378941*n**6/319334400,
    )
    phi, lam = math.radians(lat), math.radians(lon - 27)
    tau = math.tan(phi)
    sigma = math.sinh(eccentricity * math.atanh(eccentricity * tau / math.hypot(1, tau)))
    tau_p = tau * math.hypot(1, sigma) - sigma * math.hypot(1, tau)
    xi = math.atan2(tau_p, math.cos(lam))
    eta = math.asinh(math.sin(lam) / math.hypot(tau_p, math.cos(lam)))
    north = xi + sum(v * math.sin(2*j*xi) * math.cosh(2*j*eta) for j, v in enumerate(alpha, 1))
    east = eta + sum(v * math.cos(2*j*xi) * math.sinh(2*j*eta) for j, v in enumerate(alpha, 1))
    return 500000 + k0 * scale * east, k0 * scale * north
