# SPDX-License-Identifier: GPL-2.0-only

"""Coordinate normalisation only; no OSM tags or game capabilities are consulted."""

from .projection import tm35fin


def normalise(features, transform):
    cache = {}

    def project(point):
        key = tuple(point)
        if key not in cache:
            cache[key] = tuple(round(v, 3) for v in transform.world(*tm35fin(point[1], point[0])))
        return cache[key]

    result = []
    for source in features:
        feature = source.record()
        feature['points'] = [project(p) for p in source.points]
        for ring in feature['rings']:
            ring['points'] = [project(p) for p in ring['points']]
        result.append(feature)
    return result
