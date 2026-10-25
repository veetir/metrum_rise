# SPDX-License-Identifier: GPL-2.0-only

"""One ordered capability table. First matching rule per role wins.

Public service ways become ordinary streets; driveway, parking and restricted
motor access are excluded explicitly. Footpaths and grade1 tracks are disabled
by default. Grade separation is retained for the player elevation path.

Current road tool supports lane counts and central medians, not named road types or source speed
limits. Minor streets default to 1+1, including untagged service; explicit OSM
one-way tags retain their direction. Divided named roads and motorways become one
centreline in conflate.py, retaining the surveyed counts of both carriageways.
Other major ways retain explicit direction and lane counts.
POLICY owns the spatial and minimum-length rules for standalone paths.
Apartment footprints mask generic residential zoning until apartment growth
exists; other unknown buildings mask generic zoning as well. Industrial buildings
need explicit assets/work areas, not speculative residential/commercial zoning.
"""

from dataclasses import asdict, dataclass

# Offline conflation thresholds in metres/degrees. These are serialized with the
# capability table, so every lossy mapping decision is reproducible and tunable.
POLICY = {'parallel_distance_m': 35.0, 'parallel_angle_deg': 30.0,
          'parallel_fraction': 0.60, 'sample_m': 5.0,
          'carriageway_distance_m': 45.0, 'endpoint_merge_m': 4.0,
          'endpoint_snap_m': 12.0, 'dead_end_path_m': 100.0,
          'standalone_path_min_m': 65.0,
          'water_margin_m': 1.0, 'roundabout_max_diameter_m': 90.0,
          'roundabout_min_circularity': 0.85,
          'minor_default_lanes': [1, 1]}


@dataclass(frozen=True)
class Capability:
    role: str
    field: str
    values: tuple
    action: str
    reason: str
    priority: int = 9


CAPABILITIES = (
    Capability('road', 'roundabout', (True,), 'skip', 'roundabout_ROAD_62'),
    Capability('road', 'road_class', ('motorway', 'trunk'), 'road', 'major_road', 0),
    Capability('road', 'road_class', ('steps', 'corridor', 'construction', 'proposed'), 'skip', 'non_placeable_highway'),
    Capability('road', 'footway', ('sidewalk', 'crossing'), 'skip', 'road_includes_sidewalks'),
    Capability('road', 'bus_only', (True,), 'road', 'dedicated_bus_carriageway', 3),
    Capability('road', 'service', ('parking_aisle', 'driveway', 'drive-through'), 'skip', 'private_or_parking_service'),
    Capability('road', 'access', ('no', 'private', 'permit'), 'skip', 'restricted_access'),
    Capability('road', 'area', (True,), 'skip', 'highway_area_not_centerline'),
    Capability('road', 'road_class', ('primary',), 'road', 'ordinary_road', 0),
    Capability('road', 'road_class', ('secondary',), 'road', 'ordinary_road', 1),
    Capability('road', 'road_class', ('tertiary',), 'road', 'ordinary_road', 2),
    # A through-road Under profile precedes slip joints that would otherwise pin its DEM height.
    Capability('road', 'road_class', ('motorway_link', 'trunk_link', 'primary_link', 'secondary_link', 'tertiary_link'), 'road', 'connecting_road', 3),
    Capability('road', 'road_class', ('residential', 'unclassified', 'living_street', 'road'), 'road', 'ordinary_road', 3),
    Capability('road', 'service', ('alley', 'emergency_access'), 'road', 'explicit_access_street', 4),
    Capability('road', 'road_class', ('service',), 'road', 'ordinary_service_street', 4),
    Capability('road', 'road_class', ('track',), 'track', 'track_requires_grade1', 5),
    Capability('road', 'road_class', ('footway', 'cycleway', 'path', 'pedestrian', 'bridleway'), 'footpath', 'shared_footpath', 6),
    Capability('road', '*', (), 'skip', 'unsupported_highway'),
    Capability('rail', '*', (), 'skip', 'railway_unsupported'),
    Capability('building', 'building_use', ('detached', 'house', 'semidetached_house', 'bungalow'), 'residential', 'low_density_proxy'),
    Capability('building', 'building_use', ('retail', 'commercial'), 'commercial', 'commercial_proxy'),
    Capability('building', 'building_use', ('apartments', 'terrace', 'residential'), 'skip', 'apartment_or_unknown_density_deferred'),
    Capability('building', '*', (), 'skip', 'exact_or_special_building_deferred'),
    Capability('landuse', 'landuse', ('residential',), 'residential', 'coarse_low_density'),
    Capability('landuse', 'landuse', ('commercial', 'retail'), 'commercial', 'low_density_commercial'),
    Capability('landuse', 'landuse', ('industrial',), 'skip', 'industrial_requires_explicit_assets'),
    Capability('landuse', '*', (), 'skip', 'landuse_not_zoneable'),
    Capability('support', '*', (), 'retain', 'geometry_support'),
    Capability('other', '*', (), 'skip', 'no_current_capability'),
)


def table():
    return [asdict(row) for row in CAPABILITIES]


def map_role(feature, role, include_park_paths=False):
    p = feature['properties']
    rule = next(r for r in CAPABILITIES if r.role == role and (r.field == '*' or p.get(r.field) in r.values))
    result = {'action': rule.action, 'reason': rule.reason, 'priority': rule.priority}
    if role == 'road' and rule.action in ('road', 'footpath', 'track'):
        if feature['issues']:
            return {**result, 'action': 'skip', 'reason': 'incomplete_geometry'}
        if rule.action == 'track':
            if p['track_grade'] != 'grade1':
                return {**result, 'action': 'skip', 'reason': 'track_not_grade1'}
            result.update(action='footpath', reason='grade1_track_as_footpath')
        if p.get('relation_type') or len(feature['points']) < 2:
            return {**result, 'action': 'skip', 'reason': 'not_a_centerline'}
        if result['action'] == 'footpath':
            if not include_park_paths:
                return {**result, 'action': 'skip', 'reason': 'paths_disabled'}
            result.update(fwd_lanes=0, bkw_lanes=0)
        else:
            total = p.get('lanes')
            if total is not None and (total < 1 or total > 8 or total != int(total)):
                return {**result, 'action': 'skip', 'reason': 'invalid_or_excessive_lane_count'}
            minor = p['road_class'] in ('service', 'residential', 'unclassified', 'living_street', 'tertiary', 'road')
            oneway = bool(p['oneway'])
            result['oneway'] = p['oneway'] if oneway else 0
            if minor and not oneway and total is None and p.get('lanes_forward') is None and p.get('lanes_backward') is None:
                fwd, bkw = POLICY['minor_default_lanes']
                result['reason'] = 'ordinary_two_way_default'
            elif oneway:
                fwd, bkw = int(total or 1), 0
            else:
                forward, backward = p.get('lanes_forward'), p.get('lanes_backward')
                if forward is not None or backward is not None:
                    fwd = forward if forward is not None else (total-backward if total else 1)
                    bkw = backward if backward is not None else (total-forward if total else 1)
                else:
                    fwd, bkw = (max(1, int(total)//2), max(1, int(total)-int(total)//2)) if total else (1, 1)
            if total == 1 and not oneway and p.get('lanes_forward') is None and p.get('lanes_backward') is None:
                result['reason'] = 'shared_single_lane_widened_to_two'
            if fwd < 0 or bkw < 0 or fwd != int(fwd) or bkw != int(bkw) or not 1 <= fwd+bkw <= 8:
                return {**result, 'action': 'skip', 'reason': 'invalid_directional_lanes'}
            result.update(fwd_lanes=int(fwd), bkw_lanes=int(bkw))
            masks = []
            for count, tokens in ((int(fwd), p.get('bus_lanes_forward', '')),
                                  (int(bkw), p.get('bus_lanes_backward', ''))):
                values = tokens.split('|') if tokens else []
                if p.get('bus_only'):
                    masks.append((1 << count)-1)
                elif values and len(values) == count:
                    masks.append(sum(1 << i for i, token in enumerate(values) if token in ('designated', 'only')))
                else:
                    masks.append(0)
                    if values:
                        result['bus_lane_reason'] = 'source_bus_lane_count_mismatch'
            if any(masks):
                result['bus_masks'] = masks
        result['speed_applied'] = False
    return result
