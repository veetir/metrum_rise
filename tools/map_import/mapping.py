# SPDX-License-Identifier: GPL-2.0-only

"""One ordered capability table. First matching rule per role wins.

Service roads only admit explicitly designated alleys or emergency access;
untagged service often means parking circulation/yard access, so it is retained
but not imported. grade1 tracks become footpaths: their geometry is useful but
the game has no farm/forest vehicle access semantics. Private/no access, layered
structures and area highways are deferred rather than silently flattened.

Current road tool supports lane counts, not named road types or source speed
limits. Primary/secondary default to 1+1 ordinary lanes; explicit lane counts and
one-way direction are retained. Shared bidirectional single lanes widen to 1+1,
with an explicit mapping reason. No lane count is inferred from road importance.
Apartment footprints mask generic residential zoning until apartment growth
exists; other unknown buildings mask generic zoning as well. Industrial buildings
need explicit assets/work areas, not speculative residential/commercial zoning.
"""

from dataclasses import asdict, dataclass


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
    Capability('road', 'road_class', ('motorway', 'motorway_link'), 'skip', 'motorway_unsupported'),
    Capability('road', 'road_class', ('steps', 'corridor', 'construction', 'proposed'), 'skip', 'non_placeable_highway'),
    Capability('road', 'service', ('parking_aisle', 'driveway', 'drive-through'), 'skip', 'private_or_parking_service'),
    Capability('road', 'access', ('no', 'private'), 'skip', 'restricted_access'),
    Capability('road', 'bridge', (True,), 'skip', 'grade_separation_deferred'),
    Capability('road', 'tunnel', (True,), 'skip', 'grade_separation_deferred'),
    Capability('road', 'area', (True,), 'skip', 'highway_area_not_centerline'),
    Capability('road', 'road_class', ('primary', 'primary_link'), 'road', 'ordinary_road', 0),
    Capability('road', 'road_class', ('secondary', 'secondary_link'), 'road', 'ordinary_road', 1),
    Capability('road', 'road_class', ('tertiary', 'tertiary_link'), 'road', 'ordinary_road', 2),
    Capability('road', 'road_class', ('residential', 'unclassified', 'living_street', 'road'), 'road', 'ordinary_road', 3),
    Capability('road', 'service', ('alley', 'emergency_access'), 'road', 'explicit_access_street', 4),
    Capability('road', 'road_class', ('service',), 'skip', 'service_yard_semantics_unknown'),
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


def map_role(feature, role):
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
        if p.get('layer', 0) != 0:
            return {**result, 'action': 'skip', 'reason': 'grade_separation_deferred'}
        if result['action'] == 'footpath':
            result.update(fwd_lanes=0, bkw_lanes=0)
        else:
            total = p.get('lanes')
            if total is not None and (total < 1 or total > 8 or total != int(total)):
                return {**result, 'action': 'skip', 'reason': 'invalid_or_excessive_lane_count'}
            if p['oneway']:
                fwd, bkw = int(total or 1), 0
            else:
                forward, backward = p.get('lanes_forward'), p.get('lanes_backward')
                if forward is not None or backward is not None:
                    fwd = forward if forward is not None else (total-backward if total else 1)
                    bkw = backward if backward is not None else (total-forward if total else 1)
                else:
                    fwd, bkw = (max(1, int(total)//2), max(1, int(total)-int(total)//2)) if total else (1, 1)
            if total == 1 and not p['oneway'] and p.get('lanes_forward') is None and p.get('lanes_backward') is None:
                result['reason'] = 'shared_single_lane_widened_to_two'
            if fwd < 0 or bkw < 0 or fwd != int(fwd) or bkw != int(bkw) or not 1 <= fwd+bkw <= 8:
                return {**result, 'action': 'skip', 'reason': 'invalid_directional_lanes'}
            result.update(fwd_lanes=int(fwd), bkw_lanes=int(bkw))
        result['speed_applied'] = False
    return result
