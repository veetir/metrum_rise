# SPDX-License-Identifier: GPL-2.0-only

"""Source-independent exchange records. Coordinates are lon/lat before normalise.

``properties`` holds neutral semantics; ``source_tags`` is opaque provenance,
never interpreted by downstream stages. Relation members retain missing/outside
members explicitly instead of inventing geometry. Multiple roles are allowed
(e.g. a building with a commercial land-use tag).
"""

from dataclasses import asdict, dataclass, field
from typing import Protocol


@dataclass
class SourceFeature:
    id: str
    roles: list[str]
    points: list[tuple[float, float]]
    node_ids: list[str]
    properties: dict
    source_tags: dict
    members: list[dict] = field(default_factory=list)
    rings: list[dict] = field(default_factory=list)
    issues: list[str] = field(default_factory=list)

    def record(self):
        return asdict(self)


class SourceAdapter(Protocol):
    """Adapters yield stable IDs; normalise owns coordinate transformation."""

    def read(self, path) -> list[SourceFeature]: ...
