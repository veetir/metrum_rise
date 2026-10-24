// SPDX-License-Identifier: GPL-2.0-only

//! The two save histories. Road v88–95 extends the pre-SIM-02 v86 economy;
//! upstream v87–107 adds handles, slots, deadlines and rolling runtime state.
//! Version numbers alone cannot distinguish these histories.

use super::{SaveLoadError, SaveLoadResult, sqlite_table_exists};
use rusqlite::Connection;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct SaveFormat {
    pub(super) version: i64,
    pub(super) road: bool,
}

impl SaveFormat {
    pub(super) fn read(conn: &Connection) -> SaveLoadResult<Self> {
        let version = conn.query_row("SELECT version FROM save_meta LIMIT 1", [], |r| r.get(0))?;
        let handles = sqlite_table_exists(conn, "entity_handle_slots")?;
        let road = (88..=95).contains(&version) && !handles;
        if !(version == 86 || road || handles && (87..=super::SAVE_VERSION).contains(&version)) {
            return Err(SaveLoadError::custom(
                "version mismatch or unknown save history",
            ));
        }
        Ok(Self { version, road })
    }

    // Road versions retain v86's economy throughout: their additions are road authority.
    pub(super) fn since(self, upstream: i64) -> bool {
        !self.road && self.version >= upstream
    }

    pub(super) fn weather(self) -> bool {
        self.road || self.since(107)
    }

    // Explicit projections for fields introduced/renamed at a known upstream cutover.
    // Missing fields in a format that *has* them remain SQL errors, never silent defaults.
    pub(super) fn query(self, sql: &str, changes: &[(i64, &str, &str)]) -> String {
        let mut query = sql.to_owned();
        for &(since, column, older) in changes {
            if !self.since(since) {
                query = query.replace(column, older);
            }
        }
        query
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn colliding_versions_are_distinguished_by_the_handle_cutover() {
        for version in 88..=95 {
            for handles in [false, true] {
                let conn = Connection::open_in_memory().unwrap();
                conn.execute_batch("CREATE TABLE save_meta(version INTEGER);")
                    .unwrap();
                conn.execute("INSERT INTO save_meta VALUES (?1)", [version])
                    .unwrap();
                if handles {
                    conn.execute_batch("CREATE TABLE entity_handle_slots(kind INTEGER);")
                        .unwrap();
                }
                let format = SaveFormat::read(&conn).unwrap();
                assert_eq!(format.road, !handles);
                assert_eq!(format.since(88), handles);
                assert_eq!(format.weather(), !handles);
            }
        }
    }
}
