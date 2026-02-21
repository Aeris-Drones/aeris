"""Durable SQLite-backed queue for store-and-forward buffering."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence
import sqlite3
import threading
import time

_SQLITE_IN_CLAUSE_CHUNK_SIZE = 900


@dataclass(frozen=True)
class EnqueueResult:
    """Outcome returned for queue insert attempts."""

    accepted: bool
    ingest_seq: int | None
    reason: str


@dataclass(frozen=True)
class QueueRecord:
    """Single queued outbound message with replay metadata."""

    ingest_seq: int
    topic: str
    route_key: str
    message_kind: str
    event_ts: float
    dedupe_key: str
    payload: bytes
    payload_hash: str
    payload_size: int
    enqueued_monotonic: float
    priority_class: str
    source_vehicle_id: str
    relay_vehicle_id: str
    relay_hop: int
    relay_delivery_mode: str


@dataclass(frozen=True)
class StoreMetrics:
    """Operational counters for queue health and capacity."""

    queued_count: int
    bytes_on_disk: int
    evicted_count: int
    dedupe_hits: int
    oldest_buffered_age_sec: float
    queued_count_by_class: dict[str, int]
    queued_count_by_delivery_mode: dict[str, int]


class StoreForwardStore:
    """SQLite-backed queue with ordered replay, dedupe, and size-bound eviction."""

    def __init__(self, db_path: str, max_bytes: int) -> None:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be > 0")

        self._db_path = Path(db_path)
        self._max_bytes = int(max_bytes)
        self._lock = threading.RLock()

        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._db_path.parent.chmod(0o700)
        except OSError:
            # Directory permission tightening is best-effort in heterogeneous envs.
            pass

        self._conn = sqlite3.connect(
            str(self._db_path),
            isolation_level=None,
            check_same_thread=False,
        )
        self._conn.row_factory = sqlite3.Row
        self._initialize_schema()

        try:
            self._db_path.chmod(0o600)
        except OSError:
            # File permission tightening is best-effort in heterogeneous envs.
            pass

    def _initialize_schema(self) -> None:
        with self._lock:
            self._conn.execute("PRAGMA journal_mode=WAL;")
            self._conn.execute("PRAGMA synchronous=FULL;")
            self._conn.execute("PRAGMA temp_store=MEMORY;")
            self._conn.execute("PRAGMA wal_autocheckpoint=1000;")

            self._conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS queue (
                  ingest_seq INTEGER PRIMARY KEY AUTOINCREMENT,
                  topic TEXT NOT NULL,
                  route_key TEXT NOT NULL,
                  message_kind TEXT NOT NULL,
                  event_ts REAL NOT NULL,
                  dedupe_key TEXT NOT NULL,
                  payload_bytes BLOB NOT NULL,
                  payload_hash TEXT NOT NULL,
                  payload_size INTEGER NOT NULL,
                  enqueued_mono REAL NOT NULL,
                  source_vehicle_id TEXT NOT NULL DEFAULT '',
                  relay_vehicle_id TEXT NOT NULL DEFAULT '',
                  relay_hop INTEGER NOT NULL DEFAULT 0,
                  relay_delivery_mode TEXT NOT NULL DEFAULT 'direct'
                );

                CREATE INDEX IF NOT EXISTS idx_queue_ingest_seq
                ON queue(ingest_seq);

                CREATE INDEX IF NOT EXISTS idx_queue_dedupe_key
                ON queue(dedupe_key);

                CREATE TABLE IF NOT EXISTS dedupe_index (
                  dedupe_key TEXT PRIMARY KEY,
                  payload_hash TEXT NOT NULL,
                  last_ingest_seq INTEGER NOT NULL,
                  last_event_ts REAL NOT NULL
                );

                CREATE TABLE IF NOT EXISTS stats (
                  key TEXT PRIMARY KEY,
                  value INTEGER NOT NULL
                );
                """
            )
            self._ensure_queue_column("source_vehicle_id", "TEXT NOT NULL DEFAULT ''")
            self._ensure_queue_column("relay_vehicle_id", "TEXT NOT NULL DEFAULT ''")
            self._ensure_queue_column("relay_hop", "INTEGER NOT NULL DEFAULT 0")
            self._ensure_queue_column(
                "relay_delivery_mode", "TEXT NOT NULL DEFAULT 'direct'"
            )
            self._conn.execute(
                "INSERT OR IGNORE INTO stats(key, value) VALUES ('dedupe_hits', 0);"
            )
            self._conn.execute(
                "INSERT OR IGNORE INTO stats(key, value) VALUES ('evicted_count', 0);"
            )

    def set_max_bytes(self, max_bytes: int) -> None:
        """Adjust the configured size limit and evict if needed."""
        if max_bytes <= 0:
            raise ValueError("max_bytes must be > 0")
        with self._lock:
            self._max_bytes = int(max_bytes)
            self._conn.execute("BEGIN IMMEDIATE;")
            try:
                self._evict_to_capacity(self._conn.cursor())
                self._conn.execute("COMMIT;")
            except Exception:
                self._conn.execute("ROLLBACK;")
                raise

    def _ensure_queue_column(self, column_name: str, column_ddl: str) -> None:
        row = self._conn.execute(
            """
            SELECT 1
            FROM pragma_table_info('queue')
            WHERE name = ?
            LIMIT 1
            """,
            (column_name,),
        ).fetchone()
        if row is not None:
            return
        self._conn.execute(f"ALTER TABLE queue ADD COLUMN {column_name} {column_ddl}")

    def enqueue(
        self,
        *,
        topic: str,
        route_key: str = "",
        message_kind: str = "",
        event_ts: float,
        dedupe_key: str,
        payload: bytes,
        payload_hash: str,
        source_vehicle_id: str = "",
        relay_vehicle_id: str = "",
        relay_hop: int = 0,
        relay_delivery_mode: str = "direct",
    ) -> EnqueueResult:
        """Queue an outbound message unless dedupe metadata marks it duplicate."""
        with self._lock:
            cur = self._conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                duplicate = cur.execute(
                    """
                    SELECT payload_hash
                    FROM dedupe_index
                    WHERE dedupe_key = ?
                    """,
                    (dedupe_key,),
                ).fetchone()
                if duplicate is not None and duplicate["payload_hash"] == payload_hash:
                    self._increment_stat(cur, "dedupe_hits", 1)
                    cur.execute("COMMIT;")
                    return EnqueueResult(accepted=False, ingest_seq=None, reason="deduplicated")

                now_mono = time.monotonic()
                normalized_relay_mode = relay_delivery_mode.strip().lower()
                if normalized_relay_mode not in {"direct", "relay"}:
                    normalized_relay_mode = "direct"
                cur.execute(
                    """
                    INSERT INTO queue(
                      topic, route_key, message_kind, event_ts, dedupe_key,
                      payload_bytes, payload_hash, payload_size, enqueued_mono,
                      source_vehicle_id, relay_vehicle_id, relay_hop, relay_delivery_mode
                    ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        topic,
                        route_key,
                        message_kind,
                        float(event_ts),
                        dedupe_key,
                        payload,
                        payload_hash,
                        len(payload),
                        now_mono,
                        source_vehicle_id,
                        relay_vehicle_id,
                        int(relay_hop),
                        normalized_relay_mode,
                    ),
                )
                ingest_seq = int(cur.lastrowid)

                cur.execute(
                    """
                    INSERT INTO dedupe_index(dedupe_key, payload_hash, last_ingest_seq, last_event_ts)
                    VALUES(?, ?, ?, ?)
                    ON CONFLICT(dedupe_key)
                    DO UPDATE SET
                      payload_hash = excluded.payload_hash,
                      last_ingest_seq = excluded.last_ingest_seq,
                      last_event_ts = excluded.last_event_ts
                    """,
                    (dedupe_key, payload_hash, ingest_seq, float(event_ts)),
                )

                self._evict_to_capacity(cur)
                survived = cur.execute(
                    """
                    SELECT 1
                    FROM queue
                    WHERE ingest_seq = ?
                    LIMIT 1
                    """,
                    (ingest_seq,),
                ).fetchone()
                cur.execute("COMMIT;")
                if survived is None:
                    return EnqueueResult(
                        accepted=False,
                        ingest_seq=None,
                        reason="dropped-over-capacity",
                    )
                return EnqueueResult(accepted=True, ingest_seq=ingest_seq, reason="buffered")
            except Exception:
                cur.execute("ROLLBACK;")
                raise

    def peek_pending(self, *, limit: int = 256) -> list[QueueRecord]:
        """Return pending records in strict ingest order."""
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT ingest_seq, topic, route_key, message_kind, event_ts, dedupe_key,
                       payload_bytes, payload_hash, payload_size, enqueued_mono,
                       source_vehicle_id, relay_vehicle_id, relay_hop, relay_delivery_mode
                FROM queue
                ORDER BY ingest_seq ASC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()

        return [
            QueueRecord(
                ingest_seq=int(row["ingest_seq"]),
                topic=str(row["topic"]),
                route_key=str(row["route_key"]),
                message_kind=str(row["message_kind"]),
                event_ts=float(row["event_ts"]),
                dedupe_key=str(row["dedupe_key"]),
                payload=bytes(row["payload_bytes"]),
                payload_hash=str(row["payload_hash"]),
                payload_size=int(row["payload_size"]),
                enqueued_monotonic=float(row["enqueued_mono"]),
                priority_class=self.classify_priority(
                    route_key=str(row["route_key"]),
                    message_kind=str(row["message_kind"]),
                ),
                source_vehicle_id=str(row["source_vehicle_id"]),
                relay_vehicle_id=str(row["relay_vehicle_id"]),
                relay_hop=int(row["relay_hop"]),
                relay_delivery_mode=str(row["relay_delivery_mode"]),
            )
            for row in rows
        ]

    def peek_pending_prioritized(self, *, limit: int = 256) -> list[QueueRecord]:
        """Return pending records ordered by priority then ingest order."""
        with self._lock:
            rows = self._conn.execute(
                f"""
                SELECT ingest_seq, topic, route_key, message_kind, event_ts, dedupe_key,
                       payload_bytes, payload_hash, payload_size, enqueued_mono,
                       source_vehicle_id, relay_vehicle_id, relay_hop, relay_delivery_mode,
                       {self._priority_case_sql()} AS priority_class,
                       {self._priority_rank_sql()} AS priority_rank
                FROM queue
                ORDER BY priority_rank ASC, ingest_seq ASC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()

        return [
            QueueRecord(
                ingest_seq=int(row["ingest_seq"]),
                topic=str(row["topic"]),
                route_key=str(row["route_key"]),
                message_kind=str(row["message_kind"]),
                event_ts=float(row["event_ts"]),
                dedupe_key=str(row["dedupe_key"]),
                payload=bytes(row["payload_bytes"]),
                payload_hash=str(row["payload_hash"]),
                payload_size=int(row["payload_size"]),
                enqueued_monotonic=float(row["enqueued_mono"]),
                priority_class=str(row["priority_class"]),
                source_vehicle_id=str(row["source_vehicle_id"]),
                relay_vehicle_id=str(row["relay_vehicle_id"]),
                relay_hop=int(row["relay_hop"]),
                relay_delivery_mode=str(row["relay_delivery_mode"]),
            )
            for row in rows
        ]

    def ack_through(self, ingest_seq: int) -> None:
        """Delete all records with sequence <= ingest_seq after successful replay."""
        with self._lock:
            seq = int(ingest_seq)
            cur = self._conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                dedupe_rows = cur.execute(
                    "SELECT DISTINCT dedupe_key FROM queue WHERE ingest_seq <= ?",
                    (seq,),
                ).fetchall()
                affected_dedupe_keys = {
                    str(row["dedupe_key"]) for row in dedupe_rows if row["dedupe_key"] is not None
                }

                cur.execute("DELETE FROM queue WHERE ingest_seq <= ?", (seq,))
                self._refresh_dedupe_for_keys(cur, affected_dedupe_keys)
                cur.execute("COMMIT;")
            except Exception:
                cur.execute("ROLLBACK;")
                raise

    def ack_records(self, ingest_seqs: Sequence[int]) -> int:
        """Delete specific records that have been replayed and committed."""
        normalized = sorted({int(seq) for seq in ingest_seqs if int(seq) > 0})
        if not normalized:
            return 0

        with self._lock:
            cur = self._conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                affected_dedupe_keys: set[str] = set()
                deleted = 0
                for start in range(0, len(normalized), _SQLITE_IN_CLAUSE_CHUNK_SIZE):
                    chunk = normalized[start : start + _SQLITE_IN_CLAUSE_CHUNK_SIZE]
                    placeholders = ",".join("?" for _ in chunk)
                    dedupe_rows = cur.execute(
                        f"""
                        SELECT DISTINCT dedupe_key
                        FROM queue
                        WHERE ingest_seq IN ({placeholders})
                        """,
                        tuple(chunk),
                    ).fetchall()
                    affected_dedupe_keys.update(
                        str(row["dedupe_key"])
                        for row in dedupe_rows
                        if row["dedupe_key"] is not None
                    )

                    cur.execute(
                        f"""
                        DELETE FROM queue
                        WHERE ingest_seq IN ({placeholders})
                        """,
                        tuple(chunk),
                    )
                    deleted += int(cur.rowcount)
                self._refresh_dedupe_for_keys(cur, affected_dedupe_keys)
                cur.execute("COMMIT;")
                return deleted
            except Exception:
                cur.execute("ROLLBACK;")
                raise

    def pending_count(self) -> int:
        with self._lock:
            row = self._conn.execute("SELECT COUNT(*) AS c FROM queue").fetchone()
            return int(row["c"]) if row is not None else 0

    def has_pending(self) -> bool:
        """Fast path for checking whether any buffered records exist."""
        with self._lock:
            row = self._conn.execute(
                "SELECT 1 FROM queue LIMIT 1"
            ).fetchone()
            return row is not None

    def metrics(self, *, now_wallclock: float | None = None) -> StoreMetrics:
        """Compute queue size and dedupe/eviction counters."""
        now = time.time() if now_wallclock is None else float(now_wallclock)
        with self._lock:
            row = self._conn.execute(
                """
                SELECT
                  COUNT(*) AS queued_count,
                  COALESCE(SUM(payload_size), 0) AS bytes_on_disk,
                  MIN(event_ts) AS oldest_event_ts
                FROM queue
                """
            ).fetchone()
            dedupe_hits = self._read_stat("dedupe_hits")
            evicted_count = self._read_stat("evicted_count")
            class_rows = self._conn.execute(
                f"""
                SELECT {self._priority_case_sql()} AS priority_class, COUNT(*) AS c
                FROM queue
                GROUP BY priority_class
                """
            ).fetchall()
            mode_rows = self._conn.execute(
                """
                SELECT lower(relay_delivery_mode) AS relay_delivery_mode, COUNT(*) AS c
                FROM queue
                GROUP BY lower(relay_delivery_mode)
                """
            ).fetchall()

        queued_count = int(row["queued_count"]) if row is not None else 0
        bytes_on_disk = int(row["bytes_on_disk"]) if row is not None else 0
        oldest_event_ts = row["oldest_event_ts"] if row is not None else None
        if oldest_event_ts is None:
            oldest_age = 0.0
        else:
            oldest_age = max(now - float(oldest_event_ts), 0.0)

        queued_count_by_class: dict[str, int] = {
            "control": 0,
            "telemetry": 0,
            "tiles": 0,
            "bulk": 0,
        }
        for row in class_rows:
            queued_count_by_class[str(row["priority_class"])] = int(row["c"])

        queued_count_by_delivery_mode: dict[str, int] = {"direct": 0, "relay": 0}
        for row in mode_rows:
            mode = str(row["relay_delivery_mode"])
            if mode not in queued_count_by_delivery_mode:
                queued_count_by_delivery_mode[mode] = 0
            queued_count_by_delivery_mode[mode] = int(row["c"])

        return StoreMetrics(
            queued_count=queued_count,
            bytes_on_disk=bytes_on_disk,
            evicted_count=evicted_count,
            dedupe_hits=dedupe_hits,
            oldest_buffered_age_sec=oldest_age,
            queued_count_by_class=queued_count_by_class,
            queued_count_by_delivery_mode=queued_count_by_delivery_mode,
        )

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def _evict_to_capacity(self, cur: sqlite3.Cursor) -> None:
        row = cur.execute(
            "SELECT COALESCE(SUM(payload_size), 0) AS total_bytes FROM queue"
        ).fetchone()
        total_bytes = int(row["total_bytes"]) if row is not None else 0

        affected_dedupe_keys: set[str] = set()
        while total_bytes > self._max_bytes:
            victim = cur.execute(
                f"""
                SELECT ingest_seq, payload_size, dedupe_key
                FROM queue
                ORDER BY {self._priority_rank_sql()} DESC, ingest_seq ASC
                LIMIT 1
                """
            ).fetchone()
            if victim is None:
                break

            victim_seq = int(victim["ingest_seq"])
            affected_dedupe_keys.add(str(victim["dedupe_key"]))
            cur.execute("DELETE FROM queue WHERE ingest_seq = ?", (victim_seq,))
            total_bytes -= int(victim["payload_size"])
            self._increment_stat(cur, "evicted_count", 1)
        self._refresh_dedupe_for_keys(cur, affected_dedupe_keys)

    def _refresh_dedupe_for_keys(
        self, cur: sqlite3.Cursor, dedupe_keys: set[str]
    ) -> None:
        """Rebuild dedupe_index rows for keys affected by replay ack/eviction."""
        if not dedupe_keys:
            return

        for dedupe_key in dedupe_keys:
            latest = cur.execute(
                """
                SELECT payload_hash, ingest_seq, event_ts
                FROM queue
                WHERE dedupe_key = ?
                ORDER BY ingest_seq DESC
                LIMIT 1
                """,
                (dedupe_key,),
            ).fetchone()
            if latest is None:
                cur.execute("DELETE FROM dedupe_index WHERE dedupe_key = ?", (dedupe_key,))
                continue

            cur.execute(
                """
                INSERT INTO dedupe_index(dedupe_key, payload_hash, last_ingest_seq, last_event_ts)
                VALUES(?, ?, ?, ?)
                ON CONFLICT(dedupe_key)
                DO UPDATE SET
                  payload_hash = excluded.payload_hash,
                  last_ingest_seq = excluded.last_ingest_seq,
                  last_event_ts = excluded.last_event_ts
                """,
                (
                    dedupe_key,
                    str(latest["payload_hash"]),
                    int(latest["ingest_seq"]),
                    float(latest["event_ts"]),
                ),
            )

    def _increment_stat(self, cur: sqlite3.Cursor, key: str, delta: int) -> None:
        cur.execute(
            """
            INSERT INTO stats(key, value) VALUES(?, ?)
            ON CONFLICT(key) DO UPDATE SET value = value + excluded.value
            """,
            (key, int(delta)),
        )

    def _read_stat(self, key: str) -> int:
        row = self._conn.execute("SELECT value FROM stats WHERE key = ?", (key,)).fetchone()
        if row is None:
            return 0
        return int(row["value"])

    @staticmethod
    def classify_priority(*, route_key: str, message_kind: str) -> str:
        """Classify route/message into replay priority classes."""
        route = route_key.strip().lower().removeprefix("relay_")
        kind = message_kind.strip().lower()
        if route == "heartbeat" or kind in {"heartbeat", "control", "command"}:
            return "control"
        if route == "telemetry" or kind == "telemetry":
            return "telemetry"
        if route == "map_tile" or kind == "map_tile":
            return "tiles"
        return "bulk"

    @staticmethod
    def _priority_case_sql() -> str:
        return (
            "CASE "
            "WHEN lower(route_key) IN ('heartbeat', 'relay_heartbeat') OR lower(message_kind) IN ('heartbeat', 'control', 'command') "
            "THEN 'control' "
            "WHEN lower(route_key) IN ('telemetry', 'relay_telemetry') OR lower(message_kind) = 'telemetry' "
            "THEN 'telemetry' "
            "WHEN lower(route_key) IN ('map_tile', 'relay_map_tile') OR lower(message_kind) = 'map_tile' "
            "THEN 'tiles' "
            "ELSE 'bulk' "
            "END"
        )

    @staticmethod
    def _priority_rank_sql() -> str:
        return (
            "CASE "
            "WHEN lower(route_key) IN ('heartbeat', 'relay_heartbeat') OR lower(message_kind) IN ('heartbeat', 'control', 'command') "
            "THEN 0 "
            "WHEN lower(route_key) IN ('telemetry', 'relay_telemetry') OR lower(message_kind) = 'telemetry' "
            "THEN 1 "
            "WHEN lower(route_key) IN ('map_tile', 'relay_map_tile') OR lower(message_kind) = 'map_tile' "
            "THEN 2 "
            "ELSE 3 "
            "END"
        )
