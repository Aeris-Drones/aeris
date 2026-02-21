"""Durable SQLite-backed queue for store-and-forward buffering."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sqlite3
import threading
import time


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


@dataclass(frozen=True)
class StoreMetrics:
    """Operational counters for queue health and capacity."""

    queued_count: int
    bytes_on_disk: int
    evicted_count: int
    dedupe_hits: int
    oldest_buffered_age_sec: float


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
                  enqueued_mono REAL NOT NULL
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
                cur.execute(
                    """
                    INSERT INTO queue(
                      topic, route_key, message_kind, event_ts, dedupe_key,
                      payload_bytes, payload_hash, payload_size, enqueued_mono
                    ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)
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
                cur.execute("COMMIT;")
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
                       payload_bytes, payload_hash, payload_size, enqueued_mono
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
            )
            for row in rows
        ]

    def ack_through(self, ingest_seq: int) -> None:
        """Delete all records with sequence <= ingest_seq after successful replay."""
        with self._lock:
            self._conn.execute(
                "DELETE FROM queue WHERE ingest_seq <= ?",
                (int(ingest_seq),),
            )

    def pending_count(self) -> int:
        with self._lock:
            row = self._conn.execute("SELECT COUNT(*) AS c FROM queue").fetchone()
            return int(row["c"]) if row is not None else 0

    def metrics(self, *, now_monotonic: float | None = None) -> StoreMetrics:
        """Compute queue size and dedupe/eviction counters."""
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        with self._lock:
            row = self._conn.execute(
                """
                SELECT
                  COUNT(*) AS queued_count,
                  COALESCE(SUM(payload_size), 0) AS bytes_on_disk,
                  MIN(enqueued_mono) AS oldest_enqueued_mono
                FROM queue
                """
            ).fetchone()
            dedupe_hits = self._read_stat("dedupe_hits")
            evicted_count = self._read_stat("evicted_count")

        queued_count = int(row["queued_count"]) if row is not None else 0
        bytes_on_disk = int(row["bytes_on_disk"]) if row is not None else 0
        oldest_mono = row["oldest_enqueued_mono"] if row is not None else None
        oldest_age = 0.0 if oldest_mono is None else max(now - float(oldest_mono), 0.0)

        return StoreMetrics(
            queued_count=queued_count,
            bytes_on_disk=bytes_on_disk,
            evicted_count=evicted_count,
            dedupe_hits=dedupe_hits,
            oldest_buffered_age_sec=oldest_age,
        )

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def _evict_to_capacity(self, cur: sqlite3.Cursor) -> None:
        row = cur.execute(
            "SELECT COALESCE(SUM(payload_size), 0) AS total_bytes FROM queue"
        ).fetchone()
        total_bytes = int(row["total_bytes"]) if row is not None else 0

        while total_bytes > self._max_bytes:
            victim = cur.execute(
                """
                SELECT ingest_seq, payload_size
                FROM queue
                ORDER BY ingest_seq ASC
                LIMIT 1
                """
            ).fetchone()
            if victim is None:
                break

            cur.execute("DELETE FROM queue WHERE ingest_seq = ?", (int(victim["ingest_seq"]),))
            total_bytes -= int(victim["payload_size"])
            self._increment_stat(cur, "evicted_count", 1)

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
