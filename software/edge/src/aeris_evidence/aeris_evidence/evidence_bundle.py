"""Evidence bundling CLI for Aeris bench runs."""

from __future__ import annotations

import argparse
import json
import sys
import tarfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from hashlib import sha256
from io import BytesIO
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

DEFAULT_WINDOW_SEC = 900
DEFAULT_OUTPUT = "/tmp/aeris_evidence.tgz"
MANIFEST_NAME = "manifest.json"
SIGNATURE_NAME = "manifest.sig"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bundle bench artifacts with SHA-256 manifest")
    parser.add_argument(
        "--input-dir",
        dest="input_dirs",
        action="append",
        required=True,
        help="Directory to scan (can be provided multiple times)",
    )
    parser.add_argument("--out", default=DEFAULT_OUTPUT, help="Output tar.gz (or JSON when --manifest-only)")
    parser.add_argument(
        "--window-sec",
        type=int,
        default=DEFAULT_WINDOW_SEC,
        help="Include files newer than now-window seconds",
    )
    parser.add_argument(
        "--signing-key",
        type=str,
        help="Path to Ed25519 private key (raw 32-byte or hex). Uses PyNaCl if available.",
    )
    parser.add_argument(
        "--manifest-only",
        action="store_true",
        help="Write manifest(+sig) without tarring the referenced files",
    )
    return parser.parse_args(argv)


@dataclass
class FileEntry:
    path: Path
    arcname: str
    size: int
    sha256: str


def find_files(input_dirs: Sequence[str], window_sec: int) -> List[FileEntry]:
    cutoff = time.time() - window_sec
    entries: List[FileEntry] = []
    for raw_dir in input_dirs:
        root = Path(raw_dir).expanduser().resolve()
        if not root.is_dir():
            raise FileNotFoundError(f"Input dir not found: {root}")
        prefix = f"inputs/{root.name}"
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            try:
                stat = path.stat()
            except OSError as exc:
                print(f"[WARN] Skipping {path}: {exc}", file=sys.stderr)
                continue
            if stat.st_mtime < cutoff:
                continue
            rel = path.relative_to(root)
            arcname = str(Path(prefix) / rel)
            digest = sha256()
            with path.open("rb") as src:
                for chunk in iter(lambda: src.read(1024 * 1024), b""):
                    digest.update(chunk)
            entries.append(FileEntry(path=path, arcname=arcname, size=stat.st_size, sha256=digest.hexdigest()))
    entries.sort(key=lambda e: e.arcname)
    return entries


def load_signer(key_path: str):
    if not key_path:
        return None
    try:
        from nacl.signing import SigningKey
    except Exception as exc:  # pragma: no cover - missing dependency path
        print(f"[WARN] Signing unavailable (PyNaCl import failed: {exc}). Continuing without signature.", file=sys.stderr)
        return None
    data = Path(key_path).expanduser().read_bytes().strip()
    if not data:
        raise ValueError("Signing key file is empty")
    if all(c in b"0123456789abcdefABCDEF" for c in data.strip()) and len(data.strip()) in {64, 128}:
        data_bytes = bytes.fromhex(data.decode())
    else:
        data_bytes = data
    if len(data_bytes) != 32:
        raise ValueError("Ed25519 keys must be 32 raw bytes (or 64 hex chars)")
    return SigningKey(data_bytes)


def build_manifest(entries: Sequence[FileEntry], window_sec: int) -> bytes:
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "window_sec": window_sec,
        "file_count": len(entries),
        "total_bytes": sum(e.size for e in entries),
        "files": [
            {
                "path": str(e.path),
                "arcname": e.arcname,
                "size_bytes": e.size,
                "sha256": e.sha256,
            }
            for e in entries
        ],
    }
    return json.dumps(manifest, indent=2, sort_keys=True).encode("utf-8")


def maybe_write_signature(signing_key, manifest_bytes: bytes) -> Optional[bytes]:
    if not signing_key:
        return None
    signature = signing_key.sign(manifest_bytes).signature
    return signature


def write_manifest_only(out_path: Path, manifest_bytes: bytes, signature: Optional[bytes]):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(manifest_bytes)
    if signature:
        sig_path = out_path.with_suffix(out_path.suffix + ".sig") if out_path.suffix else out_path.with_name(out_path.name + ".sig")
        sig_path.write_bytes(signature)
        print(f"Signature written: {sig_path}")


def write_tarball(out_path: Path, manifest_bytes: bytes, signature: Optional[bytes], entries: Sequence[FileEntry]):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(out_path, "w:gz") as tf:
        manifest_info = tarfile.TarInfo(MANIFEST_NAME)
        manifest_info.size = len(manifest_bytes)
        tf.addfile(manifest_info, BytesIO(manifest_bytes))
        if signature:
            sig_info = tarfile.TarInfo(SIGNATURE_NAME)
            sig_info.size = len(signature)
            tf.addfile(sig_info, BytesIO(signature))
        for entry in entries:
            try:
                tf.add(str(entry.path), arcname=entry.arcname)
            except FileNotFoundError:
                print(f"[WARN] Skipping missing file during tar: {entry.path}", file=sys.stderr)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        entries = find_files(args.input_dirs, args.window_sec)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 2
    if not entries:
        print("No files matched the window; nothing to bundle.", file=sys.stderr)
        return 3
    manifest_bytes = build_manifest(entries, args.window_sec)
    signing_key = None
    if args.signing_key:
        try:
            signing_key = load_signer(args.signing_key)
        except Exception as exc:
            print(f"[WARN] Signing skipped ({exc}).", file=sys.stderr)
            signing_key = None
    signature = maybe_write_signature(signing_key, manifest_bytes)
    out_path = Path(args.out).expanduser().resolve()
    if args.manifest_only:
        write_manifest_only(out_path, manifest_bytes, signature)
    else:
        write_tarball(out_path, manifest_bytes, signature, entries)
    total_bytes = sum(e.size for e in entries)
    print(f"Bundle written: {out_path} files: {len(entries)} bytes: {total_bytes}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
