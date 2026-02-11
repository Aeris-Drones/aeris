# Map tile metadata for caching and selective synchronization of geospatial layers.
# Enables efficient bandwidth usage by tracking tile versions and avoiding redundant transfers.

string tile_id       # Unique identifier in "z/x/y" format for tile addressing
string format        # Image encoding (e.g., "png", "webp") for client-side decoding
string[] layer_ids   # Active map layers this tile contributes to (elevation, imagery, etc.)
string hash_sha256   # Content hash for cache validation and deduplication
uint32 byte_size     # Uncompressed size in bytes for storage planning
