import { MapTileMessage, parseTileId, getTileCenter, geoToLocal, getTileDimensions, GeoCoordinates, TileCoordinates } from '../ros/mapTile';

/**
 * Processed map tile ready for Three.js rendering.
 *
 * Transforms ROS map tile messages into a format suitable for the 3D scene,
 * including local coordinate positioning and blob URL generation for texture loading.
 */
export interface TileData {
  /** Tile identifier in "z/x/y" format */
  id: string;
  /** Object URL for the tile texture (PNG blob) */
  url: string;
  /** Position in Three.js world coordinates [x, y, z] */
  position: [number, number, number];
  /** Tile width/height in meters (square tiles) */
  size: number;
  /** Zoom and grid coordinates */
  coordinates: TileCoordinates;
  /** Ingestion timestamp for cache management */
  timestamp: number;
  /** Payload size for memory tracking */
  byteSize: number;
}

/**
 * Memory and performance statistics for the tile cache.
 *
 * Used by the map overlay to display cache health and network latency.
 */
export interface MapStats {
  /** Number of tiles currently cached */
  count: number;
  /** Total memory consumption in bytes */
  totalBytes: number;
  /** 95th percentile ingestion latency in milliseconds */
  latencyP95Ms: number | null;
  /** Most recent ingestion latency */
  lastLatencyMs: number | null;
}

/**
 * Result of ingesting a map tile message.
 *
 * Contains the processed tile and optionally signals a new coordinate origin
 * when this is the first tile or the origin has changed.
 */
export interface IngestResult {
  tile: TileData;
  /** New geographic origin if established by this tile */
  newOrigin?: GeoCoordinates;
}

/**
 * Manages the lifecycle of map tiles from ROS ingestion to Three.js rendering.
 *
 * Handles tile caching with LRU eviction, coordinate transformation from WGS84
 * to local meters, and memory management via blob URL lifecycle. Integrates
 * with the ROS map tile topic to build the 3D terrain visualization.
 *
 * The manager maintains a bounded cache (default 500 tiles) to prevent memory
 * exhaustion during long missions with extensive map coverage.
 */
export class MapTileManager {
  private cache: Map<string, TileData>;
  private maxTiles: number;
  private origin: GeoCoordinates | null;
  private totalBytes: number;
  private latencySamplesMs: number[];
  private lastLatencyMs: number | null;

  /**
   * @param maxTiles - Maximum tiles to cache before LRU eviction (default: 500)
   */
  constructor(maxTiles: number = 500) {
    this.cache = new Map();
    this.maxTiles = maxTiles;
    this.origin = null;
    this.totalBytes = 0;
    this.latencySamplesMs = [];
    this.lastLatencyMs = null;
  }

  /**
   * Processes a ROS map tile message into renderable tile data.
   *
   * Validates the message, converts coordinates from WGS84 to local meters,
   * creates a blob URL for the texture, and manages cache eviction. Tracks
   * ingestion latency for performance monitoring.
   *
   * @param message - Raw ROS map tile message
   * @param externalOrigin - Optional external coordinate origin (takes precedence)
   * @param latencyMs - Optional network latency measurement
   * @returns Ingestion result or null if processing failed
   */
  public ingest(
    message: MapTileMessage,
    externalOrigin: GeoCoordinates | null,
    latencyMs?: number
  ): IngestResult | null {
    if (!Number.isFinite(message.byte_size) || message.byte_size < 0) {
      console.error('Ignoring map tile with invalid byte_size', message.byte_size);
      return null;
    }

    let coords: TileCoordinates;
    try {
      coords = parseTileId(message.tile_id);
    } catch (e) {
      console.error('Failed to parse tile ID', e);
      return null;
    }

    const key = message.tile_id;

    if (this.cache.has(key)) {
      this.removeTile(key);
    }

    const url = this.getTileUrl(message);
    if (!url) return null;

    const center = getTileCenter(coords);
    let newOrigin: GeoCoordinates | undefined = undefined;

    let activeOrigin = externalOrigin || this.origin;

    if (!activeOrigin) {
      activeOrigin = center;
      this.origin = center;
      newOrigin = center;
    } else if (!this.origin) {
        this.origin = activeOrigin;
    }

    const localPos = geoToLocal(center, activeOrigin);
    const dimensions = getTileDimensions(coords.z, center.lat);

    const tile: TileData = {
      id: key,
      url: url,
      position: [localPos.x, 0, localPos.z],
      size: dimensions.width,
      coordinates: coords,
      timestamp: Date.now(),
      byteSize: message.byte_size,
    };

    this.cache.set(key, tile);
    this.totalBytes += message.byte_size;
    if (typeof latencyMs === 'number' && Number.isFinite(latencyMs) && latencyMs >= 0) {
      this.latencySamplesMs.push(latencyMs);
      this.lastLatencyMs = latencyMs;
      if (this.latencySamplesMs.length > 512) {
        this.latencySamplesMs.shift();
      }
    }

    if (this.cache.size > this.maxTiles) {
      const oldestKey = this.cache.keys().next().value;
      if (oldestKey) {
        this.removeTile(oldestKey);
      }
    }

    return { tile, newOrigin };
  }

  /**
   * Creates a blob URL from base64-encoded tile data.
   *
   * Decodes the ROS message payload into a PNG blob suitable for Three.js
   * texture loading. Returns null if data is missing or decoding fails.
   */
  private getTileUrl(message: MapTileMessage): string | null {
    if (message.data) {
      try {
        const byteCharacters = atob(message.data);
        const byteNumbers = new Array(byteCharacters.length);
        for (let i = 0; i < byteCharacters.length; i++) {
          byteNumbers[i] = byteCharacters.charCodeAt(i);
        }
        const byteArray = new Uint8Array(byteNumbers);
        const blob = new Blob([byteArray], { type: 'image/png' });
        return URL.createObjectURL(blob);
      } catch (err) {
        console.error('Failed to decode tile payload data', err);
        return null;
      }
    }

    if (typeof document === 'undefined') {
      return null;
    }

    console.warn('Skipping map tile render because payload bytes are missing', message.tile_id);
    return null;
  }

  /**
   * Removes a tile from cache and revokes its blob URL.
   *
   * Critical for preventing memory leaks from orphaned blob URLs.
   */
  private removeTile(key: string) {
    const tile = this.cache.get(key);
    if (tile) {
      URL.revokeObjectURL(tile.url);
      this.totalBytes -= tile.byteSize;
      this.cache.delete(key);
    }
  }

  /**
   * Returns all cached tiles for Three.js scene updates.
   */
  public getTiles(): TileData[] {
    return Array.from(this.cache.values());
  }

  /**
   * Computes current cache statistics for performance monitoring.
   */
  public getStats(): MapStats {
    return {
      count: this.cache.size,
      totalBytes: this.totalBytes,
      latencyP95Ms: percentile(this.latencySamplesMs, 95),
      lastLatencyMs: this.lastLatencyMs,
    };
  }

  /**
   * Clears all tiles and releases memory.
   *
   * Should be called when disconnecting from ROS or switching missions
   * to prevent blob URL accumulation.
   */
  public clear() {
    for (const tile of this.cache.values()) {
      URL.revokeObjectURL(tile.url);
    }
    this.cache.clear();
    this.totalBytes = 0;
    this.origin = null;
    this.latencySamplesMs = [];
    this.lastLatencyMs = null;
  }
}

/**
 * Computes the p-th percentile of a numeric array.
 *
 * Used for latency statistics to understand tile ingestion performance.
 * Returns null for empty arrays.
 */
function percentile(values: number[], p: number): number | null {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const index = Math.ceil((p / 100) * sorted.length) - 1;
  return sorted[Math.max(0, Math.min(index, sorted.length - 1))];
}
