import { MapTileMessage, parseTileId, getTileCenter, geoToLocal, getTileDimensions, GeoCoordinates, TileCoordinates } from '../ros/mapTile';

export interface TileData {
  id: string; // "z/x/y"
  url: string; // Blob URL
  position: [number, number, number]; // [x, y, z] in Three.js
  size: number; // Width/Height in meters (assuming square)
  coordinates: TileCoordinates;
  timestamp: number;
  byteSize: number; // Size for stats tracking
}

export interface MapStats {
  count: number;
  totalBytes: number;
  latencyP95Ms: number | null;
  lastLatencyMs: number | null;
}

export interface IngestResult {
  tile: TileData;
  newOrigin?: GeoCoordinates;
}

export class MapTileManager {
  private cache: Map<string, TileData>;
  private maxTiles: number;
  private origin: GeoCoordinates | null;
  private totalBytes: number;
  private latencySamplesMs: number[];
  private lastLatencyMs: number | null;

  constructor(maxTiles: number = 500) {
    this.cache = new Map();
    this.maxTiles = maxTiles;
    this.origin = null;
    this.totalBytes = 0;
    this.latencySamplesMs = [];
    this.lastLatencyMs = null;
  }

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

  private removeTile(key: string) {
    const tile = this.cache.get(key);
    if (tile) {
      URL.revokeObjectURL(tile.url);
      this.totalBytes -= tile.byteSize;
      this.cache.delete(key);
    }
  }

  public getTiles(): TileData[] {
    return Array.from(this.cache.values());
  }

  public getStats(): MapStats {
    return {
      count: this.cache.size,
      totalBytes: this.totalBytes,
      latencyP95Ms: percentile(this.latencySamplesMs, 95),
      lastLatencyMs: this.lastLatencyMs,
    };
  }

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

function percentile(values: number[], p: number): number | null {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const index = Math.ceil((p / 100) * sorted.length) - 1;
  return sorted[Math.max(0, Math.min(index, sorted.length - 1))];
}
