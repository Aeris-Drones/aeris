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

  constructor(maxTiles: number = 500) {
    this.cache = new Map();
    this.maxTiles = maxTiles;
    this.origin = null;
    this.totalBytes = 0;
  }

  public ingest(message: MapTileMessage, externalOrigin: GeoCoordinates | null): IngestResult | null {
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

    if (!message.data) {
      console.warn(`Tile ${key} has no data payload.`);
      return null;
    }

    const byteCharacters = atob(message.data);
    const byteNumbers = new Array(byteCharacters.length);
    for (let i = 0; i < byteCharacters.length; i++) {
      byteNumbers[i] = byteCharacters.charCodeAt(i);
    }
    const byteArray = new Uint8Array(byteNumbers);
    const blob = new Blob([byteArray], { type: 'image/png' });
    const url = URL.createObjectURL(blob);

    const center = getTileCenter(coords);
    let newOrigin: GeoCoordinates | undefined = undefined;

    let activeOrigin = externalOrigin || this.origin;

    if (!activeOrigin) {
      activeOrigin = center;
      this.origin = center;
      newOrigin = center;
      console.log(`[MapTileManager] Set origin to ${center.lat}, ${center.lon} from tile ${key}`);
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

    if (this.cache.size > this.maxTiles) {
      const oldestKey = this.cache.keys().next().value;
      if (oldestKey) {
        this.removeTile(oldestKey);
      }
    }

    return { tile, newOrigin };
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
    };
  }

  public clear() {
    for (const tile of this.cache.values()) {
      URL.revokeObjectURL(tile.url);
    }
    this.cache.clear();
    this.totalBytes = 0;
    this.origin = null;
  }
}
