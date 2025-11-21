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

  public ingest(message: MapTileMessage): TileData | null {
    // 1. Parse ID
    let coords: TileCoordinates;
    try {
      coords = parseTileId(message.tile_id);
    } catch (e) {
      console.error('Failed to parse tile ID', e);
      return null;
    }

    // 2. Handle Data & Cache Key
    const key = message.tile_id;

    // If update, remove old (LRU refresh happens on set)
    if (this.cache.has(key)) {
      this.removeTile(key);
    }

    if (!message.data) {
      console.warn(`Tile ${key} has no data payload.`);
      return null;
    }

    // 3. Create Texture URL
    // message.data is base64. We need to convert to Blob.
    const byteCharacters = atob(message.data);
    const byteNumbers = new Array(byteCharacters.length);
    for (let i = 0; i < byteCharacters.length; i++) {
      byteNumbers[i] = byteCharacters.charCodeAt(i);
    }
    const byteArray = new Uint8Array(byteNumbers);
    // Guess type from format or default to png/jpeg.
    // Message format says "mbtiles-1.3" or "image/png".
    // Usually standard MBTiles use png or jpg. We can try to detect magic numbers or just default to blob.
    // The browser is usually smart enough with images, or we can set type 'image/png'.
    const blob = new Blob([byteArray], { type: 'image/png' });
    const url = URL.createObjectURL(blob);

    // 4. Determine Position
    const center = getTileCenter(coords);

    if (!this.origin) {
      this.origin = center;
      console.log(`[MapTileManager] Set origin to ${center.lat}, ${center.lon} from tile ${key}`);
    }

    const localPos = geoToLocal(center, this.origin);
    const dimensions = getTileDimensions(coords.z, center.lat);

    // 5. Create TileData
    const tile: TileData = {
      id: key,
      url: url,
      position: [localPos.x, 0, localPos.z], // y=0 for flat map
      size: dimensions.width,
      coordinates: coords,
      timestamp: Date.now(),
      byteSize: message.byte_size,
    };

    // 6. Store and Evict
    this.cache.set(key, tile);
    this.totalBytes += message.byte_size;

    if (this.cache.size > this.maxTiles) {
      // Map iterator yields insertion order. First is oldest.
      const oldestKey = this.cache.keys().next().value;
      if (oldestKey) {
        this.removeTile(oldestKey);
      }
    }

    return tile;
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
