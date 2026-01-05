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

    const url = this.getTileUrl(message);
    if (!url) return null;

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

  private getTileUrl(message: MapTileMessage): string | null {
    if (message.data) {
      const byteCharacters = atob(message.data);
      const byteNumbers = new Array(byteCharacters.length);
      for (let i = 0; i < byteCharacters.length; i++) {
        byteNumbers[i] = byteCharacters.charCodeAt(i);
      }
      const byteArray = new Uint8Array(byteNumbers);
      const blob = new Blob([byteArray], { type: 'image/png' });
      return URL.createObjectURL(blob);
    }

    if (typeof document === 'undefined') {
      return null;
    }

    const pngBytes = this.createPlaceholderTilePng(message.tile_id);
    const blob = new Blob([pngBytes], { type: 'image/png' });
    return URL.createObjectURL(blob);
  }

  private createPlaceholderTilePng(tileId: string): Uint8Array<ArrayBuffer> {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 256;
    const ctx = canvas.getContext('2d');
    if (!ctx) return new Uint8Array(new ArrayBuffer(0));

    ctx.fillStyle = '#1f2937';
    ctx.fillRect(0, 0, 256, 256);

    ctx.strokeStyle = '#6b7280';
    ctx.lineWidth = 2;
    ctx.strokeRect(1, 1, 254, 254);

    ctx.strokeStyle = '#374151';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(256, 256);
    ctx.moveTo(0, 256);
    ctx.lineTo(256, 0);
    ctx.stroke();

    ctx.fillStyle = '#fbbf24';
    ctx.font = '14px monospace';
    ctx.fillText('MapTile', 10, 22);

    ctx.fillStyle = '#e5e7eb';
    ctx.font = '12px monospace';
    const lines = wrapText(tileId, 28);
    lines.slice(0, 4).forEach((line, index) => {
      ctx.fillText(line, 10, 50 + index * 16);
    });

    const dataUrl = canvas.toDataURL('image/png');
    const base64 = dataUrl.split(',')[1] ?? '';
    const byteCharacters = atob(base64);
    const bytes = new Uint8Array(new ArrayBuffer(byteCharacters.length));
    for (let i = 0; i < byteCharacters.length; i++) {
      bytes[i] = byteCharacters.charCodeAt(i);
    }
    return bytes;
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

function wrapText(text: string, maxLen: number): string[] {
  const chunks: string[] = [];
  let start = 0;
  while (start < text.length) {
    chunks.push(text.slice(start, start + maxLen));
    start += maxLen;
  }
  return chunks.length ? chunks : [''];
}
