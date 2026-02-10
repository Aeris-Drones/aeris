export interface MapTileMessage {
  tile_id: string; // "z/x/y"
  format: string;  // "mbtiles-1.3" or "image/png"
  layer_ids: string[];
  hash_sha256: string;
  byte_size: number;
  data?: string; // Base64 encoded payload
  published_at?: {
    sec: number;
    nanosec: number;
  };
}

export interface TileCoordinates {
  z: number;
  x: number;
  y: number;
}

export interface GeoCoordinates {
  lat: number;
  lon: number;
}

export interface LocalCoordinates {
  x: number; // East in meters
  z: number; // North in meters (Three.js usually uses Y up, so map lies on XZ plane)
}

export interface TileDimensions {
  width: number;
  height: number;
}

// Earth constants
const EARTH_RADIUS = 6378137.0; // Meters (WGS84 Major axis)
const EARTH_CIRCUMFERENCE = 2 * Math.PI * EARTH_RADIUS;

export function parseTileId(tileId: string): TileCoordinates {
  const parts = tileId.split('/');
  if (parts.length !== 3) {
    throw new Error(`Invalid tile ID format: ${tileId}. Expected z/x/y`);
  }
  if (!parts.every((part) => /^\d+$/.test(part))) {
    throw new Error(`Invalid tile ID format: ${tileId}. Expected non-negative integers z/x/y`);
  }
  const z = parseInt(parts[0], 10);
  const x = parseInt(parts[1], 10);
  const y = parseInt(parts[2], 10);
  if (!Number.isInteger(z) || !Number.isInteger(x) || !Number.isInteger(y) || z < 0 || x < 0 || y < 0) {
    throw new Error(`Invalid tile ID format: ${tileId}. Expected non-negative integers z/x/y`);
  }
  if (z > 22) {
    throw new Error(`Invalid tile ID format: ${tileId}. Zoom level must be <= 22`);
  }
  const maxIndex = Math.pow(2, z) - 1;
  if (x > maxIndex || y > maxIndex) {
    throw new Error(`Invalid tile ID format: ${tileId}. Tile indices out of bounds for zoom ${z}`);
  }
  return {
    z,
    x,
    y,
  };
}

export function tile2lon(x: number, z: number): number {
  return (x / Math.pow(2, z)) * 360 - 180;
}

export function tile2lat(y: number, z: number): number {
  const n = Math.PI - (2 * Math.PI * y) / Math.pow(2, z);
  return (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
}

export function getTileBounds(coords: TileCoordinates): { north: number; south: number; east: number; west: number } {
  const { z, x, y } = coords;
  const n = Math.pow(2, z);

  const west = (x / n) * 360 - 180;
  const east = ((x + 1) / n) * 360 - 180;

  const north = tile2lat(y, z);
  const south = tile2lat(y + 1, z);

  return { north, south, east, west };
}

export function getTileCenter(coords: TileCoordinates): GeoCoordinates {
  const bounds = getTileBounds(coords);
  return {
    lat: (bounds.north + bounds.south) / 2,
    lon: (bounds.west + bounds.east) / 2,
  };
}

// Convert Lat/Lon to local meters using equirectangular projection
// +X = East, -Z = North (Three.js right-handed system)
export function geoToLocal(
  geo: GeoCoordinates,
  origin: GeoCoordinates
): LocalCoordinates {
  const latRad = (geo.lat * Math.PI) / 180;
  const lonRad = (geo.lon * Math.PI) / 180;
  const originLatRad = (origin.lat * Math.PI) / 180;
  const originLonRad = (origin.lon * Math.PI) / 180;

  const cosLat = Math.cos(originLatRad);

  const x = (lonRad - originLonRad) * EARTH_RADIUS * cosLat;
  const z = -(latRad - originLatRad) * EARTH_RADIUS; // -Z for North

  return { x, z };
}

export function getTileDimensions(z: number, lat: number): TileDimensions {
  const latRad = (lat * Math.PI) / 180;
  const resolution = (EARTH_CIRCUMFERENCE * Math.cos(latRad)) / Math.pow(2, z);

  return {
    width: resolution,
    height: resolution
  };
}
