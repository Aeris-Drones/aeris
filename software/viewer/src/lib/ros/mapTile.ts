/**
 * ROS message format for map tile data published by the mapping pipeline.
 *
 * Represents a single tile from a slippy-map tileset, encoded as a base64 PNG
 * or mbtiles format. Tiles are published on the /map_tiles topic and ingested
 * by the 3D scene for terrain visualization.
 *
 * @see MapTileManager for ingestion and caching logic
 */
export interface MapTileMessage {
  /** Tile identifier in "z/x/y" format following slippy-map conventions */
  tile_id: string;
  /** Format identifier: "mbtiles-1.3" or "image/png" */
  format: string;
  /** Layer identifiers included in this tile (e.g., ["satellite", "elevation"]) */
  layer_ids: string[];
  /** SHA-256 hash of the tile data for integrity verification */
  hash_sha256: string;
  /** Uncompressed payload size in bytes */
  byte_size: number;
  /** Base64-encoded tile payload (PNG image data) */
  data?: string;
  /** ROS timestamp when the tile was published */
  published_at?: {
    sec: number;
    nanosec: number;
  };
  /** Optional replay provenance from mesh store-forward transport */
  delivery_mode?: 'live' | 'replayed';
  original_event_ts?: number | { sec: number; nanosec: number };
  replayed_at_ts?: number | { sec: number; nanosec: number } | null;
}

/**
 * Slippy-map tile grid coordinates.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 */
export interface TileCoordinates {
  /** Zoom level (0-22) */
  z: number;
  /** Tile column index (0 to 2^z - 1) */
  x: number;
  /** Tile row index (0 to 2^z - 1) */
  y: number;
}

/**
 * Geographic coordinates in WGS84 datum.
 */
export interface GeoCoordinates {
  /** Latitude in degrees (-90 to 90) */
  lat: number;
  /** Longitude in degrees (-180 to 180) */
  lon: number;
}

/**
 * Local Cartesian coordinates in meters, relative to an origin point.
 *
 * Uses Three.js conventions: X axis points East, Z axis points North.
 * The Y axis (up) is omitted as map tiles lie on the XZ plane.
 */
export interface LocalCoordinates {
  /** East offset in meters from origin */
  x: number;
  /** North offset in meters from origin (negative Z in Three.js) */
  z: number;
}

/**
 * Physical dimensions of a map tile at a specific zoom and latitude.
 */
export interface TileDimensions {
  /** Width in meters */
  width: number;
  /** Height in meters */
  height: number;
}

/** WGS84 equatorial radius in meters (major axis) */
const EARTH_RADIUS = 6378137.0;
/** Earth's circumference at the equator */
const EARTH_CIRCUMFERENCE = 2 * Math.PI * EARTH_RADIUS;

/**
 * Parses a slippy-map tile identifier into coordinate components.
 *
 * Validates the tile ID format (z/x/y) and ensures coordinates are within
 * valid bounds for the specified zoom level.
 *
 * @param tileId - Tile identifier in "z/x/y" format
 * @returns Parsed tile coordinates
 * @throws Error if the format is invalid or coordinates are out of bounds
 */
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

/**
 * Converts tile X coordinate to longitude.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 */
export function tile2lon(x: number, z: number): number {
  return (x / Math.pow(2, z)) * 360 - 180;
}

/**
 * Converts tile Y coordinate to latitude using the Mercator projection.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 */
export function tile2lat(y: number, z: number): number {
  const n = Math.PI - (2 * Math.PI * y) / Math.pow(2, z);
  return (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
}

/**
 * Computes the geographic bounding box of a tile.
 *
 * @returns Bounds in degrees with cardinal direction labels
 */
export function getTileBounds(coords: TileCoordinates): { north: number; south: number; east: number; west: number } {
  const { z, x, y } = coords;
  const n = Math.pow(2, z);

  const west = (x / n) * 360 - 180;
  const east = ((x + 1) / n) * 360 - 180;

  const north = tile2lat(y, z);
  const south = tile2lat(y + 1, z);

  return { north, south, east, west };
}

/**
 * Computes the geographic center of a tile.
 *
 * Used to position tiles in the 3D scene by converting to local coordinates.
 */
export function getTileCenter(coords: TileCoordinates): GeoCoordinates {
  const bounds = getTileBounds(coords);
  return {
    lat: (bounds.north + bounds.south) / 2,
    lon: (bounds.west + bounds.east) / 2,
  };
}

/**
 * Converts WGS84 coordinates to local meters using equirectangular projection.
 *
 * This projection preserves distances well for small areas (typical mission
 * extents) and is computationally efficient. Results are in Three.js
 * coordinates: +X = East, -Z = North.
 *
 * @param geo - Target geographic coordinates
 * @param origin - Reference point that becomes (0, 0) in local space
 * @returns Local coordinates in meters
 */
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
  const z = -(latRad - originLatRad) * EARTH_RADIUS;

  return { x, z };
}

/**
 * Computes the physical dimensions of a tile at a given zoom and latitude.
 *
 * Tile size varies with latitude due to the Mercator projection's distortion.
 * At higher latitudes, tiles are physically smaller in the north-south axis.
 *
 * @param z - Zoom level
 * @param lat - Latitude in degrees
 * @returns Width and height in meters
 */
export function getTileDimensions(z: number, lat: number): TileDimensions {
  const latRad = (lat * Math.PI) / 180;
  const resolution = (EARTH_CIRCUMFERENCE * Math.cos(latRad)) / Math.pow(2, z);

  return {
    width: resolution,
    height: resolution
  };
}
