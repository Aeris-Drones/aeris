import { GeoCoordinates, geoToLocal } from '../ros/mapTile';

export { geoToLocal };

/**
 * Converts WGS84 coordinates to 2D map coordinates in meters.
 *
 * Bridges the 3D scene coordinate system (Three.js) with 2D map libraries
 * that expect traditional East/North orientations. This is the primary
 * entry point for positioning markers on 2D mission planning maps.
 *
 * Coordinate Mapping:
 * - X axis: East direction (local x from geoToLocal)
 * - Y axis: North direction (negated local z from geoToLocal)
 *
 * The negation of z aligns Three.js conventions (negative Z = North)
 * with standard cartographic conventions (positive Y = North).
 *
 * @param lat - Latitude in degrees (WGS84)
 * @param lon - Longitude in degrees (WGS84)
 * @param origin - Reference point for local coordinate system
 * @returns 2D coordinates in meters from origin
 */
export function latLonToMeters(lat: number, lon: number, origin: GeoCoordinates): { x: number, y: number } {
    const local = geoToLocal({ lat, lon }, origin);
    return { x: local.x, y: -local.z };
}
