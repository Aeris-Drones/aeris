import { GeoCoordinates, geoToLocal } from '../ros/mapTile';

export { geoToLocal };

/**
 * Converts geographic coordinates (latitude, longitude) to 2D local meters.
 *
 * This is a convenience wrapper around geoToLocal that maps the 3D local
 * coordinates to a 2D plane for 2D map rendering.
 *
 * Coordinate Mapping:
 * - X axis: East direction (local x from geoToLocal)
 * - Y axis: North direction (negated local z from geoToLocal)
 *
 * The negation of z is required because geoToLocal returns z as negative
 * North (Three.js convention), but 2D maps typically expect positive North.
 *
 * @param lat - Latitude in degrees (WGS84)
 * @param lon - Longitude in degrees (WGS84)
 * @param origin - Reference point for local coordinate system
 * @returns 2D coordinates in meters from origin
 */
export function latLonToMeters(lat: number, lon: number, origin: GeoCoordinates): { x: number, y: number } {
    const local = geoToLocal({ lat, lon }, origin);
    // Convert to 2D: X=East, Y=North (negate z for positive North)
    return { x: local.x, y: -local.z };
}
