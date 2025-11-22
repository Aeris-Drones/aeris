import { GeoCoordinates, geoToLocal } from '../ros/mapTile';

// Re-export types and functions from ros/mapTile for convenience
// and potentially add more specific map logic here if needed
export { geoToLocal };

// Convenience wrapper matching the signature used in MiniMap
export function latLonToMeters(lat: number, lon: number, origin: GeoCoordinates): { x: number, y: number } {
    const local = geoToLocal({ lat, lon }, origin);
    // geoToLocal returns { x, z } where z is North (negative)
    // For 2D MiniMap, we often want X=East, Y=North.
    // So we return { x: local.x, y: -local.z } (negating z to get positive North)
    return { x: local.x, y: -local.z };
}
