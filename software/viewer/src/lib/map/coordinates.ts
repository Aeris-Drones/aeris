import { GeoCoordinates, geoToLocal } from '../ros/mapTile';

export { geoToLocal };

export function latLonToMeters(lat: number, lon: number, origin: GeoCoordinates): { x: number, y: number } {
    const local = geoToLocal({ lat, lon }, origin);
    // Convert to 2D: X=East, Y=North (negate z for positive North)
    return { x: local.x, y: -local.z };
}
