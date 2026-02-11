/**
 * Priority levels for search zones.
 *
 * Lower numbers indicate higher priority. Priority 1 zones
 * are searched first and may receive additional resources.
 */
export type ZonePriority = 1 | 2 | 3;

/**
 * Zone lifecycle status for search coordination.
 *
 * - active: Currently being or scheduled to be searched
 * - completed: Fully searched, marked as done
 * - skipped: Bypassed due to obstacles, weather, or operator decision
 */
export type ZoneStatus = 'active' | 'completed' | 'skipped';

/**
 * 2D point in local search coordinates (x=east, z=north).
 *
 * Y (altitude) is omitted as zones define horizontal search areas.
 */
export interface ZonePoint {
  x: number;
  z: number;
}

/**
 * Defined search area with priority and status tracking.
 *
 * Zones are polygonal regions drawn by operators on the map.
 * completedAt is set when status transitions to 'completed'.
 */
export interface PriorityZone {
  id: string;
  name: string;
  priority: ZonePriority;
  status: ZoneStatus;
  polygon: ZonePoint[];
  createdAt: number;
  completedAt?: number;
  notes?: string;
}

/**
 * Input data for creating a new zone.
 *
 * Name is optional and will be auto-generated if not provided.
 */
export interface ZoneInput {
  name?: string;
  priority: ZonePriority;
  polygon: ZonePoint[];
  notes?: string;
}

/**
 * Generates a unique zone identifier.
 *
 * Format: zone-{timestamp}-{random} for collision-resistant IDs.
 */
export function generateZoneId(): string {
  const timestamp = Date.now().toString(36);
  const random = Math.random().toString(36).substring(2, 6);
  return `zone-${timestamp}-${random}`;
}

/**
 * Returns CSS variable name for zone priority color.
 *
 * Maps to design token variables for theming consistency.
 */
export function getPriorityColor(priority: ZonePriority): string {
  switch (priority) {
    case 1:
      return 'var(--priority-1)';
    case 2:
      return 'var(--priority-2)';
    case 3:
      return 'var(--priority-3)';
  }
}

/**
 * Returns hex color value for zone priority.
 *
 * Used for Three.js rendering and canvas drawing where
 * CSS variables are not available.
 */
export function getPrioritySolidColor(priority: ZonePriority): string {
  switch (priority) {
    case 1:
      return '#ef4444';
    case 2:
      return '#f97316';
    case 3:
      return '#eab308';
  }
}

/**
 * Returns human-readable label for zone priority.
 */
export function getPriorityLabel(priority: ZonePriority): string {
  switch (priority) {
    case 1:
      return 'Critical';
    case 2:
      return 'High';
    case 3:
      return 'Elevated';
  }
}

/**
 * Returns complete UI styling configuration for a zone priority.
 *
 * Provides Tailwind-compatible classes for consistent visual
 * presentation across map overlays and list items.
 */
export function getPriorityConfig(priority: ZonePriority): {
  label: string;
  color: string;
  bgColor: string;
  borderColor: string;
} {
  switch (priority) {
    case 1:
      return {
        label: 'Critical',
        color: 'text-danger',
        bgColor: 'bg-danger/10',
        borderColor: 'border-danger/30',
      };
    case 2:
      return {
        label: 'High',
        color: 'text-warning',
        bgColor: 'bg-warning/10',
        borderColor: 'border-warning/30',
      };
    case 3:
      return {
        label: 'Elevated',
        color: 'text-yellow-500',
        bgColor: 'bg-yellow-500/10',
        borderColor: 'border-yellow-500/30',
      };
  }
}

/**
 * Returns UI styling configuration for zone status.
 */
export function getZoneStatusConfig(status: ZoneStatus): {
  label: string;
  color: string;
  bgColor: string;
} {
  switch (status) {
    case 'active':
      return {
        label: 'Active',
        color: 'text-info',
        bgColor: 'bg-info/10',
      };
    case 'completed':
      return {
        label: 'Completed',
        color: 'text-success',
        bgColor: 'bg-success/10',
      };
    case 'skipped':
      return {
        label: 'Skipped',
        color: 'text-muted-foreground',
        bgColor: 'bg-surface-3',
      };
  }
}

/**
 * Calculate polygon area using the shoelace formula.
 *
 * Returns area in square units of the coordinate system.
 * Returns 0 for degenerate polygons (less than 3 points).
 */
export function calculateZoneArea(polygon: ZonePoint[]): number {
  if (polygon.length < 3) return 0;

  let area = 0;
  const n = polygon.length;

  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    area += polygon[i].x * polygon[j].z;
    area -= polygon[j].x * polygon[i].z;
  }

  return Math.abs(area) / 2;
}

/**
 * Calculate centroid (geometric center) of a polygon.
 *
 * Uses simple averaging of vertices. For complex polygons,
 * this approximates the visual center suitable for map labeling.
 */
export function calculateZoneCentroid(polygon: ZonePoint[]): ZonePoint {
  if (polygon.length === 0) return { x: 0, z: 0 };

  let sumX = 0;
  let sumZ = 0;

  for (const point of polygon) {
    sumX += point.x;
    sumZ += point.z;
  }

  return {
    x: sumX / polygon.length,
    z: sumZ / polygon.length,
  };
}

/**
 * Ray-casting algorithm for point-in-polygon test.
 *
 * Determines if a point lies inside a polygon by counting
 * intersections with a horizontal ray from the point.
 *
 * Returns false for degenerate polygons (less than 3 points).
 */
export function isPointInZone(point: ZonePoint, polygon: ZonePoint[]): boolean {
  if (polygon.length < 3) return false;

  let inside = false;
  const n = polygon.length;

  for (let i = 0, j = n - 1; i < n; j = i++) {
    const xi = polygon[i].x;
    const zi = polygon[i].z;
    const xj = polygon[j].x;
    const zj = polygon[j].z;

    if (((zi > point.z) !== (zj > point.z)) &&
        (point.x < (xj - xi) * (point.z - zi) / (zj - zi) + xi)) {
      inside = !inside;
    }
  }

  return inside;
}

/**
 * Creates a new PriorityZone from input data with generated ID and defaults.
 *
 * Auto-generates a name if not provided using the zone ID suffix.
 * Initializes status as 'active' and sets creation timestamp.
 */
export function createZone(input: ZoneInput): PriorityZone {
  const id = generateZoneId();
  return {
    id,
    name: input.name || `Zone ${id.slice(-4).toUpperCase()}`,
    priority: input.priority,
    status: 'active',
    polygon: input.polygon,
    createdAt: Date.now(),
    notes: input.notes,
  };
}
