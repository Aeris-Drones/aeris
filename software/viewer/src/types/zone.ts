export type ZonePriority = 1 | 2 | 3;

export type ZoneStatus = 'active' | 'completed' | 'skipped';

export interface ZonePoint {
  x: number;
  z: number;
}

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

export interface ZoneInput {
  name?: string;
  priority: ZonePriority;
  polygon: ZonePoint[];
  notes?: string;
}

export function generateZoneId(): string {
  const timestamp = Date.now().toString(36);
  const random = Math.random().toString(36).substring(2, 6);
  return `zone-${timestamp}-${random}`;
}

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
