'use client';

import { Button } from '@/components/ui/button';
import { Flame, AudioLines, Wind, Crosshair, Check, X, MapPin, History } from 'lucide-react';
import { cn } from '@/lib/utils';
import { getConfidenceTextClass } from '@/lib/detectionViewState';

/**
 * Sensor detection with triage workflow state.
 *
 * Position uses ENU (East-North-Up) local coordinates relative to the search
 * area origin. This allows consistent spatial reasoning across vehicles without
 * requiring global geodetic transformations for short-range operations.
 *
 * Status lifecycle: new -> reviewing -> confirmed|dismissed
 *
 * Sensor-specific readings:
 * - thermal: surface temperature in Celsius
 * - acoustic: sound pressure level in decibels
 * - gas: chemical concentration in parts per million
 */
export interface Detection {
  id: string;
  sensorType: 'thermal' | 'acoustic' | 'gas';
  confidence: number;
  confidenceLevel?: 'HIGH' | 'MEDIUM' | 'LOW' | 'UNKNOWN';
  timestamp: number;
  status: 'new' | 'reviewing' | 'confirmed' | 'dismissed';
  vehicleId: string;
  vehicleName: string;
  position: [number, number, number];
  sourceModalities?: Array<'thermal' | 'acoustic' | 'gas'>;
  geometry?: [number, number, number][];
  temperature?: number;
  decibels?: number;
  concentration?: number;
  sector?: string;
  signatureType?: string;
  deliveryMode?: 'live' | 'replayed';
  originalEventTs?: number;
  replayedAtTs?: number;
  isRetroactive?: boolean;
}

export interface DetectionCardProps {
  detection: Detection;
  /** Whether this detection is newly arrived (for highlighting) */
  isNew: boolean;
  /** Callback when operator confirms detection as valid */
  onConfirm: () => void;
  /** Callback when operator marks detection as false positive */
  onDismiss: () => void;
  /** Callback to center map on detection location */
  onLocate: () => void;
}

const sensorConfig = {
  thermal: { Icon: Flame, color: 'text-orange-400', bg: 'bg-orange-500/10', label: 'Thermal' },
  acoustic: { Icon: AudioLines, color: 'text-sky-400', bg: 'bg-sky-500/10', label: 'Acoustic' },
  gas: { Icon: Wind, color: 'text-amber-400', bg: 'bg-amber-500/10', label: 'Gas' },
};

function formatTime(timestamp: number): string {
  const diff = Date.now() - timestamp;
  const min = Math.floor(diff / 60000);
  if (min < 1) return 'Just now';
  if (min < 60) return `${min}m ago`;
  return `${Math.floor(min / 60)}h ago`;
}

function getReading(detection: Detection): string {
  const hasFiniteValue = (value: unknown): value is number =>
    typeof value === 'number' && Number.isFinite(value);

  switch (detection.sensorType) {
    case 'thermal':
      return hasFiniteValue(detection.temperature) ? `${detection.temperature}°C` : '--';
    case 'acoustic':
      return hasFiniteValue(detection.decibels) ? `${detection.decibels}dB` : '--';
    case 'gas':
      return hasFiniteValue(detection.concentration) ? `${detection.concentration}ppm` : '--';
  }
}

/**
 * Generates human-readable classification based on confidence thresholds.
 *
 * Higher confidence produces more specific descriptions to help operators
 * prioritize review efforts. Low-confidence detections use vague language
 * to indicate uncertainty without dismissing the alert entirely.
 */
function getDefaultSignature(detection: Detection): string {
  const conf = detection.confidence;
  switch (detection.sensorType) {
    case 'thermal':
      return conf > 0.85 ? 'Human signature likely' : conf > 0.6 ? 'Possible heat source' : 'Anomaly detected';
    case 'acoustic':
      return conf > 0.85 ? 'Voice detected' : conf > 0.6 ? 'Movement sounds' : 'Audio anomaly';
    case 'gas':
      return conf > 0.85 ? 'Hazardous levels' : conf > 0.6 ? 'Elevated concentration' : 'Trace detected';
  }
}

/**
 * Detection triage card for operator review workflow.
 *
 * Presents sensor detections in a format optimized for rapid assessment:
 * - Large confidence percentage for quick prioritization
 * - Color-coded sensor badges for immediate type identification
 * - Contextual signature description based on confidence level
 * - Clear action hierarchy: locate (info) -> dismiss (negative) -> confirm (positive)
 *
 * Visual state changes communicate resolution status:
 * - Confirmed: Reduced opacity with emerald border accent
 * - Dismissed: Heavily reduced opacity (archived appearance)
 * - Actionable: Full prominence with interactive controls
 */
export function DetectionCard({
  detection,
  isNew,
  onConfirm,
  onDismiss,
  onLocate,
}: DetectionCardProps) {
  const sensor = sensorConfig[detection.sensorType];
  const SensorIcon = sensor.Icon;
  const isActionable = detection.status === 'new' || detection.status === 'reviewing';
  const conf = Math.round(detection.confidence * 100);
  const reading = getReading(detection);
  const isRetroactive =
    detection.deliveryMode === 'replayed' || detection.isRetroactive === true;

  // Derive sector from ENU coordinates when not explicitly provided
  const sector = detection.sector || `Zone ${detection.position[0] > 0 ? 'E' : 'W'}-${Math.abs(Math.round(detection.position[2] / 50))}`;
  const modalityLabel = detection.sourceModalities?.length
    ? detection.sourceModalities.join(' + ').toUpperCase()
    : null;
  const signature = detection.signatureType || modalityLabel || getDefaultSignature(detection);

  return (
    <div
      className={cn(
        'rounded-lg overflow-hidden',
        'bg-white/[0.02] hover:bg-white/[0.04]',
        'transition-colors duration-150',
        'border border-white/[0.04]',
        detection.status === 'confirmed' && 'opacity-60 border-emerald-500/20',
        detection.status === 'dismissed' && 'opacity-30'
      )}
    >
      <div className="flex items-center gap-3 px-4 py-3">
        <div className={cn('flex items-center gap-2 px-2.5 py-1 rounded-md', sensor.bg)}>
          <SensorIcon className={cn('h-4 w-4', sensor.color)} />
          <span className={cn('text-xs font-medium', sensor.color)}>{sensor.label}</span>
        </div>

        {isNew && (
          <span className="px-2 py-0.5 rounded bg-emerald-500/20 text-emerald-400 text-[10px] font-medium uppercase">
            New
          </span>
        )}
        {detection.status === 'confirmed' && (
          <span className="px-2 py-0.5 rounded bg-emerald-500/10 text-emerald-400/70 text-[10px] font-medium uppercase flex items-center gap-1">
            <Check className="h-3 w-3" /> Confirmed
          </span>
        )}
        {isRetroactive && (
          <span className="px-2 py-0.5 rounded bg-cyan-500/15 text-cyan-300 text-[10px] font-medium uppercase flex items-center gap-1">
            <History className="h-3 w-3" /> Retroactive
          </span>
        )}

        <div className="flex-1" />

        {/* Confidence percentage with color coding */}
        <span className={cn('font-mono text-xl font-medium', getConfidenceTextClass(detection))}>
          {conf}%
        </span>
      </div>

      <div className="px-4 pb-3 space-y-2">
        <div className="flex items-center gap-3 text-xs text-white/50">
          <span>{formatTime(detection.timestamp)}</span>
          <span>·</span>
          <span>{detection.vehicleName}</span>
          <span>·</span>
          <span className={cn(isRetroactive ? 'text-cyan-300' : 'text-emerald-300')}>
            {isRetroactive ? 'Replayed' : 'Live'}
          </span>
          <span>·</span>
          <span className="font-mono">{reading}</span>
        </div>

        <div className="flex items-center gap-2 text-sm">
          <MapPin className="h-3.5 w-3.5 text-white/30" />
          <span className="text-white/60">{sector}</span>
          <span className="text-white/20">|</span>
          <span className="text-white/80">{signature}</span>
        </div>
      </div>

      {isActionable && (
        <div className="flex items-center gap-2 px-4 py-2 border-t border-white/[0.04] bg-white/[0.01]">
          <Button
            variant="ghost"
            size="sm"
            className="h-8 px-3 text-xs text-white/50 hover:text-white"
            onClick={(e) => { e.stopPropagation(); onLocate(); }}
          >
            <Crosshair className="mr-1.5 h-3.5 w-3.5" />
            Locate
          </Button>
          <div className="flex-1" />
          <Button
            variant="ghost"
            size="sm"
            className="h-8 px-3 text-xs text-white/50 hover:text-red-400"
            onClick={(e) => { e.stopPropagation(); onDismiss(); }}
          >
            <X className="mr-1.5 h-3.5 w-3.5" />
            Dismiss
          </Button>
          <Button
            variant="ghost"
            size="sm"
            className="h-8 px-3 text-xs bg-emerald-500/10 text-emerald-400 hover:bg-emerald-500/20"
            onClick={(e) => { e.stopPropagation(); onConfirm(); }}
          >
            <Check className="mr-1.5 h-3.5 w-3.5" />
            Confirm
          </Button>
        </div>
      )}

      {!isActionable && (
        <div className="flex items-center gap-2 px-4 py-2 border-t border-white/[0.04] bg-white/[0.01]">
          <Button
            variant="ghost"
            size="sm"
            className="h-8 px-3 text-xs text-white/50 hover:text-white"
            onClick={(e) => { e.stopPropagation(); onLocate(); }}
          >
            <Crosshair className="mr-1.5 h-3.5 w-3.5" />
            View on map
          </Button>
        </div>
      )}
    </div>
  );
}
