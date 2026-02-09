'use client';

import React, { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { cardVariants, transitions, buttonVariants } from '@/lib/animations';
import { type SensorType } from '@/lib/design-tokens';
import {
  ConfidenceBar,
  ConfidenceBadge,
  ConfidenceDot,
} from '@/components/ui/ConfidenceIndicator';
import type { Detection } from '@/types/detection';
import {
  formatDetectionSummary,
  getTimeSince,
  getClassificationName,
  getSensorName,
} from '@/types/detection';
import {
  Thermometer,
  Radio,
  Wind,
  Check,
  X,
  MapPin,
} from 'lucide-react';

interface DetectionCardProps {
  detection: Detection;
  onConfirm?: () => void;
  onDismiss?: () => void;
  onFocus?: () => void;
  onExpand?: (expanded: boolean) => void;
  expanded?: boolean;
  className?: string;
}

const sensorIcons: Record<SensorType, React.ReactNode> = {
  thermal: <Thermometer className="w-4 h-4" />,
  acoustic: <Radio className="w-4 h-4" />,
  gas: <Wind className="w-4 h-4" />,
};

export function DetectionCard({
  detection,
  onConfirm,
  onDismiss,
  onFocus,
  onExpand,
  expanded = false,
  className,
}: DetectionCardProps) {
  const [isExpanded, setIsExpanded] = useState(expanded);

  const handleExpand = () => {
    const newExpanded = !isExpanded;
    setIsExpanded(newExpanded);
    onExpand?.(newExpanded);
  };

  const handleConfirm = (e: React.MouseEvent) => {
    e.stopPropagation();
    onConfirm?.();
  };

  const handleDismiss = (e: React.MouseEvent) => {
    e.stopPropagation();
    onDismiss?.();
  };

  const handleFocus = (e: React.MouseEvent) => {
    e.stopPropagation();
    onFocus?.();
  };

  const isNew = detection.status === 'new';
  const isConfirmed = detection.status === 'confirmed';
  const isDismissed = detection.status === 'dismissed';

  return (
    <motion.div
      variants={cardVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      whileHover="hover"
      className={cn(
        // Base card styling
        'relative bg-surface-2 rounded-lg overflow-hidden cursor-pointer',
        'border border-glass-border',
        // Sensor type accent
        'border-l-4',
        detection.sensorType === 'thermal' && 'border-l-sensor-thermal',
        detection.sensorType === 'acoustic' && 'border-l-sensor-acoustic',
        detection.sensorType === 'gas' && 'border-l-sensor-gas',
        // Status styling
        isNew && 'ring-2 ring-confidence-high/30 shadow-[0_0_15px_var(--confidence-high)]',
        isConfirmed && 'opacity-75',
        isDismissed && 'opacity-50',
        // Transitions
        'transition-all duration-normal',
        className
      )}
      onClick={handleExpand}
    >
      {/* New detection pulse indicator */}
      {isNew && (
        <motion.div
          className="absolute top-2 right-2"
          animate={{
            scale: [1, 1.2, 1],
            opacity: [1, 0.6, 1],
          }}
          transition={{
            duration: 1.5,
            repeat: Infinity,
            ease: 'easeInOut',
          }}
        >
          <ConfidenceDot
            confidence={detection.confidence}
            size="md"
            pulse={true}
          />
        </motion.div>
      )}

      {/* Header */}
      <div className="p-3">
        <div className="flex items-start justify-between gap-2">
          {/* Sensor type and summary */}
          <div className="flex-1 min-w-0">
            <div className="flex items-center gap-2 mb-1">
              <span
                className={cn(
                  'flex items-center gap-1.5 text-xs font-medium uppercase tracking-wider',
                  detection.sensorType === 'thermal' && 'text-sensor-thermal',
                  detection.sensorType === 'acoustic' && 'text-sensor-acoustic',
                  detection.sensorType === 'gas' && 'text-sensor-gas'
                )}
              >
                {sensorIcons[detection.sensorType]}
                {getSensorName(detection.sensorType)}
              </span>
              <span className="text-xs text-muted-foreground">
                {getTimeSince(detection.timestamp)}
              </span>
            </div>
            <p className="text-sm font-medium text-foreground truncate">
              {formatDetectionSummary(detection)}
            </p>
          </div>

          {/* Confidence indicator */}
          <ConfidenceBadge confidence={detection.confidence} />
        </div>

        {/* Confidence bar */}
        <div className="mt-2">
          <ConfidenceBar confidence={detection.confidence} size="sm" showLabel={false} />
        </div>
      </div>

      {/* Expanded details */}
      <AnimatePresence>
        {isExpanded && (
          <motion.div
            initial={{ height: 0, opacity: 0 }}
            animate={{ height: 'auto', opacity: 1 }}
            exit={{ height: 0, opacity: 0 }}
            transition={transitions.spring}
            className="overflow-hidden"
          >
            <div className="px-3 pb-3 pt-2 border-t border-glass-border space-y-2">
              {/* Sensor-specific details */}
              {detection.sensorType === 'thermal' && (
                <div className="grid grid-cols-2 gap-2 text-xs">
                  <div>
                    <span className="text-muted-foreground">Temperature</span>
                    <p className="font-mono font-semibold text-sensor-thermal">
                      {detection.temperature.toFixed(1)}°C
                    </p>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Confidence</span>
                    <p className="font-mono font-semibold">
                      {Math.round(detection.confidence * 100)}%
                    </p>
                  </div>
                </div>
              )}

              {detection.sensorType === 'acoustic' && (
                <div className="grid grid-cols-2 gap-2 text-xs">
                  <div>
                    <span className="text-muted-foreground">Type</span>
                    <p className="font-semibold text-sensor-acoustic">
                      {getClassificationName(detection.classification)}
                    </p>
                  </div>
                  <div>
                    <span className="text-muted-foreground">SNR</span>
                    <p className="font-mono font-semibold">
                      {detection.snr.toFixed(1)} dB
                    </p>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Bearing</span>
                    <p className="font-mono font-semibold">
                      {detection.bearing.toFixed(0)}°
                    </p>
                  </div>
                  {detection.vehicleId && (
                    <div>
                      <span className="text-muted-foreground">Vehicle</span>
                      <p className="font-mono text-xs">{detection.vehicleId}</p>
                    </div>
                  )}
                </div>
              )}

              {detection.sensorType === 'gas' && (
                <div className="grid grid-cols-2 gap-2 text-xs">
                  <div>
                    <span className="text-muted-foreground">Species</span>
                    <p className="font-semibold text-sensor-gas">{detection.species}</p>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Concentration</span>
                    <p className="font-mono font-semibold">
                      {detection.concentration.toFixed(1)} {detection.units}
                    </p>
                  </div>
                </div>
              )}

              {/* Notes */}
              {detection.notes && (
                <div className="pt-2 border-t border-glass-border">
                  <span className="text-xs text-muted-foreground">Notes</span>
                  <p className="text-xs text-foreground mt-1">{detection.notes}</p>
                </div>
              )}
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Actions */}
      {!isConfirmed && !isDismissed && (
        <div className="flex gap-2 p-2 bg-surface-3/50 border-t border-glass-border">
          <motion.button
            variants={buttonVariants}
            whileHover="hover"
            whileTap="tap"
            onClick={handleFocus}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 px-3 py-2 rounded-md',
              'text-xs font-medium',
              'bg-surface-3 hover:bg-surface-4',
              'text-foreground',
              'transition-colors',
              'min-h-[var(--touch-min)]'
            )}
          >
            <MapPin className="w-3.5 h-3.5" />
            Focus
          </motion.button>

          <motion.button
            variants={buttonVariants}
            whileHover="hover"
            whileTap="tap"
            onClick={handleDismiss}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 px-3 py-2 rounded-md',
              'text-xs font-medium',
              'bg-danger/10 hover:bg-danger/20',
              'text-danger',
              'border border-danger/30',
              'transition-colors',
              'min-h-[var(--touch-min)]'
            )}
          >
            <X className="w-3.5 h-3.5" />
            Dismiss
          </motion.button>

          <motion.button
            variants={buttonVariants}
            whileHover="hover"
            whileTap="tap"
            onClick={handleConfirm}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 px-3 py-2 rounded-md',
              'text-xs font-medium',
              'bg-success/10 hover:bg-success/20',
              'text-success',
              'border border-success/30',
              'transition-colors',
              'min-h-[var(--touch-min)]'
            )}
          >
            <Check className="w-3.5 h-3.5" />
            Confirm
          </motion.button>
        </div>
      )}

      {/* Status indicator */}
      {(isConfirmed || isDismissed) && (
        <div
          className={cn(
            'px-3 py-2 text-xs font-medium text-center',
            'bg-surface-3/50 border-t border-glass-border',
            isConfirmed && 'text-success',
            isDismissed && 'text-danger'
          )}
        >
          {isConfirmed ? '✓ Confirmed' : '✕ Dismissed'}
        </div>
      )}
    </motion.div>
  );
}

/**
 * Compact detection card variant for list view
 */
export function DetectionCardCompact({
  detection,
  onConfirm,
  onDismiss,
  className,
}: Omit<DetectionCardProps, 'expanded' | 'onExpand'>) {
  return (
    <motion.div
      variants={cardVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className={cn(
        'flex items-center gap-3 p-2 rounded-lg bg-surface-2',
        'border border-glass-border border-l-4',
        detection.sensorType === 'thermal' && 'border-l-sensor-thermal',
        detection.sensorType === 'acoustic' && 'border-l-sensor-acoustic',
        detection.sensorType === 'gas' && 'border-l-sensor-gas',
        detection.status === 'new' && 'ring-1 ring-confidence-high/20',
        className
      )}
    >
      <div className="flex-1 min-w-0">
        <p className="text-xs font-medium text-foreground truncate">
          {formatDetectionSummary(detection)}
        </p>
        <p className="text-xs text-muted-foreground">{getTimeSince(detection.timestamp)}</p>
      </div>

      <ConfidenceDot confidence={detection.confidence} size="sm" />

      {detection.status === 'new' && (
        <div className="flex gap-1">
          <button
            onClick={(e) => {
              e.stopPropagation();
              onDismiss?.();
            }}
            className="p-1 rounded hover:bg-danger/20 text-danger transition-colors"
            aria-label="Dismiss"
          >
            <X className="w-4 h-4" />
          </button>
          <button
            onClick={(e) => {
              e.stopPropagation();
              onConfirm?.();
            }}
            className="p-1 rounded hover:bg-success/20 text-success transition-colors"
            aria-label="Confirm"
          >
            <Check className="w-4 h-4" />
          </button>
        </div>
      )}
    </motion.div>
  );
}
