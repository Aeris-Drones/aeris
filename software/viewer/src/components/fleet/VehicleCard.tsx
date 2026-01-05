'use client';

/**
 * AERIS GCS Vehicle Card
 * 
 * Individual vehicle status card with telemetry display and quick actions.
 * Features glass-morphism styling and touch-optimized controls.
 */

import React, { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { cardVariants, transitions } from '@/lib/animations';
import type { VehicleInfo } from '@/types/vehicle';
import {
  getVehicleTypeName,
  getVehicleTypeColor,
  getVehicleStatusConfig,
  getBatteryColor,
  getSignalColor,
  formatAltitude,
  formatSpeed,
  formatDistance,
} from '@/types/vehicle';
import { useFleetContext } from '@/context/FleetContext';
import {
  Battery,
  Wifi,
  Navigation,
  ArrowUp,
  Gauge,
  Home,
  Pause,
  Play,
  Target,
  ChevronDown,
  ChevronUp,
  MapPin,
} from 'lucide-react';

// ============================================================================
// Types
// ============================================================================

interface VehicleCardProps {
  vehicle: VehicleInfo;
  compact?: boolean;
  className?: string;
  onFocus?: (vehicle: VehicleInfo) => void;
}

// ============================================================================
// Component
// ============================================================================

export function VehicleCard({
  vehicle,
  compact = false,
  className,
  onFocus,
}: VehicleCardProps) {
  const [isExpanded, setIsExpanded] = useState(false);
  const { selectVehicle, recallVehicle, holdVehicle, resumeVehicle, selectedVehicleId } = useFleetContext();
  
  const isSelected = selectedVehicleId === vehicle.id;
  const statusConfig = getVehicleStatusConfig(vehicle.status);
  const typeColor = getVehicleTypeColor(vehicle.type);
  
  const handleSelect = () => {
    selectVehicle(isSelected ? null : vehicle.id);
  };
  
  const handleFocus = () => {
    onFocus?.(vehicle);
  };
  
  // Compact card (for list view)
  if (compact) {
    return (
      <motion.div
        variants={cardVariants}
        initial="hidden"
        animate="visible"
        whileHover="hover"
        onClick={handleSelect}
        className={cn(
          'flex items-center gap-3 p-3 rounded-lg cursor-pointer',
          'bg-surface-2/50 hover:bg-surface-2 transition-colors',
          'border border-transparent',
          isSelected && 'border-info/50 bg-info/5',
          className
        )}
      >
        {/* Type indicator */}
        <div
          className="w-2 h-8 rounded-full"
          style={{ backgroundColor: typeColor }}
        />
        
        {/* Info */}
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2">
            <span className="font-medium text-sm text-foreground truncate">
              {vehicle.id}
            </span>
            <span className={cn('text-xs px-1.5 py-0.5 rounded', statusConfig.bgColor, statusConfig.color)}>
              {statusConfig.label}
            </span>
          </div>
          <div className="flex items-center gap-3 mt-0.5 text-xs text-muted-foreground">
            <span className={getBatteryColor(vehicle.batteryPercent)}>
              {vehicle.batteryPercent.toFixed(0)}%
            </span>
            <span>{formatAltitude(vehicle.altitude)}</span>
            <span>{formatSpeed(vehicle.speed)}</span>
          </div>
        </div>
        
        {/* Focus button */}
        <button
          onClick={(e) => {
            e.stopPropagation();
            handleFocus();
          }}
          className={cn(
            'p-2 rounded-md',
            'text-muted-foreground hover:text-foreground hover:bg-surface-3',
            'transition-colors',
            'min-w-[var(--touch-min)] min-h-[var(--touch-min)]',
            'flex items-center justify-center'
          )}
        >
          <Target className="w-4 h-4" />
        </button>
      </motion.div>
    );
  }
  
  // Full card
  return (
    <motion.div
      variants={cardVariants}
      initial="hidden"
      animate="visible"
      className={cn(
        'rounded-xl overflow-hidden',
        'bg-surface-2/80 backdrop-blur-sm',
        'border border-glass-border',
        isSelected && 'ring-2 ring-info/50',
        className
      )}
    >
      {/* Header */}
      <div
        onClick={handleSelect}
        className={cn(
          'flex items-center gap-3 p-3 cursor-pointer',
          'hover:bg-surface-3/50 transition-colors'
        )}
      >
        {/* Type badge */}
        <div
          className="w-10 h-10 rounded-lg flex items-center justify-center"
          style={{ backgroundColor: `${typeColor}20` }}
        >
          <Navigation className="w-5 h-5" style={{ color: typeColor }} />
        </div>
        
        {/* Info */}
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2">
            <span className="font-semibold text-foreground">
              {vehicle.id}
            </span>
            <span className="text-xs text-muted-foreground">
              {getVehicleTypeName(vehicle.type)}
            </span>
          </div>
          <div className="flex items-center gap-1.5 mt-0.5">
            <span className={cn(
              'inline-flex items-center gap-1 text-xs px-1.5 py-0.5 rounded',
              statusConfig.bgColor,
              statusConfig.color
            )}>
              {vehicle.status === 'active' && (
                <span className="relative flex h-1.5 w-1.5">
                  <span className="animate-ping absolute inline-flex h-full w-full rounded-full opacity-75 bg-current" />
                  <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-current" />
                </span>
              )}
              {statusConfig.label}
            </span>
          </div>
        </div>
        
        {/* Expand toggle */}
        <button
          onClick={(e) => {
            e.stopPropagation();
            setIsExpanded(!isExpanded);
          }}
          className={cn(
            'p-2 rounded-md',
            'text-muted-foreground hover:text-foreground hover:bg-surface-3',
            'transition-colors'
          )}
        >
          {isExpanded ? (
            <ChevronUp className="w-4 h-4" />
          ) : (
            <ChevronDown className="w-4 h-4" />
          )}
        </button>
      </div>
      
      {/* Telemetry row */}
      <div className="flex items-center gap-4 px-3 pb-3">
        {/* Battery */}
        <div className="flex items-center gap-1.5">
          <Battery className={cn('w-4 h-4', getBatteryColor(vehicle.batteryPercent))} />
          <span className={cn('text-sm font-mono', getBatteryColor(vehicle.batteryPercent))}>
            {vehicle.batteryPercent.toFixed(0)}%
          </span>
        </div>
        
        {/* Signal */}
        <div className="flex items-center gap-1.5">
          <Wifi className={cn('w-4 h-4', getSignalColor(vehicle.signalStrength))} />
          <span className={cn('text-sm font-mono', getSignalColor(vehicle.signalStrength))}>
            {vehicle.signalStrength.toFixed(0)}%
          </span>
        </div>
        
        {/* Altitude */}
        <div className="flex items-center gap-1.5 text-muted-foreground">
          <ArrowUp className="w-4 h-4" />
          <span className="text-sm font-mono">
            {formatAltitude(vehicle.altitude)}
          </span>
        </div>
        
        {/* Speed */}
        <div className="flex items-center gap-1.5 text-muted-foreground">
          <Gauge className="w-4 h-4" />
          <span className="text-sm font-mono">
            {formatSpeed(vehicle.speed)}
          </span>
        </div>
      </div>
      
      {/* Expanded content */}
      <AnimatePresence>
        {isExpanded && (
          <motion.div
            initial={{ height: 0, opacity: 0 }}
            animate={{ height: 'auto', opacity: 1 }}
            exit={{ height: 0, opacity: 0 }}
            transition={transitions.normal}
            className="overflow-hidden"
          >
            <div className="px-3 pb-3 space-y-3 border-t border-glass-border pt-3">
              {/* Position info */}
              <div className="flex items-center gap-2 text-sm">
                <MapPin className="w-4 h-4 text-muted-foreground" />
                <span className="text-muted-foreground">Distance from home:</span>
                <span className="font-mono text-foreground">
                  {formatDistance(vehicle.distanceFromHome)}
                </span>
              </div>
              
              {/* Quick actions */}
              <div className="flex gap-2">
                <button
                  onClick={() => holdVehicle(vehicle.id)}
                  className={cn(
                    'flex-1 flex items-center justify-center gap-2 px-3 py-2.5',
                    'bg-warning/10 hover:bg-warning/20 border border-warning/30',
                    'text-warning text-sm font-medium rounded-lg',
                    'transition-colors min-h-[var(--touch-min)]'
                  )}
                >
                  <Pause className="w-4 h-4" />
                  Hold
                </button>
                
                <button
                  onClick={() => resumeVehicle(vehicle.id)}
                  className={cn(
                    'flex-1 flex items-center justify-center gap-2 px-3 py-2.5',
                    'bg-success/10 hover:bg-success/20 border border-success/30',
                    'text-success text-sm font-medium rounded-lg',
                    'transition-colors min-h-[var(--touch-min)]'
                  )}
                >
                  <Play className="w-4 h-4" />
                  Resume
                </button>
                
                <button
                  onClick={() => recallVehicle(vehicle.id)}
                  className={cn(
                    'flex-1 flex items-center justify-center gap-2 px-3 py-2.5',
                    'bg-info/10 hover:bg-info/20 border border-info/30',
                    'text-info text-sm font-medium rounded-lg',
                    'transition-colors min-h-[var(--touch-min)]'
                  )}
                >
                  <Home className="w-4 h-4" />
                  Recall
                </button>
              </div>
              
              {/* Focus on map button */}
              <button
                onClick={handleFocus}
                className={cn(
                  'w-full flex items-center justify-center gap-2 px-3 py-2.5',
                  'bg-surface-3 hover:bg-surface-4',
                  'text-foreground text-sm font-medium rounded-lg',
                  'transition-colors min-h-[var(--touch-min)]'
                )}
              >
                <Target className="w-4 h-4" />
                Focus on Map
              </button>
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </motion.div>
  );
}

export default VehicleCard;
