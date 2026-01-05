'use client';

/**
 * AERIS GCS Fleet Panel
 * 
 * Complete fleet management panel with vehicle list, fleet actions,
 * and statistics. Can be used as sidebar content or floating panel.
 */

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { listContainerVariants, listItemVariants, panelVariants } from '@/lib/animations';
import { GlassPanel } from '@/components/ui/GlassPanel';
import { VehicleCard } from './VehicleCard';
import { useFleetContext } from '@/context/FleetContext';
import type { VehicleInfo } from '@/types/vehicle';
import {
  Plane,
  Home,
  Pause,
  Play,
  Battery,
  Wifi,
  AlertTriangle,
} from 'lucide-react';

// ============================================================================
// Fleet Stats Bar
// ============================================================================

function FleetStatsBar() {
  const { fleetStats } = useFleetContext();
  
  return (
    <div className="flex items-center gap-3 py-2 text-xs">
      <div className="flex items-center gap-1.5">
        <Plane className="w-3.5 h-3.5 text-muted-foreground" />
        <span className="text-muted-foreground">Active:</span>
        <span className="font-semibold text-success">{fleetStats.active}</span>
        <span className="text-muted-foreground">/ {fleetStats.total}</span>
      </div>
      
      {fleetStats.returning > 0 && (
        <div className="flex items-center gap-1.5">
          <Home className="w-3.5 h-3.5 text-warning" />
          <span className="font-semibold text-warning">{fleetStats.returning}</span>
        </div>
      )}
      
      {fleetStats.offline > 0 && (
        <div className="flex items-center gap-1.5">
          <AlertTriangle className="w-3.5 h-3.5 text-danger" />
          <span className="font-semibold text-danger">{fleetStats.offline}</span>
        </div>
      )}
      
      <div className="flex items-center gap-1.5 ml-auto">
        <Battery className="w-3.5 h-3.5 text-muted-foreground" />
        <span className="font-mono text-foreground">{fleetStats.avgBattery.toFixed(0)}%</span>
      </div>
    </div>
  );
}

// ============================================================================
// Fleet Actions Bar
// ============================================================================

function FleetActionsBar() {
  const { recallAll, holdAll, resumeAll, fleetStats } = useFleetContext();
  
  if (fleetStats.total === 0) return null;
  
  return (
    <div className="flex gap-2 py-2">
      <button
        onClick={holdAll}
        className={cn(
          'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
          'bg-warning/10 hover:bg-warning/20 border border-warning/30',
          'text-warning text-xs font-medium rounded-lg',
          'transition-colors min-h-[var(--touch-min)]'
        )}
      >
        <Pause className="w-3.5 h-3.5" />
        Hold All
      </button>
      
      <button
        onClick={resumeAll}
        className={cn(
          'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
          'bg-success/10 hover:bg-success/20 border border-success/30',
          'text-success text-xs font-medium rounded-lg',
          'transition-colors min-h-[var(--touch-min)]'
        )}
      >
        <Play className="w-3.5 h-3.5" />
        Resume All
      </button>
      
      <button
        onClick={recallAll}
        className={cn(
          'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
          'bg-info/10 hover:bg-info/20 border border-info/30',
          'text-info text-xs font-medium rounded-lg',
          'transition-colors min-h-[var(--touch-min)]'
        )}
      >
        <Home className="w-3.5 h-3.5" />
        Recall All
      </button>
    </div>
  );
}

// ============================================================================
// Vehicle List
// ============================================================================

interface VehicleListProps {
  compact?: boolean;
  onVehicleFocus?: (vehicle: VehicleInfo) => void;
}

function VehicleList({ compact = false, onVehicleFocus }: VehicleListProps) {
  const { vehicles } = useFleetContext();
  
  if (vehicles.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center py-8 text-center">
        <Plane className="w-10 h-10 text-muted-foreground/30 mb-3" />
        <p className="text-sm text-muted-foreground">No vehicles connected</p>
        <p className="text-xs text-muted-foreground/70 mt-1">
          Waiting for telemetry data...
        </p>
      </div>
    );
  }
  
  return (
    <motion.div
      variants={listContainerVariants}
      initial="hidden"
      animate="visible"
      className="space-y-2"
    >
      <AnimatePresence mode="popLayout">
        {vehicles.map((vehicle) => (
          <motion.div
            key={vehicle.id}
            variants={listItemVariants}
            layout
          >
            <VehicleCard
              vehicle={vehicle}
              compact={compact}
              onFocus={onVehicleFocus}
            />
          </motion.div>
        ))}
      </AnimatePresence>
    </motion.div>
  );
}

// ============================================================================
// Main Fleet Panel (Sidebar variant)
// ============================================================================

interface FleetPanelProps {
  onVehicleFocus?: (vehicle: VehicleInfo) => void;
  className?: string;
}

export function FleetPanel({ onVehicleFocus, className }: FleetPanelProps) {
  const { fleetStats } = useFleetContext();
  
  return (
    <div className={cn('flex flex-col h-full bg-surface-1', className)}>
      {/* Header */}
      <div className="flex-none px-4 py-3 border-b border-glass-border">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <Plane className="w-5 h-5 text-foreground" />
            <h2 className="text-base font-semibold text-foreground">Fleet</h2>
          </div>
          
          <span className="px-2 py-0.5 rounded-md bg-surface-3 text-xs font-mono text-foreground">
            {fleetStats.total} vehicles
          </span>
        </div>
        
        <FleetStatsBar />
        <FleetActionsBar />
      </div>
      
      {/* Vehicle list */}
      <div className="flex-1 overflow-y-auto px-4 py-3">
        <VehicleList onVehicleFocus={onVehicleFocus} />
      </div>
    </div>
  );
}

// ============================================================================
// Floating Fleet Panel
// ============================================================================

interface FloatingFleetPanelProps {
  onVehicleFocus?: (vehicle: VehicleInfo) => void;
  className?: string;
}

export function FloatingFleetPanel({ onVehicleFocus, className }: FloatingFleetPanelProps) {
  const { fleetStats } = useFleetContext();
  
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className={className}
    >
      <GlassPanel
        title="Fleet"
        icon={<Plane className="w-4 h-4" />}
        collapsible
        defaultCollapsed={false}
        className="w-[320px] max-h-[400px] pointer-events-auto"
        headerRight={
          <span className="text-xs font-mono text-muted-foreground">
            {fleetStats.active}/{fleetStats.total}
          </span>
        }
      >
        <div className="space-y-3">
          <FleetStatsBar />
          <FleetActionsBar />
          <div className="max-h-[220px] overflow-y-auto -mx-1 px-1">
            <VehicleList compact onVehicleFocus={onVehicleFocus} />
          </div>
        </div>
      </GlassPanel>
    </motion.div>
  );
}

// ============================================================================
// Compact Fleet Panel (for mobile bottom sheet)
// ============================================================================

export function FleetPanelCompact({ onVehicleFocus, className }: FleetPanelProps) {
  return (
    <div className={cn('flex flex-col h-full bg-surface-1 rounded-t-2xl overflow-hidden', className)}>
      {/* Drag handle */}
      <div className="flex-none flex items-center justify-center py-2">
        <div className="w-12 h-1 rounded-full bg-muted-foreground/30" />
      </div>
      
      {/* Header */}
      <div className="flex-none px-4 pb-2 border-b border-glass-border">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-semibold text-foreground">Fleet</h3>
          <FleetStatsBar />
        </div>
      </div>
      
      {/* Compact vehicle list */}
      <div className="flex-1 overflow-y-auto px-4 py-2">
        <VehicleList compact onVehicleFocus={onVehicleFocus} />
      </div>
    </div>
  );
}

// ============================================================================
// Export
// ============================================================================

export default FleetPanel;
