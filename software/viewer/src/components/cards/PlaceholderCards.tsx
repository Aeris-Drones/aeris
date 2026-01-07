'use client';

/**
 * Placeholder Cards for Phase 1
 * 
 * Per spec Phase 1 deliverable: "Static layout with placeholder cards"
 * Per spec Section 9.1: Cards use shadcn Card component
 * 
 * These will be replaced with full implementations in Phase 3:
 * - MissionCard (phase, timer, progress)
 * - FleetCard (dots, stats, warnings)
 * - DetectionsCard (sensor counts, summary)
 * - ControlsCard (start/pause/abort buttons)
 */

import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Progress } from '@/components/ui/progress';
import { Badge } from '@/components/ui/badge';
import { 
  Play, 
  Pause, 
  Square, 
  Zap, 
  Thermometer, 
  Volume2, 
  Wind 
} from 'lucide-react';

/**
 * MissionCard Placeholder
 * Per spec Section 4.3 - Compact View:
 * - Phase badge with status dot
 * - Timer (MM:SS)
 * - Progress bar with percentage
 */
export function MissionCardPlaceholder() {
  return (
    <Card className="min-w-[140px] border-[var(--glass-border)] bg-[var(--glass-bg)] backdrop-blur-xl">
      <CardHeader className="p-4 pb-2">
        <CardTitle className="text-xs font-semibold tracking-wide text-[var(--muted-foreground)]">
          MISSION
        </CardTitle>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 p-4 pt-0">
        {/* Phase badge placeholder */}
        <Badge variant="secondary" className="w-fit">
          <span className="mr-2 h-2 w-2 rounded-full bg-current opacity-50" />
          IDLE
        </Badge>
        
        {/* Timer placeholder */}
        <div className="font-mono text-2xl font-bold tabular-nums text-[var(--foreground)]">
          00:00
        </div>
        
        {/* Progress bar - using shadcn Progress */}
        <div className="flex items-center gap-2">
          <Progress value={0} className="h-1.5 flex-1" />
          <span className="text-xs text-[var(--muted-foreground)]">0%</span>
        </div>
      </CardContent>
    </Card>
  );
}

/**
 * FleetCard Placeholder
 * Per spec Section 4.3 - Compact View:
 * - Vehicle dots colored by status
 * - Active/Total count
 * - Average battery
 * - Warning indicator if any
 */
export function FleetCardPlaceholder() {
  return (
    <Card className="min-w-[160px] border-[var(--glass-border)] bg-[var(--glass-bg)] backdrop-blur-xl">
      <CardHeader className="p-4 pb-2">
        <CardTitle className="text-xs font-semibold tracking-wide text-[var(--muted-foreground)]">
          FLEET
        </CardTitle>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 p-4 pt-0">
        {/* Vehicle dots placeholder */}
        <div className="flex items-center gap-1">
          <span className="h-3 w-3 rounded-full bg-[var(--muted-foreground)]/30" />
          <span className="h-3 w-3 rounded-full bg-[var(--muted-foreground)]/30" />
          <span className="h-3 w-3 rounded-full bg-[var(--muted-foreground)]/30" />
          <span className="h-3 w-3 rounded-full bg-[var(--muted-foreground)]/30" />
          <span className="ml-2 text-sm text-[var(--muted-foreground)]">0/4</span>
        </div>
        
        {/* Stats placeholder */}
        <div className="flex items-center gap-4 text-xs text-[var(--muted-foreground)]">
          <span className="flex items-center gap-1">
            <Zap className="h-3 w-3" />
            --% avg
          </span>
          <span>--m alt</span>
        </div>
      </CardContent>
    </Card>
  );
}

/**
 * DetectionsCard Placeholder
 * Per spec Section 4.3 - Compact View:
 * - Sensor type badges with counts
 * - "Tap to review" prompt
 * - Pending vs confirmed summary
 */
export function DetectionsCardPlaceholder() {
  return (
    <Card className="min-w-[200px] border-[var(--glass-border)] bg-[var(--glass-bg)] backdrop-blur-xl">
      <CardHeader className="p-4 pb-2">
        <CardTitle className="text-xs font-semibold tracking-wide text-[var(--muted-foreground)]">
          DETECTIONS
        </CardTitle>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 p-4 pt-0">
        {/* Sensor badges placeholder - using Lucide icons instead of emojis */}
        <div className="flex items-center gap-2">
          <Badge variant="outline" className="gap-1 text-[var(--sensor-thermal)]">
            <Thermometer className="h-3 w-3" />
            <span>0</span>
          </Badge>
          <Badge variant="outline" className="gap-1 text-[var(--sensor-acoustic)]">
            <Volume2 className="h-3 w-3" />
            <span>0</span>
          </Badge>
          <Badge variant="outline" className="gap-1 text-[var(--sensor-gas)]">
            <Wind className="h-3 w-3" />
            <span>0</span>
          </Badge>
        </div>
        
        {/* Status placeholder */}
        <div className="text-xs text-[var(--muted-foreground)]">
          No detections yet
        </div>
      </CardContent>
    </Card>
  );
}

/**
 * ControlsCard Placeholder
 * Per spec Section 4.3:
 * - START: Green gradient, disabled during mission
 * - PAUSE/RESUME: Yellow, toggles based on state
 * - ABORT: Red, always enabled during mission
 */
export function ControlsCardPlaceholder() {
  return (
    <Card className="min-w-[180px] border-[var(--glass-border)] bg-[var(--glass-bg)] backdrop-blur-xl">
      <CardHeader className="p-4 pb-2">
        <CardTitle className="text-xs font-semibold tracking-wide text-[var(--muted-foreground)]">
          CONTROLS
        </CardTitle>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 p-4 pt-0">
        {/* START button - using shadcn Button */}
        <Button 
          disabled 
          variant="default"
          className="h-10 w-full gap-2 bg-[var(--success)]/20 text-[var(--success)] opacity-50 hover:bg-[var(--success)]/30"
        >
          <Play className="h-4 w-4" />
          START
        </Button>
        
        {/* PAUSE/ABORT row */}
        <div className="flex items-center gap-2">
          <Button
            disabled
            variant="outline"
            size="sm"
            className="flex-1 gap-1 opacity-50"
          >
            <Pause className="h-3 w-3" />
            PAUSE
          </Button>
          <Button
            disabled
            variant="outline"
            size="sm"
            className="gap-1 opacity-50"
          >
            <Square className="h-3 w-3" />
            ABORT
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}
