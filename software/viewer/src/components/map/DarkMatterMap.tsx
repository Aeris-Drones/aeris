'use client';

import { Map } from 'lucide-react';

/**
 * DarkMatterMap - Basic dark matter map style placeholder
 * 
 * Per spec Phase 1: "Basic dark matter map style (no 3D yet)"
 * Per spec Section 6.1 Backgrounds (Dark Matter Theme):
 * --bg-base: oklch(13% 0.005 260); Near black
 * 
 * This is a placeholder that will be replaced with the Three.js scene in Phase 2.
 * Shows the "Dark Matter" aesthetic with a subtle grid pattern.
 */

export function DarkMatterMap() {
  return (
    <div 
      className="relative h-full w-full"
      style={{ 
        background: 'oklch(13% 0.005 260)',
      }}
    >
      {/* Subtle grid pattern */}
      <div 
        className="absolute inset-0 opacity-[0.03]"
        style={{
          backgroundImage: `
            linear-gradient(to right, oklch(100% 0 0) 1px, transparent 1px),
            linear-gradient(to bottom, oklch(100% 0 0) 1px, transparent 1px)
          `,
          backgroundSize: '40px 40px',
        }}
      />
      
      {/* Radial gradient for depth */}
      <div 
        className="absolute inset-0"
        style={{
          background: 'radial-gradient(ellipse at center, transparent 0%, oklch(8% 0.005 260) 100%)',
        }}
      />
      
      {/* Center label - Phase indicator */}
      <div className="absolute inset-0 flex items-center justify-center">
        <div className="flex flex-col items-center gap-2 text-center">
          <Map className="h-10 w-10 opacity-10" />
          <span className="text-xs font-medium tracking-widest text-[var(--muted-foreground)]/30">
            3D MAP
          </span>
          <span className="text-[10px] tracking-wider text-[var(--muted-foreground)]/20">
            Phase 2: Three.js Scene
          </span>
        </div>
      </div>
    </div>
  );
}
