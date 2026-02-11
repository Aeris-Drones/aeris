'use client';

import React, { createContext, useContext, useState, ReactNode } from 'react';

/**
 * Layer visibility state for the 3D map scene.
 *
 * These layers correspond to data overlays that can be toggled independently:
 * - map: Base terrain/satellite imagery
 * - thermal: Thermal sensor detection markers and heatmaps
 * - gas: Gas sensor readings and concentration overlays
 * - acoustic: Acoustic detection markers and source indicators
 * - trajectories: Vehicle path history and planned waypoints
 */
interface LayerState {
  map: boolean;
  thermal: boolean;
  gas: boolean;
  acoustic: boolean;
  trajectories: boolean;
}

interface LayerVisibilityContextType extends LayerState {
  /** Toggle a layer's visibility state. */
  toggleLayer: (layer: keyof LayerState) => void;
  /** Explicitly set a layer's visibility. */
  setLayer: (layer: keyof LayerState, visible: boolean) => void;
}

const defaultState: LayerState = {
  map: true,
  thermal: true,
  gas: true,
  acoustic: true,
  trajectories: true,
};

const LayerVisibilityContext = createContext<LayerVisibilityContextType | undefined>(undefined);

/**
 * Controls visibility of map overlay layers.
 *
 * Layer state is local to the session (not persisted) as operator preferences
 * for data visualization vary by mission phase and environmental conditions.
 * All layers default to visible to provide maximum situational awareness
 * on initial load.
 */
export function LayerVisibilityProvider({ children }: { children: ReactNode }) {
  const [layers, setLayers] = useState<LayerState>(defaultState);

  const toggleLayer = (layer: keyof LayerState) => {
    setLayers((prev) => ({ ...prev, [layer]: !prev[layer] }));
  };

  const setLayer = (layer: keyof LayerState, visible: boolean) => {
    setLayers((prev) => ({ ...prev, [layer]: visible }));
  };

  return (
    <LayerVisibilityContext.Provider value={{ ...layers, toggleLayer, setLayer }}>
      {children}
    </LayerVisibilityContext.Provider>
  );
}

export function useLayerVisibility() {
  const context = useContext(LayerVisibilityContext);
  if (context === undefined) {
    throw new Error('useLayerVisibility must be used within a LayerVisibilityProvider');
  }
  return context;
}
