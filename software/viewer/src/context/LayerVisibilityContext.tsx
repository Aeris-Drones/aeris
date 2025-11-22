'use client';

import React, { createContext, useContext, useState, ReactNode } from 'react';

interface LayerState {
  map: boolean;
  thermal: boolean;
  gas: boolean;
  acoustic: boolean;
  trajectories: boolean;
}

interface LayerVisibilityContextType extends LayerState {
  toggleLayer: (layer: keyof LayerState) => void;
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
