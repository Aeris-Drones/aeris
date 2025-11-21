'use client';

import React, { createContext, useContext, useState, ReactNode } from 'react';
import { GeoCoordinates } from '../lib/ros/mapTile';

interface CoordinateOriginContextType {
  origin: GeoCoordinates | null;
  setOrigin: (origin: GeoCoordinates) => void;
}

const CoordinateOriginContext = createContext<CoordinateOriginContextType | undefined>(undefined);

export function CoordinateOriginProvider({ children }: { children: ReactNode }) {
  const [origin, setOrigin] = useState<GeoCoordinates | null>(null);

  return (
    <CoordinateOriginContext.Provider value={{ origin, setOrigin }}>
      {children}
    </CoordinateOriginContext.Provider>
  );
}

export function useCoordinateOrigin() {
  const context = useContext(CoordinateOriginContext);
  if (context === undefined) {
    throw new Error('useCoordinateOrigin must be used within a CoordinateOriginProvider');
  }
  return context;
}
