'use client';

import React, { createContext, useContext, useState, ReactNode } from 'react';
import { GeoCoordinates } from '../lib/ros/mapTile';

interface CoordinateOriginContextType {
  /** Geographic origin (lat/lon/alt) for the local ENU coordinate frame.
   *  All vehicle positions and zone polygons are expressed relative to this point.
   *  Null until initialized from first GPS fix or manual map selection. */
  origin: GeoCoordinates | null;
  /** Set the global coordinate origin. Typically called once on initial GPS fix
   *  or when the operator manually designates a map center point. */
  setOrigin: (origin: GeoCoordinates) => void;
}

const CoordinateOriginContext = createContext<CoordinateOriginContextType | undefined>(undefined);

/**
 * Provides the global coordinate origin for local ENU (East-North-Up) transforms.
 *
 * The GCS operates primarily in a local Cartesian frame for simplicity in
 * distance calculations and 3D visualization. This context holds the geographic
 * anchor point (lat/lon/altitude) that maps the local (x,y,z) coordinates to
 * real-world positions.
 *
 * Usage:
 * - Vehicle telemetry arrives in local coordinates from the ROS ecosystem
 * - Map tile fetching uses the origin to determine which tiles to load
 * - Zone polygons are stored in local coordinates relative to this origin
 */
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
