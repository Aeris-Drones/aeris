'use client';

import { useEffect, useMemo } from 'react';
import { useThermalPerception } from './useThermalPerception';
import { useAcousticPerception } from './useAcousticPerception';
import { useGasPerception } from './useGasPerception';
import { useCoordinateOrigin } from '@/context/CoordinateOriginContext';
import { geoToLocal } from '@/lib/ros/mapTile';
import { useDetectionContext } from '@/context/DetectionContext';
import type {
  Detection,
  ThermalDetection,
  AcousticDetection,
  GasDetection,
} from '@/types/detection';

/**
 * Unified detection aggregation hook
 *
 * Pulls from thermal, acoustic, and gas perception hooks,
 * transforms them into Detection objects, and feeds them to DetectionContext
 */
export function useDetections() {
  const { origin } = useCoordinateOrigin();
  const { setDetections, detections: currentDetections } = useDetectionContext();

  // Get sensor data
  const thermalHotspots = useThermalPerception();
  const { detections: acousticDetections } = useAcousticPerception();
  const { plumes } = useGasPerception();

  // Aggregate all detections
  const aggregatedDetections = useMemo(() => {
    const detections: Detection[] = [];

    // Transform thermal hotspots
    thermalHotspots.forEach((hotspot) => {
      const existingDetection = currentDetections.find(
        (d) => d.id === `thermal-${hotspot.id}`
      ) as ThermalDetection | undefined;

      const localPosition = origin
        ? geoToLocal(
            { lat: hotspot.latitude, lon: hotspot.longitude },
            origin
          )
        : undefined;

      const detection: ThermalDetection = {
        id: `thermal-${hotspot.id}`,
        sensorType: 'thermal',
        confidence: hotspot.confidence,
        timestamp: existingDetection?.timestamp ?? hotspot.lastUpdate,
        lastUpdate: hotspot.lastUpdate,
        location: {
          latitude: hotspot.latitude,
          longitude: hotspot.longitude,
          altitude: hotspot.altitude,
        },
        localPosition: localPosition
          ? {
              x: localPosition.x,
              y: hotspot.altitude,
              z: localPosition.z,
            }
          : undefined,
        status: existingDetection?.status ?? 'new',
        notes: existingDetection?.notes,
        temperature: hotspot.temp_c,
      };

      detections.push(detection);
    });

    // Transform acoustic detections
    acousticDetections.forEach((acoustic, vehicleId) => {
      const existingDetection = currentDetections.find(
        (d) => d.id === `acoustic-${vehicleId}`
      ) as AcousticDetection | undefined;

      // Acoustic detections don't have absolute position, only bearing from vehicle
      // For now, we'll leave localPosition undefined until we have vehicle position
      const detection: AcousticDetection = {
        id: `acoustic-${vehicleId}`,
        sensorType: 'acoustic',
        confidence: acoustic.confidence,
        timestamp: existingDetection?.timestamp ?? acoustic.timestamp,
        lastUpdate: acoustic.timestamp,
        location: {
          latitude: 0, // TODO: Calculate from vehicle position + bearing
          longitude: 0,
          altitude: 0,
        },
        localPosition: undefined, // TODO: Calculate from vehicle position
        status: existingDetection?.status ?? 'new',
        notes: existingDetection?.notes,
        vehicleId: acoustic.vehicle_id,
        bearing: acoustic.bearing_deg,
        snr: acoustic.snr_db,
        classification: mapAcousticClassification(acoustic.classification),
      };

      detections.push(detection);
    });

    // Transform gas plumes
    plumes.forEach((plume, index) => {
      const plumeId = `gas-${plume.species}-${index}`;
      const existingDetection = currentDetections.find(
        (d) => d.id === plumeId
      ) as GasDetection | undefined;

      // Use centerline center point as detection location
      let localPosition: { x: number; y: number; z: number } | undefined;
      const geoLocation = { latitude: 0, longitude: 0, altitude: 0 };

      if (plume.centerline.length > 0) {
        const centerIndex = Math.floor(plume.centerline.length / 2);
        const centerPoint = plume.centerline[centerIndex];

        localPosition = {
          x: centerPoint.x,
          y: centerPoint.y,
          z: centerPoint.z,
        };

        // TODO: Convert local back to geo if needed
        // For now, we'll use placeholder values
      }

      // Calculate approximate concentration from plume data
      // In real implementation, this would come from the gas sensor
      const concentration = 100; // Placeholder

      const detection: GasDetection = {
        id: plumeId,
        sensorType: 'gas',
        confidence: 0.7, // Placeholder - gas plumes don't have explicit confidence
        timestamp: existingDetection?.timestamp ?? plume.timestamp,
        lastUpdate: plume.timestamp,
        location: geoLocation,
        localPosition,
        status: existingDetection?.status ?? 'new',
        notes: existingDetection?.notes,
        species: plume.species,
        concentration,
        units: plume.units,
        windDirection: undefined, // TODO: Add from wind data
        windSpeed: undefined,
      };

      detections.push(detection);
    });

    return detections;
  }, [
    thermalHotspots,
    acousticDetections,
    plumes,
    origin,
    currentDetections,
  ]);

  // Update detection context
  useEffect(() => {
    setDetections(aggregatedDetections);
  }, [aggregatedDetections, setDetections]);

  return {
    detections: aggregatedDetections,
    thermalCount: thermalHotspots.length,
    acousticCount: acousticDetections.size,
    gasCount: plumes.length,
  };
}

/**
 * Map acoustic classification string to typed enum
 */
function mapAcousticClassification(
  classification: string
): AcousticDetection['classification'] {
  const lower = classification.toLowerCase();
  if (lower.includes('vocal') || lower.includes('voice')) return 'vocal';
  if (lower.includes('tap')) return 'tapping';
  if (lower.includes('mech')) return 'mechanical';
  return 'unknown';
}

/**
 * Hook to get detection counts by type (for status bar)
 */
export function useDetectionCounts() {
  const { stats } = useDetectionContext();

  return {
    total: stats.total,
    thermal: stats.byType.thermal,
    acoustic: stats.byType.acoustic,
    gas: stats.byType.gas,
    new: stats.byStatus.new,
    confirmed: stats.byStatus.confirmed,
    highConfidence: stats.byConfidence.high,
  };
}

/**
 * Hook to get new detections (for notifications/alerts)
 */
export function useNewDetections() {
  const { filteredDetections } = useDetectionContext();

  return useMemo(() => {
    return filteredDetections.filter((d) => d.status === 'new');
  }, [filteredDetections]);
}

/**
 * Hook to get confirmed detections (for mission summary)
 */
export function useConfirmedDetections() {
  const { filteredDetections } = useDetectionContext();

  return useMemo(() => {
    return filteredDetections.filter((d) => d.status === 'confirmed');
  }, [filteredDetections]);
}
