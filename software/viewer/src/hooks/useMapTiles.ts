import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { MapTileManager, TileData, MapStats } from '../lib/map/MapTileManager';
import { MapTileMessage } from '../lib/ros/mapTile';

export function useMapTiles() {
  const { ros, isConnected } = useROSConnection();
  const [tiles, setTiles] = useState<TileData[]>([]);
  const [stats, setStats] = useState<MapStats>({ count: 0, totalBytes: 0 });

  // Use a ref for the manager to persist across renders
  const managerRef = useRef<MapTileManager | null>(null);
  if (!managerRef.current) {
    managerRef.current = new MapTileManager(500);
  }

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/map/tiles',
      messageType: 'aeris_msgs/MapTile',
    });

    const handleMessage = (message: MapTileMessage) => {
      if (managerRef.current) {
        managerRef.current.ingest(message);
        // Throttle updates or just update?
        // For 60 FPS with progressive updates, React state updates might be expensive if too frequent.
        // But typical tile arrival rate is not 60Hz. Probably bursty.
        // We'll update state directly for now.
        setTiles([...managerRef.current.getTiles()]);
        setStats(managerRef.current.getStats());
      }
    };

    topic.subscribe(handleMessage);
    console.log('[useMapTiles] Subscribed to /map/tiles');

    return () => {
      topic.unsubscribe();
      // We don't clear the manager on unmount/remount strictly unless we want to reset the map.
      // But if the component unmounts, we usually want to cleanup resources (revoke URLs).
      // If we want to persist while navigating, we'd move manager to a Context.
      // For this story, we'll assume simple usage within Scene3D which persists.
      // If we navigate away, cleanup:
      if (managerRef.current) {
        managerRef.current.clear();
      }
      console.log('[useMapTiles] Unsubscribed');
    };
  }, [ros, isConnected]);

  return { tiles, stats };
}
