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
  const managerRef = useRef<MapTileManager>(new MapTileManager(500));

  useEffect(() => {
    if (!ros || !isConnected) return;

    const manager = managerRef.current;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/map/tiles',
      messageType: 'aeris_msgs/MapTile',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      const result = manager.ingest(message as unknown as MapTileMessage);
      if (result) {
        setTiles([...manager.getTiles()]);
        setStats(manager.getStats());
      }
    };

    topic.subscribe(handleMessage);
    console.log('[useMapTiles] Subscribed to /map/tiles');

    return () => {
      topic.unsubscribe();
      manager.clear();
      console.log('[useMapTiles] Unsubscribed');
    };
  }, [ros, isConnected]);

  return { tiles, stats };
}
