import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { MapTileManager, TileData, MapStats } from '../lib/map/MapTileManager';
import { MapTileMessage } from '../lib/ros/mapTile';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';

export function useMapTiles() {
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [tiles, setTiles] = useState<TileData[]>([]);
  const [stats, setStats] = useState<MapStats>({ count: 0, totalBytes: 0 });

  const managerRef = useRef<MapTileManager>(new MapTileManager(500));
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);

  useEffect(() => {
    originRef.current = origin;
    setOriginRef.current = setOrigin;
  }, [origin, setOrigin]);

  useEffect(() => {
    if (!ros || !isConnected) return;

    const manager = managerRef.current;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/map/tiles',
      messageType: 'aeris_msgs/MapTile',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      const result = manager.ingest(message as unknown as MapTileMessage, originRef.current);
      if (result) {
        if (result.newOrigin) {
          setOriginRef.current(result.newOrigin);
        }
        setTiles([...manager.getTiles()]);
        setStats(manager.getStats());
      }
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
      manager.clear();
    };
  }, [ros, isConnected]);

  return { tiles, stats };
}
