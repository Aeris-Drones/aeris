'use client';

import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useCoordinateOrigin } from '@/context/CoordinateOriginContext';
import { MapStats, MapTileManager, TileData } from '@/lib/map/MapTileManager';
import type { MapTileMessage } from '@/lib/ros/mapTile';
import { useROSConnection } from './useROSConnection';

interface UseMapTilesOptions {
  topicName?: string;
  messageType?: string;
  maxTiles?: number;
}

interface UseMapTilesResult {
  tiles: TileData[];
  stats: MapStats;
  isConnected: boolean;
}

const DEFAULT_OPTIONS: Required<UseMapTilesOptions> = {
  topicName: '/map/tiles',
  messageType: 'aeris_msgs/MapTile',
  maxTiles: 500,
};

const EMPTY_STATS: MapStats = {
  count: 0,
  totalBytes: 0,
  latencyP95Ms: null,
  lastLatencyMs: null,
};

export function useMapTiles(
  options: UseMapTilesOptions = {}
): UseMapTilesResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [manager] = useState(() => new MapTileManager(merged.maxTiles));
  const [tiles, setTiles] = useState<TileData[]>([]);
  const [stats, setStats] = useState<MapStats>(EMPTY_STATS);
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);

  useEffect(() => {
    originRef.current = origin;
    setOriginRef.current = setOrigin;
  }, [origin, setOrigin]);

  useEffect(() => {
    return () => {
      manager.clear();
    };
  }, [manager]);

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: merged.topicName,
      messageType: merged.messageType,
    });

    const handleMessage = (rawMessage: ROSLIB.Message) => {
      if (!isValidMapTileMessage(rawMessage)) {
        console.warn('[useMapTiles] Ignoring invalid map tile payload');
        return;
      }

      const ingestResult = manager.ingest(rawMessage, originRef.current);
      if (!ingestResult) {
        return;
      }
      if (ingestResult.newOrigin && !originRef.current) {
        originRef.current = ingestResult.newOrigin;
        setOriginRef.current(ingestResult.newOrigin);
      }

      setTiles(manager.getTiles());
      setStats(manager.getStats());
    };

    topic.subscribe(handleMessage);
    return () => {
      topic.unsubscribe();
    };
  }, [isConnected, manager, merged.messageType, merged.topicName, ros]);

  return { tiles, stats, isConnected };
}

function isValidMapTileMessage(value: unknown): value is MapTileMessage {
  if (!value || typeof value !== 'object') {
    return false;
  }
  const message = value as Partial<MapTileMessage>;
  return (
    typeof message.tile_id === 'string' &&
    message.tile_id.trim().length > 0 &&
    typeof message.format === 'string' &&
    Number.isFinite(Number(message.byte_size)) &&
    Number(message.byte_size) >= 0
  );
}
