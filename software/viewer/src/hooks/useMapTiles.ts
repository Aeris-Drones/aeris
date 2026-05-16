'use client';

import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useCoordinateOrigin } from '@/context/CoordinateOriginContext';
import { MapStats, MapTileManager, TileData } from '@/lib/map/MapTileManager';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import { normalizeMapTileMessage } from '@/lib/ros/mapTilePayload';

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

const EMPTY_TILE_STATE = {
  connection: null as ROSLIB.Ros | null,
  tiles: [] as TileData[],
  stats: EMPTY_STATS,
};

export function useMapTiles(
  options: UseMapTilesOptions = {}
): UseMapTilesResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useSharedROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [manager] = useState(() => new MapTileManager(merged.maxTiles));
  const [tileState, setTileState] = useState(EMPTY_TILE_STATE);
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
      manager.clear();
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: merged.topicName,
      messageType: merged.messageType,
    });

    const handleMessage = (rawMessage: ROSLIB.Message) => {
      const normalizedMessage = normalizeMapTileMessage(rawMessage);
      if (!normalizedMessage) {
        console.warn('[useMapTiles] Ignoring invalid map tile payload');
        return;
      }

      const ingestResult = manager.ingest(normalizedMessage, originRef.current);
      if (!ingestResult) {
        return;
      }
      if (ingestResult.newOrigin && !originRef.current) {
        originRef.current = ingestResult.newOrigin;
        setOriginRef.current(ingestResult.newOrigin);
      }

      setTileState({
        connection: ros,
        tiles: manager.getTiles(),
        stats: manager.getStats(),
      });
    };

    topic.subscribe(handleMessage);
    return () => {
      topic.unsubscribe();
    };
  }, [isConnected, manager, merged.messageType, merged.topicName, ros]);

  return {
    tiles: isConnected && tileState.connection === ros ? tileState.tiles : [],
    stats: isConnected && tileState.connection === ros ? tileState.stats : EMPTY_STATS,
    isConnected,
  };
}
