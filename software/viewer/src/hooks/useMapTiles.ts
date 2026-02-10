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
  const [stats, setStats] = useState<MapStats>({
    count: 0,
    totalBytes: 0,
    latencyP95Ms: null,
    lastLatencyMs: null,
  });

  const managerRef = useRef<MapTileManager>(new MapTileManager(500));
  const serviceCacheRef = useRef<Map<string, ROSLIB.Service>>(new Map());
  const rosNowMsRef = useRef<number | null>(null);
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);

  useEffect(() => {
    originRef.current = origin;
    setOriginRef.current = setOrigin;
  }, [origin, setOrigin]);

  useEffect(() => {
    if (!ros || !isConnected) return;

    const manager = managerRef.current;
    const serviceCache = serviceCacheRef.current;
    let active = true;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/map/tiles',
      messageType: 'aeris_msgs/MapTile',
    });
    const clockTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/clock',
      messageType: 'rosgraph_msgs/Clock',
    });
    const handleClock = (message: ROSLIB.Message) => {
      const payload = message as unknown as RosClockMessage;
      if (!payload.clock) {
        return;
      }
      rosNowMsRef.current = (payload.clock.sec * 1000) + (payload.clock.nanosec / 1e6);
    };
    clockTopic.subscribe(handleClock);

    const handleMessage = async (message: ROSLIB.Message) => {
      const tileMsg = message as unknown as MapTileMessage;
      const serviceName = getFetchServiceName(tileMsg.layer_ids);
      let tileBytesService = serviceCache.get(serviceName);
      if (!tileBytesService) {
        tileBytesService = new ROSLIB.Service({
          ros,
          name: serviceName,
          serviceType: 'aeris_msgs/GetMapTileBytes',
        });
        serviceCache.set(serviceName, tileBytesService);
      }

      const msgWithData = await hydrateTileData(tileBytesService, tileMsg);
      if (!msgWithData || !active) {
        return;
      }
      const publishedAt = msgWithData.published_at;
      const publishedAtMs = publishedAt
        ? (publishedAt.sec * 1000 + publishedAt.nanosec / 1e6)
        : null;
      const nowMs = publishedAtMs === null
        ? null
        : resolveNowMs(publishedAtMs, rosNowMsRef.current);
      const latencyMs = (publishedAtMs !== null && nowMs !== null)
        ? Math.max(0, nowMs - publishedAtMs)
        : undefined;

      const result = manager.ingest(msgWithData, originRef.current, latencyMs);
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
      active = false;
      topic.unsubscribe();
      clockTopic.unsubscribe();
      serviceCache.clear();
      rosNowMsRef.current = null;
      manager.clear();
    };
  }, [ros, isConnected]);

  return { tiles, stats };
}

interface GetMapTileBytesResponse {
  found: boolean;
  content_type: string;
  hash_sha256: string;
  byte_size: number;
  data: number[];
  published_at: {
    sec: number;
    nanosec: number;
  };
}

interface RosClockMessage {
  clock: {
    sec: number;
    nanosec: number;
  };
}

const TILE_SERVICE_TIMEOUT_MS = 5000;
const WALL_CLOCK_EPOCH_THRESHOLD_MS = 1_000_000_000_000;

function callTileByteService(
  service: ROSLIB.Service,
  tileId: string,
  timeoutMs: number = TILE_SERVICE_TIMEOUT_MS
): Promise<GetMapTileBytesResponse | null> {
  return new Promise((resolve) => {
    let resolved = false;
    const finish = (value: GetMapTileBytesResponse | null) => {
      if (resolved) {
        return;
      }
      resolved = true;
      clearTimeout(timeoutHandle);
      resolve(value);
    };

    const timeoutHandle: ReturnType<typeof setTimeout> = setTimeout(() => {
      finish(null);
    }, timeoutMs);

    const request = new ROSLIB.ServiceRequest({ tile_id: tileId });
    const invocable = service as unknown as RoslibServiceInvoker;
    invocable.callService(
      request,
      (response) => finish(response as unknown as GetMapTileBytesResponse),
      () => finish(null)
    );
  });
}

async function hydrateTileData(
  service: ROSLIB.Service,
  message: MapTileMessage
): Promise<MapTileMessage | null> {
  if (message.data) {
    return message;
  }

  const response = await callTileByteService(service, message.tile_id);
  if (!response || !response.found || !Array.isArray(response.data) || response.data.length === 0) {
    console.warn('Tile hydration failed or returned empty data for', message.tile_id);
    return null;
  }

  const bytes = new Uint8Array(response.data);
  return {
    ...message,
    byte_size: response.byte_size || message.byte_size,
    hash_sha256: response.hash_sha256 || message.hash_sha256,
    published_at: response.published_at,
    data: uint8ToBase64(bytes),
  };
}

function uint8ToBase64(bytes: Uint8Array): string {
  let binary = '';
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    const part = bytes.subarray(i, i + chunk);
    binary += String.fromCharCode(...part);
  }
  return btoa(binary);
}

interface RoslibServiceInvoker {
  callService(
    request: ROSLIB.ServiceRequest,
    callback: (response: unknown) => void,
    failedCallback?: () => void
  ): void;
}

function getFetchServiceName(layerIds: string[]): string {
  const fetchEntry = layerIds.find((entry) => entry.startsWith('fetch-service:'));
  if (!fetchEntry) {
    return '/map/get_tile_bytes';
  }
  const value = fetchEntry.slice('fetch-service:'.length).trim();
  return value.length > 0 ? value : '/map/get_tile_bytes';
}

function resolveNowMs(publishedAtMs: number, rosNowMs: number | null): number | null {
  if (typeof rosNowMs === 'number' && Number.isFinite(rosNowMs)) {
    return rosNowMs;
  }
  if (publishedAtMs >= WALL_CLOCK_EPOCH_THRESHOLD_MS) {
    return Date.now();
  }
  return null;
}
