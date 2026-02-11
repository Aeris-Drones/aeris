'use client';

import { useEffect, useState, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';

export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';

export interface ROSConnectionOptions {
  url?: string;
  autoConnect?: boolean;
  maxRetries?: number;
  initialRetryDelay?: number;
  maxRetryDelay?: number;
}

export interface ROSConnectionResult {
  ros: ROSLIB.Ros | null;
  state: ConnectionState;
  error: string | null;
  connect: () => void;
  disconnect: () => void;
  isConnected: boolean;
}

const DEFAULT_OPTIONS: Required<ROSConnectionOptions> = {
  url: 'ws://localhost:9090',
  autoConnect: true,
  maxRetries: 10,
  initialRetryDelay: 1000,
  maxRetryDelay: 30000,
};

/**
 * React hook for managing ROS (Robot Operating System) WebSocket connections.
 *
 * Features:
 * - Automatic reconnection with exponential backoff (capped at maxRetryDelay)
 * - Connection state tracking for UI feedback
 * - Manual connect/disconnect controls
 * - Cleanup on unmount
 *
 * Exponential backoff formula: delay = min(initialDelay * 2^retryCount, maxDelay)
 */
export function useROSConnection(options: ROSConnectionOptions = {}): ROSConnectionResult {
  const url = options.url || DEFAULT_OPTIONS.url;
  const autoConnect = options.autoConnect ?? DEFAULT_OPTIONS.autoConnect;
  const maxRetries = options.maxRetries ?? DEFAULT_OPTIONS.maxRetries;
  const initialRetryDelay = options.initialRetryDelay ?? DEFAULT_OPTIONS.initialRetryDelay;
  const maxRetryDelay = options.maxRetryDelay ?? DEFAULT_OPTIONS.maxRetryDelay;

  const [state, setState] = useState<ConnectionState>('disconnected');
  const [error, setError] = useState<string | null>(null);
  const [rosInstance, setRosInstance] = useState<ROSLIB.Ros | null>(null);

  // Refs for accessing current state in callbacks without stale closures
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const retryCountRef = useRef(0);
  const retryTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isConnectingRef = useRef(false);
  const stateRef = useRef(state);
  const connectRef = useRef<() => void>(() => {});

  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  /**
   * Calculates the next retry delay using exponential backoff.
   *
   * Formula: min(initialDelay * 2^retryCount, maxDelay)
   * Example with defaults: 1s, 2s, 4s, 8s, 16s, 30s, 30s...
   */
  const getRetryDelay = useCallback(() => {
    const delay = Math.min(
      initialRetryDelay * Math.pow(2, retryCountRef.current),
      maxRetryDelay
    );
    return delay;
  }, [initialRetryDelay, maxRetryDelay]);

  /**
   * Initiates a ROS connection.
   *
   * Guards against concurrent connection attempts. Sets up event handlers
   * for connection success, errors, and close events. On error/close,
   * schedules reconnection attempts up to maxRetries.
   */
  const connect = useCallback(() => {
    if (isConnectingRef.current || stateRef.current === 'connected') {
      return;
    }

    isConnectingRef.current = true;
    setState('connecting');
    setError(null);

    const ros = new ROSLIB.Ros({
      url: url,
    });

    ros.on('connection', () => {
      setState('connected');
      setError(null);
      retryCountRef.current = 0;
      isConnectingRef.current = false;
    });

    ros.on('error', (err: Error) => {
      const errorMessage = err.message || 'Unknown error';
      console.error('[ROS] Connection error:', errorMessage);
      setError(errorMessage);
      setState('error');
      isConnectingRef.current = false;

      if (retryCountRef.current < maxRetries) {
        const delay = getRetryDelay();
        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          connectRef.current();
        }, delay);
      }
    });

    ros.on('close', () => {
      setState('disconnected');
      isConnectingRef.current = false;

      if (retryCountRef.current < maxRetries) {
        const delay = getRetryDelay();
        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          connectRef.current();
        }, delay);
      }
    });

    rosRef.current = ros;
    setRosInstance(ros);
  }, [url, maxRetries, getRetryDelay]);

  useEffect(() => {
    connectRef.current = connect;
  }, [connect]);

  /**
   * Cleanly disconnects from ROS and cancels any pending reconnection.
   *
   * Clears retry timeout, closes the WebSocket, and resets all state refs.
   */
  const disconnect = useCallback(() => {
    if (retryTimeoutRef.current) {
      clearTimeout(retryTimeoutRef.current);
      retryTimeoutRef.current = null;
    }

    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }

    setRosInstance(null);
    setState('disconnected');
    setError(null);
    retryCountRef.current = 0;
    isConnectingRef.current = false;
  }, []);

  /**
   * Auto-connect effect: initiates connection on mount if enabled.
   *
   * Uses setTimeout(0) to defer connection until after initial render.
   * Cleans up by disconnecting on unmount to prevent memory leaks.
   */
  useEffect(() => {
    let timer: NodeJS.Timeout;

    if (autoConnect) {
      timer = setTimeout(() => {
        connect();
      }, 0);
    }

    return () => {
      if (timer) {
        clearTimeout(timer);
      }
      disconnect();
    };
  }, [connect, disconnect, autoConnect]);

  return {
    ros: rosInstance,
    state,
    error,
    connect,
    disconnect,
    isConnected: state === 'connected',
  };
}
