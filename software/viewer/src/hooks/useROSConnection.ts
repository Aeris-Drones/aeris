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

export function useROSConnection(options: ROSConnectionOptions = {}): ROSConnectionResult {
  // Destructure options with defaults to avoid object instability
  const url = options.url || DEFAULT_OPTIONS.url;
  const autoConnect = options.autoConnect ?? DEFAULT_OPTIONS.autoConnect;
  const maxRetries = options.maxRetries ?? DEFAULT_OPTIONS.maxRetries;
  const initialRetryDelay = options.initialRetryDelay ?? DEFAULT_OPTIONS.initialRetryDelay;
  const maxRetryDelay = options.maxRetryDelay ?? DEFAULT_OPTIONS.maxRetryDelay;

  const [state, setState] = useState<ConnectionState>('disconnected');
  const [error, setError] = useState<string | null>(null);
  const [rosInstance, setRosInstance] = useState<ROSLIB.Ros | null>(null);
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const retryCountRef = useRef(0);
  const retryTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isConnectingRef = useRef(false);
  const stateRef = useRef(state);
  
  // Ref to allow recursion in connect
  const connectRef = useRef<() => void>(() => {});

  // Keep stateRef in sync
  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  // Calculate exponential backoff delay
  const getRetryDelay = useCallback(() => {
    const delay = Math.min(
      initialRetryDelay * Math.pow(2, retryCountRef.current),
      maxRetryDelay
    );
    return delay;
  }, [initialRetryDelay, maxRetryDelay]);

  // Connect to ROS bridge
  const connect = useCallback(() => {
    // Prevent multiple simultaneous connection attempts using refs for stability
    if (isConnectingRef.current || stateRef.current === 'connected') {
      return;
    }

    isConnectingRef.current = true;
    setState('connecting');
    setError(null);
    console.log(`[ROS] Connecting to ${url}...`);

    // Create new ROS connection
    const ros = new ROSLIB.Ros({
      url: url,
    });

    ros.on('connection', () => {
      console.log('[ROS] Connected successfully');
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

      // Schedule retry if we haven't exceeded max retries
      if (retryCountRef.current < maxRetries) {
        const delay = getRetryDelay();
        console.log(`[ROS] Retrying in ${delay}ms (attempt ${retryCountRef.current + 1}/${maxRetries})`);

        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          // Use ref for recursive call
          connectRef.current();
        }, delay);
      } else {
        console.error(`[ROS] Max retries (${maxRetries}) exceeded. Giving up.`);
      }
    });

    ros.on('close', () => {
      console.log('[ROS] Connection closed');
      setState('disconnected');
      isConnectingRef.current = false;

      // Auto-reconnect on unexpected disconnection
      if (retryCountRef.current < maxRetries) {
        const delay = getRetryDelay();
        console.log(`[ROS] Auto-reconnecting in ${delay}ms...`);

        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          // Use ref for recursive call
          connectRef.current();
        }, delay);
      }
    });

    rosRef.current = ros;
    setRosInstance(ros);
  }, [url, maxRetries, getRetryDelay]); // Dependencies are now stable primitives

  // Update connectRef whenever connect changes
  useEffect(() => {
    connectRef.current = connect;
  }, [connect]);

  // Disconnect from ROS bridge
  const disconnect = useCallback(() => {
    console.log('[ROS] Disconnecting...');

    // Clear any pending retry timeouts
    if (retryTimeoutRef.current) {
      clearTimeout(retryTimeoutRef.current);
      retryTimeoutRef.current = null;
    }

    // Close ROS connection
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

  // Auto-connect on mount if enabled
  useEffect(() => {
    let timer: NodeJS.Timeout;
    
    if (autoConnect) {
      // Defer connection to next tick to avoid synchronous state update warning
      timer = setTimeout(() => {
        connect();
      }, 0);
    }

    // Cleanup on unmount
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
