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
  const opts = { ...DEFAULT_OPTIONS, ...options };

  const [state, setState] = useState<ConnectionState>('disconnected');
  const [error, setError] = useState<string | null>(null);
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const retryCountRef = useRef(0);
  const retryTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isConnectingRef = useRef(false);

  // Calculate exponential backoff delay
  const getRetryDelay = useCallback(() => {
    const delay = Math.min(
      opts.initialRetryDelay * Math.pow(2, retryCountRef.current),
      opts.maxRetryDelay
    );
    return delay;
  }, [opts.initialRetryDelay, opts.maxRetryDelay]);

  // Connect to ROS bridge
  const connect = useCallback(() => {
    // Prevent multiple simultaneous connection attempts
    if (isConnectingRef.current || state === 'connected') {
      return;
    }

    isConnectingRef.current = true;
    setState('connecting');
    setError(null);
    console.log(`[ROS] Connecting to ${opts.url}...`);

    // Create new ROS connection
    const ros = new ROSLIB.Ros({
      url: opts.url,
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
      if (retryCountRef.current < opts.maxRetries) {
        const delay = getRetryDelay();
        console.log(`[ROS] Retrying in ${delay}ms (attempt ${retryCountRef.current + 1}/${opts.maxRetries})`);

        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          connect();
        }, delay);
      } else {
        console.error(`[ROS] Max retries (${opts.maxRetries}) exceeded. Giving up.`);
      }
    });

    ros.on('close', () => {
      console.log('[ROS] Connection closed');
      setState('disconnected');
      isConnectingRef.current = false;

      // Auto-reconnect on unexpected disconnection
      if (retryCountRef.current < opts.maxRetries) {
        const delay = getRetryDelay();
        console.log(`[ROS] Auto-reconnecting in ${delay}ms...`);

        retryTimeoutRef.current = setTimeout(() => {
          retryCountRef.current++;
          connect();
        }, delay);
      }
    });

    rosRef.current = ros;
  }, [opts.url, opts.maxRetries, state, getRetryDelay]);

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

    setState('disconnected');
    setError(null);
    retryCountRef.current = 0;
    isConnectingRef.current = false;
  }, []);

  // Auto-connect on mount if enabled
  useEffect(() => {
    if (opts.autoConnect) {
      connect();
    }

    // Cleanup on unmount
    return () => {
      disconnect();
    };
  }, []); // Empty dependency array - only run on mount/unmount

  return {
    ros: rosRef.current,
    state,
    error,
    connect,
    disconnect,
    isConnected: state === 'connected',
  };
}
