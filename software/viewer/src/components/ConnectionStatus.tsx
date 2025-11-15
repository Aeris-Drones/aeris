'use client';

import { ConnectionState } from '@/hooks/useROSConnection';

interface ConnectionStatusProps {
  state: ConnectionState;
  error: string | null;
  onReconnect?: () => void;
}

const stateColors: Record<ConnectionState, string> = {
  disconnected: 'bg-gray-500',
  connecting: 'bg-yellow-500',
  connected: 'bg-green-500',
  error: 'bg-red-500',
};

const stateLabels: Record<ConnectionState, string> = {
  disconnected: 'Disconnected',
  connecting: 'Connecting...',
  connected: 'Connected',
  error: 'Connection Error',
};

export function ConnectionStatus({ state, error, onReconnect }: ConnectionStatusProps) {
  const colorClass = stateColors[state];
  const label = stateLabels[state];

  return (
    <div className="absolute top-4 left-4 z-10 bg-black/70 backdrop-blur-sm rounded-lg px-4 py-3 text-white shadow-lg">
      <div className="flex items-center gap-3">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${colorClass} ${state === 'connecting' ? 'animate-pulse' : ''}`} />
          <span className="font-medium">{label}</span>
        </div>

        {state === 'error' && onReconnect && (
          <button
            onClick={onReconnect}
            className="ml-2 px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded text-sm font-medium transition-colors"
          >
            Retry
          </button>
        )}
      </div>

      {error && (
        <div className="mt-2 text-sm text-red-300 max-w-sm">
          {error}
        </div>
      )}
    </div>
  );
}
