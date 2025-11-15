'use client';

import { Scene3D } from '@/components/Scene3D';
import { ConnectionStatus } from '@/components/ConnectionStatus';
import { TopicSubscriber } from '@/components/TopicSubscriber';
import { ErrorBoundary } from '@/components/ErrorBoundary';
import { useROSConnection } from '@/hooks/useROSConnection';

export default function Home() {
  const { ros, state, error, connect, isConnected } = useROSConnection({
    url: 'ws://localhost:9090',
    autoConnect: true,
  });

  return (
    <ErrorBoundary>
      <div className="relative w-screen h-screen overflow-hidden bg-black">
        {/* Connection Status Indicator */}
        <ConnectionStatus
          state={state}
          error={error}
          onReconnect={connect}
        />

        {/* Test Topic Subscriber */}
        <TopicSubscriber
          ros={ros}
          topicName="/clock"
          messageType="rosgraph_msgs/Clock"
          isConnected={isConnected}
        />

        {/* 3D Scene */}
        <Scene3D showStats={true} showGrid={true} />

        {/* Info overlay */}
        <div className="absolute bottom-4 right-4 z-10 bg-black/70 backdrop-blur-sm rounded-lg px-4 py-3 text-white shadow-lg text-sm">
          <h2 className="font-bold mb-2">Aeris Viewer</h2>
          <ul className="space-y-1 text-xs text-gray-300">
            <li>• Orbit: Left mouse drag</li>
            <li>• Pan: Right mouse drag</li>
            <li>• Zoom: Scroll wheel</li>
          </ul>
        </div>
      </div>
    </ErrorBoundary>
  );
}
