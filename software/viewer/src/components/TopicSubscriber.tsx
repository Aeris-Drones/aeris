'use client';

import { useEffect, useState, useCallback } from 'react';
import ROSLIB from 'roslib';

interface TopicSubscriberProps {
  ros: ROSLIB.Ros | null;
  topicName: string;
  messageType: string;
  isConnected: boolean;
}

interface MessageInfo {
  timestamp: string;
  data: unknown;
  count: number;
}

export function TopicSubscriber({ ros, topicName, messageType, isConnected }: TopicSubscriberProps) {
  const [lastMessage, setLastMessage] = useState<MessageInfo | null>(null);
  const [messageCount, setMessageCount] = useState(0);
  const [isSubscribed, setIsSubscribed] = useState(false);

  const handleMessage = useCallback((message: unknown) => {
    const timestamp = new Date().toISOString();
    const count = messageCount + 1;

    console.log(`[Topic: ${topicName}] Message #${count}:`, message);

    setLastMessage({
      timestamp,
      data: message,
      count,
    });
    setMessageCount(count);
  }, [topicName, messageCount]);

  useEffect(() => {
    if (!ros || !isConnected) {
      setIsSubscribed(false);
      return;
    }

    console.log(`[Topic: ${topicName}] Subscribing to topic...`);

    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
    });

    topic.subscribe(handleMessage);
    setIsSubscribed(true);

    console.log(`[Topic: ${topicName}] Successfully subscribed`);

    return () => {
      console.log(`[Topic: ${topicName}] Unsubscribing...`);
      topic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, topicName, messageType, isConnected, handleMessage]);

  return (
    <div className="absolute top-20 left-4 z-10 bg-black/70 backdrop-blur-sm rounded-lg px-4 py-3 text-white shadow-lg max-w-md">
      <div className="flex items-center gap-2 mb-2">
        <div className={`w-2 h-2 rounded-full ${isSubscribed ? 'bg-green-500' : 'bg-gray-500'}`} />
        <h3 className="font-semibold text-sm">Topic: {topicName}</h3>
      </div>

      <div className="text-xs space-y-1">
        <div className="flex justify-between">
          <span className="text-gray-400">Status:</span>
          <span className={isSubscribed ? 'text-green-400' : 'text-gray-400'}>
            {isSubscribed ? 'Subscribed' : 'Not subscribed'}
          </span>
        </div>

        <div className="flex justify-between">
          <span className="text-gray-400">Messages received:</span>
          <span className="text-white font-mono">{messageCount}</span>
        </div>

        {lastMessage && (
          <>
            <div className="flex justify-between">
              <span className="text-gray-400">Last update:</span>
              <span className="text-white font-mono text-[10px]">
                {new Date(lastMessage.timestamp).toLocaleTimeString()}
              </span>
            </div>

            <div className="mt-2 p-2 bg-gray-900/50 rounded text-[10px] font-mono max-h-32 overflow-auto">
              <pre className="whitespace-pre-wrap break-words">
                {JSON.stringify(lastMessage.data, null, 2)}
              </pre>
            </div>
          </>
        )}
      </div>
    </div>
  );
}
