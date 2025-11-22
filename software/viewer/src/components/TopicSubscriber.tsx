'use client';

import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from '@/hooks/useROSConnection';

export function TopicSubscriber() {
  const { ros, isConnected } = useROSConnection();
  const [messageCount, setMessageCount] = useState(0);

  useEffect(() => {
    if (!isConnected || !ros) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/clock',
      messageType: 'rosgraph_msgs/Clock',
    });

    const handleMessage = (message: unknown) => {
      setMessageCount((prev) => prev + 1);
      
      // Log ~1% of messages randomly to avoid spamming console
      if (Math.random() < 0.01) {
        console.log('Received /clock:', message);
      }
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [isConnected, ros]);

  if (!isConnected) return null;

  return (
    <div className="text-[10px] text-muted-foreground mt-2 font-mono">
      <div>Topic: /clock</div>
      <div>Msgs: {messageCount}</div>
    </div>
  );
}
