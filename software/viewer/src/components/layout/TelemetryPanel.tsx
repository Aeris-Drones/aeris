'use client';

import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import { TopicSubscriber } from "@/components/TopicSubscriber";

export function TelemetryPanel() {
  // Mock metrics - will be replaced by real ROS data later
  const altitude = 45.2;
  const speed = 12.5;
  const heading = 270;

  const metrics = [
    { label: "Altitude", value: altitude.toFixed(1), unit: "m", status: "success" },
    { label: "Speed", value: speed.toFixed(1), unit: "m/s", status: "success" },
    { label: "Heading", value: heading, unit: "°", status: "success" },
    { label: "Climb Rate", value: "+1.2", unit: "m/s", status: "success" },
    { label: "Dist. to Home", value: "1240", unit: "m", status: "warning" },
  ];

  return (
    <div className="h-full w-[320px] flex flex-col gap-4 p-4 bg-background/50 backdrop-blur-sm border-l overflow-y-auto">
      <Card className="bg-secondary/40 border-secondary/50">
        <CardHeader className="pb-2">
          <CardTitle className="text-xs uppercase tracking-wider text-muted-foreground">Critical Telemetry</CardTitle>
        </CardHeader>
        <CardContent className="grid grid-cols-1 gap-4">
            {metrics.slice(0, 3).map((m, i) => (
                <div key={i} className="flex justify-between items-end border-b border-white/5 pb-2 last:border-0 last:pb-0">
                    <span className="text-sm text-muted-foreground">{m.label}</span>
                    <div className="flex items-center gap-2">
                        <div className={cn("w-1.5 h-1.5 rounded-full", 
                            m.status === 'success' ? 'bg-success' : 
                            m.status === 'warning' ? 'bg-warning' : 'bg-danger'
                        )} />
                        <span className="text-xl font-mono font-medium text-foreground">{m.value} <span className="text-xs text-muted-foreground">{m.unit}</span></span>
                    </div>
                </div>
            ))}
        </CardContent>
      </Card>

      <Card className="bg-secondary/40 border-secondary/50 flex-1">
         <CardHeader className="pb-2">
          <CardTitle className="text-xs uppercase tracking-wider text-muted-foreground">Secondary Metrics</CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
            {metrics.slice(3).map((m, i) => (
                <div key={i} className="flex justify-between items-center">
                     <span className="text-sm text-muted-foreground">{m.label}</span>
                     <span className="font-mono text-foreground">{m.value} <span className="text-xs text-muted-foreground">{m.unit}</span></span>
                </div>
            ))}
            
            <div className="pt-4">
                 <div className="text-xs uppercase tracking-wider text-muted-foreground mb-2">Debug Info</div>
                 <TopicSubscriber />
            </div>
        </CardContent>
      </Card>
    </div>
  );
}
