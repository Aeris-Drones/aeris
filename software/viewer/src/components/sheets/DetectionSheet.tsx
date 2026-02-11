'use client';

import { useState } from 'react';
import {
  Drawer,
  DrawerContent,
  DrawerTrigger,
} from '@/components/ui/drawer';
import { DetectionCard, Detection } from './DetectionCard';
import { cn } from '@/lib/utils';

type FilterTab = 'all' | 'pending' | 'confirmed' | 'dismissed';

interface DetectionSheetProps {
  detections: Detection[];
  onConfirm: (id: string) => void;
  onDismiss: (id: string) => void;
  onLocate: (id: string) => void;
  trigger: React.ReactNode;
}

export function DetectionSheet({
  detections,
  onConfirm,
  onDismiss,
  onLocate,
  trigger,
}: DetectionSheetProps) {
  const [activeFilter, setActiveFilter] = useState<FilterTab>('all');

  const counts = {
    all: detections.length,
    pending: detections.filter(d => d.status === 'new' || d.status === 'reviewing').length,
    confirmed: detections.filter(d => d.status === 'confirmed').length,
    dismissed: detections.filter(d => d.status === 'dismissed').length,
  };

  const filteredDetections = detections.filter((d) => {
    if (activeFilter === 'all') return true;
    if (activeFilter === 'pending') return d.status === 'new' || d.status === 'reviewing';
    if (activeFilter === 'confirmed') return d.status === 'confirmed';
    if (activeFilter === 'dismissed') return d.status === 'dismissed';
    return true;
  });

  const tabs: FilterTab[] = ['all', 'pending', 'confirmed', 'dismissed'];

  return (
    <Drawer>
      <DrawerTrigger asChild>
        <div className="cursor-pointer">{trigger}</div>
      </DrawerTrigger>
      <DrawerContent className="max-h-[80vh] bg-[#0c0c0e] border-white/5">
        <div className="mx-auto w-full max-w-xl">
          <div className="flex items-center justify-between px-4 py-3 border-b border-white/5">
            <h2 className="text-sm font-medium text-white">Detections</h2>
            <span className="text-xs text-white/40">
              {counts.pending} pending
            </span>
          </div>

          <div className="flex border-b border-white/5">
            {tabs.map((tab) => (
              <button
                key={tab}
                onClick={() => setActiveFilter(tab)}
                className={cn(
                  'flex-1 py-2.5 text-xs font-medium transition-colors',
                  activeFilter === tab
                    ? 'text-white border-b-2 border-white'
                    : 'text-white/40 hover:text-white/60'
                )}
              >
                {tab.charAt(0).toUpperCase() + tab.slice(1)}
                <span className="ml-1.5 text-white/30">{counts[tab]}</span>
              </button>
            ))}
          </div>

          <div className="max-h-[55vh] overflow-y-auto">
            {filteredDetections.length === 0 ? (
              <div className="py-12 text-center text-sm text-white/30">
                No detections
              </div>
            ) : (
              <div className="divide-y divide-white/5">
                {filteredDetections.map((detection) => (
                  <DetectionCard
                    key={detection.id}
                    detection={detection}
                    isNew={detection.status === 'new'}
                    onConfirm={() => onConfirm(detection.id)}
                    onDismiss={() => onDismiss(detection.id)}
                    onLocate={() => onLocate(detection.id)}
                  />
                ))}
              </div>
            )}
          </div>
        </div>
      </DrawerContent>
    </Drawer>
  );
}
