'use client';

import type { ReactNode } from 'react';
import { CoordinateOriginProvider } from '@/context/CoordinateOriginContext';
import { ROSConnectionProvider } from '@/context/ROSConnectionContext';
import { ZoneProvider } from '@/context/ZoneContext';
import { MissionProvider } from '@/context/MissionContext';

interface ViewerAppProvidersProps {
  children: ReactNode;
}

export function ViewerAppProviders({ children }: ViewerAppProvidersProps) {
  return (
    <CoordinateOriginProvider>
      <ROSConnectionProvider>
        <ZoneProvider>
          <MissionProvider>{children}</MissionProvider>
        </ZoneProvider>
      </ROSConnectionProvider>
    </CoordinateOriginProvider>
  );
}
