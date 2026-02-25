'use client';

import { createContext, useContext, type ReactNode } from 'react';
import {
  useROSConnection,
  type ROSConnectionOptions,
  type ROSConnectionResult,
} from '@/hooks/useROSConnection';

const ROSConnectionContext = createContext<ROSConnectionResult | null>(null);

interface ROSConnectionProviderProps {
  children: ReactNode;
  options?: ROSConnectionOptions;
}

export function ROSConnectionProvider({
  children,
  options,
}: ROSConnectionProviderProps) {
  const connection = useROSConnection(options);
  return (
    <ROSConnectionContext.Provider value={connection}>
      {children}
    </ROSConnectionContext.Provider>
  );
}

export function useSharedROSConnection(): ROSConnectionResult {
  const context = useContext(ROSConnectionContext);
  if (!context) {
    throw new Error(
      'useSharedROSConnection must be used within a ROSConnectionProvider'
    );
  }
  return context;
}
