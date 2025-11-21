'use client';

import { Badge } from "@/components/ui/badge"
import { useROSConnection } from "@/hooks/useROSConnection"

export function ConnectionStatus() {
  const { isConnected, isConnecting, error } = useROSConnection();

  if (error) {
      return <Badge variant="destructive" className="animate-pulse">Connection Error</Badge>;
  }

  if (isConnecting) {
      return <Badge variant="warning" className="animate-pulse">Connecting...</Badge>;
  }

  if (isConnected) {
      return <Badge variant="success">Connected</Badge>;
  }

  return <Badge variant="secondary">Disconnected</Badge>;
}
