import { GCSGridLayout } from "@/components/layout/GCSLayout";
import { StatusBar } from "@/components/layout/StatusBar";
import { TelemetryPanel } from "@/components/layout/TelemetryPanel";
import { Scene3D } from "@/components/Scene3D";
import { ErrorBoundary } from "@/components/ErrorBoundary";

export default function Home() {
  return (
    <ErrorBoundary>
      <GCSGridLayout 
        statusBar={<StatusBar />}
        map={<Scene3D />}
        telemetry={<TelemetryPanel />}
      />
    </ErrorBoundary>
  );
}
