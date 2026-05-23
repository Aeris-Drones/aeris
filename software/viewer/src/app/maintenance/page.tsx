import { ViewerAppProviders } from '@/app/ViewerAppProviders';
import { MaintenanceDashboardPage } from '@/components/maintenance/MaintenanceDashboardPage';

export default function MaintenancePage() {
  return (
    <ViewerAppProviders>
      <MaintenanceDashboardPage />
    </ViewerAppProviders>
  );
}
