/**
 * Alert system exports for the Aeris GCS.
 *
 * This module provides a centralized alert management system for surfacing
 * critical operational events to GCS operators. Alerts integrate with the
 * ROS telemetry pipeline to display vehicle anomalies, comms loss, and
 * mission-critical notifications.
 *
 * @example
 * ```tsx
 * import { showAlert, AlertToaster } from '@/components/alerts';
 *
 * // Display a critical alert
 * showAlert({
 *   id: 'comms-loss',
 *   severity: 'critical',
 *   title: 'Vehicle COMMS LOST',
 *   description: 'Last contact 30s ago',
 *   dismissible: false,
 * });
 * ```
 */
export {
  AlertToaster,
  showAlert,
  dismissAlert,
  dismissAllAlerts,
  useAlerts,
  type Alert,
  type AlertSeverity,
} from './AlertStack';
