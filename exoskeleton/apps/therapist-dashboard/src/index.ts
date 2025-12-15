/**
 * WIA Exoskeleton Therapist Dashboard
 *
 * @packageDocumentation
 */

// Types
export * from './types';

// Dashboard
export {
  ITherapistDashboard,
  TherapistDashboard,
  createDashboard,
} from './dashboard';

// Version
export const VERSION = '1.0.0';

/**
 * WIA Therapist Dashboard
 *
 * A comprehensive dashboard for rehabilitation therapists to manage
 * patients, track progress, and monitor exoskeleton training sessions.
 *
 * ## Features
 *
 * - **Patient Management**: Create, update, and manage patient profiles
 * - **Program Assignment**: Assign rehabilitation programs to patients
 * - **Progress Tracking**: Monitor metrics, milestones, and trends
 * - **Session History**: Review past training sessions
 * - **Live Monitoring**: Watch active training sessions in real-time
 * - **Reporting**: Generate progress reports for patients
 * - **Alerts**: Receive and manage patient alerts
 *
 * ## Quick Start
 *
 * ```typescript
 * import { createDashboard, TherapistRole } from '@wia/therapist-dashboard';
 *
 * // Create a dashboard instance
 * const dashboard = createDashboard({
 *   userId: 'therapist-001',
 *   name: 'Dr. Kim',
 *   nameKorean: '김철수',
 *   role: TherapistRole.THERAPIST,
 *   department: 'Rehabilitation',
 *   licenseNumber: 'PT-12345',
 * });
 *
 * // Get patient list
 * const patients = await dashboard.getPatients();
 *
 * // Get patient progress
 * const progress = await dashboard.getProgress('patient-001');
 *
 * // Generate report
 * const report = await dashboard.generateProgressReport('patient-001', {
 *   start: new Date('2025-01-01'),
 *   end: new Date('2025-12-14'),
 * });
 * ```
 *
 * ## Patient Workflow
 *
 * 1. Create patient profile
 * 2. Assign rehabilitation program
 * 3. Schedule training sessions
 * 4. Monitor progress during sessions
 * 5. Record assessments
 * 6. Generate progress reports
 *
 * ## Integration with WIA Exoskeleton API
 *
 * The dashboard integrates with the WIA Exoskeleton control API
 * to receive real-time session data and send control commands.
 *
 * @module @wia/therapist-dashboard
 */

// Default export
import { createDashboard } from './dashboard';
export default createDashboard;
