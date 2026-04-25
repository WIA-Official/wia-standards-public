/**
 * WIA Surgical Robot Standard - TypeScript Type Definitions
 * Version: 1.0.0
 */

export type SurgeryType = 'laparoscopic' | 'orthopedic' | 'neurosurgery' | 'cardiac' | 'general';
export type RobotStatus = 'idle' | 'active' | 'error' | 'maintenance';

export interface SurgicalRobotData {
  format: 'WIA-SURGICAL-ROBOT-v1.0';
  timestamp: string;
  robot_id: string;
  surgery_type: SurgeryType;
  status: RobotStatus;
  position: {
    x: number;
    y: number;
    z: number;
  };
  force_feedback: {
    value: number;
    unit: 'N';
  };
  instruments: Array<{
    instrument_id: string;
    type: string;
    status: 'attached' | 'detached' | 'in_use';
  }>;
  telemetry: {
    battery_level?: number;
    temperature_celsius: number;
    vibration_level: number;
  };
}

export interface SurgicalProcedure {
  procedure_id: string;
  patient_id: string;
  surgeon_id: string;
  robot_id: string;
  surgery_type: SurgeryType;
  start_time: string;
  end_time?: string;
  complications?: string[];
  outcome?: string;
}
