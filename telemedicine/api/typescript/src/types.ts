/**
 * WIA Telemedicine Standard - TypeScript Type Definitions
 * Version: 1.0.0
 */

export type ConsultationType = 'video' | 'audio' | 'chat' | 'async';
export type ConsultationStatus = 'scheduled' | 'in_progress' | 'completed' | 'cancelled';

export interface TelemedicineSession {
  format: 'WIA-TELEMEDICINE-v1.0';
  timestamp: string;
  session_id: string;
  patient_id: string;
  provider_id: string;
  consultation_type: ConsultationType;
  status: ConsultationStatus;
  start_time: string;
  end_time?: string;
  duration_minutes?: number;
  diagnosis?: string;
  prescriptions?: Array<{
    medication: string;
    dosage: string;
    duration: string;
  }>;
  follow_up_required: boolean;
}

export interface VideoSettings {
  resolution: '720p' | '1080p' | '4K';
  framerate: 15 | 30 | 60;
  bitrate_kbps: number;
  encryption: boolean;
}

export interface TelehealthProvider {
  provider_id: string;
  name: string;
  specialty: string;
  license_number: string;
  available: boolean;
  rating?: number;
}
