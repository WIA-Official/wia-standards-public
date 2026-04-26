/**
 * WIA-SPACE-025: Interstellar Communication Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type ModulationType = 'FSK' | 'PSK' | 'OOK' | 'QAM';
export type Polarization = 'Linear' | 'Circular' | 'Elliptical';
export type MessageType = 'Beacon' | 'Greeting' | 'Scientific' | 'Cultural';
export type EncodingType = 'Binary' | 'Manchester' | 'ReedSolomon' | 'LDPC';

export interface TransmissionOrigin {
  location: 'Earth' | 'Mars' | 'Moon' | 'Other';
  coordinates?: GalacticCoordinates;
  facility: string;
  telescope?: string;
}

export interface GalacticCoordinates {
  longitude: number; // degrees
  latitude: number; // degrees
  distance?: number; // parsecs
}

export interface CelestialCoordinates {
  rightAscension: number; // degrees
  declination: number; // degrees
  distance: number; // light-years
}

export interface SignalParameters {
  frequency: number; // Hz
  bandwidth: number; // Hz
  power: number; // Watts
  modulation: ModulationType;
  polarization: Polarization;
  beamWidth?: number; // degrees
}

export interface TargetSystem {
  designation: string;
  distance: number; // light-years
  coordinates: CelestialCoordinates;
  starType?: string;
  knownPlanets?: number;
}

export interface MessagePayload {
  encoding: EncodingType;
  contentType: MessageType;
  payload: string; // base64 encoded
  length: number; // bits
  checksum: string;
  errorCorrectionRate?: number; // ratio (e.g., 2:1)
}

export interface PropagationInfo {
  transmitTime: string; // ISO8601
  arrivalTime: string; // ISO8601 (estimated)
  roundTripTime: number; // years
  doppler ShiftEstimate?: number; // Hz
}

export interface InterstellarTransmission {
  id: string;
  timestamp: string; // ISO8601
  origin: TransmissionOrigin;
  signal: SignalParameters;
  target: TargetSystem;
  message: MessagePayload;
  propagation: PropagationInfo;
}

export interface SETISignal {
  id: string;
  detectionTime: string; // ISO8601
  observatory: string;
  frequency: number; // Hz
  bandwidth: number; // Hz
  signalToNoiseRatio: number; // dB
  driftRate?: number; // Hz/s
  duration: number; // seconds
  coordinates: CelestialCoordinates;
  status: 'Candidate' | 'Verified' | 'RFI' | 'Unknown';
}

export interface SignalAnalysis {
  artificialProbability: number; // 0-1
  narrowband: boolean;
  persistent: boolean;
  terrestrialOrigin: boolean;
  recommendations: string[];
}

export interface OpticalLink {
  wavelength: number; // nm
  laserPower: number; // Watts
  beamDivergence: number; // microradians
  aperture: number; // meters
  dataRate: number; // bps
}

export interface QuantumChannel {
  entanglementFidelity: number; // 0-1
  keyRate: number; // bits/s
  distance: number; // km
  protocol: 'BB84' | 'E91' | 'Other';
}

export interface CommunicationConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
  frequencies?: {
    hydrogen: number; // 1420 MHz
    waterHole: { min: number; max: number };
  };
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
  timestamp: string;
}

export interface FrequencyBand {
  name: string;
  rangeMin: number; // Hz
  rangeMax: number; // Hz
  primaryUse: string;
  characteristics: string[];
}

export interface MessageConstructionStandard {
  name: string; // 'Pioneer Plaque' | 'Voyager Golden Record' | 'Arecibo Message'
  year: number;
  contentDescription: string;
  format: string;
  size?: string;
}
