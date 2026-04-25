/**
 * WIA-SOC-015 Voting System Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 * © 2025 World Certification Industry Association (WIA)
 * License: MIT
 */

export interface I18nString {
  en: string;
  ko?: string;
  es?: string;
  fr?: string;
  zh?: string;
  [languageCode: string]: string | undefined;
}

export type WIAVersion = string;

export interface Ballot {
  ballotId: string;
  version: WIAVersion;
  electionId: string;
  electionName: I18nString;
  jurisdiction: Jurisdiction;
  metadata: BallotMetadata;
  contests: Contest[];
  accessibility?: AccessibilityOptions;
  security: SecurityMetadata;
}

export interface Jurisdiction {
  country: string;
  state?: string;
  district?: string;
}

export interface BallotMetadata {
  created: string;
  validFrom: string;
  validUntil: string;
}

export interface Contest {
  contestId: string;
  contestType: 'candidate' | 'referendum' | 'recall';
  contestName: I18nString;
  votingMethod: 'single-choice' | 'multiple-choice' | 'ranked-choice';
  maxSelections: number;
  minSelections?: number;
  candidates?: Candidate[];
}

export interface Candidate {
  candidateId: string;
  name: string;
  party?: string;
  photoUrl?: string;
}

export interface AccessibilityOptions {
  screenReader?: {
    ballotDescription: I18nString;
    navigationHints: I18nString;
  };
  audioFormat?: {
    available: boolean;
    audioUrl?: string;
  };
}

export interface SecurityMetadata {
  digitalSignature: DigitalSignature;
  hashChain?: HashChain;
}

export interface DigitalSignature {
  algorithm: string;
  publicKey: string;
  signature: string;
  timestamp: string;
  signer: string;
}

export interface HashChain {
  algorithm: string;
  ballotHash: string;
  previousHash?: string;
  blockchainTxId?: string;
}

export interface VoterRegistrationRequest {
  personalInfo: PersonalInfo;
  address: Address;
  contactInfo: ContactInfo;
  credentials: IdentityCredential;
}

export interface PersonalInfo {
  firstName: string;
  lastName: string;
  dateOfBirth: string;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  zipCode: string;
}

export interface ContactInfo {
  email?: string;
  phone?: string;
}

export interface IdentityCredential {
  type: string;
  number: string;
}

export interface VoterRegistrationResponse {
  voterId: string;
  status: string;
  registrationDate: string;
}

export interface VoteSubmission {
  ballotId: string;
  voterId: string;
  votes: Vote[];
  metadata: VoteMetadata;
  signature: string;
}

export interface Vote {
  contestId: string;
  selections: string[];
}

export interface VoteMetadata {
  castTimestamp: string;
  castMethod: string;
}

export interface VoteSubmissionResponse {
  voteId: string;
  status: string;
  receipt: VoteReceipt;
  timestamp: string;
}

export interface VoteReceipt {
  receiptId: string;
  verificationCode: string;
  ballotHash: string;
  verifyUrl: string;
}

export interface ElectionResult {
  resultId: string;
  version: WIAVersion;
  electionId: string;
  contestId: string;
  timestamp: string;
  status: string;
  totalBallots: number;
  results: CandidateResult[];
}

export interface CandidateResult {
  candidateId: string;
  votes: number;
  percentage: number;
}

export interface VerificationRequest {
  receiptId: string;
  verificationCode?: string;
}

export interface VerificationResponse {
  receiptId: string;
  status: string;
  included: boolean;
  message: string;
}

export interface AuditEvent {
  eventId: string;
  timestamp: string;
  eventType: string;
  actor: Actor;
  action: string;
  outcome: string;
}

export interface Actor {
  id: string;
  role: string;
  authenticated: boolean;
}

export interface WIAVotingConfig {
  baseUrl: string;
  timeout?: number;
  headers?: Record<string, string>;
}

export interface APIError {
  code: string;
  message: string;
  details?: string;
  timestamp: string;
  requestId: string;
}

export interface PaginationParams {
  limit?: number;
  offset?: number;
}
