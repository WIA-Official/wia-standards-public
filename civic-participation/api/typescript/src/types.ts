/**
 * WIA-SOC-005: Civic Participation Standard - Type Definitions
 *
 * @module types
 * @description TypeScript type definitions for civic participation entities
 */

// ============================================================================
// Voting Types
// ============================================================================

export type VoteType = 'single' | 'multiple' | 'ranked' | 'approval' | 'score';
export type VoteStatus = 'draft' | 'active' | 'closed' | 'archived';

export interface VoteOption {
  id: string;
  text: string;
  description?: string;
}

export interface VoteEligibility {
  minAge?: number;
  maxAge?: number;
  residency?: string;
  customCriteria?: Record<string, any>;
}

export interface VoteSchedule {
  startDate: Date;
  endDate: Date;
  timezone: string;
}

export interface VoteSettings {
  anonymous: boolean;
  allowChanges: boolean;
  requireVerification: boolean;
  blockchainAnchor: boolean;
}

export interface VoteResults {
  totalVotes: number;
  turnout: number;
  optionCounts: Record<string, number>;
  blockchainHash?: string;
}

export interface Vote {
  id: string;
  title: string;
  description: string;
  type: VoteType;
  options: VoteOption[];
  eligibility: VoteEligibility;
  schedule: VoteSchedule;
  settings: VoteSettings;
  results?: VoteResults;
  status: VoteStatus;
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface CreateVoteRequest {
  title: string;
  description: string;
  type: VoteType;
  options: Omit<VoteOption, 'id'>[];
  eligibility?: VoteEligibility;
  schedule: Omit<VoteSchedule, 'timezone'> & { timezone?: string };
  settings?: Partial<VoteSettings>;
}

// ============================================================================
// Ballot Types
// ============================================================================

export type BallotSelection = number | number[] | Record<string, boolean | number>;

export interface BallotProof {
  eligibilityProof: string;
  uniquenessProof: string;
  validityProof: string;
}

export interface Ballot {
  id: string;
  voteId: string;
  voterToken: string;
  selection: BallotSelection;
  timestamp: Date;
  signature: string;
  blockchainTxId?: string;
  proof: BallotProof;
}

export interface CastBallotRequest {
  selection: BallotSelection;
  voterToken: string;
}

export interface BallotReceipt {
  id: string;
  voteId: string;
  timestamp: Date;
  blockchainHash?: string;
  verificationCode: string;
}

// ============================================================================
// Petition Types
// ============================================================================

export type PetitionStatus = 'draft' | 'active' | 'closed' | 'archived';
export type ResponseStatus = 'none' | 'pending' | 'responded';

export interface PetitionRecipient {
  name: string;
  organization: string;
  email?: string;
}

export interface PetitionEligibility {
  minAge?: number;
  residency?: string;
}

export interface PetitionSignatures {
  count: number;
  verified: number;
  pending: number;
}

export interface PetitionMilestone {
  threshold: number;
  reached: boolean;
  reachedAt?: Date;
}

export interface PetitionResponse {
  status: ResponseStatus;
  text?: string;
  respondedBy?: string;
  respondedAt?: Date;
}

export interface Petition {
  id: string;
  title: string;
  description: string;
  category: string;
  tags: string[];
  objectives: string[];
  signatureGoal: number;
  deadline?: Date;
  recipient: PetitionRecipient;
  eligibility: PetitionEligibility;
  signatures: PetitionSignatures;
  milestones: PetitionMilestone[];
  response: PetitionResponse;
  status: PetitionStatus;
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface CreatePetitionRequest {
  title: string;
  description: string;
  category: string;
  tags?: string[];
  objectives: string[];
  signatureGoal: number;
  deadline?: Date;
  recipient: PetitionRecipient;
  eligibility?: PetitionEligibility;
}

// ============================================================================
// Signature Types
// ============================================================================

export type VerificationMethod = 'email' | 'sms' | 'id' | 'biometric';

export interface SignatureVerification {
  method: VerificationMethod;
  verified: boolean;
  verifiedAt?: Date;
}

export interface Signature {
  id: string;
  petitionId: string;
  signerToken: string;
  name?: string;
  email: string;
  comment?: string;
  timestamp: Date;
  ipAddress: string;
  verification: SignatureVerification;
  signature: string;
  blockchainTxId?: string;
}

export interface SignPetitionRequest {
  signerToken: string;
  name?: string;
  email: string;
  comment?: string;
}

export interface SignatureReceipt {
  id: string;
  petitionId: string;
  timestamp: Date;
  verificationCode: string;
}

// ============================================================================
// Consultation Types
// ============================================================================

export type ConsultationStatus = 'draft' | 'active' | 'closed' | 'archived';
export type ResourceType = 'document' | 'video' | 'link' | 'data';
export type PhaseType = 'information' | 'discussion' | 'proposal' | 'voting';

export interface ConsultationResource {
  type: ResourceType;
  title: string;
  url: string;
  description?: string;
}

export interface ConsultationPhase {
  name: string;
  type: PhaseType;
  startDate: Date;
  endDate: Date;
}

export interface ConsultationSchedule {
  phases: ConsultationPhase[];
}

export interface ConsultationParticipants {
  total: number;
  experts: string[];
  stakeholders: string[];
}

export interface ConsultationEngagement {
  comments: number;
  proposals: number;
  votes: number;
}

export interface ConsultationOutcomes {
  summary?: string;
  recommendations: string[];
  consensus?: number;
}

export interface Consultation {
  id: string;
  topic: string;
  description: string;
  category: string;
  tags: string[];
  resources: ConsultationResource[];
  schedule: ConsultationSchedule;
  participants: ConsultationParticipants;
  engagement: ConsultationEngagement;
  outcomes: ConsultationOutcomes;
  status: ConsultationStatus;
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface CreateConsultationRequest {
  topic: string;
  description: string;
  category: string;
  tags?: string[];
  resources?: ConsultationResource[];
  schedule: ConsultationSchedule;
}

export interface ConsultationComment {
  id: string;
  consultationId: string;
  authorId: string;
  text: string;
  replyTo?: string;
  likes: number;
  timestamp: Date;
}

export interface AddCommentRequest {
  text: string;
  replyTo?: string;
}

// ============================================================================
// Budget Types
// ============================================================================

export type BudgetProposalStatus = 'draft' | 'submitted' | 'voting' | 'funded' | 'rejected' | 'completed';
export type FeasibilityStatus = 'pending' | 'approved' | 'rejected';
export type ImplementationStatus = 'not_started' | 'in_progress' | 'completed' | 'cancelled';

export interface Location {
  address?: string;
  district?: string;
  coordinates: {
    lat: number;
    lng: number;
  };
}

export interface Budget {
  requested: number; // USD cents
  approved?: number;
  spent?: number;
}

export interface Timeline {
  proposedStart: Date;
  proposedEnd: Date;
  actualStart?: Date;
  actualEnd?: Date;
}

export interface Feasibility {
  status: FeasibilityStatus;
  review?: string;
  reviewedBy?: string;
  reviewedAt?: Date;
}

export interface BudgetVoting {
  votes: number;
  rank?: number;
  funded: boolean;
}

export interface ImplementationUpdate {
  date: Date;
  text: string;
}

export interface Implementation {
  status: ImplementationStatus;
  progress: number; // 0-100
  updates: ImplementationUpdate[];
}

export interface BudgetProposal {
  id: string;
  projectName: string;
  description: string;
  category: string;
  location: Location;
  budget: Budget;
  timeline: Timeline;
  feasibility: Feasibility;
  voting: BudgetVoting;
  implementation: Implementation;
  status: BudgetProposalStatus;
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface CreateBudgetProposalRequest {
  projectName: string;
  description: string;
  category: string;
  location: Location;
  budget: {
    requested: number;
  };
  timeline: Timeline;
}

export interface VoteBudgetProposalRequest {
  voterToken: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface CivicParticipationConfig {
  apiKey: string;
  baseUrl?: string;
  network?: 'mainnet' | 'testnet' | 'devnet';
  timeout?: number;
  retries?: number;
  blockchain?: {
    enabled: boolean;
    provider?: string;
    contractAddress?: string;
  };
  websocket?: {
    enabled: boolean;
    url?: string;
  };
}

// ============================================================================
// Event Types
// ============================================================================

export type EventType =
  | 'vote_created'
  | 'vote_cast'
  | 'vote_closed'
  | 'petition_created'
  | 'petition_signed'
  | 'petition_milestone'
  | 'consultation_created'
  | 'consultation_comment'
  | 'budget_proposal_created'
  | 'budget_proposal_voted'
  | 'budget_proposal_funded';

export interface CivicEvent {
  type: EventType;
  timestamp: Date;
  data: Record<string, any>;
}

// ============================================================================
// Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  meta?: {
    timestamp: Date;
    requestId: string;
  };
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

export interface PaginationParams {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface FilterParams {
  status?: string;
  category?: string;
  startDate?: Date;
  endDate?: Date;
  search?: string;
}
