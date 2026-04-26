/**
 * WIA-EDU-022 E-Sports Education Standard - TypeScript Types
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @standard WIA-EDU-022
 */

// ============================================================================
// Core Models
// ============================================================================

export interface Program {
  id: string;
  standard: 'WIA-EDU-022';
  version: string;
  institution: Institution;
  program: ProgramDetails;
  staff: StaffMember[];
  metadata: Metadata;
}

export interface Institution {
  id: string;
  name: string;
  type: 'middle_school' | 'high_school' | 'college';
  location: Location;
}

export interface Location {
  country: string;
  state: string;
  city: string;
}

export interface ProgramDetails {
  name: string;
  type: 'club' | 'class' | 'varsity' | 'hybrid';
  status: 'planning' | 'active' | 'suspended' | 'archived';
  startDate: string;
  games: string[];
  gradeLevels: {
    min: number;
    max: number;
  };
  learningObjectives: string[];
  codeOfConduct: string;
}

export interface StaffMember {
  id: string;
  role: 'director' | 'head_coach' | 'assistant_coach' | 'advisor';
  name: string;
  contact: string;
}

export interface Metadata {
  created: string;
  updated: string;
  certificationLevel?: 'bronze' | 'silver' | 'gold' | 'platinum';
}

// ============================================================================
// Team Models
// ============================================================================

export interface Team {
  id: string;
  programId: string;
  name: string;
  game: string;
  tier: 'varsity' | 'jv' | 'novice' | 'practice';
  roster: Roster;
  season: Season;
  record: TeamRecord;
  practice: PracticeSchedule;
  metadata: Metadata;
}

export interface Roster {
  starters: string[];
  substitutes: string[];
  coaches: string[];
}

export interface Season {
  year: number;
  league: string;
  division: string;
}

export interface TeamRecord {
  wins: number;
  losses: number;
  ties: number;
  tournamentPlacements: TournamentPlacement[];
}

export interface TournamentPlacement {
  tournament: string;
  placement: number;
  date: string;
}

export interface PracticeSchedule {
  weeklyHours: number;
  schedule: PracticeSession[];
}

export interface PracticeSession {
  day: 'monday' | 'tuesday' | 'wednesday' | 'thursday' | 'friday' | 'saturday' | 'sunday';
  startTime: string;
  duration: number;
}

// ============================================================================
// Player Models
// ============================================================================

export interface Player {
  id: string;
  standard: 'WIA-EDU-022';
  personal: PersonalInfo;
  teamAssignments: TeamAssignment[];
  eligibility: Eligibility;
  performance: Performance;
  wellness: Wellness;
  consent: Consent;
  metadata: Metadata;
}

export interface PersonalInfo {
  studentId: string;
  gamerTag: string;
  grade: number;
  enrollmentYear: number;
}

export interface TeamAssignment {
  teamId: string;
  role: 'starter' | 'substitute' | 'captain' | 'practice';
  position: string;
  joinedDate: string;
  status: 'active' | 'inactive' | 'graduated';
}

export interface Eligibility {
  academicStanding: boolean;
  conductStanding: boolean;
  attendanceRequirement: boolean;
  lastVerified: string;
}

export interface Performance {
  individualStats: IndividualStats;
  teamContributions: TeamContributions;
}

export interface IndividualStats {
  gamesPlayed: number;
  winRate: number;
  skillRating: number;
  improvementRate: number;
}

export interface TeamContributions {
  communicationRating: number;
  teamworkRating: number;
  leadershipRating: number;
}

export interface Wellness {
  screenTimeWeekly: number;
  physicalActivityWeekly: number;
  sleepAverageHours: number;
  lastWellnessCheck: string;
}

export interface Consent {
  parentalConsent: boolean;
  dataSharing: boolean;
  mediaRelease: boolean;
  consentDate: string;
}

// ============================================================================
// Match Models
// ============================================================================

export interface Match {
  id: string;
  type: 'scrimmage' | 'league' | 'tournament' | 'championship';
  teamId: string;
  opponent: Opponent;
  schedule: MatchSchedule;
  roster: MatchRoster;
  result?: MatchResult;
  analysis?: MatchAnalysis;
  metadata: Metadata;
}

export interface Opponent {
  teamId?: string;
  name: string;
  institution: string;
}

export interface MatchSchedule {
  date: string;
  location: 'home' | 'away' | 'neutral' | 'online';
  venue: string;
}

export interface MatchRoster {
  starters: string[];
  substitutes: string[];
}

export interface MatchResult {
  score: {
    team: number;
    opponent: number;
  };
  outcome: 'win' | 'loss' | 'tie' | 'cancelled';
  duration: number;
  forfeit: boolean;
}

export interface MatchAnalysis {
  vod?: string;
  stats?: any;
  coachNotes?: string;
  playerReflections?: string[];
}

// ============================================================================
// Career Pathway Models
// ============================================================================

export interface CareerPathway {
  id: string;
  playerId: string;
  interests: CareerInterest[];
  experiences: Experience[];
  achievements: Achievement[];
  goals: CareerGoal[];
  portfolio: Portfolio;
  metadata: Metadata;
}

export type CareerInterest =
  | 'player'
  | 'coach'
  | 'analyst'
  | 'content_creator'
  | 'broadcaster'
  | 'event_manager'
  | 'marketing'
  | 'developer';

export interface Experience {
  type: 'competition' | 'internship' | 'workshop' | 'mentorship';
  title: string;
  organization: string;
  dateRange: {
    start: string;
    end?: string;
  };
  description: string;
  skills: string[];
}

export interface Achievement {
  type: 'award' | 'certification' | 'scholarship' | 'recognition';
  title: string;
  issuer: string;
  date: string;
  verificationUrl?: string;
  amount?: number;
}

export interface CareerGoal {
  category: 'short_term' | 'medium_term' | 'long_term';
  description: string;
  targetDate?: string;
  status: 'pending' | 'in_progress' | 'achieved' | 'revised';
  milestones: string[];
}

export interface Portfolio {
  streamingChannels: string[];
  contentSamples: string[];
  resume?: string;
  personalWebsite?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface ListProgramsRequest {
  institutionId?: string;
  status?: ProgramDetails['status'];
  page?: number;
  limit?: number;
}

export interface ListProgramsResponse {
  data: Program[];
  pagination: Pagination;
}

export interface ListTeamsRequest {
  programId: string;
  game?: string;
  tier?: Team['tier'];
  season?: number;
  page?: number;
  limit?: number;
}

export interface ListTeamsResponse {
  data: Team[];
  pagination: Pagination;
}

export interface ListPlayersRequest {
  programId: string;
  teamId?: string;
  grade?: number;
  status?: TeamAssignment['status'];
  page?: number;
  limit?: number;
}

export interface ListPlayersResponse {
  data: Player[];
  pagination: Pagination;
}

export interface ListMatchesRequest {
  teamId: string;
  type?: Match['type'];
  startDate?: string;
  endDate?: string;
  page?: number;
  limit?: number;
}

export interface ListMatchesResponse {
  data: Match[];
  pagination: Pagination;
}

export interface Pagination {
  page: number;
  limit: number;
  total: number;
  totalPages: number;
}

// ============================================================================
// API Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey?: string;
  accessToken?: string;
  baseURL?: string;
  environment?: 'production' | 'sandbox';
  timeout?: number;
}

export interface APIError {
  code: string;
  message: string;
  details?: any;
  timestamp: string;
  requestId: string;
}

// ============================================================================
// Webhook Types
// ============================================================================

export type WebhookEvent =
  | 'program.created'
  | 'program.updated'
  | 'team.created'
  | 'team.updated'
  | 'player.joined'
  | 'player.updated'
  | 'match.scheduled'
  | 'match.completed'
  | 'achievement.awarded';

export interface WebhookSubscription {
  id: string;
  url: string;
  events: WebhookEvent[];
  secret: string;
  active: boolean;
  metadata: Metadata;
}

export interface WebhookPayload<T = any> {
  event: WebhookEvent;
  timestamp: string;
  data: T;
  signature: string;
}

// ============================================================================
// Utility Types
// ============================================================================

export type CreateProgramRequest = Omit<Program, 'id' | 'metadata'>;
export type UpdateProgramRequest = Partial<Omit<Program, 'id' | 'standard' | 'metadata'>>;

export type CreateTeamRequest = Omit<Team, 'id' | 'metadata'>;
export type UpdateTeamRequest = Partial<Omit<Team, 'id' | 'programId' | 'metadata'>>;

export type CreatePlayerRequest = Omit<Player, 'id' | 'metadata'>;
export type UpdatePlayerRequest = Partial<Omit<Player, 'id' | 'standard' | 'metadata'>>;

export type CreateMatchRequest = Omit<Match, 'id' | 'result' | 'analysis' | 'metadata'>;
export type UpdateMatchRequest = Partial<Omit<Match, 'id' | 'teamId' | 'metadata'>>;
