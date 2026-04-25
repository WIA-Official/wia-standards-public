/**
 * WIA-IND-013: Esports Standard - Type Definitions
 * @module wia-ind-013
 */

export enum GameGenre {
  MOBA = 'moba', FPS = 'fps', BattleRoyale = 'battle_royale',
  RTS = 'rts', FightingGame = 'fighting', SportsGame = 'sports',
  CardGame = 'card', RacingGame = 'racing', Puzzle = 'puzzle'
}

export enum TournamentFormat {
  SingleElimination = 'single_elimination', DoubleElimination = 'double_elimination',
  RoundRobin = 'round_robin', Swiss = 'swiss', GroupStage = 'group_stage'
}

export enum MatchStatus {
  Scheduled = 'scheduled', Live = 'live', Completed = 'completed',
  Postponed = 'postponed', Cancelled = 'cancelled'
}

export interface Player {
  id: string;
  username: string;
  realName?: string;
  team?: string;
  country: string;
  games: string[];
  rating: number;
  earnings: number;
  socialLinks?: Record<string, string>;
}

export interface Team {
  id: string;
  name: string;
  tag: string;
  region: string;
  players: Player[];
  coach?: string;
  founded: string;
  achievements: { tournament: string; placement: number; date: string }[];
}

export interface Tournament {
  id: string;
  name: string;
  game: string;
  genre: GameGenre;
  format: TournamentFormat;
  prizePool: number;
  currency: string;
  startDate: string;
  endDate: string;
  location: string;
  online: boolean;
  teams: Team[];
  brackets: Bracket[];
  status: 'upcoming' | 'ongoing' | 'completed';
}

export interface Bracket {
  id: string;
  name: string;
  type: 'winners' | 'losers' | 'grand_final';
  matches: Match[];
}

export interface Match {
  id: string;
  tournamentId: string;
  roundNumber: number;
  team1: Team;
  team2: Team;
  score: { team1: number; team2: number };
  maps: MapResult[];
  status: MatchStatus;
  scheduledTime: number;
  startTime?: number;
  endTime?: number;
  streamUrl?: string;
  vodUrl?: string;
}

export interface MapResult {
  mapName: string;
  team1Score: number;
  team2Score: number;
  duration: number;
  stats: PlayerStats[];
}

export interface PlayerStats {
  playerId: string;
  kills: number;
  deaths: number;
  assists: number;
  damage?: number;
  healing?: number;
  objectives?: number;
  customStats?: Record<string, number>;
}

export interface LiveMatchData {
  matchId: string;
  timestamp: number;
  gameState: Record<string, unknown>;
  events: GameEvent[];
  playerPositions?: { playerId: string; x: number; y: number; z?: number }[];
}

export interface GameEvent {
  timestamp: number;
  type: string;
  actor?: string;
  target?: string;
  value?: number;
  details?: Record<string, unknown>;
}

export interface StreamInfo {
  platform: 'twitch' | 'youtube' | 'facebook' | 'other';
  channelId: string;
  url: string;
  viewers: number;
  language: string;
  quality: string[];
}

export interface Prediction {
  matchId: string;
  team1WinProbability: number;
  team2WinProbability: number;
  confidence: number;
  factors: { factor: string; impact: number }[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-013';
  testDate: string;
  platformId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type EsportsEventType = 'match-start' | 'match-end' | 'game-event' | 'score-update' | 'stream-live';
export type EventCallback<T = unknown> = (data: T) => void;
