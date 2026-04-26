/**
 * WIA-AI-016 Multi-Agent System Standard - TypeScript Types
 * 弘益人間 (홍익인간) · Benefit All Humanity
 *
 * © 2025 SmileStory Inc. / World Certification Industry Association
 */

/**
 * FIPA-ACL Performatives
 */
export type Performative =
  | 'inform'
  | 'request'
  | 'query-if'
  | 'query-ref'
  | 'propose'
  | 'accept-proposal'
  | 'reject-proposal'
  | 'agree'
  | 'refuse'
  | 'failure'
  | 'cfp'
  | 'subscribe'
  | 'cancel';

/**
 * Agent Identifier
 */
export interface AgentIdentifier {
  name: string;
  address: string;
  resolvers?: string[];
  userDefinedProperties?: Record<string, any>;
}

/**
 * FIPA-ACL Message
 */
export interface FIPAMessage {
  performative: Performative;
  sender: AgentIdentifier;
  receiver: AgentIdentifier[];
  content: any;
  language?: string;
  ontology?: string;
  protocol?: string;
  conversationId?: string;
  replyWith?: string;
  inReplyTo?: string;
  replyBy?: Date | string;
  formatVersion?: string;
}

/**
 * Agent Capability
 */
export interface AgentCapability {
  name: string;
  type: 'sensing' | 'acting' | 'processing' | 'communication' | 'storage';
  parameters?: Record<string, any>;
  constraints?: Record<string, any>;
}

/**
 * Task Definition
 */
export interface Task {
  taskId: string;
  type: string;
  priority?: number;
  requiredSkills?: string[];
  requiredAgents?: number;
  dependencies?: string[];
  deadline?: Date | string;
  parameters?: Record<string, any>;
  constraints?: Record<string, any>;
}

/**
 * Task Status
 */
export type TaskStatus = 'pending' | 'assigned' | 'in-progress' | 'completed' | 'failed' | 'cancelled';

/**
 * Task Result
 */
export interface TaskResult {
  taskId: string;
  status: TaskStatus;
  result?: any;
  quality?: number;
  completedAt?: Date;
  error?: string;
}

/**
 * Agent Status
 */
export type AgentStatus = 'idle' | 'busy' | 'offline' | 'maintenance';

/**
 * Agent Configuration
 */
export interface AgentConfig {
  id: string;
  name?: string;
  capabilities?: AgentCapability[];
  platformUrl?: string;
  authentication?: {
    token?: string;
    apiKey?: string;
  };
  transport?: 'http' | 'websocket' | 'mqtt' | 'rabbitmq';
  autoReconnect?: boolean;
  heartbeatInterval?: number;
}

/**
 * Reputation Data
 */
export interface Reputation {
  overall: number;
  reliability: number;
  quality: number;
  responsiveness: number;
  interactionCount: number;
  lastUpdated: Date | string;
}

/**
 * Agent Review
 */
export interface AgentReview {
  reviewer: string;
  rating: number;
  comment?: string;
  timestamp: Date | string;
}

/**
 * Coalition
 */
export interface Coalition {
  coalitionId: string;
  members: string[];
  leader?: string;
  goal: string;
  formation: Date | string;
  expiration?: Date | string;
  roles?: Record<string, string>;
}

/**
 * Performance Metrics
 */
export interface PerformanceMetrics {
  tasksCompleted: number;
  tasksFailed: number;
  avgResponseTime: number;
  avgQuality: number;
  utilization: number;
  messagesReceived: number;
  messagesSent: number;
}

/**
 * Event Type
 */
export type EventType =
  | 'task-assigned'
  | 'task-completed'
  | 'task-failed'
  | 'agent-joined'
  | 'agent-left'
  | 'message-received'
  | 'message-sent'
  | 'error';

/**
 * Agent Event
 */
export interface AgentEvent {
  type: EventType;
  timestamp: Date | string;
  source: string;
  data: any;
}

/**
 * Bid for Contract Net Protocol
 */
export interface Bid {
  bidder: string;
  taskId: string;
  cost: number;
  estimatedTime?: number;
  quality?: number;
  confidence?: number;
}

/**
 * Negotiation Proposal
 */
export interface NegotiationProposal {
  proposer: string;
  issues: Array<{
    name: string;
    value: number;
    importance: number;
  }>;
  utilityFunction?: 'weighted_sum' | 'multiplicative' | 'custom';
}

/**
 * Consensus Vote
 */
export interface ConsensusVote {
  voter: string;
  proposal: string;
  vote: boolean;
  weight?: number;
  timestamp: Date | string;
}

/**
 * Agent Message Handler
 */
export type MessageHandler = (message: FIPAMessage) => Promise<void> | void;

/**
 * Task Handler
 */
export type TaskHandler = (task: Task) => Promise<TaskResult> | TaskResult;

/**
 * Event Handler
 */
export type EventHandler = (event: AgentEvent) => Promise<void> | void;

/**
 * Transport Interface
 */
export interface Transport {
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  send(message: FIPAMessage): Promise<void>;
  onMessage(handler: MessageHandler): void;
  isConnected(): boolean;
}

/**
 * Storage Interface
 */
export interface Storage {
  get(key: string): Promise<any>;
  set(key: string, value: any): Promise<void>;
  delete(key: string): Promise<void>;
  has(key: string): Promise<boolean>;
  clear(): Promise<void>;
}

/**
 * Logger Interface
 */
export interface Logger {
  debug(message: string, ...args: any[]): void;
  info(message: string, ...args: any[]): void;
  warn(message: string, ...args: any[]): void;
  error(message: string, ...args: any[]): void;
}

/**
 * Agent Interface
 */
export interface IAgent {
  readonly id: string;
  readonly address: AgentIdentifier;
  readonly status: AgentStatus;
  readonly capabilities: AgentCapability[];

  start(): Promise<void>;
  stop(): Promise<void>;

  sendMessage(message: Partial<FIPAMessage>): Promise<void>;
  sendInform(receiver: string | string[], content: any): Promise<void>;
  sendRequest(receiver: string | string[], action: any): Promise<void>;
  sendPropose(receiver: string | string[], proposal: any): Promise<void>;

  onMessage(handler: MessageHandler): void;
  onTask(handler: TaskHandler): void;
  onEvent(type: EventType, handler: EventHandler): void;

  executeTask(task: Task): Promise<TaskResult>;
  getMetrics(): PerformanceMetrics;
  getReputation(): Reputation;
}

/**
 * Multi-Agent System Interface
 */
export interface IMultiAgentSystem {
  registerAgent(agent: IAgent): Promise<void>;
  unregisterAgent(agentId: string): Promise<void>;
  getAgent(agentId: string): IAgent | undefined;
  getAllAgents(): IAgent[];

  assignTask(task: Task): Promise<TaskResult>;
  broadcastMessage(message: FIPAMessage): Promise<void>;

  start(): Promise<void>;
  stop(): Promise<void>;
}

/**
 * HTTP API Response
 */
export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
}

/**
 * Pagination
 */
export interface Pagination {
  page: number;
  limit: number;
  total: number;
}

/**
 * Paginated Response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: Pagination;
}
