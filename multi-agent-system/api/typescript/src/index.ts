/**
 * WIA-AI-016 Multi-Agent System Standard - TypeScript SDK
 * 弘益人間 (홍익인간) · Benefit All Humanity
 *
 * © 2025 SmileStory Inc. / World Certification Industry Association
 */

import EventEmitter from 'eventemitter3';
import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';

import {
  AgentConfig,
  AgentIdentifier,
  AgentCapability,
  AgentStatus,
  FIPAMessage,
  Task,
  TaskResult,
  TaskStatus,
  PerformanceMetrics,
  Reputation,
  EventType,
  AgentEvent,
  MessageHandler,
  TaskHandler,
  EventHandler,
  IAgent,
  Performative,
} from './types';

export * from './types';

/**
 * Base Agent Implementation
 */
export class Agent extends EventEmitter implements IAgent {
  public readonly id: string;
  public readonly address: AgentIdentifier;
  public status: AgentStatus = 'offline';
  public capabilities: AgentCapability[] = [];

  private config: AgentConfig;
  private httpClient: AxiosInstance;
  private ws?: WebSocket;
  private messageHandlers: MessageHandler[] = [];
  private taskHandlers: Map<string, TaskHandler> = new Map();
  private metrics: PerformanceMetrics = {
    tasksCompleted: 0,
    tasksFailed: 0,
    avgResponseTime: 0,
    avgQuality: 0,
    utilization: 0,
    messagesReceived: 0,
    messagesSent: 0,
  };

  constructor(config: AgentConfig) {
    super();
    this.config = config;
    this.id = config.id;
    this.capabilities = config.capabilities || [];

    this.address = {
      name: config.name || config.id,
      address: `wia://${config.platformUrl || 'localhost'}/agents/${config.id}`,
    };

    this.httpClient = axios.create({
      baseURL: config.platformUrl || 'http://localhost:8080',
      headers: {
        'Authorization': `Bearer ${config.authentication?.token || ''}`,
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Start the agent
   */
  async start(): Promise<void> {
    await this.register();
    await this.connectWebSocket();
    this.status = 'idle';
    this.startHeartbeat();
    this.emit('started');
  }

  /**
   * Stop the agent
   */
  async stop(): Promise<void> {
    this.status = 'offline';
    this.disconnectWebSocket();
    await this.unregister();
    this.emit('stopped');
  }

  /**
   * Register agent with platform
   */
  private async register(): Promise<void> {
    try {
      await this.httpClient.post('/agents', {
        name: this.address.name,
        capabilities: this.capabilities,
        metadata: {},
      });
    } catch (error) {
      throw new Error(`Failed to register agent: ${error}`);
    }
  }

  /**
   * Unregister agent from platform
   */
  private async unregister(): Promise<void> {
    try {
      await this.httpClient.delete(`/agents/${this.id}`);
    } catch (error) {
      console.error('Failed to unregister agent:', error);
    }
  }

  /**
   * Connect WebSocket for real-time communication
   */
  private async connectWebSocket(): Promise<void> {
    if (this.config.transport === 'websocket' || this.config.transport === undefined) {
      const wsUrl = (this.config.platformUrl || 'ws://localhost:8080')
        .replace('http://', 'ws://')
        .replace('https://', 'wss://');

      this.ws = new WebSocket(`${wsUrl}/agents/${this.id}/ws`);

      this.ws.on('open', () => {
        this.ws?.send(JSON.stringify({
          type: 'auth',
          token: this.config.authentication?.token,
        }));
      });

      this.ws.on('message', (data) => {
        try {
          const packet = JSON.parse(data.toString());
          if (packet.type === 'message') {
            this.handleIncomingMessage(packet.data);
          } else if (packet.type === 'event') {
            this.handleEvent(packet.data);
          }
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error);
        }
      });

      this.ws.on('error', (error) => {
        console.error('WebSocket error:', error);
        this.emit('error', error);
      });

      this.ws.on('close', () => {
        if (this.config.autoReconnect && this.status !== 'offline') {
          setTimeout(() => this.connectWebSocket(), 5000);
        }
      });
    }
  }

  /**
   * Disconnect WebSocket
   */
  private disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  /**
   * Send FIPA-ACL message
   */
  async sendMessage(message: Partial<FIPAMessage>): Promise<void> {
    const fullMessage: FIPAMessage = {
      performative: message.performative as Performative,
      sender: this.address,
      receiver: message.receiver || [],
      content: message.content,
      language: message.language || 'fipa-sl',
      ontology: message.ontology,
      protocol: message.protocol,
      conversationId: message.conversationId || this.generateConversationId(),
      replyWith: message.replyWith,
      inReplyTo: message.inReplyTo,
      replyBy: message.replyBy,
      formatVersion: '1.0.0',
    };

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        type: 'message',
        data: fullMessage,
      }));
    } else {
      await this.httpClient.post(`/agents/${this.id}/messages`, fullMessage);
    }

    this.metrics.messagesSent++;
    this.emit('message-sent', fullMessage);
  }

  /**
   * Send INFORM message
   */
  async sendInform(receiver: string | string[], content: any): Promise<void> {
    await this.sendMessage({
      performative: 'inform',
      receiver: Array.isArray(receiver) ? receiver.map(r => ({ name: r, address: r })) : [{ name: receiver, address: receiver }],
      content,
    });
  }

  /**
   * Send REQUEST message
   */
  async sendRequest(receiver: string | string[], action: any): Promise<void> {
    await this.sendMessage({
      performative: 'request',
      receiver: Array.isArray(receiver) ? receiver.map(r => ({ name: r, address: r })) : [{ name: receiver, address: receiver }],
      content: { action },
      protocol: 'fipa-request',
    });
  }

  /**
   * Send PROPOSE message
   */
  async sendPropose(receiver: string | string[], proposal: any): Promise<void> {
    await this.sendMessage({
      performative: 'propose',
      receiver: Array.isArray(receiver) ? receiver.map(r => ({ name: r, address: r })) : [{ name: receiver, address: receiver }],
      content: proposal,
    });
  }

  /**
   * Register message handler
   */
  onMessage(handler: MessageHandler): void {
    this.messageHandlers.push(handler);
  }

  /**
   * Register task handler
   */
  onTask(handler: TaskHandler): void {
    this.taskHandlers.set('default', handler);
  }

  /**
   * Register event handler
   */
  onEvent(type: EventType, handler: EventHandler): void {
    this.on(type, handler);
  }

  /**
   * Execute task
   */
  async executeTask(task: Task): Promise<TaskResult> {
    this.status = 'busy';
    const startTime = Date.now();

    try {
      const handler = this.taskHandlers.get(task.type) || this.taskHandlers.get('default');

      if (!handler) {
        throw new Error(`No handler for task type: ${task.type}`);
      }

      const result = await handler(task);
      const duration = Date.now() - startTime;

      this.updateMetrics(true, duration, result.quality || 0);

      return {
        taskId: task.taskId,
        status: 'completed',
        result: result.result,
        quality: result.quality,
        completedAt: new Date(),
      };
    } catch (error) {
      this.updateMetrics(false, Date.now() - startTime, 0);

      return {
        taskId: task.taskId,
        status: 'failed',
        error: error instanceof Error ? error.message : String(error),
      };
    } finally {
      this.status = 'idle';
    }
  }

  /**
   * Get performance metrics
   */
  getMetrics(): PerformanceMetrics {
    return { ...this.metrics };
  }

  /**
   * Get reputation
   */
  getReputation(): Reputation {
    return {
      overall: 0.85,
      reliability: 0.90,
      quality: this.metrics.avgQuality,
      responsiveness: this.metrics.avgResponseTime > 0 ? 1000 / this.metrics.avgResponseTime : 1,
      interactionCount: this.metrics.messagesReceived + this.metrics.messagesSent,
      lastUpdated: new Date().toISOString(),
    };
  }

  /**
   * Handle incoming message
   */
  private async handleIncomingMessage(message: FIPAMessage): Promise<void> {
    this.metrics.messagesReceived++;
    this.emit('message-received', message);

    for (const handler of this.messageHandlers) {
      try {
        await handler(message);
      } catch (error) {
        console.error('Message handler error:', error);
      }
    }
  }

  /**
   * Handle event
   */
  private handleEvent(event: AgentEvent): void {
    this.emit(event.type, event);
  }

  /**
   * Update performance metrics
   */
  private updateMetrics(success: boolean, duration: number, quality: number): void {
    if (success) {
      this.metrics.tasksCompleted++;
    } else {
      this.metrics.tasksFailed++;
    }

    // Update average response time (exponential moving average)
    const alpha = 0.2;
    this.metrics.avgResponseTime = alpha * duration + (1 - alpha) * this.metrics.avgResponseTime;

    // Update average quality
    this.metrics.avgQuality = alpha * quality + (1 - alpha) * this.metrics.avgQuality;
  }

  /**
   * Start heartbeat
   */
  private startHeartbeat(): void {
    const interval = this.config.heartbeatInterval || 30000;

    setInterval(async () => {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, interval);
  }

  /**
   * Generate unique conversation ID
   */
  private generateConversationId(): string {
    return `${this.id}-${Date.now()}-${Math.random().toString(36).substring(7)}`;
  }
}

/**
 * Multi-Agent System
 */
export class MultiAgentSystem {
  private agents: Map<string, IAgent> = new Map();

  /**
   * Register agent
   */
  async registerAgent(agent: IAgent): Promise<void> {
    this.agents.set(agent.id, agent);
    await agent.start();
  }

  /**
   * Unregister agent
   */
  async unregisterAgent(agentId: string): Promise<void> {
    const agent = this.agents.get(agentId);
    if (agent) {
      await agent.stop();
      this.agents.delete(agentId);
    }
  }

  /**
   * Get agent by ID
   */
  getAgent(agentId: string): IAgent | undefined {
    return this.agents.get(agentId);
  }

  /**
   * Get all agents
   */
  getAllAgents(): IAgent[] {
    return Array.from(this.agents.values());
  }

  /**
   * Assign task (simple round-robin allocation)
   */
  async assignTask(task: Task): Promise<TaskResult> {
    const agents = Array.from(this.agents.values()).filter(a => a.status === 'idle');

    if (agents.length === 0) {
      throw new Error('No idle agents available');
    }

    const agent = agents[Math.floor(Math.random() * agents.length)];
    return await agent.executeTask(task);
  }

  /**
   * Broadcast message to all agents
   */
  async broadcastMessage(message: FIPAMessage): Promise<void> {
    await Promise.all(
      Array.from(this.agents.values()).map(agent =>
        agent.sendMessage(message)
      )
    );
  }

  /**
   * Start all agents
   */
  async start(): Promise<void> {
    await Promise.all(
      Array.from(this.agents.values()).map(agent => agent.start())
    );
  }

  /**
   * Stop all agents
   */
  async stop(): Promise<void> {
    await Promise.all(
      Array.from(this.agents.values()).map(agent => agent.stop())
    );
  }
}
