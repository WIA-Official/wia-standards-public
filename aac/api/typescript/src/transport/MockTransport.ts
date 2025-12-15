/**
 * WIA AAC Mock Transport
 * Mock implementation for testing purposes
 */

import {
  ITransport,
  TransportState,
  MessageHandler,
  OpenHandler,
  CloseHandler,
  ErrorHandler,
} from './ITransport';
import { WiaAacMessage, PROTOCOL_VERSION } from '../protocol/message';
import { MessageBuilder } from '../protocol/MessageBuilder';
import { SensorType } from '../types/signal';

export interface MockTransportOptions {
  /**
   * Delay before responding to messages (ms)
   */
  responseDelay?: number;

  /**
   * Simulate connection failure
   */
  simulateConnectionFailure?: boolean;

  /**
   * Available sensors for connect_ack
   */
  availableSensors?: Array<{
    type: SensorType;
    manufacturer: string;
    model: string;
    deviceId: string;
  }>;

  /**
   * Auto-generate signals after subscribe
   */
  autoGenerateSignals?: boolean;

  /**
   * Signal generation interval (ms)
   */
  signalInterval?: number;
}

const DEFAULT_OPTIONS: Required<MockTransportOptions> = {
  responseDelay: 10,
  simulateConnectionFailure: false,
  availableSensors: [
    {
      type: 'eye_tracker' as SensorType,
      manufacturer: 'MockSensor',
      model: 'Test Eye Tracker',
      deviceId: 'mock-et-001',
    },
  ],
  autoGenerateSignals: false,
  signalInterval: 100,
};

export class MockTransport implements ITransport {
  private options: Required<MockTransportOptions>;
  private state: TransportState = 'disconnected';
  private builder: MessageBuilder;

  private messageHandler: MessageHandler = () => {};
  private openHandler: OpenHandler = () => {};
  private closeHandler: CloseHandler = () => {};
  private errorHandler: ErrorHandler = () => {};

  private signalTimer: NodeJS.Timeout | null = null;
  private sessionId: string | null = null;
  private subscriptionId: string | null = null;
  private subscribedStreams: SensorType[] = [];

  constructor(options: MockTransportOptions = {}) {
    this.options = { ...DEFAULT_OPTIONS, ...options };
    this.builder = new MessageBuilder();
  }

  /**
   * Connect (simulated)
   */
  async connect(url: string): Promise<void> {
    if (this.options.simulateConnectionFailure) {
      this.state = 'error';
      throw new Error('Simulated connection failure');
    }

    this.state = 'connecting';

    await this.delay(this.options.responseDelay);

    this.state = 'connected';
    this.openHandler();
  }

  /**
   * Disconnect (simulated)
   */
  async disconnect(): Promise<void> {
    this.stopSignalGeneration();
    this.state = 'disconnected';
    this.sessionId = null;
    this.subscriptionId = null;
    this.subscribedStreams = [];
    this.closeHandler('Client disconnect');
  }

  /**
   * Send a message and simulate response
   */
  async send(message: WiaAacMessage): Promise<void> {
    if (this.state !== 'connected') {
      throw new Error('Not connected');
    }

    await this.delay(this.options.responseDelay);

    // Simulate server responses
    this.simulateResponse(message);
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.state === 'connected';
  }

  /**
   * Get transport state
   */
  getState(): TransportState {
    return this.state;
  }

  /**
   * Set message handler
   */
  onMessage(handler: MessageHandler): void {
    this.messageHandler = handler;
  }

  /**
   * Set open handler
   */
  onOpen(handler: OpenHandler): void {
    this.openHandler = handler;
  }

  /**
   * Set close handler
   */
  onClose(handler: CloseHandler): void {
    this.closeHandler = handler;
  }

  /**
   * Set error handler
   */
  onError(handler: ErrorHandler): void {
    this.errorHandler = handler;
  }

  // ========================================
  // Mock-specific methods
  // ========================================

  /**
   * Manually inject a message (for testing)
   */
  injectMessage(message: WiaAacMessage): void {
    this.messageHandler(JSON.stringify(message));
  }

  /**
   * Simulate sending a signal from the "server"
   */
  emitSignal(type: SensorType = 'eye_tracker'): void {
    const signal = this.createMockSignal(type);
    const message = this.builder.signal(signal);
    this.messageHandler(JSON.stringify(message));
  }

  // ========================================
  // Private methods
  // ========================================

  private async delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  private simulateResponse(message: WiaAacMessage): void {
    switch (message.type) {
      case 'connect':
        this.handleConnect(message);
        break;

      case 'disconnect':
        this.handleDisconnect(message);
        break;

      case 'subscribe':
        this.handleSubscribe(message);
        break;

      case 'unsubscribe':
        this.handleUnsubscribe(message);
        break;

      case 'command':
        this.handleCommand(message);
        break;

      case 'ping':
        this.handlePing(message);
        break;
    }
  }

  private handleConnect(message: WiaAacMessage): void {
    this.sessionId = `session-${Date.now()}`;
    const response = this.builder.connectAck(true, {
      sessionId: this.sessionId,
      serverName: 'WIA AAC Mock Server',
      serverVersion: '1.0.0',
      availableSensors: this.options.availableSensors,
      config: {
        maxSignalRate: 120,
        heartbeatInterval: 30000,
      },
    });
    this.messageHandler(JSON.stringify(response));
  }

  private handleDisconnect(message: WiaAacMessage): void {
    const response = this.builder.disconnectAck();
    this.messageHandler(JSON.stringify(response));
    this.stopSignalGeneration();
  }

  private handleSubscribe(message: WiaAacMessage): void {
    const payload = message.payload as { streams: SensorType[] };
    this.subscriptionId = `sub-${Date.now()}`;
    this.subscribedStreams = payload.streams || [];

    const response = this.builder.subscribeAck(true, {
      subscriptionId: this.subscriptionId,
      activeStreams: this.subscribedStreams,
      actualSignalRate: 60,
    });
    this.messageHandler(JSON.stringify(response));

    if (this.options.autoGenerateSignals) {
      this.startSignalGeneration();
    }
  }

  private handleUnsubscribe(message: WiaAacMessage): void {
    this.stopSignalGeneration();
    this.subscriptionId = null;
    this.subscribedStreams = [];

    const response = this.builder.unsubscribeAck(true);
    this.messageHandler(JSON.stringify(response));
  }

  private handleCommand(message: WiaAacMessage): void {
    const payload = message.payload as { command: string };
    const response = this.builder.commandAck(true, payload.command, {
      message: 'Command executed successfully',
    });
    this.messageHandler(JSON.stringify(response));
  }

  private handlePing(message: WiaAacMessage): void {
    const payload = message.payload as { sequence: number };
    const response = this.builder.pong(payload.sequence, 5);
    this.messageHandler(JSON.stringify(response));
  }

  private startSignalGeneration(): void {
    this.stopSignalGeneration();

    this.signalTimer = setInterval(() => {
      for (const stream of this.subscribedStreams) {
        this.emitSignal(stream);
      }
    }, this.options.signalInterval);
  }

  private stopSignalGeneration(): void {
    if (this.signalTimer) {
      clearInterval(this.signalTimer);
      this.signalTimer = null;
    }
  }

  private createMockSignal(type: SensorType): any {
    const baseSignal = {
      $schema: 'https://wia.live/aac/signal/v1/schema.json',
      version: '1.0.0',
      type,
      timestamp: {
        unix_ms: Date.now(),
        iso: new Date().toISOString(),
      },
      sequence: Math.floor(Math.random() * 10000),
      device: {
        manufacturer: 'MockSensor',
        model: `Test ${type}`,
      },
      meta: {
        confidence: 0.95,
      },
    };

    // Add type-specific data
    switch (type) {
      case 'eye_tracker':
        return {
          ...baseSignal,
          data: {
            gaze_point: { x: Math.random(), y: Math.random() },
            fixation: { active: false, duration_ms: 0 },
          },
        };

      case 'switch':
        return {
          ...baseSignal,
          data: {
            switch_id: 1,
            state: Math.random() > 0.5 ? 'pressed' : 'released',
            duration_ms: Math.floor(Math.random() * 500),
          },
        };

      case 'muscle_sensor':
        return {
          ...baseSignal,
          data: {
            channel_id: 1,
            activation_level: Math.random(),
            threshold_exceeded: Math.random() > 0.7,
          },
        };

      case 'brain_interface':
        return {
          ...baseSignal,
          data: {
            bands: {
              alpha: Math.random(),
              beta: Math.random(),
              theta: Math.random(),
              delta: Math.random(),
            },
            mental_command: 'neutral',
          },
        };

      case 'breath':
        return {
          ...baseSignal,
          data: {
            action: Math.random() > 0.5 ? 'sip' : 'puff',
            pressure_kpa: Math.random() * 5,
            intensity: 'soft',
          },
        };

      case 'head_movement':
        return {
          ...baseSignal,
          data: {
            position: { x: Math.random(), y: Math.random() },
            rotation: {
              pitch: (Math.random() - 0.5) * 20,
              yaw: (Math.random() - 0.5) * 20,
              roll: (Math.random() - 0.5) * 10,
            },
          },
        };

      default:
        return baseSignal;
    }
  }
}
