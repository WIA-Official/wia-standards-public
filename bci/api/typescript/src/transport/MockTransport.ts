/**
 * WIA BCI Mock Transport
 *
 * Mock transport for development and testing.
 */

import { BaseTransport } from './ITransport';
import { BciMessage, TransportOptions, SignalPayload } from '../protocol/types';
import { MessageBuilder } from '../protocol/MessageBuilder';

/**
 * Mock transport options
 */
export interface MockTransportOptions extends TransportOptions {
  latency?: number;          // Simulated latency in ms (default: 10)
  dropRate?: number;         // Message drop rate 0-1 (default: 0)
  disconnectChance?: number; // Random disconnect chance per message (default: 0)
  autoStream?: boolean;      // Auto-generate signals when streaming (default: true)
  signalInterval?: number;   // Signal interval in ms (default: 4 = 250Hz)
}

/**
 * Mock transport implementation for testing
 */
export class MockTransport extends BaseTransport {
  private latency: number;
  private dropRate: number;
  private disconnectChance: number;
  private autoStream: boolean;
  private signalInterval: number;
  private messageBuilder: MessageBuilder;
  private streamTimer: ReturnType<typeof setInterval> | null = null;
  private sampleIndex: number = 0;
  private channels: string[] = ['Fp1', 'Fp2', 'C3', 'C4', 'O1', 'O2', 'P3', 'P4'];
  private sessionId: string | null = null;
  private isStreaming: boolean = false;

  constructor(options: MockTransportOptions = {}) {
    super();
    this.latency = options.latency ?? 10;
    this.dropRate = options.dropRate ?? 0;
    this.disconnectChance = options.disconnectChance ?? 0;
    this.autoStream = options.autoStream ?? true;
    this.signalInterval = options.signalInterval ?? 4;
    this.messageBuilder = new MessageBuilder();
  }

  /**
   * Connect to mock server
   */
  async connect(url: string, options?: MockTransportOptions): Promise<void> {
    if (this.state !== 'disconnected') {
      throw new Error('Already connected');
    }

    if (options?.latency !== undefined) this.latency = options.latency;
    if (options?.dropRate !== undefined) this.dropRate = options.dropRate;
    if (options?.disconnectChance !== undefined) this.disconnectChance = options.disconnectChance;

    this.state = 'connecting';

    await this.delay(this.latency);

    this.state = 'connected';
    this.sessionId = `mock-session-${Date.now()}`;
    this.messageBuilder.setSessionId(this.sessionId);
    this.emitOpen();
  }

  /**
   * Disconnect
   */
  async disconnect(): Promise<void> {
    this.stopStreaming();
    this.state = 'disconnected';
    this.emitClose(1000, 'Client disconnect');
  }

  /**
   * Send message
   */
  async send(message: BciMessage): Promise<void> {
    if (this.state !== 'connected') {
      throw new Error('Not connected');
    }

    // Simulate random disconnect
    if (Math.random() < this.disconnectChance) {
      this.handleRandomDisconnect();
      return;
    }

    // Simulate message drop
    if (Math.random() < this.dropRate) {
      return;
    }

    await this.delay(this.latency);

    // Handle message types
    this.handleMessage(message);
  }

  /**
   * Handle incoming message (from client)
   */
  private handleMessage(message: BciMessage): void {
    switch (message.type) {
      case 'connect':
        this.handleConnect(message);
        break;
      case 'start_stream':
        this.handleStartStream();
        break;
      case 'stop_stream':
        this.handleStopStream();
        break;
      case 'ping':
        this.handlePing(message);
        break;
      case 'marker':
        // Echo marker back
        this.emitMessage(message);
        break;
      default:
        // No response needed
        break;
    }
  }

  /**
   * Handle connect message
   */
  private handleConnect(message: BciMessage): void {
    const response = this.messageBuilder.connectAck({
      sessionId: this.sessionId!,
      status: 'connected',
      serverInfo: {
        name: 'MockServer',
        version: '1.0.0',
      },
      deviceInfo: {
        type: 'simulator',
        manufacturer: 'WIA',
        model: 'Mock Device',
        channels: this.channels.length,
        samplingRate: 250,
      },
      negotiated: {
        samplingRate: 250,
        channels: this.channels,
        compression: false,
      },
    });

    this.emitMessage(response);
  }

  /**
   * Handle start stream
   */
  private handleStartStream(): void {
    // Send ack
    const ack = this.messageBuilder.streamAck({ status: 'started' });
    this.emitMessage(ack);

    // Start generating signals
    if (this.autoStream) {
      this.startStreaming();
    }
  }

  /**
   * Handle stop stream
   */
  private handleStopStream(): void {
    this.stopStreaming();

    const ack = this.messageBuilder.streamAck({ status: 'stopped' });
    this.emitMessage(ack);
  }

  /**
   * Handle ping
   */
  private handlePing(message: BciMessage): void {
    const payload = message.payload as { clientTime?: number };
    const pong = this.messageBuilder.pong(payload.clientTime);
    this.emitMessage(pong);
  }

  /**
   * Start streaming signals
   */
  private startStreaming(): void {
    if (this.isStreaming) return;

    this.isStreaming = true;
    this.sampleIndex = 0;

    this.streamTimer = setInterval(() => {
      this.generateSignal();
    }, this.signalInterval);
  }

  /**
   * Stop streaming
   */
  private stopStreaming(): void {
    if (this.streamTimer) {
      clearInterval(this.streamTimer);
      this.streamTimer = null;
    }
    this.isStreaming = false;
  }

  /**
   * Generate mock signal
   */
  private generateSignal(): void {
    const timestamp = Date.now();
    const data: number[] = [];

    // Generate simulated EEG data for each channel
    for (let i = 0; i < this.channels.length; i++) {
      // Simulate EEG signal: alpha (10 Hz) + noise
      const alpha = 10 * Math.sin(2 * Math.PI * 10 * (this.sampleIndex / 250));
      const beta = 5 * Math.sin(2 * Math.PI * 20 * (this.sampleIndex / 250));
      const noise = (Math.random() - 0.5) * 5;
      data.push(alpha + beta + noise);
    }

    const payload: SignalPayload = {
      sampleIndex: this.sampleIndex,
      timestamp,
      channels: this.channels.map((_, i) => i),
      data,
    };

    const signal = this.messageBuilder.signal(payload);
    this.emitMessage(signal);

    this.sampleIndex++;
  }

  /**
   * Handle random disconnect
   */
  private handleRandomDisconnect(): void {
    this.stopStreaming();
    this.state = 'disconnected';
    this.emitClose(4001, 'Random disconnect (simulated)');
  }

  /**
   * Delay helper
   */
  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Set channels for simulation
   */
  setChannels(channels: string[]): void {
    this.channels = channels;
  }

  /**
   * Inject a marker event
   */
  injectMarker(code: number, label: string, value?: string): void {
    const marker = this.messageBuilder.marker({
      sampleIndex: this.sampleIndex,
      code,
      label,
      value,
    });
    this.emitMessage(marker);
  }

  /**
   * Dispose
   */
  dispose(): void {
    this.stopStreaming();
    super.dispose();
  }
}
