/**
 * WIA Eye Gaze Standard - Gaze Aware Application
 *
 * Enables applications to participate in the gaze-aware ecosystem.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

import {
  GazeTarget,
  GazeAppMessage,
  GazeAppMessageType,
  AppCapabilities,
  TransportConfig,
  TransportType,
} from '../types';

export interface GazeAwareAppOptions {
  /** Unique application ID */
  appId: string;
  /** Application capabilities */
  capabilities: AppCapabilities;
  /** Transport configuration */
  transport?: TransportConfig;
}

export type ControlRequestHandler = (appId: string) => boolean;
export type MessageHandler = (message: GazeAppMessage) => void;

/**
 * Gaze Aware Application
 *
 * Allows applications to register in the gaze-aware ecosystem,
 * coordinate with other applications, and manage gaze control.
 *
 * @example
 * ```typescript
 * const app = new GazeAwareApp({
 *   appId: 'my-aac-app',
 *   capabilities: {
 *     name: 'My AAC App',
 *     version: '1.0.0',
 *     supportsControl: true,
 *     supportsDwell: true,
 *     supportsBlink: false,
 *     priority: 10,
 *   },
 * });
 *
 * await app.register();
 *
 * // Request control when needed
 * app.announceGazeControl();
 *
 * // Handle control requests from other apps
 * app.onGazeControlRequest(requestingAppId => {
 *   // Return true to grant, false to deny
 *   return requestingAppId === 'system-settings';
 * });
 * ```
 */
export class GazeAwareApp {
  private appId: string;
  private capabilities: AppCapabilities;
  private transport: AppTransport;
  private registered: boolean;
  private hasControl: boolean;
  private targets: Map<string, GazeTarget>;
  private controlRequestHandler: ControlRequestHandler | null;
  private messageHandlers: Set<MessageHandler>;
  private knownApps: Map<string, AppCapabilities>;

  constructor(options: GazeAwareAppOptions) {
    this.appId = options.appId;
    this.capabilities = options.capabilities;
    this.transport = createTransport(options.transport ?? { type: 'broadcast_channel' });
    this.registered = false;
    this.hasControl = false;
    this.targets = new Map();
    this.controlRequestHandler = null;
    this.messageHandlers = new Set();
    this.knownApps = new Map();

    this.setupTransportHandlers();
  }

  // ============================================
  // Registration
  // ============================================

  /**
   * Register with the gaze-aware ecosystem
   */
  async register(): Promise<void> {
    if (this.registered) return;

    await this.transport.connect();
    this.registered = true;

    // Announce presence
    this.sendMessage({
      type: GazeAppMessageType.APP_ANNOUNCE,
      appId: this.appId,
      timestamp: Date.now(),
      payload: this.capabilities,
    });

    // Query for existing apps
    this.sendMessage({
      type: GazeAppMessageType.APP_QUERY,
      appId: this.appId,
      timestamp: Date.now(),
    });
  }

  /**
   * Unregister from the ecosystem
   */
  async unregister(): Promise<void> {
    if (!this.registered) return;

    if (this.hasControl) {
      this.releaseGazeControl();
    }

    await this.transport.disconnect();
    this.registered = false;
  }

  /**
   * Check if registered
   */
  isRegistered(): boolean {
    return this.registered;
  }

  // ============================================
  // Gaze Control
  // ============================================

  /**
   * Announce that this app is taking gaze control
   */
  announceGazeControl(): void {
    this.sendMessage({
      type: GazeAppMessageType.CONTROL_REQUEST,
      appId: this.appId,
      timestamp: Date.now(),
      payload: { priority: this.capabilities.priority ?? 0 },
    });
  }

  /**
   * Release gaze control
   */
  releaseGazeControl(): void {
    if (!this.hasControl) return;

    this.hasControl = false;
    this.sendMessage({
      type: GazeAppMessageType.CONTROL_RELEASE,
      appId: this.appId,
      timestamp: Date.now(),
    });
  }

  /**
   * Check if this app has gaze control
   */
  hasGazeControl(): boolean {
    return this.hasControl;
  }

  /**
   * Register handler for control requests from other apps
   */
  onGazeControlRequest(handler: ControlRequestHandler): void {
    this.controlRequestHandler = handler;
  }

  // ============================================
  // Target Management
  // ============================================

  /**
   * Register a gaze target
   */
  registerTarget(target: GazeTarget): void {
    this.targets.set(target.elementId, target);
    this.syncTargets();
  }

  /**
   * Unregister a gaze target
   */
  unregisterTarget(targetId: string): void {
    this.targets.delete(targetId);
    this.syncTargets();
  }

  /**
   * Get all active targets
   */
  getActiveTargets(): GazeTarget[] {
    return Array.from(this.targets.values());
  }

  /**
   * Sync targets with other apps
   */
  private syncTargets(): void {
    if (!this.hasControl) return;

    this.sendMessage({
      type: GazeAppMessageType.TARGET_SYNC,
      appId: this.appId,
      timestamp: Date.now(),
      payload: { targets: Array.from(this.targets.values()) },
    });
  }

  // ============================================
  // Tracking Control
  // ============================================

  /**
   * Request other apps to pause tracking
   */
  requestPauseTracking(): void {
    this.sendMessage({
      type: GazeAppMessageType.PAUSE_TRACKING,
      appId: this.appId,
      timestamp: Date.now(),
    });
  }

  /**
   * Request other apps to resume tracking
   */
  requestResumeTracking(): void {
    this.sendMessage({
      type: GazeAppMessageType.RESUME_TRACKING,
      appId: this.appId,
      timestamp: Date.now(),
    });
  }

  // ============================================
  // App Discovery
  // ============================================

  /**
   * Get list of known gaze-aware apps
   */
  getKnownApps(): Map<string, AppCapabilities> {
    return new Map(this.knownApps);
  }

  /**
   * Check if a specific app is registered
   */
  isAppRegistered(appId: string): boolean {
    return this.knownApps.has(appId);
  }

  // ============================================
  // Message Handling
  // ============================================

  /**
   * Register a message handler for all messages
   */
  onMessage(handler: MessageHandler): void {
    this.messageHandlers.add(handler);
  }

  /**
   * Remove a message handler
   */
  offMessage(handler: MessageHandler): void {
    this.messageHandlers.delete(handler);
  }

  private setupTransportHandlers(): void {
    this.transport.onMessage((message: GazeAppMessage) => {
      // Ignore own messages
      if (message.appId === this.appId) return;

      this.handleMessage(message);

      // Forward to custom handlers
      for (const handler of this.messageHandlers) {
        try {
          handler(message);
        } catch (error) {
          console.error('Error in message handler:', error);
        }
      }
    });
  }

  private handleMessage(message: GazeAppMessage): void {
    switch (message.type) {
      case GazeAppMessageType.APP_ANNOUNCE:
        this.handleAppAnnounce(message);
        break;
      case GazeAppMessageType.APP_QUERY:
        this.handleAppQuery(message);
        break;
      case GazeAppMessageType.APP_RESPONSE:
        this.handleAppResponse(message);
        break;
      case GazeAppMessageType.CONTROL_REQUEST:
        this.handleControlRequest(message);
        break;
      case GazeAppMessageType.CONTROL_GRANT:
        this.handleControlGrant(message);
        break;
      case GazeAppMessageType.CONTROL_DENY:
        this.handleControlDeny(message);
        break;
      case GazeAppMessageType.CONTROL_RELEASE:
        this.handleControlRelease(message);
        break;
    }
  }

  private handleAppAnnounce(message: GazeAppMessage): void {
    const capabilities = message.payload as AppCapabilities;
    this.knownApps.set(message.appId, capabilities);
  }

  private handleAppQuery(_message: GazeAppMessage): void {
    // Respond with our info
    this.sendMessage({
      type: GazeAppMessageType.APP_RESPONSE,
      appId: this.appId,
      timestamp: Date.now(),
      payload: this.capabilities,
    });
  }

  private handleAppResponse(message: GazeAppMessage): void {
    const capabilities = message.payload as AppCapabilities;
    this.knownApps.set(message.appId, capabilities);
  }

  private handleControlRequest(message: GazeAppMessage): void {
    if (!this.hasControl) return;

    const granted = this.controlRequestHandler
      ? this.controlRequestHandler(message.appId)
      : false;

    this.sendMessage({
      type: granted ? GazeAppMessageType.CONTROL_GRANT : GazeAppMessageType.CONTROL_DENY,
      appId: this.appId,
      timestamp: Date.now(),
      payload: { requestingApp: message.appId },
    });

    if (granted) {
      this.hasControl = false;
    }
  }

  private handleControlGrant(_message: GazeAppMessage): void {
    this.hasControl = true;
    this.syncTargets();
  }

  private handleControlDeny(_message: GazeAppMessage): void {
    // Control request was denied
    console.log(`[GazeAwareApp] Control request denied`);
  }

  private handleControlRelease(_message: GazeAppMessage): void {
    // Another app released control - we can potentially claim it
    if (this.capabilities.supportsControl && this.capabilities.priority) {
      // Auto-request control if we have high priority
      setTimeout(() => {
        if (!this.hasControl) {
          this.announceGazeControl();
        }
      }, 100);
    }
  }

  private sendMessage(message: GazeAppMessage): void {
    if (!this.registered) return;
    this.transport.send(message);
  }

  // ============================================
  // Lifecycle
  // ============================================

  /**
   * Dispose of resources
   */
  dispose(): void {
    this.unregister();
    this.targets.clear();
    this.knownApps.clear();
    this.messageHandlers.clear();
  }
}

// ============================================
// Transport Implementations
// ============================================

interface AppTransport {
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  send(message: GazeAppMessage): void;
  onMessage(handler: (message: GazeAppMessage) => void): void;
}

class BroadcastChannelTransport implements AppTransport {
  private channel: BroadcastChannel | null = null;
  private messageHandler: ((message: GazeAppMessage) => void) | null = null;

  async connect(): Promise<void> {
    if (typeof BroadcastChannel !== 'undefined') {
      this.channel = new BroadcastChannel('wia-eye-gaze');
      this.channel.onmessage = (event) => {
        if (this.messageHandler) {
          this.messageHandler(event.data as GazeAppMessage);
        }
      };
    }
  }

  async disconnect(): Promise<void> {
    this.channel?.close();
    this.channel = null;
  }

  send(message: GazeAppMessage): void {
    this.channel?.postMessage(message);
  }

  onMessage(handler: (message: GazeAppMessage) => void): void {
    this.messageHandler = handler;
  }
}

class WebSocketTransport implements AppTransport {
  private endpoint: string;
  private ws: WebSocket | null = null;
  private messageHandler: ((message: GazeAppMessage) => void) | null = null;

  constructor(endpoint: string) {
    this.endpoint = endpoint;
  }

  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.endpoint);
      this.ws.onopen = () => resolve();
      this.ws.onerror = (error) => reject(error);
      this.ws.onmessage = (event) => {
        if (this.messageHandler) {
          try {
            const message = JSON.parse(event.data) as GazeAppMessage;
            this.messageHandler(message);
          } catch (e) {
            console.error('Failed to parse message:', e);
          }
        }
      };
    });
  }

  async disconnect(): Promise<void> {
    this.ws?.close();
    this.ws = null;
  }

  send(message: GazeAppMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  onMessage(handler: (message: GazeAppMessage) => void): void {
    this.messageHandler = handler;
  }
}

class NoopTransport implements AppTransport {
  async connect(): Promise<void> {}
  async disconnect(): Promise<void> {}
  send(_message: GazeAppMessage): void {}
  onMessage(_handler: (message: GazeAppMessage) => void): void {}
}

function createTransport(config: TransportConfig): AppTransport {
  switch (config.type) {
    case 'broadcast_channel':
      return new BroadcastChannelTransport();
    case 'websocket':
      return new WebSocketTransport(config.endpoint ?? 'ws://localhost:8080');
    case 'ipc':
      // IPC transport would be implemented for Electron/Node.js apps
      return new NoopTransport();
    default:
      return new NoopTransport();
  }
}

/**
 * Create a gaze-aware application
 */
export function createGazeAwareApp(options: GazeAwareAppOptions): GazeAwareApp {
  return new GazeAwareApp(options);
}
