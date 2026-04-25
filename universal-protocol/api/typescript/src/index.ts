/**
 * WIA-CORE-007: Universal Protocol SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides a universal communication protocol for cross-platform,
 * cross-system message exchange and RPC (Remote Procedure Call).
 */

import {
  Message,
  RequestMessage,
  ResponseMessage,
  EventMessage,
  StreamMessage,
  ProtocolConfig,
  CallOptions,
  MethodHandler,
  CallContext,
  EventHandler,
  Subscription,
  Stream,
  StreamOptions,
  Middleware,
  ServerConfig,
  ServerOptions,
  ProtocolCapabilities,
  NegotiationRequest,
  NegotiationResponse,
  ConnectionState,
  ConnectionInfo,
  ProtocolStats,
  ProtocolError,
  ProtocolErrorCode,
  MessageType,
  MessageStatus,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-007 Universal Protocol SDK
 */
export class UniversalProtocol {
  private version = '1.0.0';
  private config: ProtocolConfig;
  private connectionState: ConnectionState = ConnectionState.DISCONNECTED;
  private subscriptions: Map<string, Subscription> = new Map();
  private pendingRequests: Map<string, {
    resolve: (value: ResponseMessage) => void;
    reject: (error: Error) => void;
    timeout?: NodeJS.Timeout;
  }> = new Map();
  private middleware: Middleware[] = [];
  private stats: ProtocolStats = {
    requestsSent: 0,
    responsesReceived: 0,
    eventsReceived: 0,
    errors: 0,
    averageLatency: 0,
    successRate: 1.0,
    uptime: 0,
  };
  private startTime: number = Date.now();

  constructor(config: ProtocolConfig) {
    this.config = config;
    if (config.middleware) {
      this.middleware = config.middleware;
    }
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get connection state
   */
  getConnectionState(): ConnectionState {
    return this.connectionState;
  }

  /**
   * Get connection info
   */
  getConnectionInfo(): ConnectionInfo {
    return {
      state: this.connectionState,
      endpoint: this.config.transport.endpoint,
      transport: this.config.transport.type,
      connectedAt: this.connectionState === ConnectionState.CONNECTED
        ? new Date(this.startTime)
        : undefined,
      lastActivity: new Date(),
      messagesSent: this.stats.requestsSent,
      messagesReceived: this.stats.responsesReceived + this.stats.eventsReceived,
    };
  }

  /**
   * Get protocol statistics
   */
  getStats(): ProtocolStats {
    return {
      ...this.stats,
      uptime: Date.now() - this.startTime,
    };
  }

  /**
   * Connect to server
   */
  async connect(): Promise<void> {
    this.connectionState = ConnectionState.CONNECTING;

    try {
      // Simulate connection (in real implementation, this would establish transport connection)
      await this.delay(100);

      this.connectionState = ConnectionState.CONNECTED;
      this.startTime = Date.now();
    } catch (error) {
      this.connectionState = ConnectionState.ERROR;
      throw new ProtocolError(
        ProtocolErrorCode.TRANSPORT_ERROR,
        'Failed to connect to server',
        error
      );
    }
  }

  /**
   * Disconnect from server
   */
  async disconnect(): Promise<void> {
    this.connectionState = ConnectionState.CLOSING;

    // Cancel all pending requests
    for (const [id, pending] of this.pendingRequests.entries()) {
      if (pending.timeout) {
        clearTimeout(pending.timeout);
      }
      pending.reject(new ProtocolError(
        ProtocolErrorCode.CONNECTION_CLOSED,
        'Connection closed'
      ));
    }
    this.pendingRequests.clear();

    // Clear all subscriptions
    this.subscriptions.clear();

    this.connectionState = ConnectionState.CLOSED;
  }

  /**
   * Send RPC request
   */
  async call(method: string, payload?: any, options?: CallOptions): Promise<ResponseMessage> {
    if (this.connectionState !== ConnectionState.CONNECTED) {
      throw new ProtocolError(
        ProtocolErrorCode.TRANSPORT_ERROR,
        'Not connected to server'
      );
    }

    const message = this.createRequestMessage(method, payload, options);

    return new Promise((resolve, reject) => {
      const timeout = options?.timeout || this.config.defaultTimeout || 30000;

      // Set timeout
      const timeoutId = setTimeout(() => {
        this.pendingRequests.delete(message.id);
        reject(new ProtocolError(
          ProtocolErrorCode.TIMEOUT,
          `Request timed out after ${timeout}ms`
        ));
      }, timeout);

      // Store pending request
      this.pendingRequests.set(message.id, {
        resolve,
        reject,
        timeout: timeoutId,
      });

      // Send message
      this.sendMessage(message)
        .then(() => {
          this.stats.requestsSent++;
        })
        .catch((error) => {
          clearTimeout(timeoutId);
          this.pendingRequests.delete(message.id);
          this.stats.errors++;
          reject(error);
        });
    });
  }

  /**
   * Subscribe to events
   */
  subscribe(channel: string, handler: EventHandler): Subscription {
    const id = this.generateId('sub');

    const subscription: Subscription = {
      id,
      channel,
      handler,
      unsubscribe: () => {
        this.subscriptions.delete(id);
      },
    };

    this.subscriptions.set(id, subscription);

    // Send subscription request
    this.sendMessage(this.createRequestMessage('events.subscribe', {
      channels: [channel],
    })).catch((error) => {
      console.error('Failed to subscribe:', error);
    });

    return subscription;
  }

  /**
   * Unsubscribe from events
   */
  unsubscribe(subscription: Subscription): void {
    subscription.unsubscribe();

    // Send unsubscribe request
    this.sendMessage(this.createRequestMessage('events.unsubscribe', {
      channels: [subscription.channel],
    })).catch((error) => {
      console.error('Failed to unsubscribe:', error);
    });
  }

  /**
   * Send custom message
   */
  async send(message: Message): Promise<void> {
    return this.sendMessage(message);
  }

  /**
   * Create stream
   */
  stream(method: string, payload?: any, options?: StreamOptions): Stream {
    const streamId = this.generateId('stream');
    let handlers: Map<string, Function[]> = new Map();

    const stream: Stream = {
      id: streamId,
      method,

      async send(data: any): Promise<void> {
        const message: StreamMessage = {
          id: streamId,
          version: '1.0.0',
          type: 'stream',
          method,
          payload: data,
          metadata: {
            sequence: 0,
            final: false,
          },
        };

        return this.sendMessage(message);
      },

      async end(): Promise<void> {
        const message: StreamMessage = {
          id: streamId,
          version: '1.0.0',
          type: 'stream',
          method,
          metadata: {
            sequence: -1,
            final: true,
          },
        };

        return this.sendMessage(message);
      },

      on(event: string, handler: Function): void {
        if (!handlers.has(event)) {
          handlers.set(event, []);
        }
        handlers.get(event)!.push(handler);
      },

      off(event: string, handler: Function): void {
        const eventHandlers = handlers.get(event);
        if (eventHandlers) {
          const index = eventHandlers.indexOf(handler);
          if (index > -1) {
            eventHandlers.splice(index, 1);
          }
        }
      },

      destroy(): void {
        handlers.clear();
      },
    };

    // Bind this context
    stream.send = stream.send.bind(this);
    stream.end = stream.end.bind(this);

    return stream;
  }

  /**
   * Add middleware
   */
  use(middleware: Middleware): void {
    this.middleware.push(middleware);
  }

  /**
   * Create message
   */
  createMessage(options: {
    type: MessageType;
    method?: string;
    payload?: any;
    headers?: Record<string, string>;
  }): Message {
    return {
      id: this.generateId(options.type),
      version: this.version,
      type: options.type,
      method: options.method,
      headers: {
        ...this.config.auth?.headers,
        ...options.headers,
      },
      payload: options.payload,
      metadata: {
        timestamp: new Date().toISOString(),
        source: 'client',
      },
    };
  }

  // ============================================================================
  // Private Methods
  // ============================================================================

  /**
   * Create request message
   */
  private createRequestMessage(
    method: string,
    payload?: any,
    options?: CallOptions
  ): RequestMessage {
    return {
      id: this.generateId('req'),
      version: this.version,
      type: 'request',
      method,
      headers: {
        ...this.config.auth?.headers,
        ...options?.headers,
      },
      payload,
      metadata: {
        timestamp: new Date().toISOString(),
        source: 'client',
        ttl: options?.timeout,
      },
    };
  }

  /**
   * Send message through transport
   */
  private async sendMessage(message: Message): Promise<void> {
    // Apply middleware
    let processedMessage = message;
    for (const mw of this.middleware) {
      processedMessage = await mw(processedMessage, async () => processedMessage);
    }

    // In real implementation, this would send via actual transport
    // For now, we simulate the send operation
    if (this.config.debug) {
      console.log('Sending message:', processedMessage);
    }

    await this.delay(10);
  }

  /**
   * Handle incoming message
   */
  private handleIncomingMessage(message: Message): void {
    if (message.type === 'response') {
      this.handleResponse(message as ResponseMessage);
    } else if (message.type === 'event') {
      this.handleEvent(message as EventMessage);
    } else if (message.type === 'stream') {
      this.handleStream(message as StreamMessage);
    }
  }

  /**
   * Handle response message
   */
  private handleResponse(response: ResponseMessage): void {
    const pending = this.pendingRequests.get(response.id);
    if (pending) {
      if (pending.timeout) {
        clearTimeout(pending.timeout);
      }

      this.pendingRequests.delete(response.id);
      this.stats.responsesReceived++;

      if (response.status === 'success') {
        pending.resolve(response);
      } else {
        pending.reject(new ProtocolError(
          response.error?.code || ProtocolErrorCode.INTERNAL,
          response.error?.message || 'Unknown error',
          response.error?.details
        ));
        this.stats.errors++;
      }
    }
  }

  /**
   * Handle event message
   */
  private handleEvent(event: EventMessage): void {
    this.stats.eventsReceived++;

    for (const subscription of this.subscriptions.values()) {
      // In real implementation, we would check if the event matches the channel
      subscription.handler(event);
    }
  }

  /**
   * Handle stream message
   */
  private handleStream(stream: StreamMessage): void {
    // In real implementation, this would route to the appropriate stream handler
    if (this.config.debug) {
      console.log('Stream message received:', stream);
    }
  }

  /**
   * Generate unique ID
   */
  private generateId(prefix: string): string {
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).substring(2, 15);
    return `${prefix}_${timestamp}${random}`;
  }

  /**
   * Delay helper
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// RPC Client
// ============================================================================

/**
 * RPC Client for simplified RPC calls
 */
export class RPCClient {
  private protocol: UniversalProtocol;

  constructor(config: ProtocolConfig) {
    this.protocol = new UniversalProtocol(config);
  }

  /**
   * Connect to server
   */
  async connect(): Promise<void> {
    return this.protocol.connect();
  }

  /**
   * Disconnect from server
   */
  async disconnect(): Promise<void> {
    return this.protocol.disconnect();
  }

  /**
   * Call RPC method
   */
  async call(method: string, payload?: any, options?: CallOptions): Promise<any> {
    const response = await this.protocol.call(method, payload, options);
    return response.payload;
  }

  /**
   * Get connection info
   */
  getConnectionInfo(): ConnectionInfo {
    return this.protocol.getConnectionInfo();
  }
}

// ============================================================================
// RPC Server
// ============================================================================

/**
 * RPC Server for handling incoming requests
 */
export class RPCServer {
  private config: ServerConfig;
  private methods: Map<string, MethodHandler> = new Map();
  private middleware: Middleware[] = [];
  private running: boolean = false;

  constructor(config: ServerConfig) {
    this.config = config;
  }

  /**
   * Start server
   */
  async start(): Promise<void> {
    if (this.running) {
      throw new ProtocolError(
        ProtocolErrorCode.INTERNAL,
        'Server already running'
      );
    }

    // In real implementation, this would start the actual server
    this.running = true;
    console.log(`Server started on ${this.config.host || '0.0.0.0'}:${this.config.port}`);
  }

  /**
   * Stop server
   */
  async stop(): Promise<void> {
    if (!this.running) {
      throw new ProtocolError(
        ProtocolErrorCode.INTERNAL,
        'Server not running'
      );
    }

    // In real implementation, this would stop the actual server
    this.running = false;
    console.log('Server stopped');
  }

  /**
   * Register RPC method
   */
  register(method: string, handler: MethodHandler): void {
    this.methods.set(method, handler);
  }

  /**
   * Unregister RPC method
   */
  unregister(method: string): void {
    this.methods.delete(method);
  }

  /**
   * Add middleware
   */
  use(middleware: Middleware): void {
    this.middleware.push(middleware);
  }

  /**
   * Handle incoming request
   */
  private async handleRequest(
    request: RequestMessage,
    context: CallContext
  ): Promise<ResponseMessage> {
    const handler = this.methods.get(request.method!);

    if (!handler) {
      return {
        id: request.id,
        version: '1.0.0',
        type: 'response',
        status: 'error',
        error: {
          code: ProtocolErrorCode.NOT_FOUND,
          message: `Method '${request.method}' not found`,
        },
      };
    }

    try {
      const result = await handler(request.payload, context);

      return {
        id: request.id,
        version: '1.0.0',
        type: 'response',
        status: 'success',
        payload: result,
        metadata: {
          timestamp: new Date().toISOString(),
        },
      };
    } catch (error: any) {
      return {
        id: request.id,
        version: '1.0.0',
        type: 'response',
        status: 'error',
        error: {
          code: error.code || ProtocolErrorCode.INTERNAL,
          message: error.message || 'Internal server error',
          details: error.details,
        },
      };
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create RPC client (standalone function)
 */
export function createRPCClient(config: ProtocolConfig): RPCClient {
  return new RPCClient(config);
}

/**
 * Create RPC server (standalone function)
 */
export function createRPCServer(config: ServerConfig): RPCServer {
  return new RPCServer(config);
}

/**
 * Create protocol instance (standalone function)
 */
export function createProtocol(config: ProtocolConfig): UniversalProtocol {
  return new UniversalProtocol(config);
}

/**
 * Validate message format
 */
export function validateMessage(message: any): boolean {
  if (!message || typeof message !== 'object') {
    return false;
  }

  // Check required fields
  if (!message.id || typeof message.id !== 'string') {
    return false;
  }

  if (!message.version || typeof message.version !== 'string') {
    return false;
  }

  if (!message.type || !['request', 'response', 'event', 'stream'].includes(message.type)) {
    return false;
  }

  return true;
}

/**
 * Negotiate protocol version and capabilities
 */
export async function negotiateProtocol(
  request: NegotiationRequest
): Promise<NegotiationResponse> {
  // Simple version selection (choose latest compatible version)
  const compatibleVersion = request.versions.find(v => v === '1.0.0') || '1.0.0';

  return {
    version: compatibleVersion,
    features: request.features || [],
    capabilities: {
      versions: ['1.0.0'],
      transports: ['http', 'websocket', 'tcp', 'udp'],
      serialization: ['json', 'messagepack'],
      compression: ['gzip', 'brotli'],
      max_message_size: 10485760,
      streaming: true,
      bidirectional: true,
    },
  };
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export {
  UniversalProtocol,
  RPCClient,
  RPCServer,
};
export default UniversalProtocol;
