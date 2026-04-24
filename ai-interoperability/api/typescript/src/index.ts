/**
 * WIA AI Interoperability Standard - TypeScript SDK
 * Version: 1.0
 *
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This SDK provides a unified interface for interacting with different AI systems,
 * enabling seamless communication across platforms. By standardizing AI interactions,
 * we make advanced AI capabilities accessible to everyone.
 *
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  AIMessage,
  APIResponse,
  MessageResponse,
  SendOptions,
  StreamOptions,
  WIAClientConfig,
  CreateMessageParams,
  ListMessagesParams,
  CreateConversationParams,
  ListConversationsParams,
  Conversation,
  ModelInfo,
  ModelCapabilities,
  Frame,
  FrameType,
  ControlCommand,
  EventType,
  SenderType,
  ContentPartType,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// WIA Client - Main SDK Class
// ============================================================================

/**
 * Main WIA AI Interoperability Client
 *
 * Provides methods for:
 * - Sending messages to AI systems
 * - Streaming responses
 * - Managing conversations
 * - Discovering model capabilities
 * - Real-time communication via WebSocket
 *
 * @example
 * ```typescript
 * const client = new WIAClient({
 *   apiKey: 'wia_sk_abc123',
 *   defaultModel: 'claude-sonnet-4-5'
 * });
 *
 * const response = await client.sendMessage({
 *   content: 'Hello, AI!',
 *   senderId: 'user_123'
 * });
 *
 * console.log(response.data.message.content.parts[0].data);
 * ```
 */
export class WIAClient {
  private config: Required<WIAClientConfig>;
  private ws: WebSocket | null = null;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-ai-interop.org/v1',
      defaultModel: 'claude-sonnet-4-5',
      timeout: 60000,
      debug: false,
      headers: {},
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }

    if (this.config.debug) {
      console.log('[WIA Client] Initialized with config:', {
        baseURL: this.config.baseURL,
        defaultModel: this.config.defaultModel,
      });
    }
  }

  // ==========================================================================
  // Message Operations
  // ==========================================================================

  /**
   * Send a message to an AI system
   *
   * @param params - Message parameters
   * @param options - Send options
   * @returns API response with AI message
   *
   * @example
   * ```typescript
   * const response = await client.sendMessage({
   *   content: 'Explain quantum computing',
   *   senderId: 'user_123'
   * }, {
   *   maxTokens: 2000,
   *   temperature: 0.7
   * });
   * ```
   */
  async sendMessage(
    params: CreateMessageParams | AIMessage,
    options?: SendOptions
  ): Promise<APIResponse<MessageResponse>> {
    const message = this.isAIMessage(params)
      ? params
      : this.createMessage(params);

    const response = await this.fetch('/messages', {
      method: 'POST',
      body: JSON.stringify({
        message,
        options: options || {},
      }),
    });

    return response.json();
  }

  /**
   * Stream a message response from an AI system
   *
   * @param params - Message parameters
   * @param options - Stream options
   * @returns Async iterator of AI message chunks
   *
   * @example
   * ```typescript
   * for await (const chunk of client.streamMessage({
   *   content: 'Write a story',
   *   senderId: 'user_123'
   * })) {
   *   console.log(chunk.content.parts[0].data);
   * }
   * ```
   */
  async *streamMessage(
    params: CreateMessageParams | AIMessage,
    options?: Omit<StreamOptions, 'stream'>
  ): AsyncIterableIterator<AIMessage> {
    const message = this.isAIMessage(params)
      ? params
      : this.createMessage(params);

    const response = await this.fetch('/messages/stream', {
      method: 'POST',
      headers: {
        Accept: 'text/event-stream',
      },
      body: JSON.stringify({
        message,
        options: { ...options, stream: true },
      }),
    });

    if (!response.body) {
      throw new Error('Response body is null');
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    let currentMessage: Partial<AIMessage> = {
      ...message,
      sender: { type: 'ai', id: 'ai_assistant' },
      content: { type: 'text', parts: [] },
    };

    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (!line.trim() || line.startsWith(':')) continue;

          if (line.startsWith('event:')) {
            continue;
          }

          if (line.startsWith('data:')) {
            const data = line.slice(5).trim();
            if (!data) continue;

            try {
              const event = JSON.parse(data);

              if (event.delta?.text) {
                // Accumulate text delta
                if (currentMessage.content!.parts.length === 0) {
                  currentMessage.content!.parts.push({
                    type: 'text',
                    data: event.delta.text,
                  });
                } else {
                  currentMessage.content!.parts[0].data += event.delta.text;
                }

                yield currentMessage as AIMessage;
              }
            } catch (e) {
              if (this.config.debug) {
                console.error('[WIA Client] Failed to parse SSE data:', e);
              }
            }
          }
        }
      }
    } finally {
      reader.releaseLock();
    }
  }

  /**
   * Get a specific message by ID
   *
   * @param messageId - Message identifier
   * @returns API response with message
   */
  async getMessage(messageId: string): Promise<APIResponse<{ message: AIMessage }>> {
    const response = await this.fetch(`/messages/${messageId}`);
    return response.json();
  }

  /**
   * List messages in a conversation
   *
   * @param params - List parameters
   * @returns API response with messages and pagination
   */
  async listMessages(
    params: ListMessagesParams
  ): Promise<APIResponse<{ messages: AIMessage[]; pagination: any }>> {
    const queryParams = new URLSearchParams();
    if (params.limit) queryParams.set('limit', params.limit.toString());
    if (params.before) queryParams.set('before', params.before);
    if (params.after) queryParams.set('after', params.after);
    if (params.order) queryParams.set('order', params.order);

    const response = await this.fetch(
      `/conversations/${params.conversationId}/messages?${queryParams}`
    );
    return response.json();
  }

  // ==========================================================================
  // Conversation Operations
  // ==========================================================================

  /**
   * Create a new conversation
   *
   * @param params - Conversation parameters
   * @returns API response with conversation
   */
  async createConversation(
    params?: CreateConversationParams
  ): Promise<APIResponse<{ conversation: Conversation }>> {
    const response = await this.fetch('/conversations', {
      method: 'POST',
      body: JSON.stringify(params || {}),
    });
    return response.json();
  }

  /**
   * Get a conversation by ID
   *
   * @param conversationId - Conversation identifier
   * @returns API response with conversation
   */
  async getConversation(
    conversationId: string
  ): Promise<APIResponse<{ conversation: Conversation }>> {
    const response = await this.fetch(`/conversations/${conversationId}`);
    return response.json();
  }

  /**
   * List all conversations
   *
   * @param params - List parameters
   * @returns API response with conversations and pagination
   */
  async listConversations(
    params?: ListConversationsParams
  ): Promise<APIResponse<{ conversations: Conversation[]; pagination: any }>> {
    const queryParams = new URLSearchParams();
    if (params?.limit) queryParams.set('limit', params.limit.toString());
    if (params?.offset) queryParams.set('offset', params.offset.toString());
    if (params?.order) queryParams.set('order', params.order);

    const response = await this.fetch(`/conversations?${queryParams}`);
    return response.json();
  }

  /**
   * Delete a conversation
   *
   * @param conversationId - Conversation identifier
   * @returns API response with deletion confirmation
   */
  async deleteConversation(
    conversationId: string
  ): Promise<APIResponse<{ deleted: boolean; conversationId: string }>> {
    const response = await this.fetch(`/conversations/${conversationId}`, {
      method: 'DELETE',
    });
    return response.json();
  }

  // ==========================================================================
  // Model Operations
  // ==========================================================================

  /**
   * Get model capabilities
   *
   * @param modelId - Model identifier
   * @returns API response with model capabilities
   */
  async getModelCapabilities(
    modelId: string
  ): Promise<APIResponse<{ model: ModelInfo }>> {
    const response = await this.fetch(`/models/${modelId}/capabilities`);
    return response.json();
  }

  /**
   * List all available models
   *
   * @returns API response with models list
   */
  async listModels(): Promise<APIResponse<{ models: ModelInfo[] }>> {
    const response = await this.fetch('/models');
    return response.json();
  }

  // ==========================================================================
  // WebSocket / Real-time Operations
  // ==========================================================================

  /**
   * Connect to WebSocket for real-time communication
   *
   * @param options - Connection options
   * @returns Promise that resolves when connected
   *
   * @example
   * ```typescript
   * await client.connect();
   *
   * client.on('message', (message) => {
   *   console.log('Received:', message);
   * });
   *
   * client.on('event', (event) => {
   *   console.log('Event:', event);
   * });
   * ```
   */
  async connect(options?: { reconnect?: boolean }): Promise<void> {
    const wsURL = this.config.baseURL
      .replace('http://', 'ws://')
      .replace('https://', 'wss://')
      .replace('/v1', '/v1/connect');

    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(wsURL);

        this.ws.onopen = () => {
          if (this.config.debug) {
            console.log('[WIA Client] WebSocket connected');
          }

          // Send handshake
          this.sendFrame({
            type: 'control',
            id: 'handshake_' + Date.now(),
            timestamp: new Date().toISOString(),
            payload: {
              command: 'handshake' as ControlCommand,
              parameters: {
                protocolVersion: '1.0',
                clientId: 'ts_client_' + Date.now(),
                capabilities: ['streaming', 'multimodal'],
              },
            },
          });

          resolve();
        };

        this.ws.onmessage = (event) => {
          try {
            const frame: Frame = JSON.parse(event.data);
            this.handleFrame(frame);
          } catch (e) {
            if (this.config.debug) {
              console.error('[WIA Client] Failed to parse frame:', e);
            }
          }
        };

        this.ws.onerror = (error) => {
          if (this.config.debug) {
            console.error('[WIA Client] WebSocket error:', error);
          }
          this.eventEmitter.emit('error', error);
          reject(error);
        };

        this.ws.onclose = () => {
          if (this.config.debug) {
            console.log('[WIA Client] WebSocket closed');
          }
          this.eventEmitter.emit('disconnect');

          if (options?.reconnect) {
            setTimeout(() => this.connect(options), 5000);
          }
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Disconnect from WebSocket
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Subscribe to topics via WebSocket
   *
   * @param topics - Topics to subscribe to
   */
  subscribe(topics: string[]): void {
    this.sendFrame({
      type: 'control',
      id: 'sub_' + Date.now(),
      timestamp: new Date().toISOString(),
      payload: {
        command: 'subscribe' as ControlCommand,
        parameters: { topics },
      },
    });
  }

  /**
   * Unsubscribe from topics
   *
   * @param topics - Topics to unsubscribe from
   */
  unsubscribe(topics: string[]): void {
    this.sendFrame({
      type: 'control',
      id: 'unsub_' + Date.now(),
      timestamp: new Date().toISOString(),
      payload: {
        command: 'unsubscribe' as ControlCommand,
        parameters: { topics },
      },
    });
  }

  /**
   * Send a message via WebSocket
   *
   * @param message - AI message to send
   */
  sendRealtimeMessage(message: AIMessage): void {
    this.sendFrame({
      type: 'message',
      id: 'frame_' + Date.now(),
      timestamp: new Date().toISOString(),
      payload: {
        message,
        deliveryMode: 'immediate',
        priority: 'normal',
      },
    });
  }

  /**
   * Register event listener
   *
   * @param event - Event name
   * @param listener - Event listener function
   */
  on(event: string, listener: (...args: any[]) => void): void {
    this.eventEmitter.on(event, listener);
  }

  /**
   * Remove event listener
   *
   * @param event - Event name
   * @param listener - Event listener function
   */
  off(event: string, listener: (...args: any[]) => void): void {
    this.eventEmitter.off(event, listener);
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Create an AIMessage from simple parameters
   */
  createMessage(params: CreateMessageParams): AIMessage {
    return {
      version: '1.0',
      messageId: 'msg_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9),
      timestamp: new Date().toISOString(),
      sender: {
        type: params.senderType || 'human',
        id: params.senderId,
        name: params.senderName,
      },
      content: {
        type: 'text',
        parts: [
          {
            type: 'text',
            data: params.content,
          },
        ],
      },
      context: params.conversationId
        ? { conversationId: params.conversationId }
        : undefined,
      metadata: params.metadata,
    };
  }

  /**
   * Type guard for AIMessage
   */
  private isAIMessage(obj: any): obj is AIMessage {
    return (
      typeof obj === 'object' &&
      obj !== null &&
      'version' in obj &&
      'messageId' in obj &&
      'sender' in obj &&
      'content' in obj
    );
  }

  /**
   * Internal fetch wrapper
   */
  private async fetch(path: string, options: RequestInit = {}): Promise<Response> {
    const url = `${this.config.baseURL}${path}`;

    const headers = new Headers(options.headers);
    headers.set('Authorization', `Bearer ${this.config.apiKey}`);
    headers.set('Content-Type', 'application/json');

    for (const [key, value] of Object.entries(this.config.headers)) {
      headers.set(key, value);
    }

    if (this.config.debug) {
      console.log(`[WIA Client] ${options.method || 'GET'} ${url}`);
    }

    const response = await fetch(url, {
      ...options,
      headers,
      signal: AbortSignal.timeout(this.config.timeout),
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({
        message: response.statusText,
      }));
      throw new Error(`API Error: ${error.message || response.statusText}`);
    }

    return response;
  }

  /**
   * Send a frame via WebSocket
   */
  private sendFrame(frame: Frame): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket is not connected');
    }

    this.ws.send(JSON.stringify(frame));
  }

  /**
   * Handle incoming WebSocket frames
   */
  private handleFrame(frame: Frame): void {
    if (this.config.debug) {
      console.log('[WIA Client] Received frame:', frame.type);
    }

    switch (frame.type) {
      case 'message':
        this.eventEmitter.emit('message', frame.payload.message);
        break;
      case 'event':
        this.eventEmitter.emit('event', frame.payload);
        this.eventEmitter.emit(frame.payload.eventType, frame.payload.data);
        break;
      case 'control':
        if (frame.payload.command === 'ping') {
          // Respond to ping
          this.sendFrame({
            type: 'control',
            id: 'pong_' + Date.now(),
            timestamp: new Date().toISOString(),
            payload: { command: 'pong' as ControlCommand },
          });
        }
        this.eventEmitter.emit('control', frame.payload);
        break;
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Create a simple text message
 *
 * @param content - Message text
 * @param senderId - Sender ID
 * @param options - Additional options
 * @returns AIMessage object
 *
 * @example
 * ```typescript
 * const message = createTextMessage('Hello!', 'user_123');
 * ```
 */
export function createTextMessage(
  content: string,
  senderId: string,
  options?: {
    senderName?: string;
    senderType?: SenderType;
    conversationId?: string;
    metadata?: any;
  }
): AIMessage {
  return {
    version: '1.0',
    messageId: 'msg_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9),
    timestamp: new Date().toISOString(),
    sender: {
      type: options?.senderType || 'human',
      id: senderId,
      name: options?.senderName,
    },
    content: {
      type: 'text',
      parts: [{ type: 'text', data: content }],
    },
    context: options?.conversationId
      ? { conversationId: options.conversationId }
      : undefined,
    metadata: options?.metadata,
  };
}

/**
 * Create a multimodal message
 *
 * @param parts - Content parts
 * @param senderId - Sender ID
 * @param options - Additional options
 * @returns AIMessage object
 */
export function createMultimodalMessage(
  parts: Array<{ type: ContentPartType; data: any; metadata?: any }>,
  senderId: string,
  options?: {
    senderName?: string;
    senderType?: SenderType;
    conversationId?: string;
    metadata?: any;
  }
): AIMessage {
  return {
    version: '1.0',
    messageId: 'msg_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9),
    timestamp: new Date().toISOString(),
    sender: {
      type: options?.senderType || 'human',
      id: senderId,
      name: options?.senderName,
    },
    content: {
      type: 'multimodal',
      parts,
    },
    context: options?.conversationId
      ? { conversationId: options.conversationId }
      : undefined,
    metadata: options?.metadata,
  };
}

// ============================================================================
// Export SDK version
// ============================================================================

export const VERSION = '1.0.0';

/**
 * WIA AI Interoperability Standard
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This SDK enables seamless communication between different AI systems,
 * breaking down silos and creating a unified ecosystem for AI collaboration.
 */
export const WIA = {
  VERSION,
  Client: WIAClient,
  createTextMessage,
  createMultimodalMessage,
};

export default WIA;
