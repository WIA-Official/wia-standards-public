/**
 * WIA-TLS-LITE - Lightweight Transport Layer Security
 * Optimized for IoT and Constrained Devices
 *
 * @packageDocumentation
 */

import {
  TLSVersion,
  CipherSuite,
  KeyExchange,
  CertificateType,
  Certificate,
  HandshakeState,
  HandshakeConfig,
  TLSSession,
  ALPNProtocol,
  SNIConfig,
  TLSEventType,
  TLSEvent,
  CertificateValidationOptions,
  SecureChannelConfig,
  TLSStats
} from './types';

export * from './types';

/**
 * WIA TLS-Lite Client/Server Implementation
 * Provides lightweight TLS for IoT devices with minimal overhead
 */
export class WIATLSLite {
  private version: string = '1.0.0';
  private sessions: Map<string, TLSSession> = new Map();
  private eventHandlers: Map<TLSEventType, Set<(event: TLSEvent) => void>> = new Map();
  private stats: TLSStats;
  private handshakeState: HandshakeState = HandshakeState.IDLE;
  private currentSession?: TLSSession;

  constructor() {
    this.stats = {
      totalHandshakes: 0,
      successfulHandshakes: 0,
      failedHandshakes: 0,
      sessionResumptions: 0,
      bytesSent: 0,
      bytesReceived: 0,
      avgHandshakeTime: 0,
      activeSessions: 0
    };
  }

  /**
   * Perform TLS handshake
   * Establishes secure connection with server or client
   */
  async handshake(config: HandshakeConfig): Promise<TLSSession> {
    const startTime = Date.now();
    this.handshakeState = HandshakeState.CLIENT_HELLO_SENT;
    this.stats.totalHandshakes++;

    this.emitEvent({
      type: TLSEventType.HANDSHAKE_START,
      timestamp: new Date(),
      data: { config }
    });

    try {
      // Check for session resumption
      if (config.sessionTicket && config.enableSessionResumption) {
        const resumedSession = await this.resumeSession(config.sessionTicket);
        if (resumedSession) {
          this.stats.sessionResumptions++;
          this.handshakeState = HandshakeState.COMPLETED;

          this.emitEvent({
            type: TLSEventType.SESSION_RESUMED,
            timestamp: new Date(),
            sessionId: resumedSession.id
          });

          return resumedSession;
        }
      }

      // Perform full handshake
      const session = await this.performFullHandshake(config);

      // Update statistics
      const handshakeTime = Date.now() - startTime;
      this.updateHandshakeStats(handshakeTime, true);

      // Store session
      this.sessions.set(session.id, session);
      this.currentSession = session;
      this.stats.activeSessions = this.sessions.size;

      this.handshakeState = HandshakeState.COMPLETED;

      this.emitEvent({
        type: TLSEventType.HANDSHAKE_COMPLETE,
        timestamp: new Date(),
        sessionId: session.id,
        data: { session }
      });

      return session;

    } catch (error) {
      this.handshakeState = HandshakeState.FAILED;
      this.stats.failedHandshakes++;

      this.emitEvent({
        type: TLSEventType.HANDSHAKE_FAILED,
        timestamp: new Date(),
        error: error as Error
      });

      throw error;
    }
  }

  /**
   * Perform full TLS handshake (non-resumption)
   */
  private async performFullHandshake(config: HandshakeConfig): Promise<TLSSession> {
    // Simulate handshake process (in real implementation, this would do actual TLS negotiation)
    const sessionId = this.generateSessionId();
    const now = new Date();
    const expiresAt = new Date(now.getTime() + (24 * 60 * 60 * 1000)); // 24 hours

    // Negotiate cipher suite (select first available)
    const cipherSuite = config.cipherSuites[0] || CipherSuite.TLS_AES_128_GCM_SHA256;

    // Negotiate ALPN protocol
    const alpnProtocol = config.alpnProtocols?.[0];

    const session: TLSSession = {
      id: sessionId,
      version: config.version,
      cipherSuite,
      serverName: config.serverName,
      alpnProtocol,
      createdAt: now,
      lastUsedAt: now,
      expiresAt,
      resumable: config.enableSessionResumption ?? true,
      ticket: config.enableSessionResumption ? this.generateSessionTicket() : undefined
    };

    this.emitEvent({
      type: TLSEventType.SESSION_CREATED,
      timestamp: new Date(),
      sessionId: session.id,
      data: { session }
    });

    return session;
  }

  /**
   * Resume TLS session from ticket
   */
  async resumeSession(ticket: Buffer | Uint8Array): Promise<TLSSession | null> {
    // In real implementation, validate and decrypt ticket
    // For now, search for matching session
    for (const [id, session] of this.sessions) {
      if (session.ticket && this.compareBuffers(session.ticket, ticket)) {
        if (session.expiresAt > new Date()) {
          session.lastUsedAt = new Date();
          return session;
        } else {
          this.sessions.delete(id);
          this.emitEvent({
            type: TLSEventType.SESSION_EXPIRED,
            timestamp: new Date(),
            sessionId: id
          });
        }
      }
    }
    return null;
  }

  /**
   * Validate certificate chain
   */
  async validateCertificate(
    certificate: Certificate,
    options?: CertificateValidationOptions
  ): Promise<boolean> {
    try {
      // Custom validator takes precedence
      if (options?.customValidator) {
        return await options.customValidator(certificate);
      }

      // Allow self-signed if configured
      if (options?.allowSelfSigned && certificate.subject === certificate.issuer) {
        return true;
      }

      // In real implementation:
      // 1. Verify certificate signature
      // 2. Check certificate chain against trust anchors
      // 3. Verify certificate is not expired
      // 4. Check revocation status if enabled
      // 5. Verify hostname matches

      const now = new Date();
      if (certificate.notBefore && certificate.notAfter) {
        if (now < certificate.notBefore || now > certificate.notAfter) {
          return false;
        }
      }

      return true;

    } catch (error) {
      this.emitEvent({
        type: TLSEventType.ERROR,
        timestamp: new Date(),
        error: error as Error,
        data: { context: 'certificate_validation' }
      });
      return false;
    }
  }

  /**
   * Negotiate cipher suite
   * Selects best cipher suite from client preferences and server support
   */
  negotiateCipherSuite(
    clientSuites: CipherSuite[],
    serverSuites: CipherSuite[]
  ): CipherSuite | null {
    // Prefer client order (client preference)
    for (const clientSuite of clientSuites) {
      if (serverSuites.includes(clientSuite)) {
        return clientSuite;
      }
    }
    return null;
  }

  /**
   * Establish secure channel
   */
  async establishSecureChannel(config: SecureChannelConfig): Promise<void> {
    // In real implementation, set up encryption/decryption with negotiated parameters
    this.currentSession = config.session;

    // Set up heartbeat if configured
    if (config.heartbeatInterval) {
      // Implementation would start heartbeat timer
    }
  }

  /**
   * Send encrypted data over secure channel
   */
  async send(data: Buffer | Uint8Array): Promise<void> {
    if (!this.currentSession) {
      throw new Error('No active secure channel');
    }

    // In real implementation: encrypt and send data
    this.stats.bytesSent += data.length;

    this.emitEvent({
      type: TLSEventType.DATA_SENT,
      timestamp: new Date(),
      sessionId: this.currentSession.id,
      data: { size: data.length }
    });
  }

  /**
   * Receive and decrypt data from secure channel
   */
  async receive(): Promise<Buffer | Uint8Array> {
    if (!this.currentSession) {
      throw new Error('No active secure channel');
    }

    // In real implementation: receive and decrypt data
    const data = Buffer.from([]); // Placeholder
    this.stats.bytesReceived += data.length;

    this.emitEvent({
      type: TLSEventType.DATA_RECEIVED,
      timestamp: new Date(),
      sessionId: this.currentSession.id,
      data: { size: data.length }
    });

    return data;
  }

  /**
   * Close secure connection
   */
  async close(): Promise<void> {
    if (this.currentSession) {
      this.emitEvent({
        type: TLSEventType.CONNECTION_CLOSED,
        timestamp: new Date(),
        sessionId: this.currentSession.id
      });

      this.currentSession = undefined;
    }
  }

  /**
   * Get session by ID
   */
  getSession(sessionId: string): TLSSession | undefined {
    return this.sessions.get(sessionId);
  }

  /**
   * Get all active sessions
   */
  getActiveSessions(): TLSSession[] {
    const now = new Date();
    const activeSessions: TLSSession[] = [];

    for (const [id, session] of this.sessions) {
      if (session.expiresAt > now) {
        activeSessions.push(session);
      } else {
        this.sessions.delete(id);
      }
    }

    this.stats.activeSessions = activeSessions.length;
    return activeSessions;
  }

  /**
   * Clear expired sessions
   */
  clearExpiredSessions(): number {
    const now = new Date();
    let cleared = 0;

    for (const [id, session] of this.sessions) {
      if (session.expiresAt <= now) {
        this.sessions.delete(id);
        cleared++;

        this.emitEvent({
          type: TLSEventType.SESSION_EXPIRED,
          timestamp: new Date(),
          sessionId: id
        });
      }
    }

    this.stats.activeSessions = this.sessions.size;
    return cleared;
  }

  /**
   * Get TLS statistics
   */
  getStats(): TLSStats {
    return { ...this.stats };
  }

  /**
   * Register event handler
   */
  on(eventType: TLSEventType, handler: (event: TLSEvent) => void): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
    this.eventHandlers.get(eventType)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  off(eventType: TLSEventType, handler: (event: TLSEvent) => void): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      handlers.delete(handler);
    }
  }

  /**
   * Emit event to registered handlers
   */
  private emitEvent(event: TLSEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach(handler => {
        try {
          handler(event);
        } catch (error) {
          console.error('Event handler error:', error);
        }
      });
    }
  }

  /**
   * Generate session ID
   */
  private generateSessionId(): string {
    return `tls-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Generate session ticket for resumption
   */
  private generateSessionTicket(): Buffer {
    // In real implementation: encrypt session data with server key
    return Buffer.from(Math.random().toString(36));
  }

  /**
   * Compare two buffers
   */
  private compareBuffers(a: Buffer | Uint8Array, b: Buffer | Uint8Array): boolean {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) {
      if (a[i] !== b[i]) return false;
    }
    return true;
  }

  /**
   * Update handshake statistics
   */
  private updateHandshakeStats(handshakeTime: number, success: boolean): void {
    if (success) {
      this.stats.successfulHandshakes++;
      const totalTime = this.stats.avgHandshakeTime * (this.stats.successfulHandshakes - 1) + handshakeTime;
      this.stats.avgHandshakeTime = totalTime / this.stats.successfulHandshakes;
    }
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }
}

/**
 * Factory function to create WIATLSLite instance
 */
export function createTLSLite(): WIATLSLite {
  return new WIATLSLite();
}

/**
 * Default export
 */
export default WIATLSLite;
