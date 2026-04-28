/**
 * WIA-COMM-019: Real-Time Communication SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Real-Time Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  PeerConnectionConfig,
  MediaConstraints,
  ICECandidate,
  SessionDescription,
  RTCConnectionState,
  RTCIceConnectionState,
  RTCIceGatheringState,
  RTCSignalingState,
  CodecPreference,
  SIPClientConfig,
  SIPSession,
  ConferenceConfig,
  ConferenceParticipant,
  ConferenceLayout,
  PTTConfig,
  PTTFloorState,
  PTTPriority,
  QoEMetrics,
  RTCStats,
  InboundRTPStats,
  OutboundRTPStats,
  CandidatePairStats,
  JitterBufferConfig,
  JitterBufferStats,
  NATTraversalResult,
  NATType,
  StreamingQualityProfile,
  LowLatencyStreamingConfig,
  RTCEvents,
  ConferenceEvents,
  PTTEvents,
  RTCError,
  RTCErrorCode,
  RTC_CONSTANTS,
  VideoCodec,
  AudioCodec,
} from './types';

// ============================================================================
// Event Emitter Base
// ============================================================================

class EventEmitter<T extends Record<string, (...args: any[]) => void>> {
  private listeners: Map<keyof T, Set<Function>> = new Map();

  on<K extends keyof T>(event: K, listener: T[K]): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(listener);
  }

  off<K extends keyof T>(event: K, listener: T[K]): void {
    const eventListeners = this.listeners.get(event);
    if (eventListeners) {
      eventListeners.delete(listener);
    }
  }

  emit<K extends keyof T>(event: K, ...args: Parameters<T[K]>): void {
    const eventListeners = this.listeners.get(event);
    if (eventListeners) {
      eventListeners.forEach((listener) => listener(...args));
    }
  }

  removeAllListeners(): void {
    this.listeners.clear();
  }
}

// ============================================================================
// WebRTC Peer Connection Wrapper
// ============================================================================

export class WebRTCPeer extends EventEmitter<RTCEvents> {
  private pc: RTCPeerConnection | null = null;
  private config: PeerConnectionConfig;
  private localStream: MediaStream | null = null;
  private statsInterval: NodeJS.Timeout | null = null;

  constructor(config: PeerConnectionConfig) {
    super();
    this.config = config;
  }

  /**
   * Initialize peer connection
   */
  async initialize(): Promise<void> {
    const rtcConfig: RTCConfiguration = {
      iceServers: this.config.iceServers || [
        { urls: 'stun:stun.l.google.com:19302' },
      ],
      iceTransportPolicy: this.config.iceTransportPolicy,
      bundlePolicy: this.config.bundlePolicy,
      rtcpMuxPolicy: this.config.rtcpMuxPolicy,
      iceCandidatePoolSize: this.config.iceCandidatePoolSize,
    };

    this.pc = new RTCPeerConnection(rtcConfig);

    // Set up event handlers
    this.pc.onconnectionstatechange = () => {
      this.emit('connection-state-changed', this.pc!.connectionState as RTCConnectionState);
    };

    this.pc.onicecandidate = (event) => {
      if (event.candidate) {
        this.emit('ice-candidate', event.candidate as ICECandidate);
      }
    };

    this.pc.onicegatheringstatechange = () => {
      this.emit(
        'ice-gathering-state-changed',
        this.pc!.iceGatheringState as RTCIceGatheringState
      );
    };

    this.pc.oniceconnectionstatechange = () => {
      this.emit(
        'ice-connection-state-changed',
        this.pc!.iceConnectionState as RTCIceConnectionState
      );

      // Auto-restart ICE on failure
      if (this.pc!.iceConnectionState === 'failed') {
        this.pc!.restartIce();
      }
    };

    this.pc.onsignalingstatechange = () => {
      this.emit('signaling-state-changed', this.pc!.signalingState as RTCSignalingState);
    };

    this.pc.ontrack = (event) => {
      this.emit('track-added', event.track);
      if (event.streams[0]) {
        this.emit('stream-added', event.streams[0]);
      }
    };

    this.pc.ondatachannel = (event) => {
      this.emit('data-channel', event.channel);
    };

    // Start quality monitoring
    this.startStatsMonitoring();
  }

  /**
   * Add local media stream
   */
  addStream(stream: MediaStream): void {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    stream.getTracks().forEach((track) => {
      this.pc!.addTrack(track, stream);
    });

    this.localStream = stream;
  }

  /**
   * Create offer
   */
  async createOffer(): Promise<SessionDescription> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    const offer = await this.pc.createOffer();
    return offer as SessionDescription;
  }

  /**
   * Create answer
   */
  async createAnswer(): Promise<SessionDescription> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    const answer = await this.pc.createAnswer();
    return answer as SessionDescription;
  }

  /**
   * Set local description
   */
  async setLocalDescription(desc: SessionDescription): Promise<void> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    await this.pc.setLocalDescription(desc);
  }

  /**
   * Set remote description
   */
  async setRemoteDescription(desc: SessionDescription): Promise<void> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    await this.pc.setRemoteDescription(desc);
  }

  /**
   * Add ICE candidate
   */
  async addIceCandidate(candidate: ICECandidate): Promise<void> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    await this.pc.addIceCandidate(candidate);
  }

  /**
   * Set codec preferences
   */
  setCodecPreferences(preferences: CodecPreference): void {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    const transceivers = this.pc.getTransceivers();

    transceivers.forEach((transceiver) => {
      if (transceiver.sender.track) {
        const kind = transceiver.sender.track.kind;

        if (kind === 'video' && preferences.video) {
          const codecs = RTCRtpSender.getCapabilities('video')?.codecs || [];
          const preferredCodecs = preferences.video
            .map((codec) => codecs.find((c) => c.mimeType.includes(codec)))
            .filter(Boolean);
          if (preferredCodecs.length > 0) {
            transceiver.setCodecPreferences(preferredCodecs as RTCRtpCodecCapability[]);
          }
        }

        if (kind === 'audio' && preferences.audio) {
          const codecs = RTCRtpSender.getCapabilities('audio')?.codecs || [];
          const preferredCodecs = preferences.audio
            .map((codec) => codecs.find((c) => c.mimeType.includes(codec)))
            .filter(Boolean);
          if (preferredCodecs.length > 0) {
            transceiver.setCodecPreferences(preferredCodecs as RTCRtpCodecCapability[]);
          }
        }
      }
    });
  }

  /**
   * Get connection statistics
   */
  async getStats(): Promise<RTCStatsReport> {
    if (!this.pc) {
      throw new RTCError(RTCErrorCode.INVALID_PARAMETERS, 'Peer connection not initialized');
    }

    return await this.pc.getStats();
  }

  /**
   * Get quality metrics
   */
  async getQualityMetrics(): Promise<QoEMetrics> {
    const stats = await this.getStats();
    const metrics: QoEMetrics = {};

    stats.forEach((report: any) => {
      if (report.type === 'inbound-rtp' && report.kind === 'audio') {
        metrics.jitter = report.jitter * 1000; // Convert to ms
        metrics.packetLoss =
          (report.packetsLost / (report.packetsReceived + report.packetsLost)) * 100;
      }

      if (report.type === 'candidate-pair' && report.state === 'succeeded') {
        metrics.roundTripTime = report.currentRoundTripTime * 1000; // Convert to ms
        metrics.latency = metrics.roundTripTime / 2;
      }

      if (report.type === 'inbound-rtp' && report.kind === 'video') {
        metrics.frameRate = report.framesPerSecond;
      }
    });

    // Calculate MOS based on R-factor (E-model simplified)
    if (metrics.latency && metrics.packetLoss !== undefined) {
      const R =
        93.2 -
        (metrics.latency / 40) -
        (metrics.packetLoss * 2.5);
      metrics.rFactor = Math.max(0, Math.min(100, R));
      metrics.mos = 1 + 0.035 * R + (7e-6 * R * (R - 60) * (100 - R));
      metrics.mos = Math.max(1, Math.min(5, metrics.mos));
    }

    return metrics;
  }

  /**
   * Start monitoring connection statistics
   */
  private startStatsMonitoring(interval: number = 1000): void {
    this.statsInterval = setInterval(async () => {
      try {
        const metrics = await this.getQualityMetrics();
        this.emit('stats', metrics);
      } catch (error) {
        // Ignore errors during stats collection
      }
    }, interval);
  }

  /**
   * Stop monitoring
   */
  private stopStatsMonitoring(): void {
    if (this.statsInterval) {
      clearInterval(this.statsInterval);
      this.statsInterval = null;
    }
  }

  /**
   * Get connection state
   */
  get connectionState(): RTCConnectionState {
    return (this.pc?.connectionState || 'closed') as RTCConnectionState;
  }

  /**
   * Get ICE connection state
   */
  get iceConnectionState(): RTCIceConnectionState {
    return (this.pc?.iceConnectionState || 'closed') as RTCIceConnectionState;
  }

  /**
   * Close connection
   */
  close(): void {
    this.stopStatsMonitoring();

    if (this.localStream) {
      this.localStream.getTracks().forEach((track) => track.stop());
      this.localStream = null;
    }

    if (this.pc) {
      this.pc.close();
      this.pc = null;
    }

    this.removeAllListeners();
  }
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Real-Time Communication SDK
 *
 * Provides comprehensive interface for WebRTC, SIP, conferencing, and PTT.
 */
export class RealTimeCommunicationSDK {
  private codecPreferences: CodecPreference;

  constructor(config?: {
    iceServers?: Array<{ urls: string | string[]; username?: string; credential?: string }>;
    codecPreferences?: CodecPreference;
  }) {
    this.codecPreferences = config?.codecPreferences || {
      video: ['VP9', 'VP8', 'H264'],
      audio: ['opus', 'PCMU', 'PCMA'],
    };
  }

  // ==========================================================================
  // WebRTC Peer Connection
  // ==========================================================================

  /**
   * Create WebRTC peer connection
   */
  async createPeerConnection(config: PeerConnectionConfig): Promise<WebRTCPeer> {
    const peer = new WebRTCPeer(config);
    await peer.initialize();
    peer.setCodecPreferences(this.codecPreferences);
    return peer;
  }

  /**
   * Get user media (camera/microphone)
   */
  async getUserMedia(constraints: MediaConstraints): Promise<MediaStream> {
    try {
      return await navigator.mediaDevices.getUserMedia(constraints);
    } catch (error) {
      throw new RTCError(
        RTCErrorCode.MEDIA_ACCESS_DENIED,
        'Failed to access media devices',
        error
      );
    }
  }

  /**
   * Get display media (screen sharing)
   */
  async getDisplayMedia(constraints?: MediaConstraints): Promise<MediaStream> {
    try {
      return await navigator.mediaDevices.getDisplayMedia(constraints);
    } catch (error) {
      throw new RTCError(
        RTCErrorCode.MEDIA_ACCESS_DENIED,
        'Failed to access display',
        error
      );
    }
  }

  // ==========================================================================
  // SIP Client (Stub - requires full SIP.js integration)
  // ==========================================================================

  /**
   * Create SIP client
   */
  createSIPClient(config: SIPClientConfig): SIPClientStub {
    return new SIPClientStub(config);
  }

  // ==========================================================================
  // Video Conferencing (Stub)
  // ==========================================================================

  /**
   * Join conference
   */
  async joinConference(config: ConferenceConfig): Promise<ConferenceStub> {
    return new ConferenceStub(config);
  }

  // ==========================================================================
  // Push-to-Talk (Stub)
  // ==========================================================================

  /**
   * Create push-to-talk client
   */
  createPushToTalk(config: PTTConfig): PushToTalkStub {
    return new PushToTalkStub(config);
  }

  // ==========================================================================
  // NAT Traversal Testing
  // ==========================================================================

  /**
   * Test NAT traversal
   */
  async testNATTraversal(stunServer: string = 'stun:stun.l.google.com:19302'): Promise<NATTraversalResult> {
    // Simplified NAT type detection using STUN
    const pc = new RTCPeerConnection({
      iceServers: [{ urls: stunServer }],
    });

    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        pc.close();
        reject(new RTCError(RTCErrorCode.TIMEOUT, 'NAT traversal test timed out'));
      }, 5000);

      pc.onicecandidate = (event) => {
        if (event.candidate && event.candidate.type === 'srflx') {
          clearTimeout(timeout);

          const parts = event.candidate.candidate.split(' ');
          const ip = parts[4];
          const port = parseInt(parts[5], 10);

          // Simplified NAT type detection
          const natType: NATType = 'port-restricted-cone'; // Placeholder

          pc.close();
          resolve({
            natType,
            publicIP: ip,
            publicPort: port,
            canP2P: true,
            needsTURN: false,
          });
        }
      };

      pc.createDataChannel('test');
      pc.createOffer().then((offer) => pc.setLocalDescription(offer));
    });
  }
}

// ============================================================================
// Stub Classes (Placeholder implementations)
// ============================================================================

class SIPClientStub extends EventEmitter<any> {
  constructor(private config: SIPClientConfig) {
    super();
  }

  async register(): Promise<void> {
    console.log('SIP registration (stub)');
  }

  async invite(target: string, options?: any): Promise<SIPSession> {
    console.log('SIP invite (stub)', target);
    return {} as SIPSession;
  }
}

class ConferenceStub extends EventEmitter<ConferenceEvents> {
  constructor(private config: ConferenceConfig) {
    super();
  }

  async leave(): Promise<void> {
    console.log('Leave conference (stub)');
  }

  async muteAudio(): Promise<void> {
    console.log('Mute audio (stub)');
  }

  async unmuteAudio(): Promise<void> {
    console.log('Unmute audio (stub)');
  }

  async enableVideo(): Promise<void> {
    console.log('Enable video (stub)');
  }

  async disableVideo(): Promise<void> {
    console.log('Disable video (stub)');
  }
}

class PushToTalkStub extends EventEmitter<PTTEvents> {
  private floorState: PTTFloorState = 'idle';

  constructor(private config: PTTConfig) {
    super();
  }

  requestFloor(): void {
    console.log('Request PTT floor (stub)');
    this.floorState = 'granted';
    this.emit('floor-granted');
  }

  releaseFloor(): void {
    console.log('Release PTT floor (stub)');
    this.floorState = 'idle';
    this.emit('floor-released');
  }

  get state(): PTTFloorState {
    return this.floorState;
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

// Re-export main classes
export { RealTimeCommunicationSDK, WebRTCPeer };

// Default export
export default RealTimeCommunicationSDK;
