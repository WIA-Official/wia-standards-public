/**
 * WIA-COMM-019: Real-Time Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Real-Time Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core RTC Types
// ============================================================================

/**
 * Media stream constraints
 */
export interface MediaConstraints {
  audio?: boolean | AudioConstraints;
  video?: boolean | VideoConstraints;
}

/**
 * Audio constraints
 */
export interface AudioConstraints {
  echoCancellation?: boolean;
  noiseSuppression?: boolean;
  autoGainControl?: boolean;
  sampleRate?: number;
  sampleSize?: number;
  channelCount?: number;
  latency?: number;
  deviceId?: string;
}

/**
 * Video constraints
 */
export interface VideoConstraints {
  width?: number | { min?: number; ideal?: number; max?: number };
  height?: number | { min?: number; ideal?: number; max?: number };
  frameRate?: number | { min?: number; ideal?: number; max?: number };
  facingMode?: 'user' | 'environment' | 'left' | 'right';
  aspectRatio?: number;
  resizeMode?: 'none' | 'crop-and-scale';
  deviceId?: string;
}

/**
 * ICE server configuration
 */
export interface ICEServerConfig {
  urls: string | string[];
  username?: string;
  credential?: string;
  credentialType?: 'password' | 'oauth';
}

/**
 * Peer connection configuration
 */
export interface PeerConnectionConfig {
  userId: string;
  iceServers?: ICEServerConfig[];
  iceTransportPolicy?: 'all' | 'relay';
  bundlePolicy?: 'balanced' | 'max-compat' | 'max-bundle';
  rtcpMuxPolicy?: 'negotiate' | 'require';
  iceCandidatePoolSize?: number;
  mediaConfig?: MediaConstraints;
}

// ============================================================================
// WebRTC Types
// ============================================================================

/**
 * Connection state
 */
export type RTCConnectionState =
  | 'new'
  | 'connecting'
  | 'connected'
  | 'disconnected'
  | 'failed'
  | 'closed';

/**
 * ICE connection state
 */
export type RTCIceConnectionState =
  | 'new'
  | 'checking'
  | 'connected'
  | 'completed'
  | 'failed'
  | 'disconnected'
  | 'closed';

/**
 * ICE gathering state
 */
export type RTCIceGatheringState = 'new' | 'gathering' | 'complete';

/**
 * Signaling state
 */
export type RTCSignalingState =
  | 'stable'
  | 'have-local-offer'
  | 'have-remote-offer'
  | 'have-local-pranswer'
  | 'have-remote-pranswer'
  | 'closed';

/**
 * ICE candidate
 */
export interface ICECandidate {
  candidate: string;
  sdpMid: string | null;
  sdpMLineIndex: number | null;
  usernameFragment?: string;
}

/**
 * ICE candidate type
 */
export type ICECandidateType = 'host' | 'srflx' | 'prflx' | 'relay';

/**
 * Parsed ICE candidate
 */
export interface ParsedICECandidate {
  foundation: string;
  component: number;
  protocol: 'udp' | 'tcp';
  priority: number;
  ip: string;
  port: number;
  type: ICECandidateType;
  relatedAddress?: string;
  relatedPort?: number;
  tcpType?: 'active' | 'passive' | 'so';
}

/**
 * Session description
 */
export interface SessionDescription {
  type: 'offer' | 'answer' | 'pranswer' | 'rollback';
  sdp: string;
}

// ============================================================================
// SIP Types
// ============================================================================

/**
 * SIP URI
 */
export interface SIPURI {
  scheme: 'sip' | 'sips';
  user?: string;
  host: string;
  port?: number;
  parameters?: Record<string, string>;
}

/**
 * SIP method
 */
export type SIPMethod =
  | 'INVITE'
  | 'ACK'
  | 'BYE'
  | 'CANCEL'
  | 'REGISTER'
  | 'OPTIONS'
  | 'INFO'
  | 'PRACK'
  | 'UPDATE'
  | 'REFER'
  | 'SUBSCRIBE'
  | 'NOTIFY';

/**
 * SIP response code
 */
export type SIPResponseCode =
  | 100 // Trying
  | 180 // Ringing
  | 181 // Call Is Being Forwarded
  | 183 // Session Progress
  | 200 // OK
  | 301 // Moved Permanently
  | 302 // Moved Temporarily
  | 400 // Bad Request
  | 401 // Unauthorized
  | 403 // Forbidden
  | 404 // Not Found
  | 486 // Busy Here
  | 487 // Request Terminated
  | 500 // Server Internal Error
  | 503 // Service Unavailable
  | 600 // Busy Everywhere
  | 603; // Decline

/**
 * SIP client configuration
 */
export interface SIPClientConfig {
  uri: string;
  wsServers: string[];
  authorizationUser?: string;
  password?: string;
  displayName?: string;
  registerExpires?: number;
  userAgentString?: string;
  sessionDescriptionHandlerFactoryOptions?: {
    constraints?: MediaConstraints;
    peerConnectionOptions?: RTCConfiguration;
  };
}

/**
 * SIP session state
 */
export type SIPSessionState =
  | 'Initial'
  | 'Establishing'
  | 'Established'
  | 'Terminating'
  | 'Terminated';

/**
 * SIP session
 */
export interface SIPSession {
  id: string;
  state: SIPSessionState;
  localURI: SIPURI;
  remoteURI: SIPURI;
  startTime?: Date;
  accept(): Promise<void>;
  reject(options?: { statusCode?: SIPResponseCode }): Promise<void>;
  terminate(): Promise<void>;
  hold(): Promise<void>;
  unhold(): Promise<void>;
  sendDTMF(tone: string): void;
}

// ============================================================================
// RTP/RTCP Types
// ============================================================================

/**
 * RTP header
 */
export interface RTPHeader {
  version: number;
  padding: boolean;
  extension: boolean;
  csrcCount: number;
  marker: boolean;
  payloadType: number;
  sequenceNumber: number;
  timestamp: number;
  ssrc: number;
  csrc: number[];
}

/**
 * RTCP packet type
 */
export type RTCPPacketType = 'SR' | 'RR' | 'SDES' | 'BYE' | 'APP' | 'RTPFB' | 'PSFB';

/**
 * RTCP sender report
 */
export interface RTCPSenderReport {
  ssrc: number;
  ntpTimestamp: bigint;
  rtpTimestamp: number;
  packetCount: number;
  octetCount: number;
  reportBlocks: RTCPReportBlock[];
}

/**
 * RTCP receiver report
 */
export interface RTCPReceiverReport {
  ssrc: number;
  reportBlocks: RTCPReportBlock[];
}

/**
 * RTCP report block
 */
export interface RTCPReportBlock {
  ssrc: number;
  fractionLost: number;
  cumulativeLost: number;
  highestSequence: number;
  jitter: number;
  lastSR: number;
  delaySinceSR: number;
}

// ============================================================================
// Codec Types
// ============================================================================

/**
 * Video codec
 */
export type VideoCodec = 'VP8' | 'VP9' | 'H264' | 'H265' | 'AV1';

/**
 * Audio codec
 */
export type AudioCodec =
  | 'opus'
  | 'PCMU'
  | 'PCMA'
  | 'G722'
  | 'G729'
  | 'iLBC'
  | 'AAC-LD'
  | 'ISAC';

/**
 * Codec preference
 */
export interface CodecPreference {
  video?: VideoCodec[];
  audio?: AudioCodec[];
}

/**
 * Codec parameters
 */
export interface CodecParameters {
  mimeType: string;
  clockRate: number;
  channels?: number;
  sdpFmtpLine?: string;
}

/**
 * H.264 profile
 */
export type H264Profile = 'baseline' | 'main' | 'high' | 'constrained-baseline';

/**
 * H.264 level
 */
export type H264Level =
  | '1.0'
  | '1.1'
  | '1.2'
  | '1.3'
  | '2.0'
  | '2.1'
  | '2.2'
  | '3.0'
  | '3.1'
  | '3.2'
  | '4.0'
  | '4.1'
  | '4.2'
  | '5.0'
  | '5.1'
  | '5.2';

// ============================================================================
// Quality Metrics
// ============================================================================

/**
 * Quality of Experience metrics
 */
export interface QoEMetrics {
  mos?: number; // Mean Opinion Score (1-5)
  rFactor?: number; // R-factor (0-100)
  latency?: number; // ms
  jitter?: number; // ms
  packetLoss?: number; // percentage (0-100)
  roundTripTime?: number; // ms
  bitrate?: number; // bps
  frameRate?: number; // fps
  resolution?: { width: number; height: number };
}

/**
 * WebRTC statistics
 */
export interface RTCStats {
  timestamp: number;
  type: string;
  id: string;
  [key: string]: any;
}

/**
 * Inbound RTP statistics
 */
export interface InboundRTPStats extends RTCStats {
  type: 'inbound-rtp';
  ssrc: number;
  kind: 'audio' | 'video';
  packetsReceived: number;
  packetsLost: number;
  jitter: number;
  bytesReceived: number;
  framesDecoded?: number;
  framesDropped?: number;
}

/**
 * Outbound RTP statistics
 */
export interface OutboundRTPStats extends RTCStats {
  type: 'outbound-rtp';
  ssrc: number;
  kind: 'audio' | 'video';
  packetsSent: number;
  bytesSent: number;
  framesEncoded?: number;
  retransmittedPacketsSent?: number;
}

/**
 * Candidate pair statistics
 */
export interface CandidatePairStats extends RTCStats {
  type: 'candidate-pair';
  state: 'succeeded' | 'failed' | 'in-progress';
  priority: number;
  nominated: boolean;
  writable: boolean;
  readable: boolean;
  bytesSent: number;
  bytesReceived: number;
  currentRoundTripTime?: number;
  availableOutgoingBitrate?: number;
}

// ============================================================================
// Jitter Buffer
// ============================================================================

/**
 * Jitter buffer strategy
 */
export type JitterBufferStrategy = 'fixed' | 'adaptive';

/**
 * Jitter buffer configuration
 */
export interface JitterBufferConfig {
  strategy: JitterBufferStrategy;
  minDelay: number; // ms
  maxDelay: number; // ms
  targetDelay?: number; // ms (for adaptive)
  packetLossConcealment: boolean;
  voiceActivityDetection: boolean;
}

/**
 * Jitter buffer statistics
 */
export interface JitterBufferStats {
  currentDelay: number; // ms
  bufferOccupancy: number; // percentage
  latePackets: number;
  discardedPackets: number;
  concealedFrames: number;
}

// ============================================================================
// NAT Traversal
// ============================================================================

/**
 * STUN server
 */
export interface STUNServer {
  host: string;
  port: number;
}

/**
 * TURN server
 */
export interface TURNServer {
  host: string;
  port: number;
  username: string;
  credential: string;
  protocol: 'udp' | 'tcp' | 'tls';
}

/**
 * NAT type
 */
export type NATType =
  | 'open-internet'
  | 'full-cone'
  | 'restricted-cone'
  | 'port-restricted-cone'
  | 'symmetric'
  | 'unknown';

/**
 * NAT traversal result
 */
export interface NATTraversalResult {
  natType: NATType;
  publicIP: string;
  publicPort: number;
  canP2P: boolean;
  needsTURN: boolean;
}

// ============================================================================
// Video Conferencing
// ============================================================================

/**
 * Conference architecture
 */
export type ConferenceArchitecture = 'mcu' | 'sfu' | 'mesh';

/**
 * Conference configuration
 */
export interface ConferenceConfig {
  conferenceId: string;
  displayName: string;
  audioEnabled: boolean;
  videoEnabled: boolean;
  sfuUrl?: string;
  architecture?: ConferenceArchitecture;
  maxParticipants?: number;
}

/**
 * Conference participant
 */
export interface ConferenceParticipant {
  id: string;
  displayName: string;
  isLocal: boolean;
  audioEnabled: boolean;
  videoEnabled: boolean;
  screenShareEnabled: boolean;
  isSpeaking: boolean;
  audioLevel: number;
  joinTime: Date;
}

/**
 * Conference layout
 */
export type ConferenceLayout =
  | 'speaker-focused'
  | 'grid'
  | 'picture-in-picture'
  | 'gallery'
  | 'presentation';

/**
 * Simulcast encoding
 */
export interface SimulcastEncoding {
  rid: string;
  active: boolean;
  maxBitrate?: number;
  scaleResolutionDownBy?: number;
  maxFramerate?: number;
}

// ============================================================================
// Push-to-Talk
// ============================================================================

/**
 * PTT priority
 */
export type PTTPriority = 'emergency' | 'high' | 'normal' | 'low';

/**
 * PTT floor state
 */
export type PTTFloorState = 'idle' | 'granted' | 'queued' | 'denied';

/**
 * PTT configuration
 */
export interface PTTConfig {
  channel: string;
  codec: AudioCodec;
  priority?: PTTPriority;
  maxFloorDuration?: number; // seconds
  queueTimeout?: number; // seconds
}

/**
 * PTT floor request
 */
export interface PTTFloorRequest {
  userId: string;
  channel: string;
  priority: PTTPriority;
  timestamp: Date;
}

/**
 * PTT floor grant
 */
export interface PTTFloorGrant {
  userId: string;
  channel: string;
  grantTime: Date;
  maxDuration?: number;
}

// ============================================================================
// Signaling
// ============================================================================

/**
 * Signaling message type
 */
export type SignalingMessageType =
  | 'offer'
  | 'answer'
  | 'ice-candidate'
  | 'ice-candidate-complete'
  | 'renegotiate'
  | 'bye';

/**
 * Signaling message
 */
export interface SignalingMessage {
  type: SignalingMessageType;
  from: string;
  to: string;
  data: any;
  timestamp: Date;
}

/**
 * Signaling transport
 */
export type SignalingTransport = 'websocket' | 'sip' | 'xmpp' | 'http-polling';

// ============================================================================
// Security
// ============================================================================

/**
 * SRTP cipher suite
 */
export type SRTPCipherSuite =
  | 'AES_CM_128_HMAC_SHA1_80'
  | 'AES_CM_128_HMAC_SHA1_32'
  | 'AEAD_AES_128_GCM'
  | 'AEAD_AES_256_GCM';

/**
 * DTLS fingerprint
 */
export interface DTLSFingerprint {
  algorithm: 'sha-256' | 'sha-384' | 'sha-512';
  value: string;
}

/**
 * Security parameters
 */
export interface SecurityParameters {
  srtpCipherSuite: SRTPCipherSuite;
  dtlsFingerprint: DTLSFingerprint;
  icePassword: string;
  iceUsernameFragment: string;
}

// ============================================================================
// Streaming
// ============================================================================

/**
 * Streaming mode
 */
export type StreamingMode = 'live' | 'vod' | 'time-shift';

/**
 * Streaming quality profile
 */
export interface StreamingQualityProfile {
  name: string;
  resolution: { width: number; height: number };
  frameRate: number;
  videoBitrate: number; // bps
  audioBitrate: number; // bps
  codec: { video: VideoCodec; audio: AudioCodec };
}

/**
 * Low-latency streaming configuration
 */
export interface LowLatencyStreamingConfig {
  targetLatency: number; // ms
  gopSize: number; // frames
  encodingPreset: 'ultrafast' | 'superfast' | 'veryfast' | 'fast';
  adaptiveBitrate: boolean;
  qualityProfiles?: StreamingQualityProfile[];
}

// ============================================================================
// Events
// ============================================================================

/**
 * RTC event types
 */
export interface RTCEvents {
  'connection-state-changed': (state: RTCConnectionState) => void;
  'ice-candidate': (candidate: ICECandidate) => void;
  'ice-gathering-state-changed': (state: RTCIceGatheringState) => void;
  'ice-connection-state-changed': (state: RTCIceConnectionState) => void;
  'signaling-state-changed': (state: RTCSignalingState) => void;
  'stream-added': (stream: MediaStream) => void;
  'stream-removed': (stream: MediaStream) => void;
  'track-added': (track: MediaStreamTrack) => void;
  'data-channel': (channel: RTCDataChannel) => void;
  stats: (stats: QoEMetrics) => void;
  error: (error: Error) => void;
}

/**
 * Conference event types
 */
export interface ConferenceEvents {
  'participant-joined': (participant: ConferenceParticipant) => void;
  'participant-left': (participant: ConferenceParticipant) => void;
  'stream-added': (stream: MediaStream, participant: ConferenceParticipant) => void;
  'stream-removed': (stream: MediaStream, participant: ConferenceParticipant) => void;
  'audio-level-changed': (participant: ConferenceParticipant, level: number) => void;
  'layout-changed': (layout: ConferenceLayout) => void;
  'recording-started': () => void;
  'recording-stopped': () => void;
}

/**
 * PTT event types
 */
export interface PTTEvents {
  'floor-granted': () => void;
  'floor-denied': (reason: string) => void;
  'floor-released': () => void;
  'floor-taken': (userId: string) => void;
  'participant-joined': (userId: string) => void;
  'participant-left': (userId: string) => void;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * RTC error code
 */
export enum RTCErrorCode {
  INVALID_PARAMETERS = 'INVALID_PARAMETERS',
  CONNECTION_FAILED = 'CONNECTION_FAILED',
  ICE_FAILED = 'ICE_FAILED',
  MEDIA_ACCESS_DENIED = 'MEDIA_ACCESS_DENIED',
  CODEC_NOT_SUPPORTED = 'CODEC_NOT_SUPPORTED',
  SIGNALING_ERROR = 'SIGNALING_ERROR',
  DTLS_ERROR = 'DTLS_ERROR',
  SRTP_ERROR = 'SRTP_ERROR',
  NETWORK_ERROR = 'NETWORK_ERROR',
  TIMEOUT = 'TIMEOUT',
  UNKNOWN = 'UNKNOWN',
}

/**
 * RTC error
 */
export class RTCError extends Error {
  constructor(
    public code: RTCErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'RTCError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * RTC constants
 */
export const RTC_CONSTANTS = {
  // Latency targets (ms)
  LATENCY_TARGET_VOICE: 150,
  LATENCY_TARGET_VIDEO: 150,
  LATENCY_TARGET_GAMING: 50,
  LATENCY_TARGET_SCREEN_SHARE: 200,

  // Quality thresholds
  MOS_EXCELLENT: 4.3,
  MOS_GOOD: 4.0,
  MOS_FAIR: 3.6,
  MOS_POOR: 3.0,

  // Packet loss thresholds (%)
  PACKET_LOSS_EXCELLENT: 0.5,
  PACKET_LOSS_GOOD: 1.0,
  PACKET_LOSS_FAIR: 3.0,

  // Jitter thresholds (ms)
  JITTER_EXCELLENT: 10,
  JITTER_GOOD: 30,
  JITTER_FAIR: 50,

  // RTT thresholds (ms)
  RTT_EXCELLENT: 50,
  RTT_GOOD: 150,
  RTT_FAIR: 300,

  // Codec defaults
  DEFAULT_AUDIO_CODEC: 'opus' as AudioCodec,
  DEFAULT_VIDEO_CODEC: 'VP9' as VideoCodec,

  // Bitrate ranges (bps)
  AUDIO_BITRATE_MIN: 8000,
  AUDIO_BITRATE_MAX: 510000,
  VIDEO_BITRATE_MIN: 100000,
  VIDEO_BITRATE_MAX: 5000000,

  // Jitter buffer defaults (ms)
  JITTER_BUFFER_MIN: 20,
  JITTER_BUFFER_MAX: 200,
  JITTER_BUFFER_TARGET: 60,
} as const;
