# WIA-IND-014: Virtual Fitness Standard
## Phase 3: Protocol Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-27
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document defines communication protocols for XR (Extended Reality) fitness applications, including WebXR streaming, multiplayer synchronization, haptic feedback, and real-time voice communication. Protocols prioritize low latency, reliability, and cross-platform compatibility.

## 2. WebXR Streaming Protocol

### 2.1 Session Initialization

```javascript
// Client initiates XR session
const xrSession = await navigator.xr.requestSession('immersive-vr', {
  requiredFeatures: ['local-floor'],
  optionalFeatures: ['hand-tracking', 'body-tracking']
});

// Connect to WIA-IND-014 compliant streaming server
const streamConfig = {
  server: 'wss://xr-stream.wia-fitness.com',
  sessionId: 'uuid-v4',
  quality: 'adaptive', // ultra, high, medium, low, adaptive
  fps: 90,
  philosophy: '弘益人間'
};
```

### 2.2 Pose Data Streaming

Pose data streams via WebSocket with Protocol Buffers encoding for minimal latency:

```protobuf
message PoseFrame {
  uint64 timestamp = 1;
  uint32 frame_number = 2;
  repeated Keypoint keypoints = 3;

  message Keypoint {
    uint32 id = 1;
    float x = 2;
    float y = 3;
    float z = 4;
    float confidence = 5;
  }
}
```

**Latency Requirements:**
- **Excellent**: < 20ms end-to-end
- **Good**: 20-50ms
- **Acceptable**: 50-100ms
- **Poor**: > 100ms (triggers quality degradation)

### 2.3 Adaptive Quality Control

```javascript
const qualityController = {
  targetLatency: 50, // ms
  currentLatency: 0,
  qualityLevel: 'high',

  adjustQuality(measuredLatency) {
    if (measuredLatency > 100) {
      this.qualityLevel = 'medium';
      this.reduceFPS(60);
    } else if (measuredLatency < 30) {
      this.qualityLevel = 'high';
      this.increaseFPS(90);
    }
  },

  reduceFPS(target) {
    // Reduce frame rate to decrease bandwidth
  },

  increaseFPS(target) {
    // Increase frame rate for better experience
  }
};
```

## 3. Multiplayer Synchronization Protocol

### 3.1 State Synchronization

```javascript
// Server-authoritative state sync
const gameState = {
  sessionId: 'multiplayer-001',
  players: [
    {
      userId: 'user-001',
      position: {x: 0, y: 1.7, z: 0},
      rotation: {x: 0, y: 0, z: 0, w: 1},
      animation: 'boxing-jab',
      health: 100,
      score: 1250
    }
  ],
  timestamp: Date.now(),
  tickRate: 30 // updates per second
};

// Delta compression for bandwidth efficiency
const deltaUpdate = {
  userId: 'user-001',
  changedFields: {
    score: 1275, // only send changed values
    animation: 'boxing-hook'
  },
  timestamp: Date.now()
};
```

### 3.2 Client-Side Prediction

```javascript
class PredictiveMovement {
  constructor() {
    this.serverState = null;
    this.predictedState = null;
    this.inputBuffer = [];
  }

  // Predict movement based on local input
  predict(input) {
    this.inputBuffer.push(input);
    this.predictedState = this.simulate(this.predictedState, input);
    return this.predictedState;
  }

  // Reconcile with server state
  reconcile(serverState) {
    this.serverState = serverState;

    // Replay inputs from server timestamp
    const replayInputs = this.inputBuffer.filter(
      input => input.timestamp > serverState.timestamp
    );

    this.predictedState = serverState;
    for (const input of replayInputs) {
      this.predictedState = this.simulate(this.predictedState, input);
    }

    // Clean old inputs
    this.inputBuffer = this.inputBuffer.filter(
      input => input.timestamp > serverState.timestamp
    );
  }

  simulate(state, input) {
    // Physics simulation
    return newState;
  }
}
```

### 3.3 Lag Compensation

```javascript
// Server-side lag compensation for hit detection
function compensateForLag(shooter, target, latency) {
  // Rewind target to where they were when shooter fired
  const compensatedPosition = rewindHistory(
    target.positionHistory,
    latency
  );

  // Check hit against compensated position
  return checkHit(shooter.aim, compensatedPosition);
}
```

## 4. Haptic Feedback Protocol

### 4.1 Haptic Command Structure

```json
{
  "type": "haptic-feedback",
  "timestamp": 1706371200000,
  "device": "haptic-vest",
  "commands": [
    {
      "actuator": "chest-center",
      "intensity": 0.8,
      "duration": 150,
      "pattern": "impact"
    },
    {
      "actuator": "left-arm",
      "intensity": 0.6,
      "duration": 300,
      "pattern": "vibration"
    }
  ]
}
```

### 4.2 Haptic Patterns Library

```javascript
const HapticPatterns = {
  IMPACT: {
    profile: 'sharp',
    attack: 5,    // ms
    sustain: 50,  // ms
    release: 100  // ms
  },

  VIBRATION: {
    profile: 'continuous',
    frequency: 50, // Hz
    waveform: 'sine'
  },

  RESISTANCE: {
    profile: 'force-feedback',
    magnitude: 0.7, // 0-1 scale
    direction: {x: -1, y: 0, z: 0}
  }
};
```

### 4.3 Force Feedback for Resistance Training

```javascript
class ResistanceController {
  constructor(device) {
    this.device = device;
    this.virtualWeight = 10; // kg
  }

  calculateResistance(velocity, direction) {
    // Simulate weight based on movement
    const gravity = 9.81;
    const mass = this.virtualWeight;
    const force = mass * gravity * Math.cos(direction.angle);

    // Apply resistance to motors
    this.device.setMotorTorque(force);

    return {
      force: force,
      power: force * velocity,
      workDone: force * velocity * deltaTime
    };
  }
}
```

## 5. Voice Chat Protocol

### 5.1 WebRTC Voice Channels

```javascript
// Spatial voice chat setup
const voiceConfig = {
  codec: 'opus', // High-quality, low-latency
  bitrate: 32000, // 32 kbps
  spatialAudio: true,
  maxDistance: 20, // meters
  rolloffFactor: 1.0
};

// Create peer connection
const peerConnection = new RTCPeerConnection({
  iceServers: [
    {urls: 'stun:stun.wia-fitness.com:3478'},
    {urls: 'turn:turn.wia-fitness.com:3478', username: 'user', credential: 'pass'}
  ]
});

// Add spatial audio processing
const audioContext = new AudioContext();
const panner = audioContext.createPanner();
panner.panningModel = 'HRTF';
panner.distanceModel = 'inverse';
panner.maxDistance = voiceConfig.maxDistance;
```

### 5.2 Voice Activity Detection

```javascript
class VoiceActivityDetector {
  constructor(threshold = -50) {
    this.threshold = threshold; // dB
    this.active = false;
  }

  analyze(audioData) {
    const rms = this.calculateRMS(audioData);
    const db = 20 * Math.log10(rms);

    this.active = db > this.threshold;
    return this.active;
  }

  calculateRMS(samples) {
    const sum = samples.reduce((acc, val) => acc + val * val, 0);
    return Math.sqrt(sum / samples.length);
  }
}
```

## 6. Network Optimization

### 6.1 Bandwidth Management

```javascript
const NetworkManager = {
  // Measure available bandwidth
  async measureBandwidth() {
    const startTime = Date.now();
    const response = await fetch('https://api.wia-fitness.com/bandwidth-test');
    const data = await response.blob();
    const duration = Date.now() - startTime;

    const bandwidth = (data.size * 8) / (duration / 1000); // bits per second
    return bandwidth;
  },

  // Adjust quality based on bandwidth
  adjustQuality(bandwidth) {
    if (bandwidth > 50000000) return 'ultra'; // 50 Mbps
    if (bandwidth > 20000000) return 'high';  // 20 Mbps
    if (bandwidth > 10000000) return 'medium'; // 10 Mbps
    return 'low';
  }
};
```

### 6.2 Packet Loss Recovery

```javascript
class PacketLossHandler {
  constructor() {
    this.sequenceNumber = 0;
    this.receivedPackets = new Map();
    this.maxBufferSize = 100;
  }

  handlePacket(packet) {
    // Detect missing packets
    if (packet.seq !== this.sequenceNumber + 1) {
      const missing = this.sequenceNumber + 1;
      console.warn(`Packet loss detected: ${missing}`);

      // Request retransmission
      this.requestRetransmit(missing);

      // Interpolate missing data
      return this.interpolate(
        this.receivedPackets.get(this.sequenceNumber),
        packet
      );
    }

    this.receivedPackets.set(packet.seq, packet);
    this.sequenceNumber = packet.seq;
    return packet;
  }

  interpolate(prev, next) {
    // Linear interpolation for smooth transitions
    const alpha = 0.5;
    return {
      position: {
        x: prev.position.x * (1 - alpha) + next.position.x * alpha,
        y: prev.position.y * (1 - alpha) + next.position.y * alpha,
        z: prev.position.z * (1 - alpha) + next.position.z * alpha
      }
    };
  }
}
```

## 7. Security Protocols

### 7.1 Encrypted Communication

All real-time communication MUST use encryption:
- **DTLS-SRTP** for WebRTC voice/video
- **TLS 1.3** for WebSocket connections
- **End-to-end encryption** for peer-to-peer data

### 7.2 Anti-Cheat Measures

```javascript
class AntiCheat {
  validateMovement(current, previous, deltaTime) {
    // Check for impossible movements
    const distance = this.calculateDistance(current, previous);
    const maxSpeed = 10; // meters per second
    const actualSpeed = distance / deltaTime;

    if (actualSpeed > maxSpeed) {
      this.flagSuspiciousActivity('movement-speed', {
        actual: actualSpeed,
        max: maxSpeed,
        userId: current.userId
      });
      return false;
    }

    return true;
  }

  validateScore(scoreChange, sessionData) {
    // Verify score changes are legitimate
    const expectedScore = this.calculateExpectedScore(sessionData);
    const tolerance = 0.1; // 10% tolerance

    if (Math.abs(scoreChange - expectedScore) / expectedScore > tolerance) {
      this.flagSuspiciousActivity('score-manipulation', {
        actual: scoreChange,
        expected: expectedScore
      });
      return false;
    }

    return true;
  }
}
```

## 8. Performance Monitoring

### 8.1 Metrics Collection

```javascript
const PerformanceMonitor = {
  metrics: {
    fps: 0,
    latency: 0,
    packetLoss: 0,
    bandwidth: 0,
    jitter: 0
  },

  collect() {
    this.metrics.fps = this.measureFPS();
    this.metrics.latency = this.measureLatency();
    this.metrics.packetLoss = this.measurePacketLoss();
    this.metrics.bandwidth = this.measureBandwidth();
    this.metrics.jitter = this.measureJitter();

    // Send to analytics
    this.sendMetrics();
  },

  sendMetrics() {
    navigator.sendBeacon('/api/v1/metrics', JSON.stringify({
      timestamp: Date.now(),
      sessionId: this.sessionId,
      metrics: this.metrics,
      philosophy: '弘益人間'
    }));
  }
};
```

## 9. Fallback and Degradation

### 9.1 Graceful Degradation Strategy

```javascript
const DegradationPolicy = {
  excellent: {
    fps: 90,
    resolution: '2160p',
    haptics: 'full',
    spatialAudio: true,
    multiplayer: 100
  },

  good: {
    fps: 72,
    resolution: '1440p',
    haptics: 'reduced',
    spatialAudio: true,
    multiplayer: 50
  },

  acceptable: {
    fps: 60,
    resolution: '1080p',
    haptics: 'minimal',
    spatialAudio: false,
    multiplayer: 20
  },

  minimal: {
    fps: 30,
    resolution: '720p',
    haptics: 'disabled',
    spatialAudio: false,
    multiplayer: 0
  }
};
```

## 10. Protocol Versioning

All protocol messages include version for backward compatibility:

```json
{
  "protocolVersion": "1.0",
  "standard": "WIA-IND-014",
  "philosophy": "弘益人間",
  "messageType": "pose-update",
  "payload": {...}
}
```

---

**Document Version:** 1.0
**Status:** Draft for Public Comment
**Contact:** standards@wia.org
**License:** CC BY-SA 4.0

弘益人間 · Benefit All Humanity
