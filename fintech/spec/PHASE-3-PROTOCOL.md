# WIA Fintech Accessibility: Phase 3 Communication Protocol Specification

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Phase 3 - Communication Protocol Standard
- **Standard**: WIA-FIN-COMM-001

---

## 1. Overview

본 문서는 WIA Fintech 접근성 데이터의 통신 프로토콜 표준을 정의합니다. 금융 서비스와 WIA 기기 간의 안전하고 접근 가능한 통신을 보장합니다.

### 1.1 Protocol Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         WIA FINTECH COMMUNICATION STACK                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                          Application Layer                                   │
│     ┌───────────────┐  ┌───────────────┐  ┌───────────────┐                │
│     │ Profile Mgmt  │  │ Transaction   │  │ Notification  │                │
│     │     API       │  │     API       │  │     API       │                │
│     └───────────────┘  └───────────────┘  └───────────────┘                │
├─────────────────────────────────────────────────────────────────────────────┤
│                        Accessibility Layer                                   │
│     ┌───────────────┐  ┌───────────────┐  ┌───────────────┐                │
│     │  Multi-Modal  │  │    Input      │  │     WIA       │                │
│     │   Output      │  │  Adaptation   │  │ Integration   │                │
│     └───────────────┘  └───────────────┘  └───────────────┘                │
├─────────────────────────────────────────────────────────────────────────────┤
│                          Security Layer                                      │
│     ┌───────────────┐  ┌───────────────┐  ┌───────────────┐                │
│     │    OAuth2/    │  │   mTLS/TLS    │  │   PCI-DSS     │                │
│     │    OIDC       │  │    1.3        │  │  Compliance   │                │
│     └───────────────┘  └───────────────┘  └───────────────┘                │
├─────────────────────────────────────────────────────────────────────────────┤
│                          Transport Layer                                     │
│     ┌───────────────┐  ┌───────────────┐  ┌───────────────┐                │
│     │   HTTPS/2     │  │  WebSocket    │  │     BLE       │                │
│     │   REST API    │  │    (WSS)      │  │    GATT       │                │
│     └───────────────┘  └───────────────┘  └───────────────┘                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   弘益人間 (홍익인간) - Secure & Accessible Financial Communication         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Communication Channels

| Channel | Use Case | Protocol | Security |
|---------|----------|----------|----------|
| **Mobile App ↔ Server** | Profile management, transactions | HTTPS REST / WebSocket | OAuth2 + TLS 1.3 |
| **ATM ↔ Server** | Real-time accessibility sync | HTTPS REST / gRPC | mTLS + HSM |
| **ATM ↔ WIA Device** | Haptic/visual guidance | BLE GATT | BLE Secure Pairing |
| **POS ↔ WIA Device** | Payment assistance | NFC + BLE | PCI P2PE |
| **Server ↔ Server** | Bank interoperability | REST API / GraphQL | mTLS + API Key |

---

## 2. REST API Specification

### 2.1 Base Configuration

```yaml
openapi: 3.1.0
info:
  title: WIA Fintech Accessibility API
  version: 1.0.0
  description: |
    접근성 중심 금융 서비스 API
    - PCI-DSS Level 1 Compliant
    - WCAG 2.1 AA API Documentation
    - Multi-language error messages

servers:
  - url: https://api.wia-fintech.org/v1
    description: Production
  - url: https://sandbox.wia-fintech.org/v1
    description: Sandbox (Testing)

security:
  - OAuth2: [read:profile, write:profile, read:atm]
  - BearerToken: []
```

### 2.2 Authentication Endpoints

#### 2.2.1 Accessible Authentication

```typescript
// POST /auth/login
interface AccessibleLoginRequest {
  // Standard credentials
  credentials: {
    type: 'password' | 'biometric' | 'pin' | 'voice' | 'passkey';
    value?: string;
    biometricToken?: string;
    passkeyAssertion?: PublicKeyCredential;
  };

  // Accessibility options
  accessibility: {
    // Extended timeout for users who need more time
    extendedTimeout: boolean;  // default: false, extended: 120s

    // Voice feedback during auth
    voiceFeedback: boolean;
    voiceLanguage?: string;    // e.g., "ko-KR"

    // Haptic feedback via WIA device
    hapticFeedback: boolean;
    hapticDeviceId?: string;

    // Simplified flow for cognitive accessibility
    simplifiedFlow: boolean;
    stepByStepGuidance: boolean;
  };

  // Device context
  deviceContext: {
    deviceId: string;
    deviceType: 'mobile' | 'atm' | 'web' | 'pos';
    wiaDeviceConnected: boolean;
    wiaDeviceIds?: string[];
  };
}

interface AccessibleLoginResponse {
  // Standard auth response
  accessToken: string;
  refreshToken: string;
  expiresIn: number;
  tokenType: 'Bearer';

  // Accessibility sync
  accessibilityProfile: UserFinancialAccessibilityProfile;

  // Multi-modal confirmation
  confirmation: {
    visual: {
      message: string;
      icon: string;
    };
    audio: {
      message: string;
      ssml?: string;
    };
    haptic?: {
      pattern: string;
      deviceId: string;
    };
  };
}
```

#### 2.2.2 PIN Entry with Accessibility

```typescript
// POST /auth/pin/verify
interface AccessiblePinVerifyRequest {
  encryptedPin: string;        // RSA-OAEP encrypted

  // Accessibility features during PIN entry
  accessibility: {
    // Audio tone feedback (not digit, but confirmation)
    audioToneFeedback: boolean;

    // Haptic feedback per digit
    hapticPerDigit: boolean;

    // Extended timeout
    timeout: number;           // seconds, max 180

    // Voice confirmation of PIN length
    confirmPinLength: boolean;
  };
}

interface AccessiblePinVerifyResponse {
  verified: boolean;
  attemptsRemaining?: number;

  // Accessible error message
  error?: {
    code: string;
    message: string;
    messageSimple: string;     // Cognitive accessibility
    voiceMessage: string;
    hapticPattern?: string;
  };
}
```

### 2.3 Profile Management Endpoints

#### 2.3.1 Get User Profile

```typescript
// GET /profiles/{profileId}
// Headers:
//   Accept-Language: ko-KR
//   X-WIA-Device-Id: exo-001

interface GetProfileResponse {
  profile: UserFinancialAccessibilityProfile;

  // Sync information
  sync: {
    lastSyncAt: string;
    version: string;
    checksum: string;
  };

  // Connected WIA devices
  wiaDevices: {
    deviceId: string;
    deviceType: WIADeviceType;
    status: 'connected' | 'disconnected' | 'syncing';
    lastSeen: string;
  }[];
}
```

#### 2.3.2 Update Profile with Conflict Resolution

```typescript
// PUT /profiles/{profileId}
// Content-Type: application/json
// If-Match: "version-hash"

interface UpdateProfileRequest {
  profile: Partial<UserFinancialAccessibilityProfile>;

  // Conflict resolution strategy
  conflictResolution: 'server_wins' | 'client_wins' | 'merge' | 'manual';

  // Sync to connected devices
  syncToDevices: boolean;
  targetDevices?: string[];
}

interface UpdateProfileResponse {
  profile: UserFinancialAccessibilityProfile;

  // Conflict info if any
  conflicts?: {
    field: string;
    serverValue: any;
    clientValue: any;
    resolvedValue: any;
    resolution: string;
  }[];

  // Sync status
  syncStatus: {
    deviceId: string;
    status: 'synced' | 'pending' | 'failed';
    error?: string;
  }[];
}
```

### 2.4 ATM Discovery & Accessibility Endpoints

#### 2.4.1 Find Accessible ATMs

```typescript
// GET /atms/search
// Query Parameters:
//   lat: 37.5665
//   lng: 126.9780
//   radius: 5 (km)
//   profileId: user-001 (for personalized accessibility matching)

interface ATMSearchRequest {
  location: {
    latitude: number;
    longitude: number;
    radius: number;           // km
  };

  // User profile for matching
  profileId?: string;

  // Accessibility filters
  filters?: {
    wheelchairAccessible?: boolean;
    audioGuidance?: boolean;
    brailleLabels?: boolean;
    wiaEnabled?: boolean;
    wiaLevel?: WIACertificationLevel;
    minAccessibilityScore?: number;
  };

  // Response preferences
  preferences?: {
    maxResults?: number;
    sortBy?: 'distance' | 'accessibility_score' | 'compatibility';
    includeCompatibilityDetails?: boolean;
  };
}

interface ATMSearchResponse {
  atms: {
    atm: ATMAccessibilityProfile;
    distance: number;

    // Personalized compatibility (if profileId provided)
    compatibility?: {
      score: number;
      isCompatible: boolean;
      issues: CompatibilityIssue[];
      recommendations: string[];
    };

    // Navigation assistance
    navigation?: {
      walkingTime: number;      // minutes
      drivingTime: number;
      accessibleRoute: boolean;
      directions: NavigationStep[];
    };
  }[];

  // Meta information
  meta: {
    totalResults: number;
    searchRadius: number;
    searchTime: number;        // ms
  };
}

interface NavigationStep {
  instruction: string;
  instructionSimple: string;   // Cognitive accessibility
  distance: number;
  maneuver: string;

  // Accessibility info
  accessibility: {
    surfaceType: string;
    hasStairs: boolean;
    hasRamp: boolean;
    tactileGuideAvailable: boolean;
  };
}
```

#### 2.4.2 Get ATM Real-time Status

```typescript
// GET /atms/{atmId}/status
// WebSocket: wss://api.wia-fintech.org/v1/atms/{atmId}/status/live

interface ATMStatusResponse {
  atmId: string;
  timestamp: string;

  // Operational status
  operational: {
    status: 'operational' | 'limited' | 'offline';
    services: {
      withdrawal: boolean;
      deposit: boolean;
      transfer: boolean;
      billPay: boolean;
    };
  };

  // Accessibility status
  accessibility: {
    audioGuidanceOnline: boolean;
    headphoneJackWorking: boolean;
    brailleLabelsPresent: boolean;
    wheelchairAccessible: boolean;
    screenWorking: boolean;
    keypadWorking: boolean;
  };

  // WIA integration status
  wiaStatus?: {
    enabled: boolean;
    profileSyncAvailable: boolean;
    exoskeletonGuidanceReady: boolean;
    bionicEyeDisplayReady: boolean;
  };

  // Wait time estimate
  waitTime?: {
    estimatedMinutes: number;
    confidence: number;
  };
}
```

### 2.5 Transaction Endpoints

#### 2.5.1 Initiate Accessible Transaction

```typescript
// POST /transactions
interface AccessibleTransactionRequest {
  // Transaction details
  transaction: {
    type: TransactionType;
    amount: {
      value: number;
      currency: string;
    };
    recipient?: {
      accountNumber: string;
      bankCode?: string;
      name?: string;
    };
  };

  // Accessibility preferences for this transaction
  accessibility: {
    // Confirmation modality
    confirmationMode: 'visual' | 'audio' | 'haptic' | 'all';

    // Voice readback of amount
    voiceReadback: boolean;
    voiceLanguage?: string;

    // Haptic confirmation pattern
    hapticConfirmation: boolean;
    hapticDeviceId?: string;

    // Simple language for cognitive accessibility
    useSimpleLanguage: boolean;

    // Extended review time
    extendedReviewTime: boolean;
    reviewTimeSeconds?: number;
  };

  // Channel info
  channel: {
    type: 'mobile' | 'atm' | 'web' | 'pos';
    deviceId: string;
    wiaDeviceIds?: string[];
  };
}

interface AccessibleTransactionResponse {
  transactionId: string;
  status: 'pending_confirmation' | 'processing' | 'completed' | 'failed';

  // Multi-modal confirmation content
  confirmation: {
    visual: {
      summary: string;
      details: TransactionDetail[];
      colorCode: string;
      icon: string;
    };

    audio: {
      summary: string;
      ssml?: string;
      voiceScript: string;
    };

    haptic?: {
      pattern: string;
      intensity: number;
      deviceId: string;
    };

    simpleLanguage?: {
      summary: string;
      details: string[];
    };
  };

  // Required actions
  requiredAction?: {
    type: 'confirm' | 'authenticate' | 'review';
    deadline: string;
    accessibleInstructions: string;
  };
}
```

### 2.6 Notification Endpoints

#### 2.6.1 Register for Accessible Notifications

```typescript
// POST /notifications/register
interface NotificationRegistrationRequest {
  profileId: string;

  // Delivery channels
  channels: {
    push: {
      enabled: boolean;
      token?: string;
    };
    sms: {
      enabled: boolean;
      phoneNumber?: string;
    };
    email: {
      enabled: boolean;
      address?: string;
    };
    wiaDevice: {
      enabled: boolean;
      deviceIds?: string[];
    };
  };

  // Accessibility preferences
  accessibility: {
    // Multi-modal delivery
    preferredModalities: SensoryModality[];
    fallbackModalities: SensoryModality[];

    // Voice preferences
    voiceEnabled: boolean;
    voiceLanguage?: string;
    voiceSpeed?: number;

    // Haptic preferences
    hapticEnabled: boolean;
    hapticIntensity?: number;

    // Sign language
    signLanguageEnabled: boolean;
    preferredSignLanguage?: SignLanguageType;

    // Cognitive accessibility
    simpleLanguage: boolean;
    readingLevel?: number;
  };

  // Notification types
  subscriptions: {
    type: NotificationType;
    enabled: boolean;
    urgencyThreshold: NotificationPriority;
  }[];
}
```

---

## 3. WebSocket Real-time Protocol

### 3.1 Connection Establishment

```typescript
// Connection URL: wss://api.wia-fintech.org/v1/ws
// Headers:
//   Authorization: Bearer {token}
//   X-WIA-Profile-Id: {profileId}
//   X-WIA-Device-Ids: exo-001,eye-001

interface WSConnectionConfig {
  // Heartbeat
  heartbeatInterval: 30000;    // 30 seconds
  heartbeatTimeout: 10000;     // 10 seconds

  // Reconnection
  reconnectMaxAttempts: 5;
  reconnectBackoff: 'exponential';
  reconnectInitialDelay: 1000;

  // Accessibility
  accessibility: {
    voiceAlertOnDisconnect: boolean;
    hapticAlertOnDisconnect: boolean;
    reconnectVoiceProgress: boolean;
  };
}
```

### 3.2 Message Frame Format

```typescript
interface WSMessage {
  // Header
  id: string;                  // UUID
  type: WSMessageType;
  timestamp: string;           // ISO 8601

  // Payload
  payload: any;

  // Accessibility metadata
  accessibility?: {
    priority: 'low' | 'normal' | 'high' | 'urgent' | 'emergency';
    modalities: SensoryModality[];
    voiceText?: string;
    hapticPattern?: string;
  };
}

type WSMessageType =
  | 'NOTIFICATION'             // Server → Client
  | 'TRANSACTION_UPDATE'       // Server → Client
  | 'ATM_STATUS_CHANGE'        // Server → Client
  | 'PROFILE_SYNC'             // Bidirectional
  | 'WIA_DEVICE_EVENT'         // Client → Server
  | 'HEARTBEAT'                // Bidirectional
  | 'EMERGENCY'                // Server → Client (high priority)
  | 'ACK';                     // Client → Server
```

### 3.3 Notification Messages

```typescript
// Server → Client: Real-time notification
interface WSNotificationMessage {
  id: string;
  type: 'NOTIFICATION';
  timestamp: string;

  payload: {
    notification: AccessibleNotification;

    // Delivery requirements
    delivery: {
      requiresAck: boolean;
      ackDeadline?: string;
      escalation?: {
        enabled: boolean;
        escalateAfterSeconds: number;
        escalationContacts: string[];
      };
    };
  };

  accessibility: {
    priority: NotificationPriority;
    modalities: SensoryModality[];

    // Pre-rendered content for each modality
    rendered: {
      visual?: {
        html: string;
        plainText: string;
        highContrastHtml?: string;
      };
      audio?: {
        text: string;
        ssml: string;
        audioUrl?: string;
      };
      haptic?: {
        pattern: string;
        intensity: number;
        duration: number;
      };
      signLanguage?: {
        type: SignLanguageType;
        videoUrl: string;
        avatarData?: string;
      };
    };
  };
}
```

### 3.4 Transaction Update Messages

```typescript
interface WSTransactionUpdateMessage {
  id: string;
  type: 'TRANSACTION_UPDATE';
  timestamp: string;

  payload: {
    transactionId: string;
    status: TransactionStatus;

    // Amount (for voice readback)
    amount?: {
      value: number;
      currency: string;
      formatted: string;
      voiceText: string;
    };

    // Status-specific data
    statusData?: {
      completedAt?: string;
      failureReason?: string;
      reference?: string;
    };
  };

  accessibility: {
    priority: 'normal' | 'high';
    modalities: ['visual', 'audio', 'haptic'];

    rendered: {
      visual: {
        statusIcon: string;
        statusColor: string;
        message: string;
      };
      audio: {
        text: string;
        successSound?: string;
        failureSound?: string;
      };
      haptic: {
        pattern: string;       // 'success_pulse' | 'failure_vibrate'
        intensity: number;
      };
    };
  };
}

type TransactionStatus =
  | 'initiated'
  | 'authenticating'
  | 'processing'
  | 'pending_confirmation'
  | 'completed'
  | 'failed'
  | 'cancelled';
```

### 3.5 Emergency Messages

```typescript
interface WSEmergencyMessage {
  id: string;
  type: 'EMERGENCY';
  timestamp: string;

  payload: {
    emergencyType: EmergencyType;
    code: string;

    // Emergency details
    details: {
      title: string;
      description: string;
      descriptionSimple: string;

      // Fraud-specific
      fraudDetails?: {
        transactionId?: string;
        amount?: number;
        merchant?: string;
        location?: string;
      };

      // Security-specific
      securityDetails?: {
        eventType: string;
        deviceInfo?: string;
        location?: string;
      };
    };

    // Required action
    action: {
      type: 'confirm' | 'deny' | 'call' | 'block_card';
      deadline: string;
      url?: string;
      phoneNumber?: string;
    };
  };

  accessibility: {
    priority: 'emergency';
    modalities: ['visual', 'audio', 'haptic'];

    // Emergency requires ALL modalities
    rendered: {
      visual: {
        backgroundColor: '#FF0000';
        textColor: '#FFFFFF';
        flashPattern: 'urgent_blink';  // 2Hz max for photosensitivity
        message: string;
        icon: 'alert_emergency';
      };
      audio: {
        text: string;
        ssml: string;
        alarmSound: 'emergency_alarm';
        repeatCount: 3;
        volume: 100;
      };
      haptic: {
        pattern: 'emergency_pulse';
        intensity: 100;
        duration: 5000;
        bodyRegions: ['wrist', 'arm'];
      };
      signLanguage?: {
        type: SignLanguageType;
        videoUrl: string;
        priority: 'immediate';
      };
    };

    // Escalation
    escalation: {
      enabled: true;
      notifyEmergencyContacts: true;
      escalationDelaySeconds: 30;
    };
  };
}

type EmergencyType =
  | 'fraud_alert'
  | 'security_breach'
  | 'suspicious_activity'
  | 'card_stolen'
  | 'account_compromise';
```

---

## 4. BLE GATT Protocol for ATM/WIA Integration

### 4.1 WIA Financial Service

```
Service UUID: 0x1860 (WIA Financial Accessibility)

Characteristics:
├── Financial Profile (0x2B60)
│   ├── Properties: Read, Write, Notify
│   └── Format: Compressed JSON
│
├── Transaction Assist (0x2B61)
│   ├── Properties: Write, Notify
│   └── Format: Binary
│
├── ATM Guidance (0x2B62)
│   ├── Properties: Notify
│   └── Format: Haptic Command
│
├── PIN Entry Assist (0x2B63)
│   ├── Properties: Write, Notify
│   └── Format: Encrypted Binary
│
├── Amount Feedback (0x2B64)
│   ├── Properties: Notify
│   └── Format: Haptic/Audio Pattern
│
└── Emergency Signal (0x2B65)
    ├── Properties: Notify
    └── Format: Emergency Frame
```

### 4.2 ATM Guidance Commands

```typescript
interface ATMGuidanceCommand {
  // Command type
  type: ATMGuidanceType;

  // Target body region for exoskeleton
  target: {
    region: HapticBodyRegion;
    subRegion?: string;
  };

  // Guidance pattern
  guidance: {
    direction: 'up' | 'down' | 'left' | 'right' | 'forward' | 'back';
    intensity: number;         // 0-100
    pattern: string;           // e.g., 'pulse', 'continuous', 'morse'
    duration: number;          // ms
  };

  // ATM component being guided to
  component: {
    type: ATMComponent;
    description: string;
    voiceDescription?: string;
  };
}

type ATMGuidanceType =
  | 'locate_card_slot'
  | 'locate_keypad'
  | 'locate_cash_dispenser'
  | 'locate_receipt'
  | 'locate_headphone_jack'
  | 'locate_confirm_button'
  | 'locate_cancel_button';

type ATMComponent =
  | 'card_slot'
  | 'keypad'
  | 'cash_dispenser'
  | 'receipt_slot'
  | 'headphone_jack'
  | 'screen'
  | 'confirm_button'
  | 'cancel_button';

type HapticBodyRegion =
  | 'left_hand'
  | 'right_hand'
  | 'left_wrist'
  | 'right_wrist'
  | 'left_arm'
  | 'right_arm'
  | 'torso';
```

### 4.3 Amount Haptic Encoding

```typescript
interface AmountHapticEncoding {
  // Amount information
  amount: {
    value: number;
    currency: string;
    formatted: string;
  };

  // Encoding method
  encoding: {
    method: 'digit_pulse' | 'magnitude' | 'morse' | 'custom';

    // Digit pulse: each digit as separate pulses
    digitPulse?: {
      digits: number[];        // e.g., [1, 0, 0, 0, 0, 0] for 100000
      pulseDuration: number;   // ms per pulse
      digitGap: number;        // ms between digits
      groupGap: number;        // ms between groups (thousands)
    };

    // Magnitude: intensity proportional to amount
    magnitude?: {
      minAmount: number;
      maxAmount: number;
      minIntensity: number;
      maxIntensity: number;
    };

    // Morse code
    morse?: {
      dotDuration: number;
      dashDuration: number;
      gapDuration: number;
    };
  };

  // Confirmation
  confirmation: {
    requiresAck: boolean;
    ackPattern: string;        // User response pattern
  };
}
```

### 4.4 PIN Entry Assistance

```typescript
interface PINEntryAssist {
  // Entry mode
  mode: 'standard' | 'voice_confirm' | 'haptic_confirm' | 'guided';

  // Per-digit feedback (not revealing digit)
  digitFeedback: {
    visual: {
      showMask: boolean;       // Show * for each digit
      progressIndicator: boolean;
    };
    audio: {
      toneEnabled: boolean;
      toneFrequency: number;   // Different tone per position
      voicePosition: boolean;  // "First digit entered"
    };
    haptic: {
      enabled: boolean;
      pattern: string;         // Short pulse per digit
    };
  };

  // Completion feedback
  completionFeedback: {
    visual: {
      message: string;
      color: string;
    };
    audio: {
      message: string;
      sound: string;
    };
    haptic: {
      pattern: string;
      intensity: number;
    };
  };

  // Security
  security: {
    timeout: number;           // Auto-cancel after seconds
    scrambleKeypad: boolean;   // For shoulder surfing prevention
    privacyMode: boolean;      // Audio through headphone only
  };
}
```

---

## 5. Security Protocol

### 5.1 Authentication Flow

```typescript
interface AuthenticationFlow {
  // OAuth2 + PKCE for mobile/web
  oauth2: {
    authorizationEndpoint: string;
    tokenEndpoint: string;
    scopes: string[];
    pkceRequired: true;

    // Accessibility extensions
    accessibilityParams: {
      voice_feedback: boolean;
      extended_timeout: boolean;
      simplified_flow: boolean;
    };
  };

  // mTLS for ATM/server communication
  mtls: {
    certificateValidation: true;
    certificatePinning: true;
    minimumTLSVersion: 'TLS1.3';
  };

  // Session management
  session: {
    accessTokenLifetime: 3600;    // 1 hour
    refreshTokenLifetime: 604800; // 7 days

    // Extended session for accessibility
    extendedSession: {
      enabled: boolean;
      maxLifetime: 7200;          // 2 hours
      requiresJustification: true;
    };
  };
}
```

### 5.2 Encryption Standards

```typescript
interface EncryptionStandards {
  // Data at rest
  atRest: {
    algorithm: 'AES-256-GCM';
    keyManagement: 'HSM';
  };

  // Data in transit
  inTransit: {
    minimumTLS: 'TLS1.3';
    cipherSuites: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ];
    certificatePinning: true;
  };

  // PIN encryption
  pinEncryption: {
    algorithm: 'RSA-OAEP-256';
    keyRotation: 'daily';
    hsmRequired: true;
  };

  // WIA device communication
  wiaEncryption: {
    algorithm: 'AES-128-CCM';    // BLE standard
    keyExchange: 'ECDH-P256';
    bondingRequired: true;
  };
}
```

### 5.3 Accessible Security Features

```typescript
interface AccessibleSecurityFeatures {
  // Accessible authentication challenges
  authChallenge: {
    // CAPTCHA alternatives
    captchaAlternatives: {
      audioCaptcha: boolean;
      hapticPattern: boolean;  // Recognize pattern via WIA device
      simplifiedChallenge: boolean;
    };

    // OTP accessibility
    otpAccessibility: {
      voiceReadout: boolean;
      extendedValidity: boolean;  // 120s vs standard 30s
      hapticDelivery: boolean;    // Via exoskeleton
      largeDisplay: boolean;
    };
  };

  // Security notifications
  securityNotifications: {
    multiModal: true;
    voiceAlerts: true;
    hapticAlerts: true;
    simplifiedMessages: true;
    emergencyEscalation: true;
  };

  // Fraud detection accessibility
  fraudDetection: {
    accessibleChallenges: boolean;
    voiceVerification: boolean;
    trustedDeviceRecognition: boolean;
    caregiverNotification: boolean;
  };
}
```

---

## 6. Error Handling Protocol

### 6.1 Error Response Format

```typescript
interface AccessibleErrorResponse {
  // Standard error info
  error: {
    code: string;              // e.g., "TRANSACTION_FAILED"
    message: string;           // Technical message
    details?: any;
  };

  // Accessible error content
  accessibility: {
    // Standard language
    userMessage: string;

    // Simple language (cognitive accessibility)
    simpleMessage: string;

    // Voice script
    voiceMessage: string;
    voiceSSML?: string;

    // Haptic feedback for error
    hapticPattern?: {
      pattern: string;
      intensity: number;
      meaning: string;         // e.g., "error", "warning", "info"
    };

    // Sign language
    signLanguageVideo?: string;
  };

  // Recovery guidance
  recovery: {
    steps: RecoveryStep[];
    autoRecovery: boolean;
    retryAfterSeconds?: number;
    supportContact?: {
      phone: string;
      tty: string;
      videoRelay: string;
    };
  };

  // Localization
  localizations?: {
    [languageCode: string]: {
      userMessage: string;
      simpleMessage: string;
      voiceMessage: string;
      recoverySteps: RecoveryStep[];
    };
  };
}

interface RecoveryStep {
  order: number;
  action: string;
  description: string;
  descriptionSimple: string;
  voiceInstruction: string;
  estimatedTime?: number;      // seconds
  requiresAssistance?: boolean;
}
```

### 6.2 Error Codes

| Code | HTTP Status | Description | Recovery |
|------|-------------|-------------|----------|
| `AUTH_FAILED` | 401 | Authentication failed | Retry with voice guidance |
| `AUTH_TIMEOUT` | 408 | Extended timeout exceeded | Restart with extended time |
| `PROFILE_NOT_FOUND` | 404 | Accessibility profile missing | Create new profile |
| `PROFILE_SYNC_FAILED` | 409 | Profile sync conflict | Manual resolution |
| `ATM_UNAVAILABLE` | 503 | ATM offline | Find alternative ATM |
| `ATM_FEATURE_UNAVAILABLE` | 422 | Required feature unavailable | Find compatible ATM |
| `TRANSACTION_FAILED` | 400 | Transaction error | Review and retry |
| `INSUFFICIENT_FUNDS` | 422 | Insufficient balance | Voice balance notification |
| `CARD_BLOCKED` | 403 | Card is blocked | Contact support (TTY available) |
| `WIA_DEVICE_DISCONNECTED` | 503 | WIA device lost connection | Reconnect guidance |
| `WIA_DEVICE_INCOMPATIBLE` | 422 | WIA device version mismatch | Update device |
| `SECURITY_ALERT` | 403 | Security concern detected | Multi-modal verification |
| `RATE_LIMITED` | 429 | Too many requests | Wait with accessible countdown |

### 6.3 Error Escalation

```typescript
interface ErrorEscalation {
  // Automatic escalation conditions
  triggers: {
    repeatedFailures: {
      threshold: number;       // e.g., 3 failures
      window: number;          // within seconds
    };
    criticalErrors: string[];  // error codes
    userRequestedHelp: boolean;
  };

  // Escalation actions
  actions: {
    notifyCaregiver: {
      enabled: boolean;
      delay: number;           // seconds before notification
      channels: ('sms' | 'push' | 'call')[];
    };

    connectSupport: {
      enabled: boolean;
      supportType: 'chat' | 'voice' | 'video_relay';
      priority: 'normal' | 'high';
    };

    alternativeFlow: {
      enabled: boolean;
      flowType: 'simplified' | 'assisted' | 'branch_referral';
    };
  };

  // Accessible escalation notifications
  notification: {
    visual: {
      message: string;
      icon: string;
    };
    audio: {
      message: string;
      reassurance: string;     // "Help is on the way"
    };
    haptic: {
      pattern: string;
      meaning: string;
    };
  };
}
```

---

## 7. Rate Limiting with Accessibility

### 7.1 Adaptive Rate Limits

```typescript
interface AdaptiveRateLimits {
  // Standard limits
  standard: {
    requests: number;
    window: number;            // seconds
  };

  // Extended limits for accessibility users
  accessibility: {
    // Users with cognitive accessibility needs
    cognitive: {
      requests: number;        // 1.5x standard
      window: number;
      justification: 'Extended processing time';
    };

    // Users with motor accessibility needs
    motor: {
      requests: number;        // 2x standard
      window: number;
      justification: 'Slower input methods';
    };

    // Users with WIA devices
    wiaEnabled: {
      requests: number;        // 1.5x for device sync overhead
      window: number;
      justification: 'WIA device synchronization';
    };
  };

  // Rate limit response
  response: {
    headers: {
      'X-RateLimit-Limit': number;
      'X-RateLimit-Remaining': number;
      'X-RateLimit-Reset': number;
      'X-RateLimit-Accessibility-Extended': boolean;
    };

    // Accessible rate limit notification
    notification: {
      visual: string;
      audio: string;
      countdownEnabled: boolean;
      hapticWarning: boolean;
    };
  };
}
```

---

## 8. Protocol Compliance

### 8.1 Required Features

| Feature | Priority | Accessibility Impact |
|---------|----------|---------------------|
| Multi-modal API responses | Required | All users benefit |
| Extended timeouts | Required | Motor/cognitive users |
| Voice feedback endpoints | Required | Blind users |
| Haptic integration | Recommended | Deaf-blind users |
| Simple language variants | Required | Cognitive accessibility |
| Emergency protocol | Required | All users (safety) |
| WIA device sync | Recommended | Enhanced accessibility |
| Sign language content | Recommended | Deaf users |

### 8.2 Conformance Levels

| Level | Requirements |
|-------|-------------|
| **Bronze** | Multi-modal responses, Extended timeouts, Basic error accessibility |
| **Silver** | Bronze + Voice feedback, WIA profile sync, Simple language |
| **Gold** | Silver + Full WIA integration, Sign language, Haptic patterns |
| **Platinum** | Gold + Real-time WIA guidance, Emergency escalation, Full localization |

---

## 9. API Documentation Accessibility

### 9.1 Documentation Requirements

```typescript
interface AccessibleAPIDocumentation {
  // OpenAPI accessibility extensions
  openapi: {
    // Custom extension for accessibility
    'x-accessibility': {
      wcagLevel: 'AA';
      screenReaderOptimized: true;
      keyboardNavigable: true;
      highContrastAvailable: true;
    };
  };

  // Each endpoint includes
  endpointDocumentation: {
    // Standard
    summary: string;
    description: string;

    // Accessibility
    accessibilityNotes: {
      modalities: SensoryModality[];
      extendedTimeoutAvailable: boolean;
      voiceFeedbackSupported: boolean;
      hapticFeedbackSupported: boolean;
    };

    // Examples with accessibility context
    examples: {
      standard: any;
      withVoiceFeedback: any;
      withHapticFeedback: any;
      withSimplifiedFlow: any;
    };
  };
}
```

---

## 10. References

- OAuth 2.0 Authorization Framework (RFC 6749)
- OpenID Connect Core 1.0
- PCI-DSS v4.0
- WCAG 2.1 Level AA
- Bluetooth Core Specification v5.3
- WIA Exoskeleton Protocol v1.0
- WIA Bionic Eye Protocol v1.0
- WIA Voice-Sign Protocol v1.0

---

## Document Information

- **Document ID**: WIA-FIN-COMM-001
- **Classification**: Public Standard
- **Maintainer**: WIA Standards Committee
- **License**: Open Standard (CC BY 4.0)

弘益人間 (홍익인간) - Accessible Financial Communication for All
