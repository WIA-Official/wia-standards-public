# WIA Fintech Accessibility: Phase 1 Data Format Standard

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Phase 1 - Data Format Standard
- **Standard**: WIA-FIN-DATA-001

---

## 1. Overview

This specification defines the data format standards for WIA Fintech Accessibility, enabling interoperability between financial services, assistive technologies, and WIA ecosystem devices.

### 1.1 Design Philosophy

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WIA FINTECH ACCESSIBILITY DATA MODEL                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   User Profile  │    │ Financial Svc   │    │   Transaction   │         │
│  │  Accessibility  │◄──▶│  Accessibility  │◄──▶│  Accessibility  │         │
│  │   Preferences   │    │   Capabilities  │    │    Metadata     │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           │                      │                      │                   │
│           └──────────────────────┼──────────────────────┘                   │
│                                  │                                          │
│                                  ▼                                          │
│                    ┌─────────────────────────┐                             │
│                    │    WIA Integration      │                             │
│                    │  ┌─────┐ ┌─────┐ ┌────┐│                             │
│                    │  │ EXO │ │ EYE │ │ VS ││                             │
│                    │  └─────┘ └─────┘ └────┘│                             │
│                    └─────────────────────────┘                             │
│                                                                             │
│  弘益人間 (홍익인간) - Financial Accessibility for All                      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Core Data Entities

| Entity | Description | Schema |
|--------|-------------|--------|
| **UserFinancialAccessibilityProfile** | User's accessibility preferences for financial services | Section 3 |
| **FinancialServiceAccessibilityProfile** | Service/device accessibility capabilities | Section 4 |
| **AccessibleTransactionRecord** | Transaction with accessibility metadata | Section 5 |
| **AccessibleNotification** | Multi-modal financial notification | Section 6 |
| **ATMAccessibilityProfile** | ATM device accessibility features | Section 7 |
| **PaymentCardAccessibility** | Accessible payment card specifications | Section 8 |

---

## 2. Common Types

### 2.1 Accessibility Levels

```typescript
enum AccessibilityLevel {
  NONE = "none",
  BASIC = "basic",         // Minimum compliance
  ENHANCED = "enhanced",   // Above minimum
  FULL = "full",           // Complete accessibility
  UNIVERSAL = "universal"  // WIA Platinum level
}
```

### 2.2 Sensory Modalities

```typescript
enum SensoryModality {
  VISUAL = "visual",
  AUDITORY = "auditory",
  HAPTIC = "haptic",
  BRAILLE = "braille"
}

enum OutputChannel {
  SCREEN = "screen",
  SPEAKER = "speaker",
  HEADPHONE = "headphone",
  VIBRATION = "vibration",
  BRAILLE_DISPLAY = "braille_display",
  EXOSKELETON = "exoskeleton",
  BIONIC_EYE = "bionic_eye",
  VOICE_SIGN = "voice_sign"
}
```

### 2.3 Financial Service Types

```typescript
enum FinancialServiceType {
  BANKING = "banking",
  ATM = "atm",
  MOBILE_BANKING = "mobile_banking",
  ONLINE_BANKING = "online_banking",
  PAYMENT_TERMINAL = "payment_terminal",
  CRYPTOCURRENCY = "cryptocurrency",
  INVESTMENT = "investment",
  INSURANCE = "insurance",
  LENDING = "lending"
}

enum TransactionType {
  BALANCE_INQUIRY = "balance_inquiry",
  WITHDRAWAL = "withdrawal",
  DEPOSIT = "deposit",
  TRANSFER = "transfer",
  PAYMENT = "payment",
  BILL_PAY = "bill_pay",
  PURCHASE = "purchase",
  REFUND = "refund"
}
```

---

## 3. User Financial Accessibility Profile

### 3.1 Schema Definition

```typescript
interface UserFinancialAccessibilityProfile {
  // Profile identification
  profileId: string;
  wiaId?: string;
  version: string;
  createdAt: string;
  updatedAt: string;

  // Personal information
  personalInfo: {
    preferredName?: string;
    preferredLanguage: string;
    region: string;
    timezone: string;
  };

  // Accessibility needs
  accessibilityNeeds: {
    sensory: SensoryAccessibilityNeeds;
    motor: MotorAccessibilityNeeds;
    cognitive: CognitiveAccessibilityNeeds;
  };

  // Financial service preferences
  financialPreferences: {
    preferredAuthMethod: AuthenticationMethod[];
    transactionConfirmation: ConfirmationPreference;
    notificationPreferences: NotificationPreferences;
    securityPreferences: SecurityAccessibilityPreferences;
  };

  // ATM preferences
  atmPreferences: {
    preferredInterface: ATMInterfacePreference;
    audioGuidance: AudioGuidanceSettings;
    timeout: TimeoutPreferences;
    receiptFormat: ReceiptFormatPreference;
  };

  // Card preferences
  cardPreferences: {
    preferredCardType: AccessibleCardType;
    pinEntryMethod: PINEntryMethod[];
    contactlessPreferred: boolean;
  };

  // WIA device integration
  wiaIntegration: {
    enabled: boolean;
    connectedDevices: WIADeviceConnection[];
    preferredOutputDevice: string;
    crossDeviceSync: boolean;
  };

  // Emergency settings
  emergencySettings: {
    emergencyContacts: EmergencyContact[];
    fraudAlertPreferences: FraudAlertPreferences;
    panicButtonEnabled: boolean;
  };
}
```

### 3.2 Sensory Accessibility Needs

```typescript
interface SensoryAccessibilityNeeds {
  visual: {
    level: "none" | "low-vision" | "legally-blind" | "totally-blind";
    colorVision?: ColorVisionType;
    lightSensitivity?: "none" | "mild" | "moderate" | "severe";
    preferences: {
      fontSize: "normal" | "large" | "x-large" | "xx-large";
      fontFamily: "default" | "sans-serif" | "dyslexic-friendly";
      highContrast: boolean;
      darkMode: boolean;
      reduceMotion: boolean;
      screenMagnification?: number; // 1.0 - 10.0
    };
    assistiveTech: {
      screenReader: boolean;
      screenReaderType?: "voiceover" | "talkback" | "nvda" | "jaws" | "other";
      brailleDisplay: boolean;
      magnifier: boolean;
    };
  };

  auditory: {
    level: "none" | "mild" | "moderate" | "severe" | "profound" | "deaf";
    usesHearingAid: boolean;
    usesCochlearImplant: boolean;
    preferredSignLanguage?: SignLanguageType;
    preferences: {
      visualAlerts: boolean;
      captionsEnabled: boolean;
      signLanguageVideo: boolean;
      monoAudio: boolean;
      audioBalance: number; // -1.0 (left) to 1.0 (right)
      volumeBoost: boolean;
    };
  };
}

type ColorVisionType =
  | "normal"
  | "protanopia"    // Red-blind
  | "deuteranopia"  // Green-blind
  | "tritanopia"    // Blue-blind
  | "achromatopsia" // Complete color blindness
  | "protanomaly"   // Red-weak
  | "deuteranomaly" // Green-weak
  | "tritanomaly";  // Blue-weak

type SignLanguageType =
  | "ASL"  // American
  | "BSL"  // British
  | "KSL"  // Korean
  | "JSL"  // Japanese
  | "DGS"  // German
  | "LSF"  // French
  | "ISL"  // International
  | "other";
```

### 3.3 Motor Accessibility Needs

```typescript
interface MotorAccessibilityNeeds {
  level: "none" | "mild" | "moderate" | "severe";

  upperLimb: {
    left: "full" | "limited" | "minimal" | "none";
    right: "full" | "limited" | "minimal" | "none";
  };

  fineMotor: "normal" | "reduced" | "limited" | "none";

  preferences: {
    largeTargets: boolean;
    extendedTimeouts: boolean;
    singleHandOperation: boolean;
    preferredHand?: "left" | "right" | "either";
    voiceControl: boolean;
    switchAccess: boolean;
    eyeTracking: boolean;
    stickyKeys: boolean;
    dwellClick: boolean;
    dwellTime?: number; // milliseconds
  };

  assistiveDevices: {
    usesWheelchair: boolean;
    usesExoskeleton: boolean;
    usesMouthStick: boolean;
    usesHeadPointer: boolean;
    usesEyeTracker: boolean;
    customDevice?: string;
  };
}
```

### 3.4 Cognitive Accessibility Needs

```typescript
interface CognitiveAccessibilityNeeds {
  level: "none" | "mild" | "moderate" | "significant";

  preferences: {
    simplifiedInterface: boolean;
    reducedOptions: boolean;
    clearLabels: boolean;
    consistentLayout: boolean;
    confirmBeforeActions: boolean;
    undoSupport: boolean;
    progressIndicators: boolean;
    stepByStepGuidance: boolean;
    memoryAids: boolean;
    timeExtensions: boolean;
    errorPrevention: boolean;
  };

  readingLevel: 1 | 2 | 3 | 4 | 5; // 1=simplest, 5=standard

  supportNeeds: {
    caregiverAccess: boolean;
    authorizedHelper?: AuthorizedHelper[];
    transactionLimits?: TransactionLimits;
  };
}

interface AuthorizedHelper {
  helperId: string;
  name: string;
  relationship: string;
  permissions: HelperPermission[];
  expiresAt?: string;
}

type HelperPermission =
  | "view_balance"
  | "view_transactions"
  | "make_transfers"
  | "pay_bills"
  | "manage_settings"
  | "full_access";

interface TransactionLimits {
  dailyLimit?: number;
  singleTransactionLimit?: number;
  requiresConfirmation: boolean;
  notifyCaregiver: boolean;
}
```

### 3.5 Authentication Preferences

```typescript
type AuthenticationMethod =
  | "pin"
  | "password"
  | "biometric_fingerprint"
  | "biometric_face"
  | "biometric_voice"
  | "pattern"
  | "otp_sms"
  | "otp_email"
  | "otp_app"
  | "hardware_token"
  | "passkey";

interface SecurityAccessibilityPreferences {
  preferredMethods: AuthenticationMethod[];

  pinPreferences: {
    length: 4 | 6 | 8;
    audioFeedback: boolean;
    hapticFeedback: boolean;
    extendedTimeout: boolean;
    timeoutSeconds: number;
  };

  otpPreferences: {
    deliveryMethod: "sms" | "email" | "app" | "voice";
    extendedValidity: boolean;
    validitySeconds: number; // default 30, extended 120
    audioReadout: boolean;
  };

  biometricPreferences: {
    primaryMethod: "fingerprint" | "face" | "voice";
    fallbackMethod: AuthenticationMethod;
    accessibleEnrollment: boolean;
  };

  sessionPreferences: {
    extendedSession: boolean;
    sessionTimeoutMinutes: number;
    inactivityWarning: boolean;
    warningSeconds: number;
  };
}
```

---

## 4. Financial Service Accessibility Profile

### 4.1 Schema Definition

```typescript
interface FinancialServiceAccessibilityProfile {
  // Service identification
  serviceId: string;
  serviceName: string;
  provider: string;
  serviceType: FinancialServiceType;
  version: string;

  // Accessibility capabilities
  accessibilityCapabilities: {
    wcagLevel: "A" | "AA" | "AAA";
    wiaLevel: "bronze" | "silver" | "gold" | "platinum";
    certifications: AccessibilityCertification[];
    lastAuditDate: string;
  };

  // Visual accessibility
  visualAccessibility: {
    screenReaderCompatible: boolean;
    supportedScreenReaders: string[];
    highContrastMode: boolean;
    customFontSizes: boolean;
    fontSizeRange: { min: number; max: number };
    colorBlindModes: ColorVisionType[];
    darkMode: boolean;
    reducedMotion: boolean;
  };

  // Auditory accessibility
  auditoryAccessibility: {
    audioOutput: boolean;
    textToSpeech: boolean;
    ttsLanguages: string[];
    signLanguageSupport: boolean;
    supportedSignLanguages: SignLanguageType[];
    videoRelay: boolean;
    captioning: boolean;
    visualAlerts: boolean;
  };

  // Motor accessibility
  motorAccessibility: {
    keyboardFullAccess: boolean;
    voiceControl: boolean;
    switchAccess: boolean;
    eyeTracking: boolean;
    adjustableTimeouts: boolean;
    timeoutRange: { min: number; max: number };
    singleHandOperation: boolean;
    largeTargets: boolean;
    targetSizePixels: number;
  };

  // Cognitive accessibility
  cognitiveAccessibility: {
    simplifiedMode: boolean;
    clearLabels: boolean;
    consistentNavigation: boolean;
    errorPrevention: boolean;
    undoSupport: boolean;
    helpSystem: boolean;
    stepByStepMode: boolean;
    readingLevelOptions: number[];
  };

  // WIA integration
  wiaIntegration: {
    supported: boolean;
    exoskeletonSupport: boolean;
    bionicEyeSupport: boolean;
    voiceSignSupport: boolean;
    smartWheelchairSupport: boolean;
    wiaProfileSync: boolean;
  };

  // Multi-modal support
  multiModalSupport: {
    simultaneousModalities: boolean;
    modalitySwitching: boolean;
    customModalityCombinations: boolean;
    fallbackModalities: SensoryModality[];
  };

  // Supported languages
  supportedLanguages: LanguageSupport[];

  // Contact information
  accessibilitySupport: {
    dedicatedHelpline: boolean;
    helplineNumber?: string;
    ttyNumber?: string;
    videoRelayService?: string;
    emailSupport?: string;
    chatSupport: boolean;
    accessibleChatbot: boolean;
  };
}

interface AccessibilityCertification {
  name: string;
  level: string;
  issuedBy: string;
  issuedDate: string;
  expiryDate: string;
  certificateUrl?: string;
}

interface LanguageSupport {
  language: string;
  languageCode: string;
  screenReader: boolean;
  tts: boolean;
  signLanguage: boolean;
  braille: boolean;
  simplifiedContent: boolean;
}
```

---

## 5. Accessible Transaction Record

### 5.1 Schema Definition

```typescript
interface AccessibleTransactionRecord {
  // Transaction identification
  transactionId: string;
  referenceNumber: string;
  timestamp: string;
  timezone: string;

  // Transaction details
  transaction: {
    type: TransactionType;
    amount: {
      value: number;
      currency: string;
      formattedAmount: string; // "1,234.56 USD"
    };
    description: string;
    category?: string;
    merchant?: MerchantInfo;
  };

  // Accessibility metadata
  accessibilityMetadata: {
    // How transaction was delivered to user
    deliveryModalities: DeliveryRecord[];

    // User acknowledgment
    acknowledgment: {
      acknowledged: boolean;
      acknowledgedAt?: string;
      acknowledgedVia?: OutputChannel;
    };

    // Accessible content versions
    accessibleContent: {
      plainText: string;
      simpleLanguage: string;
      voiceScript: string;
      signLanguageAvailable: boolean;
      signLanguageUrl?: string;
      brailleText?: string;
    };

    // Alert information
    alertInfo?: {
      alertType: "info" | "warning" | "fraud" | "success";
      priority: "low" | "normal" | "high" | "urgent";
      requiresAction: boolean;
      actionDeadline?: string;
    };
  };

  // Device/channel used
  channelInfo: {
    channel: "atm" | "mobile" | "web" | "branch" | "phone" | "pos";
    deviceId?: string;
    deviceType?: string;
    accessibilityModeUsed: boolean;
    wiaDeviceUsed?: string;
  };

  // Security
  securityInfo: {
    authenticationMethod: AuthenticationMethod;
    ipAddress?: string;
    locationApproximate?: string;
    riskScore?: number;
  };
}

interface DeliveryRecord {
  modality: SensoryModality;
  channel: OutputChannel;
  deliveredAt: string;
  successful: boolean;
  failureReason?: string;
}

interface MerchantInfo {
  name: string;
  category: string;
  categoryCode: string;
  location?: string;
  accessibilityRating?: number;
}
```

---

## 6. Accessible Notification

### 6.1 Schema Definition

```typescript
interface AccessibleNotification {
  // Notification identification
  notificationId: string;
  timestamp: string;
  expiresAt?: string;

  // Notification type
  type: NotificationType;
  priority: NotificationPriority;
  category: NotificationCategory;

  // Content
  content: {
    title: string;
    body: string;

    // Multi-format content
    formats: {
      plainText: string;
      richText?: string;
      simpleLanguage: string;
      voiceScript: string;
      ssml?: string; // Speech Synthesis Markup Language
      braille?: string;
    };

    // Localized content
    localizations: {
      [languageCode: string]: {
        title: string;
        body: string;
        voiceScript: string;
      };
    };

    // Sign language
    signLanguage?: {
      available: boolean;
      languages: SignLanguageType[];
      videoUrls: { [lang: string]: string };
    };
  };

  // Action required
  action?: {
    required: boolean;
    actionType: ActionType;
    actionLabel: string;
    accessibleInstructions: string;
    deadline?: string;
    url?: string;
  };

  // Delivery preferences
  delivery: {
    channels: DeliveryChannel[];
    respectQuietHours: boolean;
    escalation: EscalationConfig;
  };

  // Accessibility features
  accessibility: {
    // Visual
    visual?: {
      iconCode: string;
      iconAltText: string;
      backgroundColor?: string;
      textColor?: string;
      flashPattern?: string;
    };

    // Auditory
    auditory?: {
      soundFile?: string;
      ttsEnabled: boolean;
      ttsVoice?: string;
      ttsRate?: number;
      ttsVolume?: number;
      repeatCount?: number;
    };

    // Haptic
    haptic?: {
      pattern: string;
      intensity: number;
      duration: number;
    };
  };

  // WIA integration
  wiaDelivery?: {
    exoskeleton?: HapticDeliveryConfig;
    bionicEye?: VisualDeliveryConfig;
    voiceSign?: SignDeliveryConfig;
  };

  // Tracking
  tracking: {
    sent: boolean;
    sentAt?: string;
    delivered: boolean;
    deliveredAt?: string;
    read: boolean;
    readAt?: string;
    actioned: boolean;
    actionedAt?: string;
  };
}

type NotificationType =
  | "transaction"
  | "balance_alert"
  | "payment_due"
  | "fraud_alert"
  | "security_alert"
  | "promotional"
  | "account_update"
  | "document_ready"
  | "appointment_reminder";

type NotificationPriority = "low" | "normal" | "high" | "urgent" | "emergency";

type NotificationCategory =
  | "financial"
  | "security"
  | "informational"
  | "promotional"
  | "legal";

type ActionType =
  | "confirm"
  | "deny"
  | "review"
  | "pay"
  | "call"
  | "visit"
  | "upload"
  | "acknowledge";

interface DeliveryChannel {
  type: "push" | "sms" | "email" | "in_app" | "voice_call" | "wia_device";
  priority: number;
  enabled: boolean;
}

interface EscalationConfig {
  enabled: boolean;
  escalateAfterMinutes: number;
  escalationChannels: DeliveryChannel[];
  maxEscalations: number;
}

interface HapticDeliveryConfig {
  enabled: boolean;
  pattern: string;
  bodyRegion: string;
  intensity: number;
}

interface VisualDeliveryConfig {
  enabled: boolean;
  displayMode: "overlay" | "full" | "minimal";
  position: string;
  duration: number;
}

interface SignDeliveryConfig {
  enabled: boolean;
  signLanguage: SignLanguageType;
  videoUrl?: string;
  avatarEnabled: boolean;
}
```

---

## 7. ATM Accessibility Profile

### 7.1 Schema Definition

```typescript
interface ATMAccessibilityProfile {
  // ATM identification
  atmId: string;
  bankCode: string;
  manufacturer: string;
  model: string;
  location: ATMLocation;
  status: "operational" | "limited" | "offline";

  // Physical accessibility
  physicalAccessibility: {
    wheelchairAccessible: boolean;
    heightAdjustable: boolean;
    screenHeight: number; // cm from ground
    keypadHeight: number;
    reachRange: "standard" | "forward" | "side";
    clearFloorSpace: boolean;
    approachType: "parallel" | "forward" | "both";
    indoorOutdoor: "indoor" | "outdoor" | "both";
    sheltered: boolean;
    wellLit: boolean;
    lightLevel?: number; // lux
  };

  // Audio accessibility
  audioAccessibility: {
    audioGuidance: boolean;
    headphoneJack: boolean;
    jackType: "3.5mm" | "2.5mm";
    jackLocation: string;
    speakerOutput: boolean;
    volumeAdjustable: boolean;
    volumeRange: { min: number; max: number };
    supportedLanguages: string[];
    speechRate: "fixed" | "adjustable";
    privacyMode: boolean; // audio only through headphones
  };

  // Tactile accessibility
  tactileAccessibility: {
    brailleLabels: boolean;
    tactileKeypad: boolean;
    keypadLayout: "phone" | "calculator";
    key5Marker: boolean; // raised dot on 5
    confirmButtonMarker: "circle" | "raised" | "none";
    cancelButtonMarker: "cross" | "raised" | "none";
    cardSlotTactile: boolean;
    cashDispenserTactile: boolean;
    receiptSlotTactile: boolean;
  };

  // Screen accessibility
  screenAccessibility: {
    touchscreen: boolean;
    physicalButtons: boolean;
    screenSize: number; // inches diagonal
    resolution: string;
    highContrastMode: boolean;
    fontSizeAdjustable: boolean;
    colorBlindMode: boolean;
    screenAngleAdjustable: boolean;
    antiGlare: boolean;
    privacyScreen: boolean;
  };

  // Operational features
  operationalFeatures: {
    extendedTimeout: boolean;
    timeoutSeconds: number;
    transactionRepeat: boolean;
    receiptOptions: ReceiptOption[];
    denominationChoice: boolean;
    cardlessTransaction: boolean;
    nfcEnabled: boolean;
    qrCodeEnabled: boolean;
    mobilePreStaging: boolean;
  };

  // WIA integration
  wiaIntegration: {
    enabled: boolean;
    wiaProfileSync: boolean;
    exoskeletonGuidance: boolean;
    bionicEyeDisplay: boolean;
    voiceSignSupport: boolean;
    smartWheelchairPositioning: boolean;
    bluetoothEnabled: boolean;
    nfcWiaEnabled: boolean;
  };

  // Accessibility certification
  certification: {
    adaCompliant: boolean;
    eaaCompliant: boolean;
    wiaLevel: "bronze" | "silver" | "gold" | "platinum";
    lastInspectionDate: string;
    certifications: string[];
  };
}

interface ATMLocation {
  address: string;
  city: string;
  state: string;
  country: string;
  postalCode: string;
  latitude: number;
  longitude: number;
  locationDescription: string;
  accessInstructions?: string;
  nearbyLandmarks?: string[];
}

type ReceiptOption =
  | "paper"
  | "email"
  | "sms"
  | "app_notification"
  | "braille_print"
  | "audio_summary"
  | "none";
```

---

## 8. Payment Card Accessibility

### 8.1 Schema Definition

```typescript
interface PaymentCardAccessibility {
  // Card identification
  cardId: string;
  issuer: string;
  network: "visa" | "mastercard" | "amex" | "discover" | "other";
  cardType: "debit" | "credit" | "prepaid";

  // Physical accessibility features
  physicalFeatures: {
    // Tactile identification
    tactile: {
      notched: boolean;
      notchLocation: "left" | "right" | "corner";
      notchShape: "semicircle" | "triangle" | "square";
      brailleEmbossed: boolean;
      brailleContent: string;
      raisedPrint: boolean;
      tactileStrip: boolean;
    };

    // Visual accessibility
    visual: {
      highContrast: boolean;
      backgroundColor: string;
      textColor: string;
      contrastRatio: number;
      largeText: boolean;
      fontSizeCardNumber: number;
      fontSizeName: number;
      fontFamily: string;
      cardNumberFormat: "flat" | "embossed" | "laser";
    };

    // Card dimensions
    dimensions: {
      standard: boolean; // ISO/IEC 7810 ID-1
      width: number;
      height: number;
      thickness: number;
    };
  };

  // Digital accessibility
  digitalFeatures: {
    contactless: boolean;
    nfcEnabled: boolean;
    mobileWalletCompatible: boolean;
    supportedWallets: string[];
    virtualCardAvailable: boolean;
    instantNotifications: boolean;
    accessibleAppIntegration: boolean;
  };

  // Usage accessibility
  usageFeatures: {
    pinBypass: boolean; // for motor impaired
    voicePinEntry: boolean;
    extendedPinTimeout: boolean;
    signatureWaiver: boolean;
    assistedTransaction: boolean;
  };

  // Accessibility metadata
  accessibilityMetadata: {
    wiaLevel: "bronze" | "silver" | "gold" | "platinum";
    targetUsers: AccessibilityTargetUser[];
    featureDescription: string;
    orderingInstructions: string;
    replacementPolicy: string;
  };
}

type AccessibilityTargetUser =
  | "blind"
  | "low_vision"
  | "deaf"
  | "motor_impaired"
  | "cognitive"
  | "elderly"
  | "general";
```

---

## 9. WIA Integration Schemas

### 9.1 Financial Exoskeleton Integration

```typescript
interface FinancialExoskeletonConfig {
  enabled: boolean;
  deviceId: string;

  // Haptic feedback mapping
  hapticMapping: {
    transactionSuccess: HapticPattern;
    transactionFailed: HapticPattern;
    fraudAlert: HapticPattern;
    balanceNotification: HapticPattern;
    pinEntry: HapticPattern;
    menuNavigation: HapticPattern;
  };

  // ATM guidance
  atmGuidance: {
    cardSlotLocation: HapticGuidance;
    keypadLocation: HapticGuidance;
    cashDispenser: HapticGuidance;
    receiptSlot: HapticGuidance;
    confirmButton: HapticGuidance;
    cancelButton: HapticGuidance;
  };

  // Amount encoding
  amountEncoding: {
    method: "morse" | "magnitude" | "sequential";
    digitsViaHaptic: boolean;
    confirmationPattern: boolean;
  };
}

interface HapticPattern {
  patternId: string;
  description: string;
  intensity: number; // 0-100
  duration: number; // ms
  bodyRegions: string[];
  sequence: HapticSequenceItem[];
}

interface HapticSequenceItem {
  region: string;
  intensity: number;
  duration: number;
  delay: number;
}

interface HapticGuidance {
  direction: "left" | "right" | "up" | "down" | "forward";
  intensity: number;
  pulsePattern: string;
}
```

### 9.2 Financial Bionic Eye Integration

```typescript
interface FinancialBionicEyeConfig {
  enabled: boolean;
  deviceId: string;

  // Display modes
  displayModes: {
    transactionOverlay: boolean;
    balanceDisplay: boolean;
    atmRecognition: boolean;
    cardRecognition: boolean;
    currencyRecognition: boolean;
    receiptReading: boolean;
  };

  // Visual enhancements
  visualEnhancements: {
    contrastEnhancement: boolean;
    magnification: number;
    colorCorrection: ColorVisionType;
    textHighlighting: boolean;
  };

  // ATM assistance
  atmAssistance: {
    screenMirroring: boolean;
    keypadOverlay: boolean;
    slotHighlighting: boolean;
    amountVerification: boolean;
  };

  // Document reading
  documentReading: {
    statementReading: boolean;
    contractReading: boolean;
    receiptParsing: boolean;
    signatureGuidance: boolean;
  };

  // Security features
  security: {
    privacyBlur: boolean;
    shoulderSurfingAlert: boolean;
    pinMasking: boolean;
  };
}
```

### 9.3 Financial Voice-Sign Integration

```typescript
interface FinancialVoiceSignConfig {
  enabled: boolean;
  deviceId: string;

  // Translation settings
  translation: {
    primaryLanguage: SignLanguageType;
    voiceToSign: boolean;
    signToVoice: boolean;
    realTimeTranslation: boolean;
  };

  // Financial vocabulary
  financialVocabulary: {
    bankingTerms: boolean;
    transactionTypes: boolean;
    securityPhrases: boolean;
    emergencyPhrases: boolean;
  };

  // Customer service integration
  customerService: {
    videoRelaySupport: boolean;
    signInterpreterOnDemand: boolean;
    chatbotSignSupport: boolean;
    branchAppointmentSign: boolean;
  };

  // Document support
  documentSupport: {
    contractExplanation: boolean;
    statementExplanation: boolean;
    formAssistance: boolean;
  };

  // Emergency phrases (preloaded)
  emergencyPhrases: EmergencyPhrase[];
}

interface EmergencyPhrase {
  phraseId: string;
  englishText: string;
  signLanguageVideo: { [lang: string]: string };
  priority: "normal" | "urgent" | "emergency";
  category: "fraud" | "help" | "medical" | "general";
}
```

---

## 10. Validation Rules

### 10.1 Required Fields

```typescript
// User Profile - Required
const requiredUserProfileFields = [
  "profileId",
  "version",
  "personalInfo.preferredLanguage",
  "accessibilityNeeds.sensory.visual.level",
  "accessibilityNeeds.sensory.auditory.level",
  "financialPreferences.preferredAuthMethod",
];

// Service Profile - Required
const requiredServiceProfileFields = [
  "serviceId",
  "serviceName",
  "provider",
  "serviceType",
  "accessibilityCapabilities.wcagLevel",
];

// ATM Profile - Required
const requiredATMProfileFields = [
  "atmId",
  "location.address",
  "physicalAccessibility.wheelchairAccessible",
  "audioAccessibility.audioGuidance",
  "tactileAccessibility.brailleLabels",
];
```

### 10.2 Validation Functions

```typescript
interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

interface ValidationError {
  field: string;
  code: string;
  message: string;
}

interface ValidationWarning {
  field: string;
  code: string;
  message: string;
  suggestion: string;
}

// Example validation codes
const ValidationCodes = {
  REQUIRED_FIELD_MISSING: "E001",
  INVALID_FORMAT: "E002",
  OUT_OF_RANGE: "E003",
  INCOMPATIBLE_VALUES: "E004",
  ACCESSIBILITY_INCOMPLETE: "W001",
  WIA_INTEGRATION_MISSING: "W002",
};
```

---

## 11. Schema Files

### 11.1 Required Schemas

| Schema File | Description |
|-------------|-------------|
| `user-financial-accessibility-profile.schema.json` | User preferences and needs |
| `financial-service-accessibility-profile.schema.json` | Service capabilities |
| `accessible-transaction-record.schema.json` | Transaction with accessibility metadata |
| `accessible-notification.schema.json` | Multi-modal notification |
| `atm-accessibility-profile.schema.json` | ATM accessibility features |
| `payment-card-accessibility.schema.json` | Accessible card specifications |
| `wia-financial-integration.schema.json` | WIA device integration |

---

## 12. Appendix

### 12.1 Currency Formatting

```typescript
interface CurrencyFormat {
  code: string;           // ISO 4217
  symbol: string;         // $, €, ₩
  symbolPosition: "before" | "after";
  decimalSeparator: "." | ",";
  thousandsSeparator: "," | "." | " " | "";
  decimalPlaces: number;
  voiceFormat: string;    // "one hundred twenty-three dollars and forty-five cents"
}

// Common currency formats
const currencyFormats: CurrencyFormat[] = [
  {
    code: "USD",
    symbol: "$",
    symbolPosition: "before",
    decimalSeparator: ".",
    thousandsSeparator: ",",
    decimalPlaces: 2,
    voiceFormat: "{amount} dollars and {cents} cents"
  },
  {
    code: "EUR",
    symbol: "€",
    symbolPosition: "after",
    decimalSeparator: ",",
    thousandsSeparator: ".",
    decimalPlaces: 2,
    voiceFormat: "{amount} euros and {cents} cents"
  },
  {
    code: "KRW",
    symbol: "₩",
    symbolPosition: "before",
    decimalSeparator: ".",
    thousandsSeparator: ",",
    decimalPlaces: 0,
    voiceFormat: "{amount} won"
  }
];
```

### 12.2 WCAG Mapping

| WIA Requirement | WCAG Criterion | Level |
|-----------------|----------------|-------|
| Screen reader support | 1.3.1, 4.1.2 | A |
| High contrast | 1.4.3, 1.4.6 | AA/AAA |
| Adjustable text | 1.4.4 | AA |
| Keyboard access | 2.1.1, 2.1.2 | A |
| Extended timeouts | 2.2.1, 2.2.3 | A/AAA |
| Error prevention | 3.3.4 | AA |
| Clear labels | 3.3.2 | A |

---

## Document Information

- **Document ID**: WIA-FIN-DATA-001
- **Classification**: Public Standard
- **Maintainer**: WIA Standards Committee
- **License**: Open Standard (CC BY 4.0)

弘益人間 (홍익인간) - Financial Accessibility for All Humanity
