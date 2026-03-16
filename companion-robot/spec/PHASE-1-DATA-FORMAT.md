# Phase 1: Data Format Specification - WIA-ROB-013

## WIA-ROB-013 Companion Robot Data Format Standard

**Version**: 1.0.0  
**Date**: 2025-01-15  
**Status**: Active  
**Standard ID**: WIA-ROB-013-PHASE1-001

---

## 1. Overview

The WIA-ROB-013 Companion Robot Standard defines comprehensive data formats for AI companions that provide emotional support, social interaction, and personal assistance. This phase establishes interoperable data structures enabling cross-platform compatibility and data portability.

### 1.1 Purpose

- Enable interoperability between companion robot platforms
- Standardize emotion representation and recognition
- Define personality trait encoding
- Establish conversation history formats
- Support data portability and user control

### 1.2 Scope

This specification covers:
- Companion robot profile data structures
- Interaction message formats
- Emotion recognition data
- Personality trait encoding
- Memory and context representations
- User preference formats

### 1.3 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

All data formats prioritize user well-being, privacy, autonomy, and safety.

---

## 2. Companion Profile Format

### 2.1 Basic Profile Structure

```typescript
interface CompanionProfile {
  id: string;                    // UUID v7 (time-sorted)
  version: string;               // Semantic version
  createdAt: ISO8601;
  lastUpdated: ISO8601;
  
  // Identity
  name: string;
  avatar?: string;               // Base64 or URI
  description: string;
  
  // Personality (OCEAN+ model)
  personality: PersonalityTraits;
  
  // Configuration
  primaryPurpose: CompanionPurpose;
  targetAgeRange: AgeRange;
  communicationStyle: CommunicationStyle;
  
  // Languages
  languagePrimary: string;       // ISO 639-1
  languagesSupported: string[];
  
  // Capabilities
  capabilities: Capability[];
  limitations: string[];
}
```

### 2.2 Personality Traits (OCEAN+)

```typescript
interface PersonalityTraits {
  openness: number;              // 0-100
  conscientiousness: number;     // 0-100
  extraversion: number;          // 0-100
  agreeableness: number;         // 0-100
  neuroticism: number;           // 0-100
  
  // Extended traits
  playfulness: number;           // 0-100
  formality: number;             // 0-100
  proactivity: number;           // 0-100
}

enum CompanionPurpose {
  EMOTIONAL_SUPPORT = "emotional_support",
  EDUCATION = "education",
  HEALTH_WELLNESS = "health_wellness",
  ENTERTAINMENT = "entertainment",
  DAILY_ASSISTANCE = "daily_assistance"
}

enum AgeRange {
  CHILD = "child",         // 5-12
  TEEN = "teen",           // 13-17
  ADULT = "adult",         // 18-64
  SENIOR = "senior"        // 65+
}
```

---

## 3. Interaction Message Format

### 3.1 Message Structure

```typescript
interface InteractionMessage {
  id: string;                    // UUID v7
  timestamp: ISO8601;
  sender: "user" | "companion";
  
  // Content
  content: MessageContent;
  
  // Context
  context: MessageContext;
  
  // Metadata
  metadata: MessageMetadata;
}

interface MessageContent {
  text: string;
  language: string;              // ISO 639-1
  
  emotionalTone?: EmotionalTone;
  
  // Optional multimodal content
  voice?: VoiceData;
  image?: ImageData;
}

interface MessageContext {
  conversationId: string;
  sessionId: string;
  threadId?: string;
  
  // User state
  userEmotion?: EmotionState;
  userActivity?: string;
  userLocation?: string;
  
  // Temporal context
  timeOfDay: "morning" | "afternoon" | "evening" | "night";
  dayOfWeek: string;
  specialOccasion?: string;
}
```

### 3.2 Emotional Tone

```typescript
interface EmotionalTone {
  valence: number;               // -1.0 (negative) to +1.0 (positive)
  arousal: number;               // 0.0 (calm) to 1.0 (excited)
  dominance: number;             // -1.0 (submissive) to +1.0 (dominant)
  intensity: number;             // 0.0 to 1.0
}
```

---

## 4. Emotion Recognition Format

### 4.1 Emotion State

```typescript
interface EmotionState {
  timestamp: ISO8601;
  
  // Discrete emotion model
  primaryEmotion: PrimaryEmotion;
  secondaryEmotions: EmotionProbability[];
  
  // Dimensional model
  dimensions: EmotionalDimensions;
  
  // Confidence and sources
  overallConfidence: number;     // 0.0 to 1.0
  sources: EmotionSources;
}

enum PrimaryEmotion {
  JOY = "joy",
  SADNESS = "sadness",
  ANGER = "anger",
  FEAR = "fear",
  SURPRISE = "surprise",
  DISGUST = "disgust",
  CONTEMPT = "contempt"
}

interface EmotionProbability {
  emotion: string;
  probability: number;           // 0.0 to 1.0
}

interface EmotionalDimensions {
  valence: number;               // -1.0 to +1.0
  arousal: number;               // 0.0 to 1.0
  dominance: number;             // -1.0 to +1.0
}

interface EmotionSources {
  text?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  voice?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  facial?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
}
```

---

## 5. Memory and Context Format

### 5.1 Long-Term Memory

```typescript
interface LongTermMemory {
  userId: string;
  companionId: string;
  
  // User profile
  userProfile: UserProfile;
  
  // Relationship history
  firstInteraction: ISO8601;
  totalInteractions: number;
  lastInteraction: ISO8601;
  
  // Learned preferences
  preferences: UserPreferences;
  
  // Important memories
  memories: Memory[];
}

interface UserProfile {
  personalInfo: {
    name?: string;
    interests: string[];
    goals: string[];
    challenges: string[];
  };
  
  communicationPreferences: {
    preferredLanguage: string;
    formalityLevel: number;      // 0-100
    verbosity: number;            // 0-100
    emojiUsage: number;           // 0-100
  };
  
  emotionalProfile: {
    baselineAffect: EmotionalTone;
    expressionStyle: "reserved" | "moderate" | "expressive";
    commonTriggers: {
      positive: string[];
      negative: string[];
    };
    copingStrategies: string[];
  };
}

interface Memory {
  id: string;
  timestamp: ISO8601;
  type: "event" | "fact" | "preference" | "goal" | "concern";
  content: string;
  importance: number;            // 0.0 to 1.0
  lastAccessed: ISO8601;
  accessCount: number;
}
```

---

## 6. Safety and Crisis Data

### 6.1 Crisis Detection

```typescript
interface CrisisDetection {
  timestamp: ISO8601;
  severity: "low" | "medium" | "high" | "critical";
  type: CrisisType;
  indicators: string[];
  confidence: number;            // 0.0 to 1.0
  
  // Required actions
  recommendedActions: string[];
  resourcesProvided: CrisisResource[];
  
  // Follow-up
  followUpRequired: boolean;
  followUpTiming?: string;
}

enum CrisisType {
  SUICIDAL_IDEATION = "suicidal_ideation",
  SELF_HARM = "self_harm",
  HARM_TO_OTHERS = "harm_to_others",
  SEVERE_DISTRESS = "severe_distress",
  ABUSE_DISCLOSURE = "abuse_disclosure",
  SUBSTANCE_CRISIS = "substance_crisis"
}

interface CrisisResource {
  type: "hotline" | "emergency" | "text_line" | "website";
  name: string;
  contact: string;
  availability: string;
  language: string[];
}
```

---

## 7. Data Validation

All data must pass validation before processing:

- **UUID Format**: Valid UUID v7 with proper timestamp encoding
- **Timestamp Format**: ISO 8601 with timezone (UTC preferred)
- **Range Validation**: All numeric scores within specified ranges
- **Language Codes**: Valid ISO 639-1 codes only
- **Text Length**: Maximum 10,000 characters per message
- **Enum Values**: Only defined enum values accepted

---

## 8. Data Portability

Users must be able to export their data in standard formats:

```typescript
interface DataExport {
  exportDate: ISO8601;
  version: string;
  
  companionProfile: CompanionProfile;
  userProfile: UserProfile;
  conversationHistory: InteractionMessage[];
  memories: Memory[];
  
  // Analytics (optional)
  analytics?: {
    totalInteractions: number;
    averageSatisfaction: number;
    topTopics: string[];
    emotionTrends: any;
  };
}
```

Export formats supported:
- JSON (required)
- CSV (for tabular data)
- PDF (human-readable report)

---

## 9. Privacy and Security

### 9.1 Data Protection Requirements

- All personal data encrypted at rest (AES-256)
- All transmissions use TLS 1.3+
- No data retention without explicit consent
- User-controlled data deletion
- Data minimization principles

### 9.2 Sensitive Data Handling

Sensitive categories requiring extra protection:
- Mental health information
- Crisis interactions
- Personal relationships
- Financial information
- Health conditions

---

## 10. Compliance Checklist

✓ Data formats follow standard specifications  
✓ All numeric values within valid ranges  
✓ Timestamps in ISO 8601 format  
✓ UUIDs properly generated and validated  
✓ Language codes are valid ISO 639-1  
✓ Emotion representations include confidence scores  
✓ Privacy and security requirements met  
✓ Data portability supported  
✓ Crisis detection properly structured  
✓ Documentation complete and accessible

---

**WIA-ROB-013 PHASE 1 - Data Format Specification**  
© 2025 World Certification Industry Association  
弘益人間 · Benefit All Humanity
