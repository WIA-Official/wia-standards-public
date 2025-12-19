/**
 * WIA Emotion AI Data Models
 */

export enum EmotionCategory {
    HAPPINESS = 'happiness',
    SADNESS = 'sadness',
    ANGER = 'anger',
    FEAR = 'fear',
    DISGUST = 'disgust',
    SURPRISE = 'surprise',
    NEUTRAL = 'neutral',
    CONTEMPT = 'contempt',
    CONFUSION = 'confusion',
    INTEREST = 'interest',
    BOREDOM = 'boredom',
    FRUSTRATION = 'frustration',
    EXCITEMENT = 'excitement',
    ANXIETY = 'anxiety'
}

export enum ModalityType {
    FACIAL = 'facial',
    VOICE = 'voice',
    TEXT = 'text',
    BIOSIGNAL = 'biosignal',
    MULTIMODAL = 'multimodal'
}

export interface Emotion {
    category: EmotionCategory;
    intensity: number; // 0.0 - 1.0
    confidence: number; // 0.0 - 1.0
    onsetTime?: number;
    apexTime?: number;
    offsetTime?: number;
    sourceModality?: ModalityType;
}

export interface ActionUnit {
    au: string; // e.g., "AU6", "AU12L"
    intensity: number; // 0.0 - 1.0
    name?: string;
    intensityLabel?: 'A' | 'B' | 'C' | 'D' | 'E';
    confidence?: number;
    symmetric?: boolean;
}

export interface DimensionalModel {
    valence: number; // -1.0 to +1.0
    arousal: number; // 0.0 to 1.0
    dominance?: number; // 0.0 to 1.0
}

export interface Modality {
    type: ModalityType;
    confidence: number;
    weight?: number;
    data?: Record<string, unknown>;
}

export interface Metadata {
    provider?: string;
    modelVersion?: string;
    processingTimeMs?: number;
    culturalContext?: string;
}

export interface EmotionEvent {
    eventId: string;
    timestamp: string;
    version: string;
    emotions: Emotion[];
    sessionId?: string;
    subjectId?: string;
    actionUnits?: ActionUnit[];
    dimensional?: DimensionalModel;
    modalities?: Modality[];
    metadata?: Metadata;
}

export interface AnalysisResult {
    requestId: string;
    timestamp: string;
    processingTimeMs: number;
    emotions: Emotion[];
    actionUnits?: ActionUnit[];
    dimensional?: DimensionalModel;
    faces?: FaceResult[];
    segments?: Segment[];
    aggregate?: AggregateResult;
}

export interface FaceResult {
    faceId: number;
    boundingBox: {
        x: number;
        y: number;
        width: number;
        height: number;
    };
    emotions: Emotion[];
    actionUnits?: ActionUnit[];
    dimensional?: DimensionalModel;
    headPose?: {
        pitch: number;
        yaw: number;
        roll: number;
    };
}

export interface Segment {
    startTime: number;
    endTime: number;
    emotions: Emotion[];
    prosody?: {
        pitch: { mean: number; std: number };
        intensity: { mean: number; std: number };
        speechRate: number;
    };
}

export interface AggregateResult {
    dominantEmotion: EmotionCategory;
    averageValence: number;
    averageArousal: number;
}
