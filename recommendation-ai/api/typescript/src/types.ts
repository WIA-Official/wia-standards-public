/**
 * WIA-AI-024 Recommendation AI - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA (World Certification Industry Association)
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Core Data Types
// ============================================================================

export interface UserInteraction {
    userId: string;
    itemId: string;
    interactionType: 'view' | 'click' | 'purchase' | 'rating' | 'like' | 'share';
    value?: number;
    timestamp: Date;
    context: InteractionContext;
    metadata?: Record<string, any>;
}

export interface InteractionContext {
    device: 'mobile' | 'desktop' | 'tablet' | 'tv';
    platform?: string;
    location?: Location;
    sessionId: string;
    referrer?: string;
    userAgent?: string;
}

export interface Location {
    country?: string;
    region?: string;
    city?: string;
    coordinates?: {
        latitude: number;
        longitude: number;
    };
}

export interface ItemProfile {
    itemId: string;
    title: string;
    description?: string;
    features: Record<string, any>;
    categories: string[];
    tags: string[];
    embeddings?: number[];
    metadata: Record<string, any>;
    createdAt: Date;
    updatedAt: Date;
}

export interface UserProfile {
    userId: string;
    preferences: number[];
    demographics?: Demographics;
    segments?: string[];
    history: UserInteraction[];
    createdAt: Date;
    lastActive: Date;
}

export interface Demographics {
    age?: number;
    gender?: string;
    location?: Location;
    language?: string;
    interests?: string[];
}

// ============================================================================
// Recommendation Types
// ============================================================================

export interface RecommendationRequest {
    userId: string;
    count?: number;
    context?: InteractionContext;
    filters?: RecommendationFilters;
    diversityWeight?: number;
    excludeItems?: string[];
}

export interface RecommendationFilters {
    categories?: string[];
    tags?: string[];
    priceRange?: [number, number];
    dateRange?: [Date, Date];
    customFilters?: Record<string, any>;
}

export interface RecommendationResponse {
    items: RecommendedItem[];
    requestId: string;
    userId: string;
    timestamp: Date;
    latencyMs: number;
    metadata?: Record<string, any>;
}

export interface RecommendedItem {
    itemId: string;
    score: number;
    rank: number;
    explanation?: Explanation;
    metadata?: Record<string, any>;
}

export interface Explanation {
    type: 'collaborative' | 'content' | 'popularity' | 'hybrid';
    reason: string;
    confidence: number;
    factors?: ExplanationFactor[];
}

export interface ExplanationFactor {
    name: string;
    value: number;
    weight: number;
}

// ============================================================================
// Algorithm Types
// ============================================================================

export interface SimilarityMetric {
    name: 'cosine' | 'pearson' | 'jaccard' | 'euclidean';
    calculate(a: number[], b: number[]): number;
}

export interface CollaborativeFilteringConfig {
    algorithm: 'user-based' | 'item-based';
    similarityMetric: SimilarityMetric;
    k: number;  // Number of neighbors
    minSupport?: number;  // Minimum common items
}

export interface ContentBasedConfig {
    featureExtractors: FeatureExtractor[];
    similarityMetric: SimilarityMetric;
    profileBuilder: ProfileBuilder;
}

export interface MatrixFactorizationConfig {
    algorithm: 'svd' | 'svd++' | 'als' | 'nmf';
    latentDim: number;
    epochs: number;
    learningRate: number;
    regularization: number;
}

export interface DeepLearningConfig {
    architecture: 'ncf' | 'wide-deep' | 'deepfm' | 'transformer';
    embeddingDim: number;
    hiddenLayers: number[];
    activation: 'relu' | 'sigmoid' | 'tanh';
    optimizer: 'adam' | 'sgd' | 'rmsprop';
    batchSize: number;
    epochs: number;
}

// ============================================================================
// Feature Engineering
// ============================================================================

export interface FeatureExtractor {
    name: string;
    extract(item: ItemProfile): number[] | Record<string, any>;
}

export interface ProfileBuilder {
    build(interactions: UserInteraction[], itemProfiles: Map<string, ItemProfile>): number[];
    update(currentProfile: number[], newInteraction: UserInteraction, itemProfile: ItemProfile): number[];
}

export interface TFIDFVectorizer {
    fit(documents: string[]): void;
    transform(document: string): number[];
    vocabulary: Map<string, number>;
    idf: Map<string, number>;
}

// ============================================================================
// Evaluation Types
// ============================================================================

export interface EvaluationMetrics {
    accuracy: AccuracyMetrics;
    ranking: RankingMetrics;
    diversity: DiversityMetrics;
    business: BusinessMetrics;
}

export interface AccuracyMetrics {
    rmse?: number;
    mae?: number;
    precision: number[];  // @K for different K values
    recall: number[];
    f1: number[];
}

export interface RankingMetrics {
    ndcg: number[];  // @K for different K values
    map: number;
    mrr: number;
}

export interface DiversityMetrics {
    intraListDiversity: number;
    coverage: number;
    giniCoefficient: number;
    novelty: number;
}

export interface BusinessMetrics {
    ctr?: number;
    conversionRate?: number;
    revenue?: number;
    engagement?: number;
    retention?: number;
}

// ============================================================================
// Model Types
// ============================================================================

export interface RecommendationModel {
    name: string;
    version: string;
    train(trainingData: TrainingData, config: any): Promise<TrainingResult>;
    predict(userId: string, itemId: string): Promise<number>;
    recommend(userId: string, k: number, filters?: RecommendationFilters): Promise<RecommendedItem[]>;
    save(path: string): Promise<void>;
    load(path: string): Promise<void>;
}

export interface TrainingData {
    interactions: UserInteraction[];
    userProfiles?: Map<string, UserProfile>;
    itemProfiles: Map<string, ItemProfile>;
    metadata?: Record<string, any>;
}

export interface TrainingResult {
    metrics: EvaluationMetrics;
    duration: number;
    iterations: number;
    convergence: boolean;
    metadata?: Record<string, any>;
}

// ============================================================================
// Production Types
// ============================================================================

export interface CacheConfig {
    enabled: boolean;
    ttl: number;  // Time to live in seconds
    maxSize?: number;
    strategy: 'lru' | 'lfu' | 'fifo';
}

export interface MonitoringMetrics {
    latency: {
        p50: number;
        p95: number;
        p99: number;
    };
    throughput: number;
    errorRate: number;
    cacheHitRate: number;
    modelFreshness: number;
}

export interface ABTestConfig {
    experimentId: string;
    name: string;
    active: boolean;
    variants: {
        control: number;  // Percentage (0-100)
        treatment: number;
    };
    startDate: Date;
    endDate?: Date;
}

export interface ExperimentResult {
    experimentId: string;
    variant: string;
    metrics: BusinessMetrics;
    sampleSize: number;
    pValue: number;
    significant: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

export class RecommendationError extends Error {
    constructor(
        message: string,
        public code: string,
        public details?: any
    ) {
        super(message);
        this.name = 'RecommendationError';
    }
}

export class ColdStartError extends RecommendationError {
    constructor(message: string, userId: string) {
        super(message, 'COLD_START', { userId });
        this.name = 'ColdStartError';
    }
}

export class ValidationError extends RecommendationError {
    constructor(message: string, field: string, value: any) {
        super(message, 'VALIDATION_ERROR', { field, value });
        this.name = 'ValidationError';
    }
}

// ============================================================================
// Utility Types
// ============================================================================

export type AsyncResult<T> = Promise<T>;
export type Optional<T> = T | undefined;
export type Nullable<T> = T | null;

export interface Paginated<T> {
    items: T[];
    total: number;
    page: number;
    pageSize: number;
    hasMore: boolean;
}

// 弘益人間 (홍익인간) · Benefit All Humanity
