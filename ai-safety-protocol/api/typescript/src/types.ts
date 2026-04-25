/**
 * WIA-AI-010 AI Safety Protocol - TypeScript Type Definitions
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export type Severity = 'critical' | 'high' | 'medium' | 'low';
export type Strictness = 'strict' | 'balanced' | 'permissive';
export type TestCategory = 'adversarial' | 'content' | 'alignment' | 'privacy' | 'bias' | 'robustness';

// ============================================================================
// Configuration
// ============================================================================

export interface SafetyConfig {
    apiKey?: string;
    baseURL?: string;
    timeout?: number;
    strictness?: Strictness;
    retries?: number;
}

export interface GuardrailConfig {
    enabled: boolean;
    threshold?: number;
    action?: 'block' | 'filter' | 'warn' | 'log';
}

export interface GuardrailsConfig {
    input?: {
        promptInjection?: GuardrailConfig;
        toxicity?: GuardrailConfig;
        pii?: GuardrailConfig;
    };
    output?: {
        contentFilter?: GuardrailConfig;
        piiRedaction?: GuardrailConfig;
        toxicity?: GuardrailConfig;
    };
    rateLimiting?: {
        requestsPerHour?: number;
        requestsPerDay?: number;
    };
    monitoring?: {
        enabled?: boolean;
        sampleRate?: number;
    };
}

// ============================================================================
// Safety Testing
// ============================================================================

export interface SafetyTestRequest {
    input: string;
    testTypes?: TestCategory[];
    config?: {
        strictness?: Strictness;
        returnDetails?: boolean;
    };
}

export interface SafetyTestResult {
    safe: boolean;
    overallScore: number;
    results: {
        adversarial?: TestCategoryResult;
        toxicity?: ToxicityResult;
        bias?: BiasResult;
        privacy?: PrivacyResult;
        alignment?: AlignmentResult;
        robustness?: RobustnessResult;
    };
    recommendations?: string[];
    processingTimeMs: number;
}

export interface TestCategoryResult {
    score: number;
    passed: boolean;
    details?: any;
}

export interface ToxicityResult extends TestCategoryResult {
    categories: {
        hate?: number;
        violence?: number;
        sexual?: number;
        harassment?: number;
        selfHarm?: number;
    };
}

export interface BiasResult extends TestCategoryResult {
    fairnessMetrics: {
        demographicParity?: number;
        equalizedOdds?: number;
        predictiveParity?: number;
    };
}

export interface PrivacyResult extends TestCategoryResult {
    piiDetected: boolean;
    piiTypes?: string[];
    redacted?: string;
}

export interface AlignmentResult extends TestCategoryResult {
    principles: {
        helpful?: number;
        harmless?: number;
        honest?: number;
    };
    violations?: string[];
}

export interface RobustnessResult extends TestCategoryResult {
    cleanAccuracy?: number;
    adversarialAccuracy?: number;
    perturbationSensitivity?: number;
}

// ============================================================================
// Adversarial Testing
// ============================================================================

export type AttackType = 'fgsm' | 'pgd' | 'carlini-wagner' | 'deepfool' | 'prompt-injection';

export interface AdversarialTestRequest {
    input: string;
    attackTypes?: AttackType[];
    epsilon?: number;
    targetModel?: string;
}

export interface AdversarialExample {
    attackType: AttackType;
    perturbedInput: string;
    perturbationNorm: number;
    successfulAttack: boolean;
}

export interface AdversarialTestResult {
    adversarialExamples: AdversarialExample[];
    robustnessScore: number;
}

// ============================================================================
// Content Filtering
// ============================================================================

export interface ContentFilterRequest {
    content: string;
    categories?: string[];
    threshold?: number;
    redactPII?: boolean;
}

export interface ContentFilterResult {
    allowed: boolean;
    filtered: boolean;
    scores: {
        toxicity?: number;
        hate?: number;
        violence?: number;
        sexual?: number;
        [key: string]: number | undefined;
    };
    piiDetections: PIIDetection[];
    filteredContent?: string;
    reason?: string;
}

export interface PIIDetection {
    type: string;
    value: string;
    position: {start: number; end: number};
    confidence: number;
}

// ============================================================================
// Alignment Verification
// ============================================================================

export interface AlignmentRequest {
    prompt: string;
    response: string;
    principles?: string[];
    context?: Record<string, any>;
}

export interface AlignmentResponse {
    aligned: boolean;
    scores: Record<string, number>;
    violations: string[];
    recommendations: string[];
}

// ============================================================================
// Benchmarking
// ============================================================================

export type BenchmarkTier = 1 | 2 | 3;

export interface BenchmarkRequest {
    benchmarkName: string;
    tier?: BenchmarkTier;
    modelEndpoint: string;
    config?: {
        parallel?: boolean;
        maxConcurrency?: number;
    };
}

export interface BenchmarkResponse {
    benchmarkId: string;
    status: 'queued' | 'running' | 'completed' | 'failed';
    estimatedCompletionTime?: string;
    progress?: number;
    resultsUrl?: string;
}

export interface BenchmarkReport {
    benchmarkId: string;
    benchmarkName: string;
    tier: BenchmarkTier;
    timestamp: string;
    system: {
        name: string;
        version: string;
        type: string;
    };
    summary: {
        totalTests: number;
        passed: number;
        failed: number;
        overallScore: number;
        certification: 'certified' | 'not-certified';
    };
    categoryScores: {
        robustness: number;
        alignment: number;
        toxicity: number;
        bias: number;
        truthfulness: number;
        privacy: number;
        compliance?: number;
    };
    detailedResults?: any[];
}

// ============================================================================
// Monitoring
// ============================================================================

export interface SafetyMetrics {
    timeRange: {
        start: string;
        end: string;
    };
    metrics: {
        totalRequests: number;
        blockedRequests: number;
        blockRate: number;
        safetyScore: number;
        categories: Record<string, {
            violations: number;
            rate: number;
        }>;
        latency: {
            p50: number;
            p95: number;
            p99: number;
        };
    };
}

// ============================================================================
// Incidents
// ============================================================================

export interface Incident {
    id: string;
    severity: Severity;
    status: 'open' | 'investigating' | 'mitigated' | 'resolved' | 'closed';
    occurred: string;
    detected: string;
    resolved?: string;
    category: string;
    summary: string;
    description?: {
        what: string;
        when: string;
        where: string;
        how?: string;
        impact: string;
    };
}

// ============================================================================
// Error Types
// ============================================================================

export interface SafetyError {
    code: string;
    message: string;
    details?: any;
    requestId?: string;
    documentation?: string;
}

export class SafetyProtocolError extends Error {
    code: string;
    details?: any;
    requestId?: string;

    constructor(error: SafetyError) {
        super(error.message);
        this.name = 'SafetyProtocolError';
        this.code = error.code;
        this.details = error.details;
        this.requestId = error.requestId;
    }
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T = any> {
    data?: T;
    error?: SafetyError;
    requestId: string;
    timestamp: string;
}

// ============================================================================
// Webhook Types
// ============================================================================

export interface WebhookConfig {
    url: string;
    events: string[];
    secret?: string;
}

export interface WebhookPayload {
    event: string;
    timestamp: string;
    data: any;
    signature?: string;
}
