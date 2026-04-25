/**
 * WIA-AI-010 AI Safety Protocol - TypeScript SDK
 * 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive AI safety testing and monitoring for TypeScript/JavaScript applications.
 *
 * @example
 * ```typescript
 * import { SafetyProtocol } from '@wia/ai-safety-protocol';
 *
 * const safety = new SafetyProtocol({
 *   apiKey: process.env.WIA_API_KEY
 * });
 *
 * const result = await safety.test({
 *   input: "User input to test",
 *   testTypes: ['adversarial', 'toxicity']
 * });
 *
 * if (!result.safe) {
 *   console.log('Input blocked:', result.recommendations);
 * }
 * ```
 */

import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';

export * from './types';
import type {
    SafetyConfig,
    SafetyTestRequest,
    SafetyTestResult,
    AdversarialTestRequest,
    AdversarialTestResult,
    ContentFilterRequest,
    ContentFilterResult,
    AlignmentRequest,
    AlignmentResponse,
    BenchmarkRequest,
    BenchmarkResponse,
    BenchmarkReport,
    SafetyMetrics,
    Incident,
    GuardrailsConfig,
    APIResponse,
    SafetyProtocolError,
} from './types';

/**
 * Main SafetyProtocol client class
 */
export class SafetyProtocol {
    private client: AxiosInstance;
    private config: SafetyConfig;

    /**
     * Initialize the Safety Protocol client
     *
     * @param config - Configuration options
     */
    constructor(config: SafetyConfig) {
        this.config = {
            baseURL: 'https://api.wia.org/v1',
            timeout: 30000,
            strictness: 'balanced',
            retries: 3,
            ...config,
        };

        this.client = axios.create({
            baseURL: this.config.baseURL,
            timeout: this.config.timeout,
            headers: {
                'Authorization': `Bearer ${this.config.apiKey}`,
                'Content-Type': 'application/json',
                'User-Agent': '@wia/ai-safety-protocol/1.0.0',
            },
        });

        // Add response interceptor for error handling
        this.client.interceptors.response.use(
            response => response,
            error => {
                if (error.response?.data?.error) {
                    throw new SafetyProtocolError(error.response.data.error);
                }
                throw error;
            }
        );
    }

    // ========================================================================
    // Safety Testing
    // ========================================================================

    /**
     * Test input for safety violations
     *
     * @param request - Test request parameters
     * @returns Safety test results
     */
    async test(request: SafetyTestRequest): Promise<SafetyTestResult> {
        const response = await this.client.post<APIResponse<SafetyTestResult>>(
            '/safety/test',
            {
                ...request,
                config: {
                    strictness: this.config.strictness,
                    ...request.config,
                },
            }
        );
        return response.data.data!;
    }

    /**
     * Test multiple inputs in batch
     *
     * @param tests - Array of test inputs
     * @returns Array of test results
     */
    async testBatch(tests: Array<{id: string; input: string}>): Promise<SafetyTestResult[]> {
        const response = await this.client.post<APIResponse<{results: SafetyTestResult[]}>>(
            '/safety/test/batch',
            { tests, config: { strictness: this.config.strictness } }
        );
        return response.data.data!.results;
    }

    // ========================================================================
    // Adversarial Testing
    // ========================================================================

    /**
     * Generate adversarial examples
     *
     * @param request - Adversarial test parameters
     * @returns Generated adversarial examples
     */
    async generateAdversarial(request: AdversarialTestRequest): Promise<AdversarialTestResult> {
        const response = await this.client.post<APIResponse<AdversarialTestResult>>(
            '/adversarial/generate',
            request
        );
        return response.data.data!;
    }

    /**
     * Test model robustness against adversarial attacks
     *
     * @param modelEndpoint - Model API endpoint to test
     * @param testSet - Test dataset identifier
     * @returns Robustness report
     */
    async testRobustness(modelEndpoint: string, testSet: string): Promise<any> {
        const response = await this.client.post<APIResponse>(
            '/adversarial/test-robustness',
            { modelEndpoint, testSet }
        );
        return response.data.data!;
    }

    // ========================================================================
    // Content Filtering
    // ========================================================================

    /**
     * Filter content for safety violations
     *
     * @param request - Content filter parameters
     * @returns Filtering results
     */
    async filterContent(request: ContentFilterRequest): Promise<ContentFilterResult> {
        const response = await this.client.post<APIResponse<ContentFilterResult>>(
            '/content/filter',
            request
        );
        return response.data.data!;
    }

    // ========================================================================
    // Alignment Verification
    // ========================================================================

    /**
     * Verify AI response alignment with principles
     *
     * @param request - Alignment verification parameters
     * @returns Alignment assessment
     */
    async verifyAlignment(request: AlignmentRequest): Promise<AlignmentResponse> {
        const response = await this.client.post<APIResponse<AlignmentResponse>>(
            '/alignment/verify',
            request
        );
        return response.data.data!;
    }

    // ========================================================================
    // Guardrails
    // ========================================================================

    /**
     * Configure safety guardrails
     *
     * @param config - Guardrail configuration
     * @returns Configuration ID and status
     */
    async configureGuardrails(config: GuardrailsConfig): Promise<{configId: string; applied: boolean}> {
        const response = await this.client.post<APIResponse>(
            '/guardrails/configure',
            { guardrails: config }
        );
        return response.data.data!;
    }

    // ========================================================================
    // Benchmarking
    // ========================================================================

    /**
     * Run safety benchmark
     *
     * @param request - Benchmark parameters
     * @returns Benchmark job information
     */
    async runBenchmark(request: BenchmarkRequest): Promise<BenchmarkResponse> {
        const response = await this.client.post<APIResponse<BenchmarkResponse>>(
            '/benchmark/run',
            request
        );
        return response.data.data!;
    }

    /**
     * Get benchmark results
     *
     * @param benchmarkId - Benchmark job ID
     * @returns Benchmark report
     */
    async getBenchmarkResults(benchmarkId: string): Promise<BenchmarkReport> {
        const response = await this.client.get<APIResponse<BenchmarkReport>>(
            `/benchmark/results/${benchmarkId}`
        );
        return response.data.data!;
    }

    // ========================================================================
    // Monitoring
    // ========================================================================

    /**
     * Get safety metrics for a time range
     *
     * @param timeRange - Time range (e.g., '24h', '7d', '30d')
     * @returns Safety metrics
     */
    async getMetrics(timeRange: string = '24h'): Promise<SafetyMetrics> {
        const response = await this.client.get<APIResponse<SafetyMetrics>>(
            `/metrics/safety?timeRange=${timeRange}`
        );
        return response.data.data!;
    }

    /**
     * Get incidents
     *
     * @param status - Filter by status
     * @param severity - Filter by severity
     * @returns List of incidents
     */
    async getIncidents(status?: string, severity?: string): Promise<Incident[]> {
        const params = new URLSearchParams();
        if (status) params.append('status', status);
        if (severity) params.append('severity', severity);

        const response = await this.client.get<APIResponse<{incidents: Incident[]}>>(
            `/incidents?${params.toString()}`
        );
        return response.data.data!.incidents;
    }

    // ========================================================================
    // Utilities
    // ========================================================================

    /**
     * Wrap an existing AI client with safety checks
     *
     * @param client - Original AI client
     * @param options - Safety options
     * @returns Wrapped client with safety
     */
    static wrap<T extends object>(client: T, options: {
        inputGuardrails?: string[];
        outputGuardrails?: string[];
        monitoring?: {enabled: boolean; sampleRate?: number};
    } = {}): T {
        const safety = new SafetyProtocol({
            apiKey: process.env.WIA_API_KEY || '',
        });

        return new Proxy(client, {
            get(target, prop) {
                const original = (target as any)[prop];
                if (typeof original === 'function') {
                    return async function(...args: any[]) {
                        // Pre-check input if guardrails enabled
                        if (options.inputGuardrails && options.inputGuardrails.length > 0) {
                            const inputCheck = await safety.test({
                                input: JSON.stringify(args),
                                testTypes: options.inputGuardrails as any[],
                            });
                            if (!inputCheck.safe) {
                                throw new Error(`Input blocked by safety guardrails: ${inputCheck.recommendations}`);
                            }
                        }

                        // Call original function
                        const result = await original.apply(target, args);

                        // Post-check output if guardrails enabled
                        if (options.outputGuardrails && options.outputGuardrails.length > 0) {
                            const outputCheck = await safety.test({
                                input: JSON.stringify(result),
                                testTypes: options.outputGuardrails as any[],
                            });
                            if (!outputCheck.safe) {
                                throw new Error(`Output blocked by safety guardrails: ${outputCheck.recommendations}`);
                            }
                        }

                        return result;
                    };
                }
                return original;
            },
        });
    }
}

/**
 * Default export for convenience
 */
export default SafetyProtocol;

/**
 * 弘益人間 (Benefit All Humanity)
 * Through this SDK, we enable developers worldwide to build safer AI systems.
 */
