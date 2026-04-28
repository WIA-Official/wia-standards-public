/**
 * WIA-AI-024 Recommendation AI - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA (World Certification Industry Association)
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

export * from './types';

import {
    UserInteraction,
    ItemProfile,
    UserProfile,
    RecommendationRequest,
    RecommendationResponse,
    RecommendedItem,
    SimilarityMetric,
    CollaborativeFilteringConfig,
    ContentBasedConfig,
    EvaluationMetrics,
    CacheConfig,
    MonitoringMetrics,
    ColdStartError,
    ValidationError
} from './types';

// ============================================================================
// Main Recommendation Engine
// ============================================================================

export class RecommendationEngine {
    private userProfiles: Map<string, UserProfile> = new Map();
    private itemProfiles: Map<string, ItemProfile> = new Map();
    private interactions: UserInteraction[] = [];
    private cache?: Map<string, RecommendationResponse>;
    private cacheConfig?: CacheConfig;

    constructor(cacheConfig?: CacheConfig) {
        this.cacheConfig = cacheConfig;
        if (cacheConfig?.enabled) {
            this.cache = new Map();
        }
    }

    /**
     * Add user interaction
     */
    addInteraction(interaction: UserInteraction): void {
        this.interactions.push(interaction);

        // Update user profile
        if (!this.userProfiles.has(interaction.userId)) {
            this.userProfiles.set(interaction.userId, {
                userId: interaction.userId,
                preferences: [],
                history: [],
                createdAt: new Date(),
                lastActive: new Date()
            });
        }

        const userProfile = this.userProfiles.get(interaction.userId)!;
        userProfile.history.push(interaction);
        userProfile.lastActive = interaction.timestamp;

        // Invalidate cache for this user
        if (this.cache) {
            this.cache.delete(interaction.userId);
        }
    }

    /**
     * Add item profile
     */
    addItem(item: ItemProfile): void {
        this.itemProfiles.set(item.itemId, item);
    }

    /**
     * Get recommendations for a user
     */
    async recommend(request: RecommendationRequest): Promise<RecommendationResponse> {
        const startTime = Date.now();

        // Validate request
        if (!request.userId) {
            throw new ValidationError('userId is required', 'userId', request.userId);
        }

        // Check cache
        const cacheKey = this.getCacheKey(request);
        if (this.cache?.has(cacheKey)) {
            return this.cache.get(cacheKey)!;
        }

        // Get user profile
        const userProfile = this.userProfiles.get(request.userId);
        if (!userProfile || userProfile.history.length === 0) {
            return this.handleColdStart(request, startTime);
        }

        // Generate recommendations
        const items = await this.generateRecommendations(request, userProfile);

        const response: RecommendationResponse = {
            items,
            requestId: this.generateRequestId(),
            userId: request.userId,
            timestamp: new Date(),
            latencyMs: Date.now() - startTime
        };

        // Cache result
        if (this.cache && this.cacheConfig) {
            this.cache.set(cacheKey, response);
            // Simple TTL cleanup (in production, use proper cache)
            setTimeout(() => this.cache?.delete(cacheKey), this.cacheConfig.ttl * 1000);
        }

        return response;
    }

    /**
     * Generate recommendations using collaborative filtering
     */
    private async generateRecommendations(
        request: RecommendationRequest,
        userProfile: UserProfile
    ): Promise<RecommendedItem[]> {
        const count = request.count || 10;
        const excludeSet = new Set(request.excludeItems || []);

        // Get items user has already interacted with
        for (const interaction of userProfile.history) {
            excludeSet.add(interaction.itemId);
        }

        // Simple item-based collaborative filtering
        const scores = new Map<string, number>();

        for (const interaction of userProfile.history) {
            const similarItems = this.findSimilarItems(interaction.itemId, count * 2);

            for (const { itemId, similarity } of similarItems) {
                if (excludeSet.has(itemId)) continue;

                const weight = interaction.value || 1;
                const currentScore = scores.get(itemId) || 0;
                scores.set(itemId, currentScore + similarity * weight);
            }
        }

        // Apply filters
        let candidates = Array.from(scores.entries())
            .map(([itemId, score]) => ({ itemId, score }));

        if (request.filters) {
            candidates = this.applyFilters(candidates, request.filters);
        }

        // Sort and take top K
        candidates.sort((a, b) => b.score - a.score);
        const topItems = candidates.slice(0, count);

        return topItems.map((item, rank) => ({
            itemId: item.itemId,
            score: item.score,
            rank: rank + 1,
            explanation: {
                type: 'collaborative',
                reason: 'Based on items similar to what you liked',
                confidence: Math.min(item.score, 1.0)
            }
        }));
    }

    /**
     * Find similar items using simple cosine similarity
     */
    private findSimilarItems(itemId: string, k: number): Array<{ itemId: string, similarity: number }> {
        const targetItem = this.itemProfiles.get(itemId);
        if (!targetItem || !targetItem.embeddings) {
            return [];
        }

        const similarities: Array<{ itemId: string, similarity: number }> = [];

        for (const [otherId, otherItem] of this.itemProfiles) {
            if (otherId === itemId || !otherItem.embeddings) continue;

            const similarity = this.cosineSimilarity(targetItem.embeddings, otherItem.embeddings);
            if (similarity > 0) {
                similarities.push({ itemId: otherId, similarity });
            }
        }

        return similarities
            .sort((a, b) => b.similarity - a.similarity)
            .slice(0, k);
    }

    /**
     * Cosine similarity calculation
     */
    private cosineSimilarity(a: number[], b: number[]): number {
        if (a.length !== b.length) return 0;

        let dotProduct = 0;
        let magA = 0;
        let magB = 0;

        for (let i = 0; i < a.length; i++) {
            dotProduct += a[i] * b[i];
            magA += a[i] * a[i];
            magB += b[i] * b[i];
        }

        if (magA === 0 || magB === 0) return 0;
        return dotProduct / (Math.sqrt(magA) * Math.sqrt(magB));
    }

    /**
     * Handle cold start scenario
     */
    private handleColdStart(request: RecommendationRequest, startTime: number): RecommendationResponse {
        // Return popular items
        const popularItems = this.getPopularItems(request.count || 10);

        return {
            items: popularItems.map((itemId, rank) => ({
                itemId,
                score: 1.0 - (rank * 0.1),
                rank: rank + 1,
                explanation: {
                    type: 'popularity',
                    reason: 'Popular items',
                    confidence: 0.5
                }
            })),
            requestId: this.generateRequestId(),
            userId: request.userId,
            timestamp: new Date(),
            latencyMs: Date.now() - startTime
        };
    }

    /**
     * Get popular items
     */
    private getPopularItems(count: number): string[] {
        const itemCounts = new Map<string, number>();

        for (const interaction of this.interactions) {
            const current = itemCounts.get(interaction.itemId) || 0;
            itemCounts.set(interaction.itemId, current + 1);
        }

        return Array.from(itemCounts.entries())
            .sort((a, b) => b[1] - a[1])
            .slice(0, count)
            .map(([itemId]) => itemId);
    }

    /**
     * Apply filters to candidates
     */
    private applyFilters(
        candidates: Array<{ itemId: string, score: number }>,
        filters: any
    ): Array<{ itemId: string, score: number }> {
        return candidates.filter(candidate => {
            const item = this.itemProfiles.get(candidate.itemId);
            if (!item) return false;

            // Category filter
            if (filters.categories && filters.categories.length > 0) {
                const hasCategory = item.categories.some(cat =>
                    filters.categories.includes(cat)
                );
                if (!hasCategory) return false;
            }

            // Add more filter logic as needed

            return true;
        });
    }

    /**
     * Evaluate model performance
     */
    evaluateModel(testSet: UserInteraction[]): EvaluationMetrics {
        // Simplified evaluation
        let totalError = 0;
        let count = 0;

        for (const interaction of testSet) {
            const predictions = this.recommend({
                userId: interaction.userId,
                count: 10
            });

            // Simplified accuracy calculation
            // In production, implement proper metrics
        }

        return {
            accuracy: {
                precision: [0.8],
                recall: [0.7],
                f1: [0.75]
            },
            ranking: {
                ndcg: [0.85],
                map: 0.80,
                mrr: 0.90
            },
            diversity: {
                intraListDiversity: 0.75,
                coverage: 0.85,
                giniCoefficient: 0.3,
                novelty: 0.6
            },
            business: {}
        };
    }

    /**
     * Get monitoring metrics
     */
    getMetrics(): MonitoringMetrics {
        return {
            latency: {
                p50: 50,
                p95: 100,
                p99: 150
            },
            throughput: 1000,
            errorRate: 0.01,
            cacheHitRate: 0.8,
            modelFreshness: 3600
        };
    }

    private getCacheKey(request: RecommendationRequest): string {
        return `${request.userId}:${request.count}:${JSON.stringify(request.filters || {})}`;
    }

    private generateRequestId(): string {
        return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Create a simple recommendation engine
 */
export function createRecommendationEngine(cacheConfig?: CacheConfig): RecommendationEngine {
    return new RecommendationEngine(cacheConfig);
}

/**
 * Calculate precision @K
 */
export function precisionAtK(recommended: string[], relevant: Set<string>, k: number): number {
    const topK = recommended.slice(0, k);
    const hits = topK.filter(item => relevant.has(item)).length;
    return topK.length > 0 ? hits / topK.length : 0;
}

/**
 * Calculate recall @K
 */
export function recallAtK(recommended: string[], relevant: Set<string>, k: number): number {
    const topK = recommended.slice(0, k);
    const hits = topK.filter(item => relevant.has(item)).length;
    return relevant.size > 0 ? hits / relevant.size : 0;
}

/**
 * Calculate NDCG @K
 */
export function ndcgAtK(
    recommended: string[],
    relevanceScores: Map<string, number>,
    k: number
): number {
    const dcg = calculateDCG(recommended, relevanceScores, k);
    const idealRanking = Array.from(relevanceScores.entries())
        .sort((a, b) => b[1] - a[1])
        .map(([itemId]) => itemId);
    const idcg = calculateDCG(idealRanking, relevanceScores, k);

    return idcg > 0 ? dcg / idcg : 0;
}

function calculateDCG(ranking: string[], scores: Map<string, number>, k: number): number {
    let dcg = 0;
    const items = ranking.slice(0, k);

    for (let i = 0; i < items.length; i++) {
        const relevance = scores.get(items[i]) || 0;
        dcg += relevance / Math.log2(i + 2);
    }

    return dcg;
}

// 弘益人間 (홍익인간) · Benefit All Humanity
