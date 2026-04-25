/**
 * WIA-CORE-008: Universal Metadata SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for creating, validating,
 * enriching, and managing universal metadata across all domains.
 */

import {
  UniversalMetadata,
  ValidationResult,
  ValidationError,
  QualityMetrics,
  EnrichmentOptions,
  EnrichmentResult,
  MetadataQuery,
  MetadataSearchResult,
  MetadataTemplate,
  MetadataDomain,
  DataType,
  Agent,
  METADATA_CONSTANTS,
  MetadataErrorCode,
  MetadataError,
  MetadataUpdate,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-008 Universal Metadata SDK
 */
export class UniversalMetadataSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create universal metadata
   *
   * @param params - Metadata parameters
   * @returns Universal metadata object
   */
  createMetadata(params: Partial<UniversalMetadata>): UniversalMetadata {
    // Generate ID if not provided
    const id = params.id || this.generateId();

    // Validate required fields
    if (!params.title) {
      throw new MetadataError(
        MetadataErrorCode.MISSING_REQUIRED_FIELD,
        'Title is required',
        'title'
      );
    }

    // Get current timestamp
    const now = new Date().toISOString();

    // Create metadata with defaults
    const metadata: UniversalMetadata = {
      id,
      title: params.title,
      created: params.created || now,
      modified: now,
      schemaVersion: METADATA_CONSTANTS.SCHEMA_VERSION,
      privacy: params.privacy || METADATA_CONSTANTS.DEFAULT_PRIVACY,
      validationStatus: 'not-validated',
      ...params,
    };

    return metadata;
  }

  /**
   * Validate metadata
   *
   * @param metadata - Metadata to validate
   * @returns Validation result
   */
  validateMetadata(metadata: UniversalMetadata): ValidationResult {
    const errors: ValidationError[] = [];
    const warnings: string[] = [];
    const suggestions: string[] = [];
    const fieldCompleteness: Record<string, boolean> = {};

    // Check required fields
    for (const field of METADATA_CONSTANTS.REQUIRED_FIELDS) {
      const hasField = metadata[field as keyof UniversalMetadata] !== undefined;
      fieldCompleteness[field] = hasField;

      if (!hasField) {
        errors.push({
          field,
          code: MetadataErrorCode.MISSING_REQUIRED_FIELD,
          message: `Required field '${field}' is missing`,
          severity: 'error',
        });
      }
    }

    // Check recommended fields
    for (const field of METADATA_CONSTANTS.RECOMMENDED_FIELDS) {
      const hasField = metadata[field as keyof UniversalMetadata] !== undefined;
      fieldCompleteness[field] = hasField;

      if (!hasField) {
        warnings.push(`Recommended field '${field}' is missing`);
        suggestions.push(`Add '${field}' to improve metadata quality`);
      }
    }

    // Validate field lengths
    if (metadata.title && metadata.title.length > METADATA_CONSTANTS.MAX_LENGTHS.title) {
      errors.push({
        field: 'title',
        code: MetadataErrorCode.INVALID_FIELD_VALUE,
        message: `Title exceeds maximum length of ${METADATA_CONSTANTS.MAX_LENGTHS.title}`,
        severity: 'error',
      });
    }

    if (metadata.description && metadata.description.length > METADATA_CONSTANTS.MAX_LENGTHS.description) {
      warnings.push(`Description exceeds recommended length of ${METADATA_CONSTANTS.MAX_LENGTHS.description}`);
    }

    // Validate keywords
    if (metadata.keywords) {
      metadata.keywords.forEach((keyword, index) => {
        if (keyword.length > METADATA_CONSTANTS.MAX_LENGTHS.keyword) {
          errors.push({
            field: `keywords[${index}]`,
            code: MetadataErrorCode.INVALID_FIELD_VALUE,
            message: `Keyword exceeds maximum length of ${METADATA_CONSTANTS.MAX_LENGTHS.keyword}`,
            severity: 'warning',
          });
        }
      });
    }

    // Calculate quality metrics
    const qualityMetrics = this.calculateQuality(metadata);
    const discoverabilityScore = this.calculateDiscoverability(metadata);

    // Add suggestions based on quality
    if (qualityMetrics.completeness < 60) {
      suggestions.push('Add more descriptive fields to improve completeness');
    }

    if (!metadata.keywords || metadata.keywords.length < 3) {
      suggestions.push('Add at least 3 keywords to improve discoverability');
    }

    if (!metadata.description) {
      suggestions.push('Add a description to help users understand the data');
    }

    return {
      valid: errors.filter(e => e.severity === 'error').length === 0,
      qualityScore: qualityMetrics.overallScore,
      completeness: qualityMetrics.completeness,
      discoverabilityScore,
      errors,
      warnings,
      suggestions,
      fieldCompleteness,
    };
  }

  /**
   * Calculate quality metrics
   *
   * @param metadata - Metadata to analyze
   * @returns Quality metrics
   */
  calculateQuality(metadata: UniversalMetadata): QualityMetrics {
    // Calculate completeness
    const allFields = [
      ...METADATA_CONSTANTS.REQUIRED_FIELDS,
      ...METADATA_CONSTANTS.RECOMMENDED_FIELDS,
      'keywords', 'tags', 'license', 'version',
    ];

    const presentFields = allFields.filter(
      field => metadata[field as keyof UniversalMetadata] !== undefined
    );

    const completeness = (presentFields.length / allFields.length) * 100;

    // Calculate accuracy (simplified - checks for valid formats)
    let accuracy = 100;
    const invalidFields: string[] = [];

    if (metadata.created && !this.isValidDate(metadata.created)) {
      accuracy -= 10;
      invalidFields.push('created');
    }

    if (metadata.modified && !this.isValidDate(metadata.modified)) {
      accuracy -= 10;
      invalidFields.push('modified');
    }

    if (metadata.size && metadata.size < 0) {
      accuracy -= 10;
      invalidFields.push('size');
    }

    // Calculate consistency
    let consistency = 100;

    if (metadata.created && metadata.modified) {
      if (new Date(metadata.created) > new Date(metadata.modified)) {
        consistency -= 20;
      }
    }

    if (metadata.temporal?.start && metadata.temporal?.end) {
      if (new Date(metadata.temporal.start) > new Date(metadata.temporal.end)) {
        consistency -= 20;
      }
    }

    // Calculate timeliness
    let timeliness = 100;

    if (metadata.modified) {
      const daysSinceUpdate = this.getDaysSince(metadata.modified);
      if (daysSinceUpdate > 365) {
        timeliness = 50;
      } else if (daysSinceUpdate > 180) {
        timeliness = 75;
      }
    }

    // Calculate discoverability
    const discoverability = this.calculateDiscoverability(metadata);

    // Overall score (weighted average)
    const overallScore =
      completeness * 0.4 +
      accuracy * 0.3 +
      consistency * 0.2 +
      timeliness * 0.1;

    // Find missing required fields
    const missingFields = METADATA_CONSTANTS.REQUIRED_FIELDS.filter(
      field => metadata[field as keyof UniversalMetadata] === undefined
    );

    return {
      overallScore: Math.round(overallScore),
      completeness: Math.round(completeness),
      accuracy: Math.round(accuracy),
      consistency: Math.round(consistency),
      timeliness: Math.round(timeliness),
      discoverability: Math.round(discoverability),
      missingFields,
      invalidFields,
    };
  }

  /**
   * Calculate discoverability score
   *
   * @param metadata - Metadata to analyze
   * @returns Discoverability score
   */
  calculateDiscoverability(metadata: UniversalMetadata): number {
    let score = 0;

    // Keywords contribution
    if (metadata.keywords && metadata.keywords.length > 0) {
      score += Math.log2(1 + metadata.keywords.length) * 8;
    }

    // Tags contribution
    if (metadata.tags && metadata.tags.length > 0) {
      score += Math.log2(1 + metadata.tags.length) * 6;
    }

    // Description bonus
    if (metadata.description) {
      score += 20;
    }

    // Abstract bonus
    if (metadata.abstract) {
      score += 15;
    }

    // Subjects contribution
    if (metadata.subjects && metadata.subjects.length > 0) {
      score += Math.log2(1 + metadata.subjects.length) * 5;
    }

    // Identifiers bonus
    if (metadata.identifiers && Object.keys(metadata.identifiers).length > 0) {
      score += 10;
    }

    return Math.min(100, score);
  }

  /**
   * Enrich metadata with auto-classification and enhancement
   *
   * @param metadata - Metadata to enrich
   * @param options - Enrichment options
   * @returns Enriched metadata
   */
  enrichMetadata(
    metadata: UniversalMetadata,
    options: EnrichmentOptions = {}
  ): EnrichmentResult {
    const enriched = { ...metadata };
    const addedFields: string[] = [];
    const enhancedFields: string[] = [];
    const beforeQuality = this.calculateQuality(metadata).overallScore;

    // Auto-classify domain if not set
    if (options.autoClassify && !enriched.domain) {
      enriched.domain = this.classifyDomain(metadata);
      addedFields.push('domain');
    }

    // Extract keywords from title and description
    if (options.extractKeywords && !enriched.keywords) {
      enriched.keywords = this.extractKeywords(metadata);
      addedFields.push('keywords');
    } else if (options.extractKeywords && enriched.keywords) {
      const extracted = this.extractKeywords(metadata);
      enriched.keywords = [...new Set([...enriched.keywords, ...extracted])];
      enhancedFields.push('keywords');
    }

    // Generate tags
    if (options.generateTags) {
      const tags = this.generateTags(enriched);
      enriched.tags = tags;
      if (metadata.tags) {
        enhancedFields.push('tags');
      } else {
        addedFields.push('tags');
      }
    }

    // Enhance description
    if (options.enhanceDescription && !enriched.description && enriched.title) {
      enriched.description = `Data resource: ${enriched.title}`;
      addedFields.push('description');
    }

    // Calculate quality
    if (options.calculateQuality || !enriched.qualityScore) {
      const quality = this.calculateQuality(enriched);
      enriched.qualityScore = quality.overallScore;
      enriched.completeness = quality.completeness;
      addedFields.push('qualityScore', 'completeness');
    }

    // Update modification timestamp
    enriched.modified = new Date().toISOString();
    enriched.lastValidated = new Date().toISOString();

    const afterQuality = this.calculateQuality(enriched).overallScore;
    const qualityImprovement = afterQuality - beforeQuality;

    return {
      metadata: enriched,
      addedFields,
      enhancedFields,
      qualityImprovement,
      confidence: 0.85, // Confidence in enrichment
    };
  }

  /**
   * Search metadata
   *
   * @param query - Search query
   * @param metadataList - List of metadata to search
   * @returns Search results
   */
  searchMetadata(
    query: MetadataQuery,
    metadataList: UniversalMetadata[]
  ): MetadataSearchResult {
    const startTime = Date.now();
    let results = [...metadataList];

    // Filter by domain
    if (query.domain) {
      results = results.filter(m => m.domain === query.domain);
    }

    // Filter by data type
    if (query.dataType) {
      results = results.filter(m => m.dataType === query.dataType);
    }

    // Filter by creator
    if (query.creator) {
      results = results.filter(m => {
        if (!m.creator) return false;
        const creators = Array.isArray(m.creator) ? m.creator : [m.creator];
        return creators.some(c => c.name.toLowerCase().includes(query.creator!.toLowerCase()));
      });
    }

    // Filter by keywords
    if (query.keywords && query.keywords.length > 0) {
      results = results.filter(m => {
        if (!m.keywords) return false;
        return query.keywords!.some(k =>
          m.keywords!.some(mk => mk.toLowerCase().includes(k.toLowerCase()))
        );
      });
    }

    // Filter by date range
    if (query.dateRange) {
      results = results.filter(m => {
        if (!m.created) return false;
        const created = new Date(m.created);
        const start = new Date(query.dateRange!.start);
        const end = new Date(query.dateRange!.end);
        return created >= start && created <= end;
      });
    }

    // Filter by privacy
    if (query.privacy && query.privacy.length > 0) {
      results = results.filter(m => m.privacy && query.privacy!.includes(m.privacy));
    }

    // Text search
    if (query.query) {
      const searchTerm = query.query.toLowerCase();
      results = results.filter(m =>
        m.title.toLowerCase().includes(searchTerm) ||
        m.description?.toLowerCase().includes(searchTerm) ||
        m.keywords?.some(k => k.toLowerCase().includes(searchTerm))
      );
    }

    // Sort
    if (query.sortBy) {
      results.sort((a, b) => {
        const aVal = a[query.sortBy as keyof UniversalMetadata];
        const bVal = b[query.sortBy as keyof UniversalMetadata];
        const order = query.sortOrder === 'desc' ? -1 : 1;
        return aVal > bVal ? order : -order;
      });
    }

    // Calculate facets
    const facets = this.calculateFacets(results);

    // Pagination
    const total = results.length;
    const offset = query.offset || 0;
    const limit = query.limit || 10;
    results = results.slice(offset, offset + limit);

    const queryTime = Date.now() - startTime;

    return {
      total,
      results,
      facets,
      queryTime,
    };
  }

  /**
   * Generate metadata template for a domain
   *
   * @param domain - Metadata domain
   * @returns Metadata template
   */
  generateTemplate(domain: MetadataDomain): MetadataTemplate {
    const templates: Record<MetadataDomain, Partial<MetadataTemplate>> = {
      healthcare: {
        requiredFields: ['id', 'title', 'created', 'privacy'],
        optionalFields: ['patientId', 'mrn', 'diagnosisCodes', 'phi'],
      },
      finance: {
        requiredFields: ['id', 'title', 'created', 'privacy'],
        optionalFields: ['transactionId', 'amount', 'currency'],
      },
      science: {
        requiredFields: ['id', 'title', 'creator', 'created'],
        optionalFields: ['experimentId', 'method', 'hypothesis', 'publicationDoi'],
      },
      iot: {
        requiredFields: ['id', 'title', 'created', 'deviceId'],
        optionalFields: ['sensorType', 'unit', 'samplingRate'],
      },
      media: {
        requiredFields: ['id', 'title', 'format', 'created'],
        optionalFields: ['dimensions', 'duration', 'codec'],
      },
      general: {
        requiredFields: METADATA_CONSTANTS.REQUIRED_FIELDS,
        optionalFields: METADATA_CONSTANTS.RECOMMENDED_FIELDS,
      },
      government: {
        requiredFields: ['id', 'title', 'created', 'privacy'],
        optionalFields: ['classification', 'agency'],
      },
      education: {
        requiredFields: ['id', 'title', 'creator', 'created'],
        optionalFields: ['course', 'institution', 'level'],
      },
      geospatial: {
        requiredFields: ['id', 'title', 'spatial', 'created'],
        optionalFields: ['coordinates', 'boundingBox', 'crs'],
      },
      biomedical: {
        requiredFields: ['id', 'title', 'created', 'privacy'],
        optionalFields: ['experimentId', 'method', 'sampleSize'],
      },
      manufacturing: {
        requiredFields: ['id', 'title', 'created'],
        optionalFields: ['productId', 'batchNumber', 'facility'],
      },
      transportation: {
        requiredFields: ['id', 'title', 'created'],
        optionalFields: ['vehicleId', 'route', 'schedule'],
      },
      energy: {
        requiredFields: ['id', 'title', 'created'],
        optionalFields: ['facilityId', 'energyType', 'capacity'],
      },
      agriculture: {
        requiredFields: ['id', 'title', 'created', 'spatial'],
        optionalFields: ['cropType', 'season', 'yield'],
      },
      custom: {
        requiredFields: METADATA_CONSTANTS.REQUIRED_FIELDS,
        optionalFields: [],
      },
    };

    const template = templates[domain] || templates.general;

    return {
      id: `template-${domain}`,
      name: `${domain} Metadata Template`,
      description: `Standard metadata template for ${domain} domain`,
      domain,
      requiredFields: template.requiredFields || METADATA_CONSTANTS.REQUIRED_FIELDS,
      optionalFields: template.optionalFields || [],
      defaults: {
        domain,
        schemaVersion: METADATA_CONSTANTS.SCHEMA_VERSION,
      },
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Generate unique ID
   */
  private generateId(): string {
    return `wia-metadata-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Validate date format
   */
  private isValidDate(dateString: string): boolean {
    const date = new Date(dateString);
    return !isNaN(date.getTime());
  }

  /**
   * Get days since date
   */
  private getDaysSince(dateString: string): number {
    const date = new Date(dateString);
    const now = new Date();
    const diff = now.getTime() - date.getTime();
    return Math.floor(diff / (1000 * 60 * 60 * 24));
  }

  /**
   * Classify domain based on content
   */
  private classifyDomain(metadata: UniversalMetadata): MetadataDomain {
    const text = `${metadata.title} ${metadata.description || ''} ${metadata.keywords?.join(' ') || ''}`.toLowerCase();

    const domainKeywords: Record<MetadataDomain, string[]> = {
      healthcare: ['patient', 'medical', 'health', 'clinical', 'diagnosis', 'treatment'],
      finance: ['transaction', 'payment', 'account', 'financial', 'currency', 'trading'],
      science: ['research', 'experiment', 'study', 'hypothesis', 'data', 'analysis'],
      iot: ['sensor', 'device', 'telemetry', 'iot', 'monitoring', 'measurement'],
      media: ['image', 'video', 'audio', 'photo', 'picture', 'recording'],
      government: ['government', 'public', 'policy', 'regulation', 'agency'],
      education: ['education', 'learning', 'course', 'school', 'university'],
      geospatial: ['map', 'gis', 'location', 'coordinates', 'spatial', 'geographic'],
      biomedical: ['gene', 'protein', 'cell', 'biology', 'genomics', 'biomedical'],
      manufacturing: ['production', 'manufacturing', 'assembly', 'quality'],
      transportation: ['vehicle', 'transport', 'route', 'logistics', 'shipping'],
      energy: ['energy', 'power', 'electricity', 'renewable', 'grid'],
      agriculture: ['farm', 'crop', 'agriculture', 'harvest', 'soil'],
      general: [],
      custom: [],
    };

    for (const [domain, keywords] of Object.entries(domainKeywords)) {
      if (keywords.some(kw => text.includes(kw))) {
        return domain as MetadataDomain;
      }
    }

    return 'general';
  }

  /**
   * Extract keywords from text
   */
  private extractKeywords(metadata: UniversalMetadata): string[] {
    const text = `${metadata.title} ${metadata.description || ''}`;
    const words = text.toLowerCase().split(/\s+/);

    // Simple keyword extraction (remove common words)
    const stopwords = new Set(['the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by']);
    const keywords = words
      .filter(w => w.length > 3 && !stopwords.has(w))
      .filter((w, i, arr) => arr.indexOf(w) === i) // unique
      .slice(0, 10);

    return keywords;
  }

  /**
   * Generate tags based on metadata
   */
  private generateTags(metadata: UniversalMetadata): string[] {
    const tags = new Set<string>();

    if (metadata.domain) {
      tags.add(metadata.domain);
    }

    if (metadata.dataType) {
      tags.add(metadata.dataType);
    }

    if (metadata.format) {
      tags.add(metadata.format);
    }

    if (metadata.privacy) {
      tags.add(metadata.privacy);
    }

    return Array.from(tags);
  }

  /**
   * Calculate search facets
   */
  private calculateFacets(results: UniversalMetadata[]): Record<string, Record<string, number>> {
    const facets: Record<string, Record<string, number>> = {
      domain: {},
      dataType: {},
      privacy: {},
      format: {},
    };

    for (const metadata of results) {
      if (metadata.domain) {
        facets.domain[metadata.domain] = (facets.domain[metadata.domain] || 0) + 1;
      }
      if (metadata.dataType) {
        facets.dataType[metadata.dataType] = (facets.dataType[metadata.dataType] || 0) + 1;
      }
      if (metadata.privacy) {
        facets.privacy[metadata.privacy] = (facets.privacy[metadata.privacy] || 0) + 1;
      }
      if (metadata.format) {
        facets.format[metadata.format] = (facets.format[metadata.format] || 0) + 1;
      }
    }

    return facets;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create metadata (standalone function)
 */
export function createMetadata(params: Partial<UniversalMetadata>): UniversalMetadata {
  const sdk = new UniversalMetadataSDK();
  return sdk.createMetadata(params);
}

/**
 * Validate metadata (standalone function)
 */
export function validateMetadata(metadata: UniversalMetadata): ValidationResult {
  const sdk = new UniversalMetadataSDK();
  return sdk.validateMetadata(metadata);
}

/**
 * Calculate quality (standalone function)
 */
export function calculateQuality(metadata: UniversalMetadata): QualityMetrics {
  const sdk = new UniversalMetadataSDK();
  return sdk.calculateQuality(metadata);
}

/**
 * Enrich metadata (standalone function)
 */
export function enrichMetadata(
  metadata: UniversalMetadata,
  options?: EnrichmentOptions
): EnrichmentResult {
  const sdk = new UniversalMetadataSDK();
  return sdk.enrichMetadata(metadata, options);
}

/**
 * Search metadata (standalone function)
 */
export function searchMetadata(
  query: MetadataQuery,
  metadataList: UniversalMetadata[]
): MetadataSearchResult {
  const sdk = new UniversalMetadataSDK();
  return sdk.searchMetadata(query, metadataList);
}

/**
 * Generate template (standalone function)
 */
export function generateTemplate(domain: MetadataDomain): MetadataTemplate {
  const sdk = new UniversalMetadataSDK();
  return sdk.generateTemplate(domain);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { UniversalMetadataSDK };
export default UniversalMetadataSDK;
