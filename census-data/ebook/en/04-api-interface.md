# Chapter 4: Census Data API Interface Specifications

## Comprehensive API Design for Census Data Access

### 4.1 API Architecture Overview

The WIA-CENSUS-DATA standard defines a comprehensive API architecture that enables programmatic access to census data while ensuring security, scalability, and compliance with statistical confidentiality requirements.

```typescript
// Census Data API Architecture
interface CensusAPIArchitecture {
  version: '1.0.0';
  apiStyle: 'REST with GraphQL option';

  designPrinciples: {
    restful: 'Resource-oriented design following REST principles';
    versionControl: 'Explicit versioning in URL path';
    pagination: 'Cursor-based for large datasets';
    rateLimit: 'Tiered limits based on authentication';
    caching: 'ETags and conditional requests supported';
    compression: 'gzip and brotli supported';
  };

  baseUrls: {
    production: 'https://api.census.gov/v1';
    sandbox: 'https://sandbox-api.census.gov/v1';
  };

  authenticationMethods: {
    apiKey: 'For public data access';
    oauth2: 'For restricted data and higher rate limits';
    mutualTls: 'For administrative operations';
  };

  responseFormats: {
    default: 'application/json';
    supported: ['application/json', 'text/csv', 'application/xml'];
    statistical: ['application/vnd.sdmx+json', 'application/vnd.jsonstat+json'];
  };
}

// API Gateway Configuration
class CensusAPIGateway {
  private rateLimiter: RateLimiter;
  private authService: AuthenticationService;
  private cacheService: CacheService;

  constructor(config: APIGatewayConfig) {
    this.rateLimiter = new RateLimiter(config.rateLimits);
    this.authService = new AuthenticationService(config.auth);
    this.cacheService = new CacheService(config.cache);
  }

  async handleRequest(request: APIRequest): Promise<APIResponse> {
    // Authenticate request
    const authResult = await this.authService.authenticate(request);
    if (!authResult.authenticated) {
      return this.errorResponse(401, 'Unauthorized', authResult.message);
    }

    // Check rate limits
    const rateLimitResult = await this.rateLimiter.check(authResult.clientId);
    if (rateLimitResult.exceeded) {
      return this.errorResponse(429, 'Too Many Requests', {
        retryAfter: rateLimitResult.retryAfter,
        limit: rateLimitResult.limit,
        remaining: 0
      });
    }

    // Check cache
    const cacheKey = this.buildCacheKey(request);
    const cachedResponse = await this.cacheService.get(cacheKey);
    if (cachedResponse && this.isValidCache(request, cachedResponse)) {
      return this.addCacheHeaders(cachedResponse, 'HIT');
    }

    // Route to appropriate handler
    const response = await this.routeRequest(request, authResult);

    // Cache response if cacheable
    if (this.isCacheable(response)) {
      await this.cacheService.set(cacheKey, response, this.getCacheTTL(request));
    }

    return this.addRateLimitHeaders(response, rateLimitResult);
  }

  private errorResponse(
    status: number,
    error: string,
    details?: any
  ): APIResponse {
    return {
      status,
      headers: { 'Content-Type': 'application/json' },
      body: {
        error: { code: status, message: error, details }
      }
    };
  }
}
```

### 4.2 Data Discovery API

```typescript
// Data Discovery Endpoints
interface DataDiscoveryAPI {
  endpoints: {
    // List available datasets
    listDatasets: {
      method: 'GET';
      path: '/datasets';
      description: 'List all available census datasets';
      parameters: {
        query: {
          topic: 'Filter by topic category';
          year: 'Filter by reference year';
          geography: 'Filter by geographic coverage';
          page: 'Page number for pagination';
          perPage: 'Items per page (default 20, max 100)';
        };
      };
      response: DatasetListResponse;
    };

    // Get dataset details
    getDataset: {
      method: 'GET';
      path: '/datasets/{datasetId}';
      description: 'Get detailed information about a dataset';
      parameters: {
        path: { datasetId: 'Unique dataset identifier' };
      };
      response: DatasetDetailResponse;
    };

    // List variables in dataset
    listVariables: {
      method: 'GET';
      path: '/datasets/{datasetId}/variables';
      description: 'List all variables in a dataset';
      parameters: {
        path: { datasetId: 'Dataset identifier' };
        query: {
          search: 'Search variable names and descriptions';
          group: 'Filter by variable group';
        };
      };
      response: VariableListResponse;
    };

    // Get variable details
    getVariable: {
      method: 'GET';
      path: '/datasets/{datasetId}/variables/{variableId}';
      description: 'Get detailed information about a variable';
      response: VariableDetailResponse;
    };

    // List geographies
    listGeographies: {
      method: 'GET';
      path: '/datasets/{datasetId}/geographies';
      description: 'List available geographic levels';
      response: GeographyListResponse;
    };
  };
}

// Data Discovery Response Types
interface DatasetListResponse {
  datasets: DatasetSummary[];
  pagination: PaginationInfo;
  _links: {
    self: string;
    next?: string;
    prev?: string;
  };
}

interface DatasetSummary {
  id: string;
  title: string;
  description: string;
  referenceYear: number;
  topics: string[];
  geographicCoverage: string;
  lowestGeography: string;
  updateFrequency: string;
  lastUpdated: string;
  _links: {
    self: string;
    variables: string;
    geographies: string;
    data: string;
  };
}

interface DatasetDetailResponse {
  id: string;
  title: string;
  description: string;
  methodology: {
    universe: string;
    dataCollectionMethod: string;
    samplingInfo?: string;
    qualityNotes: string;
  };
  temporalCoverage: {
    referenceDate: string;
    collectionPeriod: string;
  };
  geographicCoverage: {
    countries: string[];
    lowestLevel: string;
    availableLevels: string[];
  };
  variables: {
    count: number;
    groups: VariableGroup[];
  };
  access: {
    publicAccess: boolean;
    restrictedVariables: string[];
    citationRequirement: string;
  };
  metadata: ReferenceMetadata;
  _links: {
    self: string;
    variables: string;
    geographies: string;
    data: string;
    download: string;
  };
}

interface VariableDetailResponse {
  id: string;
  name: string;
  label: string;
  description: string;
  concept: string;
  group: string;
  dataType: 'numeric' | 'categorical' | 'text';
  unit?: string;
  universe: string;
  codeList?: CodeListItem[];
  statistics?: {
    min?: number;
    max?: number;
    mean?: number;
    missing?: number;
  };
  qualityFlags: string[];
  comparability: string;
  _links: {
    self: string;
    dataset: string;
    codelist?: string;
  };
}

interface CodeListItem {
  code: string;
  label: string;
  description?: string;
  parentCode?: string;
}

// Data Discovery Service Implementation
class DataDiscoveryService {
  private datasetRepository: DatasetRepository;
  private variableRepository: VariableRepository;
  private searchEngine: SearchEngine;

  async listDatasets(params: DatasetQueryParams): Promise<DatasetListResponse> {
    const query = this.buildQuery(params);
    const results = await this.datasetRepository.find(query);
    const total = await this.datasetRepository.count(query);

    return {
      datasets: results.map(this.toDatasetSummary),
      pagination: {
        page: params.page || 1,
        perPage: params.perPage || 20,
        total,
        totalPages: Math.ceil(total / (params.perPage || 20))
      },
      _links: this.buildPaginationLinks(params, total)
    };
  }

  async searchVariables(
    datasetId: string,
    searchTerm: string
  ): Promise<VariableSearchResult[]> {
    // Full-text search across variable names, labels, and descriptions
    const results = await this.searchEngine.search({
      index: 'variables',
      query: {
        bool: {
          must: [
            { term: { datasetId } },
            {
              multi_match: {
                query: searchTerm,
                fields: ['name^3', 'label^2', 'description', 'concept'],
                fuzziness: 'AUTO'
              }
            }
          ]
        }
      },
      highlight: {
        fields: {
          name: {},
          label: {},
          description: {}
        }
      }
    });

    return results.hits.map(hit => ({
      variable: hit._source,
      score: hit._score,
      highlights: hit.highlight
    }));
  }
}
```

### 4.3 Data Retrieval API

```typescript
// Data Retrieval Endpoints
interface DataRetrievalAPI {
  endpoints: {
    // Query data
    queryData: {
      method: 'GET';
      path: '/datasets/{datasetId}/data';
      description: 'Query census data with filters';
      parameters: {
        path: { datasetId: 'Dataset identifier' };
        query: {
          get: 'Comma-separated list of variables';
          for: 'Target geography (e.g., "county:*")';
          in: 'Parent geography constraint';
          time: 'Time period filter';
          filter: 'Additional variable filters';
        };
      };
      response: DataQueryResponse;
      example: '/datasets/acs5/data?get=NAME,B01001_001E&for=county:*&in=state:06';
    };

    // Query with POST (for complex queries)
    queryDataPost: {
      method: 'POST';
      path: '/datasets/{datasetId}/data/query';
      description: 'Complex data query with request body';
      requestBody: DataQueryRequest;
      response: DataQueryResponse;
    };

    // Get aggregate statistics
    getAggregates: {
      method: 'GET';
      path: '/datasets/{datasetId}/aggregates';
      description: 'Get pre-computed aggregate statistics';
      parameters: {
        query: {
          variable: 'Variable to aggregate';
          geography: 'Geographic level';
          groupBy: 'Grouping variables';
          measure: 'Statistic type (count, sum, mean, etc.)';
        };
      };
      response: AggregateResponse;
    };

    // Download bulk data
    downloadData: {
      method: 'GET';
      path: '/datasets/{datasetId}/download';
      description: 'Download complete dataset or subset';
      parameters: {
        query: {
          format: 'Output format (csv, parquet, json)';
          variables: 'Variables to include';
          geography: 'Geographic filter';
          compression: 'Compression type (gzip, zip)';
        };
      };
      response: 'Binary file stream';
    };
  };
}

// Data Query Request/Response
interface DataQueryRequest {
  variables: string[];
  geography: {
    for: GeographySelector;
    in?: GeographySelector;
  };
  filters?: VariableFilter[];
  sort?: SortSpecification[];
  pagination?: {
    limit: number;
    offset: number;
  };
  options?: {
    includeMetadata: boolean;
    includeAnnotations: boolean;
    includeGeometry: boolean;
  };
}

interface GeographySelector {
  level: string;
  codes: string[] | '*';
}

interface VariableFilter {
  variable: string;
  operator: 'eq' | 'ne' | 'gt' | 'gte' | 'lt' | 'lte' | 'in' | 'between';
  value: string | number | (string | number)[];
}

interface SortSpecification {
  variable: string;
  direction: 'asc' | 'desc';
}

interface DataQueryResponse {
  data: DataRow[];
  metadata?: {
    variables: VariableMetadata[];
    geography: GeographyMetadata;
    queryTime: number;
  };
  pagination?: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
  annotations?: DataAnnotation[];
  _links: {
    self: string;
    next?: string;
    download?: string;
  };
}

interface DataRow {
  [variableId: string]: string | number | null;
  _geography?: {
    code: string;
    name: string;
    level: string;
    geometry?: GeoJSON.Geometry;
  };
}

// Data Retrieval Service Implementation
class DataRetrievalService {
  private dataStore: CensusDataStore;
  private queryBuilder: QueryBuilder;
  private disclosureControl: DisclosureControlService;

  async queryData(
    datasetId: string,
    request: DataQueryRequest,
    userContext: UserContext
  ): Promise<DataQueryResponse> {
    // Validate request
    this.validateRequest(datasetId, request);

    // Check user access to requested variables
    const accessibleVariables = await this.filterByAccess(
      request.variables,
      userContext
    );

    // Build query
    const query = this.queryBuilder.build({
      dataset: datasetId,
      variables: accessibleVariables,
      geography: request.geography,
      filters: request.filters,
      sort: request.sort,
      pagination: request.pagination
    });

    // Execute query
    const rawResults = await this.dataStore.execute(query);

    // Apply disclosure control
    const protectedResults = await this.disclosureControl.applyControls(
      rawResults,
      datasetId,
      request
    );

    // Format response
    return this.formatResponse(
      protectedResults,
      request,
      accessibleVariables
    );
  }

  async getAggregates(
    datasetId: string,
    params: AggregateParams
  ): Promise<AggregateResponse> {
    const cacheKey = this.buildCacheKey('aggregate', datasetId, params);

    // Check for pre-computed aggregates
    const cached = await this.dataStore.getCachedAggregate(cacheKey);
    if (cached) {
      return cached;
    }

    // Compute aggregates
    const aggregateQuery = this.buildAggregateQuery(datasetId, params);
    const results = await this.dataStore.executeAggregate(aggregateQuery);

    // Apply disclosure control to aggregates
    const protected_ = await this.disclosureControl.protectAggregates(
      results,
      params
    );

    // Cache results
    await this.dataStore.cacheAggregate(cacheKey, protected_, 3600);

    return protected_;
  }

  private validateRequest(datasetId: string, request: DataQueryRequest): void {
    // Validate variable count
    if (request.variables.length > 50) {
      throw new ValidationError('Maximum 50 variables per query');
    }

    // Validate geography
    if (!request.geography?.for) {
      throw new ValidationError('Geography specification required');
    }

    // Validate pagination limits
    if (request.pagination?.limit && request.pagination.limit > 10000) {
      throw new ValidationError('Maximum 10,000 rows per request');
    }
  }
}
```

### 4.4 Geographic API

```typescript
// Geographic Data API
interface GeographicAPI {
  endpoints: {
    // List geographic hierarchy
    listHierarchy: {
      method: 'GET';
      path: '/geographies/hierarchy';
      description: 'Get geographic hierarchy structure';
      response: GeographyHierarchyResponse;
    };

    // Get geography by code
    getGeography: {
      method: 'GET';
      path: '/geographies/{geoCode}';
      description: 'Get details for a specific geography';
      parameters: {
        path: { geoCode: 'Geographic code' };
        query: {
          includeGeometry: 'Include boundary geometry';
          geometrySimplification: 'Simplification level';
          vintage: 'Geographic vintage year';
        };
      };
      response: GeographyDetailResponse;
    };

    // Search geographies
    searchGeographies: {
      method: 'GET';
      path: '/geographies/search';
      description: 'Search geographies by name';
      parameters: {
        query: {
          q: 'Search query';
          level: 'Filter by geographic level';
          within: 'Search within parent geography';
          limit: 'Maximum results';
        };
      };
      response: GeographySearchResponse;
    };

    // Get children geographies
    getChildren: {
      method: 'GET';
      path: '/geographies/{geoCode}/children';
      description: 'Get child geographies';
      parameters: {
        path: { geoCode: 'Parent geographic code' };
        query: {
          level: 'Target child level';
          includeGeometry: 'Include boundary geometries';
        };
      };
      response: GeographyListResponse;
    };

    // Get boundary geometry
    getBoundary: {
      method: 'GET';
      path: '/geographies/{geoCode}/boundary';
      description: 'Get boundary geometry';
      parameters: {
        path: { geoCode: 'Geographic code' };
        query: {
          format: 'geojson | topojson | wkt';
          simplification: '0-100 (percentage)';
          vintage: 'Geographic vintage year';
        };
      };
      response: 'GeoJSON Feature or TopoJSON';
    };

    // Geocode address
    geocode: {
      method: 'POST';
      path: '/geographies/geocode';
      description: 'Geocode addresses to census geographies';
      requestBody: GeocodeRequest;
      response: GeocodeResponse;
    };

    // Reverse geocode
    reverseGeocode: {
      method: 'GET';
      path: '/geographies/reverse';
      description: 'Get census geography for coordinates';
      parameters: {
        query: {
          lat: 'Latitude';
          lon: 'Longitude';
          levels: 'Geographic levels to return';
          vintage: 'Geographic vintage year';
        };
      };
      response: ReverseGeocodeResponse;
    };
  };
}

// Geographic Response Types
interface GeographyDetailResponse {
  geoCode: string;
  name: string;
  formalName: string;
  level: string;
  levelName: string;
  vintage: number;
  characteristics: {
    landArea: number;
    waterArea: number;
    population?: number;
    urbanRuralClass: string;
  };
  centroid: {
    latitude: number;
    longitude: number;
  };
  boundingBox: {
    west: number;
    east: number;
    south: number;
    north: number;
  };
  geometry?: GeoJSON.Geometry;
  parent?: {
    geoCode: string;
    name: string;
    level: string;
  };
  _links: {
    self: string;
    parent?: string;
    children: string;
    boundary: string;
    data: string;
  };
}

interface GeocodeRequest {
  addresses: AddressInput[];
  options?: {
    matchType: 'exact' | 'relaxed';
    returnLevels: string[];
    includeInput: boolean;
  };
}

interface AddressInput {
  id?: string;
  street: string;
  city?: string;
  state?: string;
  postalCode?: string;
  country?: string;
}

interface GeocodeResponse {
  results: GeocodeResult[];
  matchStats: {
    total: number;
    matched: number;
    matchRate: number;
  };
}

interface GeocodeResult {
  input: AddressInput;
  status: 'matched' | 'multiple' | 'unmatched';
  match?: {
    address: string;
    coordinates: {
      latitude: number;
      longitude: number;
    };
    matchScore: number;
    matchType: string;
    geographies: {
      [level: string]: {
        geoCode: string;
        name: string;
      };
    };
  };
  alternates?: GeocodeMatch[];
}

// Geographic Service Implementation
class GeographicService {
  private geoRepository: GeographyRepository;
  private geocoder: GeocodingEngine;
  private boundaryService: BoundaryService;

  async getGeography(
    geoCode: string,
    options: GeographyOptions
  ): Promise<GeographyDetailResponse> {
    const geography = await this.geoRepository.findByCode(
      geoCode,
      options.vintage || this.getCurrentVintage()
    );

    if (!geography) {
      throw new NotFoundError(`Geography not found: ${geoCode}`);
    }

    let geometry: GeoJSON.Geometry | undefined;
    if (options.includeGeometry) {
      geometry = await this.boundaryService.getBoundary(
        geoCode,
        options.vintage,
        options.geometrySimplification
      );
    }

    return {
      ...this.mapToResponse(geography),
      geometry,
      _links: this.buildLinks(geography)
    };
  }

  async geocodeAddresses(
    request: GeocodeRequest
  ): Promise<GeocodeResponse> {
    const results: GeocodeResult[] = [];

    for (const address of request.addresses) {
      const result = await this.geocoder.geocode(address, request.options);
      results.push(result);
    }

    return {
      results,
      matchStats: {
        total: results.length,
        matched: results.filter(r => r.status === 'matched').length,
        matchRate: results.filter(r => r.status === 'matched').length / results.length
      }
    };
  }

  async reverseGeocode(
    lat: number,
    lon: number,
    levels: string[],
    vintage?: number
  ): Promise<ReverseGeocodeResponse> {
    const point = { type: 'Point', coordinates: [lon, lat] };

    const geographies: { [level: string]: GeographyReference } = {};

    for (const level of levels) {
      const geo = await this.geoRepository.findContaining(
        point,
        level,
        vintage || this.getCurrentVintage()
      );

      if (geo) {
        geographies[level] = {
          geoCode: geo.geoCode,
          name: geo.name
        };
      }
    }

    return {
      coordinates: { latitude: lat, longitude: lon },
      vintage: vintage || this.getCurrentVintage(),
      geographies
    };
  }
}
```

### 4.5 Administrative API

```typescript
// Administrative API for Census Operations
interface AdministrativeAPI {
  endpoints: {
    // Data Submission
    submitData: {
      method: 'POST';
      path: '/admin/submissions';
      description: 'Submit census response data';
      authentication: 'OAuth2 with admin scope';
      requestBody: DataSubmission;
      response: SubmissionResponse;
    };

    // Validate submission
    validateSubmission: {
      method: 'POST';
      path: '/admin/submissions/validate';
      description: 'Validate data before submission';
      requestBody: DataSubmission;
      response: ValidationResponse;
    };

    // Get submission status
    getSubmissionStatus: {
      method: 'GET';
      path: '/admin/submissions/{submissionId}';
      description: 'Get status of a submission';
      response: SubmissionStatusResponse;
    };

    // List submissions
    listSubmissions: {
      method: 'GET';
      path: '/admin/submissions';
      description: 'List submissions with filtering';
      parameters: {
        query: {
          status: 'Filter by status';
          dateFrom: 'Filter by submission date';
          dateTo: 'Filter by submission date';
          region: 'Filter by geographic region';
        };
      };
      response: SubmissionListResponse;
    };

    // Quality Reports
    getQualityReport: {
      method: 'GET';
      path: '/admin/quality/{reportId}';
      description: 'Get data quality report';
      response: QualityReportResponse;
    };

    // Generate quality report
    generateQualityReport: {
      method: 'POST';
      path: '/admin/quality/generate';
      description: 'Generate new quality report';
      requestBody: QualityReportRequest;
      response: QualityReportResponse;
    };

    // System health
    getHealth: {
      method: 'GET';
      path: '/admin/health';
      description: 'Get system health status';
      response: HealthResponse;
    };

    // Usage statistics
    getUsageStats: {
      method: 'GET';
      path: '/admin/usage';
      description: 'Get API usage statistics';
      parameters: {
        query: {
          period: 'daily | weekly | monthly';
          startDate: 'Start of period';
          endDate: 'End of period';
        };
      };
      response: UsageStatsResponse;
    };
  };
}

// Administrative Data Types
interface DataSubmission {
  submissionType: 'INDIVIDUAL' | 'HOUSEHOLD' | 'BATCH';
  sourceSystem: string;
  referenceDate: string;
  geographicCode: string;
  records: SubmissionRecord[];
  metadata: {
    collectionMode: string;
    enumeratorId?: string;
    timestamp: string;
    qualityIndicators?: QualityIndicator[];
  };
}

interface SubmissionRecord {
  recordId: string;
  recordType: 'PERSON' | 'HOUSEHOLD' | 'DWELLING';
  data: { [variable: string]: any };
  linkedRecords?: {
    householdId?: string;
    dwellingId?: string;
  };
}

interface ValidationResponse {
  valid: boolean;
  recordCount: number;
  errors: ValidationError[];
  warnings: ValidationWarning[];
  summary: {
    errorsCount: number;
    warningsCount: number;
    validRecords: number;
    invalidRecords: number;
  };
}

interface ValidationError {
  recordId: string;
  field: string;
  errorCode: string;
  message: string;
  severity: 'ERROR' | 'FATAL';
}

interface ValidationWarning {
  recordId: string;
  field: string;
  warningCode: string;
  message: string;
}

// Administrative Service Implementation
class AdministrativeService {
  private submissionRepository: SubmissionRepository;
  private validationEngine: ValidationEngine;
  private qualityService: QualityService;

  async submitData(
    submission: DataSubmission,
    userContext: UserContext
  ): Promise<SubmissionResponse> {
    // Validate submission
    const validation = await this.validationEngine.validate(submission);

    if (!validation.valid) {
      throw new ValidationException('Submission contains errors', validation.errors);
    }

    // Create submission record
    const submissionId = await this.submissionRepository.create({
      ...submission,
      submittedBy: userContext.userId,
      submittedAt: new Date().toISOString(),
      status: 'PENDING'
    });

    // Queue for processing
    await this.queueForProcessing(submissionId);

    return {
      submissionId,
      status: 'ACCEPTED',
      recordsReceived: submission.records.length,
      warnings: validation.warnings,
      estimatedProcessingTime: this.estimateProcessingTime(submission),
      _links: {
        status: `/admin/submissions/${submissionId}`,
        cancel: `/admin/submissions/${submissionId}/cancel`
      }
    };
  }

  async generateQualityReport(
    request: QualityReportRequest
  ): Promise<QualityReportResponse> {
    const report = await this.qualityService.generateReport({
      datasetId: request.datasetId,
      geographicScope: request.geographicScope,
      variables: request.variables,
      metrics: request.metrics
    });

    return {
      reportId: report.id,
      generatedAt: report.timestamp,
      coverage: report.coverage,
      completeness: report.completeness,
      accuracy: report.accuracy,
      consistency: report.consistency,
      timeliness: report.timeliness,
      recommendations: report.recommendations
    };
  }
}
```

### 4.6 GraphQL API Option

```typescript
// GraphQL Schema for Census Data
const censusGraphQLSchema = `
  type Query {
    # Dataset queries
    datasets(
      topic: String
      year: Int
      geography: String
      first: Int
      after: String
    ): DatasetConnection!

    dataset(id: ID!): Dataset

    # Variable queries
    variables(
      datasetId: ID!
      search: String
      group: String
      first: Int
      after: String
    ): VariableConnection!

    variable(datasetId: ID!, variableId: ID!): Variable

    # Data queries
    censusData(
      datasetId: ID!
      variables: [String!]!
      geography: GeographyInput!
      filters: [FilterInput!]
      first: Int
      after: String
    ): DataConnection!

    # Geographic queries
    geography(code: String!, vintage: Int): Geography
    geographies(
      level: String
      within: String
      search: String
      first: Int
      after: String
    ): GeographyConnection!

    geocode(address: AddressInput!): GeocodeResult
    reverseGeocode(lat: Float!, lon: Float!, levels: [String!]): ReverseGeocodeResult
  }

  type Dataset {
    id: ID!
    title: String!
    description: String!
    referenceYear: Int!
    topics: [String!]!
    geographicCoverage: String!
    lowestGeography: String!
    variables(first: Int, after: String): VariableConnection!
    geographies: [GeographyLevel!]!
    metadata: DatasetMetadata!
  }

  type Variable {
    id: ID!
    name: String!
    label: String!
    description: String!
    concept: String!
    group: String!
    dataType: DataType!
    unit: String
    codeList: [CodeListItem!]
    statistics: VariableStatistics
  }

  type Geography {
    code: String!
    name: String!
    formalName: String!
    level: String!
    vintage: Int!
    characteristics: GeographyCharacteristics!
    centroid: Coordinates!
    boundingBox: BoundingBox!
    boundary(simplification: Int): GeoJSONGeometry
    parent: Geography
    children(level: String, first: Int, after: String): GeographyConnection!
    data(
      datasetId: ID!
      variables: [String!]!
    ): DataRow
  }

  type DataRow {
    geography: Geography!
    values: [DataValue!]!
    annotations: [Annotation!]
  }

  type DataValue {
    variable: Variable!
    value: String
    marginOfError: Float
    qualityFlag: String
  }

  input GeographyInput {
    for: GeographySelectorInput!
    in: GeographySelectorInput
  }

  input GeographySelectorInput {
    level: String!
    codes: [String!]
  }

  input FilterInput {
    variable: String!
    operator: FilterOperator!
    value: String!
  }

  enum FilterOperator {
    EQ
    NE
    GT
    GTE
    LT
    LTE
    IN
    BETWEEN
  }

  enum DataType {
    NUMERIC
    CATEGORICAL
    TEXT
  }

  # Connection types for pagination
  type DatasetConnection {
    edges: [DatasetEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type DatasetEdge {
    node: Dataset!
    cursor: String!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }
`;

// GraphQL Resolver Implementation
class CensusGraphQLResolvers {
  private dataService: DataRetrievalService;
  private discoveryService: DataDiscoveryService;
  private geoService: GeographicService;

  Query = {
    datasets: async (_, args, context) => {
      const result = await this.discoveryService.listDatasets({
        topic: args.topic,
        year: args.year,
        geography: args.geography,
        first: args.first,
        after: args.after
      });
      return this.toConnection(result);
    },

    dataset: async (_, { id }, context) => {
      return this.discoveryService.getDataset(id);
    },

    censusData: async (_, args, context) => {
      const result = await this.dataService.queryData(
        args.datasetId,
        {
          variables: args.variables,
          geography: args.geography,
          filters: args.filters,
          pagination: { limit: args.first, cursor: args.after }
        },
        context.user
      );
      return this.toDataConnection(result);
    },

    geography: async (_, { code, vintage }, context) => {
      return this.geoService.getGeography(code, { vintage });
    },

    geocode: async (_, { address }, context) => {
      const result = await this.geoService.geocodeAddresses({
        addresses: [address]
      });
      return result.results[0];
    }
  };

  Dataset = {
    variables: async (dataset, args, context) => {
      const result = await this.discoveryService.listVariables(
        dataset.id,
        { first: args.first, after: args.after }
      );
      return this.toConnection(result);
    }
  };

  Geography = {
    parent: async (geography, _, context) => {
      if (geography.parentCode) {
        return this.geoService.getGeography(geography.parentCode, {
          vintage: geography.vintage
        });
      }
      return null;
    },

    children: async (geography, args, context) => {
      const result = await this.geoService.getChildren(
        geography.code,
        { level: args.level, first: args.first, after: args.after }
      );
      return this.toConnection(result);
    },

    data: async (geography, { datasetId, variables }, context) => {
      const result = await this.dataService.queryData(
        datasetId,
        {
          variables,
          geography: { for: { level: geography.level, codes: [geography.code] } }
        },
        context.user
      );
      return result.data[0];
    }
  };
}
```

---

**WIA-CENSUS-DATA API Interface**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
