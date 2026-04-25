# 제4장: 인구조사 데이터 API 인터페이스 사양

## 인구조사 데이터 접근을 위한 포괄적인 API 설계

### 4.1 API 아키텍처 개요

WIA-CENSUS-DATA 표준은 보안, 확장성 및 통계적 기밀성 요구사항을 준수하면서 인구조사 데이터에 대한 프로그래매틱 접근을 가능하게 하는 포괄적인 API 아키텍처를 정의합니다.

```typescript
// 인구조사 데이터 API 아키텍처
interface CensusAPIArchitecture {
  version: '1.0.0';
  apiStyle: 'GraphQL 옵션이 있는 REST';

  designPrinciples: {
    restful: 'REST 원칙을 따르는 리소스 지향 설계';
    versionControl: 'URL 경로에 명시적 버전 관리';
    pagination: '대규모 데이터셋을 위한 커서 기반';
    rateLimit: '인증에 따른 계층화된 제한';
    caching: 'ETags 및 조건부 요청 지원';
    compression: 'gzip 및 brotli 지원';
  };

  baseUrls: {
    production: 'https://api.census.go.kr/v1';
    sandbox: 'https://sandbox-api.census.go.kr/v1';
  };

  authenticationMethods: {
    apiKey: '공개 데이터 접근용';
    oauth2: '제한된 데이터 및 높은 요율 제한용';
    mutualTls: '관리 운영용';
  };

  responseFormats: {
    default: 'application/json';
    supported: ['application/json', 'text/csv', 'application/xml'];
    statistical: ['application/vnd.sdmx+json', 'application/vnd.jsonstat+json'];
  };
}

// API 게이트웨이 구성
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
    // 요청 인증
    const authResult = await this.authService.authenticate(request);
    if (!authResult.authenticated) {
      return this.errorResponse(401, 'Unauthorized', authResult.message);
    }

    // 요율 제한 확인
    const rateLimitResult = await this.rateLimiter.check(authResult.clientId);
    if (rateLimitResult.exceeded) {
      return this.errorResponse(429, 'Too Many Requests', {
        retryAfter: rateLimitResult.retryAfter,
        limit: rateLimitResult.limit,
        remaining: 0
      });
    }

    // 캐시 확인
    const cacheKey = this.buildCacheKey(request);
    const cachedResponse = await this.cacheService.get(cacheKey);
    if (cachedResponse && this.isValidCache(request, cachedResponse)) {
      return this.addCacheHeaders(cachedResponse, 'HIT');
    }

    // 적절한 핸들러로 라우팅
    const response = await this.routeRequest(request, authResult);

    // 캐시 가능한 경우 응답 캐시
    if (this.isCacheable(response)) {
      await this.cacheService.set(cacheKey, response, this.getCacheTTL(request));
    }

    return this.addRateLimitHeaders(response, rateLimitResult);
  }
}
```

### 4.2 데이터 검색 API

```typescript
// 데이터 검색 엔드포인트
interface DataDiscoveryAPI {
  endpoints: {
    // 사용 가능한 데이터셋 목록
    listDatasets: {
      method: 'GET';
      path: '/datasets';
      description: '사용 가능한 모든 인구조사 데이터셋 목록';
      parameters: {
        query: {
          topic: '주제 카테고리로 필터';
          year: '기준 연도로 필터';
          geography: '지리적 범위로 필터';
          page: '페이지 번호';
          perPage: '페이지당 항목 수 (기본 20, 최대 100)';
        };
      };
      response: DatasetListResponse;
    };

    // 데이터셋 상세 정보 조회
    getDataset: {
      method: 'GET';
      path: '/datasets/{datasetId}';
      description: '데이터셋에 대한 상세 정보 조회';
      response: DatasetDetailResponse;
    };

    // 데이터셋의 변수 목록
    listVariables: {
      method: 'GET';
      path: '/datasets/{datasetId}/variables';
      description: '데이터셋의 모든 변수 목록';
      parameters: {
        query: {
          search: '변수 이름 및 설명 검색';
          group: '변수 그룹으로 필터';
        };
      };
      response: VariableListResponse;
    };

    // 지리 목록
    listGeographies: {
      method: 'GET';
      path: '/datasets/{datasetId}/geographies';
      description: '사용 가능한 지리적 수준 목록';
      response: GeographyListResponse;
    };
  };
}

// 데이터 검색 응답 유형
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
  lastUpdated: string;
  _links: {
    self: string;
    variables: string;
    geographies: string;
    data: string;
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
}

// 데이터 검색 서비스 구현
class DataDiscoveryService {
  private datasetRepository: DatasetRepository;
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
    // 변수 이름, 레이블, 설명에서 전문 검색
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

### 4.3 데이터 조회 API

```typescript
// 데이터 조회 엔드포인트
interface DataRetrievalAPI {
  endpoints: {
    // 데이터 쿼리
    queryData: {
      method: 'GET';
      path: '/datasets/{datasetId}/data';
      description: '필터로 인구조사 데이터 쿼리';
      parameters: {
        path: { datasetId: '데이터셋 식별자' };
        query: {
          get: '쉼표로 구분된 변수 목록';
          for: '대상 지리 (예: "county:*")';
          in: '상위 지리 제약';
          time: '기간 필터';
          filter: '추가 변수 필터';
        };
      };
      response: DataQueryResponse;
      example: '/datasets/census2020/data?get=NAME,POPULATION&for=district:*&in=region:11';
    };

    // POST로 쿼리 (복잡한 쿼리용)
    queryDataPost: {
      method: 'POST';
      path: '/datasets/{datasetId}/data/query';
      description: '요청 본문이 있는 복잡한 데이터 쿼리';
      requestBody: DataQueryRequest;
      response: DataQueryResponse;
    };

    // 집계 통계 조회
    getAggregates: {
      method: 'GET';
      path: '/datasets/{datasetId}/aggregates';
      description: '사전 계산된 집계 통계 조회';
      parameters: {
        query: {
          variable: '집계할 변수';
          geography: '지리적 수준';
          groupBy: '그룹화 변수';
          measure: '통계 유형 (count, sum, mean 등)';
        };
      };
      response: AggregateResponse;
    };

    // 대량 데이터 다운로드
    downloadData: {
      method: 'GET';
      path: '/datasets/{datasetId}/download';
      description: '전체 데이터셋 또는 부분 다운로드';
      parameters: {
        query: {
          format: '출력 형식 (csv, parquet, json)';
          variables: '포함할 변수';
          geography: '지리 필터';
          compression: '압축 유형 (gzip, zip)';
        };
      };
      response: '바이너리 파일 스트림';
    };
  };
}

// 데이터 쿼리 요청/응답
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
  _links: {
    self: string;
    next?: string;
    download?: string;
  };
}

// 데이터 조회 서비스 구현
class DataRetrievalService {
  private dataStore: CensusDataStore;
  private queryBuilder: QueryBuilder;
  private disclosureControl: DisclosureControlService;

  async queryData(
    datasetId: string,
    request: DataQueryRequest,
    userContext: UserContext
  ): Promise<DataQueryResponse> {
    // 요청 검증
    this.validateRequest(datasetId, request);

    // 요청된 변수에 대한 사용자 접근 확인
    const accessibleVariables = await this.filterByAccess(
      request.variables,
      userContext
    );

    // 쿼리 작성
    const query = this.queryBuilder.build({
      dataset: datasetId,
      variables: accessibleVariables,
      geography: request.geography,
      filters: request.filters,
      sort: request.sort,
      pagination: request.pagination
    });

    // 쿼리 실행
    const rawResults = await this.dataStore.execute(query);

    // 공개 제어 적용
    const protectedResults = await this.disclosureControl.applyControls(
      rawResults,
      datasetId,
      request
    );

    // 응답 형식화
    return this.formatResponse(
      protectedResults,
      request,
      accessibleVariables
    );
  }

  private validateRequest(datasetId: string, request: DataQueryRequest): void {
    // 변수 수 검증
    if (request.variables.length > 50) {
      throw new ValidationError('쿼리당 최대 50개 변수');
    }

    // 지리 검증
    if (!request.geography?.for) {
      throw new ValidationError('지리 사양 필요');
    }

    // 페이지네이션 제한 검증
    if (request.pagination?.limit && request.pagination.limit > 10000) {
      throw new ValidationError('요청당 최대 10,000행');
    }
  }
}
```

### 4.4 지리 API

```typescript
// 지리 데이터 API
interface GeographicAPI {
  endpoints: {
    // 지리적 계층 목록
    listHierarchy: {
      method: 'GET';
      path: '/geographies/hierarchy';
      description: '지리적 계층 구조 조회';
      response: GeographyHierarchyResponse;
    };

    // 코드로 지리 조회
    getGeography: {
      method: 'GET';
      path: '/geographies/{geoCode}';
      description: '특정 지리에 대한 상세 정보 조회';
      parameters: {
        query: {
          includeGeometry: '경계 지오메트리 포함';
          geometrySimplification: '단순화 수준';
          vintage: '지리 빈티지 연도';
        };
      };
      response: GeographyDetailResponse;
    };

    // 지리 검색
    searchGeographies: {
      method: 'GET';
      path: '/geographies/search';
      description: '이름으로 지리 검색';
      parameters: {
        query: {
          q: '검색 쿼리';
          level: '지리적 수준으로 필터';
          within: '상위 지리 내에서 검색';
        };
      };
      response: GeographySearchResponse;
    };

    // 하위 지리 조회
    getChildren: {
      method: 'GET';
      path: '/geographies/{geoCode}/children';
      description: '하위 지리 조회';
      response: GeographyListResponse;
    };

    // 주소 지오코딩
    geocode: {
      method: 'POST';
      path: '/geographies/geocode';
      description: '주소를 인구조사 지리로 지오코딩';
      requestBody: GeocodeRequest;
      response: GeocodeResponse;
    };

    // 역 지오코딩
    reverseGeocode: {
      method: 'GET';
      path: '/geographies/reverse';
      description: '좌표에 대한 인구조사 지리 조회';
      parameters: {
        query: {
          lat: '위도';
          lon: '경도';
          levels: '반환할 지리적 수준';
        };
      };
      response: ReverseGeocodeResponse;
    };
  };
}

// 지리 응답 유형
interface GeographyDetailResponse {
  geoCode: string;
  name: string;
  formalName: string;
  level: string;
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
  };
}

interface GeocodeResponse {
  results: GeocodeResult[];
  matchStats: {
    total: number;
    matched: number;
    matchRate: number;
  };
}

// 지리 서비스 구현
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
      throw new NotFoundError(`지리를 찾을 수 없음: ${geoCode}`);
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

### 4.5 관리 API

```typescript
// 인구조사 운영을 위한 관리 API
interface AdministrativeAPI {
  endpoints: {
    // 데이터 제출
    submitData: {
      method: 'POST';
      path: '/admin/submissions';
      description: '인구조사 응답 데이터 제출';
      authentication: '관리 범위가 있는 OAuth2';
      requestBody: DataSubmission;
      response: SubmissionResponse;
    };

    // 제출 검증
    validateSubmission: {
      method: 'POST';
      path: '/admin/submissions/validate';
      description: '제출 전 데이터 검증';
      requestBody: DataSubmission;
      response: ValidationResponse;
    };

    // 제출 상태 조회
    getSubmissionStatus: {
      method: 'GET';
      path: '/admin/submissions/{submissionId}';
      description: '제출 상태 조회';
      response: SubmissionStatusResponse;
    };

    // 품질 보고서
    getQualityReport: {
      method: 'GET';
      path: '/admin/quality/{reportId}';
      description: '데이터 품질 보고서 조회';
      response: QualityReportResponse;
    };

    // 시스템 상태
    getHealth: {
      method: 'GET';
      path: '/admin/health';
      description: '시스템 상태 조회';
      response: HealthResponse;
    };

    // 사용 통계
    getUsageStats: {
      method: 'GET';
      path: '/admin/usage';
      description: 'API 사용 통계 조회';
      response: UsageStatsResponse;
    };
  };
}

// 관리 데이터 유형
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

// 관리 서비스 구현
class AdministrativeService {
  private submissionRepository: SubmissionRepository;
  private validationEngine: ValidationEngine;
  private qualityService: QualityService;

  async submitData(
    submission: DataSubmission,
    userContext: UserContext
  ): Promise<SubmissionResponse> {
    // 제출 검증
    const validation = await this.validationEngine.validate(submission);

    if (!validation.valid) {
      throw new ValidationException('제출에 오류가 포함됨', validation.errors);
    }

    // 제출 레코드 생성
    const submissionId = await this.submissionRepository.create({
      ...submission,
      submittedBy: userContext.userId,
      submittedAt: new Date().toISOString(),
      status: 'PENDING'
    });

    // 처리 대기열에 추가
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
      recommendations: report.recommendations
    };
  }
}
```

---

**WIA-CENSUS-DATA API 인터페이스**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
