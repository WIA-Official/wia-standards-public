# 제4장: API 인터페이스 설계 (Phase 2)

## 서론

WIA-BEMS Phase 2는 표준화된 API 인터페이스를 통해 다양한 시스템 간의 상호운용성을 확보합니다. 본 장에서는 RESTful API 설계 원칙, 인증/인가 메커니즘, 실시간 데이터 스트리밍, 그리고 클라이언트 SDK 개발 방법을 상세히 다룹니다.

---

## 4.1 RESTful API 설계 원칙

### 4.1.1 API 아키텍처 개요

```typescript
// WIA-BEMS API 아키텍처
interface WIABEMSAPIArchitecture {
  apiGateway: {
    role: 'API 진입점 및 공통 기능 처리';
    functions: [
      '요청 라우팅',
      '인증/인가',
      '속도 제한',
      '로깅 및 모니터링',
      '버전 관리',
      '캐싱'
    ];
    technologies: ['Kong', 'AWS API Gateway', 'Azure APIM', 'Nginx'];
  };

  microservices: {
    buildingService: '건물 정보 관리';
    energyService: '에너지 데이터 관리';
    equipmentService: '설비 데이터 관리';
    analyticsService: '분석 및 리포팅';
    controlService: '제어 명령 관리';
    alertService: '알람 및 알림 관리';
    userService: '사용자 및 권한 관리';
  };

  dataFlow: {
    synchronous: {
      protocol: 'REST over HTTPS';
      format: 'JSON';
      useCase: 'CRUD 작업, 쿼리';
    };
    asynchronous: {
      protocol: 'WebSocket, MQTT';
      format: 'JSON';
      useCase: '실시간 데이터, 이벤트';
    };
    batch: {
      protocol: 'REST with pagination';
      format: 'JSON, CSV, Parquet';
      useCase: '대량 데이터 내보내기';
    };
  };
}

// API 버전 관리 전략
interface APIVersioningStrategy {
  strategy: 'URL Path Versioning';
  format: '/api/v{major}/resource';
  rules: {
    majorVersion: '하위 호환성 깨지는 변경';
    minorVersion: '하위 호환 기능 추가 (헤더로 표시)';
    patchVersion: '버그 수정 (변경 없음)';
  };
  deprecation: {
    policy: '새 버전 출시 후 12개월간 이전 버전 지원';
    notification: 'Sunset 헤더 및 문서 공지';
  };
  example: {
    currentVersion: 'v1';
    endpoints: [
      '/api/v1/buildings',
      '/api/v1/energy',
      '/api/v1/equipment'
    ];
  };
}
```

### 4.1.2 API 엔드포인트 설계

```typescript
// 건물 API 명세
interface BuildingAPISpecification {
  baseUrl: '/api/v1/buildings';

  endpoints: {
    // 건물 목록 조회
    listBuildings: {
      method: 'GET';
      path: '/';
      description: '건물 목록을 조회합니다';
      queryParams: {
        page: { type: 'integer', default: 1, description: '페이지 번호' };
        limit: { type: 'integer', default: 20, max: 100, description: '페이지당 항목 수' };
        sort: { type: 'string', enum: ['name', 'createdAt', 'grossFloorArea'], description: '정렬 기준' };
        order: { type: 'string', enum: ['asc', 'desc'], default: 'asc' };
        buildingType: { type: 'string', description: '건물 유형 필터' };
        city: { type: 'string', description: '도시 필터' };
        search: { type: 'string', description: '건물명 검색' };
      };
      response: {
        200: {
          data: 'Building[]';
          pagination: 'PaginationInfo';
        };
      };
    };

    // 건물 상세 조회
    getBuilding: {
      method: 'GET';
      path: '/{buildingId}';
      description: '특정 건물의 상세 정보를 조회합니다';
      pathParams: {
        buildingId: { type: 'string', required: true };
      };
      queryParams: {
        include: {
          type: 'string[]',
          description: '포함할 관계 데이터',
          enum: ['systems', 'equipment', 'meters', 'zones', 'certifications']
        };
      };
      response: {
        200: 'Building';
        404: 'NotFoundError';
      };
    };

    // 건물 생성
    createBuilding: {
      method: 'POST';
      path: '/';
      description: '새 건물을 생성합니다';
      requestBody: 'CreateBuildingRequest';
      response: {
        201: 'Building';
        400: 'ValidationError';
        409: 'ConflictError';
      };
    };

    // 건물 수정
    updateBuilding: {
      method: 'PATCH';
      path: '/{buildingId}';
      description: '건물 정보를 수정합니다';
      pathParams: {
        buildingId: { type: 'string', required: true };
      };
      requestBody: 'UpdateBuildingRequest';
      response: {
        200: 'Building';
        400: 'ValidationError';
        404: 'NotFoundError';
      };
    };

    // 건물 삭제
    deleteBuilding: {
      method: 'DELETE';
      path: '/{buildingId}';
      description: '건물을 삭제합니다';
      pathParams: {
        buildingId: { type: 'string', required: true };
      };
      response: {
        204: 'No Content';
        404: 'NotFoundError';
        409: 'ConflictError (관련 데이터 존재)';
      };
    };

    // 건물 에너지 요약
    getBuildingEnergySummary: {
      method: 'GET';
      path: '/{buildingId}/energy/summary';
      description: '건물 에너지 사용 요약을 조회합니다';
      pathParams: {
        buildingId: { type: 'string', required: true };
      };
      queryParams: {
        period: { type: 'string', enum: ['day', 'week', 'month', 'year'] };
        startDate: { type: 'string', format: 'date' };
        endDate: { type: 'string', format: 'date' };
      };
      response: {
        200: 'EnergySummary';
      };
    };

    // 건물 설비 목록
    getBuildingEquipment: {
      method: 'GET';
      path: '/{buildingId}/equipment';
      description: '건물 내 설비 목록을 조회합니다';
      pathParams: {
        buildingId: { type: 'string', required: true };
      };
      queryParams: {
        type: { type: 'string', description: '설비 유형 필터' };
        status: { type: 'string', enum: ['running', 'stopped', 'fault'] };
        floor: { type: 'string' };
      };
      response: {
        200: 'Equipment[]';
      };
    };
  };
}

// 에너지 데이터 API 명세
interface EnergyDataAPISpecification {
  baseUrl: '/api/v1/energy';

  endpoints: {
    // 실시간 에너지 데이터
    getCurrentEnergy: {
      method: 'GET';
      path: '/current';
      description: '현재 에너지 사용량을 조회합니다';
      queryParams: {
        buildingId: { type: 'string', required: true };
        meterId: { type: 'string' };
        meterType: { type: 'string' };
      };
      response: {
        200: 'CurrentEnergyData';
      };
    };

    // 시계열 에너지 데이터
    getEnergyTimeSeries: {
      method: 'GET';
      path: '/timeseries';
      description: '에너지 시계열 데이터를 조회합니다';
      queryParams: {
        buildingId: { type: 'string', required: true };
        meterId: { type: 'string' };
        startTime: { type: 'string', format: 'date-time', required: true };
        endTime: { type: 'string', format: 'date-time', required: true };
        interval: { type: 'string', enum: ['1m', '5m', '15m', '30m', '1h', '1d'], default: '15m' };
        aggregation: { type: 'string', enum: ['sum', 'avg', 'min', 'max', 'delta'], default: 'sum' };
        metrics: { type: 'string[]', description: '조회할 메트릭', example: ['activeEnergy', 'activePower'] };
      };
      response: {
        200: 'EnergyTimeSeries';
      };
    };

    // 에너지 데이터 대량 내보내기
    exportEnergyData: {
      method: 'POST';
      path: '/export';
      description: '대량 에너지 데이터를 내보냅니다 (비동기)';
      requestBody: {
        buildingIds: 'string[]';
        startTime: 'string';
        endTime: 'string';
        interval: 'string';
        format: 'csv | parquet | json';
        deliveryMethod: 'download | s3 | email';
      };
      response: {
        202: 'ExportJobCreated';
      };
    };

    // 에너지 비교 분석
    getEnergyComparison: {
      method: 'GET';
      path: '/comparison';
      description: '에너지 사용량을 비교 분석합니다';
      queryParams: {
        buildingId: { type: 'string', required: true };
        compareWith: { type: 'string', enum: ['previous_period', 'same_period_last_year', 'baseline', 'benchmark'] };
        period: { type: 'string', enum: ['day', 'week', 'month', 'year'] };
        startDate: { type: 'string', format: 'date' };
      };
      response: {
        200: 'EnergyComparison';
      };
    };

    // 피크 수요 분석
    getPeakDemand: {
      method: 'GET';
      path: '/peak-demand';
      description: '피크 수요 데이터를 조회합니다';
      queryParams: {
        buildingId: { type: 'string', required: true };
        period: { type: 'string', enum: ['day', 'month', 'year'] };
        startDate: { type: 'string', format: 'date' };
        endDate: { type: 'string', format: 'date' };
        top: { type: 'integer', default: 10, description: '상위 N개 피크' };
      };
      response: {
        200: 'PeakDemandAnalysis';
      };
    };
  };
}
```

### 4.1.3 API 응답 형식

```typescript
// 표준 API 응답 형식
interface StandardAPIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMeta;
}

interface APIError {
  code: string;
  message: string;
  details?: ErrorDetail[];
  requestId: string;
  timestamp: string;
}

interface ErrorDetail {
  field?: string;
  message: string;
  code: string;
}

interface ResponseMeta {
  requestId: string;
  timestamp: string;
  processingTime: number; // milliseconds
  pagination?: PaginationInfo;
  rateLimit?: RateLimitInfo;
}

interface PaginationInfo {
  page: number;
  limit: number;
  totalItems: number;
  totalPages: number;
  hasNextPage: boolean;
  hasPrevPage: boolean;
  links: {
    self: string;
    first: string;
    last: string;
    next?: string;
    prev?: string;
  };
}

interface RateLimitInfo {
  limit: number;
  remaining: number;
  reset: number; // Unix timestamp
}

// 에러 코드 정의
const ErrorCodes = {
  // 인증/인가 에러 (AUTH_xxx)
  AUTH_INVALID_TOKEN: 'AUTH_001',
  AUTH_EXPIRED_TOKEN: 'AUTH_002',
  AUTH_INSUFFICIENT_PERMISSIONS: 'AUTH_003',
  AUTH_INVALID_API_KEY: 'AUTH_004',

  // 유효성 검증 에러 (VAL_xxx)
  VAL_REQUIRED_FIELD: 'VAL_001',
  VAL_INVALID_FORMAT: 'VAL_002',
  VAL_OUT_OF_RANGE: 'VAL_003',
  VAL_INVALID_ENUM: 'VAL_004',

  // 리소스 에러 (RES_xxx)
  RES_NOT_FOUND: 'RES_001',
  RES_ALREADY_EXISTS: 'RES_002',
  RES_CONFLICT: 'RES_003',
  RES_GONE: 'RES_004',

  // 서버 에러 (SRV_xxx)
  SRV_INTERNAL_ERROR: 'SRV_001',
  SRV_SERVICE_UNAVAILABLE: 'SRV_002',
  SRV_TIMEOUT: 'SRV_003',
  SRV_RATE_LIMITED: 'SRV_004'
};

// 응답 예시
const successResponseExample = {
  success: true,
  data: {
    buildingId: 'BLD-ABCD1234',
    name: '스마트타워',
    location: {
      city: '서울특별시',
      district: '강남구'
    },
    grossFloorArea: 85000,
    energyProfile: {
      eui: 165,
      annualConsumption: 12500000
    }
  },
  meta: {
    requestId: 'req-550e8400-e29b-41d4-a716-446655440000',
    timestamp: '2024-11-15T10:30:00Z',
    processingTime: 45
  }
};

const errorResponseExample = {
  success: false,
  error: {
    code: 'VAL_001',
    message: '유효성 검증 오류가 발생했습니다',
    details: [
      {
        field: 'grossFloorArea',
        message: '연면적은 0보다 커야 합니다',
        code: 'VAL_OUT_OF_RANGE'
      },
      {
        field: 'buildingType',
        message: '유효하지 않은 건물 유형입니다',
        code: 'VAL_INVALID_ENUM'
      }
    ],
    requestId: 'req-550e8400-e29b-41d4-a716-446655440001',
    timestamp: '2024-11-15T10:31:00Z'
  }
};

const paginatedResponseExample = {
  success: true,
  data: [
    { buildingId: 'BLD-001', name: '건물 1' },
    { buildingId: 'BLD-002', name: '건물 2' }
  ],
  meta: {
    requestId: 'req-xxx',
    timestamp: '2024-11-15T10:32:00Z',
    processingTime: 120,
    pagination: {
      page: 2,
      limit: 20,
      totalItems: 156,
      totalPages: 8,
      hasNextPage: true,
      hasPrevPage: true,
      links: {
        self: '/api/v1/buildings?page=2&limit=20',
        first: '/api/v1/buildings?page=1&limit=20',
        last: '/api/v1/buildings?page=8&limit=20',
        next: '/api/v1/buildings?page=3&limit=20',
        prev: '/api/v1/buildings?page=1&limit=20'
      }
    }
  }
};
```

---

## 4.2 인증 및 인가

### 4.2.1 OAuth 2.0 인증

```typescript
// OAuth 2.0 인증 구현
interface OAuth2Configuration {
  authorizationServer: {
    issuer: 'https://auth.wia-bems.org';
    authorizationEndpoint: '/oauth/authorize';
    tokenEndpoint: '/oauth/token';
    revocationEndpoint: '/oauth/revoke';
    introspectionEndpoint: '/oauth/introspect';
    jwksUri: '/.well-known/jwks.json';
    userInfoEndpoint: '/userinfo';
  };

  supportedGrantTypes: [
    'authorization_code',
    'client_credentials',
    'refresh_token'
  ];

  supportedScopes: {
    'buildings:read': '건물 정보 읽기';
    'buildings:write': '건물 정보 쓰기';
    'energy:read': '에너지 데이터 읽기';
    'energy:write': '에너지 데이터 쓰기';
    'equipment:read': '설비 정보 읽기';
    'equipment:write': '설비 정보 쓰기';
    'control:execute': '제어 명령 실행';
    'alerts:read': '알람 읽기';
    'alerts:manage': '알람 관리';
    'admin': '관리자 권한';
  };

  tokenConfiguration: {
    accessTokenLifetime: 3600; // 1시간
    refreshTokenLifetime: 604800; // 7일
    authorizationCodeLifetime: 600; // 10분
  };
}

// OAuth 2.0 토큰 발급 예시
class OAuth2Service {
  async issueToken(grantRequest: TokenRequest): Promise<TokenResponse> {
    switch (grantRequest.grant_type) {
      case 'authorization_code':
        return this.handleAuthorizationCodeGrant(grantRequest);
      case 'client_credentials':
        return this.handleClientCredentialsGrant(grantRequest);
      case 'refresh_token':
        return this.handleRefreshTokenGrant(grantRequest);
      default:
        throw new OAuth2Error('unsupported_grant_type');
    }
  }

  private async handleClientCredentialsGrant(
    request: ClientCredentialsRequest
  ): Promise<TokenResponse> {
    // 클라이언트 인증
    const client = await this.validateClient(
      request.client_id,
      request.client_secret
    );

    if (!client) {
      throw new OAuth2Error('invalid_client');
    }

    // 요청된 스코프 검증
    const grantedScopes = this.validateScopes(
      request.scope,
      client.allowedScopes
    );

    // 액세스 토큰 생성
    const accessToken = await this.generateAccessToken({
      clientId: client.clientId,
      scopes: grantedScopes,
      tokenType: 'client_credentials'
    });

    return {
      access_token: accessToken.token,
      token_type: 'Bearer',
      expires_in: this.config.accessTokenLifetime,
      scope: grantedScopes.join(' ')
    };
  }

  private async generateAccessToken(payload: TokenPayload): Promise<GeneratedToken> {
    const now = Math.floor(Date.now() / 1000);

    const claims = {
      iss: this.config.issuer,
      sub: payload.clientId,
      aud: 'wia-bems-api',
      exp: now + this.config.accessTokenLifetime,
      iat: now,
      jti: generateUUID(),
      scope: payload.scopes.join(' '),
      client_id: payload.clientId,
      token_type: payload.tokenType
    };

    const token = jwt.sign(claims, this.privateKey, {
      algorithm: 'RS256',
      keyid: this.currentKeyId
    });

    // 토큰 메타데이터 저장 (revocation 지원)
    await this.tokenStore.save({
      jti: claims.jti,
      clientId: payload.clientId,
      expiresAt: new Date(claims.exp * 1000),
      scopes: payload.scopes,
      revoked: false
    });

    return { token, claims };
  }
}

// JWT 토큰 검증
class TokenValidator {
  private jwksClient: JwksClient;
  private tokenStore: TokenStore;

  async validateToken(token: string): Promise<ValidatedToken> {
    // 1. JWT 파싱 및 서명 검증
    const decoded = await this.verifyJWT(token);

    // 2. 만료 시간 확인
    if (decoded.exp < Math.floor(Date.now() / 1000)) {
      throw new TokenError('token_expired');
    }

    // 3. 토큰 폐기 여부 확인
    const tokenRecord = await this.tokenStore.findByJti(decoded.jti);
    if (tokenRecord?.revoked) {
      throw new TokenError('token_revoked');
    }

    // 4. 발급자 확인
    if (decoded.iss !== this.config.issuer) {
      throw new TokenError('invalid_issuer');
    }

    return {
      clientId: decoded.client_id,
      scopes: decoded.scope.split(' '),
      expiresAt: new Date(decoded.exp * 1000),
      tokenType: decoded.token_type
    };
  }

  private async verifyJWT(token: string): Promise<JWTPayload> {
    const decodedHeader = jwt.decode(token, { complete: true });

    if (!decodedHeader || !decodedHeader.header.kid) {
      throw new TokenError('invalid_token_format');
    }

    const key = await this.jwksClient.getSigningKey(decodedHeader.header.kid);
    const publicKey = key.getPublicKey();

    return jwt.verify(token, publicKey, {
      algorithms: ['RS256'],
      issuer: this.config.issuer
    }) as JWTPayload;
  }
}
```

### 4.2.2 권한 관리 (RBAC)

```typescript
// 역할 기반 접근 제어
interface RBACConfiguration {
  roles: {
    admin: {
      name: '관리자';
      description: '시스템 전체 관리 권한';
      permissions: ['*'];
    };
    building_manager: {
      name: '건물 관리자';
      description: '특정 건물의 관리 권한';
      permissions: [
        'buildings:read',
        'buildings:write',
        'energy:read',
        'equipment:read',
        'equipment:write',
        'alerts:read',
        'alerts:manage'
      ];
    };
    energy_manager: {
      name: '에너지 관리자';
      description: '에너지 데이터 및 분석 권한';
      permissions: [
        'buildings:read',
        'energy:read',
        'energy:write',
        'equipment:read',
        'alerts:read'
      ];
    };
    operator: {
      name: '운영자';
      description: '일상 운영 권한';
      permissions: [
        'buildings:read',
        'energy:read',
        'equipment:read',
        'control:execute',
        'alerts:read'
      ];
    };
    viewer: {
      name: '열람자';
      description: '읽기 전용 권한';
      permissions: [
        'buildings:read',
        'energy:read',
        'equipment:read',
        'alerts:read'
      ];
    };
    api_client: {
      name: 'API 클라이언트';
      description: '외부 시스템 연동용';
      permissions: [
        'energy:read',
        'equipment:read'
      ];
    };
  };

  resourceScoping: {
    type: 'building' | 'portfolio' | 'global';
    description: '권한 범위를 특정 건물/포트폴리오로 제한';
  };
}

// 권한 검증 미들웨어
class AuthorizationMiddleware {
  async authorize(
    request: AuthenticatedRequest,
    requiredPermissions: string[],
    resourceScope?: ResourceScope
  ): Promise<void> {
    const userPermissions = await this.getUserPermissions(
      request.user,
      resourceScope
    );

    for (const required of requiredPermissions) {
      if (!this.hasPermission(userPermissions, required)) {
        throw new AuthorizationError(
          `권한이 부족합니다: ${required}`,
          'AUTH_INSUFFICIENT_PERMISSIONS'
        );
      }
    }

    // 리소스 스코프 검증
    if (resourceScope) {
      const hasAccess = await this.checkResourceAccess(
        request.user,
        resourceScope
      );

      if (!hasAccess) {
        throw new AuthorizationError(
          '해당 리소스에 대한 접근 권한이 없습니다',
          'AUTH_RESOURCE_ACCESS_DENIED'
        );
      }
    }
  }

  private hasPermission(
    userPermissions: string[],
    required: string
  ): boolean {
    // 와일드카드 권한 체크
    if (userPermissions.includes('*')) {
      return true;
    }

    // 정확한 매칭
    if (userPermissions.includes(required)) {
      return true;
    }

    // 계층적 권한 체크 (예: buildings:* 가 buildings:read 포함)
    const [resource, action] = required.split(':');
    if (userPermissions.includes(`${resource}:*`)) {
      return true;
    }

    return false;
  }

  private async checkResourceAccess(
    user: AuthenticatedUser,
    scope: ResourceScope
  ): Promise<boolean> {
    // 전역 권한 확인
    if (user.scope === 'global') {
      return true;
    }

    // 포트폴리오 범위 확인
    if (user.scope === 'portfolio') {
      const portfolioBuildings = await this.getPortfolioBuildings(
        user.portfolioId
      );
      return portfolioBuildings.includes(scope.buildingId);
    }

    // 개별 건물 범위 확인
    if (user.scope === 'building') {
      return user.buildingIds.includes(scope.buildingId);
    }

    return false;
  }
}

// API 키 인증
interface APIKeyAuthentication {
  keyFormat: 'wia_live_xxxx' | 'wia_test_xxxx';
  keyLength: 32;
  hashAlgorithm: 'SHA-256';
  storage: 'hashed_in_database';

  rateLimits: {
    basic: { requestsPerMinute: 60, requestsPerDay: 10000 };
    standard: { requestsPerMinute: 600, requestsPerDay: 100000 };
    enterprise: { requestsPerMinute: 6000, requestsPerDay: 1000000 };
  };
}

class APIKeyService {
  async generateAPIKey(
    userId: string,
    keyName: string,
    scopes: string[],
    tier: 'basic' | 'standard' | 'enterprise'
  ): Promise<APIKeyResponse> {
    const prefix = process.env.NODE_ENV === 'production' ? 'wia_live_' : 'wia_test_';
    const rawKey = prefix + generateSecureRandom(32);
    const hashedKey = await bcrypt.hash(rawKey, 10);

    const keyRecord = await this.keyStore.create({
      keyId: generateUUID(),
      userId,
      keyName,
      hashedKey,
      scopes,
      tier,
      createdAt: new Date(),
      lastUsedAt: null,
      expiresAt: null, // 만료 없음, 또는 설정 가능
      active: true
    });

    return {
      keyId: keyRecord.keyId,
      apiKey: rawKey, // 이 시점에만 원본 키 반환
      keyName,
      scopes,
      tier,
      createdAt: keyRecord.createdAt
    };
  }

  async validateAPIKey(apiKey: string): Promise<APIKeyValidation> {
    // 접두사 확인
    if (!apiKey.startsWith('wia_live_') && !apiKey.startsWith('wia_test_')) {
      throw new AuthenticationError('유효하지 않은 API 키 형식');
    }

    // 모든 활성 키에 대해 해시 비교 (실제로는 캐시 활용)
    const keys = await this.keyStore.findActiveKeys();

    for (const key of keys) {
      if (await bcrypt.compare(apiKey, key.hashedKey)) {
        // 마지막 사용 시간 업데이트
        await this.keyStore.updateLastUsed(key.keyId);

        return {
          keyId: key.keyId,
          userId: key.userId,
          scopes: key.scopes,
          tier: key.tier
        };
      }
    }

    throw new AuthenticationError('API 키를 찾을 수 없습니다');
  }
}
```

---

## 4.3 실시간 데이터 스트리밍

### 4.3.1 WebSocket API

```typescript
// WebSocket 연결 관리
interface WebSocketConfiguration {
  endpoint: 'wss://api.wia-bems.org/ws/v1';
  heartbeatInterval: 30000; // 30초
  reconnectStrategy: {
    maxRetries: 5;
    backoffMultiplier: 2;
    initialDelay: 1000;
    maxDelay: 30000;
  };
  messageFormat: 'JSON';
  compression: 'permessage-deflate';
}

// WebSocket 메시지 프로토콜
interface WebSocketProtocol {
  messageTypes: {
    // 클라이언트 → 서버
    subscribe: {
      type: 'subscribe';
      channel: string;
      params?: Record<string, any>;
    };
    unsubscribe: {
      type: 'unsubscribe';
      channel: string;
    };
    ping: {
      type: 'ping';
      timestamp: number;
    };

    // 서버 → 클라이언트
    subscribed: {
      type: 'subscribed';
      channel: string;
      subscriptionId: string;
    };
    unsubscribed: {
      type: 'unsubscribed';
      channel: string;
    };
    data: {
      type: 'data';
      channel: string;
      payload: any;
      timestamp: string;
    };
    error: {
      type: 'error';
      code: string;
      message: string;
    };
    pong: {
      type: 'pong';
      timestamp: number;
    };
  };

  channels: {
    'energy.realtime.{buildingId}': '실시간 에너지 데이터';
    'equipment.status.{equipmentId}': '설비 상태 변경';
    'alerts.{buildingId}': '알람 이벤트';
    'environment.{zoneId}': '환경 센서 데이터';
    'control.response.{buildingId}': '제어 명령 응답';
  };
}

// WebSocket 서버 구현
class WebSocketServer {
  private wss: WebSocket.Server;
  private subscriptions: Map<string, Set<WebSocket>>;
  private clientInfo: Map<WebSocket, ClientInfo>;

  constructor(server: http.Server) {
    this.wss = new WebSocket.Server({ server, path: '/ws/v1' });
    this.subscriptions = new Map();
    this.clientInfo = new Map();

    this.wss.on('connection', this.handleConnection.bind(this));
  }

  private async handleConnection(ws: WebSocket, request: http.IncomingMessage) {
    // 인증 검증
    const token = this.extractToken(request);
    let clientAuth: AuthenticatedClient;

    try {
      clientAuth = await this.authenticateClient(token);
    } catch (error) {
      ws.close(4001, 'Authentication failed');
      return;
    }

    // 클라이언트 정보 저장
    this.clientInfo.set(ws, {
      clientId: clientAuth.clientId,
      scopes: clientAuth.scopes,
      connectedAt: new Date(),
      subscriptions: new Set()
    });

    // 메시지 핸들러 등록
    ws.on('message', (data) => this.handleMessage(ws, data));
    ws.on('close', () => this.handleDisconnect(ws));
    ws.on('error', (error) => this.handleError(ws, error));

    // 연결 성공 메시지
    this.send(ws, {
      type: 'connected',
      clientId: clientAuth.clientId,
      timestamp: new Date().toISOString()
    });

    // 하트비트 시작
    this.startHeartbeat(ws);
  }

  private async handleMessage(ws: WebSocket, data: WebSocket.Data) {
    let message: any;

    try {
      message = JSON.parse(data.toString());
    } catch {
      this.send(ws, { type: 'error', code: 'INVALID_JSON', message: '유효하지 않은 JSON' });
      return;
    }

    const clientInfo = this.clientInfo.get(ws);
    if (!clientInfo) return;

    switch (message.type) {
      case 'subscribe':
        await this.handleSubscribe(ws, clientInfo, message);
        break;
      case 'unsubscribe':
        await this.handleUnsubscribe(ws, clientInfo, message);
        break;
      case 'ping':
        this.send(ws, { type: 'pong', timestamp: Date.now() });
        break;
      default:
        this.send(ws, { type: 'error', code: 'UNKNOWN_TYPE', message: '알 수 없는 메시지 유형' });
    }
  }

  private async handleSubscribe(
    ws: WebSocket,
    clientInfo: ClientInfo,
    message: SubscribeMessage
  ) {
    const { channel, params } = message;

    // 권한 검증
    const requiredScope = this.getRequiredScope(channel);
    if (!clientInfo.scopes.includes(requiredScope)) {
      this.send(ws, {
        type: 'error',
        code: 'PERMISSION_DENIED',
        message: `채널 ${channel}에 대한 권한이 없습니다`
      });
      return;
    }

    // 구독 등록
    if (!this.subscriptions.has(channel)) {
      this.subscriptions.set(channel, new Set());
    }
    this.subscriptions.get(channel)!.add(ws);
    clientInfo.subscriptions.add(channel);

    // 구독 확인 메시지
    this.send(ws, {
      type: 'subscribed',
      channel,
      subscriptionId: generateUUID()
    });

    // 초기 데이터 전송 (선택적)
    if (params?.sendInitial !== false) {
      const initialData = await this.getInitialData(channel);
      if (initialData) {
        this.send(ws, {
          type: 'data',
          channel,
          payload: initialData,
          timestamp: new Date().toISOString()
        });
      }
    }
  }

  // 채널에 데이터 발행
  publish(channel: string, payload: any) {
    const subscribers = this.subscriptions.get(channel);
    if (!subscribers || subscribers.size === 0) return;

    const message = {
      type: 'data',
      channel,
      payload,
      timestamp: new Date().toISOString()
    };

    const data = JSON.stringify(message);

    for (const ws of subscribers) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(data);
      }
    }
  }

  private send(ws: WebSocket, message: any) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }
}

// WebSocket 클라이언트 SDK
class WIABEMSWebSocketClient {
  private ws: WebSocket | null = null;
  private subscriptions: Map<string, SubscriptionCallback[]>;
  private reconnectAttempts: number = 0;
  private heartbeatTimer: NodeJS.Timer | null = null;

  constructor(
    private config: WebSocketClientConfig
  ) {
    this.subscriptions = new Map();
  }

  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const url = `${this.config.endpoint}?token=${this.config.accessToken}`;
      this.ws = new WebSocket(url);

      this.ws.onopen = () => {
        console.log('WebSocket 연결됨');
        this.reconnectAttempts = 0;
        this.startHeartbeat();
        resolve();
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(JSON.parse(event.data));
      };

      this.ws.onclose = (event) => {
        console.log('WebSocket 연결 종료:', event.code, event.reason);
        this.stopHeartbeat();
        this.attemptReconnect();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket 오류:', error);
        reject(error);
      };
    });
  }

  subscribe(channel: string, callback: SubscriptionCallback): () => void {
    // 구독 메시지 전송
    this.send({
      type: 'subscribe',
      channel,
      params: {}
    });

    // 콜백 등록
    if (!this.subscriptions.has(channel)) {
      this.subscriptions.set(channel, []);
    }
    this.subscriptions.get(channel)!.push(callback);

    // 구독 해제 함수 반환
    return () => {
      const callbacks = this.subscriptions.get(channel);
      if (callbacks) {
        const index = callbacks.indexOf(callback);
        if (index > -1) {
          callbacks.splice(index, 1);
        }
        if (callbacks.length === 0) {
          this.send({ type: 'unsubscribe', channel });
          this.subscriptions.delete(channel);
        }
      }
    };
  }

  private handleMessage(message: any) {
    switch (message.type) {
      case 'data':
        const callbacks = this.subscriptions.get(message.channel);
        if (callbacks) {
          for (const callback of callbacks) {
            try {
              callback(message.payload, message.timestamp);
            } catch (error) {
              console.error('구독 콜백 오류:', error);
            }
          }
        }
        break;
      case 'error':
        console.error('서버 오류:', message.code, message.message);
        break;
      case 'pong':
        // 하트비트 응답 처리
        break;
    }
  }

  private send(message: any) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  private startHeartbeat() {
    this.heartbeatTimer = setInterval(() => {
      this.send({ type: 'ping', timestamp: Date.now() });
    }, 30000);
  }

  private stopHeartbeat() {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  private async attemptReconnect() {
    if (this.reconnectAttempts >= this.config.maxReconnectAttempts) {
      console.error('최대 재연결 시도 횟수 초과');
      return;
    }

    const delay = Math.min(
      this.config.initialReconnectDelay * Math.pow(2, this.reconnectAttempts),
      this.config.maxReconnectDelay
    );

    this.reconnectAttempts++;
    console.log(`${delay}ms 후 재연결 시도 (${this.reconnectAttempts}/${this.config.maxReconnectAttempts})`);

    await new Promise(resolve => setTimeout(resolve, delay));
    await this.connect();

    // 기존 구독 복원
    for (const channel of this.subscriptions.keys()) {
      this.send({ type: 'subscribe', channel, params: {} });
    }
  }

  disconnect() {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
  }
}
```

---

## 4.4 클라이언트 SDK

### 4.4.1 TypeScript SDK

```typescript
// WIA-BEMS TypeScript SDK
class WIABEMSClient {
  private baseUrl: string;
  private accessToken: string;
  private httpClient: AxiosInstance;
  private wsClient: WIABEMSWebSocketClient | null = null;

  constructor(config: WIABEMSClientConfig) {
    this.baseUrl = config.baseUrl || 'https://api.wia-bems.org';
    this.accessToken = config.accessToken;

    this.httpClient = axios.create({
      baseURL: this.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json',
        'X-API-Version': 'v1'
      }
    });

    // 응답 인터셉터
    this.httpClient.interceptors.response.use(
      (response) => response,
      (error) => this.handleError(error)
    );
  }

  // 건물 API
  buildings = {
    list: async (params?: ListBuildingsParams): Promise<PaginatedResponse<Building>> => {
      const response = await this.httpClient.get('/api/v1/buildings', { params });
      return response.data;
    },

    get: async (buildingId: string, include?: string[]): Promise<Building> => {
      const response = await this.httpClient.get(`/api/v1/buildings/${buildingId}`, {
        params: { include: include?.join(',') }
      });
      return response.data.data;
    },

    create: async (building: CreateBuildingRequest): Promise<Building> => {
      const response = await this.httpClient.post('/api/v1/buildings', building);
      return response.data.data;
    },

    update: async (buildingId: string, updates: UpdateBuildingRequest): Promise<Building> => {
      const response = await this.httpClient.patch(`/api/v1/buildings/${buildingId}`, updates);
      return response.data.data;
    },

    delete: async (buildingId: string): Promise<void> => {
      await this.httpClient.delete(`/api/v1/buildings/${buildingId}`);
    },

    getEnergySummary: async (
      buildingId: string,
      params: EnergySummaryParams
    ): Promise<EnergySummary> => {
      const response = await this.httpClient.get(
        `/api/v1/buildings/${buildingId}/energy/summary`,
        { params }
      );
      return response.data.data;
    }
  };

  // 에너지 API
  energy = {
    getCurrent: async (params: CurrentEnergyParams): Promise<CurrentEnergyData> => {
      const response = await this.httpClient.get('/api/v1/energy/current', { params });
      return response.data.data;
    },

    getTimeSeries: async (params: TimeSeriesParams): Promise<EnergyTimeSeries> => {
      const response = await this.httpClient.get('/api/v1/energy/timeseries', { params });
      return response.data.data;
    },

    exportData: async (request: ExportDataRequest): Promise<ExportJobResponse> => {
      const response = await this.httpClient.post('/api/v1/energy/export', request);
      return response.data.data;
    },

    getComparison: async (params: ComparisonParams): Promise<EnergyComparison> => {
      const response = await this.httpClient.get('/api/v1/energy/comparison', { params });
      return response.data.data;
    }
  };

  // 설비 API
  equipment = {
    list: async (params?: ListEquipmentParams): Promise<PaginatedResponse<Equipment>> => {
      const response = await this.httpClient.get('/api/v1/equipment', { params });
      return response.data;
    },

    get: async (equipmentId: string): Promise<Equipment> => {
      const response = await this.httpClient.get(`/api/v1/equipment/${equipmentId}`);
      return response.data.data;
    },

    getStatus: async (equipmentId: string): Promise<EquipmentStatus> => {
      const response = await this.httpClient.get(`/api/v1/equipment/${equipmentId}/status`);
      return response.data.data;
    },

    sendCommand: async (
      equipmentId: string,
      command: EquipmentCommand
    ): Promise<CommandResponse> => {
      const response = await this.httpClient.post(
        `/api/v1/equipment/${equipmentId}/commands`,
        command
      );
      return response.data.data;
    }
  };

  // 알람 API
  alerts = {
    list: async (params?: ListAlertsParams): Promise<PaginatedResponse<Alert>> => {
      const response = await this.httpClient.get('/api/v1/alerts', { params });
      return response.data;
    },

    acknowledge: async (alertId: string, note?: string): Promise<Alert> => {
      const response = await this.httpClient.post(`/api/v1/alerts/${alertId}/acknowledge`, { note });
      return response.data.data;
    },

    resolve: async (alertId: string, resolution: string): Promise<Alert> => {
      const response = await this.httpClient.post(`/api/v1/alerts/${alertId}/resolve`, { resolution });
      return response.data.data;
    }
  };

  // 실시간 데이터 구독
  async connectRealtime(): Promise<void> {
    this.wsClient = new WIABEMSWebSocketClient({
      endpoint: this.baseUrl.replace('https', 'wss') + '/ws/v1',
      accessToken: this.accessToken,
      maxReconnectAttempts: 5,
      initialReconnectDelay: 1000,
      maxReconnectDelay: 30000
    });

    await this.wsClient.connect();
  }

  subscribeToEnergy(buildingId: string, callback: (data: EnergyData) => void): () => void {
    if (!this.wsClient) {
      throw new Error('실시간 연결이 필요합니다. connectRealtime()을 먼저 호출하세요.');
    }
    return this.wsClient.subscribe(`energy.realtime.${buildingId}`, callback);
  }

  subscribeToEquipmentStatus(
    equipmentId: string,
    callback: (status: EquipmentStatus) => void
  ): () => void {
    if (!this.wsClient) {
      throw new Error('실시간 연결이 필요합니다.');
    }
    return this.wsClient.subscribe(`equipment.status.${equipmentId}`, callback);
  }

  subscribeToAlerts(buildingId: string, callback: (alert: Alert) => void): () => void {
    if (!this.wsClient) {
      throw new Error('실시간 연결이 필요합니다.');
    }
    return this.wsClient.subscribe(`alerts.${buildingId}`, callback);
  }

  disconnectRealtime() {
    if (this.wsClient) {
      this.wsClient.disconnect();
      this.wsClient = null;
    }
  }

  private handleError(error: any): never {
    if (error.response) {
      const { status, data } = error.response;
      throw new WIABEMSError(
        data.error?.message || '알 수 없는 오류',
        data.error?.code || 'UNKNOWN',
        status,
        data.error?.details
      );
    }
    throw new WIABEMSError('네트워크 오류', 'NETWORK_ERROR', 0);
  }
}

// 에러 클래스
class WIABEMSError extends Error {
  constructor(
    message: string,
    public code: string,
    public status: number,
    public details?: any[]
  ) {
    super(message);
    this.name = 'WIABEMSError';
  }
}

// 사용 예시
async function exampleUsage() {
  const client = new WIABEMSClient({
    baseUrl: 'https://api.wia-bems.org',
    accessToken: 'your_access_token'
  });

  // 건물 목록 조회
  const buildings = await client.buildings.list({
    page: 1,
    limit: 20,
    buildingType: 'office'
  });

  console.log(`총 ${buildings.meta.pagination.totalItems}개 건물`);

  // 특정 건물 에너지 데이터 조회
  const energyData = await client.energy.getTimeSeries({
    buildingId: buildings.data[0].buildingId,
    startTime: '2024-11-01T00:00:00Z',
    endTime: '2024-11-15T00:00:00Z',
    interval: '1h',
    metrics: ['activeEnergy', 'activePower']
  });

  // 실시간 데이터 구독
  await client.connectRealtime();

  const unsubscribe = client.subscribeToEnergy(
    buildings.data[0].buildingId,
    (data) => {
      console.log('실시간 에너지 데이터:', data);
    }
  );

  // 알람 구독
  client.subscribeToAlerts(buildings.data[0].buildingId, (alert) => {
    console.log('새 알람:', alert);
  });

  // 나중에 구독 해제
  // unsubscribe();
  // client.disconnectRealtime();
}
```

---

## 4.5 장 요약

### API 설계 핵심 원칙

| 원칙 | 설명 |
|------|------|
| RESTful 설계 | 리소스 중심 URL, HTTP 메서드 활용 |
| 버전 관리 | URL 경로 버저닝 (/api/v1/) |
| 표준 응답 | success, data, error, meta 구조 |
| 페이지네이션 | 커서/오프셋 기반, 링크 포함 |
| 에러 처리 | 표준화된 에러 코드 및 메시지 |

### 인증/인가 요약

| 방식 | 용도 | 특징 |
|------|------|------|
| OAuth 2.0 | 사용자 인증 | Authorization Code, Client Credentials |
| API Key | M2M 통신 | 간편한 외부 시스템 연동 |
| JWT | 토큰 형식 | 자체 검증 가능, 클레임 포함 |
| RBAC | 권한 관리 | 역할 기반 접근 제어 |

### 실시간 통신 요약

- **WebSocket**: 양방향 실시간 통신
- **채널 구독**: 에너지, 설비, 알람 등
- **자동 재연결**: 지수 백오프 전략
- **하트비트**: 30초 간격 연결 유지

### 다음 장 미리보기

제5장에서는 WIA-BEMS Phase 3인 제어 프로토콜에 대해 다룹니다. HVAC 제어 시퀀스, 최적화 알고리즘, 고장 감지 및 진단(FDD) 방법론을 학습합니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
