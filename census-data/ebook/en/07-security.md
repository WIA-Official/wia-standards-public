# Chapter 7: Census Data Security and Privacy

## Comprehensive Security Framework for Statistical Confidentiality

### 7.1 Security Architecture Overview

The WIA-CENSUS-DATA standard establishes a comprehensive security framework that protects individual confidentiality while enabling statistical analysis and research access. This chapter covers the multi-layered security approach essential for census data protection.

```typescript
// Census Security Architecture
interface CensusSecurityArchitecture {
  version: '1.0.0';

  securityLayers: {
    perimeter: {
      description: 'Network boundary protection';
      controls: [
        'Web Application Firewall',
        'DDoS protection',
        'Intrusion Detection/Prevention',
        'Network segmentation'
      ];
    };
    transport: {
      description: 'Data in transit protection';
      controls: [
        'TLS 1.3 encryption',
        'Certificate pinning',
        'Mutual TLS for services',
        'VPN for administrative access'
      ];
    };
    application: {
      description: 'Application-level security';
      controls: [
        'Authentication and authorization',
        'Input validation',
        'Output encoding',
        'Session management'
      ];
    };
    data: {
      description: 'Data protection controls';
      controls: [
        'Encryption at rest',
        'Key management',
        'Data masking',
        'Disclosure control'
      ];
    };
    operational: {
      description: 'Operational security';
      controls: [
        'Audit logging',
        'Monitoring and alerting',
        'Incident response',
        'Backup and recovery'
      ];
    };
  };

  complianceFrameworks: [
    'ISO 27001',
    'NIST Cybersecurity Framework',
    'FedRAMP (US)',
    'GDPR (EU)',
    'National statistical confidentiality laws'
  ];
}

// Security Service Implementation
class CensusSecurityService {
  private authService: AuthenticationService;
  private authzService: AuthorizationService;
  private encryptionService: EncryptionService;
  private auditService: AuditService;

  constructor(config: SecurityConfig) {
    this.authService = new AuthenticationService(config.auth);
    this.authzService = new AuthorizationService(config.authz);
    this.encryptionService = new EncryptionService(config.encryption);
    this.auditService = new AuditService(config.audit);
  }

  async authenticateRequest(
    credentials: Credentials
  ): Promise<AuthenticationResult> {
    // Validate credentials
    const validation = await this.authService.validate(credentials);

    if (!validation.valid) {
      await this.auditService.log({
        event: 'AUTH_FAILURE',
        details: { reason: validation.reason },
        severity: 'WARNING'
      });

      return {
        authenticated: false,
        reason: validation.reason
      };
    }

    // Generate session
    const session = await this.authService.createSession(validation.identity);

    await this.auditService.log({
      event: 'AUTH_SUCCESS',
      identity: validation.identity.id,
      sessionId: session.id
    });

    return {
      authenticated: true,
      identity: validation.identity,
      session
    };
  }

  async authorizeAccess(
    identity: Identity,
    resource: Resource,
    action: Action
  ): Promise<AuthorizationResult> {
    // Evaluate access policies
    const decision = await this.authzService.evaluate({
      subject: identity,
      resource,
      action,
      context: this.buildContext()
    });

    await this.auditService.log({
      event: 'AUTHZ_DECISION',
      identity: identity.id,
      resource: resource.id,
      action,
      decision: decision.allowed ? 'PERMIT' : 'DENY'
    });

    return decision;
  }
}
```

### 7.2 Authentication and Access Control

```typescript
// Authentication Framework
interface AuthenticationFramework {
  methods: {
    apiKey: {
      useCase: 'Public API access';
      strength: 'BASIC';
      rateLimit: 'Lower tier';
    };
    oauth2: {
      useCase: 'Application and user authentication';
      flows: ['Authorization Code', 'Client Credentials'];
      strength: 'STANDARD';
    };
    saml2: {
      useCase: 'Enterprise SSO integration';
      strength: 'STANDARD';
    };
    mutualTls: {
      useCase: 'Service-to-service, administrative';
      strength: 'HIGH';
    };
    multiFactor: {
      useCase: 'Sensitive data access';
      factors: ['Password', 'TOTP', 'Hardware key'];
      strength: 'HIGHEST';
    };
  };
}

// OAuth2 Authentication Service
class OAuth2AuthenticationService {
  private tokenStore: TokenStore;
  private keyManager: KeyManager;
  private userStore: UserStore;

  async authorizeRequest(
    request: AuthorizationRequest
  ): Promise<AuthorizationCode> {
    // Validate client
    const client = await this.validateClient(request.clientId);

    // Validate redirect URI
    if (!this.isValidRedirectUri(client, request.redirectUri)) {
      throw new SecurityError('Invalid redirect URI');
    }

    // Generate authorization code
    const code = await this.generateAuthCode({
      client,
      scope: request.scope,
      state: request.state,
      codeChallenge: request.codeChallenge,
      codeChallengeMethod: request.codeChallengeMethod
    });

    return {
      code: code.value,
      expiresIn: 600 // 10 minutes
    };
  }

  async exchangeToken(
    request: TokenRequest
  ): Promise<TokenResponse> {
    switch (request.grantType) {
      case 'authorization_code':
        return this.handleAuthorizationCodeGrant(request);

      case 'client_credentials':
        return this.handleClientCredentialsGrant(request);

      case 'refresh_token':
        return this.handleRefreshTokenGrant(request);

      default:
        throw new SecurityError(`Unsupported grant type: ${request.grantType}`);
    }
  }

  private async handleAuthorizationCodeGrant(
    request: TokenRequest
  ): Promise<TokenResponse> {
    // Validate authorization code
    const authCode = await this.tokenStore.getAuthCode(request.code);

    if (!authCode || authCode.used || new Date() > authCode.expiresAt) {
      throw new SecurityError('Invalid or expired authorization code');
    }

    // Verify PKCE if used
    if (authCode.codeChallenge) {
      const valid = await this.verifyPKCE(
        request.codeVerifier,
        authCode.codeChallenge,
        authCode.codeChallengeMethod
      );

      if (!valid) {
        throw new SecurityError('PKCE verification failed');
      }
    }

    // Mark code as used
    await this.tokenStore.markAuthCodeUsed(request.code);

    // Generate tokens
    const accessToken = await this.generateAccessToken(
      authCode.client,
      authCode.user,
      authCode.scope
    );

    const refreshToken = await this.generateRefreshToken(
      authCode.client,
      authCode.user
    );

    return {
      accessToken: accessToken.value,
      tokenType: 'Bearer',
      expiresIn: accessToken.expiresIn,
      refreshToken: refreshToken.value,
      scope: authCode.scope.join(' ')
    };
  }

  private async generateAccessToken(
    client: Client,
    user: User,
    scope: string[]
  ): Promise<Token> {
    const payload = {
      sub: user.id,
      client_id: client.id,
      scope: scope,
      iat: Math.floor(Date.now() / 1000),
      exp: Math.floor(Date.now() / 1000) + 3600 // 1 hour
    };

    const privateKey = await this.keyManager.getSigningKey();

    const token = await this.signJWT(payload, privateKey);

    return {
      value: token,
      expiresIn: 3600
    };
  }
}

// Role-Based Access Control
interface RBACModel {
  roles: {
    PUBLIC_USER: {
      description: 'Anonymous or registered public user';
      permissions: [
        'READ_PUBLIC_DATA',
        'ACCESS_PUBLIC_API'
      ];
      dataAccessLevel: 'AGGREGATE_ONLY';
    };
    RESEARCHER: {
      description: 'Approved research user';
      permissions: [
        'READ_PUBLIC_DATA',
        'READ_RESEARCH_MICRODATA',
        'SUBMIT_ANALYSIS_REQUESTS'
      ];
      dataAccessLevel: 'RESEARCH_MICRODATA';
      requirements: ['Approved research proposal', 'Ethics clearance'];
    };
    STATISTICIAN: {
      description: 'Internal statistical staff';
      permissions: [
        'READ_ALL_DATA',
        'PROCESS_DATA',
        'GENERATE_OUTPUTS',
        'APPLY_DISCLOSURE_CONTROLS'
      ];
      dataAccessLevel: 'FULL_MICRODATA';
    };
    DATA_ADMINISTRATOR: {
      description: 'System and data administrator';
      permissions: [
        'MANAGE_USERS',
        'MANAGE_ACCESS',
        'AUDIT_REVIEW',
        'SYSTEM_CONFIG'
      ];
      dataAccessLevel: 'ADMINISTRATIVE';
    };
  };
}

// Attribute-Based Access Control Policy
class ABACPolicyEngine {
  private policyStore: PolicyStore;

  async evaluate(request: AccessRequest): Promise<PolicyDecision> {
    // Get applicable policies
    const policies = await this.policyStore.findPolicies({
      resourceType: request.resource.type,
      action: request.action
    });

    // Evaluate each policy
    const decisions: PolicyEvaluation[] = [];

    for (const policy of policies) {
      const evaluation = await this.evaluatePolicy(policy, request);
      decisions.push(evaluation);
    }

    // Combine decisions
    return this.combineDecisions(decisions);
  }

  private async evaluatePolicy(
    policy: Policy,
    request: AccessRequest
  ): Promise<PolicyEvaluation> {
    // Evaluate target
    if (!this.matchesTarget(policy.target, request)) {
      return { policyId: policy.id, applicable: false };
    }

    // Evaluate conditions
    for (const condition of policy.conditions) {
      const result = await this.evaluateCondition(condition, request);

      if (!result) {
        return {
          policyId: policy.id,
          applicable: true,
          decision: 'DENY',
          reason: `Condition not met: ${condition.description}`
        };
      }
    }

    return {
      policyId: policy.id,
      applicable: true,
      decision: policy.effect
    };
  }

  private async evaluateCondition(
    condition: PolicyCondition,
    request: AccessRequest
  ): Promise<boolean> {
    const attributeValue = this.getAttributeValue(
      condition.attribute,
      request
    );

    switch (condition.operator) {
      case 'EQUALS':
        return attributeValue === condition.value;

      case 'IN':
        return (condition.value as any[]).includes(attributeValue);

      case 'GREATER_THAN':
        return attributeValue > condition.value;

      case 'MATCHES':
        return new RegExp(condition.value as string).test(attributeValue);

      case 'TIME_BETWEEN':
        const now = new Date();
        const [start, end] = condition.value as [string, string];
        return now >= new Date(start) && now <= new Date(end);

      default:
        return false;
    }
  }
}

// Example Census Data Access Policies
const censusAccessPolicies: Policy[] = [
  {
    id: 'POL-001',
    name: 'Public Data Access',
    description: 'Allow public access to aggregate statistics',
    target: {
      resourceType: 'AGGREGATE_DATA',
      actions: ['READ']
    },
    effect: 'PERMIT',
    conditions: [
      {
        attribute: 'data.disclosureControlApplied',
        operator: 'EQUALS',
        value: true,
        description: 'Data must have disclosure controls applied'
      }
    ]
  },
  {
    id: 'POL-002',
    name: 'Researcher Microdata Access',
    description: 'Allow approved researchers to access microdata',
    target: {
      resourceType: 'MICRODATA',
      actions: ['READ']
    },
    effect: 'PERMIT',
    conditions: [
      {
        attribute: 'subject.role',
        operator: 'IN',
        value: ['RESEARCHER', 'STATISTICIAN'],
        description: 'Must be researcher or statistician'
      },
      {
        attribute: 'subject.projectApproved',
        operator: 'EQUALS',
        value: true,
        description: 'Must have approved project'
      },
      {
        attribute: 'resource.sensitivityLevel',
        operator: 'IN',
        value: ['LOW', 'MEDIUM'],
        description: 'Only access low/medium sensitivity'
      },
      {
        attribute: 'context.accessLocation',
        operator: 'IN',
        value: ['SECURE_FACILITY', 'APPROVED_REMOTE'],
        description: 'Must access from approved location'
      }
    ]
  }
];
```

### 7.3 Statistical Disclosure Control

```typescript
// Statistical Disclosure Control Framework
interface DisclosureControlFramework {
  methods: {
    suppression: {
      description: 'Remove cells with few observations';
      techniques: ['Primary suppression', 'Secondary suppression'];
      useCase: 'Tabular data protection';
    };
    perturbation: {
      description: 'Add noise to data values';
      techniques: ['Random rounding', 'Controlled rounding', 'Cell perturbation'];
      useCase: 'Aggregate statistics';
    };
    generalization: {
      description: 'Reduce precision or detail';
      techniques: ['Top/bottom coding', 'Recoding', 'Data swapping'];
      useCase: 'Microdata protection';
    };
    syntheticData: {
      description: 'Generate artificial data with similar properties';
      techniques: ['Fully synthetic', 'Partially synthetic', 'Hybrid'];
      useCase: 'Public use files, testing';
    };
    differentialPrivacy: {
      description: 'Mathematically provable privacy guarantee';
      techniques: ['Laplace mechanism', 'Gaussian mechanism', 'Exponential mechanism'];
      useCase: 'Interactive queries, official statistics';
    };
  };
}

// Disclosure Control Service
class DisclosureControlService {
  private disclosureRules: DisclosureRules;
  private perturbationEngine: PerturbationEngine;
  private syntheticGenerator: SyntheticDataGenerator;

  async applyDisclosureControl(
    data: CensusData,
    outputType: OutputType
  ): Promise<ProtectedData> {
    switch (outputType) {
      case 'AGGREGATE_TABLE':
        return this.protectAggregateTable(data);

      case 'MICRODATA_FILE':
        return this.protectMicrodata(data);

      case 'INTERACTIVE_QUERY':
        return this.protectInteractiveQuery(data);

      case 'SYNTHETIC_FILE':
        return this.generateSyntheticData(data);

      default:
        throw new Error(`Unsupported output type: ${outputType}`);
    }
  }

  private async protectAggregateTable(
    data: CensusData
  ): Promise<ProtectedData> {
    const table = data as AggregateTable;
    const protectedTable = { ...table };

    // Apply primary suppression
    const primarySuppressions = this.identifyPrimarySuppression(table);

    // Apply complementary suppression
    const allSuppressions = await this.applyComplementarySuppression(
      table,
      primarySuppressions
    );

    // Suppress cells
    for (const cellKey of allSuppressions) {
      protectedTable.cells[cellKey] = {
        ...protectedTable.cells[cellKey],
        value: null,
        suppressed: true,
        suppressionReason: primarySuppressions.has(cellKey)
          ? 'PRIMARY'
          : 'COMPLEMENTARY'
      };
    }

    return {
      data: protectedTable,
      disclosureMethod: 'SUPPRESSION',
      cellsAffected: allSuppressions.size,
      protectionApplied: new Date().toISOString()
    };
  }

  private identifyPrimarySuppression(
    table: AggregateTable
  ): Set<string> {
    const suppressions = new Set<string>();

    for (const [cellKey, cell] of Object.entries(table.cells)) {
      // Frequency rule: suppress if count below threshold
      if (cell.count < this.disclosureRules.minimumFrequency) {
        suppressions.add(cellKey);
        continue;
      }

      // Dominance rule: suppress if top contributors dominate
      if (cell.contributorShares) {
        const topShare = cell.contributorShares[0];
        if (topShare > this.disclosureRules.dominanceThreshold) {
          suppressions.add(cellKey);
          continue;
        }
      }

      // p% rule: suppress if value can be estimated too closely
      if (cell.pPercentRisk && cell.pPercentRisk > this.disclosureRules.pPercent) {
        suppressions.add(cellKey);
      }
    }

    return suppressions;
  }

  private async applyComplementarySuppression(
    table: AggregateTable,
    primarySuppressions: Set<string>
  ): Promise<Set<string>> {
    // Use linear programming to find optimal complementary suppressions
    const optimizer = new SuppressionOptimizer(table, primarySuppressions);

    const complementarySuppressions = await optimizer.optimize({
      minimizeInformationLoss: true,
      ensureProtection: true
    });

    return new Set([...primarySuppressions, ...complementarySuppressions]);
  }

  private async protectMicrodata(
    data: CensusData
  ): Promise<ProtectedData> {
    const microdata = data as MicrodataFile;
    const protectedRecords: MicrodataRecord[] = [];

    for (const record of microdata.records) {
      const protectedRecord = { ...record };

      // Apply local suppression for quasi-identifiers
      for (const variable of microdata.quasiIdentifiers) {
        if (this.shouldSuppress(record, variable)) {
          protectedRecord[variable] = null;
        }
      }

      // Apply top/bottom coding for continuous variables
      for (const variable of microdata.continuousVariables) {
        protectedRecord[variable] = this.applyTopBottomCoding(
          record[variable],
          variable
        );
      }

      // Apply global recoding for categorical variables
      for (const variable of microdata.categoricalVariables) {
        protectedRecord[variable] = this.applyRecoding(
          record[variable],
          variable
        );
      }

      protectedRecords.push(protectedRecord);
    }

    // Apply data swapping
    const swappedRecords = await this.applyDataSwapping(
      protectedRecords,
      microdata.swappingRate
    );

    return {
      data: { ...microdata, records: swappedRecords },
      disclosureMethod: 'MICRODATA_PROTECTION',
      recordsAffected: swappedRecords.length,
      protectionApplied: new Date().toISOString()
    };
  }
}

// Differential Privacy Implementation
class DifferentialPrivacyService {
  private epsilon: number; // Privacy budget
  private delta: number; // Failure probability

  constructor(config: DPConfig) {
    this.epsilon = config.epsilon;
    this.delta = config.delta;
  }

  async queryWithDP(
    query: StatisticalQuery,
    dataset: CensusDataset
  ): Promise<DPQueryResult> {
    // Calculate query sensitivity
    const sensitivity = this.calculateSensitivity(query);

    // Determine noise scale
    const noiseScale = this.calculateNoiseScale(sensitivity);

    // Execute true query
    const trueResult = await this.executeQuery(query, dataset);

    // Add calibrated noise
    const noisyResult = this.addNoise(trueResult, noiseScale);

    // Update privacy budget
    await this.updatePrivacyBudget(query, this.epsilon);

    return {
      value: noisyResult,
      epsilon: this.epsilon,
      mechanism: 'LAPLACE',
      confidenceInterval: this.calculateConfidenceInterval(noisyResult, noiseScale),
      privacyGuarantee: `(${this.epsilon}, ${this.delta})-differential privacy`
    };
  }

  private calculateSensitivity(query: StatisticalQuery): number {
    switch (query.type) {
      case 'COUNT':
        return 1; // Adding/removing one person changes count by 1

      case 'SUM':
        return query.bounds.max - query.bounds.min;

      case 'MEAN':
        return (query.bounds.max - query.bounds.min) / query.minGroupSize;

      case 'HISTOGRAM':
        return 2; // One person can affect two bins

      default:
        throw new Error(`Unknown query type: ${query.type}`);
    }
  }

  private calculateNoiseScale(sensitivity: number): number {
    // Laplace mechanism: scale = sensitivity / epsilon
    return sensitivity / this.epsilon;
  }

  private addNoise(value: number, scale: number): number {
    // Sample from Laplace distribution
    const u = Math.random() - 0.5;
    const noise = -scale * Math.sign(u) * Math.log(1 - 2 * Math.abs(u));
    return value + noise;
  }

  // Post-processing for multiple queries
  async compositeQuery(
    queries: StatisticalQuery[],
    dataset: CensusDataset,
    totalBudget: number
  ): Promise<DPCompositeResult> {
    // Allocate budget across queries
    const budgetPerQuery = totalBudget / queries.length;
    const originalEpsilon = this.epsilon;

    const results: DPQueryResult[] = [];

    try {
      this.epsilon = budgetPerQuery;

      for (const query of queries) {
        const result = await this.queryWithDP(query, dataset);
        results.push(result);
      }
    } finally {
      this.epsilon = originalEpsilon;
    }

    return {
      results,
      totalEpsilon: totalBudget,
      composition: 'SEQUENTIAL'
    };
  }
}

// Synthetic Data Generator
class SyntheticDataGenerator {
  private generativeModel: GenerativeModel;
  private privacyValidator: PrivacyValidator;

  async generateSyntheticData(
    originalData: MicrodataFile,
    config: SyntheticConfig
  ): Promise<SyntheticDataset> {
    // Train generative model on original data
    await this.generativeModel.train(originalData, {
      differentialPrivacy: config.useDifferentialPrivacy,
      epsilon: config.epsilon
    });

    // Generate synthetic records
    const syntheticRecords: MicrodataRecord[] = [];
    const targetSize = config.targetSize || originalData.records.length;

    for (let i = 0; i < targetSize; i++) {
      const record = await this.generativeModel.sample();
      syntheticRecords.push(record);
    }

    // Validate utility and privacy
    const utilityMetrics = await this.assessUtility(
      originalData.records,
      syntheticRecords
    );

    const privacyMetrics = await this.privacyValidator.assess(
      originalData.records,
      syntheticRecords
    );

    return {
      records: syntheticRecords,
      metadata: {
        generationMethod: this.generativeModel.name,
        originalRecordCount: originalData.records.length,
        syntheticRecordCount: syntheticRecords.length,
        epsilon: config.epsilon,
        generatedAt: new Date().toISOString()
      },
      quality: {
        utility: utilityMetrics,
        privacy: privacyMetrics
      }
    };
  }

  private async assessUtility(
    original: MicrodataRecord[],
    synthetic: MicrodataRecord[]
  ): Promise<UtilityMetrics> {
    return {
      marginalDistributions: await this.compareMarginals(original, synthetic),
      correlations: await this.compareCorrelations(original, synthetic),
      regressionCoefficients: await this.compareRegressions(original, synthetic),
      queryAccuracy: await this.compareQueries(original, synthetic)
    };
  }
}
```

### 7.4 Encryption and Key Management

```typescript
// Encryption Framework
interface EncryptionFramework {
  atRest: {
    algorithm: 'AES-256-GCM';
    keyLength: 256;
    keyRotation: '90 days';
    scope: ['Database columns', 'File storage', 'Backups'];
  };

  inTransit: {
    protocol: 'TLS 1.3';
    cipherSuites: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ];
    certificateAuthority: 'Government-approved CA';
  };

  applicationLevel: {
    sensitiveFields: 'Field-level encryption';
    searchableEncryption: 'Deterministic encryption for lookups';
    homomorphic: 'Research/future use';
  };
}

// Key Management Service
class KeyManagementService {
  private hsm: HSMInterface;
  private keyStore: SecureKeyStore;
  private rotationScheduler: RotationScheduler;

  async generateDataEncryptionKey(
    purpose: string
  ): Promise<DataEncryptionKey> {
    // Generate DEK in HSM
    const dekId = await this.hsm.generateKey({
      algorithm: 'AES',
      keyLength: 256,
      extractable: true
    });

    // Get KEK for wrapping
    const kek = await this.getKeyEncryptionKey();

    // Wrap DEK with KEK
    const wrappedDek = await this.hsm.wrapKey(dekId, kek.id);

    // Store wrapped DEK
    const dek: DataEncryptionKey = {
      id: crypto.randomUUID(),
      purpose,
      wrappedKey: wrappedDek,
      kekId: kek.id,
      createdAt: new Date().toISOString(),
      expiresAt: this.calculateExpiration(),
      status: 'ACTIVE'
    };

    await this.keyStore.store(dek);

    return dek;
  }

  async rotateKey(keyId: string): Promise<KeyRotationResult> {
    // Get current key
    const currentKey = await this.keyStore.get(keyId);

    if (!currentKey) {
      throw new Error(`Key not found: ${keyId}`);
    }

    // Generate new key
    const newKey = await this.generateDataEncryptionKey(currentKey.purpose);

    // Mark old key for retirement
    currentKey.status = 'RETIRING';
    currentKey.retiredAt = new Date().toISOString();
    currentKey.successorId = newKey.id;

    await this.keyStore.update(currentKey);

    // Schedule re-encryption of data
    await this.scheduleReEncryption(currentKey, newKey);

    return {
      oldKeyId: currentKey.id,
      newKeyId: newKey.id,
      status: 'ROTATION_INITIATED',
      reEncryptionRequired: true
    };
  }

  async encryptData(
    data: Buffer,
    keyId: string
  ): Promise<EncryptedData> {
    // Get key
    const dek = await this.keyStore.get(keyId);

    if (!dek || dek.status !== 'ACTIVE') {
      throw new Error(`Invalid or inactive key: ${keyId}`);
    }

    // Unwrap DEK
    const unwrappedKey = await this.hsm.unwrapKey(dek.wrappedKey, dek.kekId);

    // Generate IV
    const iv = crypto.randomBytes(12);

    // Encrypt data
    const cipher = crypto.createCipheriv('aes-256-gcm', unwrappedKey, iv);
    const encrypted = Buffer.concat([
      cipher.update(data),
      cipher.final()
    ]);
    const authTag = cipher.getAuthTag();

    // Clear unwrapped key from memory
    unwrappedKey.fill(0);

    return {
      ciphertext: encrypted,
      iv,
      authTag,
      keyId: dek.id,
      algorithm: 'AES-256-GCM'
    };
  }

  async decryptData(
    encryptedData: EncryptedData
  ): Promise<Buffer> {
    // Get key
    const dek = await this.keyStore.get(encryptedData.keyId);

    if (!dek) {
      throw new Error(`Key not found: ${encryptedData.keyId}`);
    }

    // Unwrap DEK
    const unwrappedKey = await this.hsm.unwrapKey(dek.wrappedKey, dek.kekId);

    // Decrypt data
    const decipher = crypto.createDecipheriv(
      'aes-256-gcm',
      unwrappedKey,
      encryptedData.iv
    );
    decipher.setAuthTag(encryptedData.authTag);

    const decrypted = Buffer.concat([
      decipher.update(encryptedData.ciphertext),
      decipher.final()
    ]);

    // Clear unwrapped key from memory
    unwrappedKey.fill(0);

    return decrypted;
  }
}
```

### 7.5 Audit and Compliance

```typescript
// Audit Framework
interface AuditFramework {
  auditableEvents: {
    authentication: ['Login', 'Logout', 'Failed login', 'Password change'];
    authorization: ['Access granted', 'Access denied', 'Permission change'];
    dataAccess: ['Query', 'Download', 'Export', 'Print'];
    dataModification: ['Create', 'Update', 'Delete', 'Import'];
    administration: ['User management', 'Role change', 'Config change'];
    security: ['Key rotation', 'Certificate renewal', 'Security alert'];
  };

  retentionPolicy: {
    securityEvents: '7 years';
    dataAccessEvents: '10 years';
    administrativeEvents: '7 years';
  };

  auditLogProtection: {
    integrity: 'Hash chain';
    confidentiality: 'Encryption at rest';
    availability: 'Geographic replication';
  };
}

// Audit Service Implementation
class AuditService {
  private auditStore: AuditStore;
  private hashChain: HashChain;
  private alertService: AlertService;

  async log(event: AuditEvent): Promise<void> {
    // Enrich event
    const enrichedEvent = {
      ...event,
      eventId: crypto.randomUUID(),
      timestamp: new Date().toISOString(),
      serverInstance: this.getServerInstance(),
      clientInfo: this.getClientInfo()
    };

    // Calculate hash
    const previousHash = await this.hashChain.getLastHash();
    const eventHash = this.calculateHash(enrichedEvent, previousHash);

    enrichedEvent.previousHash = previousHash;
    enrichedEvent.eventHash = eventHash;

    // Store event
    await this.auditStore.store(enrichedEvent);

    // Update hash chain
    await this.hashChain.append(eventHash);

    // Check for alert conditions
    await this.checkAlertConditions(enrichedEvent);
  }

  async query(
    criteria: AuditQueryCriteria
  ): Promise<AuditQueryResult> {
    // Validate query authorization
    // Only authorized personnel can query audit logs

    const events = await this.auditStore.query(criteria);

    // Verify integrity of returned events
    const integrityResults = await this.verifyIntegrity(events);

    return {
      events,
      total: events.length,
      integrityVerified: integrityResults.allValid,
      integrityIssues: integrityResults.issues
    };
  }

  private async verifyIntegrity(
    events: AuditEvent[]
  ): Promise<IntegrityVerification> {
    const issues: IntegrityIssue[] = [];

    // Sort by timestamp
    const sortedEvents = [...events].sort(
      (a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime()
    );

    for (let i = 0; i < sortedEvents.length; i++) {
      const event = sortedEvents[i];

      // Verify event hash
      const expectedPrevious = i > 0 ? sortedEvents[i - 1].eventHash : null;
      const calculatedHash = this.calculateHash(event, expectedPrevious);

      if (calculatedHash !== event.eventHash) {
        issues.push({
          eventId: event.eventId,
          issue: 'HASH_MISMATCH',
          expected: calculatedHash,
          actual: event.eventHash
        });
      }

      // Verify chain continuity
      if (event.previousHash !== expectedPrevious) {
        issues.push({
          eventId: event.eventId,
          issue: 'CHAIN_BREAK',
          expected: expectedPrevious,
          actual: event.previousHash
        });
      }
    }

    return {
      allValid: issues.length === 0,
      issues
    };
  }

  private calculateHash(
    event: AuditEvent,
    previousHash: string | null
  ): string {
    const hashInput = {
      eventId: event.eventId,
      timestamp: event.timestamp,
      eventType: event.eventType,
      actor: event.actor,
      action: event.action,
      resource: event.resource,
      previousHash
    };

    return crypto
      .createHash('sha256')
      .update(JSON.stringify(hashInput))
      .digest('hex');
  }

  async generateComplianceReport(
    period: DateRange,
    framework: ComplianceFramework
  ): Promise<ComplianceReport> {
    // Query relevant audit events
    const events = await this.query({
      startDate: period.start,
      endDate: period.end
    });

    // Analyze against compliance requirements
    const findings: ComplianceFinding[] = [];

    for (const requirement of framework.requirements) {
      const finding = await this.assessRequirement(
        requirement,
        events.events
      );
      findings.push(finding);
    }

    return {
      period,
      framework: framework.name,
      overallStatus: this.calculateOverallStatus(findings),
      findings,
      generatedAt: new Date().toISOString()
    };
  }
}
```

### 7.6 Incident Response

```typescript
// Incident Response Framework
class IncidentResponseService {
  private alertManager: AlertManager;
  private notificationService: NotificationService;
  private containmentService: ContainmentService;

  async handleSecurityIncident(
    incident: SecurityIncident
  ): Promise<IncidentResponse> {
    // Initial assessment
    const assessment = await this.assessIncident(incident);

    // Escalate if needed
    if (assessment.severity >= IncidentSeverity.HIGH) {
      await this.escalate(incident, assessment);
    }

    // Contain if active threat
    if (assessment.activeTheat) {
      await this.containmentService.contain(incident, assessment);
    }

    // Create incident record
    const incidentRecord = await this.createIncidentRecord(
      incident,
      assessment
    );

    // Initiate response workflow
    await this.initiateResponseWorkflow(incidentRecord);

    return {
      incidentId: incidentRecord.id,
      status: 'RESPONSE_INITIATED',
      severity: assessment.severity,
      containmentActions: assessment.activeTheat
        ? await this.containmentService.getActions(incidentRecord.id)
        : [],
      nextSteps: this.determineNextSteps(assessment)
    };
  }

  private async assessIncident(
    incident: SecurityIncident
  ): Promise<IncidentAssessment> {
    return {
      severity: this.calculateSeverity(incident),
      activeTheat: this.isActiveThreat(incident),
      affectedSystems: await this.identifyAffectedSystems(incident),
      potentialDataExposure: await this.assessDataExposure(incident),
      containmentRequired: this.requiresContainment(incident),
      regulatoryNotificationRequired: this.requiresNotification(incident)
    };
  }

  private calculateSeverity(incident: SecurityIncident): IncidentSeverity {
    let score = 0;

    // Data sensitivity
    if (incident.dataInvolved?.includes('MICRODATA')) score += 3;
    if (incident.dataInvolved?.includes('IDENTIFIABLE')) score += 4;

    // Scope
    if (incident.recordsAffected > 1000000) score += 3;
    else if (incident.recordsAffected > 10000) score += 2;
    else if (incident.recordsAffected > 100) score += 1;

    // Threat actor
    if (incident.threatActor === 'NATION_STATE') score += 3;
    if (incident.threatActor === 'ORGANIZED_CRIME') score += 2;

    // Determine severity level
    if (score >= 8) return IncidentSeverity.CRITICAL;
    if (score >= 5) return IncidentSeverity.HIGH;
    if (score >= 3) return IncidentSeverity.MEDIUM;
    return IncidentSeverity.LOW;
  }
}

enum IncidentSeverity {
  LOW = 1,
  MEDIUM = 2,
  HIGH = 3,
  CRITICAL = 4
}
```

---

**WIA-CENSUS-DATA Security and Privacy**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
