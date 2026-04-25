# 제5장: 인구조사 데이터 제어 프로토콜

## 통신 및 데이터 전송 표준

### 5.1 프로토콜 아키텍처 개요

WIA-CENSUS-DATA 표준은 시스템 간 안전하고 신뢰할 수 있는 인구조사 데이터 전송을 위한 포괄적인 통신 프로토콜을 정의합니다. 이 장에서는 인구조사 운영에 필수적인 전송 계층 프로토콜, 메시지 형식 및 품질 제어 메커니즘을 다룹니다.

```typescript
// 인구조사 통신 프로토콜 아키텍처
interface CensusProtocolArchitecture {
  version: '1.0.0';

  transportLayers: {
    primary: {
      protocol: 'HTTPS/TLS 1.3';
      ports: [443];
      encryption: 'AES-256-GCM';
      certificateRequirements: '정부 승인 CA';
    };
    realtime: {
      protocol: 'WebSocket Secure (WSS)';
      purpose: '실시간 조사 업데이트';
      heartbeatInterval: 30000; // ms
    };
    bulk: {
      protocol: 'SFTP / SCP';
      purpose: '대용량 파일 전송';
      encryption: 'AES-256';
    };
  };

  messageFormats: {
    standard: '스키마 검증이 있는 JSON';
    statistical: 'SDMX-JSON / SDMX-ML';
    binary: '고용량용 Protocol Buffers';
    streaming: '실시간용 NDJSON';
  };

  qualityOfService: {
    reliability: '최소 한 번 전달 보장';
    ordering: '순서를 위한 시퀀스 번호';
    idempotency: '멱등성 키 필요';
    retry: '지터가 있는 지수 백오프';
  };
}

// 프로토콜 핸들러 구현
class CensusProtocolHandler {
  private tlsConfig: TLSConfiguration;
  private messageValidator: MessageValidator;
  private sequenceManager: SequenceManager;

  constructor(config: ProtocolConfig) {
    this.tlsConfig = this.initializeTLS(config.tls);
    this.messageValidator = new MessageValidator(config.schemas);
    this.sequenceManager = new SequenceManager();
  }

  async sendMessage(
    message: CensusMessage,
    destination: Endpoint
  ): Promise<SendResult> {
    // 시퀀스 번호 할당
    const sequenceNumber = this.sequenceManager.nextSequence(destination);

    // 프로토콜 헤더 추가
    const enrichedMessage = this.enrichMessage(message, sequenceNumber);

    // 메시지 검증
    const validation = await this.messageValidator.validate(enrichedMessage);
    if (!validation.valid) {
      throw new ValidationError('메시지 검증 실패', validation.errors);
    }

    // 메시지 직렬화
    const serialized = this.serializeMessage(enrichedMessage);

    // 재시도 로직으로 전송
    return this.sendWithRetry(serialized, destination, {
      maxRetries: 3,
      backoffBase: 1000,
      backoffMax: 30000
    });
  }

  private async sendWithRetry(
    data: Buffer,
    destination: Endpoint,
    retryConfig: RetryConfig
  ): Promise<SendResult> {
    let lastError: Error | null = null;
    let attempt = 0;

    while (attempt < retryConfig.maxRetries) {
      try {
        const response = await this.transmit(data, destination);
        return {
          success: true,
          responseTime: response.timing,
          acknowledgment: response.ack
        };
      } catch (error) {
        lastError = error as Error;
        attempt++;

        if (attempt < retryConfig.maxRetries) {
          const delay = this.calculateBackoff(
            attempt,
            retryConfig.backoffBase,
            retryConfig.backoffMax
          );
          await this.sleep(delay);
        }
      }
    }

    throw new TransmissionError(
      `${retryConfig.maxRetries}회 시도 후 실패`,
      lastError
    );
  }

  private calculateBackoff(
    attempt: number,
    base: number,
    max: number
  ): number {
    const exponential = base * Math.pow(2, attempt);
    const jitter = Math.random() * base;
    return Math.min(exponential + jitter, max);
  }
}
```

### 5.2 안전한 데이터 전송 프로토콜

```typescript
// 안전한 인구조사 데이터 전송 프로토콜
interface SecureCensusTransferProtocol {
  name: 'SCDTP';
  version: '1.0';

  securityLayers: {
    transport: {
      protocol: 'TLS 1.3';
      cipherSuites: [
        'TLS_AES_256_GCM_SHA384',
        'TLS_CHACHA20_POLY1305_SHA256'
      ];
      certificateValidation: {
        ocspStapling: true;
        certificateTransparency: true;
        pinning: '선택 사항이지만 권장';
      };
    };

    message: {
      encryption: '봉투 암호화';
      keyManagement: 'HSM 지원 키 저장';
      integrityCheck: 'HMAC-SHA256';
    };

    application: {
      authentication: '상호 TLS + OAuth2';
      authorization: '속성 기반 접근 제어';
      auditLogging: '모든 작업 로깅';
    };
  };
}

// 안전한 전송 서비스 구현
class SecureTransferService {
  private keyManager: KeyManager;
  private encryptionService: EncryptionService;
  private auditLogger: AuditLogger;

  async secureTransfer(
    data: CensusData,
    recipient: TransferRecipient
  ): Promise<TransferResult> {
    // 수신자의 공개 키 가져오기
    const recipientKey = await this.keyManager.getPublicKey(recipient.id);

    // 데이터 암호화 키 생성
    const dataKey = await this.encryptionService.generateDataKey();

    // 데이터 키로 데이터 암호화
    const encryptedData = await this.encryptionService.encrypt(
      this.serializeData(data),
      dataKey
    );

    // 수신자의 공개 키로 데이터 키 암호화
    const encryptedKey = await this.encryptionService.encryptKey(
      dataKey,
      recipientKey
    );

    // 전송 봉투 생성
    const envelope: TransferEnvelope = {
      version: '1.0',
      sender: this.getSenderId(),
      recipient: recipient.id,
      timestamp: new Date().toISOString(),
      encryptedKey: encryptedKey.toString('base64'),
      encryptedData: encryptedData.toString('base64'),
      algorithm: 'AES-256-GCM',
      keyAlgorithm: 'RSA-OAEP-256',
      integrity: await this.calculateIntegrity(encryptedData)
    };

    // 봉투 서명
    const signature = await this.signEnvelope(envelope);
    envelope.signature = signature;

    // 전송 시작 로깅
    await this.auditLogger.log({
      action: 'TRANSFER_INITIATED',
      sender: this.getSenderId(),
      recipient: recipient.id,
      dataSize: data.records?.length || 0,
      timestamp: new Date().toISOString()
    });

    // 전송
    const result = await this.transmitEnvelope(envelope, recipient.endpoint);

    // 완료 로깅
    await this.auditLogger.log({
      action: 'TRANSFER_COMPLETED',
      transferId: result.transferId,
      status: result.status,
      timestamp: new Date().toISOString()
    });

    return result;
  }

  async receiveTransfer(
    envelope: TransferEnvelope
  ): Promise<DecryptedData> {
    // 서명 검증
    const signatureValid = await this.verifySignature(
      envelope,
      envelope.signature
    );

    if (!signatureValid) {
      throw new SecurityError('유효하지 않은 봉투 서명');
    }

    // 무결성 검증
    const integrityValid = await this.verifyIntegrity(
      Buffer.from(envelope.encryptedData, 'base64'),
      envelope.integrity
    );

    if (!integrityValid) {
      throw new SecurityError('데이터 무결성 검사 실패');
    }

    // 데이터 키 복호화
    const dataKey = await this.encryptionService.decryptKey(
      Buffer.from(envelope.encryptedKey, 'base64'),
      await this.keyManager.getPrivateKey()
    );

    // 데이터 복호화
    const decryptedData = await this.encryptionService.decrypt(
      Buffer.from(envelope.encryptedData, 'base64'),
      dataKey
    );

    // 수신 로깅
    await this.auditLogger.log({
      action: 'TRANSFER_RECEIVED',
      sender: envelope.sender,
      timestamp: new Date().toISOString()
    });

    return this.deserializeData(decryptedData);
  }
}

interface TransferEnvelope {
  version: string;
  sender: string;
  recipient: string;
  timestamp: string;
  encryptedKey: string;
  encryptedData: string;
  algorithm: string;
  keyAlgorithm: string;
  integrity: string;
  signature?: string;
}
```

### 5.3 실시간 데이터 스트리밍 프로토콜

```typescript
// 실시간 인구조사 스트리밍 프로토콜
interface CensusStreamingProtocol {
  name: 'Census Real-Time Stream';
  transport: 'WebSocket Secure';

  messageTypes: {
    // 제어 메시지
    CONNECT: '연결 시작';
    DISCONNECT: '정상 연결 해제';
    HEARTBEAT: '연결 유지 핑';
    ACK: '메시지 확인';

    // 데이터 메시지
    ENUMERATION_UPDATE: '새 조사 데이터';
    RESPONSE_RECEIVED: '인구조사 응답 수신';
    QUALITY_ALERT: '데이터 품질 문제';
    PROGRESS_UPDATE: '수집 진행 상황';

    // 관리
    CONFIG_UPDATE: '구성 변경';
    PAUSE_STREAM: '데이터 흐름 일시 정지';
    RESUME_STREAM: '데이터 흐름 재개';
  };

  qualityOfService: {
    deliveryGuarantee: '최소 한 번';
    orderingGuarantee: '파티션별 순서';
    acknowledgmentRequired: true;
    windowSize: 1000; // 대기 중인 메시지
  };
}

// 스트리밍 메시지 형식
interface StreamMessage {
  header: {
    messageId: string;
    messageType: string;
    timestamp: string;
    sequenceNumber: number;
    partitionKey: string;
  };
  payload: any;
  metadata?: {
    priority: 'HIGH' | 'NORMAL' | 'LOW';
    ttl?: number;
    correlationId?: string;
  };
}

// 실시간 스트리밍 서비스
class CensusStreamingService {
  private connections: Map<string, WebSocket>;
  private messageBuffer: MessageBuffer;
  private acknowledgmentTracker: AcknowledgmentTracker;

  constructor(config: StreamingConfig) {
    this.connections = new Map();
    this.messageBuffer = new MessageBuffer(config.bufferSize);
    this.acknowledgmentTracker = new AcknowledgmentTracker(config.ackTimeout);
  }

  async connect(
    clientId: string,
    options: ConnectionOptions
  ): Promise<StreamConnection> {
    const ws = new WebSocket(options.endpoint, {
      headers: {
        'Authorization': `Bearer ${options.token}`,
        'X-Client-ID': clientId
      }
    });

    return new Promise((resolve, reject) => {
      ws.on('open', () => {
        this.connections.set(clientId, ws);
        this.setupHeartbeat(clientId, ws);

        resolve({
          clientId,
          status: 'CONNECTED',
          subscribe: (topics) => this.subscribe(clientId, topics),
          publish: (message) => this.publish(clientId, message),
          disconnect: () => this.disconnect(clientId)
        });
      });

      ws.on('error', (error) => {
        reject(new ConnectionError('WebSocket 연결 실패', error));
      });

      ws.on('message', (data) => {
        this.handleMessage(clientId, data);
      });

      ws.on('close', () => {
        this.handleDisconnect(clientId);
      });
    });
  }

  async publish(
    clientId: string,
    message: StreamMessage
  ): Promise<PublishResult> {
    const ws = this.connections.get(clientId);
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      throw new ConnectionError('연결되지 않음');
    }

    // 필수 필드가 있는 메시지 보장
    const enrichedMessage: StreamMessage = {
      header: {
        ...message.header,
        messageId: message.header.messageId || crypto.randomUUID(),
        timestamp: message.header.timestamp || new Date().toISOString(),
        sequenceNumber: this.getNextSequence(clientId)
      },
      payload: message.payload,
      metadata: message.metadata
    };

    // 잠재적 재시도를 위해 메시지 버퍼링
    this.messageBuffer.add(enrichedMessage);

    // 확인 추적
    const ackPromise = this.acknowledgmentTracker.waitForAck(
      enrichedMessage.header.messageId
    );

    // 메시지 전송
    ws.send(JSON.stringify(enrichedMessage));

    // 확인 대기
    const ack = await ackPromise;

    return {
      messageId: enrichedMessage.header.messageId,
      acknowledged: true,
      timestamp: ack.timestamp
    };
  }
}
```

### 5.4 배치 전송 프로토콜

```typescript
// 배치 인구조사 데이터 전송 프로토콜
interface BatchTransferProtocol {
  name: 'Census Batch Transfer';
  version: '1.0';

  transferModes: {
    push: {
      description: '발신자가 전송 시작';
      protocol: 'SFTP / HTTPS PUT';
      useCases: ['조사 데이터 업로드', '행정 데이터'];
    };
    pull: {
      description: '수신자가 데이터 요청';
      protocol: 'HTTPS GET / SFTP GET';
      useCases: ['데이터 배포', '파트너 접근'];
    };
    exchange: {
      description: '양방향 동기화';
      protocol: '사용자 정의 동기화 프로토콜';
      useCases: ['기관 간 데이터 공유'];
    };
  };

  fileFormats: {
    data: ['CSV', 'Parquet', 'JSON Lines', 'SDMX-ML'];
    compression: ['gzip', 'bzip2', 'zstd', 'lz4'];
    encryption: ['AES-256-GCM', 'ChaCha20-Poly1305'];
  };

  transferStages: [
    'INITIATED',
    'VALIDATING',
    'TRANSFERRING',
    'VERIFYING',
    'COMPLETED',
    'FAILED'
  ];
}

// 배치 전송 서비스 구현
class BatchTransferService {
  private storageService: StorageService;
  private validationService: ValidationService;
  private checksumService: ChecksumService;

  async initiateBatchTransfer(
    request: BatchTransferRequest
  ): Promise<BatchTransferJob> {
    // 전송 작업 생성
    const jobId = crypto.randomUUID();

    const job: BatchTransferJob = {
      jobId,
      status: 'INITIATED',
      request,
      createdAt: new Date().toISOString(),
      stages: []
    };

    // 요청 검증
    job.stages.push({
      stage: 'VALIDATING',
      startedAt: new Date().toISOString()
    });

    const validation = await this.validateRequest(request);
    if (!validation.valid) {
      job.status = 'FAILED';
      job.error = validation.errors;
      return job;
    }

    job.stages[0].completedAt = new Date().toISOString();

    // 전송 시작
    this.executeTransfer(job);

    return job;
  }

  private async executeTransfer(job: BatchTransferJob): Promise<void> {
    try {
      // 단계: TRANSFERRING
      job.status = 'TRANSFERRING';
      job.stages.push({
        stage: 'TRANSFERRING',
        startedAt: new Date().toISOString()
      });

      const files = await this.transferFiles(job.request);

      job.stages[job.stages.length - 1].completedAt = new Date().toISOString();
      job.stages[job.stages.length - 1].details = {
        filesTransferred: files.length,
        totalBytes: files.reduce((sum, f) => sum + f.size, 0)
      };

      // 단계: VERIFYING
      job.status = 'VERIFYING';
      job.stages.push({
        stage: 'VERIFYING',
        startedAt: new Date().toISOString()
      });

      const verification = await this.verifyTransfer(files, job.request);

      if (!verification.success) {
        throw new TransferError('검증 실패', verification.errors);
      }

      job.stages[job.stages.length - 1].completedAt = new Date().toISOString();

      // 완료
      job.status = 'COMPLETED';
      job.completedAt = new Date().toISOString();

    } catch (error) {
      job.status = 'FAILED';
      job.error = (error as Error).message;
      job.completedAt = new Date().toISOString();
    }
  }

  private async transferFiles(
    request: BatchTransferRequest
  ): Promise<TransferredFile[]> {
    const files: TransferredFile[] = [];

    for (const file of request.files) {
      // 전송 전 체크섬 계산
      const sourceChecksum = await this.checksumService.calculate(
        file.sourcePath,
        'SHA-256'
      );

      // 진행 상황 추적과 함께 전송
      const result = await this.storageService.transfer(
        file.sourcePath,
        file.destinationPath,
        {
          encryption: request.encryption,
          compression: request.compression,
          onProgress: (progress) => {
            this.emitProgress(request.jobId, file.sourcePath, progress);
          }
        }
      );

      // 체크섬 검증
      const destChecksum = await this.checksumService.calculate(
        file.destinationPath,
        'SHA-256'
      );

      if (sourceChecksum !== destChecksum) {
        throw new TransferError(
          `${file.sourcePath}에 대한 체크섬 불일치`
        );
      }

      files.push({
        sourcePath: file.sourcePath,
        destinationPath: result.path,
        size: result.size,
        checksum: destChecksum,
        transferredAt: new Date().toISOString()
      });
    }

    return files;
  }
}
```

### 5.5 품질 제어 프로토콜

```typescript
// 데이터 품질 제어 프로토콜
interface QualityControlProtocol {
  checkpoints: {
    ingestion: {
      timing: '데이터 수신 시';
      checks: [
        '형식 검증',
        '스키마 준수',
        '완전성 검사',
        '중복 탐지'
      ];
    };
    processing: {
      timing: '데이터 처리 중';
      checks: [
        '편집 규칙 검증',
        '일관성 검사',
        '범위 검증',
        '변수 간 검증'
      ];
    };
    output: {
      timing: '데이터 공개 전';
      checks: [
        '공개 위험 평가',
        '정확성 검증',
        '메타데이터 완전성',
        '형식 검증'
      ];
    };
  };

  alertLevels: {
    INFO: '정보성, 조치 필요 없음';
    WARNING: '검토 권장';
    ERROR: '수정 필요';
    CRITICAL: '처리 중단';
  };
}

// 품질 제어 서비스 구현
class QualityControlService {
  private ruleEngine: RuleEngine;
  private alertService: AlertService;
  private auditService: AuditService;

  async validateData(
    data: CensusData,
    checkpoint: QualityCheckpoint
  ): Promise<QualityReport> {
    const report: QualityReport = {
      checkpointId: crypto.randomUUID(),
      checkpoint: checkpoint.name,
      timestamp: new Date().toISOString(),
      recordsChecked: data.records.length,
      issues: [],
      summary: {
        passed: 0,
        warnings: 0,
        errors: 0,
        critical: 0
      }
    };

    // 체크포인트의 모든 규칙 실행
    for (const rule of checkpoint.rules) {
      const ruleResults = await this.ruleEngine.evaluate(data, rule);

      for (const result of ruleResults) {
        if (!result.passed) {
          const issue: QualityIssue = {
            ruleId: rule.id,
            ruleName: rule.name,
            severity: result.severity,
            recordId: result.recordId,
            field: result.field,
            message: result.message,
            suggestion: result.suggestion
          };

          report.issues.push(issue);

          // 요약 업데이트
          switch (result.severity) {
            case 'WARNING':
              report.summary.warnings++;
              break;
            case 'ERROR':
              report.summary.errors++;
              break;
            case 'CRITICAL':
              report.summary.critical++;
              break;
          }
        } else {
          report.summary.passed++;
        }
      }
    }

    // 필요한 경우 알림 생성
    if (report.summary.critical > 0) {
      await this.alertService.sendAlert({
        level: 'CRITICAL',
        message: `심각한 품질 문제 탐지: ${report.summary.critical}`,
        checkpoint: checkpoint.name,
        reportId: report.checkpointId
      });
    }

    // 감사 로깅
    await this.auditService.log({
      action: 'QUALITY_CHECK',
      checkpoint: checkpoint.name,
      result: report.summary
    });

    return report;
  }

  async runEditRules(
    record: CensusRecord
  ): Promise<EditResult[]> {
    const results: EditResult[] = [];

    // 인구통계 일관성
    results.push(await this.checkAgeConsistency(record));
    results.push(await this.checkMaritalStatusAge(record));
    results.push(await this.checkEducationAge(record));
    results.push(await this.checkEmploymentAge(record));

    // 가구 일관성
    if (record.householdRelationship) {
      results.push(await this.checkHouseholdConsistency(record));
    }

    // 지리적 일관성
    results.push(await this.checkGeographicConsistency(record));

    return results;
  }

  private async checkAgeConsistency(
    record: CensusRecord
  ): Promise<EditResult> {
    const age = record.age;
    const dateOfBirth = record.dateOfBirth;

    if (age !== undefined && dateOfBirth !== undefined) {
      const calculatedAge = this.calculateAge(dateOfBirth);

      if (Math.abs(age - calculatedAge) > 1) {
        return {
          passed: false,
          ruleId: 'AGE_DOB_CONSISTENCY',
          severity: 'ERROR',
          message: `나이 (${age})가 생년월일과 불일치 (계산: ${calculatedAge})`,
          suggestion: '나이와 생년월일 확인'
        };
      }
    }

    return { passed: true, ruleId: 'AGE_DOB_CONSISTENCY' };
  }
}
```

### 5.6 동기화 프로토콜

```typescript
// 시스템 간 동기화 프로토콜
interface SynchronizationProtocol {
  name: 'Census Sync Protocol';
  version: '1.0';

  syncModes: {
    fullSync: {
      description: '전체 데이터 대체';
      useCase: '초기 로드 또는 복구';
      frequency: '요청 시';
    };
    incrementalSync: {
      description: '마지막 동기화 이후 변경 사항';
      useCase: '정기 업데이트';
      frequency: '시간별 ~ 일별';
    };
    realTimeSync: {
      description: '즉시 전파';
      useCase: '중요 업데이트';
      frequency: '지속적';
    };
  };

  conflictResolution: {
    strategy: '감사가 있는 최종 작성자 승리';
    timestampPrecision: '마이크로초';
    tieBreaker: '소스 시스템 우선순위';
  };
}

// 동기화 서비스 구현
class CensusSyncService {
  private syncState: SyncStateManager;
  private changeTracker: ChangeTracker;
  private conflictResolver: ConflictResolver;

  async performIncrementalSync(
    sourceSystem: string,
    targetSystem: string
  ): Promise<SyncResult> {
    // 마지막 동기화 상태 가져오기
    const lastSync = await this.syncState.getLastSync(
      sourceSystem,
      targetSystem
    );

    // 마지막 동기화 이후 변경 사항 가져오기
    const changes = await this.changeTracker.getChangesSince(
      sourceSystem,
      lastSync?.timestamp || new Date(0).toISOString()
    );

    if (changes.length === 0) {
      return {
        syncId: crypto.randomUUID(),
        status: 'NO_CHANGES',
        changesProcessed: 0
      };
    }

    const syncId = crypto.randomUUID();
    const results: ChangeResult[] = [];

    for (const change of changes) {
      try {
        const result = await this.applyChange(
          change,
          targetSystem,
          lastSync
        );
        results.push(result);
      } catch (error) {
        results.push({
          changeId: change.id,
          status: 'FAILED',
          error: (error as Error).message
        });
      }
    }

    // 동기화 상태 업데이트
    await this.syncState.updateSync({
      syncId,
      sourceSystem,
      targetSystem,
      timestamp: new Date().toISOString(),
      changesProcessed: results.filter(r => r.status === 'APPLIED').length,
      changesFailed: results.filter(r => r.status === 'FAILED').length
    });

    return {
      syncId,
      status: 'COMPLETED',
      changesProcessed: results.length,
      applied: results.filter(r => r.status === 'APPLIED').length,
      conflicts: results.filter(r => r.status === 'CONFLICT').length,
      failed: results.filter(r => r.status === 'FAILED').length,
      details: results
    };
  }
}
```

---

**WIA-CENSUS-DATA 제어 프로토콜**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
