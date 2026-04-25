# Chapter 5: Census Data Control Protocols

## Communication and Data Transfer Standards

### 5.1 Protocol Architecture Overview

The WIA-CENSUS-DATA standard defines comprehensive communication protocols for secure, reliable census data transfer between systems. This chapter covers the transport layer protocols, message formats, and quality control mechanisms essential for census operations.

```typescript
// Census Communication Protocol Architecture
interface CensusProtocolArchitecture {
  version: '1.0.0';

  transportLayers: {
    primary: {
      protocol: 'HTTPS/TLS 1.3';
      ports: [443];
      encryption: 'AES-256-GCM';
      certificateRequirements: 'Government-approved CA';
    };
    realtime: {
      protocol: 'WebSocket Secure (WSS)';
      purpose: 'Real-time enumeration updates';
      heartbeatInterval: 30000; // ms
    };
    bulk: {
      protocol: 'SFTP / SCP';
      purpose: 'Large file transfers';
      encryption: 'AES-256';
    };
    legacy: {
      protocol: 'HTTPS with VPN';
      purpose: 'Legacy system integration';
      deprecationDate: '2027-12-31';
    };
  };

  messageFormats: {
    standard: 'JSON with schema validation';
    statistical: 'SDMX-JSON / SDMX-ML';
    binary: 'Protocol Buffers for high-volume';
    streaming: 'NDJSON for real-time';
  };

  qualityOfService: {
    reliability: 'At-least-once delivery guaranteed';
    ordering: 'Sequence numbers for ordering';
    idempotency: 'Idempotency keys required';
    retry: 'Exponential backoff with jitter';
  };
}

// Protocol Handler Implementation
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
    // Assign sequence number
    const sequenceNumber = this.sequenceManager.nextSequence(destination);

    // Add protocol headers
    const enrichedMessage = this.enrichMessage(message, sequenceNumber);

    // Validate message
    const validation = await this.messageValidator.validate(enrichedMessage);
    if (!validation.valid) {
      throw new ValidationError('Message validation failed', validation.errors);
    }

    // Serialize message
    const serialized = this.serializeMessage(enrichedMessage);

    // Send with retry logic
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
      `Failed after ${retryConfig.maxRetries} attempts`,
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

### 5.2 Secure Data Transfer Protocol

```typescript
// Secure Census Data Transfer Protocol
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
        pinning: 'Optional but recommended';
      };
    };

    message: {
      encryption: 'Envelope encryption';
      keyManagement: 'HSM-backed key storage';
      integrityCheck: 'HMAC-SHA256';
    };

    application: {
      authentication: 'Mutual TLS + OAuth2';
      authorization: 'Attribute-based access control';
      auditLogging: 'All operations logged';
    };
  };
}

// Secure Transfer Service Implementation
class SecureTransferService {
  private keyManager: KeyManager;
  private encryptionService: EncryptionService;
  private auditLogger: AuditLogger;

  async secureTransfer(
    data: CensusData,
    recipient: TransferRecipient
  ): Promise<TransferResult> {
    // Get recipient's public key
    const recipientKey = await this.keyManager.getPublicKey(recipient.id);

    // Generate data encryption key
    const dataKey = await this.encryptionService.generateDataKey();

    // Encrypt data with data key
    const encryptedData = await this.encryptionService.encrypt(
      this.serializeData(data),
      dataKey
    );

    // Encrypt data key with recipient's public key
    const encryptedKey = await this.encryptionService.encryptKey(
      dataKey,
      recipientKey
    );

    // Create transfer envelope
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

    // Sign envelope
    const signature = await this.signEnvelope(envelope);
    envelope.signature = signature;

    // Log transfer initiation
    await this.auditLogger.log({
      action: 'TRANSFER_INITIATED',
      sender: this.getSenderId(),
      recipient: recipient.id,
      dataSize: data.records?.length || 0,
      timestamp: new Date().toISOString()
    });

    // Transmit
    const result = await this.transmitEnvelope(envelope, recipient.endpoint);

    // Log completion
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
    // Verify signature
    const signatureValid = await this.verifySignature(
      envelope,
      envelope.signature
    );

    if (!signatureValid) {
      throw new SecurityError('Invalid envelope signature');
    }

    // Verify integrity
    const integrityValid = await this.verifyIntegrity(
      Buffer.from(envelope.encryptedData, 'base64'),
      envelope.integrity
    );

    if (!integrityValid) {
      throw new SecurityError('Data integrity check failed');
    }

    // Decrypt data key
    const dataKey = await this.encryptionService.decryptKey(
      Buffer.from(envelope.encryptedKey, 'base64'),
      await this.keyManager.getPrivateKey()
    );

    // Decrypt data
    const decryptedData = await this.encryptionService.decrypt(
      Buffer.from(envelope.encryptedData, 'base64'),
      dataKey
    );

    // Log receipt
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

### 5.3 Real-Time Data Streaming Protocol

```typescript
// Real-Time Census Streaming Protocol
interface CensusStreamingProtocol {
  name: 'Census Real-Time Stream';
  transport: 'WebSocket Secure';

  messageTypes: {
    // Control messages
    CONNECT: 'Initiate connection';
    DISCONNECT: 'Graceful disconnect';
    HEARTBEAT: 'Keep-alive ping';
    ACK: 'Message acknowledgment';

    // Data messages
    ENUMERATION_UPDATE: 'New enumeration data';
    RESPONSE_RECEIVED: 'Census response received';
    QUALITY_ALERT: 'Data quality issue';
    PROGRESS_UPDATE: 'Collection progress';

    // Administrative
    CONFIG_UPDATE: 'Configuration change';
    PAUSE_STREAM: 'Pause data flow';
    RESUME_STREAM: 'Resume data flow';
  };

  qualityOfService: {
    deliveryGuarantee: 'At-least-once';
    orderingGuarantee: 'Per-partition ordering';
    acknowledgmentRequired: true;
    windowSize: 1000; // Outstanding messages
  };
}

// Streaming Message Format
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

// Real-Time Streaming Service
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
        reject(new ConnectionError('WebSocket connection failed', error));
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
      throw new ConnectionError('Not connected');
    }

    // Ensure message has required fields
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

    // Buffer message for potential retry
    this.messageBuffer.add(enrichedMessage);

    // Track acknowledgment
    const ackPromise = this.acknowledgmentTracker.waitForAck(
      enrichedMessage.header.messageId
    );

    // Send message
    ws.send(JSON.stringify(enrichedMessage));

    // Wait for acknowledgment
    const ack = await ackPromise;

    return {
      messageId: enrichedMessage.header.messageId,
      acknowledged: true,
      timestamp: ack.timestamp
    };
  }

  private handleMessage(clientId: string, data: WebSocket.Data): void {
    const message: StreamMessage = JSON.parse(data.toString());

    switch (message.header.messageType) {
      case 'ACK':
        this.acknowledgmentTracker.acknowledge(
          message.payload.messageId,
          message.header.timestamp
        );
        break;

      case 'HEARTBEAT':
        this.sendHeartbeatResponse(clientId);
        break;

      case 'ENUMERATION_UPDATE':
      case 'RESPONSE_RECEIVED':
      case 'QUALITY_ALERT':
        this.emitDataEvent(clientId, message);
        // Send acknowledgment
        this.sendAck(clientId, message.header.messageId);
        break;

      default:
        console.warn(`Unknown message type: ${message.header.messageType}`);
    }
  }

  private setupHeartbeat(clientId: string, ws: WebSocket): void {
    const interval = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
          header: {
            messageId: crypto.randomUUID(),
            messageType: 'HEARTBEAT',
            timestamp: new Date().toISOString()
          },
          payload: {}
        }));
      } else {
        clearInterval(interval);
      }
    }, 30000);
  }
}

// Message Buffer for Reliability
class MessageBuffer {
  private buffer: Map<string, BufferedMessage>;
  private maxSize: number;

  constructor(maxSize: number) {
    this.buffer = new Map();
    this.maxSize = maxSize;
  }

  add(message: StreamMessage): void {
    if (this.buffer.size >= this.maxSize) {
      // Remove oldest unacknowledged message
      const oldest = this.findOldest();
      if (oldest) {
        this.buffer.delete(oldest);
      }
    }

    this.buffer.set(message.header.messageId, {
      message,
      addedAt: Date.now(),
      retryCount: 0
    });
  }

  acknowledge(messageId: string): void {
    this.buffer.delete(messageId);
  }

  getUnacknowledged(olderThan: number): BufferedMessage[] {
    const threshold = Date.now() - olderThan;
    return Array.from(this.buffer.values())
      .filter(m => m.addedAt < threshold);
  }

  private findOldest(): string | undefined {
    let oldest: { id: string; time: number } | undefined;

    for (const [id, msg] of this.buffer) {
      if (!oldest || msg.addedAt < oldest.time) {
        oldest = { id, time: msg.addedAt };
      }
    }

    return oldest?.id;
  }
}

interface BufferedMessage {
  message: StreamMessage;
  addedAt: number;
  retryCount: number;
}
```

### 5.4 Batch Transfer Protocol

```typescript
// Batch Census Data Transfer Protocol
interface BatchTransferProtocol {
  name: 'Census Batch Transfer';
  version: '1.0';

  transferModes: {
    push: {
      description: 'Sender initiates transfer';
      protocol: 'SFTP / HTTPS PUT';
      useCases: ['Enumeration data upload', 'Administrative data'];
    };
    pull: {
      description: 'Receiver requests data';
      protocol: 'HTTPS GET / SFTP GET';
      useCases: ['Data dissemination', 'Partner access'];
    };
    exchange: {
      description: 'Bidirectional sync';
      protocol: 'Custom sync protocol';
      useCases: ['Inter-agency data sharing'];
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

// Batch Transfer Service Implementation
class BatchTransferService {
  private storageService: StorageService;
  private validationService: ValidationService;
  private checksumService: ChecksumService;

  async initiateBatchTransfer(
    request: BatchTransferRequest
  ): Promise<BatchTransferJob> {
    // Create transfer job
    const jobId = crypto.randomUUID();

    const job: BatchTransferJob = {
      jobId,
      status: 'INITIATED',
      request,
      createdAt: new Date().toISOString(),
      stages: []
    };

    // Validate request
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

    // Start transfer
    this.executeTransfer(job);

    return job;
  }

  private async executeTransfer(job: BatchTransferJob): Promise<void> {
    try {
      // Stage: TRANSFERRING
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

      // Stage: VERIFYING
      job.status = 'VERIFYING';
      job.stages.push({
        stage: 'VERIFYING',
        startedAt: new Date().toISOString()
      });

      const verification = await this.verifyTransfer(files, job.request);

      if (!verification.success) {
        throw new TransferError('Verification failed', verification.errors);
      }

      job.stages[job.stages.length - 1].completedAt = new Date().toISOString();

      // Complete
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
      // Calculate checksum before transfer
      const sourceChecksum = await this.checksumService.calculate(
        file.sourcePath,
        'SHA-256'
      );

      // Transfer with progress tracking
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

      // Verify checksum
      const destChecksum = await this.checksumService.calculate(
        file.destinationPath,
        'SHA-256'
      );

      if (sourceChecksum !== destChecksum) {
        throw new TransferError(
          `Checksum mismatch for ${file.sourcePath}`
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

  async getTransferStatus(jobId: string): Promise<BatchTransferJob> {
    return this.storageService.getJob(jobId);
  }

  async cancelTransfer(jobId: string): Promise<void> {
    const job = await this.getTransferStatus(jobId);

    if (job.status === 'COMPLETED' || job.status === 'FAILED') {
      throw new Error('Cannot cancel completed or failed transfer');
    }

    // Mark as cancelled and clean up
    job.status = 'FAILED';
    job.error = 'Cancelled by user';
    job.completedAt = new Date().toISOString();

    await this.storageService.updateJob(job);
    await this.storageService.cleanup(job);
  }
}

interface BatchTransferRequest {
  jobId?: string;
  transferType: 'PUSH' | 'PULL';
  files: FileSpec[];
  encryption: EncryptionSpec;
  compression: CompressionSpec;
  validation: ValidationSpec;
  notification?: NotificationSpec;
}

interface BatchTransferJob {
  jobId: string;
  status: string;
  request: BatchTransferRequest;
  createdAt: string;
  completedAt?: string;
  stages: TransferStage[];
  error?: any;
}

interface TransferStage {
  stage: string;
  startedAt: string;
  completedAt?: string;
  details?: any;
}
```

### 5.5 Quality Control Protocol

```typescript
// Data Quality Control Protocol
interface QualityControlProtocol {
  checkpoints: {
    ingestion: {
      timing: 'On data receipt';
      checks: [
        'Format validation',
        'Schema compliance',
        'Completeness check',
        'Duplicate detection'
      ];
    };
    processing: {
      timing: 'During data processing';
      checks: [
        'Edit rules validation',
        'Consistency checks',
        'Range validation',
        'Cross-variable validation'
      ];
    };
    output: {
      timing: 'Before data release';
      checks: [
        'Disclosure risk assessment',
        'Accuracy verification',
        'Metadata completeness',
        'Format validation'
      ];
    };
  };

  alertLevels: {
    INFO: 'Informational, no action required';
    WARNING: 'Review recommended';
    ERROR: 'Correction required';
    CRITICAL: 'Processing halted';
  };
}

// Quality Control Service Implementation
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

    // Run all rules for checkpoint
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

          // Update summary
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

    // Generate alerts if needed
    if (report.summary.critical > 0) {
      await this.alertService.sendAlert({
        level: 'CRITICAL',
        message: `Critical quality issues detected: ${report.summary.critical}`,
        checkpoint: checkpoint.name,
        reportId: report.checkpointId
      });
    }

    // Log audit
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

    // Demographic consistency
    results.push(await this.checkAgeConsistency(record));
    results.push(await this.checkMaritalStatusAge(record));
    results.push(await this.checkEducationAge(record));
    results.push(await this.checkEmploymentAge(record));

    // Household consistency
    if (record.householdRelationship) {
      results.push(await this.checkHouseholdConsistency(record));
    }

    // Geographic consistency
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
          message: `Age (${age}) inconsistent with date of birth (calculated: ${calculatedAge})`,
          suggestion: 'Verify age and date of birth'
        };
      }
    }

    return { passed: true, ruleId: 'AGE_DOB_CONSISTENCY' };
  }

  private async checkMaritalStatusAge(
    record: CensusRecord
  ): Promise<EditResult> {
    const age = record.age;
    const maritalStatus = record.maritalStatus;

    if (age !== undefined && maritalStatus !== undefined) {
      // Check if age is consistent with marital status
      if (age < 15 && maritalStatus !== 'NEVER_MARRIED') {
        return {
          passed: false,
          ruleId: 'MARITAL_AGE_CONSISTENCY',
          severity: 'WARNING',
          message: `Marital status '${maritalStatus}' unusual for age ${age}`,
          suggestion: 'Verify marital status for young person'
        };
      }
    }

    return { passed: true, ruleId: 'MARITAL_AGE_CONSISTENCY' };
  }
}

interface QualityReport {
  checkpointId: string;
  checkpoint: string;
  timestamp: string;
  recordsChecked: number;
  issues: QualityIssue[];
  summary: {
    passed: number;
    warnings: number;
    errors: number;
    critical: number;
  };
}

interface QualityIssue {
  ruleId: string;
  ruleName: string;
  severity: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
  recordId?: string;
  field?: string;
  message: string;
  suggestion?: string;
}

interface EditResult {
  passed: boolean;
  ruleId: string;
  severity?: string;
  message?: string;
  suggestion?: string;
}
```

### 5.6 Synchronization Protocol

```typescript
// Inter-System Synchronization Protocol
interface SynchronizationProtocol {
  name: 'Census Sync Protocol';
  version: '1.0';

  syncModes: {
    fullSync: {
      description: 'Complete data replacement';
      useCase: 'Initial load or recovery';
      frequency: 'On demand';
    };
    incrementalSync: {
      description: 'Changes since last sync';
      useCase: 'Regular updates';
      frequency: 'Hourly to daily';
    };
    realTimeSync: {
      description: 'Immediate propagation';
      useCase: 'Critical updates';
      frequency: 'Continuous';
    };
  };

  conflictResolution: {
    strategy: 'Last-writer-wins with audit';
    timestampPrecision: 'Microseconds';
    tieBreaker: 'Source system priority';
  };
}

// Synchronization Service Implementation
class CensusSyncService {
  private syncState: SyncStateManager;
  private changeTracker: ChangeTracker;
  private conflictResolver: ConflictResolver;

  async performIncrementalSync(
    sourceSystem: string,
    targetSystem: string
  ): Promise<SyncResult> {
    // Get last sync state
    const lastSync = await this.syncState.getLastSync(
      sourceSystem,
      targetSystem
    );

    // Get changes since last sync
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

    // Update sync state
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

  private async applyChange(
    change: DataChange,
    targetSystem: string,
    lastSync: SyncState | null
  ): Promise<ChangeResult> {
    // Check for conflicts
    const targetRecord = await this.getTargetRecord(
      targetSystem,
      change.recordId
    );

    if (targetRecord) {
      const conflict = this.detectConflict(change, targetRecord, lastSync);

      if (conflict) {
        const resolution = await this.conflictResolver.resolve(
          change,
          targetRecord,
          conflict
        );

        if (resolution.action === 'SKIP') {
          return {
            changeId: change.id,
            status: 'CONFLICT',
            resolution: 'SKIPPED',
            details: conflict
          };
        }
      }
    }

    // Apply change
    await this.applyToTarget(targetSystem, change);

    return {
      changeId: change.id,
      status: 'APPLIED',
      recordId: change.recordId
    };
  }

  private detectConflict(
    change: DataChange,
    targetRecord: CensusRecord,
    lastSync: SyncState | null
  ): Conflict | null {
    // Check if target was modified after last sync
    if (lastSync && targetRecord.modifiedAt > lastSync.timestamp) {
      // Check if same fields were modified
      const sourceFields = Object.keys(change.changedFields);
      const targetChanges = this.getFieldChanges(
        targetRecord,
        lastSync.timestamp
      );

      const conflictingFields = sourceFields.filter(f =>
        targetChanges.includes(f)
      );

      if (conflictingFields.length > 0) {
        return {
          type: 'CONCURRENT_MODIFICATION',
          fields: conflictingFields,
          sourceTimestamp: change.timestamp,
          targetTimestamp: targetRecord.modifiedAt
        };
      }
    }

    return null;
  }
}

interface DataChange {
  id: string;
  recordId: string;
  changeType: 'INSERT' | 'UPDATE' | 'DELETE';
  changedFields: { [field: string]: any };
  timestamp: string;
  sourceSystem: string;
}

interface SyncResult {
  syncId: string;
  status: string;
  changesProcessed: number;
  applied?: number;
  conflicts?: number;
  failed?: number;
  details?: ChangeResult[];
}

interface Conflict {
  type: string;
  fields: string[];
  sourceTimestamp: string;
  targetTimestamp: string;
}
```

---

**WIA-CENSUS-DATA Control Protocols**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
