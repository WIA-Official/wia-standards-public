/**
 * WIA-TIME-012: Matter Transmission SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  MatterTransmissionConfig,
  AnalysisRequest,
  AnalysisResult,
  DisassemblyRequest,
  DisassemblyResult,
  TransmissionRequest,
  TransmissionResult,
  TransmissionProgress,
  TransmissionError,
  ReassemblyRequest,
  ReassemblyResult,
  VerificationRequest,
  VerificationResult,
  BackupRequest,
  BackupResult,
  LivingMatterSubject,
  LivingMatterProtocol,
  Destination,
  TransmissionStats,
  ObjectIdentifier,
  EncodedMatter,
  MatterTransmissionError,
  EventHandler,
  DisassemblyResolution,
  TransmissionPriority,
  ErrorCorrectionLevel
} from './types';

/**
 * Matter Transmission SDK
 */
export class MatterTransmissionSDK {
  private config: MatterTransmissionConfig;
  private apiClient: ApiClient;

  /**
   * Initialize the Matter Transmission SDK
   */
  constructor(config: MatterTransmissionConfig) {
    this.config = {
      baseUrl: 'https://api.wiastandards.com/time-012',
      defaultErrorCorrection: ErrorCorrectionLevel.MAXIMUM,
      timeout: 3600,
      retryAttempts: 3,
      logging: true,
      logLevel: 'info',
      ...config
    };

    this.apiClient = new ApiClient(this.config);
  }

  // ========================================================================
  // Matter Analysis
  // ========================================================================

  /**
   * Analyze matter for transmission compatibility
   */
  async analyze(request: AnalysisRequest): Promise<AnalysisResult> {
    this.log('info', 'Analyzing matter', { objectId: request.object.id });

    try {
      const result = await this.apiClient.post<AnalysisResult>(
        '/analyze',
        request
      );

      this.log('info', 'Analysis complete', {
        objectId: request.object.id,
        transmissible: result.transmissible,
        atomCount: result.atomCount
      });

      return result;
    } catch (error) {
      this.log('error', 'Analysis failed', { error });
      throw new MatterTransmissionError(
        'Matter analysis failed',
        'ANALYSIS_ERROR',
        { error }
      );
    }
  }

  // ========================================================================
  // Disassembly
  // ========================================================================

  /**
   * Disassemble matter to molecular/atomic level
   */
  async disassemble(request: DisassemblyRequest): Promise<DisassemblyResult> {
    this.log('info', 'Disassembling matter', {
      object: request.object,
      resolution: request.resolution
    });

    try {
      // Create backup if requested
      if (request.createBackup) {
        await this.backup.create({
          object: request.object,
          storage: 'quantum_vault',
          redundancy: request.backupRedundancy || 3
        });
      }

      const result = await this.apiClient.post<DisassemblyResult>(
        '/disassemble',
        request
      );

      // Verify disassembly
      if (result.verification.completeness < 0.99) {
        throw new MatterTransmissionError(
          'Disassembly verification failed',
          'DISASSEMBLY_INCOMPLETE',
          { verification: result.verification }
        );
      }

      this.log('info', 'Disassembly complete', {
        atomCount: result.atomCount,
        encodingSize: result.encodingSize
      });

      return result;
    } catch (error) {
      this.log('error', 'Disassembly failed', { error });
      throw error;
    }
  }

  // ========================================================================
  // Transmission
  // ========================================================================

  /**
   * Transmit matter through time
   */
  async transmit(request: TransmissionRequest): Promise<TransmissionResult> {
    this.log('info', 'Initiating matter transmission', {
      destination: request.destination,
      priority: request.priority
    });

    try {
      // Validate transmission request
      this.validateTransmissionRequest(request);

      // Apply default quantum preservation if not specified
      if (!request.quantumStatePreservation) {
        request.quantumStatePreservation = this.config.defaultQuantumPreservation;
      }

      const result = await this.apiClient.post<TransmissionResult>(
        '/transmit',
        request
      );

      this.log('info', 'Transmission initiated', {
        transmissionId: result.id,
        status: result.status,
        estimatedArrival: result.estimatedArrival
      });

      return result;
    } catch (error) {
      this.log('error', 'Transmission failed', { error });
      throw new MatterTransmissionError(
        'Matter transmission failed',
        'TRANSMISSION_ERROR',
        { error }
      );
    }
  }

  /**
   * Monitor transmission progress
   */
  monitor(transmissionId: string): TransmissionMonitor {
    return new TransmissionMonitor(transmissionId, this.apiClient, this.config);
  }

  /**
   * Abort transmission
   */
  async abort(transmissionId: string): Promise<void> {
    this.log('warn', 'Aborting transmission', { transmissionId });

    await this.apiClient.post(`/transmit/${transmissionId}/abort`, {});

    this.log('info', 'Transmission aborted', { transmissionId });
  }

  // ========================================================================
  // Reassembly
  // ========================================================================

  /**
   * Reassemble matter at destination
   */
  async reassemble(request: ReassemblyRequest): Promise<ReassemblyResult> {
    this.log('info', 'Reassembling matter', {
      transmissionId: request.transmissionId,
      verificationLevel: request.verificationLevel
    });

    try {
      const result = await this.apiClient.post<ReassemblyResult>(
        '/reassemble',
        request
      );

      if (!result.success) {
        throw new MatterTransmissionError(
          'Reassembly failed',
          'REASSEMBLY_ERROR',
          { errors: result.errors }
        );
      }

      this.log('info', 'Reassembly complete', {
        accuracy: result.accuracy,
        quantumFidelity: result.quantumFidelity
      });

      return result;
    } catch (error) {
      this.log('error', 'Reassembly failed', { error });
      throw error;
    }
  }

  // ========================================================================
  // Verification
  // ========================================================================

  /**
   * Verify transmission integrity
   */
  async verify(request: VerificationRequest): Promise<VerificationResult> {
    this.log('info', 'Verifying transmission integrity');

    try {
      const result = await this.apiClient.post<VerificationResult>(
        '/verify',
        request
      );

      if (!result.verified) {
        this.log('warn', 'Verification failed', {
          errors: result.errors,
          accuracy: result.accuracy
        });
      }

      return result;
    } catch (error) {
      this.log('error', 'Verification error', { error });
      throw new MatterTransmissionError(
        'Verification failed',
        'VERIFICATION_ERROR',
        { error }
      );
    }
  }

  // ========================================================================
  // Living Matter
  // ========================================================================

  /**
   * Transmit living matter (special protocol)
   */
  async transmitLiving(params: {
    subject: LivingMatterSubject;
    destination: Destination;
    protocol: LivingMatterProtocol;
  }): Promise<TransmissionResult> {
    this.log('info', 'Initiating living matter transmission', {
      subjectId: params.subject.id,
      subjectType: params.subject.type
    });

    try {
      // Verify ethics compliance
      this.verifyEthicsCompliance(params);

      // Perform medical screening
      const screening = await this.medicalScreening(params.subject);
      if (!screening.approved) {
        throw new MatterTransmissionError(
          'Medical screening failed',
          'MEDICAL_SCREENING_FAILED',
          { reasons: screening.reasons }
        );
      }

      // Create neural backup
      if (params.protocol.neuralStateBackup) {
        await this.createNeuralBackup(params.subject);
      }

      // Execute living matter transmission
      const result = await this.apiClient.post<TransmissionResult>(
        '/transmit/living',
        params
      );

      this.log('info', 'Living matter transmission initiated', {
        transmissionId: result.id
      });

      return result;
    } catch (error) {
      this.log('error', 'Living matter transmission failed', { error });
      throw error;
    }
  }

  // ========================================================================
  // Backup Management
  // ========================================================================

  /**
   * Backup management
   */
  backup = {
    /**
     * Create matter backup
     */
    create: async (request: BackupRequest): Promise<BackupResult> => {
      this.log('info', 'Creating backup', { object: request.object });

      const result = await this.apiClient.post<BackupResult>(
        '/backup/create',
        request
      );

      this.log('info', 'Backup created', { backupId: result.id });

      return result;
    },

    /**
     * Restore from backup
     */
    restore: async (backupId: string): Promise<void> => {
      this.log('info', 'Restoring from backup', { backupId });

      await this.apiClient.post(`/backup/${backupId}/restore`, {});

      this.log('info', 'Backup restored', { backupId });
    },

    /**
     * Delete backup
     */
    delete: async (backupId: string): Promise<void> => {
      this.log('info', 'Deleting backup', { backupId });

      await this.apiClient.delete(`/backup/${backupId}`);

      this.log('info', 'Backup deleted', { backupId });
    },

    /**
     * List backups
     */
    list: async (filter?: string): Promise<BackupResult[]> => {
      const result = await this.apiClient.get<BackupResult[]>('/backup/list', {
        filter
      });

      return result;
    }
  };

  // ========================================================================
  // Statistics
  // ========================================================================

  /**
   * Get transmission statistics
   */
  async getStats(params: {
    period?: string;
    groupBy?: string;
  }): Promise<TransmissionStats> {
    const result = await this.apiClient.get<TransmissionStats>('/stats', params);
    return result;
  }

  // ========================================================================
  // Private Methods
  // ========================================================================

  private validateTransmissionRequest(request: TransmissionRequest): void {
    // Check encoding size
    if (request.encodedMatter.metadata.totalAtoms > 1e24) {
      throw new MatterTransmissionError(
        'Object too complex for transmission',
        'COMPLEXITY_LIMIT_EXCEEDED',
        { atomCount: request.encodedMatter.metadata.totalAtoms }
      );
    }

    // Check mass limit
    if (request.encodedMatter.metadata.totalMass > 1000) {
      throw new MatterTransmissionError(
        'Mass exceeds transmission limit (1000kg)',
        'MASS_LIMIT_EXCEEDED',
        { mass: request.encodedMatter.metadata.totalMass }
      );
    }

    // Validate destination
    if (!request.destination.time || !request.destination.location) {
      throw new MatterTransmissionError(
        'Invalid destination coordinates',
        'INVALID_DESTINATION'
      );
    }
  }

  private verifyEthicsCompliance(params: {
    subject: LivingMatterSubject;
    protocol: LivingMatterProtocol;
  }): void {
    // Check informed consent
    if (!params.subject.consent) {
      throw new MatterTransmissionError(
        'Informed consent required',
        'ETHICS_VIOLATION'
      );
    }

    // Check no-cloning compliance
    if (params.protocol.backupClone) {
      throw new MatterTransmissionError(
        'Backup cloning violates no-cloning theorem',
        'ETHICS_VIOLATION'
      );
    }
  }

  private async medicalScreening(
    subject: LivingMatterSubject
  ): Promise<{ approved: boolean; reasons?: string[] }> {
    const result = await this.apiClient.post<{ approved: boolean; reasons?: string[] }>(
      '/medical/screening',
      { subject }
    );

    return result;
  }

  private async createNeuralBackup(subject: LivingMatterSubject): Promise<void> {
    await this.apiClient.post('/neural/backup', { subject });
  }

  private log(level: string, message: string, data?: unknown): void {
    if (this.config.logging) {
      const logLevels = ['debug', 'info', 'warn', 'error'];
      const configLevel = logLevels.indexOf(this.config.logLevel || 'info');
      const messageLevel = logLevels.indexOf(level);

      if (messageLevel >= configLevel) {
        console.log(`[WIA-TIME-012] [${level.toUpperCase()}] ${message}`, data || '');
      }
    }
  }
}

/**
 * Transmission Monitor
 */
class TransmissionMonitor {
  private transmissionId: string;
  private apiClient: ApiClient;
  private config: MatterTransmissionConfig;
  private handlers: Map<string, EventHandler<any>[]> = new Map();
  private intervalId?: NodeJS.Timeout;

  constructor(
    transmissionId: string,
    apiClient: ApiClient,
    config: MatterTransmissionConfig
  ) {
    this.transmissionId = transmissionId;
    this.apiClient = apiClient;
    this.config = config;
  }

  /**
   * Register event handler
   */
  on<T>(event: string, handler: EventHandler<T>): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, []);
    }
    this.handlers.get(event)!.push(handler);
  }

  /**
   * Remove event handler
   */
  off<T>(event: string, handler: EventHandler<T>): void {
    const handlers = this.handlers.get(event);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event
   */
  private emit(event: string, data: any): void {
    const handlers = this.handlers.get(event);
    if (handlers) {
      handlers.forEach(handler => handler(data));
    }
  }

  /**
   * Start monitoring
   */
  start(intervalMs: number = 1000): void {
    this.intervalId = setInterval(async () => {
      try {
        const progress = await this.apiClient.get<TransmissionProgress>(
          `/transmit/${this.transmissionId}/progress`
        );

        this.emit('progress', progress);

        if (progress.percentage >= 100) {
          this.stop();
          this.emit('complete', progress);
        }
      } catch (error) {
        this.emit('error', error);
      }
    }, intervalMs);
  }

  /**
   * Stop monitoring
   */
  stop(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = undefined;
    }
  }

  /**
   * Wait for transmission completion
   */
  async waitForCompletion(): Promise<TransmissionProgress> {
    return new Promise((resolve, reject) => {
      this.on('complete', resolve);
      this.on('error', reject);
      this.start();
    });
  }
}

/**
 * API Client
 */
class ApiClient {
  private config: MatterTransmissionConfig;

  constructor(config: MatterTransmissionConfig) {
    this.config = config;
  }

  async get<T>(endpoint: string, params?: Record<string, any>): Promise<T> {
    const url = new URL(endpoint, this.config.baseUrl);
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    const response = await fetch(url.toString(), {
      method: 'GET',
      headers: this.getHeaders()
    });

    return this.handleResponse<T>(response);
  }

  async post<T>(endpoint: string, data: any): Promise<T> {
    const url = new URL(endpoint, this.config.baseUrl);

    const response = await fetch(url.toString(), {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify(data)
    });

    return this.handleResponse<T>(response);
  }

  async delete(endpoint: string): Promise<void> {
    const url = new URL(endpoint, this.config.baseUrl);

    const response = await fetch(url.toString(), {
      method: 'DELETE',
      headers: this.getHeaders()
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }
  }

  private getHeaders(): Record<string, string> {
    return {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
      'X-WIA-SDK-Version': '1.0.0'
    };
  }

  private async handleResponse<T>(response: Response): Promise<T> {
    if (!response.ok) {
      const error = await response.json();
      throw new MatterTransmissionError(
        error.message || `HTTP ${response.status}`,
        error.code || 'API_ERROR',
        error
      );
    }

    return response.json();
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export default MatterTransmissionSDK;
