/**
 * WIA-LEGAL_TECH SDK
 * Version: 1.0.0
 * Philosophy: 弘익人間 (Hongik Ingan - Benefit All Humanity)
 *
 * AI-powered legal document analysis, contract management, and compliance checking
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import FormData from 'form-data';
import * as fs from 'fs';
import {
  WIALegalTechConfig,
  APIResponse,
  PaginatedResponse,
  UploadResponse,
  ContractAnalysis,
  ContractAnalysisRequest,
  Contract,
  Clause,
  ComplianceCheck,
  ComplianceFramework,
  DocumentReview,
  ResearchResult,
  ResearchFilters,
  LegalResult,
  Template,
  GenerateDocumentRequest,
  GeneratedDocument,
  Workflow,
  WorkflowInstance,
  DocumentSearchResult,
  SearchRequest,
  CaseRecord,
  AuthTokenResponse,
} from './types';

/**
 * Main WIA Legal Tech SDK Client
 */
export class WIALegalTechClient {
  private apiClient: AxiosInstance;
  private config: Required<WIALegalTechConfig>;
  private accessToken?: string;

  constructor(config: WIALegalTechConfig = {}) {
    this.config = {
      apiKey: config.apiKey || process.env.WIA_LEGAL_TECH_API_KEY || '',
      baseUrl: config.baseUrl || process.env.WIA_LEGAL_TECH_BASE_URL || 'https://api.wia-legal.tech/v1',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      debug: config.debug || false,
    };

    this.apiClient = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Legal-Tech-SDK/1.0.0',
      },
    });

    // Add request interceptor for authentication
    this.apiClient.interceptors.request.use((config) => {
      if (this.accessToken) {
        config.headers.Authorization = `Bearer ${this.accessToken}`;
      } else if (this.config.apiKey) {
        config.headers['X-API-Key'] = this.config.apiKey;
      }
      return config;
    });

    // Add response interceptor for error handling
    this.apiClient.interceptors.response.use(
      (response) => response,
      async (error) => {
        if (this.config.debug) {
          console.error('API Error:', error.response?.data || error.message);
        }
        return Promise.reject(error);
      }
    );
  }

  // ============================================================================
  // Authentication
  // ============================================================================

  /**
   * Authenticate with client credentials
   */
  async authenticate(clientId: string, clientSecret: string): Promise<AuthTokenResponse> {
    const response = await this.apiClient.post<AuthTokenResponse>('/auth/token', {
      client_id: clientId,
      client_secret: clientSecret,
      grant_type: 'client_credentials',
      scope: 'contracts.read contracts.write ediscovery.analyze',
    });

    this.accessToken = response.data.access_token;
    return response.data;
  }

  /**
   * Set access token manually
   */
  setAccessToken(token: string): void {
    this.accessToken = token;
  }

  // ============================================================================
  // Contract Analysis
  // ============================================================================

  /**
   * Upload contract for AI analysis
   */
  async uploadContract(
    filePath: string,
    options: Partial<ContractAnalysisRequest> = {}
  ): Promise<UploadResponse> {
    const formData = new FormData();
    formData.append('file', fs.createReadStream(filePath));
    formData.append('analysis_type', options.analysisType || 'full');

    if (options.metadata) {
      formData.append('metadata', JSON.stringify(options.metadata));
    }

    const response = await this.apiClient.post<UploadResponse>(
      '/contracts/upload',
      formData,
      {
        headers: formData.getHeaders(),
      }
    );

    return response.data;
  }

  /**
   * Get contract analysis results
   */
  async getContractAnalysis(contractId: string): Promise<ContractAnalysis> {
    const response = await this.apiClient.get<ContractAnalysis>(
      `/contracts/${contractId}/analysis`
    );
    return response.data;
  }

  /**
   * Wait for contract analysis to complete
   */
  async waitForAnalysis(
    contractId: string,
    pollInterval: number = 2000,
    maxWaitTime: number = 300000
  ): Promise<ContractAnalysis> {
    const startTime = Date.now();

    while (Date.now() - startTime < maxWaitTime) {
      const analysis = await this.getContractAnalysis(contractId);

      if (analysis.status === 'completed') {
        return analysis;
      } else if (analysis.status === 'failed') {
        throw new Error('Contract analysis failed');
      }

      await this.sleep(pollInterval);
    }

    throw new Error('Contract analysis timed out');
  }

  /**
   * Analyze contract from file path (upload + wait for results)
   */
  async analyzeContract(
    filePath: string,
    options: Partial<ContractAnalysisRequest> = {}
  ): Promise<ContractAnalysis> {
    const upload = await this.uploadContract(filePath, options);
    return await this.waitForAnalysis(upload.contractId);
  }

  /**
   * Compare multiple contracts
   */
  async compareContracts(
    contractIds: string[],
    comparisonType: 'full' | 'clauses' | 'financial' | 'obligations' = 'full'
  ): Promise<any> {
    const response = await this.apiClient.post('/contracts/compare', {
      contractIds,
      comparisonType,
    });
    return response.data;
  }

  /**
   * Extract specific clauses from contract
   */
  async extractClauses(
    contractId: string,
    clauseTypes: string[],
    includeContext: boolean = true
  ): Promise<{ contractId: string; clauses: Clause[] }> {
    const response = await this.apiClient.post(`/contracts/${contractId}/extract-clauses`, {
      clauseTypes,
      includeContext,
    });
    return response.data;
  }

  /**
   * Get clause recommendations for contract type
   */
  async getClauseRecommendations(
    contractType: string,
    jurisdiction: string,
    riskTolerance: 'low' | 'medium' | 'high' = 'medium',
    context?: string
  ): Promise<any> {
    const response = await this.apiClient.post('/contracts/clauses/recommend', {
      contractType,
      jurisdiction,
      riskTolerance,
      context,
    });
    return response.data;
  }

  // ============================================================================
  // Compliance Checking
  // ============================================================================

  /**
   * Run compliance check on document
   */
  async checkCompliance(
    documentId: string,
    frameworks: ComplianceFramework[],
    jurisdiction?: string
  ): Promise<ComplianceCheck> {
    const response = await this.apiClient.post<ComplianceCheck>('/compliance/check', {
      documentId,
      frameworks,
      jurisdiction,
    });
    return response.data;
  }

  /**
   * Get compliance report
   */
  async getComplianceReport(checkId: string): Promise<ComplianceCheck> {
    const response = await this.apiClient.get<ComplianceCheck>(`/compliance/checks/${checkId}`);
    return response.data;
  }

  /**
   * Validate contract against playbook
   */
  async validatePlaybook(contractId: string, playbookId: string): Promise<any> {
    const response = await this.apiClient.post('/compliance/playbook/validate', {
      contractId,
      playbookId,
    });
    return response.data;
  }

  // ============================================================================
  // E-Discovery
  // ============================================================================

  /**
   * Create e-discovery review project
   */
  async createReviewProject(
    name: string,
    caseId: string,
    reviewers: string[],
    settings?: any
  ): Promise<{ projectId: string; status: string }> {
    const response = await this.apiClient.post('/ediscovery/projects', {
      name,
      caseId,
      reviewers,
      settings: settings || {
        aiAssisted: true,
        predictiveCoding: true,
        duplicateDetection: true,
        threadingEnabled: true,
      },
    });
    return response.data;
  }

  /**
   * Upload documents for review
   */
  async uploadDocumentsForReview(
    projectId: string,
    files: string[],
    custodian?: string
  ): Promise<{ uploadId: string; documentsQueued: number }> {
    const formData = new FormData();

    files.forEach((filePath) => {
      formData.append('files', fs.createReadStream(filePath));
    });

    if (custodian) {
      formData.append('custodian', custodian);
    }

    const response = await this.apiClient.post(
      `/ediscovery/projects/${projectId}/documents`,
      formData,
      {
        headers: formData.getHeaders(),
      }
    );

    return response.data;
  }

  /**
   * Search documents in review project
   */
  async searchDocuments(
    projectId: string,
    searchRequest: SearchRequest
  ): Promise<PaginatedResponse<DocumentSearchResult>> {
    const response = await this.apiClient.post<PaginatedResponse<DocumentSearchResult>>(
      `/ediscovery/projects/${projectId}/search`,
      searchRequest
    );
    return response.data;
  }

  /**
   * Get AI analysis for document
   */
  async getDocumentAIAnalysis(projectId: string, documentId: string): Promise<any> {
    const response = await this.apiClient.get(
      `/ediscovery/projects/${projectId}/documents/${documentId}/ai-analysis`
    );
    return response.data;
  }

  /**
   * Train predictive coding model
   */
  async trainPredictiveCoding(
    projectId: string,
    trainingSet: Array<{ documentId: string; coding: any }>,
    modelType: 'classification' | 'ranking' = 'classification'
  ): Promise<{ trainingId: string; status: string }> {
    const response = await this.apiClient.post(
      `/ediscovery/projects/${projectId}/predictive-coding/train`,
      {
        trainingSet,
        modelType,
      }
    );
    return response.data;
  }

  /**
   * Submit document review
   */
  async submitDocumentReview(
    projectId: string,
    documentId: string,
    review: Partial<DocumentReview>
  ): Promise<DocumentReview> {
    const response = await this.apiClient.post<DocumentReview>(
      `/ediscovery/projects/${projectId}/documents/${documentId}/review`,
      review
    );
    return response.data;
  }

  // ============================================================================
  // Document Automation
  // ============================================================================

  /**
   * Generate document from template
   */
  async generateDocument(request: GenerateDocumentRequest): Promise<GeneratedDocument> {
    const response = await this.apiClient.post<GeneratedDocument>('/documents/generate', request);
    return response.data;
  }

  /**
   * Create document template
   */
  async createTemplate(template: Partial<Template>): Promise<Template> {
    const response = await this.apiClient.post<Template>('/documents/templates', template);
    return response.data;
  }

  /**
   * Get template by ID
   */
  async getTemplate(templateId: string): Promise<Template> {
    const response = await this.apiClient.get<Template>(`/documents/templates/${templateId}`);
    return response.data;
  }

  /**
   * List available templates
   */
  async listTemplates(category?: string): Promise<Template[]> {
    const params = category ? { category } : {};
    const response = await this.apiClient.get<Template[]>('/documents/templates', { params });
    return response.data;
  }

  /**
   * Generate redline comparison
   */
  async generateRedline(
    originalDocumentId: string,
    revisedDocumentId: string,
    outputFormat: 'pdf' | 'docx' | 'html' = 'pdf'
  ): Promise<any> {
    const response = await this.apiClient.post('/documents/redline', {
      originalDocumentId,
      revisedDocumentId,
      outputFormat,
    });
    return response.data;
  }

  // ============================================================================
  // Legal Research
  // ============================================================================

  /**
   * Search case law
   */
  async searchCaseLaw(
    query: string,
    filters?: ResearchFilters,
    limit: number = 50
  ): Promise<ResearchResult> {
    const response = await this.apiClient.post<ResearchResult>('/research/cases/search', {
      query,
      filters,
      limit,
    });
    return response.data;
  }

  /**
   * Search statutes
   */
  async searchStatutes(
    query: string,
    jurisdiction?: string[],
    limit: number = 50
  ): Promise<ResearchResult> {
    const response = await this.apiClient.post<ResearchResult>('/research/statutes/search', {
      query,
      filters: { jurisdiction },
      limit,
    });
    return response.data;
  }

  /**
   * Cite check citations
   */
  async citeCheck(documentId: string, citations: string[]): Promise<any> {
    const response = await this.apiClient.post('/research/cite-check', {
      documentId,
      citations,
    });
    return response.data;
  }

  /**
   * Get shepard's signals for citations
   */
  async shepardize(citations: string[]): Promise<any> {
    const response = await this.apiClient.post('/research/shepardize', {
      citations,
    });
    return response.data;
  }

  /**
   * Find similar cases
   */
  async findSimilarCases(caseId: string, limit: number = 10): Promise<LegalResult[]> {
    const response = await this.apiClient.get<LegalResult[]>(`/research/cases/${caseId}/similar`, {
      params: { limit },
    });
    return response.data;
  }

  // ============================================================================
  // Case Management
  // ============================================================================

  /**
   * Create case
   */
  async createCase(caseData: Partial<CaseRecord>): Promise<CaseRecord> {
    const response = await this.apiClient.post<CaseRecord>('/cases', caseData);
    return response.data;
  }

  /**
   * Get case by ID
   */
  async getCase(caseId: string): Promise<CaseRecord> {
    const response = await this.apiClient.get<CaseRecord>(`/cases/${caseId}`);
    return response.data;
  }

  /**
   * Update case
   */
  async updateCase(caseId: string, updates: Partial<CaseRecord>): Promise<CaseRecord> {
    const response = await this.apiClient.patch<CaseRecord>(`/cases/${caseId}`, updates);
    return response.data;
  }

  /**
   * Add docket entry
   */
  async addDocketEntry(caseId: string, entry: any): Promise<any> {
    const response = await this.apiClient.post(`/cases/${caseId}/docket`, entry);
    return response.data;
  }

  /**
   * Add deadline
   */
  async addDeadline(caseId: string, deadline: any): Promise<any> {
    const response = await this.apiClient.post(`/cases/${caseId}/deadlines`, deadline);
    return response.data;
  }

  // ============================================================================
  // Workflow Management
  // ============================================================================

  /**
   * Create workflow
   */
  async createWorkflow(workflow: Partial<Workflow>): Promise<Workflow> {
    const response = await this.apiClient.post<Workflow>('/workflows', workflow);
    return response.data;
  }

  /**
   * Get workflow
   */
  async getWorkflow(workflowId: string): Promise<Workflow> {
    const response = await this.apiClient.get<Workflow>(`/workflows/${workflowId}`);
    return response.data;
  }

  /**
   * Start workflow instance
   */
  async startWorkflow(
    workflowId: string,
    documentId: string,
    metadata?: any,
    priority: 'low' | 'normal' | 'high' | 'urgent' = 'normal'
  ): Promise<WorkflowInstance> {
    const response = await this.apiClient.post<WorkflowInstance>(
      `/workflows/${workflowId}/instances`,
      {
        documentId,
        metadata,
        priority,
      }
    );
    return response.data;
  }

  /**
   * Get workflow instance status
   */
  async getWorkflowInstance(instanceId: string): Promise<WorkflowInstance> {
    const response = await this.apiClient.get<WorkflowInstance>(`/workflow-instances/${instanceId}`);
    return response.data;
  }

  /**
   * Complete workflow step
   */
  async completeWorkflowStep(
    instanceId: string,
    stepId: string,
    decision: 'approve' | 'reject',
    comments?: string
  ): Promise<WorkflowInstance> {
    const response = await this.apiClient.post<WorkflowInstance>(
      `/workflow-instances/${instanceId}/steps/${stepId}/complete`,
      {
        decision,
        comments,
      }
    );
    return response.data;
  }

  // ============================================================================
  // AI Legal Assistant
  // ============================================================================

  /**
   * Ask legal question with AI
   */
  async askLegalQuestion(
    question: string,
    context?: {
      jurisdiction?: string;
      practiceArea?: string;
      documentIds?: string[];
    },
    includeReferences: boolean = true
  ): Promise<any> {
    const response = await this.apiClient.post('/ai/legal-qa', {
      question,
      context,
      includeReferences,
    });
    return response.data;
  }

  /**
   * Summarize legal document
   */
  async summarizeDocument(documentId: string, summaryType: 'brief' | 'detailed' = 'brief'): Promise<any> {
    const response = await this.apiClient.post('/ai/summarize', {
      documentId,
      summaryType,
    });
    return response.data;
  }

  /**
   * Extract entities from legal text
   */
  async extractEntities(text: string): Promise<any> {
    const response = await this.apiClient.post('/ai/extract-entities', {
      text,
    });
    return response.data;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Sleep utility
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Download file from URL
   */
  async downloadFile(url: string, outputPath: string): Promise<void> {
    const response = await this.apiClient.get(url, {
      responseType: 'stream',
    });

    const writer = fs.createWriteStream(outputPath);
    response.data.pipe(writer);

    return new Promise((resolve, reject) => {
      writer.on('finish', resolve);
      writer.on('error', reject);
    });
  }

  /**
   * Get API health status
   */
  async healthCheck(): Promise<any> {
    const response = await this.apiClient.get('/health');
    return response.data;
  }

  /**
   * Get API version
   */
  async getVersion(): Promise<{ version: string; buildDate: string }> {
    const response = await this.apiClient.get('/version');
    return response.data;
  }
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';

// ============================================================================
// Default Export
// ============================================================================

export default WIALegalTechClient;

/**
 * Philosophy: 弘益人間 (Hongik Ingan - Benefit All Humanity)
 *
 * This SDK provides comprehensive legal technology capabilities:
 * - AI-powered contract analysis and risk assessment
 * - Automated compliance checking across multiple frameworks
 * - E-discovery with predictive coding
 * - Document automation and generation
 * - Legal research and citation validation
 * - Case management and docket tracking
 * - Workflow automation
 * - AI legal assistant
 *
 * Built to democratize access to advanced legal technology and
 * improve efficiency in legal practice worldwide.
 */
