/**
 * WIA Data Catalog Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataCatalogProject, ProjectResponse, ValidationResult, PaginatedResponse,
  DataAsset, Namespace, Taxonomy, TaxonomyCategory, Connector, SearchQuery, SearchResult,
  LineageGraph, QualityRule, GovernancePolicy, AccessRole, CustomAttribute, DataCatalogEntry
} from './types';

// ============================================================================
// WIA Data Catalog Client
// ============================================================================

export class WIADataCatalogClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // Project Management
  async createProject(project: WIADataCatalogProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataCatalogProject> {
    return (await this.axios.get<WIADataCatalogProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; domain?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataCatalogProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Namespace Management
  async listNamespaces(projectId: string): Promise<Namespace[]> {
    return (await this.axios.get<Namespace[]>(`/projects/${projectId}/namespaces`)).data;
  }

  async createNamespace(projectId: string, namespace: Partial<Namespace>): Promise<Namespace> {
    return (await this.axios.post<Namespace>(`/projects/${projectId}/namespaces`, namespace)).data;
  }

  async updateNamespace(projectId: string, namespaceId: string, updates: Partial<Namespace>): Promise<Namespace> {
    return (await this.axios.put<Namespace>(`/projects/${projectId}/namespaces/${namespaceId}`, updates)).data;
  }

  async deleteNamespace(projectId: string, namespaceId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/namespaces/${namespaceId}`);
  }

  // Taxonomy Management
  async listTaxonomies(projectId: string): Promise<Taxonomy[]> {
    return (await this.axios.get<Taxonomy[]>(`/projects/${projectId}/taxonomies`)).data;
  }

  async createTaxonomy(projectId: string, taxonomy: Partial<Taxonomy>): Promise<Taxonomy> {
    return (await this.axios.post<Taxonomy>(`/projects/${projectId}/taxonomies`, taxonomy)).data;
  }

  async getTaxonomy(projectId: string, taxonomyId: string): Promise<Taxonomy> {
    return (await this.axios.get<Taxonomy>(`/projects/${projectId}/taxonomies/${taxonomyId}`)).data;
  }

  async addCategory(projectId: string, taxonomyId: string, category: Partial<TaxonomyCategory>): Promise<TaxonomyCategory> {
    return (await this.axios.post<TaxonomyCategory>(`/projects/${projectId}/taxonomies/${taxonomyId}/categories`, category)).data;
  }

  // Asset Management
  async listAssets(projectId: string, params?: { type?: string; namespace?: string; tags?: string[]; limit?: number }): Promise<PaginatedResponse<DataAsset>> {
    return (await this.axios.get<PaginatedResponse<DataAsset>>(`/projects/${projectId}/assets`, { params })).data;
  }

  async getAsset(projectId: string, assetId: string): Promise<DataAsset> {
    return (await this.axios.get<DataAsset>(`/projects/${projectId}/assets/${assetId}`)).data;
  }

  async getAssetByQualifiedName(projectId: string, qualifiedName: string): Promise<DataAsset> {
    return (await this.axios.get<DataAsset>(`/projects/${projectId}/assets/by-name/${encodeURIComponent(qualifiedName)}`)).data;
  }

  async createAsset(projectId: string, asset: Partial<DataAsset>): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets`, asset)).data;
  }

  async updateAsset(projectId: string, assetId: string, updates: Partial<DataAsset>): Promise<DataAsset> {
    return (await this.axios.put<DataAsset>(`/projects/${projectId}/assets/${assetId}`, updates)).data;
  }

  async deleteAsset(projectId: string, assetId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/assets/${assetId}`);
  }

  async addAssetTag(projectId: string, assetId: string, tag: string): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets/${assetId}/tags`, { tag })).data;
  }

  async removeAssetTag(projectId: string, assetId: string, tag: string): Promise<DataAsset> {
    return (await this.axios.delete<DataAsset>(`/projects/${projectId}/assets/${assetId}/tags/${tag}`)).data;
  }

  async classifyAsset(projectId: string, assetId: string, classifications: string[]): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets/${assetId}/classify`, { classifications })).data;
  }

  // Search & Discovery
  async search(projectId: string, query: SearchQuery): Promise<SearchResult> {
    return (await this.axios.post<SearchResult>(`/projects/${projectId}/search`, query)).data;
  }

  async suggest(projectId: string, prefix: string, limit?: number): Promise<string[]> {
    return (await this.axios.get<string[]>(`/projects/${projectId}/search/suggest`, { params: { prefix, limit } })).data;
  }

  async getPopularAssets(projectId: string, limit?: number): Promise<DataAsset[]> {
    return (await this.axios.get<DataAsset[]>(`/projects/${projectId}/assets/popular`, { params: { limit } })).data;
  }

  async getRecentAssets(projectId: string, limit?: number): Promise<DataAsset[]> {
    return (await this.axios.get<DataAsset[]>(`/projects/${projectId}/assets/recent`, { params: { limit } })).data;
  }

  // Connector Management
  async listConnectors(projectId: string): Promise<Connector[]> {
    return (await this.axios.get<Connector[]>(`/projects/${projectId}/connectors`)).data;
  }

  async createConnector(projectId: string, connector: Partial<Connector>): Promise<Connector> {
    return (await this.axios.post<Connector>(`/projects/${projectId}/connectors`, connector)).data;
  }

  async updateConnector(projectId: string, connectorId: string, updates: Partial<Connector>): Promise<Connector> {
    return (await this.axios.put<Connector>(`/projects/${projectId}/connectors/${connectorId}`, updates)).data;
  }

  async testConnector(projectId: string, connectorId: string): Promise<{ success: boolean; message?: string }> {
    return (await this.axios.post<{ success: boolean; message?: string }>(`/projects/${projectId}/connectors/${connectorId}/test`)).data;
  }

  async triggerScan(projectId: string, connectorId: string, scanType: 'full' | 'incremental'): Promise<{ scanId: string; status: string }> {
    return (await this.axios.post<{ scanId: string; status: string }>(`/projects/${projectId}/connectors/${connectorId}/scan`, { type: scanType })).data;
  }

  async getScanStatus(projectId: string, scanId: string): Promise<ScanStatus> {
    return (await this.axios.get<ScanStatus>(`/projects/${projectId}/scans/${scanId}`)).data;
  }

  // Lineage
  async getAssetLineage(projectId: string, assetId: string, depth?: number): Promise<LineageGraph> {
    return (await this.axios.get<LineageGraph>(`/projects/${projectId}/assets/${assetId}/lineage`, { params: { depth } })).data;
  }

  async getUpstreamAssets(projectId: string, assetId: string, depth?: number): Promise<DataAsset[]> {
    return (await this.axios.get<DataAsset[]>(`/projects/${projectId}/assets/${assetId}/lineage/upstream`, { params: { depth } })).data;
  }

  async getDownstreamAssets(projectId: string, assetId: string, depth?: number): Promise<DataAsset[]> {
    return (await this.axios.get<DataAsset[]>(`/projects/${projectId}/assets/${assetId}/lineage/downstream`, { params: { depth } })).data;
  }

  async getImpactAnalysis(projectId: string, assetId: string): Promise<ImpactAnalysis> {
    return (await this.axios.get<ImpactAnalysis>(`/projects/${projectId}/assets/${assetId}/impact`)).data;
  }

  // Data Quality
  async listQualityRules(projectId: string): Promise<QualityRule[]> {
    return (await this.axios.get<QualityRule[]>(`/projects/${projectId}/quality/rules`)).data;
  }

  async createQualityRule(projectId: string, rule: Partial<QualityRule>): Promise<QualityRule> {
    return (await this.axios.post<QualityRule>(`/projects/${projectId}/quality/rules`, rule)).data;
  }

  async runQualityCheck(projectId: string, assetId: string, ruleIds?: string[]): Promise<QualityCheckResult> {
    return (await this.axios.post<QualityCheckResult>(`/projects/${projectId}/assets/${assetId}/quality/check`, { ruleIds })).data;
  }

  async getQualityScore(projectId: string, assetId: string): Promise<QualityScoreDetail> {
    return (await this.axios.get<QualityScoreDetail>(`/projects/${projectId}/assets/${assetId}/quality/score`)).data;
  }

  async getQualityTrend(projectId: string, assetId: string, period: string): Promise<QualityTrend[]> {
    return (await this.axios.get<QualityTrend[]>(`/projects/${projectId}/assets/${assetId}/quality/trend`, { params: { period } })).data;
  }

  // Governance
  async listPolicies(projectId: string): Promise<GovernancePolicy[]> {
    return (await this.axios.get<GovernancePolicy[]>(`/projects/${projectId}/governance/policies`)).data;
  }

  async createPolicy(projectId: string, policy: Partial<GovernancePolicy>): Promise<GovernancePolicy> {
    return (await this.axios.post<GovernancePolicy>(`/projects/${projectId}/governance/policies`, policy)).data;
  }

  async listAccessRoles(projectId: string): Promise<AccessRole[]> {
    return (await this.axios.get<AccessRole[]>(`/projects/${projectId}/governance/roles`)).data;
  }

  async createAccessRole(projectId: string, role: Partial<AccessRole>): Promise<AccessRole> {
    return (await this.axios.post<AccessRole>(`/projects/${projectId}/governance/roles`, role)).data;
  }

  async assignRole(projectId: string, userId: string, roleId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/governance/assignments`, { userId, roleId });
  }

  // Custom Attributes
  async listCustomAttributes(projectId: string): Promise<CustomAttribute[]> {
    return (await this.axios.get<CustomAttribute[]>(`/projects/${projectId}/attributes`)).data;
  }

  async createCustomAttribute(projectId: string, attribute: Partial<CustomAttribute>): Promise<CustomAttribute> {
    return (await this.axios.post<CustomAttribute>(`/projects/${projectId}/attributes`, attribute)).data;
  }

  // Collaboration
  async addComment(projectId: string, assetId: string, comment: string): Promise<Comment> {
    return (await this.axios.post<Comment>(`/projects/${projectId}/assets/${assetId}/comments`, { comment })).data;
  }

  async listComments(projectId: string, assetId: string): Promise<Comment[]> {
    return (await this.axios.get<Comment[]>(`/projects/${projectId}/assets/${assetId}/comments`)).data;
  }

  async rateAsset(projectId: string, assetId: string, rating: number): Promise<void> {
    await this.axios.post(`/projects/${projectId}/assets/${assetId}/rating`, { rating });
  }

  async bookmarkAsset(projectId: string, assetId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/assets/${assetId}/bookmark`);
  }

  // Validation
  validateProject(project: WIADataCatalogProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-CATALOG') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-CATALOG"' });
    }
    if (!project.version) {
      errors.push({ path: 'version', message: 'Version is required' });
    }
    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }
    if (!project.metadata?.name) {
      errors.push({ path: 'metadata.name', message: 'Project name is required' });
    }
    if (!project.catalog) {
      errors.push({ path: 'catalog', message: 'Catalog configuration is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface ScanStatus {
  id: string;
  status: 'pending' | 'running' | 'completed' | 'failed';
  startedAt?: string;
  completedAt?: string;
  assetsDiscovered: number;
  assetsUpdated: number;
  errors?: string[];
}

export interface ImpactAnalysis {
  asset: DataAsset;
  impactedAssets: { asset: DataAsset; impactLevel: 'direct' | 'indirect'; distance: number }[];
  impactedPipelines: string[];
  impactedReports: string[];
}

export interface QualityCheckResult {
  assetId: string;
  timestamp: string;
  passed: number;
  failed: number;
  results: { ruleId: string; passed: boolean; value: number; threshold: number }[];
}

export interface QualityScoreDetail {
  overall: number;
  dimensions: { name: string; score: number; weight: number }[];
  trend: 'up' | 'down' | 'stable';
  lastChecked: string;
}

export interface QualityTrend {
  timestamp: string;
  score: number;
  issues: number;
}

export interface Comment {
  id: string;
  author: string;
  content: string;
  createdAt: string;
  updatedAt?: string;
}

export interface DataCatalogEntry {
  id: string;
  qualifiedName: string;
  type: string;
  name: string;
  description?: string;
  owner: string;
  tags: string[];
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProject(name: string, organization: string): WIADataCatalogProject {
  return {
    standard: 'WIA-DATA-CATALOG',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, contact: { name: '', email: '', role: 'Data Catalog Admin' } },
      domain: 'enterprise',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    catalog: {
      name,
      namespaces: [],
      taxonomies: [],
      customAttributes: [],
      searchConfig: { fullText: true, fuzzyMatching: true, synonyms: [], boostFields: [], facets: ['type', 'owner', 'tags'] },
    },
    assets: {
      assetTypes: [],
      schemas: { enabled: true, format: 'auto', compatibility: 'backward', evolution: true },
      relationships: { types: [], autoDetection: true, inference: { enabled: true, methods: ['schema', 'naming'], confidence: 0.8 } },
      versioning: { enabled: true, retentionPolicy: '90 days', diffTracking: true },
    },
    discovery: {
      connectors: [],
      scanSchedule: { fullScan: '0 0 * * 0', incrementalScan: '0 */4 * * *', metadataRefresh: '0 * * * *', statisticsRefresh: '0 0 * * *' },
      profiling: { enabled: true, sampleSize: 10000, columns: 'all', metrics: ['cardinality', 'nulls', 'distribution'] },
      classification: { enabled: true, rules: [], patterns: [], mlEnabled: false },
    },
    lineage: { enabled: true, sources: [], granularity: 'column', retention: '1 year', visualization: { layout: 'dagre', depth: 3, expandable: true, impactAnalysis: true } },
    quality: { dimensions: [], rules: [], monitoring: { enabled: true, schedule: '0 0 * * *', trending: true, anomalyDetection: true }, alerts: [] },
    governance: {
      policies: [],
      accessControl: { model: 'rbac', roles: [], permissions: [] },
      dataPrivacy: { piiFields: [], masking: [], encryption: [] },
      compliance: { frameworks: [], requirements: [], auditing: { enabled: true, events: ['read', 'write', 'schema-change'], retention: '7 years' } },
    },
    collaboration: { comments: true, ratings: true, bookmarks: true, wikis: false, notifications: { channels: ['email'], events: ['asset-update', 'quality-alert'], subscriptions: true } },
  };
}

export default { WIADataCatalogClient, generateUUID, createMinimalProject };
