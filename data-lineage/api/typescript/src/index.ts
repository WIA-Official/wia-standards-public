/**
 * WIA Data Lineage Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADataLineage,
  LineageResponse,
  LineageNode,
  LineageEdge,
  Transformation,
  ImpactAnalysis,
  LineageQuery,
  LineageEvent,
  ValidationResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// WIA Data Lineage Client
// ============================================================================

export class WIADataLineageClient {
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

  // ========================================================================
  // Lineage Management
  // ========================================================================

  /**
   * Create a new data lineage graph
   */
  async createLineage(lineage: WIADataLineage): Promise<LineageResponse> {
    const response = await this.axios.post<LineageResponse>('/lineages', lineage);
    return response.data;
  }

  /**
   * Get lineage graph by ID
   */
  async getLineage(id: string): Promise<WIADataLineage> {
    const response = await this.axios.get<WIADataLineage>(`/lineages/${id}`);
    return response.data;
  }

  /**
   * List all lineage graphs
   */
  async listLineages(params?: {
    owner?: string;
    domain?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<LineageResponse>> {
    const response = await this.axios.get<PaginatedResponse<LineageResponse>>('/lineages', {
      params,
    });
    return response.data;
  }

  /**
   * Update existing lineage graph
   */
  async updateLineage(id: string, updates: Partial<WIADataLineage>): Promise<LineageResponse> {
    const response = await this.axios.put<LineageResponse>(`/lineages/${id}`, updates);
    return response.data;
  }

  /**
   * Delete lineage graph
   */
  async deleteLineage(id: string): Promise<void> {
    await this.axios.delete(`/lineages/${id}`);
  }

  // ========================================================================
  // Node Management
  // ========================================================================

  /**
   * Add a node to lineage graph
   */
  async addNode(lineageId: string, node: LineageNode): Promise<LineageNode> {
    const response = await this.axios.post<LineageNode>(
      `/lineages/${lineageId}/nodes`,
      node
    );
    return response.data;
  }

  /**
   * Get node by ID
   */
  async getNode(lineageId: string, nodeId: string): Promise<LineageNode> {
    const response = await this.axios.get<LineageNode>(
      `/lineages/${lineageId}/nodes/${nodeId}`
    );
    return response.data;
  }

  /**
   * List all nodes in lineage graph
   */
  async listNodes(
    lineageId: string,
    params?: {
      type?: string;
      system?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<LineageNode>> {
    const response = await this.axios.get<PaginatedResponse<LineageNode>>(
      `/lineages/${lineageId}/nodes`,
      { params }
    );
    return response.data;
  }

  /**
   * Update node
   */
  async updateNode(
    lineageId: string,
    nodeId: string,
    updates: Partial<LineageNode>
  ): Promise<LineageNode> {
    const response = await this.axios.put<LineageNode>(
      `/lineages/${lineageId}/nodes/${nodeId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete node
   */
  async deleteNode(lineageId: string, nodeId: string): Promise<void> {
    await this.axios.delete(`/lineages/${lineageId}/nodes/${nodeId}`);
  }

  // ========================================================================
  // Edge Management
  // ========================================================================

  /**
   * Add an edge to lineage graph
   */
  async addEdge(lineageId: string, edge: LineageEdge): Promise<LineageEdge> {
    const response = await this.axios.post<LineageEdge>(
      `/lineages/${lineageId}/edges`,
      edge
    );
    return response.data;
  }

  /**
   * Get edge by ID
   */
  async getEdge(lineageId: string, edgeId: string): Promise<LineageEdge> {
    const response = await this.axios.get<LineageEdge>(
      `/lineages/${lineageId}/edges/${edgeId}`
    );
    return response.data;
  }

  /**
   * List all edges in lineage graph
   */
  async listEdges(
    lineageId: string,
    params?: {
      type?: string;
      sourceId?: string;
      targetId?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<LineageEdge>> {
    const response = await this.axios.get<PaginatedResponse<LineageEdge>>(
      `/lineages/${lineageId}/edges`,
      { params }
    );
    return response.data;
  }

  /**
   * Delete edge
   */
  async deleteEdge(lineageId: string, edgeId: string): Promise<void> {
    await this.axios.delete(`/lineages/${lineageId}/edges/${edgeId}`);
  }

  // ========================================================================
  // Transformation Management
  // ========================================================================

  /**
   * Add transformation
   */
  async addTransformation(
    lineageId: string,
    transformation: Transformation
  ): Promise<Transformation> {
    const response = await this.axios.post<Transformation>(
      `/lineages/${lineageId}/transformations`,
      transformation
    );
    return response.data;
  }

  /**
   * Get transformation by ID
   */
  async getTransformation(lineageId: string, transformationId: string): Promise<Transformation> {
    const response = await this.axios.get<Transformation>(
      `/lineages/${lineageId}/transformations/${transformationId}`
    );
    return response.data;
  }

  /**
   * List all transformations
   */
  async listTransformations(
    lineageId: string,
    params?: {
      type?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<Transformation>> {
    const response = await this.axios.get<PaginatedResponse<Transformation>>(
      `/lineages/${lineageId}/transformations`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Lineage Analysis
  // ========================================================================

  /**
   * Get upstream lineage for a node
   */
  async getUpstreamLineage(
    lineageId: string,
    nodeId: string,
    depth?: number
  ): Promise<WIADataLineage> {
    const response = await this.axios.get<WIADataLineage>(
      `/lineages/${lineageId}/nodes/${nodeId}/upstream`,
      { params: { depth } }
    );
    return response.data;
  }

  /**
   * Get downstream lineage for a node
   */
  async getDownstreamLineage(
    lineageId: string,
    nodeId: string,
    depth?: number
  ): Promise<WIADataLineage> {
    const response = await this.axios.get<WIADataLineage>(
      `/lineages/${lineageId}/nodes/${nodeId}/downstream`,
      { params: { depth } }
    );
    return response.data;
  }

  /**
   * Perform impact analysis
   */
  async analyzeImpact(lineageId: string, nodeId: string): Promise<ImpactAnalysis> {
    const response = await this.axios.get<ImpactAnalysis>(
      `/lineages/${lineageId}/nodes/${nodeId}/impact`
    );
    return response.data;
  }

  /**
   * Query lineage with filters
   */
  async queryLineage(lineageId: string, query: LineageQuery): Promise<WIADataLineage> {
    const response = await this.axios.post<WIADataLineage>(
      `/lineages/${lineageId}/query`,
      query
    );
    return response.data;
  }

  /**
   * Get column-level lineage
   */
  async getColumnLineage(
    lineageId: string,
    nodeId: string,
    columnName: string
  ): Promise<WIADataLineage> {
    const response = await this.axios.get<WIADataLineage>(
      `/lineages/${lineageId}/nodes/${nodeId}/columns/${columnName}/lineage`
    );
    return response.data;
  }

  // ========================================================================
  // Events
  // ========================================================================

  /**
   * Get lineage events
   */
  async getEvents(
    lineageId: string,
    params?: {
      type?: string;
      startTime?: string;
      endTime?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<LineageEvent>> {
    const response = await this.axios.get<PaginatedResponse<LineageEvent>>(
      `/lineages/${lineageId}/events`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  /**
   * Validate lineage graph
   */
  validateLineage(lineage: WIADataLineage): ValidationResult {
    const errors: any[] = [];

    if (!lineage.standard || lineage.standard !== 'WIA-DATA-LINEAGE') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-DATA-LINEAGE"',
      });
    }

    if (!lineage.version || !/^\d+\.\d+\.\d+$/.test(lineage.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!lineage.lineage || !lineage.lineage.id) {
      errors.push({
        path: 'lineage.id',
        message: 'Lineage ID is required',
      });
    }

    if (!lineage.nodes || lineage.nodes.length === 0) {
      errors.push({
        path: 'nodes',
        message: 'At least one node is required',
      });
    }

    // Validate edge references
    if (lineage.edges) {
      const nodeIds = new Set(lineage.nodes?.map((n) => n.id) || []);
      lineage.edges.forEach((edge, index) => {
        if (!nodeIds.has(edge.source)) {
          errors.push({
            path: `edges[${index}].source`,
            message: `Source node "${edge.source}" not found`,
          });
        }
        if (!nodeIds.has(edge.target)) {
          errors.push({
            path: `edges[${index}].target`,
            message: `Target node "${edge.target}" not found`,
          });
        }
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Generate a UUID v4
 */
export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Create a minimal valid data lineage
 */
export function createMinimalLineage(name: string): WIADataLineage {
  return {
    standard: 'WIA-DATA-LINEAGE',
    version: '1.0.0',
    lineage: {
      id: generateUUID(),
      name,
      owner: 'default',
      createdAt: new Date().toISOString(),
    },
    nodes: [],
    edges: [],
    transformations: [],
  };
}

/**
 * Create a source node
 */
export function createSourceNode(
  id: string,
  name: string,
  systemName: string,
  systemType: 'database' | 'data-warehouse' | 'data-lake' | 'api' | 'file-system' = 'database'
): LineageNode {
  return {
    id,
    name,
    type: 'source',
    system: {
      name: systemName,
      type: systemType,
    },
  };
}

/**
 * Create a destination node
 */
export function createDestinationNode(
  id: string,
  name: string,
  systemName: string,
  systemType: 'database' | 'data-warehouse' | 'data-lake' | 'api' | 'file-system' = 'data-warehouse'
): LineageNode {
  return {
    id,
    name,
    type: 'destination',
    system: {
      name: systemName,
      type: systemType,
    },
  };
}

/**
 * Create an edge between nodes
 */
export function createEdge(
  source: string,
  target: string,
  type: 'data-flow' | 'schema-derivation' | 'dependency' = 'data-flow'
): LineageEdge {
  return {
    id: generateUUID(),
    source,
    target,
    type,
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIADataLineageClient,
  generateUUID,
  createMinimalLineage,
  createSourceNode,
  createDestinationNode,
  createEdge,
};
