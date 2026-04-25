/**
 * WIA Data Warehouse Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADataWarehouse,
  WarehouseResponse,
  SchemaDefinition,
  TableDefinition,
  TableResponse,
  ColumnDefinition,
  RelationshipDefinition,
  DataPolicy,
  QueryDefinition,
  QueryResult,
  ValidationResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// WIA Data Warehouse Client
// ============================================================================

export class WIADataWarehouseClient {
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
  // Warehouse Management
  // ========================================================================

  /**
   * Create a new data warehouse
   */
  async createWarehouse(warehouse: WIADataWarehouse): Promise<WarehouseResponse> {
    const response = await this.axios.post<WarehouseResponse>('/warehouses', warehouse);
    return response.data;
  }

  /**
   * Get warehouse by ID
   */
  async getWarehouse(id: string): Promise<WIADataWarehouse> {
    const response = await this.axios.get<WIADataWarehouse>(`/warehouses/${id}`);
    return response.data;
  }

  /**
   * List all warehouses
   */
  async listWarehouses(params?: {
    type?: string;
    provider?: string;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<WarehouseResponse>> {
    const response = await this.axios.get<PaginatedResponse<WarehouseResponse>>('/warehouses', {
      params,
    });
    return response.data;
  }

  /**
   * Update existing warehouse
   */
  async updateWarehouse(id: string, updates: Partial<WIADataWarehouse>): Promise<WarehouseResponse> {
    const response = await this.axios.put<WarehouseResponse>(`/warehouses/${id}`, updates);
    return response.data;
  }

  /**
   * Delete warehouse
   */
  async deleteWarehouse(id: string): Promise<void> {
    await this.axios.delete(`/warehouses/${id}`);
  }

  /**
   * Suspend warehouse
   */
  async suspendWarehouse(id: string): Promise<void> {
    await this.axios.post(`/warehouses/${id}/suspend`);
  }

  /**
   * Resume warehouse
   */
  async resumeWarehouse(id: string): Promise<void> {
    await this.axios.post(`/warehouses/${id}/resume`);
  }

  /**
   * Scale warehouse
   */
  async scaleWarehouse(id: string, size: string): Promise<void> {
    await this.axios.post(`/warehouses/${id}/scale`, { size });
  }

  // ========================================================================
  // Schema Management
  // ========================================================================

  /**
   * Create schema
   */
  async createSchema(warehouseId: string, schema: SchemaDefinition): Promise<SchemaDefinition> {
    const response = await this.axios.post<SchemaDefinition>(
      `/warehouses/${warehouseId}/schemas`,
      schema
    );
    return response.data;
  }

  /**
   * Get schema by ID
   */
  async getSchema(warehouseId: string, schemaId: string): Promise<SchemaDefinition> {
    const response = await this.axios.get<SchemaDefinition>(
      `/warehouses/${warehouseId}/schemas/${schemaId}`
    );
    return response.data;
  }

  /**
   * List schemas
   */
  async listSchemas(
    warehouseId: string,
    params?: {
      type?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<SchemaDefinition>> {
    const response = await this.axios.get<PaginatedResponse<SchemaDefinition>>(
      `/warehouses/${warehouseId}/schemas`,
      { params }
    );
    return response.data;
  }

  /**
   * Delete schema
   */
  async deleteSchema(warehouseId: string, schemaId: string, cascade?: boolean): Promise<void> {
    await this.axios.delete(`/warehouses/${warehouseId}/schemas/${schemaId}`, {
      params: { cascade },
    });
  }

  // ========================================================================
  // Table Management
  // ========================================================================

  /**
   * Create table
   */
  async createTable(warehouseId: string, table: TableDefinition): Promise<TableResponse> {
    const response = await this.axios.post<TableResponse>(
      `/warehouses/${warehouseId}/tables`,
      table
    );
    return response.data;
  }

  /**
   * Get table by ID
   */
  async getTable(warehouseId: string, tableId: string): Promise<TableDefinition> {
    const response = await this.axios.get<TableDefinition>(
      `/warehouses/${warehouseId}/tables/${tableId}`
    );
    return response.data;
  }

  /**
   * List tables
   */
  async listTables(
    warehouseId: string,
    params?: {
      schemaId?: string;
      type?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<TableResponse>> {
    const response = await this.axios.get<PaginatedResponse<TableResponse>>(
      `/warehouses/${warehouseId}/tables`,
      { params }
    );
    return response.data;
  }

  /**
   * Update table
   */
  async updateTable(
    warehouseId: string,
    tableId: string,
    updates: Partial<TableDefinition>
  ): Promise<TableResponse> {
    const response = await this.axios.put<TableResponse>(
      `/warehouses/${warehouseId}/tables/${tableId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete table
   */
  async deleteTable(warehouseId: string, tableId: string): Promise<void> {
    await this.axios.delete(`/warehouses/${warehouseId}/tables/${tableId}`);
  }

  /**
   * Get table columns
   */
  async getColumns(warehouseId: string, tableId: string): Promise<ColumnDefinition[]> {
    const response = await this.axios.get<ColumnDefinition[]>(
      `/warehouses/${warehouseId}/tables/${tableId}/columns`
    );
    return response.data;
  }

  /**
   * Add column to table
   */
  async addColumn(
    warehouseId: string,
    tableId: string,
    column: ColumnDefinition
  ): Promise<ColumnDefinition> {
    const response = await this.axios.post<ColumnDefinition>(
      `/warehouses/${warehouseId}/tables/${tableId}/columns`,
      column
    );
    return response.data;
  }

  /**
   * Get table statistics
   */
  async getTableStats(warehouseId: string, tableId: string): Promise<{
    rowCount: number;
    sizeBytes: number;
    lastModified: string;
    partitionCount?: number;
    columnStats?: Record<string, { nullCount: number; distinctCount: number; min?: unknown; max?: unknown }>;
  }> {
    const response = await this.axios.get(
      `/warehouses/${warehouseId}/tables/${tableId}/stats`
    );
    return response.data;
  }

  // ========================================================================
  // Query Execution
  // ========================================================================

  /**
   * Execute query
   */
  async executeQuery(warehouseId: string, sql: string, params?: {
    parameters?: Record<string, unknown>;
    limit?: number;
    timeout?: number;
  }): Promise<QueryResult> {
    const response = await this.axios.post<QueryResult>(
      `/warehouses/${warehouseId}/query`,
      { sql, ...params }
    );
    return response.data;
  }

  /**
   * Get query status
   */
  async getQueryStatus(warehouseId: string, queryId: string): Promise<QueryResult> {
    const response = await this.axios.get<QueryResult>(
      `/warehouses/${warehouseId}/queries/${queryId}`
    );
    return response.data;
  }

  /**
   * Cancel query
   */
  async cancelQuery(warehouseId: string, queryId: string): Promise<void> {
    await this.axios.post(`/warehouses/${warehouseId}/queries/${queryId}/cancel`);
  }

  /**
   * Get query history
   */
  async getQueryHistory(
    warehouseId: string,
    params?: {
      startTime?: string;
      endTime?: string;
      status?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<QueryResult>> {
    const response = await this.axios.get<PaginatedResponse<QueryResult>>(
      `/warehouses/${warehouseId}/queries`,
      { params }
    );
    return response.data;
  }

  /**
   * Save query
   */
  async saveQuery(warehouseId: string, query: QueryDefinition): Promise<QueryDefinition> {
    const response = await this.axios.post<QueryDefinition>(
      `/warehouses/${warehouseId}/saved-queries`,
      query
    );
    return response.data;
  }

  // ========================================================================
  // Policy Management
  // ========================================================================

  /**
   * Create policy
   */
  async createPolicy(warehouseId: string, policy: DataPolicy): Promise<DataPolicy> {
    const response = await this.axios.post<DataPolicy>(
      `/warehouses/${warehouseId}/policies`,
      policy
    );
    return response.data;
  }

  /**
   * Get policy
   */
  async getPolicy(warehouseId: string, policyId: string): Promise<DataPolicy> {
    const response = await this.axios.get<DataPolicy>(
      `/warehouses/${warehouseId}/policies/${policyId}`
    );
    return response.data;
  }

  /**
   * List policies
   */
  async listPolicies(
    warehouseId: string,
    params?: {
      type?: string;
      enabled?: boolean;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<DataPolicy>> {
    const response = await this.axios.get<PaginatedResponse<DataPolicy>>(
      `/warehouses/${warehouseId}/policies`,
      { params }
    );
    return response.data;
  }

  /**
   * Update policy
   */
  async updatePolicy(
    warehouseId: string,
    policyId: string,
    updates: Partial<DataPolicy>
  ): Promise<DataPolicy> {
    const response = await this.axios.put<DataPolicy>(
      `/warehouses/${warehouseId}/policies/${policyId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete policy
   */
  async deletePolicy(warehouseId: string, policyId: string): Promise<void> {
    await this.axios.delete(`/warehouses/${warehouseId}/policies/${policyId}`);
  }

  // ========================================================================
  // Validation
  // ========================================================================

  /**
   * Validate warehouse configuration
   */
  validateWarehouse(warehouse: WIADataWarehouse): ValidationResult {
    const errors: any[] = [];

    if (!warehouse.standard || warehouse.standard !== 'WIA-DATA-WAREHOUSE') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-DATA-WAREHOUSE"',
      });
    }

    if (!warehouse.version || !/^\d+\.\d+\.\d+$/.test(warehouse.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!warehouse.warehouse || !warehouse.warehouse.id) {
      errors.push({
        path: 'warehouse.id',
        message: 'Warehouse ID is required',
      });
    }

    if (!warehouse.schemas || warehouse.schemas.length === 0) {
      errors.push({
        path: 'schemas',
        message: 'At least one schema is required',
      });
    }

    // Validate table schema references
    if (warehouse.tables && warehouse.schemas) {
      const schemaIds = new Set(warehouse.schemas.map((s) => s.id));
      warehouse.tables.forEach((table, index) => {
        if (!schemaIds.has(table.schemaId)) {
          errors.push({
            path: `tables[${index}].schemaId`,
            message: `Schema "${table.schemaId}" not found`,
          });
        }
      });
    }

    // Validate relationships
    if (warehouse.relationships && warehouse.tables) {
      const tableIds = new Set(warehouse.tables.map((t) => t.id));
      warehouse.relationships.forEach((rel, index) => {
        if (!tableIds.has(rel.source.tableId)) {
          errors.push({
            path: `relationships[${index}].source.tableId`,
            message: `Source table "${rel.source.tableId}" not found`,
          });
        }
        if (!tableIds.has(rel.target.tableId)) {
          errors.push({
            path: `relationships[${index}].target.tableId`,
            message: `Target table "${rel.target.tableId}" not found`,
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
 * Create a minimal valid data warehouse
 */
export function createMinimalWarehouse(name: string, provider: 'snowflake' | 'bigquery' | 'redshift' = 'snowflake'): WIADataWarehouse {
  return {
    standard: 'WIA-DATA-WAREHOUSE',
    version: '1.0.0',
    warehouse: {
      id: generateUUID(),
      name,
      type: 'cloud-native',
      provider,
      region: 'us-east-1',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    schemas: [
      {
        id: generateUUID(),
        name: 'raw',
        type: 'raw',
        description: 'Raw data from source systems',
      },
      {
        id: generateUUID(),
        name: 'staging',
        type: 'staging',
        description: 'Staging area for data transformation',
      },
      {
        id: generateUUID(),
        name: 'mart',
        type: 'mart',
        description: 'Business-ready data marts',
      },
    ],
    tables: [],
    relationships: [],
    policies: [],
  };
}

/**
 * Create a table definition
 */
export function createTableDefinition(
  schemaId: string,
  name: string,
  columns: ColumnDefinition[]
): TableDefinition {
  return {
    id: generateUUID(),
    schemaId,
    name,
    type: 'table',
    columns,
    primaryKey: columns.filter((c) => c.name === 'id').map((c) => c.name),
  };
}

/**
 * Create a column definition
 */
export function createColumn(
  name: string,
  type: 'string' | 'integer' | 'bigint' | 'decimal' | 'boolean' | 'date' | 'timestamp',
  options?: {
    nullable?: boolean;
    description?: string;
    sensitivity?: 'none' | 'pii' | 'phi' | 'pci';
  }
): ColumnDefinition {
  return {
    name,
    type,
    nullable: options?.nullable ?? true,
    description: options?.description,
    sensitivity: options?.sensitivity || 'none',
  };
}

/**
 * Create common columns for dimension tables
 */
export function createDimensionColumns(idColumn: string = 'id'): ColumnDefinition[] {
  return [
    createColumn(idColumn, 'bigint', { nullable: false, description: 'Surrogate key' }),
    createColumn('created_at', 'timestamp', { nullable: false }),
    createColumn('updated_at', 'timestamp', { nullable: true }),
    createColumn('is_active', 'boolean', { nullable: false }),
  ];
}

/**
 * Create common columns for fact tables
 */
export function createFactColumns(): ColumnDefinition[] {
  return [
    createColumn('id', 'bigint', { nullable: false, description: 'Fact ID' }),
    createColumn('event_timestamp', 'timestamp', { nullable: false }),
    createColumn('loaded_at', 'timestamp', { nullable: false }),
  ];
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIADataWarehouseClient,
  generateUUID,
  createMinimalWarehouse,
  createTableDefinition,
  createColumn,
  createDimensionColumns,
  createFactColumns,
};
