/**
 * WIA Access Control Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-access-control
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main Access Control Engine class
 * Provides unified interface for RBAC, ABAC, and policy-based access control
 *
 * @example
 * ```typescript
 * const acl = new WIAAccessControl({
 *   model: AccessControlModel.RBAC,
 *   evaluationStrategy: 'first_match',
 *   defaultDecision: 'deny'
 * });
 *
 * acl.addRole({
 *   id: 'admin',
 *   name: 'Administrator',
 *   permissions: [...],
 *   priority: 100,
 *   isSystem: true
 * });
 *
 * const decision = await acl.evaluate(request);
 * ```
 */
export class WIAAccessControl extends EventEmitter {
  private config: types.AccessControlConfig;
  private roles: Map<string, types.Role> = new Map();
  private policies: Map<string, types.Policy> = new Map();
  private auditLog: types.AuditLogEntry[] = [];

  /**
   * Create a new Access Control instance
   * @param config - Configuration options
   */
  constructor(config: types.AccessControlConfig) {
    super();
    this.config = config;
  }

  /**
   * Add a role to the system
   * @param role - Role definition
   */
  addRole(role: types.Role): void {
    this.roles.set(role.id, role);
    this.emit('role-added', role);
  }

  /**
   * Remove a role from the system
   * @param roleId - Role identifier
   */
  removeRole(roleId: string): boolean {
    const removed = this.roles.delete(roleId);
    if (removed) {
      this.emit('role-removed', roleId);
    }
    return removed;
  }

  /**
   * Get a role by ID
   * @param roleId - Role identifier
   */
  getRole(roleId: string): types.Role | undefined {
    return this.roles.get(roleId);
  }

  /**
   * Get all roles
   */
  getAllRoles(): types.Role[] {
    return Array.from(this.roles.values());
  }

  /**
   * Add a policy to the system
   * @param policy - Policy definition
   */
  addPolicy(policy: types.Policy): void {
    this.policies.set(policy.id, policy);
    this.emit('policy-added', policy);
  }

  /**
   * Remove a policy from the system
   * @param policyId - Policy identifier
   */
  removePolicy(policyId: string): boolean {
    const removed = this.policies.delete(policyId);
    if (removed) {
      this.emit('policy-removed', policyId);
    }
    return removed;
  }

  /**
   * Get a policy by ID
   * @param policyId - Policy identifier
   */
  getPolicy(policyId: string): types.Policy | undefined {
    return this.policies.get(policyId);
  }

  /**
   * Evaluate an access request
   * @param request - Access request to evaluate
   * @returns Access decision
   */
  async evaluate(request: types.AccessRequest): Promise<types.AccessDecision> {
    const startTime = Date.now();
    let decision: types.AccessDecision;

    try {
      if (this.config.model === types.AccessControlModel.RBAC) {
        decision = await this.evaluateRBAC(request);
      } else if (this.config.model === types.AccessControlModel.ABAC) {
        decision = await this.evaluateABAC(request);
      } else {
        decision = await this.evaluatePolicies(request);
      }
    } catch (error) {
      decision = {
        requestId: request.requestId,
        decision: 'deny',
        reason: `Evaluation error: ${error}`,
        matchingPolicies: [],
        timestamp: new Date()
      };
    }

    // Audit logging
    if (this.config.auditEnabled) {
      const logEntry: types.AuditLogEntry = {
        id: `audit-${Date.now()}`,
        request,
        decision,
        timestamp: new Date(),
        duration: Date.now() - startTime
      };
      this.auditLog.push(logEntry);
      this.emit('access-evaluated', logEntry);
    }

    return decision;
  }

  /**
   * Evaluate using RBAC model
   * @param request - Access request
   */
  private async evaluateRBAC(request: types.AccessRequest): Promise<types.AccessDecision> {
    const matchingPolicies: string[] = [];

    for (const roleId of request.subject.roles) {
      const role = this.roles.get(roleId);
      if (!role) continue;

      for (const permission of role.permissions) {
        if (this.matchesPermission(permission, request)) {
          matchingPolicies.push(`role:${roleId}:${permission.id}`);
          if (permission.effect === 'deny') {
            return {
              requestId: request.requestId,
              decision: 'deny',
              reason: `Denied by permission ${permission.id} in role ${roleId}`,
              matchingPolicies,
              timestamp: new Date()
            };
          }
        }
      }
    }

    if (matchingPolicies.length > 0) {
      return {
        requestId: request.requestId,
        decision: 'allow',
        reason: 'Allowed by role permissions',
        matchingPolicies,
        timestamp: new Date()
      };
    }

    return {
      requestId: request.requestId,
      decision: this.config.defaultDecision,
      reason: 'No matching permissions found',
      matchingPolicies: [],
      timestamp: new Date()
    };
  }

  /**
   * Evaluate using ABAC model
   * @param request - Access request
   */
  private async evaluateABAC(request: types.AccessRequest): Promise<types.AccessDecision> {
    return this.evaluatePolicies(request);
  }

  /**
   * Evaluate policies
   * @param request - Access request
   */
  private async evaluatePolicies(request: types.AccessRequest): Promise<types.AccessDecision> {
    const applicablePolicies: types.Policy[] = [];

    for (const policy of this.policies.values()) {
      if (!policy.enabled) continue;
      if (this.policyApplies(policy, request)) {
        applicablePolicies.push(policy);
      }
    }

    if (applicablePolicies.length === 0) {
      return {
        requestId: request.requestId,
        decision: this.config.defaultDecision,
        reason: 'No applicable policies',
        matchingPolicies: [],
        timestamp: new Date()
      };
    }

    // Sort by priority
    applicablePolicies.sort((a, b) => b.priority - a.priority);

    const matchingPolicies = applicablePolicies.map(p => p.id);

    if (this.config.evaluationStrategy === 'first_match') {
      const firstPolicy = applicablePolicies[0];
      return {
        requestId: request.requestId,
        decision: firstPolicy.effect,
        reason: `Matched policy: ${firstPolicy.name}`,
        matchingPolicies: [firstPolicy.id],
        timestamp: new Date()
      };
    }

    // All applicable - deny takes precedence
    const hasDeny = applicablePolicies.some(p => p.effect === 'deny');
    return {
      requestId: request.requestId,
      decision: hasDeny ? 'deny' : 'allow',
      reason: hasDeny ? 'Denied by policy' : 'Allowed by policy',
      matchingPolicies,
      timestamp: new Date()
    };
  }

  /**
   * Check if a permission matches the request
   * @param permission - Permission to check
   * @param request - Access request
   */
  private matchesPermission(
    permission: types.Permission,
    request: types.AccessRequest
  ): boolean {
    // Check action
    if (!permission.actions.includes(request.action)) {
      return false;
    }

    // Check resource pattern
    if (!this.matchesPattern(permission.resource, request.resource.path)) {
      return false;
    }

    // Check resource type
    if (permission.resourceType !== request.resource.type) {
      return false;
    }

    // Check conditions
    if (permission.conditions && permission.conditions.length > 0) {
      return this.evaluateConditions(permission.conditions, request);
    }

    return true;
  }

  /**
   * Check if a policy applies to the request
   * @param policy - Policy to check
   * @param request - Access request
   */
  private policyApplies(policy: types.Policy, request: types.AccessRequest): boolean {
    // Check if action matches
    if (!policy.actions.includes(request.action)) {
      return false;
    }

    // Check subjects
    const subjectMatches = policy.subjects.some(s => {
      if (s.type === 'any') return true;
      if (s.type === 'user') return this.matchesPattern(s.pattern, request.subject.id);
      if (s.type === 'role') return request.subject.roles.some(r => this.matchesPattern(s.pattern, r));
      return false;
    });

    if (!subjectMatches) return false;

    // Check resources
    const resourceMatches = policy.resources.some(r => {
      if (r.type !== request.resource.type) return false;
      return this.matchesPattern(r.pattern, request.resource.path);
    });

    if (!resourceMatches) return false;

    // Check conditions
    if (policy.conditions && policy.conditions.length > 0) {
      return this.evaluateConditions(policy.conditions, request);
    }

    return true;
  }

  /**
   * Match a pattern against a value
   * @param pattern - Glob-style pattern
   * @param value - Value to match
   */
  private matchesPattern(pattern: string, value: string): boolean {
    const regex = new RegExp(
      '^' + pattern.replace(/\*/g, '.*').replace(/\?/g, '.') + '$'
    );
    return regex.test(value);
  }

  /**
   * Evaluate conditions against the request
   * @param conditions - Conditions to evaluate
   * @param request - Access request
   */
  private evaluateConditions(
    conditions: types.Condition[],
    request: types.AccessRequest
  ): boolean {
    return conditions.every(condition => {
      const value = this.getFieldValue(condition.field, request);
      return this.evaluateCondition(condition, value);
    });
  }

  /**
   * Get a field value from the request
   * @param field - Field path (e.g., "context.ipAddress")
   * @param request - Access request
   */
  private getFieldValue(field: string, request: types.AccessRequest): unknown {
    const parts = field.split('.');
    let value: unknown = request;

    for (const part of parts) {
      if (value && typeof value === 'object') {
        value = (value as Record<string, unknown>)[part];
      } else {
        return undefined;
      }
    }

    return value;
  }

  /**
   * Evaluate a single condition
   * @param condition - Condition to evaluate
   * @param value - Actual value
   */
  private evaluateCondition(condition: types.Condition, value: unknown): boolean {
    switch (condition.operator) {
      case types.ConditionOperator.Equals:
        return value === condition.value;
      case types.ConditionOperator.NotEquals:
        return value !== condition.value;
      case types.ConditionOperator.GreaterThan:
        return (value as number) > (condition.value as number);
      case types.ConditionOperator.LessThan:
        return (value as number) < (condition.value as number);
      case types.ConditionOperator.Contains:
        return String(value).includes(String(condition.value));
      case types.ConditionOperator.In:
        return (condition.value as unknown[]).includes(value);
      case types.ConditionOperator.Exists:
        return value !== undefined && value !== null;
      case types.ConditionOperator.NotExists:
        return value === undefined || value === null;
      default:
        return false;
    }
  }

  /**
   * Get audit log entries
   * @param limit - Maximum entries to return
   */
  getAuditLog(limit?: number): types.AuditLogEntry[] {
    const entries = this.auditLog.slice().reverse();
    return limit ? entries.slice(0, limit) : entries;
  }

  /**
   * Clear audit log
   */
  clearAuditLog(): void {
    this.auditLog = [];
  }

  /**
   * Check WIA compliance
   * @param targetLevel - Target certification level
   */
  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    // Test 1: Configuration validation
    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.model !== undefined && this.config.evaluationStrategy !== undefined,
      notes: 'Access control model and evaluation strategy must be defined'
    });

    // Test 2: Has at least one role or policy
    tests.push({
      testName: 'Policy/Role Existence',
      passed: this.roles.size > 0 || this.policies.size > 0,
      notes: 'System must have at least one role or policy defined'
    });

    // Test 3: Audit logging enabled (for Silver+)
    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Audit Logging',
        passed: this.config.auditEnabled === true,
        notes: 'Audit logging must be enabled for Silver and Gold certification'
      });
    }

    // Test 4: Default deny policy (for Gold)
    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'Default Deny Policy',
        passed: this.config.defaultDecision === 'deny',
        notes: 'Gold certification requires default deny policy'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-ACCESS-CONTROL',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

/**
 * Create a permission builder for fluent API
 *
 * @example
 * ```typescript
 * const permission = createPermission('read-files')
 *   .actions(['read', 'list'])
 *   .resource('/files/*')
 *   .resourceType('file')
 *   .allow()
 *   .build();
 * ```
 */
export function createPermission(id: string): PermissionBuilder {
  return new PermissionBuilder(id);
}

/**
 * Permission builder for fluent API
 */
export class PermissionBuilder {
  private permission: Partial<types.Permission>;

  constructor(id: string) {
    this.permission = { id, actions: [], effect: 'allow' };
  }

  name(name: string): this {
    this.permission.name = name;
    return this;
  }

  actions(actions: types.PermissionAction[]): this {
    this.permission.actions = actions;
    return this;
  }

  resource(pattern: string): this {
    this.permission.resource = pattern;
    return this;
  }

  resourceType(type: types.ResourceType): this {
    this.permission.resourceType = type;
    return this;
  }

  allow(): this {
    this.permission.effect = 'allow';
    return this;
  }

  deny(): this {
    this.permission.effect = 'deny';
    return this;
  }

  condition(field: string, operator: types.ConditionOperator, value: unknown): this {
    if (!this.permission.conditions) {
      this.permission.conditions = [];
    }
    this.permission.conditions.push({ field, operator, value });
    return this;
  }

  build(): types.Permission {
    return this.permission as types.Permission;
  }
}

/**
 * Create an access request builder
 */
export function createAccessRequest(requestId: string): AccessRequestBuilder {
  return new AccessRequestBuilder(requestId);
}

/**
 * Access request builder
 */
export class AccessRequestBuilder {
  private request: Partial<types.AccessRequest>;

  constructor(requestId: string) {
    this.request = { requestId, timestamp: new Date() };
  }

  subject(subject: types.Subject): this {
    this.request.subject = subject;
    return this;
  }

  action(action: types.PermissionAction): this {
    this.request.action = action;
    return this;
  }

  resource(resource: types.Resource): this {
    this.request.resource = resource;
    return this;
  }

  context(context: types.RequestContext): this {
    this.request.context = context;
    return this;
  }

  build(): types.AccessRequest {
    return this.request as types.AccessRequest;
  }
}

/**
 * Default export for convenience
 */
export default {
  WIAAccessControl,
  createPermission,
  createAccessRequest,
  PermissionBuilder,
  AccessRequestBuilder
};
