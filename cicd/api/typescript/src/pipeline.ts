/**
 * Pipeline model. Mirrors the WIA-CICD pipeline.schema.json.
 */

export const PIPELINE_NODES    = ['SOURCE', 'BUILD', 'TEST_GATE', 'SECURITY_GATE', 'CD_HANDOFF'] as const;
export const PIPELINE_TRIGGERS = ['PUSH', 'PR', 'TAG', 'MANUAL', 'SCHEDULE'] as const;
export const SECURITY_GATES    = ['SAST', 'SCA', 'DAST', 'SECRETS', 'CONTAINER', 'IAC'] as const;
export const POLICY_MODES      = ['AUDIT', 'ENFORCE'] as const;
export const DEPLOY_STRATEGIES = ['ROLLING', 'BLUE_GREEN', 'CANARY', 'PROGRESSIVE'] as const;
export const TRAFFIC_CURVE     = [5, 25, 50, 100] as const;

export type PipelineNode      = typeof PIPELINE_NODES[number];
export type PipelineTrigger   = typeof PIPELINE_TRIGGERS[number];
export type SecurityGate      = typeof SECURITY_GATES[number];
export type PolicyMode        = typeof POLICY_MODES[number];
export type DeployStrategy    = typeof DEPLOY_STRATEGIES[number];

export interface PipelineSlo {
  unit_ms?:       number;
  integration_s?: number;
  e2e_s?:         number;
}

export interface PipelineCache {
  local?:     string;
  remote?:    string;
  execution?: string;
}

export interface PipelineNodeDef {
  id:    PipelineNode;
  type:  'source' | 'build' | 'test' | 'security' | 'deliver';
  needs?: PipelineNode[];
  cache?: PipelineCache;
  slo?:   PipelineSlo;
  gates?: SecurityGate[];
  policy?: PolicyMode;
  strategy?: DeployStrategy;
  traffic_curve?: number[];
}

export interface PipelineDef {
  pipeline: {
    name:    string;
    trigger: PipelineTrigger;
    nodes:   PipelineNodeDef[];
  };
}

export interface ValidationError {
  path:    string;
  message: string;
}
export interface ValidationResult {
  valid:   boolean;
  errors:  ValidationError[];
}

/** Lightweight schema check (no external dependency). */
export function validatePipeline(def: unknown): ValidationResult {
  const errors: ValidationError[] = [];
  if (!def || typeof def !== 'object' || !(def as { pipeline?: unknown }).pipeline) {
    return { valid: false, errors: [{ path: '/', message: 'missing "pipeline"' }] };
  }
  const p = (def as { pipeline: PipelineDef['pipeline'] }).pipeline;
  if (!p.name)                                      errors.push({ path: '/pipeline/name',    message: 'required' });
  if (!PIPELINE_TRIGGERS.includes(p.trigger as PipelineTrigger)) errors.push({ path: '/pipeline/trigger', message: 'invalid trigger' });
  if (!Array.isArray(p.nodes) || p.nodes.length === 0) {
    errors.push({ path: '/pipeline/nodes', message: 'at least one node required' });
  } else {
    p.nodes.forEach((n, i) => {
      if (!PIPELINE_NODES.includes(n.id as PipelineNode)) {
        errors.push({ path: `/pipeline/nodes/${i}/id`, message: `invalid id ${String(n.id)}` });
      }
      if (n.gates) {
        n.gates.forEach((g, j) => {
          if (!SECURITY_GATES.includes(g)) {
            errors.push({ path: `/pipeline/nodes/${i}/gates/${j}`, message: `invalid gate ${g}` });
          }
        });
      }
      if (n.policy   && !POLICY_MODES.includes(n.policy))           errors.push({ path: `/pipeline/nodes/${i}/policy`,   message: `invalid policy ${n.policy}` });
      if (n.strategy && !DEPLOY_STRATEGIES.includes(n.strategy))    errors.push({ path: `/pipeline/nodes/${i}/strategy`, message: `invalid strategy ${n.strategy}` });
    });
  }
  return { valid: errors.length === 0, errors };
}

/** Topological sort over node `needs:` edges. Throws on cycle. */
export function topoSort(def: PipelineDef): PipelineNode[] {
  const nodes  = def.pipeline.nodes;
  const inDeg  = new Map<PipelineNode, number>();
  const edges  = new Map<PipelineNode, PipelineNode[]>();
  nodes.forEach(n => { inDeg.set(n.id, 0); edges.set(n.id, []); });
  nodes.forEach(n => (n.needs ?? []).forEach(prev => {
    inDeg.set(n.id, (inDeg.get(n.id) ?? 0) + 1);
    edges.get(prev)?.push(n.id);
  }));
  const ready: PipelineNode[] = [];
  inDeg.forEach((d, k) => { if (d === 0) ready.push(k); });
  ready.sort();
  const order: PipelineNode[] = [];
  while (ready.length > 0) {
    const cur = ready.shift() as PipelineNode;
    order.push(cur);
    edges.get(cur)?.forEach(next => {
      const left = (inDeg.get(next) ?? 0) - 1;
      inDeg.set(next, left);
      if (left === 0) { ready.push(next); ready.sort(); }
    });
  }
  if (order.length !== nodes.length) throw new Error('cycle detected in pipeline DAG');
  return order;
}
