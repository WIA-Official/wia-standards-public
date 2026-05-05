/**
 * Unit tests for @wia/cicd-sdk. Run with `npm test` (vitest).
 */
import { describe, it, expect } from 'vitest';
import {
  PIPELINE_NODES, PIPELINE_TRIGGERS, SECURITY_GATES, DEPLOY_STRATEGIES,
  validatePipeline, topoSort, type PipelineDef
} from '../pipeline';
import { emitCycloneDx, emitSpdxTagValue, meetsSlsaTarget, SBOM_FORMATS } from '../sbom';
import { evaluateDora, composeCacheHit, errorBudgetBurn, DORA_ELITE } from '../dora';

const sample: PipelineDef = {
  pipeline: {
    name: 'sample', trigger: 'PUSH',
    nodes: [
      { id: 'SOURCE',        type: 'source' },
      { id: 'BUILD',         type: 'build',    needs: ['SOURCE'] },
      { id: 'TEST_GATE',     type: 'test',     needs: ['BUILD'] },
      { id: 'SECURITY_GATE', type: 'security', needs: ['TEST_GATE'], gates: ['SAST', 'SCA'], policy: 'ENFORCE' },
      { id: 'CD_HANDOFF',    type: 'deliver',  needs: ['SECURITY_GATE'], strategy: 'CANARY', traffic_curve: [5, 25, 50, 100] }
    ]
  }
};

describe('pipeline ENUMs', () => {
  it('has the canonical 5 nodes', () => { expect(PIPELINE_NODES).toEqual(['SOURCE','BUILD','TEST_GATE','SECURITY_GATE','CD_HANDOFF']); });
  it('has 5 triggers',           () => { expect(PIPELINE_TRIGGERS).toHaveLength(5); });
  it('has 6 security gates',     () => { expect(SECURITY_GATES).toHaveLength(6); });
  it('has 4 deploy strategies',  () => { expect(DEPLOY_STRATEGIES).toHaveLength(4); });
});

describe('validatePipeline', () => {
  it('accepts the canonical pipeline',     () => { expect(validatePipeline(sample).valid).toBe(true); });
  it('rejects unknown trigger', () => {
    const bad = JSON.parse(JSON.stringify(sample)); bad.pipeline.trigger = 'NONSENSE';
    expect(validatePipeline(bad).valid).toBe(false);
  });
  it('rejects unknown gate', () => {
    const bad = JSON.parse(JSON.stringify(sample)); bad.pipeline.nodes[3].gates = ['XYZ'];
    expect(validatePipeline(bad).valid).toBe(false);
  });
});

describe('topoSort', () => {
  it('orders nodes left to right', () => {
    expect(topoSort(sample)).toEqual(['SOURCE','BUILD','TEST_GATE','SECURITY_GATE','CD_HANDOFF']);
  });
  it('throws on cycle', () => {
    const bad: PipelineDef = JSON.parse(JSON.stringify(sample));
    bad.pipeline.nodes[0].needs = ['CD_HANDOFF'];
    expect(() => topoSort(bad)).toThrow();
  });
});

describe('SBOM emitters', () => {
  const meta = { serialNumber: 'urn:uuid:test', timestamp: '2026-05-05T00:00:00Z', toolVendor: 'WIA', toolName: 'cicd', toolVersion: '1.0.0' };
  it('emits CycloneDX 1.5', () => {
    const out = emitCycloneDx(meta, [{ type: 'library', name: 'lib', version: '1.0.0', purl: 'pkg:npm/lib@1.0.0' }]) as { specVersion: string; bomFormat: string };
    expect(out.bomFormat).toBe('CycloneDX'); expect(out.specVersion).toBe('1.5');
  });
  it('emits SPDX tag-value', () => {
    const out = emitSpdxTagValue(meta, [{ type: 'library', name: 'lib', version: '1.0.0' }]);
    expect(out).toContain('SPDXVersion: SPDX-2.3');
    expect(out).toContain('PackageName: lib');
  });
  it('SBOM format ENUM has 3 entries', () => { expect(SBOM_FORMATS).toHaveLength(3); });
  it('SLSA L3 + signed satisfies target', () => { expect(meetsSlsaTarget('L3', true)).toBe(true); expect(meetsSlsaTarget('L2', true)).toBe(false); });
});

describe('DORA evaluator', () => {
  it('passes when within Elite thresholds', () => {
    const v = evaluateDora({ lead_time_hr: 0.5, deploy_freq_per_day: 4, failure_rate: 0.03, mttr_min: 4, rework_rate: 0.04 });
    expect(v.elite).toBe(true); expect(v.score).toBe(1);
  });
  it('fails when failure_rate exceeds 0.05', () => {
    const v = evaluateDora({ lead_time_hr: 0.5, deploy_freq_per_day: 4, failure_rate: 0.10, mttr_min: 4, rework_rate: 0.04 });
    expect(v.elite).toBe(false); expect(v.failures).toContain('failure_rate');
  });
  it('exposes Elite constants', () => {
    expect(DORA_ELITE.LEAD_TIME_HR).toBe(1);
    expect(DORA_ELITE.FAILURE_RATE).toBe(0.05);
    expect(DORA_ELITE.REWORK_RATE).toBe(0.05);
  });
});

describe('composeCacheHit + errorBudgetBurn', () => {
  it('combines tiers monotonically', () => {
    expect(composeCacheHit(0, 0, 0)).toBe(0);
    expect(composeCacheHit(1, 0, 0)).toBe(1);
    expect(composeCacheHit(0.8, 0.6, 0.4)).toBeCloseTo(1 - 0.2 * 0.4 * 0.6, 6);
  });
  it('error budget burn matches simulator', () => {
    // STD SLO 0.999 → budget 0.001. err 0.0002 → 0.2 burn.
    expect(errorBudgetBurn(0.0002, 0.999)).toBeCloseTo(0.2, 6);
  });
});
