/**
 * WIA CI Standard - SDK Implementation
 * @packageDocumentation
 * @module wia-ci
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

export class WIACI extends EventEmitter {
  private config: types.CIConfig;
  private pipelines: Map<string, types.Pipeline> = new Map();
  private runs: Map<string, types.PipelineRun> = new Map();
  private artifacts: Map<string, types.Artifact> = new Map();
  private targets: Map<string, types.DeploymentTarget> = new Map();

  constructor(config: types.CIConfig) {
    super();
    this.config = config;
  }

  async createPipeline(pipeline: Omit<types.Pipeline, 'id' | 'createdAt' | 'updatedAt'>): Promise<types.Pipeline> {
    const newPipeline: types.Pipeline = {
      ...pipeline,
      id: `pipeline-${Date.now()}`,
      createdAt: new Date(),
      updatedAt: new Date()
    };

    this.pipelines.set(newPipeline.id, newPipeline);
    this.emit('pipeline-created', newPipeline);
    return newPipeline;
  }

  getPipeline(pipelineId: string): types.Pipeline | undefined {
    return this.pipelines.get(pipelineId);
  }

  async updatePipeline(pipelineId: string, updates: Partial<types.Pipeline>): Promise<types.Pipeline> {
    const pipeline = this.pipelines.get(pipelineId);
    if (!pipeline) throw new Error('Pipeline not found');

    const updated = { ...pipeline, ...updates, updatedAt: new Date() };
    this.pipelines.set(pipelineId, updated);
    this.emit('pipeline-updated', updated);
    return updated;
  }

  async deletePipeline(pipelineId: string): Promise<void> {
    const pipeline = this.pipelines.get(pipelineId);
    if (!pipeline) throw new Error('Pipeline not found');

    this.pipelines.delete(pipelineId);
    this.emit('pipeline-deleted', { pipelineId });
  }

  async triggerRun(pipelineId: string, trigger: types.TriggerInfo): Promise<types.PipelineRun> {
    const pipeline = this.pipelines.get(pipelineId);
    if (!pipeline) throw new Error('Pipeline not found');

    const runNumber = Array.from(this.runs.values())
      .filter(r => r.pipelineId === pipelineId).length + 1;

    const run: types.PipelineRun = {
      id: `run-${Date.now()}`,
      pipelineId,
      number: runNumber,
      status: types.PipelineStatus.Pending,
      trigger,
      stages: pipeline.stages.map(stage => ({
        stageId: stage.id,
        stageName: stage.name,
        status: types.PipelineStatus.Pending,
        steps: stage.steps.map(step => ({
          stepId: step.id,
          stepName: step.name,
          status: types.PipelineStatus.Pending,
          attempt: 1
        }))
      })),
      artifacts: [],
      logs: []
    };

    this.runs.set(run.id, run);
    this.emit('run-triggered', run);
    return run;
  }

  async startRun(runId: string): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    run.status = types.PipelineStatus.Running;
    run.startedAt = new Date();

    this.addLog(run, 'info', 'Pipeline run started');
    this.emit('run-started', run);
  }

  async executeStage(runId: string, stageId: string): Promise<types.StageRun> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    const stageRun = run.stages.find(s => s.stageId === stageId);
    if (!stageRun) throw new Error('Stage not found');

    stageRun.status = types.PipelineStatus.Running;
    stageRun.startedAt = new Date();

    this.addLog(run, 'info', `Stage "${stageRun.stageName}" started`, stageId);
    this.emit('stage-started', { runId, stageRun });

    return stageRun;
  }

  async completeStep(runId: string, stageId: string, stepId: string, result: Partial<types.StepRun>): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    const stageRun = run.stages.find(s => s.stageId === stageId);
    if (!stageRun) throw new Error('Stage not found');

    const stepRun = stageRun.steps.find(s => s.stepId === stepId);
    if (!stepRun) throw new Error('Step not found');

    Object.assign(stepRun, result);
    stepRun.finishedAt = new Date();
    if (stepRun.startedAt) {
      stepRun.duration = stepRun.finishedAt.getTime() - stepRun.startedAt.getTime();
    }

    this.addLog(run, stepRun.status === types.PipelineStatus.Success ? 'info' : 'error',
      `Step "${stepRun.stepName}" ${stepRun.status}`, stageId, stepId);
    this.emit('step-completed', { runId, stageId, stepRun });
  }

  async completeStage(runId: string, stageId: string, status: types.PipelineStatus): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    const stageRun = run.stages.find(s => s.stageId === stageId);
    if (!stageRun) throw new Error('Stage not found');

    stageRun.status = status;
    stageRun.finishedAt = new Date();
    if (stageRun.startedAt) {
      stageRun.duration = stageRun.finishedAt.getTime() - stageRun.startedAt.getTime();
    }

    this.addLog(run, status === types.PipelineStatus.Success ? 'info' : 'error',
      `Stage "${stageRun.stageName}" ${status}`, stageId);
    this.emit('stage-completed', { runId, stageRun });
  }

  async completeRun(runId: string, status: types.PipelineStatus): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    run.status = status;
    run.finishedAt = new Date();
    if (run.startedAt) {
      run.duration = run.finishedAt.getTime() - run.startedAt.getTime();
    }

    this.addLog(run, status === types.PipelineStatus.Success ? 'info' : 'error',
      `Pipeline run ${status}`);
    this.emit('run-completed', run);
  }

  async cancelRun(runId: string): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    run.status = types.PipelineStatus.Cancelled;
    run.finishedAt = new Date();

    this.addLog(run, 'warn', 'Pipeline run cancelled');
    this.emit('run-cancelled', run);
  }

  private addLog(run: types.PipelineRun, level: 'debug' | 'info' | 'warn' | 'error',
    message: string, stage?: string, step?: string): void {
    run.logs.push({ timestamp: new Date(), level, stage, step, message });
  }

  getRun(runId: string): types.PipelineRun | undefined {
    return this.runs.get(runId);
  }

  getPipelineRuns(pipelineId: string, limit?: number): types.PipelineRun[] {
    const runs = Array.from(this.runs.values())
      .filter(r => r.pipelineId === pipelineId)
      .sort((a, b) => b.number - a.number);
    return limit ? runs.slice(0, limit) : runs;
  }

  async uploadArtifact(runId: string, artifact: Omit<types.Artifact, 'id' | 'createdAt'>): Promise<types.Artifact> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    const newArtifact: types.Artifact = {
      ...artifact,
      id: `artifact-${Date.now()}`,
      createdAt: new Date()
    };

    this.artifacts.set(newArtifact.id, newArtifact);
    run.artifacts.push(newArtifact);
    this.emit('artifact-uploaded', { runId, artifact: newArtifact });
    return newArtifact;
  }

  getArtifact(artifactId: string): types.Artifact | undefined {
    return this.artifacts.get(artifactId);
  }

  async registerTarget(target: Omit<types.DeploymentTarget, 'id'>): Promise<types.DeploymentTarget> {
    const newTarget: types.DeploymentTarget = { ...target, id: `target-${Date.now()}` };
    this.targets.set(newTarget.id, newTarget);
    this.emit('target-registered', newTarget);
    return newTarget;
  }

  getTarget(targetId: string): types.DeploymentTarget | undefined {
    return this.targets.get(targetId);
  }

  getTargetsByEnvironment(env: types.EnvironmentType): types.DeploymentTarget[] {
    return Array.from(this.targets.values()).filter(t => t.environment === env);
  }

  async recordTestResults(runId: string, results: types.TestResult[]): Promise<{ passed: number; failed: number; skipped: number }> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    const summary = {
      passed: results.filter(r => r.status === 'passed').length,
      failed: results.filter(r => r.status === 'failed').length,
      skipped: results.filter(r => r.status === 'skipped').length
    };

    this.emit('test-results', { runId, results, summary });
    return summary;
  }

  async recordCoverage(runId: string, coverage: types.CoverageReport): Promise<void> {
    const run = this.runs.get(runId);
    if (!run) throw new Error('Run not found');

    this.emit('coverage-reported', { runId, coverage });
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.ComplianceTest[] = [];

    tests.push({
      testName: 'API Endpoint Configuration',
      passed: this.config.apiEndpoint !== undefined,
      notes: 'API endpoint must be configured'
    });

    tests.push({
      testName: 'Pipeline Support',
      passed: true,
      notes: 'Pipeline creation and execution supported'
    });

    tests.push({
      testName: 'Artifact Management',
      passed: this.config.artifactRetention > 0,
      notes: 'Artifact retention must be configured'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Metrics Collection',
        passed: this.config.enableMetrics === true,
        notes: 'Metrics required for Silver/Gold'
      });

      tests.push({
        testName: 'Concurrent Runs',
        passed: this.config.maxConcurrentRuns >= 5,
        notes: 'At least 5 concurrent runs required for Silver/Gold'
      });
    }

    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'Log Retention',
        passed: this.config.logRetention >= 90,
        notes: '90-day log retention required for Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-CI',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export class PipelineBuilder {
  private pipeline: Partial<types.Pipeline> = { stages: [], triggers: [], notifications: [] };

  name(name: string): this { this.pipeline.name = name; return this; }
  description(desc: string): this { this.pipeline.description = desc; return this; }
  repository(repo: types.RepositoryConfig): this { this.pipeline.repository = repo; return this; }
  timeout(ms: number): this { this.pipeline.timeout = ms; return this; }

  addTrigger(trigger: types.TriggerConfig): this {
    this.pipeline.triggers!.push(trigger);
    return this;
  }

  addStage(stage: types.Stage): this {
    this.pipeline.stages!.push(stage);
    return this;
  }

  addNotification(config: types.NotificationConfig): this {
    this.pipeline.notifications!.push(config);
    return this;
  }

  environment(env: types.EnvironmentVariables): this {
    this.pipeline.environment = env;
    return this;
  }

  build(): Omit<types.Pipeline, 'id' | 'createdAt' | 'updatedAt'> {
    if (!this.pipeline.name) throw new Error('Pipeline name is required');
    if (!this.pipeline.repository) throw new Error('Repository is required');
    return this.pipeline as Omit<types.Pipeline, 'id' | 'createdAt' | 'updatedAt'>;
  }
}

export default { WIACI, PipelineBuilder };
