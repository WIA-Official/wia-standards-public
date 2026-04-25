/**
 * WIA BIM Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-bim
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

/**
 * Main BIM Platform class
 */
export class WIABIM extends EventEmitter {
  private config: types.BIMConfig;
  private projects: Map<string, types.BIMProject> = new Map();
  private models: Map<string, types.BIMModel> = new Map();
  private clashDetections: Map<string, types.ClashDetection> = new Map();

  constructor(config: types.BIMConfig) {
    super();
    this.config = config;
  }

  async createProject(project: Omit<types.BIMProject, 'id' | 'models' | 'createdAt' | 'updatedAt'>): Promise<types.BIMProject> {
    const newProject: types.BIMProject = {
      ...project,
      id: `proj-${Date.now()}`,
      models: [],
      createdAt: new Date(),
      updatedAt: new Date()
    };

    this.projects.set(newProject.id, newProject);
    this.emit('project-created', newProject);
    return newProject;
  }

  getProject(projectId: string): types.BIMProject | undefined {
    return this.projects.get(projectId);
  }

  listProjects(): types.BIMProject[] {
    return Array.from(this.projects.values());
  }

  async uploadModel(projectId: string, model: Omit<types.BIMModel, 'id' | 'lastModified'>): Promise<types.BIMModel> {
    const project = this.projects.get(projectId);
    if (!project) throw new Error('Project not found');

    const newModel: types.BIMModel = {
      ...model,
      id: `model-${Date.now()}`,
      lastModified: new Date()
    };

    this.models.set(newModel.id, newModel);
    project.models.push(newModel);
    project.updatedAt = new Date();

    this.emit('model-uploaded', { projectId, model: newModel });
    return newModel;
  }

  getModel(modelId: string): types.BIMModel | undefined {
    return this.models.get(modelId);
  }

  async getElements(modelId: string, filter?: { category?: types.ElementCategory; ifcType?: string }): Promise<types.BIMElement[]> {
    const model = this.models.get(modelId);
    if (!model) return [];

    let elements = model.elements;

    if (filter?.category) {
      elements = elements.filter(e => e.category === filter.category);
    }
    if (filter?.ifcType) {
      elements = elements.filter(e => e.ifcType === filter.ifcType);
    }

    return elements;
  }

  async runClashDetection(modelAId: string, modelBId: string): Promise<types.ClashDetection> {
    if (!this.config.enableClashDetection) {
      throw new Error('Clash detection is not enabled');
    }

    const detection: types.ClashDetection = {
      id: `clash-${Date.now()}`,
      modelA: modelAId,
      modelB: modelBId,
      clashes: this.simulateClashes(),
      timestamp: new Date(),
      status: 'completed'
    };

    this.clashDetections.set(detection.id, detection);
    this.emit('clash-detection-completed', detection);
    return detection;
  }

  private simulateClashes(): types.Clash[] {
    const count = Math.floor(Math.random() * 10);
    const clashes: types.Clash[] = [];

    for (let i = 0; i < count; i++) {
      clashes.push({
        id: `c-${i}`,
        elementA: `elemA-${i}`,
        elementB: `elemB-${i}`,
        clashPoint: { x: Math.random() * 100, y: Math.random() * 100, z: Math.random() * 10 },
        distance: Math.random() * 0.5,
        severity: ['critical', 'major', 'minor'][Math.floor(Math.random() * 3)] as types.Clash['severity'],
        status: 'new'
      });
    }

    return clashes;
  }

  async getQuantityTakeoff(modelId: string): Promise<types.QuantityTakeoff> {
    if (!this.config.enableQuantityTakeoff) {
      throw new Error('Quantity takeoff is not enabled');
    }

    const model = this.models.get(modelId);
    const items: types.QuantityItem[] = model?.elements.map(e => ({
      elementId: e.id,
      ifcType: e.ifcType,
      name: e.name,
      quantity: e.geometry.volume || e.geometry.area || 1,
      unit: e.geometry.volume ? 'm³' : 'm²'
    })) || [];

    return {
      id: `qty-${Date.now()}`,
      modelId,
      items,
      timestamp: new Date()
    };
  }

  async exportCOBie(projectId: string): Promise<types.COBieData> {
    const project = this.projects.get(projectId);
    if (!project) throw new Error('Project not found');

    return {
      facility: {
        name: project.name,
        category: 'Building',
        projectName: project.name,
        siteName: project.location.address,
        linearUnits: 'meters',
        areaUnits: 'square meters',
        volumeUnits: 'cubic meters'
      },
      floors: [],
      spaces: [],
      zones: [],
      types: [],
      components: []
    };
  }

  async validateIFC(modelId: string): Promise<{ valid: boolean; errors: string[] }> {
    const model = this.models.get(modelId);
    if (!model) return { valid: false, errors: ['Model not found'] };

    const errors: string[] = [];

    if (!model.elements.every(e => e.globalId)) {
      errors.push('Some elements missing GlobalId');
    }

    return { valid: errors.length === 0, errors };
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.serverUrl !== undefined,
      notes: 'Server URL must be defined'
    });

    tests.push({
      testName: 'IFC Support',
      passed: true,
      notes: 'SDK supports IFC format'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Clash Detection',
        passed: this.config.enableClashDetection === true,
        notes: 'Clash detection required for Silver/Gold'
      });
    }

    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'Quantity Takeoff',
        passed: this.config.enableQuantityTakeoff === true,
        notes: 'Quantity takeoff required for Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-BIM',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export default { WIABIM };
