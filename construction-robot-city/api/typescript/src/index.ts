/**
 * WIA Construction Robot City Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIARobotCityProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  Robot,
  RobotStatus,
  ConstructionTask,
  TaskStatus,
  CityZone,
  BuildingPlan,
  Metric,
  SafetyZone,
} from './types';

// ============================================================================
// WIA Robot City Client
// ============================================================================

export class WIARobotCityClient {
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
  // Project Management
  // ========================================================================

  async createProject(project: WIARobotCityProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIARobotCityProject> {
    const response = await this.axios.get<WIARobotCityProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    status?: string;
    country?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIARobotCityProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // ========================================================================
  // Robot Fleet Management
  // ========================================================================

  async listRobots(projectId: string, params?: {
    type?: string;
    status?: string;
    zone?: string;
    limit?: number;
  }): Promise<PaginatedResponse<Robot>> {
    const response = await this.axios.get<PaginatedResponse<Robot>>(
      `/projects/${projectId}/robots`,
      { params }
    );
    return response.data;
  }

  async getRobot(projectId: string, robotId: string): Promise<Robot> {
    const response = await this.axios.get<Robot>(
      `/projects/${projectId}/robots/${robotId}`
    );
    return response.data;
  }

  async registerRobot(projectId: string, robot: Partial<Robot>): Promise<Robot> {
    const response = await this.axios.post<Robot>(
      `/projects/${projectId}/robots`,
      robot
    );
    return response.data;
  }

  async updateRobotStatus(
    projectId: string,
    robotId: string,
    status: RobotStatus
  ): Promise<Robot> {
    const response = await this.axios.put<Robot>(
      `/projects/${projectId}/robots/${robotId}/status`,
      { status }
    );
    return response.data;
  }

  async assignTask(
    projectId: string,
    robotId: string,
    taskId: string
  ): Promise<TaskAssignment> {
    const response = await this.axios.post<TaskAssignment>(
      `/projects/${projectId}/robots/${robotId}/assign`,
      { taskId }
    );
    return response.data;
  }

  async getRobotTelemetry(
    projectId: string,
    robotId: string,
    params?: { start?: string; end?: string }
  ): Promise<TelemetryData[]> {
    const response = await this.axios.get<TelemetryData[]>(
      `/projects/${projectId}/robots/${robotId}/telemetry`,
      { params }
    );
    return response.data;
  }

  async sendRobotCommand(
    projectId: string,
    robotId: string,
    command: RobotCommand
  ): Promise<CommandResult> {
    const response = await this.axios.post<CommandResult>(
      `/projects/${projectId}/robots/${robotId}/command`,
      command
    );
    return response.data;
  }

  async getFleetStatus(projectId: string): Promise<FleetStatus> {
    const response = await this.axios.get<FleetStatus>(
      `/projects/${projectId}/fleet/status`
    );
    return response.data;
  }

  // ========================================================================
  // Construction Tasks
  // ========================================================================

  async listTasks(projectId: string, params?: {
    zone?: string;
    type?: string;
    status?: string;
    limit?: number;
  }): Promise<PaginatedResponse<ConstructionTask>> {
    const response = await this.axios.get<PaginatedResponse<ConstructionTask>>(
      `/projects/${projectId}/tasks`,
      { params }
    );
    return response.data;
  }

  async getTask(projectId: string, taskId: string): Promise<ConstructionTask> {
    const response = await this.axios.get<ConstructionTask>(
      `/projects/${projectId}/tasks/${taskId}`
    );
    return response.data;
  }

  async createTask(projectId: string, task: Partial<ConstructionTask>): Promise<ConstructionTask> {
    const response = await this.axios.post<ConstructionTask>(
      `/projects/${projectId}/tasks`,
      task
    );
    return response.data;
  }

  async updateTaskStatus(
    projectId: string,
    taskId: string,
    status: TaskStatus,
    progress?: number
  ): Promise<ConstructionTask> {
    const response = await this.axios.put<ConstructionTask>(
      `/projects/${projectId}/tasks/${taskId}/status`,
      { status, progress }
    );
    return response.data;
  }

  async scheduleTask(
    projectId: string,
    taskId: string,
    schedule: TaskSchedule
  ): Promise<ConstructionTask> {
    const response = await this.axios.post<ConstructionTask>(
      `/projects/${projectId}/tasks/${taskId}/schedule`,
      schedule
    );
    return response.data;
  }

  async getTaskDependencies(projectId: string, taskId: string): Promise<TaskDependency[]> {
    const response = await this.axios.get<TaskDependency[]>(
      `/projects/${projectId}/tasks/${taskId}/dependencies`
    );
    return response.data;
  }

  // ========================================================================
  // City Zones & Buildings
  // ========================================================================

  async listZones(projectId: string): Promise<CityZone[]> {
    const response = await this.axios.get<CityZone[]>(`/projects/${projectId}/zones`);
    return response.data;
  }

  async getZone(projectId: string, zoneId: string): Promise<CityZone> {
    const response = await this.axios.get<CityZone>(
      `/projects/${projectId}/zones/${zoneId}`
    );
    return response.data;
  }

  async createZone(projectId: string, zone: Partial<CityZone>): Promise<CityZone> {
    const response = await this.axios.post<CityZone>(
      `/projects/${projectId}/zones`,
      zone
    );
    return response.data;
  }

  async getZoneProgress(projectId: string, zoneId: string): Promise<ZoneProgress> {
    const response = await this.axios.get<ZoneProgress>(
      `/projects/${projectId}/zones/${zoneId}/progress`
    );
    return response.data;
  }

  async listBuildings(projectId: string, params?: {
    zone?: string;
    type?: string;
    status?: string;
  }): Promise<BuildingPlan[]> {
    const response = await this.axios.get<BuildingPlan[]>(
      `/projects/${projectId}/buildings`,
      { params }
    );
    return response.data;
  }

  async getBuilding(projectId: string, buildingId: string): Promise<BuildingPlan> {
    const response = await this.axios.get<BuildingPlan>(
      `/projects/${projectId}/buildings/${buildingId}`
    );
    return response.data;
  }

  async getBuildingProgress(projectId: string, buildingId: string): Promise<BuildingProgress> {
    const response = await this.axios.get<BuildingProgress>(
      `/projects/${projectId}/buildings/${buildingId}/progress`
    );
    return response.data;
  }

  // ========================================================================
  // Safety & Monitoring
  // ========================================================================

  async getSafetyZones(projectId: string): Promise<SafetyZone[]> {
    const response = await this.axios.get<SafetyZone[]>(
      `/projects/${projectId}/safety/zones`
    );
    return response.data;
  }

  async createSafetyZone(projectId: string, zone: Partial<SafetyZone>): Promise<SafetyZone> {
    const response = await this.axios.post<SafetyZone>(
      `/projects/${projectId}/safety/zones`,
      zone
    );
    return response.data;
  }

  async triggerEmergencyStop(projectId: string, scope: 'all' | 'zone' | 'robot', target?: string): Promise<EmergencyStopResult> {
    const response = await this.axios.post<EmergencyStopResult>(
      `/projects/${projectId}/safety/emergency-stop`,
      { scope, target }
    );
    return response.data;
  }

  async reportIncident(projectId: string, incident: IncidentReport): Promise<IncidentResult> {
    const response = await this.axios.post<IncidentResult>(
      `/projects/${projectId}/safety/incidents`,
      incident
    );
    return response.data;
  }

  async getIncidents(projectId: string, params?: {
    severity?: string;
    start?: string;
    end?: string;
  }): Promise<IncidentRecord[]> {
    const response = await this.axios.get<IncidentRecord[]>(
      `/projects/${projectId}/safety/incidents`,
      { params }
    );
    return response.data;
  }

  async getMetrics(projectId: string, metrics?: string[]): Promise<MetricValue[]> {
    const response = await this.axios.get<MetricValue[]>(
      `/projects/${projectId}/monitoring/metrics`,
      { params: { metrics: metrics?.join(',') } }
    );
    return response.data;
  }

  async getAlerts(projectId: string, params?: {
    severity?: string;
    acknowledged?: boolean;
    limit?: number;
  }): Promise<Alert[]> {
    const response = await this.axios.get<Alert[]>(
      `/projects/${projectId}/monitoring/alerts`,
      { params }
    );
    return response.data;
  }

  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/monitoring/alerts/${alertId}/acknowledge`);
  }

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(
      `/projects/${projectId}/dashboard`
    );
    return response.data;
  }

  // ========================================================================
  // Simulation & Planning
  // ========================================================================

  async runSimulation(projectId: string, scenario: SimulationScenario): Promise<SimulationResult> {
    const response = await this.axios.post<SimulationResult>(
      `/projects/${projectId}/simulation`,
      scenario
    );
    return response.data;
  }

  async optimizeSchedule(projectId: string, constraints: OptimizationConstraints): Promise<OptimizedSchedule> {
    const response = await this.axios.post<OptimizedSchedule>(
      `/projects/${projectId}/optimize/schedule`,
      constraints
    );
    return response.data;
  }

  async optimizeFleet(projectId: string, objectives: OptimizationObjectives): Promise<FleetOptimization> {
    const response = await this.axios.post<FleetOptimization>(
      `/projects/${projectId}/optimize/fleet`,
      objectives
    );
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIARobotCityProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CONSTRUCTION-ROBOT-CITY') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CONSTRUCTION-ROBOT-CITY"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.robotFleet?.robots?.length) {
      errors.push({
        path: 'robotFleet.robots',
        message: 'At least one robot is required',
      });
    }

    if (!project.cityDesign?.zones?.length) {
      errors.push({
        path: 'cityDesign.zones',
        message: 'At least one city zone is required',
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface TaskAssignment {
  robotId: string;
  taskId: string;
  assignedAt: string;
  estimatedDuration: number;
  status: 'assigned' | 'started' | 'completed';
}

export interface TelemetryData {
  timestamp: string;
  robotId: string;
  location: { latitude: number; longitude: number; elevation: number };
  battery: number;
  speed: number;
  sensors: Record<string, number>;
  status: string;
}

export interface RobotCommand {
  type: 'move' | 'stop' | 'execute' | 'return' | 'charge';
  parameters?: Record<string, unknown>;
  priority: 'low' | 'normal' | 'high' | 'emergency';
}

export interface CommandResult {
  commandId: string;
  status: 'accepted' | 'rejected' | 'executing' | 'completed';
  message?: string;
  timestamp: string;
}

export interface FleetStatus {
  total: number;
  active: number;
  idle: number;
  charging: number;
  maintenance: number;
  error: number;
  utilization: number;
}

export interface TaskSchedule {
  startTime: string;
  endTime?: string;
  robots: string[];
  priority: number;
}

export interface TaskDependency {
  taskId: string;
  dependsOn: string;
  type: 'finish-start' | 'start-start' | 'finish-finish';
  lag?: number;
}

export interface ZoneProgress {
  zoneId: string;
  tasksTotal: number;
  tasksCompleted: number;
  buildingsTotal: number;
  buildingsCompleted: number;
  progress: number;
  estimatedCompletion: string;
}

export interface BuildingProgress {
  buildingId: string;
  phase: string;
  progress: number;
  tasksCompleted: number;
  tasksRemaining: number;
  robotsAssigned: number;
  estimatedCompletion: string;
}

export interface EmergencyStopResult {
  initiated: boolean;
  affectedRobots: number;
  timestamp: string;
  reason?: string;
}

export interface IncidentReport {
  type: 'safety' | 'equipment' | 'environmental' | 'other';
  severity: 'low' | 'medium' | 'high' | 'critical';
  location?: { latitude: number; longitude: number };
  description: string;
  robotsInvolved?: string[];
  injuries?: number;
}

export interface IncidentResult {
  incidentId: string;
  status: 'reported' | 'investigating' | 'resolved';
  reportedAt: string;
}

export interface IncidentRecord extends IncidentReport {
  id: string;
  status: string;
  reportedAt: string;
  resolvedAt?: string;
  resolution?: string;
}

export interface MetricValue {
  id: string;
  name: string;
  value: number;
  unit: string;
  timestamp: string;
  trend?: 'up' | 'down' | 'stable';
}

export interface Alert {
  id: string;
  type: string;
  severity: 'info' | 'warning' | 'critical';
  message: string;
  timestamp: string;
  acknowledged: boolean;
  source?: string;
}

export interface DashboardData {
  project: { name: string; status: string; progress: number };
  fleet: FleetStatus;
  tasks: { pending: number; inProgress: number; completed: number; blocked: number };
  zones: { total: number; active: number; completed: number };
  safety: { incidents: number; alerts: number; score: number };
  metrics: MetricValue[];
}

export interface SimulationScenario {
  type: 'construction' | 'fleet' | 'logistics' | 'emergency';
  duration: number;
  parameters: Record<string, unknown>;
}

export interface SimulationResult {
  id: string;
  status: 'completed' | 'failed';
  duration: number;
  metrics: Record<string, number>;
  recommendations: string[];
}

export interface OptimizationConstraints {
  deadline?: string;
  budget?: number;
  robotLimit?: number;
  priorities?: string[];
}

export interface OptimizedSchedule {
  tasks: { taskId: string; start: string; end: string; robots: string[] }[];
  duration: number;
  efficiency: number;
  bottlenecks: string[];
}

export interface OptimizationObjectives {
  minimize: ('time' | 'cost' | 'energy' | 'robots')[];
  constraints: Record<string, unknown>;
}

export interface FleetOptimization {
  recommendations: { type: string; description: string; impact: number }[];
  optimalSize: number;
  distribution: Record<string, number>;
  savings: number;
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export function createMinimalProject(name: string, country: string): WIARobotCityProject {
  return {
    standard: 'WIA-CONSTRUCTION-ROBOT-CITY',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: 'smart-city',
      location: { country, coordinates: { latitude: 0, longitude: 0 }, area: { value: 100, unit: 'km2' }, terrain: 'flat' },
      scale: { population: 100000, buildings: 1000, roads: 200, greenSpace: 30 },
      timeline: { startDate: new Date().toISOString().split('T')[0], phases: [], milestones: [] },
      developer: { name: 'Developer', type: 'developer', country },
      status: 'concept',
    },
    cityDesign: { masterPlan: { designPhilosophy: 'Smart sustainable city', densityTarget: 150, sustainabilityGoals: [], smartFeatures: [], blueprints: [] }, zones: [], buildings: [], transportation: { roads: { totalLength: 0, lanes: 4, surfaceType: 'asphalt', smartFeatures: [] }, publicTransit: { type: [], stations: 0, coverage: 0, capacity: 0 }, pedestrian: { walkways: 0, bikeways: 0, accessibility: 'universal' }, autonomous: { coverage: 0, chargingStations: 0, controlCenter: '' } }, utilities: { electricity: { capacity: 0, renewable: 0, smartGrid: true, storage: 0 }, water: { source: '', treatment: '', recycling: 0, smartMetering: true }, waste: { collection: '', recycling: 0, processing: '', automation: true }, telecommunications: { coverage: '', technology: [], bandwidth: 0, redundancy: true } }, greenSpaces: { percentage: 30, parks: [], vegetation: { trees: 0, coverage: 0, species: [], maintenance: '' }, ecosystem: { biodiversity: '', wildlife: [], sustainability: [] } } },
    robotFleet: { totalRobots: 0, categories: [], robots: [], controlSystem: { type: 'hybrid', aiEngine: { model: '', capabilities: [], learningEnabled: true, decisionLatency: 0 }, coordination: { algorithm: '', conflictResolution: '', taskAllocation: '' }, communication: { protocol: '', frequency: '', range: 0, redundancy: true, encryption: true } }, maintenance: { schedule: { preventive: '', predictive: true, emergency: '' }, facilities: [], spares: { categories: [], reorderPoint: 0, supplier: '' } } },
    construction: { methodology: { approach: 'hybrid', automation: 80, humanRoles: [], robotRoles: [] }, tasks: [], materials: { requirements: [], suppliers: [], storage: [], tracking: { technology: 'RFID', realTime: true, integration: [] } }, logistics: { routes: [], vehicles: [], scheduling: { algorithm: '', optimization: [], constraints: [] } }, quality: { standards: [], inspections: { automated: true, frequency: '', checkpoints: [], robotInspectors: true }, testing: { types: [], equipment: [], acceptance: '' }, documentation: '' } },
    infrastructure: { digital: { network: { type: '5G', coverage: 100, bandwidth: 1000, latency: 1, redundancy: true }, dataCenter: { location: '', capacity: 0, tier: 3, backup: '' }, iotPlatform: { sensors: 0, devices: 0, protocol: '', analytics: '' }, digitalTwin: { scope: 'full', realTime: true, simulation: true, predictive: true } }, physical: { foundations: '', structures: '', utilities: '', accessibility: '' }, energy: { generation: { sources: [], renewable: 100, backup: '' }, storage: { type: '', capacity: 0, duration: 0 }, distribution: { grid: '', smartMetering: true, loadBalancing: true }, efficiency: 95 }, resilience: { hazards: [], mitigation: [], recovery: '', redundancy: [] } },
    automation: { level: 4, systems: [], integration: { platform: '', protocols: [], dataExchange: '', interoperability: [] }, humanInterface: { controlCenter: { location: '', staffing: 0, capabilities: [], redundancy: true }, monitoring: { dashboards: [], alerts: [], visualization: [] }, intervention: { triggers: [], procedures: [], authority: [] } } },
    safety: { standards: [], zones: [], procedures: [], emergency: { scenarios: [], resources: [], communication: '', evacuation: '' }, humanSafety: { training: [], ppe: [], protocols: [], monitoring: '' } },
    monitoring: { realTime: { dashboards: [], metrics: [], refresh: 1 }, analytics: { capabilities: [], predictive: true, optimization: [], reporting: [] }, reporting: { automated: true, templates: [], schedule: '', formats: [] }, alerts: { channels: [], escalation: [], response: '' } },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIARobotCityClient,
  generateUUID,
  createMinimalProject,
};
