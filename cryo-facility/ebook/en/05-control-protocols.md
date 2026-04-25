# Chapter 5: Cryogenic Facility Control Protocols

## Operational Workflows, Equipment Control, and Safety Procedures

This chapter defines the control protocols and operational workflows that govern cryogenic facility operations. These protocols ensure consistent, safe, and compliant management of equipment, specimens, and personnel across all facility operations.

---

## Protocol Architecture Overview

### Control System Hierarchy

```typescript
/**
 * WIA Cryo Facility Control Protocol Architecture
 * Hierarchical control system design
 */

interface ControlSystemArchitecture {
  // Strategic layer - facility-wide policies
  strategic: StrategicControlLayer;

  // Tactical layer - operational workflows
  tactical: TacticalControlLayer;

  // Operational layer - equipment control
  operational: OperationalControlLayer;

  // Safety layer - override controls
  safety: SafetyControlLayer;
}

interface StrategicControlLayer {
  policies: FacilityPolicy[];
  compliance: ComplianceFramework;
  qualitySystem: QualityManagementSystem;
  riskManagement: RiskManagementFramework;
}

interface TacticalControlLayer {
  workflows: OperationalWorkflow[];
  scheduling: SchedulingSystem;
  resourceManagement: ResourceManager;
  reporting: ReportingSystem;
}

interface OperationalControlLayer {
  equipmentControl: EquipmentControlSystem;
  environmentalControl: EnvironmentalControlSystem;
  accessControl: AccessControlSystem;
  monitoringControl: MonitoringControlSystem;
}

interface SafetyControlLayer {
  emergencyProtocols: EmergencyProtocol[];
  safetyInterlocks: SafetyInterlock[];
  alarmSystem: AlarmSystem;
  overrideControls: OverrideControl[];
}

// Control system implementation
class CryoFacilityControlSystem {
  private strategic: StrategicControlManager;
  private tactical: TacticalControlManager;
  private operational: OperationalControlManager;
  private safety: SafetyControlManager;

  constructor(config: ControlSystemConfig) {
    this.strategic = new StrategicControlManager(config.strategic);
    this.tactical = new TacticalControlManager(config.tactical);
    this.operational = new OperationalControlManager(config.operational);
    this.safety = new SafetyControlManager(config.safety);
  }

  async executeWorkflow(
    workflowId: string,
    context: WorkflowContext
  ): Promise<WorkflowResult> {
    // Safety check first
    const safetyStatus = await this.safety.checkSafetyConditions(context);
    if (!safetyStatus.safe) {
      return {
        success: false,
        error: `Safety check failed: ${safetyStatus.reason}`,
        safetyHold: true
      };
    }

    // Compliance check
    const complianceStatus = await this.strategic.checkCompliance(workflowId, context);
    if (!complianceStatus.compliant) {
      return {
        success: false,
        error: `Compliance check failed: ${complianceStatus.violations.join(', ')}`,
        complianceHold: true
      };
    }

    // Execute workflow
    return this.tactical.executeWorkflow(workflowId, context);
  }

  async controlEquipment(
    command: EquipmentCommand
  ): Promise<EquipmentCommandResult> {
    // Safety validation
    const safetyResult = await this.safety.validateEquipmentCommand(command);
    if (!safetyResult.allowed) {
      throw new SafetyViolationError(safetyResult.reason);
    }

    // Execute command
    return this.operational.executeEquipmentCommand(command);
  }

  async handleEmergency(
    emergencyType: EmergencyType,
    context: EmergencyContext
  ): Promise<EmergencyResponse> {
    // Emergency protocols take precedence
    return this.safety.executeEmergencyProtocol(emergencyType, context);
  }
}

interface ControlSystemConfig {
  strategic: StrategicConfig;
  tactical: TacticalConfig;
  operational: OperationalConfig;
  safety: SafetyConfig;
}

interface WorkflowContext {
  facilityId: string;
  userId: string;
  timestamp: string;
  parameters: Record<string, unknown>;
}

interface WorkflowResult {
  success: boolean;
  error?: string;
  safetyHold?: boolean;
  complianceHold?: boolean;
  data?: unknown;
}

class SafetyViolationError extends Error {
  constructor(message: string) {
    super(`Safety violation: ${message}`);
    this.name = 'SafetyViolationError';
  }
}
```

---

## Equipment Control Protocols

### Equipment State Management

```typescript
/**
 * Equipment Control State Machine
 * State transitions and control commands
 */

// Equipment states
type EquipmentState =
  | 'idle'
  | 'starting'
  | 'running'
  | 'cooling'
  | 'warming'
  | 'maintenance'
  | 'alarm'
  | 'emergency_shutdown'
  | 'offline';

// State transition events
type EquipmentEvent =
  | 'start'
  | 'stop'
  | 'cool_down'
  | 'warm_up'
  | 'enter_maintenance'
  | 'exit_maintenance'
  | 'alarm_triggered'
  | 'alarm_cleared'
  | 'emergency_stop'
  | 'reset';

interface StateTransition {
  from: EquipmentState;
  event: EquipmentEvent;
  to: EquipmentState;
  guards: TransitionGuard[];
  actions: TransitionAction[];
}

interface TransitionGuard {
  name: string;
  condition: (context: EquipmentContext) => boolean;
  errorMessage: string;
}

interface TransitionAction {
  name: string;
  execute: (context: EquipmentContext) => Promise<void>;
  rollback?: (context: EquipmentContext) => Promise<void>;
}

class EquipmentStateMachine {
  private currentState: EquipmentState;
  private transitions: StateTransition[];
  private context: EquipmentContext;

  constructor(
    initialState: EquipmentState,
    transitions: StateTransition[],
    context: EquipmentContext
  ) {
    this.currentState = initialState;
    this.transitions = transitions;
    this.context = context;
  }

  async transition(event: EquipmentEvent): Promise<TransitionResult> {
    const validTransitions = this.transitions.filter(
      t => t.from === this.currentState && t.event === event
    );

    if (validTransitions.length === 0) {
      return {
        success: false,
        error: `No valid transition for event ${event} in state ${this.currentState}`
      };
    }

    const transition = validTransitions[0];

    // Check guards
    for (const guard of transition.guards) {
      if (!guard.condition(this.context)) {
        return {
          success: false,
          error: guard.errorMessage,
          guardFailed: guard.name
        };
      }
    }

    // Execute actions
    const executedActions: TransitionAction[] = [];
    try {
      for (const action of transition.actions) {
        await action.execute(this.context);
        executedActions.push(action);
      }

      const previousState = this.currentState;
      this.currentState = transition.to;

      return {
        success: true,
        previousState,
        newState: this.currentState
      };
    } catch (error) {
      // Rollback executed actions
      for (const action of executedActions.reverse()) {
        if (action.rollback) {
          await action.rollback(this.context);
        }
      }

      return {
        success: false,
        error: `Action failed: ${error}`,
        rolledBack: true
      };
    }
  }

  getState(): EquipmentState {
    return this.currentState;
  }

  canTransition(event: EquipmentEvent): boolean {
    return this.transitions.some(
      t => t.from === this.currentState && t.event === event
    );
  }

  getAvailableEvents(): EquipmentEvent[] {
    return this.transitions
      .filter(t => t.from === this.currentState)
      .map(t => t.event);
  }
}

interface EquipmentContext {
  equipmentId: string;
  facilityId: string;
  currentTemperature: number;
  targetTemperature: number;
  currentLevel: number;
  operatorId: string;
  timestamp: string;
  metadata: Record<string, unknown>;
}

interface TransitionResult {
  success: boolean;
  error?: string;
  previousState?: EquipmentState;
  newState?: EquipmentState;
  guardFailed?: string;
  rolledBack?: boolean;
}

// Predefined transitions for LN2 tank
const ln2TankTransitions: StateTransition[] = [
  {
    from: 'idle',
    event: 'start',
    to: 'running',
    guards: [
      {
        name: 'level_check',
        condition: (ctx) => ctx.currentLevel > 20,
        errorMessage: 'LN2 level too low to start'
      },
      {
        name: 'temperature_check',
        condition: (ctx) => ctx.currentTemperature < -150,
        errorMessage: 'Temperature not low enough to start'
      }
    ],
    actions: [
      {
        name: 'log_start',
        execute: async (ctx) => {
          console.log(`Starting LN2 tank ${ctx.equipmentId}`);
        }
      },
      {
        name: 'enable_monitoring',
        execute: async (ctx) => {
          // Enable continuous monitoring
        }
      }
    ]
  },
  {
    from: 'running',
    event: 'alarm_triggered',
    to: 'alarm',
    guards: [],
    actions: [
      {
        name: 'notify_operators',
        execute: async (ctx) => {
          // Send notifications
        }
      },
      {
        name: 'increase_monitoring',
        execute: async (ctx) => {
          // Increase monitoring frequency
        }
      }
    ]
  },
  {
    from: 'alarm',
    event: 'emergency_stop',
    to: 'emergency_shutdown',
    guards: [],
    actions: [
      {
        name: 'emergency_procedures',
        execute: async (ctx) => {
          // Execute emergency shutdown procedures
        }
      },
      {
        name: 'notify_emergency',
        execute: async (ctx) => {
          // Notify emergency contacts
        }
      }
    ]
  },
  {
    from: 'running',
    event: 'enter_maintenance',
    to: 'maintenance',
    guards: [
      {
        name: 'specimen_check',
        condition: (ctx) => {
          // Check if specimens can be safely maintained during maintenance
          return true;
        },
        errorMessage: 'Cannot enter maintenance with current specimen load'
      }
    ],
    actions: [
      {
        name: 'prepare_maintenance',
        execute: async (ctx) => {
          // Prepare for maintenance mode
        }
      }
    ]
  }
];
```

---

### Equipment Control Commands

```typescript
/**
 * Equipment Control Command System
 * Comprehensive command interface
 */

interface EquipmentCommand {
  commandId: string;
  equipmentId: string;
  facilityId: string;
  type: CommandType;
  parameters: CommandParameters;
  requestedBy: string;
  requestedAt: string;
  priority: CommandPriority;
  timeout?: number;
}

type CommandType =
  | 'start'
  | 'stop'
  | 'set_temperature'
  | 'set_level_alarm'
  | 'calibrate_sensor'
  | 'fill_ln2'
  | 'drain_ln2'
  | 'enter_maintenance'
  | 'exit_maintenance'
  | 'emergency_stop'
  | 'reset_alarm'
  | 'test_alarm';

type CommandPriority = 'low' | 'normal' | 'high' | 'critical';

interface CommandParameters {
  targetTemperature?: number;
  targetLevel?: number;
  sensorId?: string;
  maintenanceType?: string;
  alarmId?: string;
  testDuration?: number;
  [key: string]: unknown;
}

interface EquipmentCommandResult {
  commandId: string;
  success: boolean;
  executedAt: string;
  completedAt?: string;
  error?: string;
  result?: unknown;
  telemetry?: CommandTelemetry;
}

interface CommandTelemetry {
  executionTime: number;
  resourcesUsed: string[];
  stateChanges: StateChange[];
}

interface StateChange {
  property: string;
  previousValue: unknown;
  newValue: unknown;
  timestamp: string;
}

class EquipmentCommandExecutor {
  private commandHandlers: Map<CommandType, CommandHandler>;
  private commandQueue: PriorityQueue<EquipmentCommand>;
  private runningCommands: Map<string, EquipmentCommand>;

  constructor() {
    this.commandHandlers = new Map();
    this.commandQueue = new PriorityQueue();
    this.runningCommands = new Map();
    this.registerHandlers();
  }

  private registerHandlers(): void {
    this.commandHandlers.set('set_temperature', new SetTemperatureHandler());
    this.commandHandlers.set('fill_ln2', new FillLN2Handler());
    this.commandHandlers.set('calibrate_sensor', new CalibrateSensorHandler());
    this.commandHandlers.set('enter_maintenance', new MaintenanceModeHandler());
    this.commandHandlers.set('emergency_stop', new EmergencyStopHandler());
    this.commandHandlers.set('reset_alarm', new ResetAlarmHandler());
  }

  async executeCommand(command: EquipmentCommand): Promise<EquipmentCommandResult> {
    const handler = this.commandHandlers.get(command.type);
    if (!handler) {
      return {
        commandId: command.commandId,
        success: false,
        executedAt: new Date().toISOString(),
        error: `Unknown command type: ${command.type}`
      };
    }

    // Validate command
    const validation = await handler.validate(command);
    if (!validation.valid) {
      return {
        commandId: command.commandId,
        success: false,
        executedAt: new Date().toISOString(),
        error: `Validation failed: ${validation.errors?.join(', ')}`
      };
    }

    // Execute command
    this.runningCommands.set(command.commandId, command);
    const startTime = Date.now();

    try {
      const result = await handler.execute(command);
      const executionTime = Date.now() - startTime;

      return {
        commandId: command.commandId,
        success: true,
        executedAt: new Date(startTime).toISOString(),
        completedAt: new Date().toISOString(),
        result: result.data,
        telemetry: {
          executionTime,
          resourcesUsed: result.resourcesUsed,
          stateChanges: result.stateChanges
        }
      };
    } catch (error) {
      return {
        commandId: command.commandId,
        success: false,
        executedAt: new Date(startTime).toISOString(),
        completedAt: new Date().toISOString(),
        error: String(error)
      };
    } finally {
      this.runningCommands.delete(command.commandId);
    }
  }

  queueCommand(command: EquipmentCommand): void {
    this.commandQueue.enqueue(command, this.getPriorityValue(command.priority));
  }

  private getPriorityValue(priority: CommandPriority): number {
    const values: Record<CommandPriority, number> = {
      critical: 0,
      high: 1,
      normal: 2,
      low: 3
    };
    return values[priority];
  }
}

interface CommandHandler {
  validate(command: EquipmentCommand): Promise<ValidationResult>;
  execute(command: EquipmentCommand): Promise<CommandExecutionResult>;
}

interface CommandExecutionResult {
  data?: unknown;
  resourcesUsed: string[];
  stateChanges: StateChange[];
}

// Example command handlers
class SetTemperatureHandler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    const errors: ValidationError[] = [];

    if (command.parameters.targetTemperature === undefined) {
      errors.push({ path: 'targetTemperature', message: 'Target temperature required' });
    }

    const temp = command.parameters.targetTemperature as number;
    if (temp > -80 || temp < -200) {
      errors.push({
        path: 'targetTemperature',
        message: 'Temperature must be between -200°C and -80°C'
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    const targetTemp = command.parameters.targetTemperature as number;

    // Simulate temperature change
    console.log(`Setting temperature to ${targetTemp}°C for equipment ${command.equipmentId}`);

    return {
      data: { newTargetTemperature: targetTemp },
      resourcesUsed: ['temperature-controller'],
      stateChanges: [
        {
          property: 'targetTemperature',
          previousValue: -196,
          newValue: targetTemp,
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

class EmergencyStopHandler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    // Emergency stop always valid
    return { valid: true };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    console.log(`EMERGENCY STOP executed for equipment ${command.equipmentId}`);

    // Execute emergency stop procedures
    return {
      data: { emergencyStopActivated: true },
      resourcesUsed: ['emergency-controller', 'alarm-system', 'notification-system'],
      stateChanges: [
        {
          property: 'operationalState',
          previousValue: 'running',
          newValue: 'emergency_shutdown',
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

class FillLN2Handler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    const errors: ValidationError[] = [];

    if (command.parameters.targetLevel === undefined) {
      errors.push({ path: 'targetLevel', message: 'Target level required' });
    }

    const level = command.parameters.targetLevel as number;
    if (level < 0 || level > 100) {
      errors.push({
        path: 'targetLevel',
        message: 'Target level must be between 0 and 100%'
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    const targetLevel = command.parameters.targetLevel as number;

    console.log(`Initiating LN2 fill to ${targetLevel}% for equipment ${command.equipmentId}`);

    return {
      data: { fillInitiated: true, targetLevel },
      resourcesUsed: ['ln2-supply', 'fill-valve'],
      stateChanges: [
        {
          property: 'fillStatus',
          previousValue: 'idle',
          newValue: 'filling',
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

class CalibrateSensorHandler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    if (!command.parameters.sensorId) {
      return {
        valid: false,
        errors: [{ path: 'sensorId', message: 'Sensor ID required' }]
      };
    }
    return { valid: true };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    const sensorId = command.parameters.sensorId as string;

    console.log(`Calibrating sensor ${sensorId} on equipment ${command.equipmentId}`);

    // Simulate calibration process
    return {
      data: {
        calibrationCompleted: true,
        sensorId,
        newCalibrationDate: new Date().toISOString(),
        nextCalibrationDue: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000).toISOString()
      },
      resourcesUsed: ['calibration-reference'],
      stateChanges: [
        {
          property: `sensors.${sensorId}.calibration`,
          previousValue: null,
          newValue: new Date().toISOString(),
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

class MaintenanceModeHandler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    return { valid: true };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    const maintenanceType = command.parameters.maintenanceType as string || 'general';

    console.log(`Entering maintenance mode (${maintenanceType}) for equipment ${command.equipmentId}`);

    return {
      data: {
        maintenanceMode: true,
        type: maintenanceType,
        enteredAt: new Date().toISOString()
      },
      resourcesUsed: [],
      stateChanges: [
        {
          property: 'operationalState',
          previousValue: 'running',
          newValue: 'maintenance',
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

class ResetAlarmHandler implements CommandHandler {
  async validate(command: EquipmentCommand): Promise<ValidationResult> {
    if (!command.parameters.alarmId) {
      return {
        valid: false,
        errors: [{ path: 'alarmId', message: 'Alarm ID required' }]
      };
    }
    return { valid: true };
  }

  async execute(command: EquipmentCommand): Promise<CommandExecutionResult> {
    const alarmId = command.parameters.alarmId as string;

    console.log(`Resetting alarm ${alarmId} on equipment ${command.equipmentId}`);

    return {
      data: { alarmReset: true, alarmId },
      resourcesUsed: ['alarm-system'],
      stateChanges: [
        {
          property: `alarms.${alarmId}.status`,
          previousValue: 'active',
          newValue: 'cleared',
          timestamp: new Date().toISOString()
        }
      ]
    };
  }
}

// Priority queue implementation
class PriorityQueue<T> {
  private items: { item: T; priority: number }[] = [];

  enqueue(item: T, priority: number): void {
    this.items.push({ item, priority });
    this.items.sort((a, b) => a.priority - b.priority);
  }

  dequeue(): T | undefined {
    return this.items.shift()?.item;
  }

  peek(): T | undefined {
    return this.items[0]?.item;
  }

  isEmpty(): boolean {
    return this.items.length === 0;
  }
}
```

---

## Operational Workflows

### Workflow Engine

```typescript
/**
 * Operational Workflow Engine
 * Process automation and orchestration
 */

interface OperationalWorkflow {
  id: string;
  name: string;
  description: string;
  type: WorkflowType;
  version: string;
  steps: WorkflowStep[];
  triggers: WorkflowTrigger[];
  errorHandling: ErrorHandlingStrategy;
  timeout: number;
  retryPolicy: RetryPolicy;
}

type WorkflowType =
  | 'specimen_receipt'
  | 'specimen_storage'
  | 'specimen_retrieval'
  | 'equipment_maintenance'
  | 'calibration'
  | 'quality_control'
  | 'emergency_response'
  | 'ln2_fill'
  | 'inventory_audit';

interface WorkflowStep {
  id: string;
  name: string;
  type: StepType;
  config: StepConfig;
  inputs: StepInput[];
  outputs: StepOutput[];
  transitions: StepTransition[];
  timeout?: number;
}

type StepType =
  | 'manual'
  | 'automated'
  | 'decision'
  | 'parallel'
  | 'wait'
  | 'notification'
  | 'integration';

interface StepConfig {
  handler?: string;
  parameters?: Record<string, unknown>;
  validation?: ValidationRule[];
  assignments?: Assignment[];
}

interface StepInput {
  name: string;
  type: string;
  source: 'workflow' | 'step' | 'external';
  required: boolean;
  default?: unknown;
}

interface StepOutput {
  name: string;
  type: string;
  target: 'workflow' | 'step' | 'external';
}

interface StepTransition {
  target: string;
  condition?: string;
  default?: boolean;
}

interface WorkflowTrigger {
  type: TriggerType;
  config: TriggerConfig;
}

type TriggerType = 'manual' | 'scheduled' | 'event' | 'threshold';

interface TriggerConfig {
  schedule?: string;  // Cron expression
  eventType?: string;
  threshold?: ThresholdConfig;
  authorization?: string[];
}

interface ThresholdConfig {
  parameter: string;
  operator: 'gt' | 'lt' | 'eq' | 'gte' | 'lte';
  value: number;
}

interface ErrorHandlingStrategy {
  onStepError: 'fail' | 'retry' | 'skip' | 'compensate';
  onWorkflowError: 'fail' | 'rollback' | 'notify';
  compensationSteps?: string[];
  notificationRecipients?: string[];
}

interface RetryPolicy {
  maxRetries: number;
  initialDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}

class WorkflowEngine {
  private workflows: Map<string, OperationalWorkflow> = new Map();
  private instances: Map<string, WorkflowInstance> = new Map();
  private stepHandlers: Map<string, StepHandler> = new Map();

  registerWorkflow(workflow: OperationalWorkflow): void {
    this.workflows.set(workflow.id, workflow);
  }

  registerStepHandler(stepType: string, handler: StepHandler): void {
    this.stepHandlers.set(stepType, handler);
  }

  async startWorkflow(
    workflowId: string,
    context: WorkflowContext
  ): Promise<WorkflowInstance> {
    const workflow = this.workflows.get(workflowId);
    if (!workflow) {
      throw new Error(`Workflow not found: ${workflowId}`);
    }

    const instance: WorkflowInstance = {
      id: this.generateInstanceId(),
      workflowId,
      status: 'running',
      currentStepId: workflow.steps[0].id,
      context,
      variables: new Map(),
      history: [],
      startedAt: new Date().toISOString()
    };

    this.instances.set(instance.id, instance);

    // Start execution
    this.executeWorkflow(instance).catch(error => {
      instance.status = 'failed';
      instance.error = String(error);
    });

    return instance;
  }

  private async executeWorkflow(instance: WorkflowInstance): Promise<void> {
    const workflow = this.workflows.get(instance.workflowId)!;

    while (instance.status === 'running') {
      const currentStep = workflow.steps.find(s => s.id === instance.currentStepId);
      if (!currentStep) {
        instance.status = 'completed';
        instance.completedAt = new Date().toISOString();
        break;
      }

      try {
        const result = await this.executeStep(currentStep, instance);

        instance.history.push({
          stepId: currentStep.id,
          status: 'completed',
          result,
          timestamp: new Date().toISOString()
        });

        // Determine next step
        const nextStepId = this.determineNextStep(currentStep, result, instance);
        if (!nextStepId) {
          instance.status = 'completed';
          instance.completedAt = new Date().toISOString();
        } else {
          instance.currentStepId = nextStepId;
        }
      } catch (error) {
        instance.history.push({
          stepId: currentStep.id,
          status: 'failed',
          error: String(error),
          timestamp: new Date().toISOString()
        });

        const handled = await this.handleStepError(
          workflow,
          currentStep,
          error,
          instance
        );

        if (!handled) {
          instance.status = 'failed';
          instance.error = String(error);
          instance.completedAt = new Date().toISOString();
        }
      }
    }
  }

  private async executeStep(
    step: WorkflowStep,
    instance: WorkflowInstance
  ): Promise<StepResult> {
    const handler = this.stepHandlers.get(step.type);
    if (!handler) {
      throw new Error(`No handler for step type: ${step.type}`);
    }

    // Prepare inputs
    const inputs: Record<string, unknown> = {};
    for (const input of step.inputs) {
      inputs[input.name] = this.resolveInput(input, instance);
    }

    // Execute step
    const result = await handler.execute(step, inputs, instance.context);

    // Store outputs
    for (const output of step.outputs) {
      if (result.outputs && result.outputs[output.name] !== undefined) {
        instance.variables.set(output.name, result.outputs[output.name]);
      }
    }

    return result;
  }

  private resolveInput(input: StepInput, instance: WorkflowInstance): unknown {
    switch (input.source) {
      case 'workflow':
        return instance.context.parameters[input.name];
      case 'step':
        return instance.variables.get(input.name);
      case 'external':
        return this.fetchExternalInput(input.name);
      default:
        return input.default;
    }
  }

  private fetchExternalInput(name: string): unknown {
    // Fetch from external source
    return null;
  }

  private determineNextStep(
    step: WorkflowStep,
    result: StepResult,
    instance: WorkflowInstance
  ): string | null {
    for (const transition of step.transitions) {
      if (transition.condition) {
        if (this.evaluateCondition(transition.condition, result, instance)) {
          return transition.target;
        }
      } else if (transition.default) {
        return transition.target;
      }
    }

    return null;
  }

  private evaluateCondition(
    condition: string,
    result: StepResult,
    instance: WorkflowInstance
  ): boolean {
    // Simple expression evaluation
    try {
      const context = {
        result: result.outputs,
        variables: Object.fromEntries(instance.variables)
      };
      // In production, use a safe expression evaluator
      return eval(condition);
    } catch {
      return false;
    }
  }

  private async handleStepError(
    workflow: OperationalWorkflow,
    step: WorkflowStep,
    error: unknown,
    instance: WorkflowInstance
  ): Promise<boolean> {
    switch (workflow.errorHandling.onStepError) {
      case 'retry':
        // Implement retry logic
        return false;
      case 'skip':
        // Move to next step
        const defaultTransition = step.transitions.find(t => t.default);
        if (defaultTransition) {
          instance.currentStepId = defaultTransition.target;
          return true;
        }
        return false;
      case 'compensate':
        // Execute compensation
        return false;
      default:
        return false;
    }
  }

  private generateInstanceId(): string {
    return `wf-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  getInstanceStatus(instanceId: string): WorkflowInstance | undefined {
    return this.instances.get(instanceId);
  }
}

interface WorkflowInstance {
  id: string;
  workflowId: string;
  status: 'running' | 'completed' | 'failed' | 'paused';
  currentStepId: string;
  context: WorkflowContext;
  variables: Map<string, unknown>;
  history: StepExecution[];
  startedAt: string;
  completedAt?: string;
  error?: string;
}

interface StepExecution {
  stepId: string;
  status: 'completed' | 'failed' | 'skipped';
  result?: StepResult;
  error?: string;
  timestamp: string;
}

interface StepHandler {
  execute(
    step: WorkflowStep,
    inputs: Record<string, unknown>,
    context: WorkflowContext
  ): Promise<StepResult>;
}

interface StepResult {
  success: boolean;
  outputs?: Record<string, unknown>;
  error?: string;
}

interface ValidationRule {
  field: string;
  type: string;
  constraint: unknown;
  message: string;
}

interface Assignment {
  assignee: string;
  role?: string;
  escalation?: EscalationRule[];
}

interface EscalationRule {
  after: number;
  to: string;
  notification: string;
}
```

---

## Emergency Protocols

### Emergency Response System

```typescript
/**
 * Emergency Response Protocol System
 * Critical situation handling
 */

type EmergencyType =
  | 'temperature_excursion'
  | 'ln2_level_critical'
  | 'power_failure'
  | 'equipment_failure'
  | 'fire'
  | 'flood'
  | 'gas_leak'
  | 'security_breach'
  | 'medical_emergency';

interface EmergencyProtocol {
  type: EmergencyType;
  severity: EmergencySeverity;
  description: string;
  immediateActions: EmergencyAction[];
  followUpActions: EmergencyAction[];
  notificationChain: NotificationChain;
  escalationProcedure: EscalationProcedure;
  resourceRequirements: ResourceRequirement[];
  documentationRequirements: string[];
}

type EmergencySeverity = 'warning' | 'critical' | 'catastrophic';

interface EmergencyAction {
  id: string;
  description: string;
  responsible: string;
  automated: boolean;
  timeout?: number;
  verification?: string;
  dependencies?: string[];
}

interface NotificationChain {
  immediate: NotificationRecipient[];
  escalation: NotificationEscalation[];
  external: ExternalNotification[];
}

interface NotificationRecipient {
  role: string;
  method: 'sms' | 'call' | 'email' | 'page';
  priority: number;
}

interface NotificationEscalation {
  afterMinutes: number;
  recipients: NotificationRecipient[];
}

interface ExternalNotification {
  organization: string;
  condition: string;
  contact: string;
}

interface EscalationProcedure {
  levels: EscalationLevel[];
  defaultTimeout: number;
}

interface EscalationLevel {
  level: number;
  authority: string;
  actions: string[];
  timeout: number;
}

interface ResourceRequirement {
  resource: string;
  quantity: number;
  location: string;
}

class EmergencyResponseSystem {
  private protocols: Map<EmergencyType, EmergencyProtocol> = new Map();
  private activeEmergencies: Map<string, ActiveEmergency> = new Map();
  private notificationService: NotificationService;
  private auditLog: AuditLogService;

  constructor(
    notificationService: NotificationService,
    auditLog: AuditLogService
  ) {
    this.notificationService = notificationService;
    this.auditLog = auditLog;
    this.initializeProtocols();
  }

  private initializeProtocols(): void {
    // Temperature Excursion Protocol
    this.protocols.set('temperature_excursion', {
      type: 'temperature_excursion',
      severity: 'critical',
      description: 'Storage temperature exceeds acceptable range',
      immediateActions: [
        {
          id: 'temp-1',
          description: 'Verify sensor readings on alternative sensors',
          responsible: 'on-call-technician',
          automated: false,
          timeout: 5
        },
        {
          id: 'temp-2',
          description: 'Check LN2 levels',
          responsible: 'automated-system',
          automated: true
        },
        {
          id: 'temp-3',
          description: 'Initiate emergency LN2 fill if needed',
          responsible: 'on-call-technician',
          automated: false,
          dependencies: ['temp-2']
        },
        {
          id: 'temp-4',
          description: 'Assess specimen relocation if temperature exceeds -150°C',
          responsible: 'lab-supervisor',
          automated: false,
          timeout: 15
        }
      ],
      followUpActions: [
        {
          id: 'temp-f1',
          description: 'Root cause analysis',
          responsible: 'quality-manager',
          automated: false
        },
        {
          id: 'temp-f2',
          description: 'Equipment inspection and maintenance',
          responsible: 'maintenance-team',
          automated: false
        },
        {
          id: 'temp-f3',
          description: 'Update deviation report',
          responsible: 'quality-manager',
          automated: false
        }
      ],
      notificationChain: {
        immediate: [
          { role: 'on-call-technician', method: 'sms', priority: 1 },
          { role: 'on-call-technician', method: 'call', priority: 2 },
          { role: 'lab-supervisor', method: 'sms', priority: 3 }
        ],
        escalation: [
          {
            afterMinutes: 5,
            recipients: [
              { role: 'lab-director', method: 'call', priority: 1 },
              { role: 'facility-manager', method: 'sms', priority: 2 }
            ]
          },
          {
            afterMinutes: 15,
            recipients: [
              { role: 'executive-director', method: 'call', priority: 1 }
            ]
          }
        ],
        external: []
      },
      escalationProcedure: {
        levels: [
          { level: 1, authority: 'on-call-technician', actions: ['monitor', 'basic-intervention'], timeout: 15 },
          { level: 2, authority: 'lab-supervisor', actions: ['specimen-relocation', 'equipment-shutdown'], timeout: 30 },
          { level: 3, authority: 'lab-director', actions: ['emergency-transport', 'vendor-escalation'], timeout: 60 }
        ],
        defaultTimeout: 15
      },
      resourceRequirements: [
        { resource: 'portable-ln2-dewar', quantity: 2, location: 'emergency-storage' },
        { resource: 'transport-containers', quantity: 5, location: 'supply-room' }
      ],
      documentationRequirements: [
        'Temperature log during event',
        'Actions taken with timestamps',
        'Personnel involved',
        'Specimens affected',
        'Root cause analysis'
      ]
    });

    // Power Failure Protocol
    this.protocols.set('power_failure', {
      type: 'power_failure',
      severity: 'critical',
      description: 'Complete or partial power failure at facility',
      immediateActions: [
        {
          id: 'power-1',
          description: 'Verify UPS systems engaged',
          responsible: 'automated-system',
          automated: true
        },
        {
          id: 'power-2',
          description: 'Confirm generator startup',
          responsible: 'automated-system',
          automated: true,
          timeout: 2
        },
        {
          id: 'power-3',
          description: 'Verify critical equipment powered',
          responsible: 'on-call-technician',
          automated: false,
          timeout: 10
        },
        {
          id: 'power-4',
          description: 'Contact utility company',
          responsible: 'facility-manager',
          automated: false
        }
      ],
      followUpActions: [
        {
          id: 'power-f1',
          description: 'Review generator performance',
          responsible: 'maintenance-team',
          automated: false
        },
        {
          id: 'power-f2',
          description: 'Replenish generator fuel',
          responsible: 'maintenance-team',
          automated: false
        }
      ],
      notificationChain: {
        immediate: [
          { role: 'facility-manager', method: 'call', priority: 1 },
          { role: 'on-call-technician', method: 'sms', priority: 2 }
        ],
        escalation: [
          {
            afterMinutes: 10,
            recipients: [
              { role: 'lab-director', method: 'call', priority: 1 }
            ]
          }
        ],
        external: [
          {
            organization: 'Utility Company',
            condition: 'extended_outage',
            contact: 'emergency-line'
          }
        ]
      },
      escalationProcedure: {
        levels: [
          { level: 1, authority: 'facility-manager', actions: ['monitor-backup', 'utility-contact'], timeout: 30 },
          { level: 2, authority: 'lab-director', actions: ['extended-backup-plan'], timeout: 120 }
        ],
        defaultTimeout: 30
      },
      resourceRequirements: [
        { resource: 'portable-generator', quantity: 1, location: 'backup-storage' },
        { resource: 'diesel-fuel', quantity: 100, location: 'fuel-storage' }
      ],
      documentationRequirements: [
        'Time of power failure',
        'Duration of outage',
        'Backup system performance',
        'Equipment status during event'
      ]
    });
  }

  async triggerEmergency(
    type: EmergencyType,
    context: EmergencyContext
  ): Promise<ActiveEmergency> {
    const protocol = this.protocols.get(type);
    if (!protocol) {
      throw new Error(`Unknown emergency type: ${type}`);
    }

    const emergency: ActiveEmergency = {
      id: this.generateEmergencyId(),
      type,
      protocol,
      status: 'active',
      startedAt: new Date().toISOString(),
      context,
      actionLog: [],
      escalationLevel: 1,
      acknowledgedBy: null,
      notifications: []
    };

    this.activeEmergencies.set(emergency.id, emergency);

    // Log emergency start
    await this.auditLog.log({
      type: 'emergency_started',
      emergencyId: emergency.id,
      emergencyType: type,
      context,
      timestamp: new Date().toISOString()
    });

    // Send immediate notifications
    await this.sendNotifications(emergency, protocol.notificationChain.immediate);

    // Execute automated immediate actions
    await this.executeAutomatedActions(emergency, protocol.immediateActions);

    // Start escalation timer
    this.startEscalationTimer(emergency);

    return emergency;
  }

  async acknowledgeEmergency(
    emergencyId: string,
    userId: string
  ): Promise<void> {
    const emergency = this.activeEmergencies.get(emergencyId);
    if (!emergency) {
      throw new Error(`Emergency not found: ${emergencyId}`);
    }

    emergency.acknowledgedBy = userId;
    emergency.acknowledgedAt = new Date().toISOString();

    await this.auditLog.log({
      type: 'emergency_acknowledged',
      emergencyId,
      userId,
      timestamp: new Date().toISOString()
    });
  }

  async completeAction(
    emergencyId: string,
    actionId: string,
    result: ActionResult
  ): Promise<void> {
    const emergency = this.activeEmergencies.get(emergencyId);
    if (!emergency) {
      throw new Error(`Emergency not found: ${emergencyId}`);
    }

    emergency.actionLog.push({
      actionId,
      completedAt: new Date().toISOString(),
      completedBy: result.completedBy,
      result: result.success ? 'success' : 'failed',
      notes: result.notes
    });

    await this.auditLog.log({
      type: 'emergency_action_completed',
      emergencyId,
      actionId,
      result,
      timestamp: new Date().toISOString()
    });

    // Check if all immediate actions completed
    const allCompleted = this.checkActionsCompleted(emergency);
    if (allCompleted) {
      await this.transitionToFollowUp(emergency);
    }
  }

  async resolveEmergency(
    emergencyId: string,
    resolution: EmergencyResolution
  ): Promise<void> {
    const emergency = this.activeEmergencies.get(emergencyId);
    if (!emergency) {
      throw new Error(`Emergency not found: ${emergencyId}`);
    }

    emergency.status = 'resolved';
    emergency.resolvedAt = new Date().toISOString();
    emergency.resolution = resolution;

    await this.auditLog.log({
      type: 'emergency_resolved',
      emergencyId,
      resolution,
      timestamp: new Date().toISOString()
    });

    // Generate incident report
    await this.generateIncidentReport(emergency);
  }

  private async sendNotifications(
    emergency: ActiveEmergency,
    recipients: NotificationRecipient[]
  ): Promise<void> {
    for (const recipient of recipients) {
      const notification = await this.notificationService.send({
        emergency,
        recipient,
        timestamp: new Date().toISOString()
      });
      emergency.notifications.push(notification);
    }
  }

  private async executeAutomatedActions(
    emergency: ActiveEmergency,
    actions: EmergencyAction[]
  ): Promise<void> {
    const automatedActions = actions.filter(a => a.automated);

    for (const action of automatedActions) {
      try {
        // Execute automated action
        await this.executeAction(action);

        emergency.actionLog.push({
          actionId: action.id,
          completedAt: new Date().toISOString(),
          completedBy: 'automated-system',
          result: 'success'
        });
      } catch (error) {
        emergency.actionLog.push({
          actionId: action.id,
          completedAt: new Date().toISOString(),
          completedBy: 'automated-system',
          result: 'failed',
          notes: String(error)
        });
      }
    }
  }

  private async executeAction(action: EmergencyAction): Promise<void> {
    // Execute the automated action based on action.id
    console.log(`Executing automated action: ${action.id} - ${action.description}`);
  }

  private startEscalationTimer(emergency: ActiveEmergency): void {
    const escalations = emergency.protocol.notificationChain.escalation;

    for (const escalation of escalations) {
      setTimeout(async () => {
        if (
          emergency.status === 'active' &&
          emergency.escalationLevel < escalations.length + 1
        ) {
          await this.sendNotifications(emergency, escalation.recipients);
          emergency.escalationLevel++;
        }
      }, escalation.afterMinutes * 60 * 1000);
    }
  }

  private checkActionsCompleted(emergency: ActiveEmergency): boolean {
    const immediateActions = emergency.protocol.immediateActions;
    const completedActions = new Set(emergency.actionLog.map(a => a.actionId));

    return immediateActions.every(a => completedActions.has(a.id));
  }

  private async transitionToFollowUp(emergency: ActiveEmergency): Promise<void> {
    emergency.status = 'follow_up';

    await this.auditLog.log({
      type: 'emergency_follow_up',
      emergencyId: emergency.id,
      timestamp: new Date().toISOString()
    });
  }

  private async generateIncidentReport(emergency: ActiveEmergency): Promise<void> {
    // Generate comprehensive incident report
    console.log(`Generating incident report for emergency ${emergency.id}`);
  }

  private generateEmergencyId(): string {
    return `EMG-${Date.now()}-${Math.random().toString(36).substr(2, 6).toUpperCase()}`;
  }
}

interface EmergencyContext {
  facilityId: string;
  equipmentId?: string;
  zone?: string;
  detectedBy: string;
  initialReadings?: Record<string, number>;
  additionalInfo?: string;
}

interface ActiveEmergency {
  id: string;
  type: EmergencyType;
  protocol: EmergencyProtocol;
  status: 'active' | 'follow_up' | 'resolved';
  startedAt: string;
  context: EmergencyContext;
  actionLog: ActionLogEntry[];
  escalationLevel: number;
  acknowledgedBy: string | null;
  acknowledgedAt?: string;
  notifications: any[];
  resolvedAt?: string;
  resolution?: EmergencyResolution;
}

interface ActionLogEntry {
  actionId: string;
  completedAt: string;
  completedBy: string;
  result: 'success' | 'failed';
  notes?: string;
}

interface ActionResult {
  success: boolean;
  completedBy: string;
  notes?: string;
}

interface EmergencyResolution {
  summary: string;
  rootCause?: string;
  preventiveMeasures?: string[];
  specimensAffected?: string[];
  equipmentStatus: string;
  followUpRequired: boolean;
}

// Notification service stub
class NotificationService {
  async send(params: any): Promise<any> {
    return { sent: true, ...params };
  }
}

// Audit log service stub
class AuditLogService {
  async log(entry: any): Promise<void> {
    console.log('Audit log:', entry);
  }
}
```

---

## Chapter Summary

This chapter covered the comprehensive control protocols for cryogenic facilities:

- **Equipment State Management**: State machine-based equipment control
- **Command System**: Structured command execution with validation
- **Workflow Engine**: Process automation and orchestration
- **Emergency Response**: Comprehensive emergency protocols and handling

---

*© 2025 World Industry Association. All rights reserved.*

*弘益人間 (Benefit All Humanity)*
