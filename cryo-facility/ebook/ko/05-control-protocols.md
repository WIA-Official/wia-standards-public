# 제5장: 제어 프로토콜

## 5.1 개요

극저온 시설의 효과적인 운영을 위해서는 장비 제어, 워크플로우 관리, 비상 대응을 위한 체계적인 프로토콜이 필수적입니다. 이 장에서는 장비 상태 관리, 명령 실행, 운영 워크플로우, 비상 대응 시스템을 상세히 다룹니다.

```typescript
// 제어 프로토콜 아키텍처
const controlProtocolArchitecture = {
  equipmentControl: {
    stateMachine: '장비 상태 전이 관리',
    commandExecutor: '명령 실행 및 검증',
    safetyInterlock: '안전 인터락 시스템'
  },
  workflowEngine: {
    processDefinition: '프로세스 정의',
    taskOrchestration: '작업 오케스트레이션',
    approvalWorkflow: '승인 워크플로우'
  },
  emergencyResponse: {
    protocolDefinition: '비상 프로토콜 정의',
    automaticResponse: '자동 대응 시스템',
    escalationManagement: '에스컬레이션 관리'
  },
  safetyFeatures: [
    '다중 레벨 인터락',
    '수동 오버라이드 방지',
    '감사 로깅',
    '실시간 모니터링'
  ]
};
```

## 5.2 장비 상태 기계

### 5.2.1 상태 기계 구현

```typescript
import { EventEmitter } from 'events';

// 장비 상태 열거형
export enum EquipmentState {
  OFFLINE = 'offline',
  INITIALIZING = 'initializing',
  OPERATIONAL = 'operational',
  STANDBY = 'standby',
  MAINTENANCE = 'maintenance',
  CALIBRATION = 'calibration',
  FAULT = 'fault',
  EMERGENCY_STOP = 'emergency-stop',
  DECOMMISSIONED = 'decommissioned'
}

// 상태 전이 이벤트
export enum StateTransitionEvent {
  POWER_ON = 'power-on',
  INITIALIZATION_COMPLETE = 'initialization-complete',
  ACTIVATE = 'activate',
  DEACTIVATE = 'deactivate',
  START_MAINTENANCE = 'start-maintenance',
  END_MAINTENANCE = 'end-maintenance',
  START_CALIBRATION = 'start-calibration',
  END_CALIBRATION = 'end-calibration',
  FAULT_DETECTED = 'fault-detected',
  FAULT_CLEARED = 'fault-cleared',
  EMERGENCY_STOP = 'emergency-stop',
  EMERGENCY_RESET = 'emergency-reset',
  DECOMMISSION = 'decommission',
  POWER_OFF = 'power-off'
}

// 상태 전이 정의
interface StateTransition {
  from: EquipmentState | EquipmentState[];
  to: EquipmentState;
  event: StateTransitionEvent;
  guard?: (context: TransitionContext) => boolean;
  action?: (context: TransitionContext) => Promise<void>;
}

interface TransitionContext {
  equipmentId: string;
  currentState: EquipmentState;
  event: StateTransitionEvent;
  data?: any;
  user?: User;
  timestamp: Date;
}

// 장비 상태 기계
export class EquipmentStateMachine extends EventEmitter {
  private state: EquipmentState;
  private transitions: StateTransition[];
  private history: TransitionRecord[] = [];

  constructor(
    private equipmentId: string,
    initialState: EquipmentState = EquipmentState.OFFLINE
  ) {
    super();
    this.state = initialState;
    this.transitions = this.defineTransitions();
  }

  // 상태 전이 정의
  private defineTransitions(): StateTransition[] {
    return [
      // 전원 켜기
      {
        from: EquipmentState.OFFLINE,
        to: EquipmentState.INITIALIZING,
        event: StateTransitionEvent.POWER_ON,
        action: async (ctx) => {
          await this.runInitializationSequence(ctx);
        }
      },

      // 초기화 완료
      {
        from: EquipmentState.INITIALIZING,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.INITIALIZATION_COMPLETE,
        guard: (ctx) => this.isInitializationSuccessful(ctx)
      },

      // 활성화
      {
        from: EquipmentState.STANDBY,
        to: EquipmentState.OPERATIONAL,
        event: StateTransitionEvent.ACTIVATE,
        guard: (ctx) => this.canActivate(ctx),
        action: async (ctx) => {
          await this.startMonitoring(ctx);
        }
      },

      // 비활성화
      {
        from: EquipmentState.OPERATIONAL,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.DEACTIVATE,
        action: async (ctx) => {
          await this.stopMonitoring(ctx);
        }
      },

      // 유지보수 시작
      {
        from: [EquipmentState.STANDBY, EquipmentState.OPERATIONAL],
        to: EquipmentState.MAINTENANCE,
        event: StateTransitionEvent.START_MAINTENANCE,
        guard: (ctx) => this.hasMaintenancePermission(ctx),
        action: async (ctx) => {
          await this.prepareForMaintenance(ctx);
        }
      },

      // 유지보수 종료
      {
        from: EquipmentState.MAINTENANCE,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.END_MAINTENANCE,
        guard: (ctx) => this.isMaintenanceComplete(ctx),
        action: async (ctx) => {
          await this.postMaintenanceChecks(ctx);
        }
      },

      // 교정 시작
      {
        from: [EquipmentState.STANDBY, EquipmentState.MAINTENANCE],
        to: EquipmentState.CALIBRATION,
        event: StateTransitionEvent.START_CALIBRATION,
        guard: (ctx) => this.hasCalibrationPermission(ctx)
      },

      // 교정 종료
      {
        from: EquipmentState.CALIBRATION,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.END_CALIBRATION,
        guard: (ctx) => this.isCalibrationValid(ctx),
        action: async (ctx) => {
          await this.recordCalibration(ctx);
        }
      },

      // 고장 감지
      {
        from: [
          EquipmentState.INITIALIZING,
          EquipmentState.OPERATIONAL,
          EquipmentState.STANDBY,
          EquipmentState.MAINTENANCE,
          EquipmentState.CALIBRATION
        ],
        to: EquipmentState.FAULT,
        event: StateTransitionEvent.FAULT_DETECTED,
        action: async (ctx) => {
          await this.handleFault(ctx);
        }
      },

      // 고장 해제
      {
        from: EquipmentState.FAULT,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.FAULT_CLEARED,
        guard: (ctx) => this.isFaultResolved(ctx),
        action: async (ctx) => {
          await this.clearFaultCondition(ctx);
        }
      },

      // 비상 정지
      {
        from: [
          EquipmentState.OPERATIONAL,
          EquipmentState.STANDBY,
          EquipmentState.MAINTENANCE,
          EquipmentState.CALIBRATION,
          EquipmentState.FAULT
        ],
        to: EquipmentState.EMERGENCY_STOP,
        event: StateTransitionEvent.EMERGENCY_STOP,
        action: async (ctx) => {
          await this.executeEmergencyStop(ctx);
        }
      },

      // 비상 정지 해제
      {
        from: EquipmentState.EMERGENCY_STOP,
        to: EquipmentState.STANDBY,
        event: StateTransitionEvent.EMERGENCY_RESET,
        guard: (ctx) => this.canResetEmergency(ctx),
        action: async (ctx) => {
          await this.resetEmergencyCondition(ctx);
        }
      },

      // 퇴역
      {
        from: [EquipmentState.OFFLINE, EquipmentState.STANDBY],
        to: EquipmentState.DECOMMISSIONED,
        event: StateTransitionEvent.DECOMMISSION,
        guard: (ctx) => this.canDecommission(ctx),
        action: async (ctx) => {
          await this.recordDecommission(ctx);
        }
      },

      // 전원 끄기
      {
        from: EquipmentState.STANDBY,
        to: EquipmentState.OFFLINE,
        event: StateTransitionEvent.POWER_OFF,
        action: async (ctx) => {
          await this.performShutdown(ctx);
        }
      }
    ];
  }

  // 상태 전이 실행
  async transition(
    event: StateTransitionEvent,
    data?: any,
    user?: User
  ): Promise<TransitionResult> {
    const context: TransitionContext = {
      equipmentId: this.equipmentId,
      currentState: this.state,
      event,
      data,
      user,
      timestamp: new Date()
    };

    // 유효한 전이 찾기
    const transition = this.findTransition(event);
    if (!transition) {
      return {
        success: false,
        error: `상태 '${this.state}'에서 이벤트 '${event}'는 허용되지 않습니다`,
        previousState: this.state,
        currentState: this.state
      };
    }

    // 가드 조건 확인
    if (transition.guard && !transition.guard(context)) {
      return {
        success: false,
        error: '전이 조건을 충족하지 않습니다',
        previousState: this.state,
        currentState: this.state
      };
    }

    const previousState = this.state;

    try {
      // 전이 액션 실행
      if (transition.action) {
        await transition.action(context);
      }

      // 상태 변경
      this.state = transition.to;

      // 이력 기록
      const record: TransitionRecord = {
        id: this.generateRecordId(),
        equipmentId: this.equipmentId,
        event,
        fromState: previousState,
        toState: this.state,
        data,
        userId: user?.id,
        timestamp: context.timestamp
      };
      this.history.push(record);

      // 이벤트 발행
      this.emit('stateChanged', {
        equipmentId: this.equipmentId,
        previousState,
        currentState: this.state,
        event,
        timestamp: context.timestamp
      });

      return {
        success: true,
        previousState,
        currentState: this.state
      };
    } catch (error) {
      // 전이 실패 처리
      this.emit('transitionError', {
        equipmentId: this.equipmentId,
        event,
        error
      });

      return {
        success: false,
        error: error instanceof Error ? error.message : '전이 실행 실패',
        previousState,
        currentState: this.state
      };
    }
  }

  // 유효한 전이 찾기
  private findTransition(event: StateTransitionEvent): StateTransition | undefined {
    return this.transitions.find(t => {
      const fromStates = Array.isArray(t.from) ? t.from : [t.from];
      return t.event === event && fromStates.includes(this.state);
    });
  }

  // 현재 상태에서 가능한 이벤트들
  getAvailableEvents(): StateTransitionEvent[] {
    return this.transitions
      .filter(t => {
        const fromStates = Array.isArray(t.from) ? t.from : [t.from];
        return fromStates.includes(this.state);
      })
      .map(t => t.event);
  }

  // 상태 조회
  getCurrentState(): EquipmentState {
    return this.state;
  }

  // 전이 이력 조회
  getHistory(): TransitionRecord[] {
    return [...this.history];
  }

  // 가드 조건 메서드들
  private isInitializationSuccessful(ctx: TransitionContext): boolean {
    return ctx.data?.initializationStatus === 'success';
  }

  private canActivate(ctx: TransitionContext): boolean {
    // 안전 조건 확인
    return ctx.data?.safetyChecks?.allPassed === true;
  }

  private hasMaintenancePermission(ctx: TransitionContext): boolean {
    return ctx.user?.roles?.includes('maintenance-technician') ||
           ctx.user?.roles?.includes('admin');
  }

  private isMaintenanceComplete(ctx: TransitionContext): boolean {
    return ctx.data?.maintenanceComplete === true;
  }

  private hasCalibrationPermission(ctx: TransitionContext): boolean {
    return ctx.user?.roles?.includes('calibration-technician') ||
           ctx.user?.roles?.includes('admin');
  }

  private isCalibrationValid(ctx: TransitionContext): boolean {
    return ctx.data?.calibrationPassed === true;
  }

  private isFaultResolved(ctx: TransitionContext): boolean {
    return ctx.data?.faultCleared === true;
  }

  private canResetEmergency(ctx: TransitionContext): boolean {
    return ctx.data?.emergencyClear === true &&
           ctx.user?.roles?.includes('supervisor');
  }

  private canDecommission(ctx: TransitionContext): boolean {
    return ctx.user?.roles?.includes('admin') &&
           ctx.data?.decommissionApproved === true;
  }

  // 액션 메서드들
  private async runInitializationSequence(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 초기화 시퀀스 시작`);
    // 초기화 로직
  }

  private async startMonitoring(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 모니터링 시작`);
    // 모니터링 시작 로직
  }

  private async stopMonitoring(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 모니터링 중지`);
    // 모니터링 중지 로직
  }

  private async prepareForMaintenance(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 유지보수 준비`);
    // 유지보수 준비 로직
  }

  private async postMaintenanceChecks(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 유지보수 후 점검`);
    // 점검 로직
  }

  private async recordCalibration(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 교정 기록`);
    // 교정 기록 로직
  }

  private async handleFault(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 고장 처리: ${ctx.data?.faultCode}`);
    // 고장 처리 로직
  }

  private async clearFaultCondition(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 고장 상태 해제`);
    // 고장 해제 로직
  }

  private async executeEmergencyStop(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 비상 정지 실행`);
    // 비상 정지 로직
  }

  private async resetEmergencyCondition(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 비상 상태 리셋`);
    // 비상 리셋 로직
  }

  private async recordDecommission(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 퇴역 기록`);
    // 퇴역 기록 로직
  }

  private async performShutdown(ctx: TransitionContext): Promise<void> {
    console.log(`장비 ${ctx.equipmentId} 종료 수행`);
    // 종료 로직
  }

  private generateRecordId(): string {
    return `tr_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 전이 결과 타입
interface TransitionResult {
  success: boolean;
  error?: string;
  previousState: EquipmentState;
  currentState: EquipmentState;
}

// 전이 기록 타입
interface TransitionRecord {
  id: string;
  equipmentId: string;
  event: StateTransitionEvent;
  fromState: EquipmentState;
  toState: EquipmentState;
  data?: any;
  userId?: string;
  timestamp: Date;
}
```

### 5.2.2 장비 명령 실행기

```typescript
// 장비 명령 타입
export enum EquipmentCommand {
  // LN2 탱크 명령
  FILL_TANK = 'fill-tank',
  SET_AUTO_FILL = 'set-auto-fill',
  CHECK_LEVEL = 'check-level',

  // 냉동고 명령
  SET_TEMPERATURE = 'set-temperature',
  START_DEFROST = 'start-defrost',
  STOP_DEFROST = 'stop-defrost',
  ENABLE_CO2_BACKUP = 'enable-co2-backup',

  // 공통 명령
  START = 'start',
  STOP = 'stop',
  RESET = 'reset',
  SELF_TEST = 'self-test',
  CALIBRATE = 'calibrate',
  EMERGENCY_STOP = 'emergency-stop'
}

// 명령 파라미터 스키마
interface CommandParameters {
  [EquipmentCommand.FILL_TANK]: {
    targetLevel: number; // 퍼센트
    source?: 'bulk' | 'dewar';
  };
  [EquipmentCommand.SET_TEMPERATURE]: {
    setpoint: number;
    unit: 'celsius' | 'fahrenheit';
    rampRate?: number; // 도/분
  };
  [EquipmentCommand.START_DEFROST]: {
    duration?: number; // 분
    type?: 'manual' | 'scheduled';
  };
  [EquipmentCommand.CALIBRATE]: {
    sensorId?: string;
    referenceValue?: number;
  };
}

// 명령 실행 결과
interface CommandResult {
  success: boolean;
  commandId: string;
  command: EquipmentCommand;
  startTime: Date;
  endTime?: Date;
  status: 'pending' | 'executing' | 'completed' | 'failed' | 'cancelled';
  result?: any;
  error?: string;
}

// 장비 명령 실행기
export class EquipmentCommandExecutor {
  private pendingCommands: Map<string, CommandExecution> = new Map();
  private commandHandlers: Map<EquipmentCommand, CommandHandler> = new Map();

  constructor(
    private equipmentService: EquipmentService,
    private auditLogger: AuditLogger
  ) {
    this.registerHandlers();
  }

  // 명령 핸들러 등록
  private registerHandlers() {
    // LN2 탱크 명령
    this.commandHandlers.set(EquipmentCommand.FILL_TANK, {
      validate: this.validateFillTank.bind(this),
      execute: this.executeFillTank.bind(this),
      timeout: 30 * 60 * 1000, // 30분
      requiresConfirmation: true
    });

    // 온도 설정 명령
    this.commandHandlers.set(EquipmentCommand.SET_TEMPERATURE, {
      validate: this.validateSetTemperature.bind(this),
      execute: this.executeSetTemperature.bind(this),
      timeout: 5 * 60 * 1000, // 5분
      requiresConfirmation: false
    });

    // 해동 시작 명령
    this.commandHandlers.set(EquipmentCommand.START_DEFROST, {
      validate: this.validateStartDefrost.bind(this),
      execute: this.executeStartDefrost.bind(this),
      timeout: 60 * 60 * 1000, // 1시간
      requiresConfirmation: true,
      preConditions: ['equipment-empty-or-protected', 'no-active-alarms']
    });

    // 비상 정지 명령
    this.commandHandlers.set(EquipmentCommand.EMERGENCY_STOP, {
      validate: () => ({ valid: true }), // 항상 허용
      execute: this.executeEmergencyStop.bind(this),
      timeout: 10 * 1000, // 10초
      requiresConfirmation: false,
      priority: 'critical'
    });

    // 자가 테스트 명령
    this.commandHandlers.set(EquipmentCommand.SELF_TEST, {
      validate: this.validateSelfTest.bind(this),
      execute: this.executeSelfTest.bind(this),
      timeout: 15 * 60 * 1000, // 15분
      requiresConfirmation: false
    });

    // 교정 명령
    this.commandHandlers.set(EquipmentCommand.CALIBRATE, {
      validate: this.validateCalibrate.bind(this),
      execute: this.executeCalibrate.bind(this),
      timeout: 30 * 60 * 1000, // 30분
      requiresConfirmation: true,
      requiredRole: 'calibration-technician'
    });
  }

  // 명령 실행
  async execute(
    equipmentId: string,
    command: EquipmentCommand,
    parameters: any,
    user: User
  ): Promise<CommandResult> {
    const commandId = this.generateCommandId();
    const startTime = new Date();

    // 핸들러 확인
    const handler = this.commandHandlers.get(command);
    if (!handler) {
      return {
        success: false,
        commandId,
        command,
        startTime,
        status: 'failed',
        error: `알 수 없는 명령: ${command}`
      };
    }

    // 권한 확인
    if (handler.requiredRole && !user.roles.includes(handler.requiredRole)) {
      await this.auditLogger.log({
        action: 'COMMAND_DENIED',
        equipmentId,
        command,
        userId: user.id,
        reason: '권한 부족'
      });

      return {
        success: false,
        commandId,
        command,
        startTime,
        status: 'failed',
        error: `이 명령을 실행하려면 '${handler.requiredRole}' 역할이 필요합니다`
      };
    }

    // 장비 상태 확인
    const equipment = await this.equipmentService.findById(equipmentId);
    if (!equipment) {
      return {
        success: false,
        commandId,
        command,
        startTime,
        status: 'failed',
        error: `장비를 찾을 수 없습니다: ${equipmentId}`
      };
    }

    // 명령 유효성 검증
    const validation = await handler.validate(equipment, parameters);
    if (!validation.valid) {
      return {
        success: false,
        commandId,
        command,
        startTime,
        status: 'failed',
        error: validation.error
      };
    }

    // 사전 조건 확인
    if (handler.preConditions) {
      for (const condition of handler.preConditions) {
        const conditionMet = await this.checkPreCondition(equipment, condition);
        if (!conditionMet) {
          return {
            success: false,
            commandId,
            command,
            startTime,
            status: 'failed',
            error: `사전 조건 미충족: ${condition}`
          };
        }
      }
    }

    // 명령 실행 기록
    const execution: CommandExecution = {
      commandId,
      equipmentId,
      command,
      parameters,
      userId: user.id,
      startTime,
      status: 'pending',
      timeout: handler.timeout
    };

    this.pendingCommands.set(commandId, execution);

    // 감사 로그
    await this.auditLogger.log({
      action: 'COMMAND_STARTED',
      equipmentId,
      command,
      commandId,
      parameters,
      userId: user.id
    });

    try {
      // 명령 실행
      execution.status = 'executing';

      const result = await Promise.race([
        handler.execute(equipment, parameters, user),
        this.createTimeout(handler.timeout, commandId)
      ]);

      execution.status = 'completed';
      execution.endTime = new Date();
      execution.result = result;

      // 감사 로그
      await this.auditLogger.log({
        action: 'COMMAND_COMPLETED',
        equipmentId,
        command,
        commandId,
        result,
        userId: user.id,
        duration: execution.endTime.getTime() - startTime.getTime()
      });

      return {
        success: true,
        commandId,
        command,
        startTime,
        endTime: execution.endTime,
        status: 'completed',
        result
      };

    } catch (error) {
      execution.status = 'failed';
      execution.endTime = new Date();
      execution.error = error instanceof Error ? error.message : '명령 실행 실패';

      // 감사 로그
      await this.auditLogger.log({
        action: 'COMMAND_FAILED',
        equipmentId,
        command,
        commandId,
        error: execution.error,
        userId: user.id
      });

      return {
        success: false,
        commandId,
        command,
        startTime,
        endTime: execution.endTime,
        status: 'failed',
        error: execution.error
      };

    } finally {
      this.pendingCommands.delete(commandId);
    }
  }

  // 명령 취소
  async cancel(commandId: string, user: User): Promise<boolean> {
    const execution = this.pendingCommands.get(commandId);
    if (!execution) {
      return false;
    }

    execution.status = 'cancelled';
    execution.endTime = new Date();

    await this.auditLogger.log({
      action: 'COMMAND_CANCELLED',
      equipmentId: execution.equipmentId,
      command: execution.command,
      commandId,
      userId: user.id
    });

    this.pendingCommands.delete(commandId);
    return true;
  }

  // 유효성 검증 메서드들
  private async validateFillTank(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.FILL_TANK]
  ): Promise<ValidationResult> {
    if (params.targetLevel < 0 || params.targetLevel > 100) {
      return { valid: false, error: '목표 수위는 0-100% 범위여야 합니다' };
    }

    if (equipment.category !== 'ln2-tank') {
      return { valid: false, error: '이 명령은 LN2 탱크에만 적용됩니다' };
    }

    return { valid: true };
  }

  private async validateSetTemperature(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.SET_TEMPERATURE]
  ): Promise<ValidationResult> {
    if (equipment.category !== 'mechanical-freezer') {
      return { valid: false, error: '이 명령은 기계식 냉동고에만 적용됩니다' };
    }

    const specs = equipment.specifications as any;
    const { min, max } = specs.temperatureRange;

    if (params.setpoint < min || params.setpoint > max) {
      return {
        valid: false,
        error: `설정 온도는 ${min}°C ~ ${max}°C 범위여야 합니다`
      };
    }

    return { valid: true };
  }

  private async validateStartDefrost(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.START_DEFROST]
  ): Promise<ValidationResult> {
    if (equipment.category !== 'mechanical-freezer') {
      return { valid: false, error: '이 명령은 기계식 냉동고에만 적용됩니다' };
    }

    if (equipment.currentState.defrostActive) {
      return { valid: false, error: '해동이 이미 진행 중입니다' };
    }

    return { valid: true };
  }

  private async validateSelfTest(
    equipment: Equipment,
    params: any
  ): Promise<ValidationResult> {
    if (equipment.status !== 'operational' && equipment.status !== 'standby') {
      return { valid: false, error: '장비가 가동 또는 대기 상태여야 합니다' };
    }

    return { valid: true };
  }

  private async validateCalibrate(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.CALIBRATE]
  ): Promise<ValidationResult> {
    if (equipment.status === 'operational') {
      return { valid: false, error: '교정 전 장비를 대기 상태로 전환해야 합니다' };
    }

    return { valid: true };
  }

  // 명령 실행 메서드들
  private async executeFillTank(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.FILL_TANK],
    user: User
  ): Promise<any> {
    console.log(`LN2 탱크 ${equipment.id} 충전 시작: 목표 ${params.targetLevel}%`);

    // 실제 충전 로직
    // 외부 제어 시스템과 통신

    return {
      previousLevel: equipment.currentState.fillLevel,
      targetLevel: params.targetLevel,
      source: params.source || 'bulk'
    };
  }

  private async executeSetTemperature(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.SET_TEMPERATURE],
    user: User
  ): Promise<any> {
    console.log(`냉동고 ${equipment.id} 온도 설정: ${params.setpoint}°C`);

    // 실제 온도 설정 로직

    return {
      previousSetpoint: equipment.currentState.setpointTemperature,
      newSetpoint: params.setpoint,
      unit: params.unit
    };
  }

  private async executeStartDefrost(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.START_DEFROST],
    user: User
  ): Promise<any> {
    console.log(`냉동고 ${equipment.id} 해동 시작`);

    // 실제 해동 로직

    return {
      startTime: new Date(),
      duration: params.duration || 60,
      type: params.type || 'manual'
    };
  }

  private async executeEmergencyStop(
    equipment: Equipment,
    params: any,
    user: User
  ): Promise<any> {
    console.log(`장비 ${equipment.id} 비상 정지`);

    // 즉시 정지 로직
    // 모든 활성 작업 중단

    return {
      stoppedAt: new Date(),
      previousState: equipment.status
    };
  }

  private async executeSelfTest(
    equipment: Equipment,
    params: any,
    user: User
  ): Promise<any> {
    console.log(`장비 ${equipment.id} 자가 테스트 시작`);

    // 자가 테스트 로직
    const testResults = {
      sensors: 'pass',
      alarms: 'pass',
      communication: 'pass',
      power: 'pass'
    };

    return {
      testTime: new Date(),
      results: testResults,
      passed: Object.values(testResults).every(r => r === 'pass')
    };
  }

  private async executeCalibrate(
    equipment: Equipment,
    params: CommandParameters[EquipmentCommand.CALIBRATE],
    user: User
  ): Promise<any> {
    console.log(`장비 ${equipment.id} 교정 시작`);

    // 교정 로직

    return {
      calibrationTime: new Date(),
      sensorId: params.sensorId,
      referenceValue: params.referenceValue,
      status: 'completed'
    };
  }

  // 사전 조건 확인
  private async checkPreCondition(
    equipment: Equipment,
    condition: string
  ): Promise<boolean> {
    switch (condition) {
      case 'equipment-empty-or-protected':
        // 장비가 비어있거나 검체가 보호되어 있는지 확인
        return true;

      case 'no-active-alarms':
        // 활성 알람이 없는지 확인
        const alarms = await this.equipmentService.getActiveAlarms(equipment.id);
        return alarms.length === 0;

      default:
        return true;
    }
  }

  // 타임아웃 생성
  private createTimeout(ms: number, commandId: string): Promise<never> {
    return new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error(`명령 시간 초과 (${ms}ms): ${commandId}`));
      }, ms);
    });
  }

  private generateCommandId(): string {
    return `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 명령 핸들러 인터페이스
interface CommandHandler {
  validate: (equipment: Equipment, params: any) => Promise<ValidationResult>;
  execute: (equipment: Equipment, params: any, user: User) => Promise<any>;
  timeout: number;
  requiresConfirmation?: boolean;
  preConditions?: string[];
  requiredRole?: string;
  priority?: 'normal' | 'high' | 'critical';
}

// 유효성 검증 결과
interface ValidationResult {
  valid: boolean;
  error?: string;
}

// 명령 실행 정보
interface CommandExecution {
  commandId: string;
  equipmentId: string;
  command: EquipmentCommand;
  parameters: any;
  userId: string;
  startTime: Date;
  endTime?: Date;
  status: 'pending' | 'executing' | 'completed' | 'failed' | 'cancelled';
  result?: any;
  error?: string;
  timeout: number;
}
```

## 5.3 워크플로우 엔진

### 5.3.1 워크플로우 정의 및 실행

```typescript
// 워크플로우 정의
interface WorkflowDefinition {
  id: string;
  name: string;
  description: string;
  version: string;
  category: 'operational' | 'maintenance' | 'quality' | 'emergency';
  triggers: WorkflowTrigger[];
  steps: WorkflowStep[];
  variables: WorkflowVariable[];
}

// 워크플로우 트리거
interface WorkflowTrigger {
  type: 'manual' | 'scheduled' | 'event' | 'condition';
  config: any;
}

// 워크플로우 단계
interface WorkflowStep {
  id: string;
  name: string;
  type: 'task' | 'approval' | 'notification' | 'decision' | 'parallel' | 'wait';
  config: any;
  onSuccess?: string; // 다음 단계 ID
  onFailure?: string; // 실패 시 단계 ID
  timeout?: number;
}

// 워크플로우 엔진
export class WorkflowEngine {
  private definitions: Map<string, WorkflowDefinition> = new Map();
  private instances: Map<string, WorkflowInstance> = new Map();

  constructor(
    private taskService: TaskService,
    private notificationService: NotificationService,
    private auditLogger: AuditLogger
  ) {}

  // 워크플로우 정의 등록
  registerWorkflow(definition: WorkflowDefinition): void {
    this.definitions.set(definition.id, definition);
  }

  // 워크플로우 인스턴스 시작
  async startWorkflow(
    workflowId: string,
    context: WorkflowContext
  ): Promise<WorkflowInstance> {
    const definition = this.definitions.get(workflowId);
    if (!definition) {
      throw new Error(`워크플로우를 찾을 수 없습니다: ${workflowId}`);
    }

    const instance: WorkflowInstance = {
      id: this.generateInstanceId(),
      workflowId,
      status: 'running',
      currentStepId: definition.steps[0]?.id,
      context,
      variables: {},
      stepHistory: [],
      startedAt: new Date()
    };

    this.instances.set(instance.id, instance);

    await this.auditLogger.log({
      action: 'WORKFLOW_STARTED',
      workflowId,
      instanceId: instance.id,
      userId: context.initiatedBy
    });

    // 첫 번째 단계 실행
    await this.executeStep(instance, definition.steps[0]);

    return instance;
  }

  // 단계 실행
  private async executeStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<void> {
    instance.currentStepId = step.id;

    const stepExecution: StepExecution = {
      stepId: step.id,
      status: 'executing',
      startedAt: new Date()
    };
    instance.stepHistory.push(stepExecution);

    try {
      let result: StepResult;

      switch (step.type) {
        case 'task':
          result = await this.executeTaskStep(instance, step);
          break;
        case 'approval':
          result = await this.executeApprovalStep(instance, step);
          break;
        case 'notification':
          result = await this.executeNotificationStep(instance, step);
          break;
        case 'decision':
          result = await this.executeDecisionStep(instance, step);
          break;
        case 'parallel':
          result = await this.executeParallelStep(instance, step);
          break;
        case 'wait':
          result = await this.executeWaitStep(instance, step);
          break;
        default:
          throw new Error(`알 수 없는 단계 유형: ${step.type}`);
      }

      stepExecution.status = result.success ? 'completed' : 'failed';
      stepExecution.completedAt = new Date();
      stepExecution.result = result;

      // 다음 단계 결정
      const nextStepId = result.success ? step.onSuccess : step.onFailure;

      if (nextStepId) {
        const definition = this.definitions.get(instance.workflowId)!;
        const nextStep = definition.steps.find(s => s.id === nextStepId);
        if (nextStep) {
          await this.executeStep(instance, nextStep);
        }
      } else {
        // 워크플로우 완료
        instance.status = result.success ? 'completed' : 'failed';
        instance.completedAt = new Date();

        await this.auditLogger.log({
          action: 'WORKFLOW_COMPLETED',
          workflowId: instance.workflowId,
          instanceId: instance.id,
          status: instance.status
        });
      }

    } catch (error) {
      stepExecution.status = 'failed';
      stepExecution.completedAt = new Date();
      stepExecution.error = error instanceof Error ? error.message : '실행 오류';

      instance.status = 'failed';
      instance.error = stepExecution.error;

      await this.auditLogger.log({
        action: 'WORKFLOW_FAILED',
        workflowId: instance.workflowId,
        instanceId: instance.id,
        stepId: step.id,
        error: stepExecution.error
      });
    }
  }

  // 작업 단계 실행
  private async executeTaskStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { taskType, assignee, instructions } = step.config;

    const task = await this.taskService.create({
      type: taskType,
      workflowInstanceId: instance.id,
      stepId: step.id,
      assignee,
      instructions,
      dueDate: step.timeout
        ? new Date(Date.now() + step.timeout)
        : undefined
    });

    // 작업 완료 대기 (실제로는 이벤트 기반)
    const result = await this.taskService.waitForCompletion(task.id, step.timeout);

    return {
      success: result.status === 'completed',
      data: result
    };
  }

  // 승인 단계 실행
  private async executeApprovalStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { approvers, requiredApprovals, escalationTimeout } = step.config;

    // 승인 요청 생성
    const approvalRequest = await this.taskService.createApprovalRequest({
      workflowInstanceId: instance.id,
      stepId: step.id,
      approvers,
      requiredApprovals: requiredApprovals || 1,
      context: instance.context,
      escalationTimeout
    });

    // 승인자들에게 알림
    for (const approverId of approvers) {
      await this.notificationService.send({
        recipientId: approverId,
        type: 'approval-request',
        title: `승인 요청: ${step.name}`,
        message: step.config.message,
        actionUrl: `/approvals/${approvalRequest.id}`,
        priority: 'high'
      });
    }

    // 승인 결과 대기
    const result = await this.taskService.waitForApproval(
      approvalRequest.id,
      step.timeout
    );

    return {
      success: result.approved,
      data: {
        approvals: result.approvals,
        rejections: result.rejections
      }
    };
  }

  // 알림 단계 실행
  private async executeNotificationStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { recipients, channels, template, variables } = step.config;

    for (const recipient of recipients) {
      for (const channel of channels) {
        await this.notificationService.send({
          recipientId: recipient,
          channel,
          template,
          variables: {
            ...instance.variables,
            ...variables
          }
        });
      }
    }

    return { success: true };
  }

  // 결정 단계 실행
  private async executeDecisionStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { condition, trueStep, falseStep } = step.config;

    // 조건 평가
    const result = this.evaluateCondition(condition, instance.variables);

    return {
      success: true,
      nextStepId: result ? trueStep : falseStep
    };
  }

  // 병렬 단계 실행
  private async executeParallelStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { branches, joinType } = step.config;

    const branchPromises = branches.map(async (branchSteps: WorkflowStep[]) => {
      for (const branchStep of branchSteps) {
        await this.executeStep(instance, branchStep);
      }
    });

    if (joinType === 'all') {
      await Promise.all(branchPromises);
    } else {
      await Promise.race(branchPromises);
    }

    return { success: true };
  }

  // 대기 단계 실행
  private async executeWaitStep(
    instance: WorkflowInstance,
    step: WorkflowStep
  ): Promise<StepResult> {
    const { waitType, duration, eventType } = step.config;

    if (waitType === 'duration') {
      await new Promise(resolve => setTimeout(resolve, duration));
    } else if (waitType === 'event') {
      // 이벤트 대기 로직
      await this.waitForEvent(eventType, instance.id, step.timeout);
    }

    return { success: true };
  }

  // 조건 평가
  private evaluateCondition(condition: string, variables: any): boolean {
    // 간단한 조건 평가 (실제로는 더 복잡한 표현식 엔진 사용)
    try {
      const func = new Function('vars', `with(vars) { return ${condition}; }`);
      return func(variables);
    } catch {
      return false;
    }
  }

  // 이벤트 대기
  private async waitForEvent(
    eventType: string,
    instanceId: string,
    timeout?: number
  ): Promise<any> {
    // 이벤트 대기 로직
    return new Promise((resolve, reject) => {
      const timer = timeout ? setTimeout(() => {
        reject(new Error('이벤트 대기 시간 초과'));
      }, timeout) : null;

      // 이벤트 리스너 등록
      // 이벤트 발생 시 resolve
    });
  }

  private generateInstanceId(): string {
    return `wf_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 워크플로우 인스턴스
interface WorkflowInstance {
  id: string;
  workflowId: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';
  currentStepId?: string;
  context: WorkflowContext;
  variables: Record<string, any>;
  stepHistory: StepExecution[];
  startedAt: Date;
  completedAt?: Date;
  error?: string;
}

// 워크플로우 컨텍스트
interface WorkflowContext {
  facilityId: string;
  equipmentId?: string;
  initiatedBy: string;
  triggerEvent?: any;
  data?: any;
}

// 단계 실행 정보
interface StepExecution {
  stepId: string;
  status: 'pending' | 'executing' | 'completed' | 'failed' | 'skipped';
  startedAt: Date;
  completedAt?: Date;
  result?: StepResult;
  error?: string;
}

// 단계 결과
interface StepResult {
  success: boolean;
  data?: any;
  nextStepId?: string;
}

// 워크플로우 변수
interface WorkflowVariable {
  name: string;
  type: 'string' | 'number' | 'boolean' | 'object' | 'array';
  defaultValue?: any;
  required?: boolean;
}
```

## 5.4 비상 대응 시스템

### 5.4.1 비상 대응 프로토콜

```typescript
// 비상 상황 유형
export enum EmergencyType {
  LN2_LEAK = 'ln2-leak',
  TEMPERATURE_EXCURSION = 'temperature-excursion',
  POWER_FAILURE = 'power-failure',
  FIRE = 'fire',
  EQUIPMENT_FAILURE = 'equipment-failure',
  SECURITY_BREACH = 'security-breach',
  NATURAL_DISASTER = 'natural-disaster',
  CONTAMINATION = 'contamination'
}

// 비상 심각도
export enum EmergencySeverity {
  LEVEL_1 = 'level-1', // 경미
  LEVEL_2 = 'level-2', // 보통
  LEVEL_3 = 'level-3', // 심각
  LEVEL_4 = 'level-4'  // 재난
}

// 비상 프로토콜 정의
interface EmergencyProtocol {
  id: string;
  type: EmergencyType;
  severity: EmergencySeverity;
  name: string;
  description: string;
  automaticActions: AutomaticAction[];
  manualActions: ManualAction[];
  notifications: NotificationConfig[];
  escalation: EscalationConfig;
  recoveryProcedure: RecoveryProcedure;
}

// 자동 조치
interface AutomaticAction {
  id: string;
  name: string;
  type: 'equipment-command' | 'system-action' | 'notification';
  config: any;
  condition?: string;
  priority: number;
}

// 수동 조치
interface ManualAction {
  id: string;
  name: string;
  instructions: string;
  responsible: string;
  timeLimit?: number;
  required: boolean;
}

// 비상 대응 시스템
export class EmergencyResponseSystem {
  private protocols: Map<EmergencyType, EmergencyProtocol[]> = new Map();
  private activeEmergencies: Map<string, ActiveEmergency> = new Map();

  constructor(
    private equipmentService: EquipmentService,
    private notificationService: NotificationService,
    private auditLogger: AuditLogger,
    private websocketServer: any
  ) {
    this.loadProtocols();
  }

  // 프로토콜 로드
  private async loadProtocols(): Promise<void> {
    // LN2 누출 프로토콜
    this.registerProtocol({
      id: 'proto-ln2-leak',
      type: EmergencyType.LN2_LEAK,
      severity: EmergencySeverity.LEVEL_3,
      name: '액체질소 누출 대응',
      description: '액체질소 누출 감지 시 대응 프로토콜',
      automaticActions: [
        {
          id: 'act-1',
          name: '환기 시스템 가동',
          type: 'equipment-command',
          config: {
            targetSystem: 'hvac',
            command: 'emergency-ventilation',
            parameters: { mode: 'maximum' }
          },
          priority: 1
        },
        {
          id: 'act-2',
          name: '구역 출입 차단',
          type: 'system-action',
          config: {
            action: 'lockdown-zone',
            scope: 'affected-area'
          },
          priority: 2
        },
        {
          id: 'act-3',
          name: '산소 모니터링 강화',
          type: 'equipment-command',
          config: {
            targetSystem: 'oxygen-monitor',
            command: 'high-frequency-mode',
            parameters: { interval: 10 }
          },
          priority: 3
        }
      ],
      manualActions: [
        {
          id: 'manual-1',
          name: '구역 대피',
          instructions: '영향 받는 구역의 모든 인원을 즉시 대피시키세요',
          responsible: 'facility-manager',
          timeLimit: 300,
          required: true
        },
        {
          id: 'manual-2',
          name: '누출원 확인',
          instructions: '안전 장비 착용 후 누출 원인을 파악하세요',
          responsible: 'safety-officer',
          required: true
        },
        {
          id: 'manual-3',
          name: '검체 상태 확인',
          instructions: '영향 받을 수 있는 검체의 상태를 확인하세요',
          responsible: 'lab-technician',
          required: false
        }
      ],
      notifications: [
        {
          recipientGroups: ['safety-team', 'facility-managers'],
          channels: ['sms', 'push', 'email'],
          priority: 'critical',
          template: 'emergency-ln2-leak'
        },
        {
          recipientGroups: ['all-staff'],
          channels: ['pa-system', 'push'],
          priority: 'high',
          template: 'evacuation-notice'
        }
      ],
      escalation: {
        levels: [
          {
            level: 1,
            delay: 0,
            contacts: ['on-duty-supervisor']
          },
          {
            level: 2,
            delay: 300,
            contacts: ['facility-manager', 'safety-manager']
          },
          {
            level: 3,
            delay: 600,
            contacts: ['director', 'emergency-services']
          }
        ]
      },
      recoveryProcedure: {
        steps: [
          '산소 농도 정상 확인 (19.5% 이상)',
          '누출 원인 제거 확인',
          '환기 시스템 정상 작동 확인',
          '구역 안전 검사 완료',
          '복귀 승인 획득'
        ],
        requiredApprovals: ['safety-manager', 'facility-manager']
      }
    });

    // 온도 이탈 프로토콜
    this.registerProtocol({
      id: 'proto-temp-excursion',
      type: EmergencyType.TEMPERATURE_EXCURSION,
      severity: EmergencySeverity.LEVEL_2,
      name: '온도 이탈 대응',
      description: '보관 장비 온도 이탈 시 대응 프로토콜',
      automaticActions: [
        {
          id: 'act-1',
          name: '백업 냉각 활성화',
          type: 'equipment-command',
          config: {
            command: 'activate-backup-cooling'
          },
          condition: 'temperature > criticalHigh',
          priority: 1
        },
        {
          id: 'act-2',
          name: '알람 강화',
          type: 'system-action',
          config: {
            action: 'increase-monitoring-frequency',
            interval: 30
          },
          priority: 2
        }
      ],
      manualActions: [
        {
          id: 'manual-1',
          name: '장비 점검',
          instructions: '영향 받는 장비의 상태를 점검하세요',
          responsible: 'equipment-operator',
          timeLimit: 600,
          required: true
        },
        {
          id: 'manual-2',
          name: '검체 이동 평가',
          instructions: '검체 이동 필요성을 평가하세요',
          responsible: 'lab-supervisor',
          required: true
        }
      ],
      notifications: [
        {
          recipientGroups: ['equipment-team', 'lab-supervisors'],
          channels: ['sms', 'push'],
          priority: 'high',
          template: 'temperature-excursion'
        }
      ],
      escalation: {
        levels: [
          {
            level: 1,
            delay: 0,
            contacts: ['on-duty-technician']
          },
          {
            level: 2,
            delay: 600,
            contacts: ['equipment-manager']
          },
          {
            level: 3,
            delay: 1200,
            contacts: ['facility-manager', 'quality-manager']
          }
        ]
      },
      recoveryProcedure: {
        steps: [
          '온도 정상 범위 복귀 확인',
          '장비 정상 작동 확인',
          '검체 영향 평가 완료',
          '사고 보고서 작성'
        ],
        requiredApprovals: ['quality-manager']
      }
    });

    // 정전 프로토콜
    this.registerProtocol({
      id: 'proto-power-failure',
      type: EmergencyType.POWER_FAILURE,
      severity: EmergencySeverity.LEVEL_3,
      name: '정전 대응',
      description: '전력 공급 중단 시 대응 프로토콜',
      automaticActions: [
        {
          id: 'act-1',
          name: 'UPS 상태 확인',
          type: 'system-action',
          config: {
            action: 'check-ups-status'
          },
          priority: 1
        },
        {
          id: 'act-2',
          name: '비상 발전기 가동',
          type: 'equipment-command',
          config: {
            targetSystem: 'generator',
            command: 'start',
            parameters: { mode: 'automatic' }
          },
          priority: 2
        },
        {
          id: 'act-3',
          name: '비필수 부하 차단',
          type: 'system-action',
          config: {
            action: 'shed-non-critical-loads'
          },
          priority: 3
        }
      ],
      manualActions: [
        {
          id: 'manual-1',
          name: '장비 상태 확인',
          instructions: '모든 중요 장비의 전원 상태를 확인하세요',
          responsible: 'facility-operator',
          timeLimit: 300,
          required: true
        },
        {
          id: 'manual-2',
          name: '연료 잔량 확인',
          instructions: '발전기 연료 잔량을 확인하세요',
          responsible: 'facility-operator',
          required: true
        }
      ],
      notifications: [
        {
          recipientGroups: ['facility-team', 'management'],
          channels: ['sms', 'push', 'email'],
          priority: 'critical',
          template: 'power-failure'
        }
      ],
      escalation: {
        levels: [
          {
            level: 1,
            delay: 0,
            contacts: ['on-duty-engineer']
          },
          {
            level: 2,
            delay: 300,
            contacts: ['facility-manager']
          },
          {
            level: 3,
            delay: 900,
            contacts: ['director', 'utility-company']
          }
        ]
      },
      recoveryProcedure: {
        steps: [
          '주 전원 복구 확인',
          '모든 장비 정상 작동 확인',
          '데이터 무결성 검증',
          '발전기 정상 대기 상태 전환'
        ],
        requiredApprovals: ['facility-manager']
      }
    });
  }

  // 프로토콜 등록
  private registerProtocol(protocol: EmergencyProtocol): void {
    const existing = this.protocols.get(protocol.type) || [];
    existing.push(protocol);
    this.protocols.set(protocol.type, existing);
  }

  // 비상 상황 감지 및 대응
  async triggerEmergency(
    type: EmergencyType,
    context: EmergencyContext
  ): Promise<ActiveEmergency> {
    // 적용 가능한 프로토콜 찾기
    const protocols = this.protocols.get(type) || [];
    const applicableProtocol = protocols.find(p =>
      this.evaluateSeverity(context) >= this.severityToNumber(p.severity)
    );

    if (!applicableProtocol) {
      throw new Error(`적용 가능한 비상 프로토콜이 없습니다: ${type}`);
    }

    const emergency: ActiveEmergency = {
      id: this.generateEmergencyId(),
      protocolId: applicableProtocol.id,
      type,
      severity: this.evaluateSeverity(context) as EmergencySeverity,
      status: 'active',
      context,
      triggeredAt: new Date(),
      automaticActionsCompleted: [],
      manualActionsStatus: {},
      escalationLevel: 1
    };

    this.activeEmergencies.set(emergency.id, emergency);

    // 감사 로그
    await this.auditLogger.log({
      action: 'EMERGENCY_TRIGGERED',
      emergencyId: emergency.id,
      type,
      severity: emergency.severity,
      context
    });

    // 자동 조치 실행
    await this.executeAutomaticActions(emergency, applicableProtocol);

    // 알림 발송
    await this.sendNotifications(emergency, applicableProtocol);

    // 에스컬레이션 타이머 시작
    this.startEscalationTimer(emergency, applicableProtocol);

    // 실시간 브로드캐스트
    this.websocketServer.broadcast(`emergency.${context.facilityId}`, {
      type: 'emergency-triggered',
      emergency
    });

    return emergency;
  }

  // 자동 조치 실행
  private async executeAutomaticActions(
    emergency: ActiveEmergency,
    protocol: EmergencyProtocol
  ): Promise<void> {
    const sortedActions = [...protocol.automaticActions]
      .sort((a, b) => a.priority - b.priority);

    for (const action of sortedActions) {
      try {
        // 조건 확인
        if (action.condition && !this.evaluateCondition(action.condition, emergency.context)) {
          continue;
        }

        switch (action.type) {
          case 'equipment-command':
            await this.equipmentService.executeCommand(
              action.config.targetSystem || emergency.context.equipmentId,
              {
                command: action.config.command,
                parameters: action.config.parameters,
                executedBy: 'emergency-system'
              }
            );
            break;

          case 'system-action':
            await this.executeSystemAction(action.config, emergency);
            break;

          case 'notification':
            // 별도 알림 처리
            break;
        }

        emergency.automaticActionsCompleted.push({
          actionId: action.id,
          completedAt: new Date(),
          success: true
        });

      } catch (error) {
        emergency.automaticActionsCompleted.push({
          actionId: action.id,
          completedAt: new Date(),
          success: false,
          error: error instanceof Error ? error.message : '실행 오류'
        });

        await this.auditLogger.log({
          action: 'EMERGENCY_ACTION_FAILED',
          emergencyId: emergency.id,
          actionId: action.id,
          error
        });
      }
    }
  }

  // 시스템 조치 실행
  private async executeSystemAction(
    config: any,
    emergency: ActiveEmergency
  ): Promise<void> {
    switch (config.action) {
      case 'lockdown-zone':
        // 구역 봉쇄 로직
        console.log(`구역 봉쇄: ${config.scope}`);
        break;

      case 'increase-monitoring-frequency':
        // 모니터링 빈도 증가
        console.log(`모니터링 빈도 증가: ${config.interval}초`);
        break;

      case 'check-ups-status':
        // UPS 상태 확인
        console.log('UPS 상태 확인');
        break;

      case 'shed-non-critical-loads':
        // 비필수 부하 차단
        console.log('비필수 부하 차단');
        break;
    }
  }

  // 알림 발송
  private async sendNotifications(
    emergency: ActiveEmergency,
    protocol: EmergencyProtocol
  ): Promise<void> {
    for (const notificationConfig of protocol.notifications) {
      await this.notificationService.sendToGroups({
        groups: notificationConfig.recipientGroups,
        channels: notificationConfig.channels,
        priority: notificationConfig.priority,
        template: notificationConfig.template,
        variables: {
          emergencyId: emergency.id,
          type: emergency.type,
          severity: emergency.severity,
          ...emergency.context
        }
      });
    }
  }

  // 에스컬레이션 타이머
  private startEscalationTimer(
    emergency: ActiveEmergency,
    protocol: EmergencyProtocol
  ): void {
    const escalate = async () => {
      const currentLevel = emergency.escalationLevel;
      const nextLevel = protocol.escalation.levels.find(
        l => l.level === currentLevel + 1
      );

      if (nextLevel && emergency.status === 'active') {
        emergency.escalationLevel = nextLevel.level;

        // 에스컬레이션 알림
        await this.notificationService.sendToUsers({
          userIds: nextLevel.contacts,
          channels: ['sms', 'phone'],
          priority: 'critical',
          template: 'emergency-escalation',
          variables: {
            emergencyId: emergency.id,
            level: nextLevel.level,
            ...emergency.context
          }
        });

        await this.auditLogger.log({
          action: 'EMERGENCY_ESCALATED',
          emergencyId: emergency.id,
          level: nextLevel.level
        });

        // 다음 레벨 타이머
        const nextNextLevel = protocol.escalation.levels.find(
          l => l.level === nextLevel.level + 1
        );

        if (nextNextLevel) {
          setTimeout(escalate, nextNextLevel.delay * 1000);
        }
      }
    };

    const firstEscalation = protocol.escalation.levels.find(l => l.level === 2);
    if (firstEscalation) {
      setTimeout(escalate, firstEscalation.delay * 1000);
    }
  }

  // 비상 상황 해제
  async resolveEmergency(
    emergencyId: string,
    resolution: EmergencyResolution
  ): Promise<void> {
    const emergency = this.activeEmergencies.get(emergencyId);
    if (!emergency) {
      throw new Error(`활성 비상 상황을 찾을 수 없습니다: ${emergencyId}`);
    }

    emergency.status = 'resolved';
    emergency.resolvedAt = new Date();
    emergency.resolution = resolution;

    await this.auditLogger.log({
      action: 'EMERGENCY_RESOLVED',
      emergencyId,
      resolution,
      resolvedBy: resolution.resolvedBy
    });

    // 해제 알림
    await this.notificationService.sendToGroups({
      groups: ['all-notified'],
      channels: ['push', 'email'],
      priority: 'high',
      template: 'emergency-resolved',
      variables: {
        emergencyId,
        type: emergency.type,
        resolution: resolution.summary
      }
    });

    // 실시간 브로드캐스트
    this.websocketServer.broadcast(`emergency.${emergency.context.facilityId}`, {
      type: 'emergency-resolved',
      emergency
    });
  }

  // 심각도 평가
  private evaluateSeverity(context: EmergencyContext): EmergencySeverity {
    // 심각도 평가 로직
    if (context.specimenAtRisk > 1000) return EmergencySeverity.LEVEL_4;
    if (context.specimenAtRisk > 100) return EmergencySeverity.LEVEL_3;
    if (context.deviation > 50) return EmergencySeverity.LEVEL_3;
    if (context.deviation > 20) return EmergencySeverity.LEVEL_2;
    return EmergencySeverity.LEVEL_1;
  }

  private severityToNumber(severity: EmergencySeverity): number {
    const map: Record<EmergencySeverity, number> = {
      [EmergencySeverity.LEVEL_1]: 1,
      [EmergencySeverity.LEVEL_2]: 2,
      [EmergencySeverity.LEVEL_3]: 3,
      [EmergencySeverity.LEVEL_4]: 4
    };
    return map[severity];
  }

  private evaluateCondition(condition: string, context: any): boolean {
    try {
      const func = new Function('ctx', `with(ctx) { return ${condition}; }`);
      return func(context);
    } catch {
      return false;
    }
  }

  private generateEmergencyId(): string {
    return `emg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 비상 상황 컨텍스트
interface EmergencyContext {
  facilityId: string;
  equipmentId?: string;
  zoneId?: string;
  triggeredBy?: string;
  deviation?: number;
  specimenAtRisk?: number;
  additionalData?: any;
}

// 활성 비상 상황
interface ActiveEmergency {
  id: string;
  protocolId: string;
  type: EmergencyType;
  severity: EmergencySeverity;
  status: 'active' | 'resolved' | 'escalated';
  context: EmergencyContext;
  triggeredAt: Date;
  resolvedAt?: Date;
  resolution?: EmergencyResolution;
  automaticActionsCompleted: any[];
  manualActionsStatus: Record<string, any>;
  escalationLevel: number;
}

// 비상 해제 정보
interface EmergencyResolution {
  resolvedBy: string;
  summary: string;
  rootCause?: string;
  preventiveActions?: string[];
  documentation?: string[];
}
```

## 5.5 요약

이 장에서는 극저온 시설의 제어 프로토콜을 상세히 다루었습니다:

| 구성 요소 | 목적 | 주요 기능 |
|----------|------|----------|
| 장비 상태 기계 | 장비 상태 관리 | 상태 전이, 가드 조건, 액션 실행 |
| 명령 실행기 | 장비 제어 | 명령 검증, 실행, 타임아웃 관리 |
| 워크플로우 엔진 | 프로세스 자동화 | 작업, 승인, 알림, 분기 처리 |
| 비상 대응 시스템 | 위기 관리 | 자동 대응, 에스컬레이션, 복구 |

핵심 안전 기능:
- 다중 레벨 인터락
- 권한 기반 명령 실행
- 완전한 감사 로깅
- 자동 에스컬레이션

다음 장에서는 외부 시스템과의 통합에 대해 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
