# 6장: 청소 로봇 통합

## 스마트 빌딩 및 생태계 통합

### 6.1 통합 아키텍처 개요

WIA-CLEANING-ROBOT 표준은 자율 청소 로봇을 스마트 빌딩 시스템, 시설 관리 플랫폼 및 소비자 스마트 홈 생태계와 연결하기 위한 종합 통합 패턴을 정의합니다.

```typescript
// 통합 아키텍처 정의
interface CleaningRobotIntegrationArchitecture {
  version: '1.0.0';

  integrationLayers: {
    deviceLayer: {
      description: '직접 로봇 연결';
      protocols: ['로컬 API', 'Bluetooth', 'Wi-Fi Direct'];
      capabilities: ['직접 제어', '로컬 스토리지', '오프라인 작동'];
    };
    gatewayLayer: {
      description: '허브 및 게이트웨이 통합';
      protocols: ['Matter', 'Zigbee', 'Z-Wave', 'Thread'];
      capabilities: ['프로토콜 브리징', '로컬 자동화', '다중 장치 조정'];
    };
    cloudLayer: {
      description: '클라우드 플랫폼 통합';
      protocols: ['REST API', 'WebSocket', 'MQTT', 'GraphQL'];
      capabilities: ['원격 접근', '분석', 'AI 서비스', '다중 사이트 관리'];
    };
    enterpriseLayer: {
      description: '기업 시스템 통합';
      protocols: ['SOAP', 'REST', '메시지 큐', 'EDI'];
      capabilities: ['BMS 통합', 'CAFM 시스템', 'ERP 연결'];
    };
  };

  integrationPatterns: {
    smartHome: '소비자 음성 어시스턴트 및 앱 통합';
    facilityManagement: '상업용 빌딩 관리 통합';
    fleetManagement: '다중 로봇 조정 및 오케스트레이션';
    iotPlatform: '일반 IoT 플랫폼 연결';
  };
}

// 통합 커넥터 인터페이스
interface IntegrationConnector {
  type: ConnectorType;
  status: ConnectionStatus;
  capabilities: ConnectorCapabilities;

  connect(): Promise<ConnectionResult>;
  disconnect(): Promise<void>;
  sendCommand(command: RobotCommand): Promise<CommandResult>;
  subscribeToEvents(handler: EventHandler): Subscription;
}
```

### 6.2 스마트 홈 통합

```typescript
// 스마트 홈 통합 플랫폼
interface SmartHomeIntegration {
  platforms: {
    googleHome: GoogleHomeIntegration;
    amazonAlexa: AlexaIntegration;
    appleHomeKit: HomeKitIntegration;
    samsungSmartThings: SmartThingsIntegration;
    matter: MatterIntegration;
  };

  commonCapabilities: {
    voiceControl: VoiceControlCapabilities;
    appControl: AppControlCapabilities;
    automation: AutomationCapabilities;
    presence: PresenceDetection;
  };
}

// Google Home 통합
class GoogleHomeConnector implements IntegrationConnector {
  type: ConnectorType = 'SMART_HOME';

  private config: GoogleHomeConfig;
  private oauth2Client: OAuth2Client;
  private robotService: RobotService;

  async handleSync(userId: string): Promise<SyncResponse> {
    const robots = await this.robotService.getUserRobots(userId);

    return {
      requestId: generateId(),
      payload: {
        agentUserId: userId,
        devices: robots.map(robot => ({
          id: robot.id,
          type: 'action.devices.types.VACUUM',
          traits: [
            'action.devices.traits.OnOff',
            'action.devices.traits.StartStop',
            'action.devices.traits.Dock',
            'action.devices.traits.EnergyStorage',
            'action.devices.traits.Locator',
            'action.devices.traits.RunCycle'
          ],
          name: {
            name: robot.nickname || robot.model.name,
            defaultNames: [robot.model.name],
            nicknames: robot.nickname ? [robot.nickname] : []
          },
          willReportState: true,
          roomHint: robot.configuration.defaultRoom,
          deviceInfo: {
            manufacturer: robot.manufacturer,
            model: robot.model.name,
            hwVersion: robot.hardwareRevision,
            swVersion: robot.firmwareVersion.toString()
          }
        }))
      }
    };
  }

  async handleExecute(
    userId: string,
    commands: ExecuteCommand[]
  ): Promise<ExecuteResponse> {
    const results: CommandResult[] = [];

    for (const command of commands) {
      for (const device of command.devices) {
        try {
          const result = await this.executeCommand(
            device.id,
            command.execution[0]
          );
          results.push({
            ids: [device.id],
            status: 'SUCCESS',
            states: result.states
          });
        } catch (error) {
          results.push({
            ids: [device.id],
            status: 'ERROR',
            errorCode: this.mapErrorCode(error)
          });
        }
      }
    }

    return {
      requestId: generateId(),
      payload: { commands: results }
    };
  }

  private async executeCommand(
    robotId: string,
    execution: Execution
  ): Promise<ExecutionResult> {
    switch (execution.command) {
      case 'action.devices.commands.OnOff':
        if (execution.params.on) {
          await this.robotService.startCleaning(robotId, {});
        } else {
          await this.robotService.stopCleaning(robotId);
        }
        return { states: { on: execution.params.on } };

      case 'action.devices.commands.StartStop':
        if (execution.params.start) {
          if (execution.params.zone) {
            await this.robotService.cleanRooms(robotId, {
              rooms: [execution.params.zone]
            });
          } else {
            await this.robotService.startCleaning(robotId, {});
          }
        } else {
          await this.robotService.stopCleaning(robotId);
        }
        return { states: { isRunning: execution.params.start } };

      case 'action.devices.commands.Dock':
        await this.robotService.returnToDock(robotId);
        return { states: { isDocked: true } };

      default:
        throw new Error(`알 수 없는 명령: ${execution.command}`);
    }
  }
}

// Amazon Alexa 통합
class AlexaConnector implements IntegrationConnector {
  type: ConnectorType = 'SMART_HOME';

  private config: AlexaConfig;
  private robotService: RobotService;

  async handleDiscovery(userId: string): Promise<DiscoveryResponse> {
    const robots = await this.robotService.getUserRobots(userId);

    return {
      event: {
        header: {
          namespace: 'Alexa.Discovery',
          name: 'Discover.Response',
          payloadVersion: '3',
          messageId: generateId()
        },
        payload: {
          endpoints: robots.map(robot => ({
            endpointId: robot.id,
            manufacturerName: robot.manufacturer,
            friendlyName: robot.nickname || robot.model.name,
            description: `${robot.manufacturer} ${robot.model.name} 청소 로봇`,
            displayCategories: ['VACUUM_CLEANER'],
            capabilities: [
              {
                type: 'AlexaInterface',
                interface: 'Alexa.PowerController',
                version: '3',
                properties: {
                  supported: [{ name: 'powerState' }],
                  proactivelyReported: true,
                  retrievable: true
                }
              }
            ]
          }))
        }
      }
    };
  }
}
```

### 6.3 빌딩 관리 시스템 통합

```typescript
// 빌딩 관리 시스템 (BMS) 통합
interface BMSIntegration {
  protocols: {
    bacnet: BACnetIntegration;
    modbus: ModbusIntegration;
    knx: KNXIntegration;
    lonworks: LonWorksIntegration;
  };
}

// BACnet 통합
class BACnetConnector implements IntegrationConnector {
  type: ConnectorType = 'BUILDING_MANAGEMENT';

  private bacnetClient: BACnetClient;
  private deviceInstance: number;
  private robotService: RobotService;

  async registerDevice(robot: CleaningRobot): Promise<void> {
    // BACnet 장치 객체 생성
    const device = new BACnetDevice({
      objectIdentifier: {
        type: ObjectType.DEVICE,
        instance: this.deviceInstance++
      },
      objectName: `CleaningRobot_${robot.serialNumber}`,
      description: `${robot.manufacturer} ${robot.model.name}`,
      vendorName: robot.manufacturer,
      modelName: robot.model.name,
      firmwareRevision: robot.firmwareVersion.toString()
    });

    // 상태 객체 추가
    device.addObject(new BACnetObject({
      objectIdentifier: { type: ObjectType.BINARY_INPUT, instance: 1 },
      objectName: 'Cleaning_Active',
      presentValue: false,
      description: '로봇이 적극적으로 청소 중'
    }));

    device.addObject(new BACnetObject({
      objectIdentifier: { type: ObjectType.ANALOG_INPUT, instance: 1 },
      objectName: 'Battery_Level',
      presentValue: 100,
      units: EngineeringUnits.PERCENT,
      description: '배터리 충전 수준'
    }));

    // 제어 객체 추가
    device.addObject(new BACnetObject({
      objectIdentifier: { type: ObjectType.BINARY_OUTPUT, instance: 1 },
      objectName: 'Start_Cleaning',
      presentValue: false,
      description: '청소 시작 명령'
    }));

    await this.bacnetClient.registerDevice(device);
  }
}

// CAFM (컴퓨터 지원 시설 관리) 통합
class CAFMConnector implements IntegrationConnector {
  type: ConnectorType = 'BUILDING_MANAGEMENT';

  private apiClient: HttpClient;
  private robotService: RobotService;

  async syncWithCAFM(
    facility: Facility,
    robots: CleaningRobot[]
  ): Promise<void> {
    // 자산 동기화
    for (const robot of robots) {
      await this.syncAsset(facility.id, robot);
    }

    // 공간/구역 동기화
    for (const robot of robots) {
      await this.syncSpaces(facility.id, robot);
    }

    // 작업 주문 통합 설정
    await this.setupWorkOrderIntegration(facility.id, robots);

    // 청소 로그 통합 설정
    await this.setupCleaningLogIntegration(facility.id, robots);
  }

  private async setupWorkOrderIntegration(
    facilityId: string,
    robots: CleaningRobot[]
  ): Promise<void> {
    // 유지 관리 이벤트 구독
    for (const robot of robots) {
      this.robotService.subscribeToEvents(robot.id, async (event) => {
        if (event.type === 'MAINTENANCE_REQUIRED') {
          // CAFM에 작업 주문 생성
          await this.createWorkOrder({
            facilityId,
            assetId: `ROBOT_${robot.serialNumber}`,
            type: 'PREVENTIVE_MAINTENANCE',
            priority: event.data.priority,
            description: event.data.description,
            dueDate: event.data.suggestedDate
          });
        }
      });
    }
  }
}
```

### 6.4 플릿 관리 통합

```typescript
// 플릿 관리 플랫폼 통합
interface FleetManagementIntegration {
  capabilities: {
    multiRobotCoordination: boolean;
    centralizedScheduling: boolean;
    loadBalancing: boolean;
    realTimeMonitoring: boolean;
    analyticsAndReporting: boolean;
  };

  features: {
    zoneAssignment: ZoneAssignmentStrategy;
    taskDistribution: TaskDistributionStrategy;
    conflictResolution: ConflictResolutionStrategy;
    energyManagement: EnergyManagementStrategy;
  };
}

// 플릿 관리 서비스
class FleetManagementService {
  private robots: Map<string, ManagedRobot>;
  private zoneAssignments: Map<string, string[]>;
  private taskQueue: PriorityQueue<FleetTask>;
  private scheduler: FleetScheduler;
  private coordinator: RobotCoordinator;

  async scheduleFleetCleaning(
    request: FleetCleaningRequest
  ): Promise<FleetCleaningPlan> {
    // 청소 요구 사항 분석
    const requirements = this.analyzeCleaningRequirements(request);

    // 사용 가능한 로봇 가져오기
    const availableRobots = this.getAvailableRobots(request.timeWindow);

    // 구역 할당 최적화
    const assignments = await this.optimizeAssignments(
      requirements,
      availableRobots
    );

    // 일정 생성
    const schedule = await this.scheduler.generateSchedule(assignments);

    // 플릿 청소 계획 생성
    const plan: FleetCleaningPlan = {
      id: generateId(),
      request,
      assignments,
      schedule,
      estimatedCompletion: this.calculateCompletion(schedule),
      estimatedCoverage: this.calculateExpectedCoverage(assignments)
    };

    // 작업 대기열에 추가
    for (const assignment of assignments) {
      await this.queueTask(assignment);
    }

    return plan;
  }

  private async optimizeAssignments(
    requirements: CleaningRequirements,
    robots: ManagedRobot[]
  ): Promise<RobotAssignment[]> {
    // 최적화 모델 구축
    const model = new AssignmentOptimizer();

    // 구역을 작업으로 추가
    for (const zone of requirements.zones) {
      model.addTask({
        id: zone.id,
        location: zone.centroid,
        area: zone.area,
        estimatedTime: this.estimateCleaningTime(zone),
        priority: zone.priority
      });
    }

    // 로봇을 리소스로 추가
    for (const robot of robots) {
      model.addResource({
        id: robot.id,
        location: robot.status.pose?.position || robot.configuration.dockLocation,
        batteryLevel: robot.status.power.batteryLevel,
        capabilities: robot.capabilities
      });
    }

    // 할당 문제 해결
    const solution = await model.solve({
      objective: 'MINIMIZE_TOTAL_TIME',
      constraints: {
        maxTasksPerRobot: requirements.maxTasksPerRobot,
        minBatteryForTask: 20,
        respectZoneAssignments: true
      }
    });

    return solution.assignments;
  }
}

// 로봇 코디네이터
class RobotCoordinator {
  private pathPlanner: MultiRobotPathPlanner;
  private communicator: RobotCommunicator;

  async checkPathConflict(
    robot1: ManagedRobot,
    robot2: ManagedRobot
  ): Promise<PathConflict | null> {
    // 예측 경로 가져오기
    const path1 = await this.predictPath(robot1);
    const path2 = await this.predictPath(robot2);

    // 교차 확인
    const intersection = this.findPathIntersection(path1, path2);
    if (!intersection) return null;

    // 타이밍 확인
    const arrival1 = this.estimateArrivalTime(robot1, intersection.point);
    const arrival2 = this.estimateArrivalTime(robot2, intersection.point);

    const timeDiff = Math.abs(arrival1 - arrival2);
    if (timeDiff < 5000) {  // 5초 이내
      return {
        location: intersection.point,
        time: Math.min(arrival1, arrival2),
        robot1Path: path1,
        robot2Path: path2
      };
    }

    return null;
  }

  async commandYield(
    robotId: string,
    yieldCommand: YieldCommand
  ): Promise<void> {
    await this.communicator.sendCommand(robotId, {
      type: 'YIELD',
      ...yieldCommand
    });
  }
}
```

### 6.5 IoT 플랫폼 통합

```typescript
// IoT 플랫폼 통합
class IoTPlatformConnector implements IntegrationConnector {
  type: ConnectorType = 'IOT_PLATFORM';

  private mqttClient: MQTTClient;
  private config: IoTPlatformConfig;
  private robotService: RobotService;

  async connect(): Promise<ConnectionResult> {
    await this.mqttClient.connect({
      broker: this.config.brokerUrl,
      clientId: `wia-cleaning-robot-${generateId()}`,
      username: this.config.username,
      password: this.config.password,
      tls: this.config.useTLS
    });

    // 명령 토픽 구독
    await this.subscribeToCommands();

    return { success: true };
  }

  async registerRobot(robot: CleaningRobot): Promise<void> {
    // 장치 등록 게시
    await this.mqttClient.publish(
      `${this.config.topicPrefix}/devices/register`,
      JSON.stringify({
        deviceId: robot.id,
        deviceType: 'CLEANING_ROBOT',
        manufacturer: robot.manufacturer,
        model: robot.model.name,
        capabilities: robot.capabilities
      })
    );

    // 텔레메트리 게시 시작
    this.startTelemetryPublishing(robot);
  }

  private startTelemetryPublishing(robot: CleaningRobot): void {
    // 정기적으로 상태 게시
    setInterval(async () => {
      const state = await this.robotService.getRobotState(robot.id);

      await this.mqttClient.publish(
        `${this.config.topicPrefix}/devices/${robot.id}/telemetry`,
        JSON.stringify({
          timestamp: new Date().toISOString(),
          state: state.operatingState.state,
          battery: state.power.batteryLevel,
          position: state.pose,
          cleaning: {
            vacuum: state.cleaningSystem.vacuum.active,
            mop: state.cleaningSystem.mop.active,
            dustbin: state.cleaningSystem.dustbin.level,
            waterTank: state.cleaningSystem.waterTank.level
          }
        })
      );
    }, this.config.telemetryInterval);

    // 로봇 이벤트 구독
    this.robotService.subscribeToEvents(robot.id, async (event) => {
      await this.mqttClient.publish(
        `${this.config.topicPrefix}/devices/${robot.id}/events`,
        JSON.stringify({
          timestamp: event.timestamp,
          type: event.type,
          data: event.data
        })
      );
    });
  }
}
```

---

**WIA-CLEANING-ROBOT 통합**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
