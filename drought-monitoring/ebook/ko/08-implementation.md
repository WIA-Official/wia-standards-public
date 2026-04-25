# 제8장: 구현 가이드

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:

1. WIA 가뭄 모니터링 시스템의 단계별 구축 로드맵 수립
2. 클라우드 및 온프레미스 인프라 아키텍처 설계
3. 관측 장비 및 센서 네트워크 구축
4. 시스템 검증 및 정확도 교정
5. 운영 및 유지보수 체계 구축

---

## 8.1 구축 로드맵

### 8.1.1 단계별 구현 계획

```
┌─────────────────────────────────────────────────────────────────────┐
│                    WIA 가뭄 모니터링 시스템 구축 로드맵               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1단계: 기반 구축 (3개월)                                            │
│  ├─ 인프라 프로비저닝                                                │
│  ├─ 핵심 데이터 파이프라인 구축                                       │
│  └─ 기본 가뭄 지수 계산 엔진 개발                                     │
│                                                                     │
│  2단계: 데이터 통합 (3개월)                                          │
│  ├─ 기상청/위성 데이터 연동                                          │
│  ├─ 현장 센서 네트워크 통합                                          │
│  └─ 데이터 품질 관리 체계 구축                                        │
│                                                                     │
│  3단계: 분석 고도화 (4개월)                                          │
│  ├─ 예측 모델 개발 및 훈련                                           │
│  ├─ 경보 시스템 구축                                                 │
│  └─ 시각화 대시보드 개발                                             │
│                                                                     │
│  4단계: 시스템 통합 (2개월)                                          │
│  ├─ 외부 시스템 연동 (스마트팜, 관개 등)                              │
│  ├─ API 서비스 공개                                                 │
│  └─ 사용자 교육 및 문서화                                            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.1.2 구현 체크리스트

```typescript
// 구현 체크리스트 인터페이스
interface ImplementationChecklist {
  phase1_foundation: {
    infrastructure: [
      "클라우드 계정 및 리소스 그룹 생성",
      "네트워크 설계 (VPC, 서브넷, 보안 그룹)",
      "쿠버네티스 클러스터 프로비저닝",
      "데이터베이스 클러스터 구성",
      "스토리지 (객체 스토리지, 블록 스토리지)",
      "CI/CD 파이프라인 구축",
      "모니터링 및 로깅 설정"
    ];
    corePipeline: [
      "데이터 수집기 개발",
      "메시지 큐 구성 (Kafka/RabbitMQ)",
      "스트림 처리 엔진 설정",
      "데이터 레이크 구조 설계",
      "ETL 워크플로우 구현"
    ];
    droughtEngine: [
      "SPI 계산 모듈",
      "PDSI 계산 모듈",
      "토양 수분 백분위 계산",
      "복합 가뭄 지수 엔진",
      "시계열 저장소 구성"
    ];
  };

  phase2_integration: {
    externalData: [
      "기상청 ASOS/AWS API 연동",
      "기상청 예보 데이터 연동",
      "천리안 2A 위성 데이터 수신",
      "Sentinel-2 데이터 자동 수집",
      "SMAP 토양 수분 데이터 연동"
    ];
    sensorNetwork: [
      "IoT 게이트웨이 설치",
      "토양 수분 센서 배치",
      "기상 관측 장비 설치",
      "데이터 전송 프로토콜 구현",
      "센서 캘리브레이션"
    ];
    qualityControl: [
      "이상치 탐지 알고리즘",
      "결측치 보간 로직",
      "데이터 일관성 검증",
      "품질 플래그 체계",
      "감사 로깅"
    ];
  };

  phase3_analytics: {
    prediction: [
      "예측 모델 아키텍처 설계",
      "훈련 데이터셋 구축",
      "모델 훈련 파이프라인",
      "하이퍼파라미터 튜닝",
      "모델 검증 및 평가"
    ];
    alerting: [
      "경보 규칙 엔진",
      "다중 채널 알림 시스템",
      "에스컬레이션 정책 구현",
      "경보 대시보드",
      "경보 이력 관리"
    ];
    visualization: [
      "웹 대시보드 개발",
      "지도 시각화 (GIS)",
      "시계열 차트",
      "리포트 생성기",
      "모바일 앱 개발"
    ];
  };

  phase4_deployment: {
    externalIntegration: [
      "스마트팜 플랫폼 연동",
      "관개 제어 시스템 연동",
      "재난 경보 시스템 연동",
      "농진청/K-water API 연동"
    ];
    apiService: [
      "REST API 문서화",
      "개발자 포털",
      "API 키 관리",
      "사용량 제한 및 과금",
      "SLA 정의"
    ];
    training: [
      "운영자 교육",
      "사용자 매뉴얼",
      "API 가이드",
      "FAQ 및 문제 해결 가이드"
    ];
  };
}
```

### 8.1.3 마일스톤 정의

| 마일스톤 | 완료 기준 | 검증 방법 |
|---------|---------|---------|
| M1: 인프라 완료 | 모든 클라우드 리소스 프로비저닝 | 인프라 자동 테스트 |
| M2: 데이터 수집 가동 | 3개 이상 소스에서 실시간 데이터 수신 | 데이터 플로우 모니터링 |
| M3: 가뭄 지수 생산 | SPI, PDSI, SM% 일일 생산 | 출력 검증 및 비교 |
| M4: 예측 모델 배포 | 7일 예측 RMSE < 0.5 | 교차 검증 결과 |
| M5: 경보 시스템 가동 | 10분 이내 경보 발송 | End-to-end 테스트 |
| M6: API 공개 | 99.9% 가용성 달성 | 부하 테스트 |
| M7: 외부 연동 완료 | 5개 이상 시스템 연동 | 통합 테스트 |
| M8: 정식 운영 | 모든 기능 안정적 운영 | 2주 파일럿 운영 |

---

## 8.2 인프라 아키텍처

### 8.2.1 클라우드 아키텍처

```typescript
// 클라우드 인프라 구성
interface CloudInfrastructure {
  // 컴퓨팅
  compute: {
    kubernetes: {
      provider: "EKS" | "AKS" | "GKE" | "NKS";  // 네이버 클라우드 포함
      nodeGroups: [
        {
          name: "general",
          instanceType: "m5.xlarge",
          minSize: 3,
          maxSize: 10,
          purpose: "일반 워크로드"
        },
        {
          name: "compute",
          instanceType: "c5.2xlarge",
          minSize: 2,
          maxSize: 20,
          purpose: "가뭄 지수 계산",
          spotEnabled: true
        },
        {
          name: "gpu",
          instanceType: "p3.2xlarge",
          minSize: 0,
          maxSize: 4,
          purpose: "위성 이미지 처리, ML 훈련",
          spotEnabled: true
        }
      ];
    };
  };

  // 데이터베이스
  databases: {
    timeseries: {
      type: "TimescaleDB";
      size: "db.r5.2xlarge";
      storage: "500GB";
      replication: "single-region-ha";
      purpose: "가뭄 지수 시계열 데이터"
    };
    spatial: {
      type: "PostGIS";
      size: "db.r5.xlarge";
      storage: "200GB";
      purpose: "공간 데이터, 지역 경계"
    };
    cache: {
      type: "Redis";
      size: "cache.r5.large";
      clusterMode: true;
      purpose: "API 캐싱, 세션"
    };
    search: {
      type: "Elasticsearch";
      nodes: 3;
      storage: "100GB each";
      purpose: "로그, 검색, 분석"
    };
  };

  // 스토리지
  storage: {
    objectStorage: {
      buckets: [
        { name: "raw-satellite", size: "10TB", lifecycle: "90days-to-glacier" },
        { name: "processed-data", size: "5TB", lifecycle: "1year-delete" },
        { name: "model-artifacts", size: "100GB", versioning: true }
      ];
    };
    blockStorage: {
      volumes: [
        { name: "database-primary", size: "500GB", iops: 10000 },
        { name: "database-replica", size: "500GB", iops: 5000 }
      ];
    };
  };

  // 네트워킹
  networking: {
    vpc: {
      cidr: "10.0.0.0/16";
      subnets: {
        public: ["10.0.1.0/24", "10.0.2.0/24"];
        private: ["10.0.10.0/24", "10.0.11.0/24"];
        database: ["10.0.20.0/24", "10.0.21.0/24"];
      };
    };
    loadBalancer: {
      type: "ALB";
      waf: true;
      ddosProtection: true;
    };
    cdn: {
      enabled: true;
      origins: ["api.drought-monitor.kr", "maps.drought-monitor.kr"];
    };
  };
}

// 인프라 as 코드 (Terraform)
const terraformConfig = `
# VPC 구성
resource "aws_vpc" "drought_monitoring" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_hostnames = true
  enable_dns_support   = true

  tags = {
    Name        = "drought-monitoring-vpc"
    Environment = "production"
    Project     = "wia-drought"
  }
}

# EKS 클러스터
module "eks" {
  source          = "terraform-aws-modules/eks/aws"
  cluster_name    = "drought-monitoring-cluster"
  cluster_version = "1.28"
  vpc_id          = aws_vpc.drought_monitoring.id
  subnet_ids      = module.vpc.private_subnets

  eks_managed_node_groups = {
    general = {
      min_size       = 3
      max_size       = 10
      desired_size   = 3
      instance_types = ["m5.xlarge"]
      capacity_type  = "ON_DEMAND"
    }

    compute = {
      min_size       = 2
      max_size       = 20
      desired_size   = 3
      instance_types = ["c5.2xlarge"]
      capacity_type  = "SPOT"

      labels = {
        workload = "compute-intensive"
      }

      taints = [{
        key    = "compute-intensive"
        value  = "true"
        effect = "NO_SCHEDULE"
      }]
    }
  }
}

# TimescaleDB (RDS)
resource "aws_db_instance" "timescale" {
  identifier     = "drought-timescale"
  engine         = "postgres"
  engine_version = "15"
  instance_class = "db.r5.2xlarge"

  allocated_storage     = 500
  max_allocated_storage = 2000
  storage_type          = "gp3"
  storage_encrypted     = true

  db_name  = "drought_indices"
  username = var.db_username
  password = var.db_password

  multi_az               = true
  vpc_security_group_ids = [aws_security_group.database.id]
  db_subnet_group_name   = aws_db_subnet_group.database.name

  backup_retention_period = 30
  backup_window          = "03:00-04:00"
  maintenance_window     = "Mon:04:00-Mon:05:00"

  tags = {
    Name = "drought-timescale-primary"
  }
}
`;
```

### 8.2.2 쿠버네티스 배포 구성

```yaml
# 가뭄 지수 계산 서비스 배포
apiVersion: apps/v1
kind: Deployment
metadata:
  name: drought-index-calculator
  namespace: drought-monitoring
spec:
  replicas: 3
  selector:
    matchLabels:
      app: drought-index-calculator
  template:
    metadata:
      labels:
        app: drought-index-calculator
    spec:
      nodeSelector:
        workload: compute-intensive
      tolerations:
        - key: "compute-intensive"
          operator: "Equal"
          value: "true"
          effect: "NoSchedule"
      containers:
        - name: calculator
          image: drought-monitoring/index-calculator:v1.2.0
          resources:
            requests:
              cpu: "2"
              memory: "4Gi"
            limits:
              cpu: "4"
              memory: "8Gi"
          env:
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: database-credentials
                  key: timescale-url
            - name: REDIS_URL
              valueFrom:
                configMapKeyRef:
                  name: drought-config
                  key: redis-url
            - name: CALCULATION_INTERVAL
              value: "3600"  # 1시간마다 계산
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 30
            periodSeconds: 10
          readinessProbe:
            httpGet:
              path: /ready
              port: 8080
            initialDelaySeconds: 5
            periodSeconds: 5
---
# Horizontal Pod Autoscaler
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: drought-index-calculator-hpa
  namespace: drought-monitoring
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: drought-index-calculator
  minReplicas: 3
  maxReplicas: 20
  metrics:
    - type: Resource
      resource:
        name: cpu
        target:
          type: Utilization
          averageUtilization: 70
    - type: Resource
      resource:
        name: memory
        target:
          type: Utilization
          averageUtilization: 80
---
# API 서비스 배포
apiVersion: apps/v1
kind: Deployment
metadata:
  name: drought-api
  namespace: drought-monitoring
spec:
  replicas: 5
  selector:
    matchLabels:
      app: drought-api
  template:
    metadata:
      labels:
        app: drought-api
    spec:
      containers:
        - name: api
          image: drought-monitoring/api:v1.2.0
          ports:
            - containerPort: 8080
          resources:
            requests:
              cpu: "500m"
              memory: "512Mi"
            limits:
              cpu: "1"
              memory: "1Gi"
          env:
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: database-credentials
                  key: timescale-readonly-url
            - name: REDIS_URL
              valueFrom:
                configMapKeyRef:
                  name: drought-config
                  key: redis-url
            - name: RATE_LIMIT_RPM
              value: "100"
---
apiVersion: v1
kind: Service
metadata:
  name: drought-api
  namespace: drought-monitoring
spec:
  selector:
    app: drought-api
  ports:
    - port: 80
      targetPort: 8080
  type: ClusterIP
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: drought-api-ingress
  namespace: drought-monitoring
  annotations:
    kubernetes.io/ingress.class: alb
    alb.ingress.kubernetes.io/scheme: internet-facing
    alb.ingress.kubernetes.io/certificate-arn: arn:aws:acm:...
    alb.ingress.kubernetes.io/ssl-policy: ELBSecurityPolicy-TLS-1-2-2017-01
spec:
  rules:
    - host: api.drought-monitor.kr
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: drought-api
                port:
                  number: 80
```

### 8.2.3 온프레미스 하이브리드 구성

| 구성 요소 | 클라우드 배치 | 온프레미스 배치 | 이유 |
|----------|-------------|---------------|-----|
| API 서버 | O | - | 탄력적 확장, 글로벌 배포 |
| 데이터베이스 | O | - | 관리형 서비스, 백업 |
| 위성 처리 | - | O | 대용량 데이터, 전용 GPU |
| 센서 게이트웨이 | - | O | 네트워크 지연 최소화 |
| ML 훈련 | O (Spot) | O | 비용/성능 최적화 |
| 백업 저장소 | O | O | 재해 복구, 규정 준수 |

---

## 8.3 센서 네트워크 구축

### 8.3.1 현장 센서 배치

```typescript
// 센서 네트워크 설계
interface SensorNetworkDesign {
  // 센서 유형별 사양
  sensorTypes: {
    soilMoisture: {
      model: "Decagon 5TM" | "Campbell CS616" | "국산 SoilWatch";
      measurementDepth: number[];  // [10, 30, 60, 100] cm
      accuracy: number;            // ±3% VWC
      powerRequirement: number;    // mW
      outputInterface: "SDI-12" | "analog";
      samplingInterval: number;    // 분
    };

    weather: {
      model: "Davis Vantage Pro2" | "Campbell CR1000X";
      parameters: string[];        // 기온, 습도, 풍속, 강수, 일사
      accuracy: {
        temperature: number;       // ±0.3°C
        humidity: number;          // ±2%
        precipitation: number;     // ±0.2mm
      };
    };

    waterLevel: {
      model: "OTT PLS-C" | "Campbell CS475";
      range: number;               // m
      accuracy: number;            // mm
      interface: "RS-485" | "SDI-12";
    };
  };

  // 관측소 배치 기준
  stationPlacement: {
    density: {
      agricultural: "10km × 10km 격자당 1개소";
      critical: "5km × 5km 격자 (주요 농업 지역)";
      reference: "30km × 30km 격자 (기준 관측소)";
    };

    siteSelection: [
      "대표성 있는 토양 및 지형",
      "농경지 내부 또는 인접",
      "전원 및 통신 인프라 접근성",
      "장기 운영 보장 (토지 사용 동의)",
      "유지보수 접근성"
    ];
  };

  // 통신 아키텍처
  communication: {
    fieldToGateway: "LoRaWAN" | "NB-IoT" | "LTE-M";
    gatewayToCloud: "4G/LTE" | "유선";
    dataFormat: "WIA-DROUGHT-SENSOR-v1";
    encryption: "TLS 1.3";
    updateInterval: {
      normal: 15,          // 분
      drought: 5,          // 분
      emergency: 1         // 분
    };
  };
}

// 센서 설치 워크플로우
class SensorInstallationWorkflow {
  // 사이트 조사
  async conductSiteSurvey(location: Coordinates): Promise<SiteSurveyResult> {
    const survey = {
      location,
      surveyDate: new Date(),

      // 토양 조사
      soil: {
        textureClass: await this.determineSoilTexture(location),
        drainageClass: await this.assessDrainage(location),
        depthToBedrock: await this.measureDepth(location),
        representativeness: "high" | "medium" | "low"
      },

      // 인프라 조사
      infrastructure: {
        powerAvailable: boolean,
        powerDistance: number,        // m
        cellularSignal: number,       // dBm
        loraGatewayDistance: number,  // km
        accessRoad: boolean
      },

      // 토지 정보
      landInfo: {
        ownership: string,
        landUse: string,
        longTermAgreement: boolean,
        nearbyObstructions: string[]
      },

      // 평가 점수
      suitabilityScore: this.calculateSuitability()
    };

    return survey;
  }

  // 센서 설치
  async installSensorStation(
    survey: SiteSurveyResult,
    config: StationConfig
  ): Promise<InstallationResult> {
    const steps = [
      // 1. 구조물 설치
      {
        step: "structure_install",
        tasks: [
          "기초 콘크리트 타설",
          "센서 마스트 설치 (3m)",
          "태양광 패널 설치",
          "배터리함 설치"
        ]
      },

      // 2. 토양 센서 설치
      {
        step: "soil_sensor_install",
        tasks: [
          "토양 시추 (최대 100cm)",
          "센서 프로브 삽입 (10, 30, 60, 100cm)",
          "케이블 매설 및 보호",
          "시추공 복원"
        ]
      },

      // 3. 기상 센서 설치
      {
        step: "weather_sensor_install",
        tasks: [
          "기상 센서 어레이 마스트 장착",
          "우량계 설치 (지상 1m)",
          "센서 수평/방향 조정",
          "차폐 및 방수 처리"
        ]
      },

      // 4. 데이터로거 및 통신
      {
        step: "datalogger_install",
        tasks: [
          "데이터로거 장착",
          "센서 케이블 연결 및 단자 방수",
          "LoRa/LTE 모뎀 설치",
          "안테나 설치"
        ]
      },

      // 5. 전원 시스템
      {
        step: "power_install",
        tasks: [
          "태양광 컨트롤러 설치",
          "배터리 연결 (12V 100Ah)",
          "전원 분배 및 과전압 보호",
          "접지 설치"
        ]
      },

      // 6. 검증 및 등록
      {
        step: "verification",
        tasks: [
          "모든 센서 판독 확인",
          "데이터 전송 테스트",
          "시스템 등록 및 메타데이터 입력",
          "현장 사진 촬영 및 문서화"
        ]
      }
    ];

    const results = [];
    for (const step of steps) {
      results.push(await this.executeStep(step));
    }

    return {
      stationId: this.generateStationId(),
      location: survey.location,
      installDate: new Date(),
      sensors: config.sensors,
      communicationType: config.communication,
      verificationStatus: this.checkAllPassed(results)
    };
  }

  // 센서 캘리브레이션
  async calibrateSensors(stationId: string): Promise<CalibrationResult> {
    // 토양 수분 센서 캘리브레이션
    const soilCalibration = await this.calibrateSoilMoisture(stationId, {
      method: "gravimetric",
      sampleCount: 10,
      moistureRange: [10, 50]  // % VWC
    });

    // 강수계 캘리브레이션
    const rainCalibration = await this.calibrateRainGauge(stationId, {
      method: "known_volume",
      testVolume: 500,  // mL
      tipCount: 100
    });

    return {
      stationId,
      calibrationDate: new Date(),
      soilMoisture: {
        offset: soilCalibration.offset,
        slope: soilCalibration.slope,
        r2: soilCalibration.r2
      },
      precipitation: {
        mmPerTip: rainCalibration.mmPerTip,
        accuracy: rainCalibration.accuracy
      },
      nextCalibrationDue: this.addMonths(new Date(), 12)
    };
  }
}
```

### 8.3.2 센서 비용 및 사양 비교

| 센서 유형 | 모델 | 가격 (원) | 정확도 | 출력 | 전원 |
|----------|-----|---------|-------|-----|-----|
| 토양수분 (저가) | Decagon EC-5 | 150,000 | ±3% | Analog | 2.5V |
| 토양수분 (고급) | Campbell CS650 | 800,000 | ±1% | SDI-12 | 12V |
| 기상관측 | Davis VP2 | 1,200,000 | 복합 | USB | 태양광 |
| 기상관측 (연구용) | CR1000X 시스템 | 8,000,000 | 고정밀 | RS-232 | 12V |
| 수위계 | OTT PLS-C | 2,500,000 | ±1mm | RS-485 | 12V |
| LoRa 게이트웨이 | Kerlink iFemtoCell | 600,000 | N/A | LTE | PoE |

### 8.3.3 관측망 운영 예산

| 항목 | 단위 비용 | 수량 | 연간 비용 |
|-----|---------|-----|---------|
| 센서 스테이션 (표준) | 5,000,000원/개소 | 100개 | 500,000,000원 (초기) |
| 유지보수 | 500,000원/개소/년 | 100개 | 50,000,000원 |
| 통신비 (LTE) | 15,000원/개소/월 | 100개 | 18,000,000원 |
| 교정 서비스 | 200,000원/개소/년 | 100개 | 20,000,000원 |
| 운영 인력 | 50,000,000원/명/년 | 3명 | 150,000,000원 |
| **연간 운영 총액** | | | **238,000,000원** |

---

## 8.4 시스템 검증

### 8.4.1 정확도 검증 프레임워크

```typescript
// 검증 프레임워크
interface ValidationFramework {
  // 검증 대상
  targets: {
    droughtIndices: ["SPI", "PDSI", "soil_moisture_percentile"];
    predictions: ["7day_forecast", "seasonal_outlook"];
    alerts: ["timing_accuracy", "spatial_accuracy"];
  };

  // 검증 방법
  methods: {
    // 기준 데이터 비교
    referenceComparison: {
      spi: {
        reference: "기상청 공식 SPI",
        metrics: ["correlation", "rmse", "bias"],
        threshold: { correlation: 0.95, rmse: 0.3 }
      };
      pdsi: {
        reference: "기존 연구 PDSI (논문)",
        metrics: ["correlation", "classification_accuracy"],
        threshold: { correlation: 0.90, accuracy: 0.85 }
      };
    };

    // 현장 검증
    fieldValidation: {
      soilMoisture: {
        method: "gravimetric_sampling",
        sampleSize: 100,
        frequency: "monthly"
      };
      vegetationStress: {
        method: "visual_assessment",
        protocol: "standardized_survey",
        frequency: "bi-weekly during growing season"
      };
    };

    // 사용자 피드백
    userFeedback: {
      channels: ["web_survey", "mobile_app", "field_reports"];
      metrics: ["usefulness", "timeliness", "accuracy_perception"];
    };
  };
}

// 검증 실행 클래스
class DroughtSystemValidator {
  private referenceData: ReferenceDataStore;
  private fieldData: FieldDataCollector;

  // SPI 검증
  async validateSPI(
    testPeriod: DateRange,
    regions: string[]
  ): Promise<ValidationResult> {
    const results: RegionValidation[] = [];

    for (const region of regions) {
      // WIA 계산 SPI 로드
      const wiaSPI = await this.loadWIASPI(region, testPeriod);

      // 기상청 기준 SPI 로드
      const kmaSPI = await this.referenceData.loadKMASPI(region, testPeriod);

      // 통계 분석
      const correlation = this.calculateCorrelation(wiaSPI, kmaSPI);
      const rmse = this.calculateRMSE(wiaSPI, kmaSPI);
      const bias = this.calculateBias(wiaSPI, kmaSPI);

      // 등급 분류 일치도
      const classificationMatrix = this.buildConfusionMatrix(
        wiaSPI.map(v => this.classifySPI(v)),
        kmaSPI.map(v => this.classifySPI(v)),
        ["D0", "D1", "D2", "D3", "D4"]
      );

      results.push({
        region,
        metrics: { correlation, rmse, bias },
        confusionMatrix: classificationMatrix,
        passed: correlation > 0.95 && rmse < 0.3
      });
    }

    return {
      testPeriod,
      indexType: "SPI",
      regionResults: results,
      overallPassed: results.every(r => r.passed),
      summary: this.generateSummary(results)
    };
  }

  // 토양 수분 현장 검증
  async validateSoilMoisture(
    stationIds: string[],
    samplingDate: Date
  ): Promise<FieldValidationResult> {
    const samples: SoilSample[] = [];

    for (const stationId of stationIds) {
      // 센서 측정값
      const sensorReading = await this.getSensorReading(stationId, samplingDate);

      // 현장 채취 (가정: 현장 작업자가 입력)
      const fieldSample = await this.fieldData.getGravimetricSample(
        stationId,
        samplingDate
      );

      if (fieldSample) {
        samples.push({
          stationId,
          sensorValue: sensorReading.volumetricWaterContent,
          gravimetricValue: fieldSample.volumetricWaterContent,
          depth: fieldSample.depth,
          soilTexture: fieldSample.texture
        });
      }
    }

    // 분석
    const analysis = {
      sampleCount: samples.length,
      correlation: this.calculateCorrelation(
        samples.map(s => s.sensorValue),
        samples.map(s => s.gravimetricValue)
      ),
      rmse: this.calculateRMSE(
        samples.map(s => s.sensorValue),
        samples.map(s => s.gravimetricValue)
      ),
      byDepth: this.analyzeByDepth(samples),
      byTexture: this.analyzeByTexture(samples)
    };

    return {
      date: samplingDate,
      samples,
      analysis,
      calibrationNeeded: analysis.rmse > 5 || analysis.correlation < 0.85
    };
  }

  // 예측 정확도 검증
  async validateForecast(
    forecastHorizon: number,
    testPeriod: DateRange
  ): Promise<ForecastValidationResult> {
    const forecasts: ForecastEvaluation[] = [];

    // 테스트 기간 내 모든 예측 수집
    let currentDate = testPeriod.start;
    while (currentDate <= testPeriod.end) {
      // 해당 날짜에 발행된 예측 조회
      const forecast = await this.loadForecast(currentDate, forecastHorizon);

      if (forecast) {
        // 실제 관측값 조회
        const verificationDate = this.addDays(currentDate, forecastHorizon);
        const observed = await this.loadObserved(verificationDate);

        if (observed) {
          forecasts.push({
            issueDate: currentDate,
            verificationDate,
            predicted: forecast.value,
            observed: observed.value,
            error: forecast.value - observed.value
          });
        }
      }

      currentDate = this.addDays(currentDate, 1);
    }

    // 통계 계산
    const errors = forecasts.map(f => f.error);
    const absErrors = errors.map(Math.abs);

    return {
      forecastHorizon,
      testPeriod,
      sampleSize: forecasts.length,
      metrics: {
        mae: this.mean(absErrors),
        rmse: Math.sqrt(this.mean(errors.map(e => e * e))),
        bias: this.mean(errors),
        correlation: this.calculateCorrelation(
          forecasts.map(f => f.predicted),
          forecasts.map(f => f.observed)
        ),
        skillScore: this.calculateSkillScore(forecasts)
      },
      percentiles: {
        p50Error: this.percentile(absErrors, 50),
        p90Error: this.percentile(absErrors, 90),
        p95Error: this.percentile(absErrors, 95)
      }
    };
  }

  // 혼동 행렬 생성
  private buildConfusionMatrix(
    predicted: string[],
    actual: string[],
    classes: string[]
  ): ConfusionMatrix {
    const matrix = {};

    for (const actualClass of classes) {
      matrix[actualClass] = {};
      for (const predictedClass of classes) {
        matrix[actualClass][predictedClass] = 0;
      }
    }

    for (let i = 0; i < predicted.length; i++) {
      matrix[actual[i]][predicted[i]]++;
    }

    // 정확도 계산
    let correct = 0;
    for (const cls of classes) {
      correct += matrix[cls][cls];
    }
    const accuracy = correct / predicted.length;

    return { matrix, classes, accuracy };
  }
}
```

### 8.4.2 검증 기준 및 임계값

| 지표 | 메트릭 | 목표 | 최소 허용 |
|-----|-------|-----|---------|
| SPI | 상관계수 | > 0.95 | > 0.90 |
| SPI | RMSE | < 0.2 | < 0.3 |
| PDSI | 등급 일치도 | > 90% | > 85% |
| 토양 수분 | 상관계수 | > 0.90 | > 0.85 |
| 토양 수분 | RMSE | < 3% VWC | < 5% VWC |
| 7일 예측 | MAE | < 0.3 | < 0.5 |
| 경보 정확도 | F1 Score | > 0.85 | > 0.80 |

---

## 8.5 운영 및 유지보수

### 8.5.1 운영 모니터링

```typescript
// 시스템 모니터링 구성
interface OperationalMonitoring {
  // 인프라 모니터링
  infrastructure: {
    metrics: [
      "cpu_utilization",
      "memory_usage",
      "disk_io",
      "network_throughput",
      "pod_status",
      "node_health"
    ];
    tools: ["Prometheus", "Grafana", "CloudWatch"];
    alerting: {
      pagerduty: true;
      slack: "#drought-ops";
      email: "ops-team@drought-monitor.kr";
    };
  };

  // 데이터 품질 모니터링
  dataQuality: {
    checks: [
      "data_freshness",        // 데이터 최신성
      "completeness",          // 완전성
      "value_range",           // 값 범위
      "spatial_consistency",   // 공간 일관성
      "temporal_consistency"   // 시간 일관성
    ];
    frequency: "hourly";
    alertThresholds: {
      maxStalenessMinutes: 60,
      minCompleteness: 0.95,
      maxAnomalyRate: 0.05
    };
  };

  // 센서 네트워크 모니터링
  sensorNetwork: {
    metrics: [
      "station_online_status",
      "battery_level",
      "signal_strength",
      "data_transmission_rate",
      "sensor_drift"
    ];
    maintenanceAlerts: {
      lowBattery: "< 11.5V",
      weakSignal: "< -100 dBm",
      noData: "> 2 hours"
    };
  };
}

// SLA 정의
interface ServiceLevelAgreement {
  // 가용성
  availability: {
    apiUptime: "99.9%";           // 연간 다운타임 < 8.76시간
    dataFreshness: "< 1 hour";    // 최신 데이터 1시간 이내
    alertLatency: "< 10 minutes"; // 경보 발송 지연
  };

  // 성능
  performance: {
    apiResponseTime: {
      p50: "200ms",
      p95: "500ms",
      p99: "1000ms"
    };
    batchProcessing: {
      dailyIndices: "< 2 hours",
      weeklyReport: "< 4 hours"
    };
  };

  // 지원
  support: {
    incidentResponse: {
      critical: "15 minutes",
      major: "1 hour",
      minor: "4 hours"
    };
    plannedMaintenance: "48 hours notice";
  };
}

// 운영 대시보드 메트릭
const operationalDashboard = {
  // 시스템 건강
  systemHealth: {
    panels: [
      {
        title: "서비스 상태",
        type: "status",
        metrics: ["api", "calculator", "database", "cache"]
      },
      {
        title: "CPU/메모리 사용률",
        type: "timeseries",
        metrics: ["cpu_avg", "memory_avg"]
      },
      {
        title: "요청 처리량",
        type: "counter",
        metric: "http_requests_total"
      }
    ]
  },

  // 데이터 파이프라인
  dataPipeline: {
    panels: [
      {
        title: "데이터 수집 현황",
        type: "table",
        columns: ["source", "last_update", "record_count", "status"]
      },
      {
        title: "처리 대기열",
        type: "gauge",
        metric: "queue_depth"
      },
      {
        title: "처리 지연",
        type: "timeseries",
        metric: "processing_lag_seconds"
      }
    ]
  },

  // 센서 네트워크
  sensorNetwork: {
    panels: [
      {
        title: "관측소 온라인 현황",
        type: "map",
        metric: "station_online_status"
      },
      {
        title: "배터리 상태",
        type: "heatmap",
        metric: "battery_voltage"
      },
      {
        title: "데이터 수신율",
        type: "bar",
        metric: "data_receive_rate_by_station"
      }
    ]
  }
};
```

### 8.5.2 장애 대응 절차

```
┌─────────────────────────────────────────────────────────────────────┐
│                         장애 대응 프로세스                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. 탐지 (Detection)                                                │
│  ├─ 자동 알림 수신 (PagerDuty/Slack)                                │
│  ├─ 사용자 신고 접수                                                │
│  └─ 모니터링 대시보드 확인                                           │
│                                                                     │
│  2. 분류 (Triage)                                                   │
│  ├─ 심각도 평가 (P1-P4)                                             │
│  ├─ 영향 범위 파악                                                  │
│  └─ 담당자 배정                                                     │
│                                                                     │
│  3. 진단 (Diagnosis)                                                │
│  ├─ 로그 분석                                                       │
│  ├─ 메트릭 검토                                                     │
│  └─ 최근 변경 사항 확인                                             │
│                                                                     │
│  4. 완화 (Mitigation)                                               │
│  ├─ 즉각적인 영향 최소화                                            │
│  ├─ 트래픽 우회 / 롤백                                              │
│  └─ 임시 조치 적용                                                  │
│                                                                     │
│  5. 해결 (Resolution)                                               │
│  ├─ 근본 원인 수정                                                  │
│  ├─ 테스트 및 검증                                                  │
│  └─ 정상화 확인                                                     │
│                                                                     │
│  6. 사후 검토 (Post-mortem)                                         │
│  ├─ 인시던트 보고서 작성                                            │
│  ├─ 재발 방지 대책 수립                                             │
│  └─ 프로세스 개선                                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.5.3 유지보수 일정

| 활동 | 주기 | 담당 | 소요 시간 |
|-----|-----|-----|---------|
| 센서 현장 점검 | 월 1회 | 현장 엔지니어 | 2시간/개소 |
| 센서 캘리브레이션 | 연 1회 | 전문 기술자 | 1일/10개소 |
| 소프트웨어 업데이트 | 격주 | 개발팀 | 2시간 |
| 보안 패치 | 월 1회 | 인프라팀 | 4시간 |
| 데이터베이스 최적화 | 월 1회 | DBA | 2시간 |
| 백업 검증 | 주 1회 | 인프라팀 | 1시간 |
| 재해복구 훈련 | 분기 1회 | 전체팀 | 4시간 |
| 보안 감사 | 연 1회 | 외부 감사 | 1주일 |

---

## 8.6 확장 및 고도화

### 8.6.1 기능 확장 로드맵

```typescript
// 향후 기능 확장 계획
interface ExpansionRoadmap {
  nearTerm: {  // 1년 이내
    features: [
      {
        name: "AI 기반 가뭄 예측 고도화",
        description: "LSTM/Transformer 모델로 30일 예측 정확도 향상",
        priority: "high"
      },
      {
        name: "모바일 앱 출시",
        description: "농업인 대상 모바일 앱 (iOS/Android)",
        priority: "high"
      },
      {
        name: "지역 맞춤 경보",
        description: "시군구 단위 세분화 경보 서비스",
        priority: "medium"
      }
    ];
  };

  midTerm: {  // 1-2년
    features: [
      {
        name: "작물별 피해 예측",
        description: "주요 작물(벼, 과수, 채소) 피해액 추정",
        priority: "high"
      },
      {
        name: "계절 전망 서비스",
        description: "3개월 계절 가뭄 전망 제공",
        priority: "medium"
      },
      {
        name: "동아시아 확장",
        description: "일본, 대만, 중국 일부 지역 서비스",
        priority: "medium"
      }
    ];
  };

  longTerm: {  // 2년 이상
    features: [
      {
        name: "글로벌 서비스",
        description: "전 세계 가뭄 모니터링 플랫폼",
        priority: "high"
      },
      {
        name: "기후변화 시나리오 분석",
        description: "RCP/SSP 시나리오별 미래 가뭄 전망",
        priority: "medium"
      },
      {
        name: "물 거래 플랫폼",
        description: "농업용수 거래 시장 연계",
        priority: "low"
      }
    ];
  };
}

// 확장성 설계 원칙
const scalabilityPrinciples = {
  horizontal: {
    compute: "Kubernetes HPA로 자동 스케일링",
    database: "읽기 복제본 추가, 샤딩 준비",
    storage: "객체 스토리지 무제한 확장"
  },

  geographic: {
    multiRegion: "AWS/Azure 다중 리전 배포",
    edgeCache: "CDN 엣지 캐싱",
    dataResidency: "국가별 데이터 주권 준수"
  },

  integration: {
    apiVersioning: "하위 호환 API 버저닝",
    webhooks: "이벤트 기반 외부 연동",
    plugins: "플러그인 아키텍처"
  }
};
```

### 8.6.2 성능 최적화 전략

| 영역 | 현재 | 목표 | 최적화 방안 |
|-----|-----|-----|-----------|
| API 응답 | P95 500ms | P95 200ms | Redis 캐싱, 쿼리 최적화 |
| 지수 계산 | 2시간/일 | 30분/일 | Spark 클러스터, 병렬 처리 |
| 위성 처리 | 8시간/장면 | 2시간/장면 | GPU 가속, 파이프라인 최적화 |
| 스토리지 비용 | $10K/월 | $5K/월 | 수명주기 정책, 압축 |

---

## 8.7 복습 문제

### 문제 1
WIA 가뭄 모니터링 시스템 구축의 4단계와 각 단계의 주요 마일스톤을 설명하시오.

### 문제 2
쿠버네티스 기반 가뭄 지수 계산 서비스 배포 시, 컴퓨팅 집약적 워크로드를 위한 노드 선택 및 HPA 설정 방법을 설명하시오.

### 문제 3
현장 토양 수분 센서 설치 시 고려해야 할 사이트 선정 기준 5가지와 각 기준의 중요성을 서술하시오.

### 문제 4
SPI 검증에서 혼동 행렬(Confusion Matrix)의 역할과 가뭄 등급 분류 정확도 평가 방법을 설명하시오.

### 문제 5
시스템 장애 발생 시 6단계 대응 프로세스와 P1(Critical) 장애 시 목표 대응 시간을 설명하시오.

---

## 8.8 핵심 요약

### 주요 학습 내용

| 항목 | 핵심 내용 |
|-----|---------|
| 구축 로드맵 | 4단계 12개월, 8개 마일스톤 |
| 인프라 | K8s 기반, 3개 노드 그룹, TimescaleDB |
| 센서 네트워크 | 10km 격자 배치, LoRaWAN/LTE, 연 2.4억 운영비 |
| 검증 | SPI 상관 >0.95, 토양수분 RMSE <5% |
| 운영 | 99.9% 가용성, P1 장애 15분 대응 |

### 핵심 기억 사항

1. **점진적 구축**: 기반 → 통합 → 분석 → 배포 순서 준수
2. **하이브리드 아키텍처**: 위성 처리는 온프레미스, API는 클라우드
3. **현장 검증 필수**: 센서 데이터와 실측값 주기적 비교
4. **자동화**: 인프라, 배포, 모니터링 전 과정 자동화
5. **SLA 관리**: 명확한 목표와 지속적 측정/개선

### 다음 단계

이 ebook 시리즈를 통해 WIA 가뭄 모니터링 표준의 전체적인 이해와 구현 역량을 갖추셨습니다. 다음 단계로:

1. **실습 환경 구축**: 개발/테스트 환경에서 핵심 컴포넌트 구현
2. **파일럿 프로젝트**: 제한된 지역에서 시범 운영
3. **커뮤니티 참여**: WIA 표준 커뮤니티에서 경험 공유
4. **인증 획득**: WIA 가뭄 모니터링 표준 적합성 인증

---

## 부록: 참고 자료

### A. 관련 표준
- ISO 17089: 물 품질 - 가뭄 영향 평가
- WMO No. 1006: 가뭄 관측 가이드라인
- OGC 17-007: 환경 관측 서비스 인터페이스

### B. 유용한 도구
- GDAL/OGR: 지리공간 데이터 처리
- xarray: 다차원 배열 분석
- dask: 병렬 처리
- Apache Airflow: 워크플로우 오케스트레이션

### C. 데이터 소스
- 기상청 ASOS/AWS (https://data.kma.go.kr)
- 국가수자원관리종합정보시스템 (https://wamis.go.kr)
- NASA EARTHDATA (https://earthdata.nasa.gov)
- Copernicus Open Access Hub (https://scihub.copernicus.eu)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
