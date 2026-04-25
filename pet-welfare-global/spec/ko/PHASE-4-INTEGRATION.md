# WIA Pet Welfare Global 통합 표준
## Phase 4 사양서

---

**버전**: 1.0.0
**상태**: Draft
**날짜**: 2025-12-18
**작성자**: WIA Standards Committee
**라이선스**: MIT
**Primary Color**: #F59E0B (Amber)

---

## 목차

1. [개요](#개요)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [통합 패턴](#통합-패턴)
4. [보호소 관리 시스템](#보호소-관리-시스템)
5. [정부 시스템](#정부-시스템)
6. [제3자 통합](#제3자-통합)
7. [데이터 마이그레이션](#데이터-마이그레이션)
8. [테스트 및 인증](#테스트-및-인증)
9. [배포 전략](#배포-전략)
10. [코드 예제](#코드-예제)

---

## 개요

### 1.1 목적

WIA Pet Welfare Global 통합 표준은 동물 복지 시스템, 보호소, 정부 기관, NGO 및 제3자 서비스를 글로벌 반려동물 복지 네트워크에 연결하기 위한 포괄적인 통합 패턴, 시스템 아키텍처, 마이그레이션 전략 및 구현 가이드라인을 정의합니다.

**핵심 목표**:
- 기존 보호소 관리 시스템과의 원활한 통합 지원
- 정부 데이터베이스 및 레지스트리를 위한 표준화된 커넥터 제공
- 레거시 시스템 마이그레이션 및 데이터 변환 지원
- 규정 준수 구현을 위한 테스트 및 인증 절차 정의
- 배포 및 운영을 위한 모범 사례 수립
- 다양한 기술 스택 간 상호 운용성 촉진

### 1.2 통합 범위

| 통합 유형 | 설명 | 우선순위 |
|----------|------|---------|
| **Shelter Management** | PetPoint, ShelterLuv, Chameleon, 커스텀 시스템 | High |
| **Government Registries** | 국가 반려동물 데이터베이스, 마이크로칩 레지스트리 | High |
| **Veterinary Systems** | 진료 관리 소프트웨어, 건강 기록 | High |
| **Transport & Logistics** | IATA 시스템, 항공 화물 플랫폼 | High |
| **Payment Processing** | 입양비, 기부금, 마이크로칩 등록 | Medium |
| **Social Media** | 입양 목록, 성공 사례 | Medium |
| **Mapping Services** | 위치 추적, 서비스 지역 | Medium |
| **Analytics Platforms** | 비즈니스 인텔리전스, 복지 동향 | Low |

### 1.3 통합 아키텍처 계층

| 계층 | 컴포넌트 | 표준 |
|------|---------|------|
| **Application** | 비즈니스 로직, 워크플로우 | WIA-PET-PROTOCOL |
| **API Gateway** | 인증, 속도 제한, 라우팅 | OAuth 2.0, REST |
| **Integration** | 어댑터, 변환기, 커넥터 | ETL, ESB |
| **Data** | 저장소, 복제, 동기화 | PostgreSQL, MongoDB |
| **Infrastructure** | 컨테이너, 오케스트레이션, 모니터링 | Docker, Kubernetes |

---

## 시스템 아키텍처

### 2.1 참조 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                        WIA Pet Welfare Global                        │
│                         통합 아키텍처                                 │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                         외부 시스템                                   │
├─────────────────┬───────────────┬───────────────┬───────────────────┤
│ 보호소 시스템    │  정부 시스템   │  수의사 시스템 │  제3자 시스템     │
│  - PetPoint     │   - 국가       │  - Avimark    │  - 결제          │
│  - ShelterLuv   │     레지스트리  │  - Cornerstone│  - 소셜 미디어    │
│  - Chameleon    │   - USDA      │  - eVetPractice│ - 지도/GIS       │
└────────┬────────┴───────┬───────┴───────┬───────┴──────┬────────────┘
         │                │               │              │
         │                │               │              │
┌────────▼────────────────▼───────────────▼──────────────▼────────────┐
│                      API Gateway 계층                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │ Auth &      │  │ Rate        │  │ Request     │                 │
│  │ AuthZ       │  │ Limiting    │  │ Routing     │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                   통합 서비스 계층                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  어댑터      │  │ 변환기       │  │  검증기      │              │
│  │  - REST API  │  │ - Data Maps  │  │ - Schema     │              │
│  │  - SOAP      │  │ - Format     │  │   Check      │              │
│  │  - File/FTP  │  │   Convert    │  │ - Business   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │ Sync Engine  │  │  Event Bus   │  │  Workflow    │              │
│  │ - Real-time  │  │ - Pub/Sub    │  │  Engine      │              │
│  │ - Scheduled  │  │ - Topics     │  │ - Processes  │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                      핵심 서비스 계층                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Animal      │  │  Adoption    │  │  Transport   │              │
│  │  Management  │  │  Service     │  │  Service     │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Health      │  │  Abuse       │  │  Welfare     │              │
│  │  Passport    │  │  Reporting   │  │  Assessment  │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                        데이터 계층                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Primary DB  │  │  Document    │  │  Cache       │              │
│  │  PostgreSQL  │  │  Store       │  │  Redis       │              │
│  │              │  │  MongoDB     │  │              │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Blob        │  │  Search      │  │  Analytics   │              │
│  │  Storage     │  │  Elasticsearch│ │  Warehouse   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└───────────────────────────────────────────────────────────────────────┘
```

### 2.2 컴포넌트 사양

| 컴포넌트 | 기술 옵션 | 목적 |
|----------|---------|------|
| **API Gateway** | Kong, AWS API Gateway, Nginx | 요청 라우팅, 인증 |
| **Integration Bus** | Apache Camel, MuleSoft, WSO2 | 서비스 오케스트레이션 |
| **Message Queue** | RabbitMQ, Apache Kafka, AWS SQS | 비동기 통신 |
| **Primary Database** | PostgreSQL, MySQL | 관계형 데이터 |
| **Document Store** | MongoDB, CouchDB | 유연한 스키마 |
| **Cache** | Redis, Memcached | 성능 최적화 |
| **Search** | Elasticsearch, Apache Solr | 전문 검색 |
| **Blob Storage** | AWS S3, Azure Blob, MinIO | 미디어 파일 |
| **Container Runtime** | Docker, containerd | 애플리케이션 패키징 |
| **Orchestration** | Kubernetes, Docker Swarm | 컨테이너 관리 |

### 2.3 배포 토폴로지

#### 2.3.1 클라우드 네이티브 배포

```yaml
deployment_type: cloud_native
infrastructure:
  cloud_provider: aws  # or azure, gcp
  regions:
    - us-east-1
    - eu-west-1
    - ap-southeast-1
  services:
    compute: eks  # Kubernetes
    database: rds_postgresql
    cache: elasticache_redis
    storage: s3
    cdn: cloudfront
    dns: route53
  high_availability: true
  auto_scaling: true
  disaster_recovery:
    rpo_hours: 1
    rto_hours: 4
```

#### 2.3.2 온프레미스 배포

```yaml
deployment_type: on_premises
infrastructure:
  datacenter: primary_dc
  hardware:
    app_servers: 4
    database_servers: 2
    load_balancers: 2
  networking:
    load_balancer: haproxy
    firewall: pfsense
    vpn: wireguard
  storage:
    type: san
    capacity_tb: 10
  backup:
    type: tape_and_offsite
    frequency: daily
```

#### 2.3.3 하이브리드 배포

```yaml
deployment_type: hybrid
on_premises:
  services:
    - primary_database
    - sensitive_data_storage
  connectivity: vpn_tunnel
cloud:
  services:
    - api_gateway
    - integration_services
    - analytics
  provider: aws
sync_strategy:
  type: bidirectional
  frequency: real_time
```

---

## 통합 패턴

### 3.1 RESTful API 통합

#### 3.1.1 기본 REST 어댑터

```python
import requests
import json
from typing import Dict, List, Optional
from datetime import datetime

class WIARestAdapter:
    """
    WIA Pet Welfare Global 통합을 위한 RESTful API 어댑터
    """

    def __init__(self, base_url: str, api_key: str, organization_id: str):
        self.base_url = base_url
        self.api_key = api_key
        self.organization_id = organization_id
        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
            'X-WIA-Organization-ID': organization_id
        })

    def create_animal(self, animal_data: Dict) -> Dict:
        """새 동물 프로필 생성"""
        response = self.session.post(
            f'{self.base_url}/animals',
            json=animal_data
        )
        response.raise_for_status()
        return response.json()

    def update_animal(self, animal_id: str, updates: Dict) -> Dict:
        """기존 동물 프로필 업데이트"""
        response = self.session.patch(
            f'{self.base_url}/animals/{animal_id}',
            json=updates
        )
        response.raise_for_status()
        return response.json()

    def add_vaccination(self, animal_id: str, vaccination: Dict) -> Dict:
        """건강 여권에 예방접종 기록 추가"""
        response = self.session.post(
            f'{self.base_url}/animals/{animal_id}/health-passport/vaccinations',
            json=vaccination
        )
        response.raise_for_status()
        return response.json()

    def search_animals(self, filters: Dict) -> List[Dict]:
        """필터를 사용하여 동물 검색"""
        response = self.session.get(
            f'{self.base_url}/animals/search',
            params=filters
        )
        response.raise_for_status()
        return response.json()['data']['animals']

    def submit_welfare_assessment(self, animal_id: str,
                                   assessment: Dict) -> Dict:
        """복지 평가 제출"""
        response = self.session.post(
            f'{self.base_url}/animals/{animal_id}/welfare-assessments',
            json=assessment
        )
        response.raise_for_status()
        return response.json()

# 사용 예제
adapter = WIARestAdapter(
    base_url='https://api.wia.org/pet-welfare-global/v1',
    api_key='your_api_key',
    organization_id='WIA-ORG-US-001234'
)

# 동물 생성
animal = adapter.create_animal({
    'identifiers': {
        'microchip_id': '900123456789012'
    },
    'basic_info': {
        'name': 'Max',
        'species': 'canis_lupus_familiaris',
        'breed': 'Golden Retriever',
        'sex': 'male',
        'date_of_birth': '2023-06-15'
    }
})
print(f"동물 생성됨: {animal['data']['wia_id']}")
```

### 3.2 이벤트 기반 통합

#### 3.2.1 웹훅 컨슈머

```javascript
const express = require('express');
const crypto = require('crypto');

class WIAWebhookConsumer {
  constructor(webhookSecret) {
    this.webhookSecret = webhookSecret;
    this.app = express();
    this.setupMiddleware();
    this.setupRoutes();
  }

  setupMiddleware() {
    this.app.use(express.json());
  }

  verifySignature(payload, signature) {
    const hmac = crypto.createHmac('sha256', this.webhookSecret);
    const digest = hmac.update(JSON.stringify(payload)).digest('hex');
    return crypto.timingSafeEqual(
      Buffer.from(signature),
      Buffer.from(digest)
    );
  }

  setupRoutes() {
    this.app.post('/webhooks/wia', (req, res) => {
      const signature = req.headers['x-wia-signature'];

      if (!this.verifySignature(req.body, signature)) {
        return res.status(401).send('Invalid signature');
      }

      const { event, data } = req.body;

      try {
        this.handleEvent(event, data);
        res.status(200).send('Webhook processed');
      } catch (error) {
        console.error('웹훅 처리 오류:', error);
        res.status(500).send('Processing error');
      }
    });
  }

  handleEvent(event, data) {
    switch (event) {
      case 'animal.created':
        this.onAnimalCreated(data);
        break;

      case 'animal.updated':
        this.onAnimalUpdated(data);
        break;

      case 'animal.adopted':
        this.onAnimalAdopted(data);
        break;

      case 'adoption.application.submitted':
        this.onAdoptionApplication(data);
        break;

      case 'transport.status.updated':
        this.onTransportUpdate(data);
        break;

      case 'abuse.report.submitted':
        this.onAbuseReport(data);
        break;

      case 'welfare.assessment.completed':
        this.onWelfareAssessment(data);
        break;

      default:
        console.log(`처리되지 않은 이벤트: ${event}`);
    }
  }

  onAnimalCreated(data) {
    console.log(`새 동물 생성됨: ${data.animal_id}`);
    // 로컬 데이터베이스 업데이트
    // 알림 트리거
  }

  onAnimalUpdated(data) {
    console.log(`동물 업데이트됨: ${data.animal_id}`);
    // 로컬 시스템에 변경 사항 동기화
  }

  onAnimalAdopted(data) {
    console.log(`동물 입양됨: ${data.animal_id}`);
    // 재고 업데이트
    // 축하 이메일 전송
    // 통계 업데이트
  }

  onAdoptionApplication(data) {
    console.log(`새 입양 신청: ${data.application_id}`);
    // 입양 코디네이터에게 알림
    // 검토 워크플로우 시작
  }

  onTransportUpdate(data) {
    console.log(`운송 업데이트: ${data.transport_id} - ${data.status}`);
    // 추적 시스템 업데이트
    // 관련 당사자에게 알림
  }

  onAbuseReport(data) {
    console.log(`학대 신고 제출됨: ${data.incident_id}`);
    // 조사팀에 경고
    // 로컬 시스템에 사례 생성
  }

  onWelfareAssessment(data) {
    console.log(`복지 평가 완료됨: ${data.assessment_id}`);
    // 동물 복지 점수 업데이트
    // 필요시 개입 트리거
  }

  start(port = 3000) {
    this.app.listen(port, () => {
      console.log(`웹훅 컨슈머가 포트 ${port}에서 수신 중입니다`);
    });
  }
}

// 사용 예시
const consumer = new WIAWebhookConsumer('your_webhook_secret');
consumer.start(3000);
```

### 3.3 배치 통합 패턴

#### 3.3.1 ETL 파이프라인

```java
import java.sql.*;
import java.util.*;
import org.json.*;

public class WIABatchIntegration {

    private Connection localDb;
    private String wiaApiEndpoint;
    private String apiKey;

    public WIABatchIntegration(Connection localDb, String wiaApiEndpoint,
                                String apiKey) {
        this.localDb = localDb;
        this.wiaApiEndpoint = wiaApiEndpoint;
        this.apiKey = apiKey;
    }

    /**
     * 로컬 보호소 관리 시스템에서 동물 추출
     */
    public List<Animal> extractAnimals(Date since) throws SQLException {
        List<Animal> animals = new ArrayList<>();

        String query = "SELECT * FROM animals WHERE updated_at > ? " +
                       "ORDER BY updated_at ASC LIMIT 1000";

        try (PreparedStatement stmt = localDb.prepareStatement(query)) {
            stmt.setTimestamp(1, new Timestamp(since.getTime()));

            ResultSet rs = stmt.executeQuery();
            while (rs.next()) {
                animals.add(extractAnimalFromResultSet(rs));
            }
        }

        return animals;
    }

    /**
     * 로컬 동물 데이터를 WIA 형식으로 변환
     */
    public JSONObject transformAnimal(Animal animal) {
        JSONObject wiaAnimal = new JSONObject();

        // 기본 정보 변환
        JSONObject basicInfo = new JSONObject();
        basicInfo.put("name", animal.getName());
        basicInfo.put("species", mapSpecies(animal.getSpecies()));
        basicInfo.put("breed", animal.getBreed());
        basicInfo.put("sex", animal.getSex().toLowerCase());
        basicInfo.put("date_of_birth", animal.getDateOfBirth().toString());

        // 식별자 변환
        JSONObject identifiers = new JSONObject();
        if (animal.getMicrochipId() != null) {
            identifiers.put("microchip_id", animal.getMicrochipId());
        }

        // 현재 상태 변환
        JSONObject currentStatus = new JSONObject();
        currentStatus.put("status", mapStatus(animal.getStatus()));
        currentStatus.put("organization_id", "WIA-ORG-US-001234");

        wiaAnimal.put("basic_info", basicInfo);
        wiaAnimal.put("identifiers", identifiers);
        wiaAnimal.put("current_status", currentStatus);

        return wiaAnimal;
    }

    /**
     * 변환된 데이터를 WIA 시스템에 로드
     */
    public void loadAnimals(List<JSONObject> wiaAnimals) throws Exception {
        int successCount = 0;
        int errorCount = 0;

        for (JSONObject animal : wiaAnimals) {
            try {
                createOrUpdateAnimal(animal);
                successCount++;
            } catch (Exception e) {
                errorCount++;
                logError("동물 동기화 실패", animal, e);
            }

            // 속도 제한: 요청 간 대기
            Thread.sleep(100);
        }

        System.out.printf("배치 완료: %d 성공, %d 오류%n",
                          successCount, errorCount);
    }

    /**
     * WIA 시스템에서 동물 생성 또는 업데이트
     */
    private void createOrUpdateAnimal(JSONObject animal) throws Exception {
        String microchipId = animal.getJSONObject("identifiers")
                                   .optString("microchip_id");

        // 마이크로칩으로 동물이 존재하는지 확인
        String existingId = findAnimalByMicrochip(microchipId);

        if (existingId != null) {
            // 기존 항목 업데이트
            updateAnimal(existingId, animal);
        } else {
            // 새로 생성
            createAnimal(animal);
        }
    }

    /**
     * 전체 ETL 파이프라인 실행
     */
    public void runPipeline(Date since) throws Exception {
        System.out.println("ETL 파이프라인 시작...");

        // Extract
        System.out.println("로컬 데이터베이스에서 동물 추출 중...");
        List<Animal> localAnimals = extractAnimals(since);
        System.out.printf("%d마리의 동물 추출됨%n", localAnimals.size());

        // Transform
        System.out.println("WIA 형식으로 변환 중...");
        List<JSONObject> wiaAnimals = new ArrayList<>();
        for (Animal animal : localAnimals) {
            wiaAnimals.add(transformAnimal(animal));
        }

        // Load
        System.out.println("WIA 시스템에 로드 중...");
        loadAnimals(wiaAnimals);

        System.out.println("ETL 파이프라인 완료");
    }

    // 헬퍼 메서드
    private String mapSpecies(String localSpecies) {
        Map<String, String> speciesMap = new HashMap<>();
        speciesMap.put("Dog", "canis_lupus_familiaris");
        speciesMap.put("Cat", "felis_catus");
        speciesMap.put("Rabbit", "oryctolagus_cuniculus");
        return speciesMap.getOrDefault(localSpecies, localSpecies.toLowerCase());
    }

    private String mapStatus(String localStatus) {
        Map<String, String> statusMap = new HashMap<>();
        statusMap.put("Available", "shelter_care");
        statusMap.put("Adopted", "adopted");
        statusMap.put("Foster", "foster_care");
        statusMap.put("Hold", "hold");
        return statusMap.getOrDefault(localStatus, "shelter_care");
    }

    public static void main(String[] args) throws Exception {
        // 데이터베이스 연결
        Connection db = DriverManager.getConnection(
            "jdbc:postgresql://localhost/shelter_db",
            "user", "password"
        );

        WIABatchIntegration integration = new WIABatchIntegration(
            db,
            "https://api.wia.org/pet-welfare-global/v1",
            "your_api_key"
        );

        // 지난 24시간 동안 업데이트된 동물에 대해 파이프라인 실행
        Date yesterday = new Date(System.currentTimeMillis() - 86400000);
        integration.runPipeline(yesterday);
    }
}
```

### 3.4 메시지 큐 통합

#### 3.4.1 RabbitMQ 통합

```python
import pika
import json
import logging
from typing import Callable, Dict

class WIAMessageQueueIntegration:
    """
    비동기 WIA 데이터 처리를 위한 RabbitMQ 통합
    """

    def __init__(self, rabbitmq_url: str, exchange: str = 'wia.pet.welfare'):
        self.rabbitmq_url = rabbitmq_url
        self.exchange = exchange
        self.connection = None
        self.channel = None
        self.handlers = {}

    def connect(self):
        """RabbitMQ 연결 설정"""
        parameters = pika.URLParameters(self.rabbitmq_url)
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()

        # Exchange 선언
        self.channel.exchange_declare(
            exchange=self.exchange,
            exchange_type='topic',
            durable=True
        )

    def publish_event(self, routing_key: str, message: Dict):
        """메시지 큐에 이벤트 게시"""
        self.channel.basic_publish(
            exchange=self.exchange,
            routing_key=routing_key,
            body=json.dumps(message),
            properties=pika.BasicProperties(
                delivery_mode=2,  # 메시지를 영구적으로 만듦
                content_type='application/json'
            )
        )
        logging.info(f"이벤트 게시됨: {routing_key}")

    def subscribe(self, routing_key: str, handler: Callable):
        """라우팅 키와 일치하는 이벤트 구독"""
        # 큐 선언
        queue_name = f'wia.{routing_key}.queue'
        self.channel.queue_declare(queue=queue_name, durable=True)

        # 큐를 Exchange에 바인딩
        self.channel.queue_bind(
            exchange=self.exchange,
            queue=queue_name,
            routing_key=routing_key
        )

        # 핸들러 등록
        self.handlers[routing_key] = handler

        # 소비 시작
        self.channel.basic_consume(
            queue=queue_name,
            on_message_callback=self._handle_message,
            auto_ack=False
        )

    def _handle_message(self, ch, method, properties, body):
        """내부 메시지 핸들러"""
        try:
            message = json.loads(body)
            routing_key = method.routing_key

            if routing_key in self.handlers:
                self.handlers[routing_key](message)

            # 메시지 확인
            ch.basic_ack(delivery_tag=method.delivery_tag)

        except Exception as e:
            logging.error(f"메시지 처리 오류: {e}")
            # 메시지 거부 및 재시도
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=True)

    def start_consuming(self):
        """메시지 소비 시작"""
        logging.info("메시지 컨슈머 시작 중...")
        self.channel.start_consuming()

    def close(self):
        """연결 종료"""
        if self.connection:
            self.connection.close()


# 사용 예제
def handle_animal_created(message):
    print(f"동물 생성됨: {message['animal_id']}")
    # 동물 생성 처리
    # 로컬 데이터베이스에 동기화
    # 알림 전송

def handle_adoption_approved(message):
    print(f"입양 승인됨: {message['adoption_contract_id']}")
    # 동물 상태 업데이트
    # 입양자에게 알림
    # 운송 일정 예약

def handle_welfare_alert(message):
    print(f"복지 경고: 동물 {message['animal_id']} 점수: {message['score']}")
    # 직원에게 경고
    # 개입 작업 생성

# 통합 설정
mq = WIAMessageQueueIntegration('amqp://localhost')
mq.connect()

# 이벤트 구독
mq.subscribe('animal.created', handle_animal_created)
mq.subscribe('adoption.approved', handle_adoption_approved)
mq.subscribe('welfare.alert', handle_welfare_alert)

# 소비 시작 (차단됨)
mq.start_consuming()
```

---

## 보호소 관리 시스템

### 4.1 PetPoint 통합

#### 4.1.1 PetPoint에서 WIA로 동기화

```csharp
using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text.Json;
using System.Threading.Tasks;

public class PetPointWIAIntegration
{
    private readonly HttpClient _petPointClient;
    private readonly HttpClient _wiaClient;
    private readonly string _wiaApiKey;
    private readonly string _organizationId;

    public PetPointWIAIntegration(string petPointApiKey, string wiaApiKey,
                                   string organizationId)
    {
        _petPointClient = new HttpClient();
        _petPointClient.DefaultRequestHeaders.Add("Authorization",
            $"Bearer {petPointApiKey}");

        _wiaClient = new HttpClient();
        _wiaClient.DefaultRequestHeaders.Add("Authorization",
            $"Bearer {wiaApiKey}");
        _wiaClient.DefaultRequestHeaders.Add("X-WIA-Organization-ID",
            organizationId);

        _wiaApiKey = wiaApiKey;
        _organizationId = organizationId;
    }

    public async Task SyncAnimalsFromPetPoint()
    {
        // PetPoint에서 동물 가져오기
        var petPointAnimals = await FetchPetPointAnimals();

        foreach (var animal in petPointAnimals)
        {
            try
            {
                // WIA 형식으로 변환
                var wiaAnimal = TransformPetPointToWIA(animal);

                // WIA에서 생성 또는 업데이트
                await CreateOrUpdateWIAAnimal(wiaAnimal);

                Console.WriteLine($"동물 동기화됨: {animal.AnimalId}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"동물 {animal.AnimalId} 동기화 오류: {ex.Message}");
            }
        }
    }

    private async Task<List<PetPointAnimal>> FetchPetPointAnimals()
    {
        var response = await _petPointClient.GetAsync(
            "https://api.petpoint.com/v2/animals?status=available");

        response.EnsureSuccessStatusCode();

        var json = await response.Content.ReadAsStringAsync();
        return JsonSerializer.Deserialize<List<PetPointAnimal>>(json);
    }

    private WIAAnimal TransformPetPointToWIA(PetPointAnimal petPointAnimal)
    {
        return new WIAAnimal
        {
            Identifiers = new Identifiers
            {
                MicrochipId = petPointAnimal.MicrochipNumber,
                RegistrationNumber = petPointAnimal.AnimalId
            },
            BasicInfo = new BasicInfo
            {
                Name = petPointAnimal.Name,
                Species = MapSpecies(petPointAnimal.Species),
                Breed = petPointAnimal.PrimaryBreed,
                Sex = petPointAnimal.Sex.ToLower(),
                DateOfBirth = petPointAnimal.DateOfBirth,
                Color = petPointAnimal.Color
            },
            CurrentStatus = new CurrentStatus
            {
                Status = MapStatus(petPointAnimal.Status),
                OrganizationId = _organizationId,
                Since = petPointAnimal.IntakeDate,
                AvailableForAdoption = petPointAnimal.Status == "Available"
            },
            BehaviorProfile = new BehaviorProfile
            {
                Temperament = petPointAnimal.Temperament,
                Socialization = new Socialization
                {
                    GoodWithDogs = petPointAnimal.GoodWithDogs,
                    GoodWithCats = petPointAnimal.GoodWithCats,
                    GoodWithChildren = petPointAnimal.GoodWithKids
                }
            }
        };
    }

    private async Task CreateOrUpdateWIAAnimal(WIAAnimal animal)
    {
        // 마이크로칩으로 동물이 존재하는지 확인
        var existingId = await FindWIAAnimalByMicrochip(
            animal.Identifiers.MicrochipId);

        var content = new StringContent(
            JsonSerializer.Serialize(animal),
            System.Text.Encoding.UTF8,
            "application/json"
        );

        HttpResponseMessage response;

        if (existingId != null)
        {
            // 기존 항목 업데이트
            response = await _wiaClient.PatchAsync(
                $"https://api.wia.org/pet-welfare-global/v1/animals/{existingId}",
                content
            );
        }
        else
        {
            // 새로 생성
            response = await _wiaClient.PostAsync(
                "https://api.wia.org/pet-welfare-global/v1/animals",
                content
            );
        }

        response.EnsureSuccessStatusCode();
    }

    private string MapSpecies(string petPointSpecies)
    {
        return petPointSpecies.ToLower() switch
        {
            "dog" => "canis_lupus_familiaris",
            "cat" => "felis_catus",
            "rabbit" => "oryctolagus_cuniculus",
            _ => petPointSpecies.ToLower()
        };
    }

    private string MapStatus(string petPointStatus)
    {
        return petPointStatus switch
        {
            "Available" => "shelter_care",
            "Adopted" => "adopted",
            "Foster" => "foster_care",
            "Hold" => "hold",
            _ => "shelter_care"
        };
    }
}
```

---

## 정부 시스템

### 5.1 국가 반려동물 레지스트리 통합

```go
package main

import (
    "bytes"
    "encoding/json"
    "fmt"
    "net/http"
    "time"
)

type NationalRegistryIntegration struct {
    RegistryAPIKey string
    RegistryURL    string
    WIAAPIKey      string
    WIAURL         string
}

type RegistryAnimal struct {
    MicrochipID    string    `json:"microchip_id"`
    OwnerName      string    `json:"owner_name"`
    OwnerContact   string    `json:"owner_contact"`
    RegisteredDate time.Time `json:"registered_date"`
    AnimalType     string    `json:"animal_type"`
    Breed          string    `json:"breed"`
}

func (nri *NationalRegistryIntegration) RegisterAnimalWithGovernment(
    wiaAnimalID string) error {

    // WIA에서 동물 가져오기
    animal, err := nri.fetchWIAAnimal(wiaAnimalID)
    if err != nil {
        return fmt.Errorf("WIA 동물 가져오기 실패: %w", err)
    }

    // 레지스트리 형식으로 변환
    registryAnimal := nri.transformToRegistryFormat(animal)

    // 국가 레지스트리에 제출
    err = nri.submitToRegistry(registryAnimal)
    if err != nil {
        return fmt.Errorf("레지스트리 제출 실패: %w", err)
    }

    fmt.Printf("국가 레지스트리에 동물 %s 등록 성공\n", wiaAnimalID)
    return nil
}

func (nri *NationalRegistryIntegration) fetchWIAAnimal(animalID string) (
    map[string]interface{}, error) {

    url := fmt.Sprintf("%s/animals/%s", nri.WIAURL, animalID)

    req, err := http.NewRequest("GET", url, nil)
    if err != nil {
        return nil, err
    }

    req.Header.Set("Authorization", "Bearer "+nri.WIAAPIKey)

    client := &http.Client{Timeout: 10 * time.Second}
    resp, err := client.Do(req)
    if err != nil {
        return nil, err
    }
    defer resp.Body.Close()

    var result map[string]interface{}
    if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
        return nil, err
    }

    return result, nil
}

func (nri *NationalRegistryIntegration) transformToRegistryFormat(
    wiaAnimal map[string]interface{}) RegistryAnimal {

    data := wiaAnimal["data"].(map[string]interface{})
    profile := data["animal_profile"].(map[string]interface{})
    identifiers := profile["identifiers"].(map[string]interface{})
    basicInfo := profile["basic_info"].(map[string]interface{})

    return RegistryAnimal{
        MicrochipID:    identifiers["microchip_id"].(string),
        RegisteredDate: time.Now(),
        AnimalType:     basicInfo["species"].(string),
        Breed:          basicInfo["breed"].(string),
    }
}

func (nri *NationalRegistryIntegration) submitToRegistry(
    animal RegistryAnimal) error {

    jsonData, err := json.Marshal(animal)
    if err != nil {
        return err
    }

    req, err := http.NewRequest("POST",
        nri.RegistryURL+"/register",
        bytes.NewBuffer(jsonData))
    if err != nil {
        return err
    }

    req.Header.Set("Authorization", "Bearer "+nri.RegistryAPIKey)
    req.Header.Set("Content-Type", "application/json")

    client := &http.Client{Timeout: 30 * time.Second}
    resp, err := client.Do(req)
    if err != nil {
        return err
    }
    defer resp.Body.Close()

    if resp.StatusCode != http.StatusOK &&
       resp.StatusCode != http.StatusCreated {
        return fmt.Errorf("레지스트리가 상태 반환: %d", resp.StatusCode)
    }

    return nil
}

func main() {
    integration := &NationalRegistryIntegration{
        RegistryAPIKey: "registry_api_key",
        RegistryURL:    "https://nationalpetreg.gov/api/v1",
        WIAAPIKey:      "wia_api_key",
        WIAURL:         "https://api.wia.org/pet-welfare-global/v1",
    }

    err := integration.RegisterAnimalWithGovernment("WIA-PET-2025-000001")
    if err != nil {
        fmt.Printf("오류: %v\n", err)
    }
}
```

---

## 데이터 마이그레이션

### 7.1 마이그레이션 계획

| 단계 | 활동 | 기간 |
|------|------|------|
| **평가** | 데이터 감사, 매핑, 품질 분석 | 2-4주 |
| **설계** | 마이그레이션 전략, 도구 선택 | 2-3주 |
| **개발** | 마이그레이션 스크립트, 변환기 구축 | 4-6주 |
| **테스트** | 파일럿 마이그레이션, 검증 | 2-3주 |
| **실행** | 전체 마이그레이션, 모니터링 | 1-2주 |
| **검증** | 데이터 검증, 조정 | 1주 |

### 7.2 데이터 매핑 예제

| 소스 필드 (레거시) | 대상 필드 (WIA) | 변환 |
|-------------------|----------------|------|
| `animal_id` | `identifiers.registration_number` | 직접 복사 |
| `pet_name` | `basic_info.name` | 직접 복사 |
| `species_code` | `basic_info.species` | 조회 테이블로 매핑 |
| `breed_name` | `basic_info.breed` | 대문자 정규화 |
| `gender` | `basic_info.sex` | 소문자로 변환 |
| `birth_date` | `basic_info.date_of_birth` | ISO 날짜 형식 |
| `chip_number` | `identifiers.microchip_id` | 15자리 검증 |
| `current_location` | `current_status.organization_id` | WIA 조직 ID로 매핑 |

### 7.3 마이그레이션 검증 체크리스트

- [ ] 모든 필수 필드가 채워짐
- [ ] 데이터 타입이 올바름
- [ ] 참조 무결성 유지됨
- [ ] 중복 레코드 없음
- [ ] 마이크로칩 ID 검증됨 (15자리, ISO 준수)
- [ ] 날짜가 올바르게 형식화됨 (ISO 8601)
- [ ] 국가 코드가 유효함 (ISO 3166-1 alpha-2)
- [ ] 열거형 값이 허용된 집합 내에 있음
- [ ] 교차 참조가 해결됨
- [ ] 사진 및 문서가 마이그레이션됨
- [ ] 레거시 ID가 메타데이터에 보존됨

---

## 테스트 및 인증

### 8.1 규정 준수 테스트

```yaml
wia_compliance_test:
  organization: "WIA-ORG-US-001234"
  test_date: "2025-12-18"

  api_tests:
    - name: "인증"
      tests:
        - oauth2_flow
        - api_key_validation
        - token_expiration
      status: "passed"

    - name: "동물 관리"
      tests:
        - create_animal_profile
        - update_animal_profile
        - search_animals
        - delete_animal_profile
      status: "passed"

    - name: "데이터 형식 검증"
      tests:
        - json_schema_validation
        - required_fields_check
        - data_type_validation
        - enum_value_validation
      status: "passed"

  protocol_tests:
    - name: "국경 간 조정"
      tests:
        - organization_handshake
        - data_request_response
        - document_verification
      status: "passed"

  integration_tests:
    - name: "웹훅 전달"
      tests:
        - webhook_signature_verification
        - event_delivery
        - retry_logic
      status: "passed"

  certification_status: "compliant"
  certification_expires: "2026-12-18"
```

### 8.2 성능 테스트

| 테스트 | 목표 | 결과 | 상태 |
|--------|------|------|------|
| API 응답 시간 (p95) | < 200ms | 145ms | ✓ 통과 |
| 처리량 | > 1000 req/sec | 1250 req/sec | ✓ 통과 |
| 데이터베이스 쿼리 시간 | < 100ms | 78ms | ✓ 통과 |
| 웹훅 전달 | < 5 sec | 2.8 sec | ✓ 통과 |
| 동기화 지연 | < 10 sec | 6.2 sec | ✓ 통과 |
| 파일 업로드 (10MB) | < 30 sec | 22 sec | ✓ 통과 |

---

## 배포 전략

### 9.1 Blue-Green 배포

```yaml
blue_green_deployment:
  strategy: blue_green

  blue_environment:
    version: "1.0.0"
    status: "active"
    traffic_percentage: 100
    instances: 5

  green_environment:
    version: "1.1.0"
    status: "ready"
    traffic_percentage: 0
    instances: 5

  deployment_steps:
    - Green 환경에 새 버전 배포
    - Green에서 스모크 테스트 실행
    - Green으로 트래픽 10% 라우팅
    - 1시간 동안 메트릭 모니터링
    - 성공하면 트래픽 50% 라우팅
    - 30분 동안 모니터링
    - Green으로 트래픽 100% 라우팅
    - Blue를 대기 상태로 표시
```

### 9.2 카나리 배포

```yaml
canary_deployment:
  strategy: canary

  rollout_plan:
    - phase: 1
      traffic_percentage: 5
      duration: "2 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 2
      traffic_percentage: 25
      duration: "4 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 3
      traffic_percentage: 50
      duration: "8 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 4
      traffic_percentage: 100
      duration: "stable"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

  rollback_trigger:
    error_rate: "> 0.5%"
    latency_p95: "> 500ms"
    manual: true
```

---

## 코드 예제

### 예제 1: 완전한 통합 설정

```python
from wia_sdk import WIAClient
from legacy_system import LegacyDatabase

class CompleteWIAIntegration:
    def __init__(self):
        self.wia = WIAClient(
            api_key='your_api_key',
            organization_id='WIA-ORG-US-001234'
        )
        self.legacy_db = LegacyDatabase()

    def setup_integration(self):
        # 1. 초기 데이터 마이그레이션
        print("초기 데이터 마이그레이션 시작...")
        self.migrate_animals()
        self.migrate_vaccinations()

        # 2. 실시간 동기화 설정
        print("실시간 동기화 설정 중...")
        self.setup_webhooks()
        self.setup_event_listeners()

        # 3. 예약 작업 구성
        print("예약 작업 구성 중...")
        self.setup_daily_sync()
        self.setup_weekly_reports()

        print("통합 설정 완료!")

    def migrate_animals(self):
        animals = self.legacy_db.get_all_animals()
        for animal in animals:
            wia_animal = self.transform_animal(animal)
            self.wia.create_animal(wia_animal)

    def transform_animal(self, legacy_animal):
        return {
            'identifiers': {
                'microchip_id': legacy_animal.microchip,
                'registration_number': legacy_animal.id
            },
            'basic_info': {
                'name': legacy_animal.name,
                'species': self.map_species(legacy_animal.species),
                'breed': legacy_animal.breed
            }
        }

# 통합 실행
integration = CompleteWIAIntegration()
integration.setup_integration()
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
