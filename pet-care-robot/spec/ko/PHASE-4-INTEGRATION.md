# Phase 4: Pet Care Robot 통합 사양

## WIA-PET-CARE-ROBOT 통합 표준

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE4-001
**Primary Color**: #F59E0B (Amber)

---

## 1. 개요

### 1.1 목적

이 사양은 펫 케어 로봇 시스템의 통합 패턴, 배포 전략, 테스트 방법론 및 실제 구현 시나리오를 정의합니다. 보안, 안정성 및 사용자 경험을 유지하면서 기존 생태계, 플랫폼 및 타사 서비스와의 원활한 통합을 보장합니다.

**통합 목표**:
- 크로스 플랫폼 호환성 활성화
- 타사 통합 촉진
- 확장 가능한 배포 아키텍처 지원
- 포괄적인 테스트 전략 정의
- 인증 요구 사항 설정
- 구현 모범 사례 제공
- 생태계 성장 활성화

### 1.2 통합 계층

| 계층 | 구성 요소 | 목적 |
|------|----------|------|
| **하드웨어** | 로봇, 센서, 액추에이터 | 물리적 펫 케어 작업 |
| **엣지** | 로컬 컨트롤러, 게이트웨이 | 저지연 처리 |
| **클라우드** | 백엔드 서비스, 데이터베이스 | 중앙 집중식 관리 |
| **애플리케이션** | 모바일 앱, 웹 대시보드 | 사용자 인터페이스 |
| **통합** | API, webhook, 커넥터 | 타사 연결 |
| **분석** | ML 모델, 보고 | 인사이트 및 인텔리전스 |

### 1.3 배포 모델

| 모델 | 설명 | 사용 사례 | 복잡성 |
|------|------|----------|--------|
| **클라우드 우선** | 클라우드에서 주요 처리 | 다중 위치 펫 케어 시설 | 중간 |
| **엣지 우선** | 로컬 처리, 클라우드 백업 | 가정 사용자, 개인정보 중심 | 낮음 |
| **하이브리드** | 균형 잡힌 엣지 및 클라우드 | 여러 장치가 있는 스마트 홈 | 중간 |
| **엔터프라이즈** | 온프레미스 + 클라우드 | 수의학 클리닉, 펫 호텔 | 높음 |

---

## 2. 스마트 홈 통합

### 2.1 플랫폼 지원 매트릭스

| 플랫폼 | 음성 제어 | 자동화 | 알림 | 상태 |
|--------|----------|--------|------|------|
| **Amazon Alexa** | ✓ 전체 | ✓ 루틴 | ✓ 푸시 | 프로덕션 |
| **Google Home** | ✓ 전체 | ✓ 루틴 | ✓ 푸시 | 프로덕션 |
| **Apple HomeKit** | ✓ Siri | ✓ 장면 | ✓ 푸시 | 베타 |
| **Samsung SmartThings** | ✓ Bixby | ✓ 자동화 | ✓ 푸시 | 프로덕션 |
| **Home Assistant** | ✓ 커스텀 | ✓ 전체 | ✓ MQTT | 프로덕션 |
| **IFTTT** | - | ✓ 애플릿 | ✓ Webhook | 프로덕션 |

### 2.2 Alexa 통합 예제

```javascript
// Alexa 스마트 홈 스킬 핸들러
const Alexa = require('ask-sdk-core');
const PetCareAPI = require('./petcare-api');

const DiscoveryHandler = {
  canHandle(handlerInput) {
    return handlerInput.requestEnvelope.directive.header.name === 'Discover';
  },
  async handle(handlerInput) {
    const accessToken = handlerInput.requestEnvelope.directive.payload.scope.token;
    const api = new PetCareAPI(accessToken);
    const robots = await api.getRobots();

    const endpoints = robots.map(robot => ({
      endpointId: robot.robotId,
      manufacturerName: robot.manufacturer.name,
      friendlyName: `${robot.deviceType} ${robot.location.room}에 위치`,
      description: `WIA 펫 케어 로봇 - ${robot.deviceType}`,
      displayCategories: ['OTHER'],
      cookie: {},
      capabilities: [
        // 전원 컨트롤러
        {
          type: 'AlexaInterface',
          interface: 'Alexa.PowerController',
          version: '3',
          properties: {
            supported: [{ name: 'powerState' }],
            proactivelyReported: true,
            retrievable: true
          }
        },
        // 급식을 위한 모드 컨트롤러
        {
          type: 'AlexaInterface',
          interface: 'Alexa.ModeController',
          version: '3',
          instance: 'FeedingMode',
          properties: {
            supported: [{ name: 'mode' }],
            proactivelyReported: true,
            retrievable: true
          },
          capabilityResources: {
            friendlyNames: [
              { '@type': 'text', value: { text: '급식 모드', locale: 'ko-KR' } }
            ]
          },
          configuration: {
            ordered: false,
            supportedModes: [
              {
                value: 'FeedingMode.Scheduled',
                modeResources: {
                  friendlyNames: [
                    { '@type': 'text', value: { text: '예약', locale: 'ko-KR' } }
                  ]
                }
              },
              {
                value: 'FeedingMode.Manual',
                modeResources: {
                  friendlyNames: [
                    { '@type': 'text', value: { text: '수동', locale: 'ko-KR' } }
                  ]
                }
              }
            ]
          }
        },
        // 엔드포인트 상태
        {
          type: 'AlexaInterface',
          interface: 'Alexa.EndpointHealth',
          version: '3',
          properties: {
            supported: [{ name: 'connectivity' }],
            proactivelyReported: true,
            retrievable: true
          }
        }
      ]
    }));

    return {
      event: {
        header: {
          namespace: 'Alexa.Discovery',
          name: 'Discover.Response',
          payloadVersion: '3',
          messageId: handlerInput.requestEnvelope.directive.header.messageId
        },
        payload: {
          endpoints: endpoints
        }
      }
    };
  }
};

const PowerControllerHandler = {
  canHandle(handlerInput) {
    return handlerInput.requestEnvelope.directive.header.namespace === 'Alexa.PowerController';
  },
  async handle(handlerInput) {
    const directive = handlerInput.requestEnvelope.directive;
    const endpointId = directive.endpoint.endpointId;
    const powerState = directive.header.name === 'TurnOn' ? 'ON' : 'OFF';

    const accessToken = directive.endpoint.scope.token;
    const api = new PetCareAPI(accessToken);

    await api.setRobotStatus(endpointId, powerState === 'ON' ? 'online' : 'standby');

    return {
      event: {
        header: {
          namespace: 'Alexa',
          name: 'Response',
          payloadVersion: '3',
          messageId: directive.header.messageId,
          correlationToken: directive.header.correlationToken
        },
        endpoint: {
          endpointId: endpointId
        },
        payload: {}
      },
      context: {
        properties: [
          {
            namespace: 'Alexa.PowerController',
            name: 'powerState',
            value: powerState,
            timeOfSample: new Date().toISOString(),
            uncertaintyInMilliseconds: 500
          }
        ]
      }
    };
  }
};

const CustomIntentHandler = {
  canHandle(handlerInput) {
    return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest';
  },
  async handle(handlerInput) {
    const intentName = Alexa.getIntentName(handlerInput.requestEnvelope);
    const accessToken = handlerInput.requestEnvelope.context.System.user.accessToken;
    const api = new PetCareAPI(accessToken);

    let speechText = '';

    switch (intentName) {
      case 'FeedPetIntent':
        const petName = Alexa.getSlotValue(handlerInput.requestEnvelope, 'PetName');
        await api.dispenseFood({ petName: petName });
        speechText = `네, ${petName}에게 급식했습니다`;
        break;

      case 'CheckFoodLevelIntent':
        const status = await api.getRobotStatus();
        speechText = `음식 수준은 ${status.foodLevel} 퍼센트입니다`;
        break;

      case 'StartPlayTimeIntent':
        const playType = Alexa.getSlotValue(handlerInput.requestEnvelope, 'PlayType') || '레이저';
        await api.startPlaySession({ playType: playType });
        speechText = `${playType} 놀이 세션을 시작합니다`;
        break;

      default:
        speechText = '어떻게 도와드릴지 잘 모르겠습니다';
    }

    return handlerInput.responseBuilder
      .speak(speechText)
      .getResponse();
  }
};

exports.handler = Alexa.SkillBuilders.custom()
  .addRequestHandlers(
    DiscoveryHandler,
    PowerControllerHandler,
    CustomIntentHandler
  )
  .lambda();
```

### 2.3 Google Home 통합

Google Home 플랫폼과의 통합을 위한 Flask 기반 구현입니다.

```python
from flask import Flask, request, jsonify
from google.oauth2.credentials import Credentials
from petcare_api import PetCareAPI

app = Flask(__name__)

@app.route('/smarthome', methods=['POST'])
def smarthome():
    """Google Home 요청 처리"""
    request_json = request.get_json()
    inputs = request_json.get('inputs')
    intent = inputs[0].get('intent')

    if intent == 'action.devices.SYNC':
        return handle_sync(request_json)
    elif intent == 'action.devices.QUERY':
        return handle_query(request_json)
    elif intent == 'action.devices.EXECUTE':
        return handle_execute(request_json)
    elif intent == 'action.devices.DISCONNECT':
        return handle_disconnect(request_json)

def handle_sync(request_json):
    """장치 발견"""
    user_id = request_json['requestId']
    api = PetCareAPI(get_user_token(user_id))
    robots = api.get_robots()

    devices = []
    for robot in robots:
        devices.append({
            'id': robot['robotId'],
            'type': 'action.devices.types.PETFEEDER',
            'traits': [
                'action.devices.traits.OnOff',
                'action.devices.traits.Dispense',
                'action.devices.traits.SensorState'
            ],
            'name': {
                'defaultNames': [f"펫 케어 로봇 {robot['robotId']}"],
                'name': f"{robot['deviceType']} - {robot['location']['room']}",
                'nicknames': ['펫 급식기', '음식 디스펜서']
            },
            'willReportState': True,
            'attributes': {
                'supportedDispenseItems': [
                    {
                        'item_name': 'food',
                        'item_name_synonyms': [
                            {'lang': 'ko', 'synonyms': ['사료', '음식']}
                        ],
                        'supported_units': ['GRAMS'],
                        'default_portion': {
                            'amount': 100,
                            'unit': 'GRAMS'
                        }
                    }
                ],
                'supportedDispensePresets': [
                    {'preset_name': '소량'},
                    {'preset_name': '보통'},
                    {'preset_name': '대량'}
                ],
                'sensorStatesSupported': [
                    {'name': '음식수준', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}},
                    {'name': '물수준', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}},
                    {'name': '배터리수준', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}}
                ]
            },
            'deviceInfo': {
                'manufacturer': robot['manufacturer']['name'],
                'model': robot['manufacturer']['model'],
                'hwVersion': robot['manufacturer']['hardwareRevision'],
                'swVersion': robot['manufacturer']['firmwareVersion']
            }
        })

    return jsonify({
        'requestId': request_json['requestId'],
        'payload': {
            'agentUserId': user_id,
            'devices': devices
        }
    })

def handle_execute(request_json):
    """명령 실행"""
    user_id = request_json['requestId']
    api = PetCareAPI(get_user_token(user_id))

    commands = []
    for command in request_json['inputs'][0]['payload']['commands']:
        for device in command['devices']:
            device_id = device['id']

            for execution in command['execution']:
                command_name = execution['command']
                params = execution.get('params', {})

                if command_name == 'action.devices.commands.Dispense':
                    # 분배 명령 처리
                    preset = params.get('presetName')
                    amount = params.get('amount', 100)

                    if preset == '소량':
                        amount = 50
                    elif preset == '보통':
                        amount = 100
                    elif preset == '대량':
                        amount = 150

                    result = api.dispense_food(device_id, amount)

                    commands.append({
                        'ids': [device_id],
                        'status': 'SUCCESS' if result['dispensed'] else 'ERROR',
                        'states': {
                            'dispenseRemaining': 0,
                            'isDispensing': False
                        }
                    })

                elif command_name == 'action.devices.commands.OnOff':
                    on = params.get('on', True)
                    api.set_robot_status(device_id, 'online' if on else 'standby')

                    commands.append({
                        'ids': [device_id],
                        'status': 'SUCCESS',
                        'states': {'on': on}
                    })

    return jsonify({
        'requestId': request_json['requestId'],
        'payload': {'commands': commands}
    })

def handle_query(request_json):
    """장치 상태 조회"""
    user_id = request_json['requestId']
    api = PetCareAPI(get_user_token(user_id))

    devices = {}
    for device in request_json['inputs'][0]['payload']['devices']:
        device_id = device['id']
        status = api.get_robot_status(device_id)

        devices[device_id] = {
            'on': status['operational'] == 'online',
            'online': True,
            'currentSensorStateData': [
                {
                    'name': '음식수준',
                    'currentSensorState': 'normal',
                    'rawValue': status['foodLevel']
                },
                {
                    'name': '물수준',
                    'currentSensorState': 'normal',
                    'rawValue': status['waterLevel']
                },
                {
                    'name': '배터리수준',
                    'currentSensorState': 'normal',
                    'rawValue': status['batteryLevel']
                }
            ]
        }

    return jsonify({
        'requestId': request_json['requestId'],
        'payload': {'devices': devices}
    })

def get_user_token(user_id):
    # 데이터베이스에서 사용자의 OAuth 토큰 조회
    return "user-oauth-token"

if __name__ == '__main__':
    app.run(debug=True, port=8080)
```

### 2.4 Home Assistant 통합

```yaml
# configuration.yaml
petcare:
  platform: wia_petcare
  api_key: !secret petcare_api_key
  api_secret: !secret petcare_api_secret
  scan_interval: 30

# 자동화 예제
automation:
  - alias: "집에 도착하면 반려동물 급식"
    trigger:
      platform: state
      entity_id: person.owner
      to: "home"
    condition:
      - condition: time
        after: "17:00:00"
        before: "20:00:00"
      - condition: template
        value_template: "{{ states('sensor.last_feeding_time') | as_timestamp < (now() - timedelta(hours=4)) | as_timestamp }}"
    action:
      - service: petcare.feed
        data:
          robot_id: "PCR-ABC123456789"
          pet_id: "PET-DOG001"
          portion_size: 150

  - alias: "음식 부족 경고"
    trigger:
      platform: numeric_state
      entity_id: sensor.petcare_food_level
      below: 20
    action:
      - service: notify.mobile_app
        data:
          title: "펫 케어 경고"
          message: "음식 수준이 낮습니다 ({{ states('sensor.petcare_food_level') }}%). 곧 보충하세요."
          data:
            priority: high

  - alias: "매일 놀이 시간"
    trigger:
      platform: time
      at: "15:00:00"
    action:
      - service: petcare.start_play
        data:
          robot_id: "PCR-ABC123456789"
          pet_id: "PET-DOG001"
          play_type: "laser_chase"
          duration: 600
```

---

## 3. 수의학 시스템 통합

### 3.1 반려동물 건강을 위한 FHIR 통합

```json
{
  "resourceType": "Observation",
  "id": "pet-weight-001",
  "status": "final",
  "category": [
    {
      "coding": [
        {
          "system": "http://terminology.hl7.org/CodeSystem/observation-category",
          "code": "vital-signs",
          "display": "활력 징후"
        }
      ]
    }
  ],
  "code": {
    "coding": [
      {
        "system": "http://wia.org/petcare/codes",
        "code": "body-weight",
        "display": "체중"
      }
    ]
  },
  "subject": {
    "reference": "Patient/PET-DOG001",
    "display": "맥스 (골든 리트리버)"
  },
  "effectiveDateTime": "2025-12-18T12:00:00Z",
  "issued": "2025-12-18T12:00:15Z",
  "performer": [
    {
      "reference": "Device/PCR-ABC123456789",
      "display": "펫 케어 로봇 - 거실"
    }
  ],
  "valueQuantity": {
    "value": 22.5,
    "unit": "kg",
    "system": "http://unitsofmeasure.org",
    "code": "kg"
  },
  "note": [
    {
      "text": "급식 중 자동 체중 측정"
    }
  ]
}
```

### 3.2 수의학 클리닉 통합

```python
from typing import List, Dict
import requests
from datetime import datetime

class VeterinaryIntegration:
    def __init__(self, clinic_api_url: str, api_key: str):
        self.clinic_api_url = clinic_api_url
        self.api_key = api_key
        self.headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }

    def sync_pet_health_data(self, pet_id: str, start_date: str, end_date: str) -> Dict:
        """건강 관찰을 수의학 시스템에 동기화"""
        # 펫 케어 로봇에서 건강 데이터 가져오기
        observations = self.get_health_observations(pet_id, start_date, end_date)

        # 수의학 시스템 형식으로 변환
        vet_records = []
        for obs in observations:
            vet_record = self.convert_to_vet_format(obs)
            vet_records.append(vet_record)

        # 수의학 시스템에 업로드
        response = requests.post(
            f"{self.clinic_api_url}/api/v1/patients/{pet_id}/observations",
            headers=self.headers,
            json={'observations': vet_records}
        )

        return response.json()

    def convert_to_vet_format(self, observation: Dict) -> Dict:
        """WIA 펫 케어 관찰을 수의학 형식으로 변환"""
        obs_type = observation['observationType']

        if obs_type == 'weight_measurement':
            return {
                'type': 'weight',
                'value': observation['weight']['value'],
                'unit': observation['weight']['unit'],
                'timestamp': observation['timestamp'],
                'source': 'automated_feeder',
                'notes': '급식 루틴 중 측정됨'
            }
        elif obs_type == 'activity_level':
            return {
                'type': 'activity',
                'duration_minutes': observation['activityLevel']['duration'],
                'intensity': observation['activityLevel']['level'],
                'timestamp': observation['timestamp'],
                'source': 'play_robot',
                'notes': '놀이 세션 중 활동 추적'
            }
        elif obs_type == 'eating_behavior':
            return {
                'type': 'appetite',
                'level': observation['eatingBehavior']['appetiteLevel'],
                'consumption_rate': observation['eatingBehavior']['eatingSpeed'],
                'timestamp': observation['timestamp'],
                'source': 'automated_feeder',
                'notes': '섭식 행동 모니터링'
            }

    def get_vet_recommendations(self, pet_id: str) -> List[Dict]:
        """수의사 권장 사항 조회"""
        response = requests.get(
            f"{self.clinic_api_url}/api/v1/patients/{pet_id}/recommendations",
            headers=self.headers
        )

        return response.json()

    def send_alert_to_vet(self, pet_id: str, alert: Dict) -> Dict:
        """수의사에게 건강 경고 전송"""
        alert_data = {
            'petId': pet_id,
            'alertType': alert['alertType'],
            'severity': alert['severity'],
            'message': alert['message'],
            'timestamp': alert['timestamp'],
            'relatedData': alert.get('relatedObservations', []),
            'urgency': self.calculate_urgency(alert)
        }

        response = requests.post(
            f"{self.clinic_api_url}/api/v1/alerts",
            headers=self.headers,
            json=alert_data
        )

        return response.json()

    def calculate_urgency(self, alert: Dict) -> str:
        """경고 심각도 및 유형에 따라 긴급성 계산"""
        severity = alert['severity']
        alert_type = alert['alertType']

        # 높은 긴급성 조건
        if severity == 'urgent':
            return 'immediate'
        elif severity == 'high' and alert_type in ['weight_change', 'behavioral_concern']:
            return 'within_24_hours'
        elif severity == 'medium':
            return 'within_week'
        else:
            return 'routine'

# 사용
vet_integration = VeterinaryIntegration(
    clinic_api_url='https://clinic.example.com',
    api_key='vet-api-key'
)

# 건강 데이터 동기화
result = vet_integration.sync_pet_health_data(
    pet_id='PET-DOG001',
    start_date='2025-12-11',
    end_date='2025-12-18'
)

# 경고 전송
alert_response = vet_integration.send_alert_to_vet(
    pet_id='PET-DOG001',
    alert={
        'alertType': 'weight_change',
        'severity': 'medium',
        'message': '지난 주 동안 체중이 5% 증가했습니다',
        'timestamp': datetime.now().isoformat()
    }
)
```

---

## 4. 타사 서비스 통합

### 4.1 펫 음식 배송 서비스

자동 재주문 시스템을 통해 음식 공급이 부족해지기 전에 자동으로 주문합니다.

```typescript
interface FoodDeliveryService {
  providerId: string;
  apiEndpoint: string;
  supportsAutomation: boolean;
}

class AutomaticReorderingSystem {
  private deliveryService: FoodDeliveryService;
  private petCareAPI: any;

  constructor(deliveryService: FoodDeliveryService, petCareAPI: any) {
    this.deliveryService = deliveryService;
    this.petCareAPI = petCareAPI;
  }

  async monitorFoodLevels(robotId: string): Promise<void> {
    // 1시간마다 음식 수준 확인
    setInterval(async () => {
      const status = await this.petCareAPI.getRobotStatus(robotId);
      const foodLevel = status.foodLevel;

      if (foodLevel < 30) {
        await this.checkReorderNeed(robotId, foodLevel);
      }
    }, 3600000); // 1시간
  }

  async checkReorderNeed(robotId: string, currentLevel: number): Promise<void> {
    // 소비율 계산
    const history = await this.petCareAPI.getFeedingHistory(robotId, 7); // 지난 7일
    const dailyConsumption = this.calculateDailyConsumption(history);

    // 고갈 날짜 예측
    const robot = await this.petCareAPI.getRobotDetails(robotId);
    const remainingGrams = (robot.capabilities.feeding.containerCapacity * currentLevel) / 100;
    const daysRemaining = remainingGrams / dailyConsumption;

    // 5일 미만 공급이면 재주문
    if (daysRemaining < 5) {
      await this.placeReorder(robotId, dailyConsumption * 30); // 30일 공급 주문
    }
  }

  calculateDailyConsumption(history: any[]): number {
    const totalConsumed = history.reduce((sum, event) => sum + event.portionSize, 0);
    const days = history.length > 0 ? Math.max(1, history.length / 2) : 1; // 대략적인 일수
    return totalConsumed / days;
  }

  async placeReorder(robotId: string, quantityGrams: number): Promise<any> {
    // 펫 음식 선호도 가져오기
    const robot = await this.petCareAPI.getRobotDetails(robotId);
    const foodType = robot.capabilities.feeding.foodTypes[0];

    // 배송 서비스로 주문
    const order = {
      productType: foodType,
      quantity: quantityGrams,
      deliveryAddress: await this.getDeliveryAddress(robotId),
      urgency: 'standard',
      scheduledDelivery: this.calculateDeliveryDate()
    };

    const response = await fetch(`${this.deliveryService.apiEndpoint}/orders`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${process.env.DELIVERY_API_KEY}`
      },
      body: JSON.stringify(order)
    });

    const orderResult = await response.json();

    // 사용자에게 알림
    await this.petCareAPI.sendNotification({
      type: 'reorder_placed',
      message: `자동 재주문 완료: ${quantityGrams}g의 ${foodType}`,
      orderId: orderResult.orderId,
      estimatedDelivery: orderResult.estimatedDelivery
    });

    return orderResult;
  }

  calculateDeliveryDate(): string {
    const date = new Date();
    date.setDate(date.getDate() + 3); // 지금부터 3일 후
    return date.toISOString().split('T')[0];
  }

  async getDeliveryAddress(robotId: string): Promise<any> {
    // 사용자 프로필에서 배송 주소 조회
    return {
      street: '123 Pet Lane',
      city: 'Pet City',
      state: 'PC',
      zip: '12345',
      country: 'KR'
    };
  }
}

// 사용
const deliveryService: FoodDeliveryService = {
  providerId: 'chewy',
  apiEndpoint: 'https://api.chewy.com',
  supportsAutomation: true
};

const reorderingSystem = new AutomaticReorderingSystem(deliveryService, petCareAPI);
reorderingSystem.monitorFoodLevels('PCR-ABC123456789');
```

### 4.2 펫 보험 통합

```python
class PetInsuranceIntegration:
    def __init__(self, insurance_provider_api: str, policy_number: str):
        self.api_url = insurance_provider_api
        self.policy_number = policy_number

    def submit_wellness_data(self, pet_id: str, period: str) -> Dict:
        """보험 할인을 위한 웰니스 데이터 제출"""
        # 웰니스 메트릭 수집
        metrics = {
            'policy_number': self.policy_number,
            'pet_id': pet_id,
            'period': period,
            'metrics': {
                'activity_score': self.calculate_activity_score(pet_id, period),
                'feeding_consistency': self.calculate_feeding_consistency(pet_id, period),
                'weight_stability': self.calculate_weight_stability(pet_id, period),
                'play_engagement': self.calculate_play_engagement(pet_id, period)
            },
            'preventive_care': {
                'regular_feeding': True,
                'exercise_routine': True,
                'health_monitoring': True
            }
        }

        # 보험 제공자에게 제출
        response = requests.post(
            f"{self.api_url}/wellness/submit",
            json=metrics
        )

        return response.json()

    def calculate_activity_score(self, pet_id: str, period: str) -> float:
        """활동 점수 계산 (0-100)"""
        # 놀이 세션 데이터 가져오기
        # 빈도, 지속 시간 및 강도를 기반으로 계산
        return 85.0  # 예제 점수

    def calculate_feeding_consistency(self, pet_id: str, period: str) -> float:
        """급식 일관성 점수 계산"""
        # 급식 일정 준수 확인
        return 95.0

    def calculate_weight_stability(self, pet_id: str, period: str) -> float:
        """체중 안정성 점수 계산"""
        # 체중 변동 확인
        return 92.0

    def calculate_play_engagement(self, pet_id: str, period: str) -> float:
        """놀이 참여도 점수 계산"""
        # 놀이 세션 참여도 수준 확인
        return 88.0

    def get_wellness_discount(self) -> Dict:
        """현재 웰니스 할인 자격 조회"""
        response = requests.get(
            f"{self.api_url}/wellness/discount/{self.policy_number}"
        )

        return response.json()
```

---

## 5. 배포 아키텍처

### 5.1 클라우드 인프라 (AWS 예제)

```yaml
# AWS 배포를 위한 Terraform 구성
provider "aws" {
  region = "ap-northeast-2"  # 서울 리전
}

# VPC 구성
resource "aws_vpc" "petcare" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_hostnames = true
  enable_dns_support   = true

  tags = {
    Name = "petcare-vpc"
  }
}

# 마이크로서비스를 위한 EKS 클러스터
resource "aws_eks_cluster" "petcare" {
  name     = "petcare-cluster"
  role_arn = aws_iam_role.eks_cluster.arn

  vpc_config {
    subnet_ids = aws_subnet.private[*].id
  }
}

# PostgreSQL 데이터베이스를 위한 RDS
resource "aws_db_instance" "petcare" {
  identifier           = "petcare-db"
  engine              = "postgres"
  engine_version      = "15.3"
  instance_class      = "db.t3.medium"
  allocated_storage   = 100
  storage_encrypted   = true
  db_name             = "petcare"
  username            = "petcare_admin"
  password            = var.db_password
  multi_az            = true
  backup_retention_period = 7

  vpc_security_group_ids = [aws_security_group.database.id]
  db_subnet_group_name   = aws_db_subnet_group.petcare.name
}

# Redis를 위한 ElastiCache (캐싱 및 세션)
resource "aws_elasticache_cluster" "petcare" {
  cluster_id           = "petcare-redis"
  engine               = "redis"
  engine_version       = "7.0"
  node_type            = "cache.t3.medium"
  num_cache_nodes      = 2
  parameter_group_name = "default.redis7"
  port                 = 6379

  subnet_group_name    = aws_elasticache_subnet_group.petcare.name
  security_group_ids   = [aws_security_group.redis.id]
}

# MQTT를 위한 IoT Core
resource "aws_iot_policy" "robot_policy" {
  name = "petcare-robot-policy"

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "iot:Connect",
          "iot:Publish",
          "iot:Subscribe",
          "iot:Receive"
        ]
        Resource = "*"
      }
    ]
  })
}

# 미디어 저장을 위한 S3
resource "aws_s3_bucket" "media" {
  bucket = "petcare-media-${var.environment}"

  versioning {
    enabled = true
  }

  lifecycle_rule {
    enabled = true

    transition {
      days          = 30
      storage_class = "STANDARD_IA"
    }

    transition {
      days          = 90
      storage_class = "GLACIER"
    }
  }
}

# CDN을 위한 CloudFront
resource "aws_cloudfront_distribution" "media" {
  origin {
    domain_name = aws_s3_bucket.media.bucket_regional_domain_name
    origin_id   = "S3-petcare-media"
  }

  enabled             = true
  default_cache_behavior {
    allowed_methods  = ["GET", "HEAD"]
    cached_methods   = ["GET", "HEAD"]
    target_origin_id = "S3-petcare-media"

    forwarded_values {
      query_string = false
      cookies {
        forward = "none"
      }
    }

    viewer_protocol_policy = "redirect-to-https"
    min_ttl                = 0
    default_ttl            = 3600
    max_ttl                = 86400
  }
}

# 서버리스 함수를 위한 Lambda
resource "aws_lambda_function" "webhook_processor" {
  filename      = "webhook_processor.zip"
  function_name = "petcare-webhook-processor"
  role          = aws_iam_role.lambda_exec.arn
  handler       = "index.handler"
  runtime       = "nodejs18.x"
  timeout       = 30

  environment {
    variables = {
      DB_HOST = aws_db_instance.petcare.endpoint
      REDIS_HOST = aws_elasticache_cluster.petcare.cache_nodes[0].address
    }
  }
}
```

---

## 6. 테스트 전략

### 6.1 테스트 커버리지 매트릭스

| 테스트 유형 | 커버리지 | 도구 | 빈도 |
|------------|----------|------|------|
| **단위 테스트** | 80%+ | Jest, PyTest | 모든 커밋 |
| **통합 테스트** | 70%+ | Postman, Newman | 모든 PR |
| **엔드투엔드 테스트** | 핵심 경로 | Cypress, Selenium | 매일 |
| **성능 테스트** | API 엔드포인트 | JMeter, k6 | 주간 |
| **보안 테스트** | 모든 엔드포인트 | OWASP ZAP, Burp | 주간 |
| **부하 테스트** | 피크 시나리오 | Locust, Gatling | 월간 |
| **카오스 테스트** | 장애 시나리오 | Chaos Monkey | 월간 |

---

## 7. 인증 및 준수

### 7.1 인증 요구 사항

| 인증 | 범위 | 필수 대상 | 유효성 |
|------|------|----------|--------|
| **WIA-PET-CARE-ROBOT** | 전체 표준 준수 | 모든 구현 | 연간 |
| **안전 인증** | 반려동물 안전 프로토콜 | 하드웨어 장치 | 3년 |
| **데이터 개인정보** | GDPR, CCPA 준수 | 모든 구현 | 연간 |
| **IoT 보안** | 장치 보안 | 연결된 장치 | 2년 |
| **API 준수** | API 표준 준수 | API 구현 | 연간 |

### 7.2 준수 체크리스트

```markdown
## WIA-PET-CARE-ROBOT 준수 체크리스트

### Phase 1: 데이터 포맷
- [ ] 모든 데이터 구조가 JSON 스키마를 따름
- [ ] 로봇 ID가 PCR-[A-Z0-9]{12} 패턴을 따름
- [ ] 타임스탬프가 ISO 8601 형식 사용
- [ ] 메시지에 모든 필수 필드 존재
- [ ] 데이터 검증 구현

### Phase 2: API 인터페이스
- [ ] 모든 필수 엔드포인트 구현
- [ ] OAuth 2.0 인증 지원
- [ ] 속도 제한 구현
- [ ] 오류 응답이 표준 형식을 따름
- [ ] API 버전 관리 적용
- [ ] Webhook 지원 구현

### Phase 3: 프로토콜
- [ ] MQTT 5.0 지원
- [ ] TLS 1.3 암호화 활성화
- [ ] 오프라인 작동 지원
- [ ] 데이터 동기화 구현
- [ ] 다중 로봇 조정 가능
- [ ] 리소스 잠금 구현

### Phase 4: 통합
- [ ] 최소 2개 스마트 홈 플랫폼 지원
- [ ] 수의학 데이터 내보내기 가능
- [ ] 타사 통합 문서화
- [ ] 배포 아키텍처 문서화
- [ ] 통합 테스트 ≥70% 커버리지
- [ ] 성능 벤치마크 충족

### 보안
- [ ] 민감한 데이터에 대한 종단간 암호화
- [ ] API 요청 서명 구현
- [ ] 클라이언트 인증서 인증 가능
- [ ] 보안 감사 완료
- [ ] 침투 테스트 통과

### 안전
- [ ] 비상 정지 메커니즘 구현
- [ ] 반려동물 감지 및 회피 작동
- [ ] 충돌 회피 기능
- [ ] 최대 안전 속도 적용
- [ ] 페일세이프 메커니즘 테스트

### 개인정보
- [ ] GDPR 준수 확인
- [ ] 데이터 보존 정책 구현
- [ ] 사용자 동의 메커니즘 적용
- [ ] 데이터 내보내기/삭제 지원
- [ ] 개인정보 처리방침 게시
```

---

## 8. 모범 사례 및 권장 사항

### 8.1 구현 모범 사례

| 영역 | 모범 사례 | 근거 |
|------|----------|------|
| **오류 처리** | 지수 백오프를 사용한 재시도 로직 구현 | 불안정한 네트워크에서 안정성 향상 |
| **데이터 검증** | API 경계에서 모든 입력 검증 | 잘못된 데이터 전파 방지 |
| **로깅** | 상관 ID를 사용한 구조화된 로깅 | 디버깅 및 추적 촉진 |
| **모니터링** | 상태 확인 및 메트릭 구현 | 사전 문제 감지 활성화 |
| **보안** | 최소 권한 원칙 적용 | 보안 위험 최소화 |
| **테스트** | 70%+ 테스트 커버리지 유지 | 코드 품질 보장 |
| **문서화** | 코드에서 자동 생성된 API 문서 유지 | 문서 정확성 보장 |
| **버전 관리** | 의미론적 버전 관리 사용 | 변경 사항의 명확한 커뮤니케이션 |

---

**弘益人間 (홍익인간)** - 인류와 모든 생명체의 이익을 위하여
© 2025 WIA
MIT License
