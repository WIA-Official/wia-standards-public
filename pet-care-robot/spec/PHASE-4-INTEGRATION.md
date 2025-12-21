# Phase 4: Pet Care Robot Integration Specification

## WIA-PET-CARE-ROBOT Integration Standard

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE4-001
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

This specification defines integration patterns, deployment strategies, testing methodologies, and real-world implementation scenarios for pet care robot systems. It ensures seamless integration with existing ecosystems, platforms, and third-party services while maintaining security, reliability, and user experience.

**Integration Objectives**:
- Enable cross-platform compatibility
- Facilitate third-party integrations
- Support scalable deployment architectures
- Define comprehensive testing strategies
- Establish certification requirements
- Provide implementation best practices
- Enable ecosystem growth

### 1.2 Integration Layers

| Layer | Components | Purpose |
|-------|-----------|---------|
| **Hardware** | Robots, sensors, actuators | Physical pet care operations |
| **Edge** | Local controllers, gateways | Low-latency processing |
| **Cloud** | Backend services, databases | Centralized management |
| **Application** | Mobile apps, web dashboards | User interfaces |
| **Integration** | APIs, webhooks, connectors | Third-party connectivity |
| **Analytics** | ML models, reporting | Insights and intelligence |

### 1.3 Deployment Models

| Model | Description | Use Case | Complexity |
|-------|-------------|----------|------------|
| **Cloud-First** | Primary processing in cloud | Multi-location pet care facilities | Medium |
| **Edge-Primary** | Local processing, cloud backup | Home users, privacy-focused | Low |
| **Hybrid** | Balanced edge and cloud | Smart homes with multiple devices | Medium |
| **Enterprise** | On-premise + cloud | Veterinary clinics, pet hotels | High |

---

## 2. Smart Home Integration

### 2.1 Platform Support Matrix

| Platform | Voice Control | Automation | Notifications | Status |
|----------|--------------|------------|---------------|--------|
| **Amazon Alexa** | ✓ Full | ✓ Routines | ✓ Push | Production |
| **Google Home** | ✓ Full | ✓ Routines | ✓ Push | Production |
| **Apple HomeKit** | ✓ Siri | ✓ Scenes | ✓ Push | Beta |
| **Samsung SmartThings** | ✓ Bixby | ✓ Automations | ✓ Push | Production |
| **Home Assistant** | ✓ Custom | ✓ Full | ✓ MQTT | Production |
| **IFTTT** | - | ✓ Applets | ✓ Webhook | Production |

### 2.2 Alexa Integration Example

```javascript
// Alexa Smart Home Skill Handler
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
      friendlyName: `${robot.deviceType} in ${robot.location.room}`,
      description: `WIA Pet Care Robot - ${robot.deviceType}`,
      displayCategories: ['OTHER'],
      cookie: {},
      capabilities: [
        // Power Controller
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
        // Mode Controller for feeding
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
              { '@type': 'text', value: { text: 'Feeding Mode', locale: 'en-US' } }
            ]
          },
          configuration: {
            ordered: false,
            supportedModes: [
              {
                value: 'FeedingMode.Scheduled',
                modeResources: {
                  friendlyNames: [
                    { '@type': 'text', value: { text: 'Scheduled', locale: 'en-US' } }
                  ]
                }
              },
              {
                value: 'FeedingMode.Manual',
                modeResources: {
                  friendlyNames: [
                    { '@type': 'text', value: { text: 'Manual', locale: 'en-US' } }
                  ]
                }
              }
            ]
          }
        },
        // Endpoint Health
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
        speechText = `Okay, I've fed ${petName}`;
        break;

      case 'CheckFoodLevelIntent':
        const status = await api.getRobotStatus();
        speechText = `The food level is at ${status.foodLevel} percent`;
        break;

      case 'StartPlayTimeIntent':
        const playType = Alexa.getSlotValue(handlerInput.requestEnvelope, 'PlayType') || 'laser';
        await api.startPlaySession({ playType: playType });
        speechText = `Starting ${playType} play session`;
        break;

      default:
        speechText = 'I'm not sure how to help with that';
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

### 2.3 Google Home Integration

```python
from flask import Flask, request, jsonify
from google.oauth2.credentials import Credentials
from petcare_api import PetCareAPI

app = Flask(__name__)

@app.route('/smarthome', methods=['POST'])
def smarthome():
    """Handle Google Home requests"""
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
    """Discover devices"""
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
                'defaultNames': [f"Pet Care Robot {robot['robotId']}"],
                'name': f"{robot['deviceType']} - {robot['location']['room']}",
                'nicknames': ['pet feeder', 'food dispenser']
            },
            'willReportState': True,
            'attributes': {
                'supportedDispenseItems': [
                    {
                        'item_name': 'food',
                        'item_name_synonyms': [
                            {'lang': 'en', 'synonyms': ['kibble', 'pet food']}
                        ],
                        'supported_units': ['GRAMS'],
                        'default_portion': {
                            'amount': 100,
                            'unit': 'GRAMS'
                        }
                    }
                ],
                'supportedDispensePresets': [
                    {'preset_name': 'small_portion'},
                    {'preset_name': 'regular_portion'},
                    {'preset_name': 'large_portion'}
                ],
                'sensorStatesSupported': [
                    {'name': 'FoodLevel', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}},
                    {'name': 'WaterLevel', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}},
                    {'name': 'BatteryLevel', 'numericCapabilities': {'rawValueUnit': 'PERCENTAGE'}}
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
    """Execute commands"""
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
                    # Handle dispense command
                    preset = params.get('presetName')
                    amount = params.get('amount', 100)

                    if preset == 'small_portion':
                        amount = 50
                    elif preset == 'regular_portion':
                        amount = 100
                    elif preset == 'large_portion':
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
    """Query device state"""
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
                    'name': 'FoodLevel',
                    'currentSensorState': 'normal',
                    'rawValue': status['foodLevel']
                },
                {
                    'name': 'WaterLevel',
                    'currentSensorState': 'normal',
                    'rawValue': status['waterLevel']
                },
                {
                    'name': 'BatteryLevel',
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
    # Retrieve user's OAuth token from database
    return "user-oauth-token"

if __name__ == '__main__':
    app.run(debug=True, port=8080)
```

### 2.4 Home Assistant Integration

```yaml
# configuration.yaml
petcare:
  platform: wia_petcare
  api_key: !secret petcare_api_key
  api_secret: !secret petcare_api_secret
  scan_interval: 30

# Automation example
automation:
  - alias: "Feed pet when arriving home"
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

  - alias: "Low food alert"
    trigger:
      platform: numeric_state
      entity_id: sensor.petcare_food_level
      below: 20
    action:
      - service: notify.mobile_app
        data:
          title: "Pet Care Alert"
          message: "Food level is low ({{ states('sensor.petcare_food_level') }}%). Please refill soon."
          data:
            priority: high

  - alias: "Daily play session"
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

## 3. Veterinary System Integration

### 3.1 FHIR Integration for Pet Health

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
          "display": "Vital Signs"
        }
      ]
    }
  ],
  "code": {
    "coding": [
      {
        "system": "http://wia.org/petcare/codes",
        "code": "body-weight",
        "display": "Body Weight"
      }
    ]
  },
  "subject": {
    "reference": "Patient/PET-DOG001",
    "display": "Max (Golden Retriever)"
  },
  "effectiveDateTime": "2025-12-18T12:00:00Z",
  "issued": "2025-12-18T12:00:15Z",
  "performer": [
    {
      "reference": "Device/PCR-ABC123456789",
      "display": "PetCare Robot - Living Room"
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
      "text": "Automated weight measurement during feeding"
    }
  ]
}
```

### 3.2 Veterinary Clinic Integration

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
        """Sync health observations to veterinary system"""
        # Get health data from pet care robot
        observations = self.get_health_observations(pet_id, start_date, end_date)

        # Convert to veterinary system format
        vet_records = []
        for obs in observations:
            vet_record = self.convert_to_vet_format(obs)
            vet_records.append(vet_record)

        # Upload to veterinary system
        response = requests.post(
            f"{self.clinic_api_url}/api/v1/patients/{pet_id}/observations",
            headers=self.headers,
            json={'observations': vet_records}
        )

        return response.json()

    def convert_to_vet_format(self, observation: Dict) -> Dict:
        """Convert WIA pet care observation to veterinary format"""
        obs_type = observation['observationType']

        if obs_type == 'weight_measurement':
            return {
                'type': 'weight',
                'value': observation['weight']['value'],
                'unit': observation['weight']['unit'],
                'timestamp': observation['timestamp'],
                'source': 'automated_feeder',
                'notes': 'Measured during feeding routine'
            }
        elif obs_type == 'activity_level':
            return {
                'type': 'activity',
                'duration_minutes': observation['activityLevel']['duration'],
                'intensity': observation['activityLevel']['level'],
                'timestamp': observation['timestamp'],
                'source': 'play_robot',
                'notes': 'Activity tracking during play session'
            }
        elif obs_type == 'eating_behavior':
            return {
                'type': 'appetite',
                'level': observation['eatingBehavior']['appetiteLevel'],
                'consumption_rate': observation['eatingBehavior']['eatingSpeed'],
                'timestamp': observation['timestamp'],
                'source': 'automated_feeder',
                'notes': 'Eating behavior monitoring'
            }

    def get_vet_recommendations(self, pet_id: str) -> List[Dict]:
        """Retrieve veterinarian recommendations"""
        response = requests.get(
            f"{self.clinic_api_url}/api/v1/patients/{pet_id}/recommendations",
            headers=self.headers
        )

        return response.json()

    def send_alert_to_vet(self, pet_id: str, alert: Dict) -> Dict:
        """Send health alert to veterinarian"""
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
        """Calculate alert urgency based on severity and type"""
        severity = alert['severity']
        alert_type = alert['alertType']

        # High urgency conditions
        if severity == 'urgent':
            return 'immediate'
        elif severity == 'high' and alert_type in ['weight_change', 'behavioral_concern']:
            return 'within_24_hours'
        elif severity == 'medium':
            return 'within_week'
        else:
            return 'routine'

# Usage
vet_integration = VeterinaryIntegration(
    clinic_api_url='https://clinic.example.com',
    api_key='vet-api-key'
)

# Sync health data
result = vet_integration.sync_pet_health_data(
    pet_id='PET-DOG001',
    start_date='2025-12-11',
    end_date='2025-12-18'
)

# Send alert
alert_response = vet_integration.send_alert_to_vet(
    pet_id='PET-DOG001',
    alert={
        'alertType': 'weight_change',
        'severity': 'medium',
        'message': 'Weight increased by 5% over past week',
        'timestamp': datetime.now().isoformat()
    }
)
```

---

## 4. Third-Party Service Integration

### 4.1 Pet Food Delivery Services

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
    // Check food level every hour
    setInterval(async () => {
      const status = await this.petCareAPI.getRobotStatus(robotId);
      const foodLevel = status.foodLevel;

      if (foodLevel < 30) {
        await this.checkReorderNeed(robotId, foodLevel);
      }
    }, 3600000); // 1 hour
  }

  async checkReorderNeed(robotId: string, currentLevel: number): Promise<void> {
    // Calculate consumption rate
    const history = await this.petCareAPI.getFeedingHistory(robotId, 7); // Last 7 days
    const dailyConsumption = this.calculateDailyConsumption(history);

    // Predict depletion date
    const robot = await this.petCareAPI.getRobotDetails(robotId);
    const remainingGrams = (robot.capabilities.feeding.containerCapacity * currentLevel) / 100;
    const daysRemaining = remainingGrams / dailyConsumption;

    // Reorder if less than 5 days supply
    if (daysRemaining < 5) {
      await this.placeReorder(robotId, dailyConsumption * 30); // Order 30 days supply
    }
  }

  calculateDailyConsumption(history: any[]): number {
    const totalConsumed = history.reduce((sum, event) => sum + event.portionSize, 0);
    const days = history.length > 0 ? Math.max(1, history.length / 2) : 1; // Approximate days
    return totalConsumed / days;
  }

  async placeReorder(robotId: string, quantityGrams: number): Promise<any> {
    // Get pet food preferences
    const robot = await this.petCareAPI.getRobotDetails(robotId);
    const foodType = robot.capabilities.feeding.foodTypes[0];

    // Place order with delivery service
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

    // Notify user
    await this.petCareAPI.sendNotification({
      type: 'reorder_placed',
      message: `Automatic reorder placed: ${quantityGrams}g of ${foodType}`,
      orderId: orderResult.orderId,
      estimatedDelivery: orderResult.estimatedDelivery
    });

    return orderResult;
  }

  calculateDeliveryDate(): string {
    const date = new Date();
    date.setDate(date.getDate() + 3); // 3 days from now
    return date.toISOString().split('T')[0];
  }

  async getDeliveryAddress(robotId: string): Promise<any> {
    // Retrieve delivery address from user profile
    return {
      street: '123 Pet Lane',
      city: 'Pet City',
      state: 'PC',
      zip: '12345',
      country: 'US'
    };
  }
}

// Usage
const deliveryService: FoodDeliveryService = {
  providerId: 'chewy',
  apiEndpoint: 'https://api.chewy.com',
  supportsAutomation: true
};

const reorderingSystem = new AutomaticReorderingSystem(deliveryService, petCareAPI);
reorderingSystem.monitorFoodLevels('PCR-ABC123456789');
```

### 4.2 Pet Insurance Integration

```python
class PetInsuranceIntegration:
    def __init__(self, insurance_provider_api: str, policy_number: str):
        self.api_url = insurance_provider_api
        self.policy_number = policy_number

    def submit_wellness_data(self, pet_id: str, period: str) -> Dict:
        """Submit wellness data for insurance discounts"""
        # Collect wellness metrics
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

        # Submit to insurance provider
        response = requests.post(
            f"{self.api_url}/wellness/submit",
            json=metrics
        )

        return response.json()

    def calculate_activity_score(self, pet_id: str, period: str) -> float:
        """Calculate activity score (0-100)"""
        # Get play session data
        # Calculate based on frequency, duration, and intensity
        return 85.0  # Example score

    def calculate_feeding_consistency(self, pet_id: str, period: str) -> float:
        """Calculate feeding consistency score"""
        # Check adherence to feeding schedule
        return 95.0

    def calculate_weight_stability(self, pet_id: str, period: str) -> float:
        """Calculate weight stability score"""
        # Check weight variations
        return 92.0

    def calculate_play_engagement(self, pet_id: str, period: str) -> float:
        """Calculate play engagement score"""
        # Check play session engagement levels
        return 88.0

    def get_wellness_discount(self) -> Dict:
        """Get current wellness discount eligibility"""
        response = requests.get(
            f"{self.api_url}/wellness/discount/{self.policy_number}"
        )

        return response.json()
```

---

## 5. Deployment Architecture

### 5.1 Cloud Infrastructure (AWS Example)

```yaml
# Terraform configuration for AWS deployment
provider "aws" {
  region = "us-east-1"
}

# VPC Configuration
resource "aws_vpc" "petcare" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_hostnames = true
  enable_dns_support   = true

  tags = {
    Name = "petcare-vpc"
  }
}

# EKS Cluster for microservices
resource "aws_eks_cluster" "petcare" {
  name     = "petcare-cluster"
  role_arn = aws_iam_role.eks_cluster.arn

  vpc_config {
    subnet_ids = aws_subnet.private[*].id
  }
}

# RDS for PostgreSQL database
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

# ElastiCache for Redis (caching and sessions)
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

# IoT Core for MQTT
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

# S3 for media storage
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

# CloudFront for CDN
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

# Lambda for serverless functions
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

### 5.2 Kubernetes Deployment

```yaml
# petcare-api-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: petcare-api
  namespace: petcare
spec:
  replicas: 3
  selector:
    matchLabels:
      app: petcare-api
  template:
    metadata:
      labels:
        app: petcare-api
    spec:
      containers:
      - name: api
        image: petcare/api:1.0.0
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: petcare-secrets
              key: database-url
        - name: REDIS_URL
          valueFrom:
            secretKeyRef:
              name: petcare-secrets
              key: redis-url
        - name: MQTT_BROKER
          value: "mqtt.petcare.wia.org"
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
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
          initialDelaySeconds: 10
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: petcare-api
  namespace: petcare
spec:
  selector:
    app: petcare-api
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8080
  type: LoadBalancer
---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: petcare-api-hpa
  namespace: petcare
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: petcare-api
  minReplicas: 3
  maxReplicas: 10
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
```

---

## 6. Testing Strategy

### 6.1 Test Coverage Matrix

| Test Type | Coverage | Tools | Frequency |
|-----------|----------|-------|-----------|
| **Unit Tests** | 80%+ | Jest, PyTest | Every commit |
| **Integration Tests** | 70%+ | Postman, Newman | Every PR |
| **End-to-End Tests** | Critical paths | Cypress, Selenium | Daily |
| **Performance Tests** | API endpoints | JMeter, k6 | Weekly |
| **Security Tests** | All endpoints | OWASP ZAP, Burp | Weekly |
| **Load Tests** | Peak scenarios | Locust, Gatling | Monthly |
| **Chaos Tests** | Failure scenarios | Chaos Monkey | Monthly |

### 6.2 Integration Test Example

```python
import pytest
import requests
from datetime import datetime

class TestPetCareRobotIntegration:
    """Integration tests for pet care robot system"""

    @pytest.fixture
    def api_client(self):
        return PetCareAPIClient(
            base_url='https://api-staging.petcare.wia.org',
            api_key=os.getenv('TEST_API_KEY')
        )

    @pytest.fixture
    def test_robot_id(self):
        return 'PCR-TEST-123456789'

    @pytest.fixture
    def test_pet_id(self):
        return 'PET-TEST-DOG001'

    def test_end_to_end_feeding_workflow(self, api_client, test_robot_id, test_pet_id):
        """Test complete feeding workflow"""

        # Step 1: Create feeding schedule
        schedule = api_client.create_feeding_schedule({
            'robotId': test_robot_id,
            'petId': test_pet_id,
            'time': '12:00:00',
            'portionSize': 150,
            'recurring': True
        })

        assert schedule['scheduleId'] is not None
        assert schedule['enabled'] == True

        # Step 2: Trigger manual feeding
        feeding_result = api_client.dispense_food({
            'robotId': test_robot_id,
            'petId': test_pet_id,
            'portionSize': 100
        })

        assert feeding_result['dispensed'] == True
        assert feeding_result['eventId'] is not None

        # Step 3: Verify feeding history
        history = api_client.get_feeding_history(
            robotId=test_robot_id,
            startDate=datetime.now().date().isoformat()
        )

        assert len(history['data']) > 0
        assert history['data'][0]['eventId'] == feeding_result['eventId']

        # Step 4: Check robot status updated
        status = api_client.get_robot_status(test_robot_id)
        assert status['foodLevel'] < 100  # Food dispensed

        # Cleanup
        api_client.delete_feeding_schedule(schedule['scheduleId'])

    def test_play_session_integration(self, api_client, test_robot_id, test_pet_id):
        """Test play session workflow"""

        # Start play session
        session = api_client.start_play_session({
            'robotId': test_robot_id,
            'petId': test_pet_id,
            'playType': 'laser_chase',
            'duration': 300
        })

        assert session['sessionId'] is not None
        assert session['status'] == 'active'

        # Monitor session
        time.sleep(5)
        session_status = api_client.get_play_session_status(session['sessionId'])
        assert session_status['status'] == 'active'
        assert session_status['elapsedTime'] > 0

        # Stop session
        result = api_client.stop_play_session(session['sessionId'])
        assert result['status'] == 'completed'

        # Verify in history
        history = api_client.get_play_history(petId=test_pet_id)
        assert any(s['sessionId'] == session['sessionId'] for s in history['data'])

    def test_multi_robot_coordination(self, api_client):
        """Test multiple robots coordinating"""

        robot1_id = 'PCR-TEST-001'
        robot2_id = 'PCR-TEST-002'
        pet_id = 'PET-TEST-DOG001'

        # Both robots try to feed same pet simultaneously
        result1 = api_client.dispense_food({
            'robotId': robot1_id,
            'petId': pet_id,
            'portionSize': 100
        })

        result2 = api_client.dispense_food({
            'robotId': robot2_id,
            'petId': pet_id,
            'portionSize': 100
        })

        # Only one should succeed (resource locking)
        assert result1['dispensed'] != result2['dispensed']

    def test_offline_sync_workflow(self, api_client, test_robot_id):
        """Test offline operation and sync"""

        # Simulate offline operation
        offline_events = [
            {
                'eventId': 'FEED-OFFLINE-001',
                'timestamp': datetime.now().isoformat(),
                'type': 'feeding',
                'data': {'portionSize': 150}
            }
        ]

        # Sync offline events
        sync_result = api_client.sync_offline_data({
            'robotId': test_robot_id,
            'events': offline_events
        })

        assert sync_result['processed']['accepted'] == 1
        assert sync_result['status'] == 'success'

        # Verify synced data appears in history
        history = api_client.get_feeding_history(robotId=test_robot_id)
        assert any(e['eventId'] == 'FEED-OFFLINE-001' for e in history['data'])

    @pytest.mark.performance
    def test_api_response_time(self, api_client, test_robot_id):
        """Test API response times meet SLA"""

        import time

        # Test status endpoint
        start = time.time()
        api_client.get_robot_status(test_robot_id)
        duration = time.time() - start

        assert duration < 0.5  # Must respond in < 500ms

        # Test feeding endpoint
        start = time.time()
        api_client.dispense_food({
            'robotId': test_robot_id,
            'petId': 'PET-TEST-001',
            'portionSize': 100
        })
        duration = time.time() - start

        assert duration < 2.0  # Must complete in < 2s

# Run tests
if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
```

---

## 7. Certification and Compliance

### 7.1 Certification Requirements

| Certification | Scope | Required For | Validity |
|--------------|-------|--------------|----------|
| **WIA-PET-CARE-ROBOT** | Full standard compliance | All implementations | Annual |
| **Safety Certification** | Pet safety protocols | Hardware devices | 3 years |
| **Data Privacy** | GDPR, CCPA compliance | All implementations | Annual |
| **IoT Security** | Device security | Connected devices | 2 years |
| **API Compliance** | API standard adherence | API implementations | Annual |

### 7.2 Compliance Checklist

```markdown
## WIA-PET-CARE-ROBOT Compliance Checklist

### Phase 1: Data Format
- [ ] All data structures follow JSON schemas
- [ ] Robot IDs follow PCR-[A-Z0-9]{12} pattern
- [ ] Timestamps use ISO 8601 format
- [ ] All required fields present in messages
- [ ] Data validation implemented

### Phase 2: API Interface
- [ ] All required endpoints implemented
- [ ] OAuth 2.0 authentication supported
- [ ] Rate limiting implemented
- [ ] Error responses follow standard format
- [ ] API versioning in place
- [ ] Webhook support implemented

### Phase 3: Protocol
- [ ] MQTT 5.0 supported
- [ ] TLS 1.3 encryption enabled
- [ ] Offline operation supported
- [ ] Data synchronization implemented
- [ ] Multi-robot coordination available
- [ ] Resource locking implemented

### Phase 4: Integration
- [ ] At least 2 smart home platforms supported
- [ ] Veterinary data export available
- [ ] Third-party integrations documented
- [ ] Deployment architecture documented
- [ ] Integration tests ≥70% coverage
- [ ] Performance benchmarks met

### Security
- [ ] End-to-end encryption for sensitive data
- [ ] API request signing implemented
- [ ] Client certificate authentication available
- [ ] Security audit completed
- [ ] Penetration testing passed

### Safety
- [ ] Emergency stop mechanism implemented
- [ ] Pet detection and avoidance working
- [ ] Collision avoidance functional
- [ ] Maximum safe speeds enforced
- [ ] Failsafe mechanisms tested

### Privacy
- [ ] GDPR compliance verified
- [ ] Data retention policies implemented
- [ ] User consent mechanisms in place
- [ ] Data export/deletion supported
- [ ] Privacy policy published
```

---

## 8. Best Practices and Recommendations

### 8.1 Implementation Best Practices

| Area | Best Practice | Rationale |
|------|--------------|-----------|
| **Error Handling** | Implement retry logic with exponential backoff | Improves reliability in unstable networks |
| **Data Validation** | Validate all inputs at API boundary | Prevents invalid data propagation |
| **Logging** | Use structured logging with correlation IDs | Facilitates debugging and tracing |
| **Monitoring** | Implement health checks and metrics | Enables proactive issue detection |
| **Security** | Apply principle of least privilege | Minimizes security risk |
| **Testing** | Maintain 70%+ test coverage | Ensures code quality |
| **Documentation** | Keep API docs auto-generated from code | Ensures documentation accuracy |
| **Versioning** | Use semantic versioning | Clear communication of changes |

### 8.2 Performance Optimization

```python
# Example: Caching strategy for frequently accessed data
from functools import lru_cache
import redis

class PetCareCache:
    def __init__(self, redis_client):
        self.redis = redis_client
        self.ttl = 300  # 5 minutes

    def get_robot_status(self, robot_id: str) -> Dict:
        """Get robot status with caching"""
        cache_key = f"robot:status:{robot_id}"

        # Try cache first
        cached = self.redis.get(cache_key)
        if cached:
            return json.loads(cached)

        # Fetch from API
        status = self.fetch_robot_status(robot_id)

        # Cache result
        self.redis.setex(
            cache_key,
            self.ttl,
            json.dumps(status)
        )

        return status

    @lru_cache(maxsize=1000)
    def get_pet_profile(self, pet_id: str) -> Dict:
        """Get pet profile with in-memory caching"""
        return self.fetch_pet_profile(pet_id)

    def invalidate_cache(self, robot_id: str):
        """Invalidate cache when robot status changes"""
        cache_key = f"robot:status:{robot_id}"
        self.redis.delete(cache_key)
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
