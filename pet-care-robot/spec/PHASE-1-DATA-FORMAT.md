# Phase 1: Pet Care Robot Data Format Specification

## WIA-PET-CARE-ROBOT Data Format Standard

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE1-001
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

WIA-PET-CARE-ROBOT is a comprehensive standard for autonomous and semi-autonomous robotic systems designed to provide care, enrichment, and monitoring for companion animals. This standard enables interoperability between pet care robots, smart home systems, veterinary platforms, and pet owner applications.

**Core Objectives**:
- Standardize robot device specifications and capabilities
- Enable automatic feeding, watering, and waste management
- Support interactive play and exercise automation
- Integrate health monitoring and behavioral tracking
- Provide remote control and scheduling interfaces
- Ensure pet safety through detection and protocols
- Coordinate multi-pet household operations
- Facilitate smart home ecosystem integration

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| **Robot Specifications** | Device types, sensors, actuators, and capabilities |
| **Feeding Systems** | Automatic food and water dispensing with portion control |
| **Play & Exercise** | Interactive toys, laser pointers, ball launchers, and activity devices |
| **Health Monitoring** | Weight tracking, activity levels, behavioral analysis integration |
| **Safety Protocols** | Pet detection, collision avoidance, emergency shutdown |
| **Scheduling** | Time-based and event-based automation routines |
| **Multi-Pet Support** | Individual pet recognition and customized care |
| **Smart Home Integration** | Alexa, Google Home, HomeKit, and IoT platform connectivity |

### 1.3 Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity and All Living Beings

Pet care robots enhance animal welfare by ensuring consistent care, enrichment, and monitoring while supporting pet owners in providing the best possible care for their companions.

---

## 2. Robot Device Schema

### 2.1 Core Robot Device

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PetCareRobot",
  "type": "object",
  "required": [
    "robotId",
    "deviceType",
    "manufacturer",
    "modelNumber",
    "capabilities",
    "status",
    "location",
    "timestamp"
  ],
  "properties": {
    "robotId": {
      "type": "string",
      "pattern": "^PCR-[A-Z0-9]{12}$",
      "description": "Unique robot identifier"
    },
    "deviceType": {
      "type": "string",
      "enum": [
        "feeder",
        "water_fountain",
        "litter_box",
        "play_robot",
        "exercise_wheel",
        "treat_dispenser",
        "laser_toy",
        "ball_launcher",
        "camera_monitor",
        "grooming_station",
        "multi_function"
      ]
    },
    "manufacturer": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "model": {"type": "string"},
        "serialNumber": {"type": "string"},
        "firmwareVersion": {"type": "string"},
        "hardwareRevision": {"type": "string"},
        "manufactureDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "capabilities": {
      "type": "object",
      "properties": {
        "feeding": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "portionControl": {"type": "boolean"},
            "scheduledFeeding": {"type": "boolean"},
            "manualDispense": {"type": "boolean"},
            "maxPortionSize": {
              "type": "number",
              "description": "Maximum portion in grams"
            },
            "containerCapacity": {
              "type": "number",
              "description": "Food container capacity in grams"
            },
            "foodTypes": {
              "type": "array",
              "items": {
                "type": "string",
                "enum": ["dry_kibble", "wet_food", "treats", "prescription"]
              }
            }
          }
        },
        "watering": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "waterFiltered": {"type": "boolean"},
            "waterCirculation": {"type": "boolean"},
            "capacityLiters": {"type": "number"},
            "temperatureControl": {"type": "boolean"},
            "minTemperature": {"type": "number"},
            "maxTemperature": {"type": "number"}
          }
        },
        "play": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "interactiveToys": {"type": "boolean"},
            "laserPointer": {"type": "boolean"},
            "ballLauncher": {"type": "boolean"},
            "motionTracking": {"type": "boolean"},
            "adaptiveDifficulty": {"type": "boolean"},
            "maxPlayDuration": {
              "type": "number",
              "description": "Maximum play session in minutes"
            }
          }
        },
        "monitoring": {
          "type": "object",
          "properties": {
            "camera": {"type": "boolean"},
            "nightVision": {"type": "boolean"},
            "audioRecording": {"type": "boolean"},
            "twoWayAudio": {"type": "boolean"},
            "motionDetection": {"type": "boolean"},
            "petRecognition": {"type": "boolean"},
            "weightScale": {"type": "boolean"},
            "activityTracking": {"type": "boolean"}
          }
        },
        "mobility": {
          "type": "object",
          "properties": {
            "stationary": {"type": "boolean"},
            "wheeled": {"type": "boolean"},
            "maxSpeed": {
              "type": "number",
              "description": "Maximum speed in meters per second"
            },
            "obstacleAvoidance": {"type": "boolean"},
            "autonomousNavigation": {"type": "boolean"},
            "chargingStation": {"type": "boolean"}
          }
        }
      }
    },
    "sensors": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "sensorId": {"type": "string"},
          "sensorType": {
            "type": "string",
            "enum": [
              "camera",
              "microphone",
              "weight_scale",
              "proximity",
              "ultrasonic",
              "infrared",
              "temperature",
              "humidity",
              "food_level",
              "water_level",
              "battery",
              "motion_detector"
            ]
          },
          "location": {"type": "string"},
          "status": {
            "type": "string",
            "enum": ["active", "inactive", "error", "calibrating"]
          },
          "lastCalibration": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "status": {
      "type": "object",
      "required": ["operational", "batteryLevel"],
      "properties": {
        "operational": {
          "type": "string",
          "enum": ["online", "offline", "maintenance", "error", "standby"]
        },
        "batteryLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Battery percentage"
        },
        "charging": {"type": "boolean"},
        "foodLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Food container level percentage"
        },
        "waterLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Water container level percentage"
        },
        "errors": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "errorCode": {"type": "string"},
              "message": {"type": "string"},
              "severity": {
                "type": "string",
                "enum": ["info", "warning", "error", "critical"]
              },
              "timestamp": {
                "type": "string",
                "format": "date-time"
              }
            }
          }
        }
      }
    },
    "location": {
      "type": "object",
      "properties": {
        "room": {"type": "string"},
        "zone": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"}
          }
        },
        "fixedPosition": {"type": "boolean"}
      }
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

### 2.2 Robot Types Comparison

| Robot Type | Primary Function | Mobility | Typical Sensors | Power Source |
|------------|------------------|----------|-----------------|--------------|
| **Automatic Feeder** | Dispense food on schedule | Stationary | Food level, weight scale, camera | AC/Battery |
| **Water Fountain** | Provide fresh filtered water | Stationary | Water level, flow sensor, filter status | AC |
| **Litter Box** | Automated waste removal | Stationary | Weight, motion, odor sensors | AC |
| **Play Robot** | Interactive engagement | Mobile/Stationary | Camera, motion, proximity | Battery/AC |
| **Ball Launcher** | Exercise and fetch play | Stationary | Ball sensor, camera, range finder | AC/Battery |
| **Treat Dispenser** | Reward-based interaction | Stationary | Camera, microphone, dispense sensor | Battery |
| **Multi-Function** | Combined capabilities | Variable | Multiple sensor types | AC |

### 2.3 Feeding Event Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FeedingEvent",
  "type": "object",
  "required": [
    "eventId",
    "robotId",
    "petId",
    "timestamp",
    "foodType",
    "portionSize",
    "dispensed"
  ],
  "properties": {
    "eventId": {
      "type": "string",
      "pattern": "^FEED-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {
      "type": "string",
      "description": "Pet identifier or 'unknown'"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "feedingType": {
      "type": "string",
      "enum": ["scheduled", "manual", "automated", "treat", "supplement"]
    },
    "foodType": {
      "type": "string",
      "enum": ["dry_kibble", "wet_food", "treats", "prescription", "supplement"]
    },
    "portionSize": {
      "type": "number",
      "description": "Portion size in grams"
    },
    "dispensed": {
      "type": "boolean",
      "description": "Whether food was successfully dispensed"
    },
    "consumed": {
      "type": "object",
      "properties": {
        "detected": {"type": "boolean"},
        "estimatedAmount": {
          "type": "number",
          "description": "Estimated consumption in grams"
        },
        "duration": {
          "type": "number",
          "description": "Eating duration in seconds"
        }
      }
    },
    "schedule": {
      "type": "object",
      "properties": {
        "scheduleId": {"type": "string"},
        "plannedTime": {
          "type": "string",
          "format": "date-time"
        },
        "onTime": {"type": "boolean"},
        "delaySeconds": {"type": "number"}
      }
    },
    "nutrition": {
      "type": "object",
      "properties": {
        "calories": {"type": "number"},
        "protein": {"type": "number"},
        "fat": {"type": "number"},
        "carbohydrates": {"type": "number"},
        "fiber": {"type": "number"}
      }
    }
  }
}
```

---

## 3. Play and Exercise Data

### 3.1 Play Session Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PlaySession",
  "type": "object",
  "required": [
    "sessionId",
    "robotId",
    "petId",
    "startTime",
    "endTime",
    "playType"
  ],
  "properties": {
    "sessionId": {
      "type": "string",
      "pattern": "^PLAY-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {"type": "string"},
    "startTime": {
      "type": "string",
      "format": "date-time"
    },
    "endTime": {
      "type": "string",
      "format": "date-time"
    },
    "duration": {
      "type": "number",
      "description": "Duration in seconds"
    },
    "playType": {
      "type": "string",
      "enum": [
        "laser_chase",
        "ball_fetch",
        "treat_puzzle",
        "interactive_toy",
        "motion_game",
        "sound_stimulation",
        "automated_feather",
        "custom"
      ]
    },
    "intensity": {
      "type": "string",
      "enum": ["low", "medium", "high", "adaptive"]
    },
    "engagement": {
      "type": "object",
      "properties": {
        "level": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Engagement percentage"
        },
        "interactions": {
          "type": "number",
          "description": "Number of pet interactions detected"
        },
        "attentionSpan": {
          "type": "number",
          "description": "Average attention duration in seconds"
        }
      }
    },
    "activity": {
      "type": "object",
      "properties": {
        "movementDetected": {"type": "boolean"},
        "distanceTraveled": {
          "type": "number",
          "description": "Estimated distance in meters"
        },
        "averageSpeed": {
          "type": "number",
          "description": "Average speed in m/s"
        },
        "jumps": {"type": "number"},
        "caloriesBurned": {
          "type": "number",
          "description": "Estimated calories burned"
        }
      }
    },
    "robotActions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "timestamp": {
            "type": "string",
            "format": "date-time"
          },
          "action": {
            "type": "string",
            "enum": [
              "laser_move",
              "ball_launch",
              "treat_dispense",
              "sound_play",
              "movement",
              "pause"
            ]
          },
          "parameters": {
            "type": "object",
            "description": "Action-specific parameters"
          }
        }
      }
    },
    "terminationReason": {
      "type": "string",
      "enum": [
        "scheduled_end",
        "manual_stop",
        "pet_disengaged",
        "battery_low",
        "error",
        "timeout"
      ]
    }
  }
}
```

### 3.2 Play Activity Metrics

| Metric | Unit | Description | Typical Range |
|--------|------|-------------|---------------|
| **Session Duration** | minutes | Length of play session | 5-30 |
| **Engagement Level** | percentage | Pet's interest and participation | 0-100 |
| **Movement Distance** | meters | Total distance pet traveled | 10-500 |
| **Calorie Burn** | kcal | Estimated energy expenditure | 10-150 |
| **Interaction Count** | count | Number of pet-robot interactions | 5-200 |
| **Success Rate** | percentage | Successful catches/completions | 0-100 |

---

## 4. Health Monitoring Integration

### 4.1 Health Observation Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "HealthObservation",
  "type": "object",
  "required": [
    "observationId",
    "robotId",
    "petId",
    "timestamp",
    "observationType"
  ],
  "properties": {
    "observationId": {
      "type": "string",
      "pattern": "^OBS-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {"type": "string"},
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "observationType": {
      "type": "string",
      "enum": [
        "weight_measurement",
        "activity_level",
        "eating_behavior",
        "drinking_behavior",
        "elimination",
        "movement_pattern",
        "vocalization",
        "behavioral_change"
      ]
    },
    "weight": {
      "type": "object",
      "properties": {
        "value": {
          "type": "number",
          "description": "Weight in kilograms"
        },
        "unit": {
          "type": "string",
          "enum": ["kg", "lbs"]
        },
        "trend": {
          "type": "string",
          "enum": ["increasing", "decreasing", "stable", "unknown"]
        },
        "changePercent": {
          "type": "number",
          "description": "Change from previous measurement"
        }
      }
    },
    "activityLevel": {
      "type": "object",
      "properties": {
        "level": {
          "type": "string",
          "enum": ["very_low", "low", "normal", "high", "very_high"]
        },
        "duration": {
          "type": "number",
          "description": "Active time in minutes"
        },
        "intensityScore": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    },
    "eatingBehavior": {
      "type": "object",
      "properties": {
        "appetiteLevel": {
          "type": "string",
          "enum": ["none", "poor", "normal", "increased", "excessive"]
        },
        "eatingSpeed": {
          "type": "string",
          "enum": ["very_slow", "slow", "normal", "fast", "very_fast"]
        },
        "foodLeftover": {
          "type": "number",
          "description": "Percentage of food not consumed"
        },
        "unusualBehavior": {
          "type": "boolean"
        }
      }
    },
    "alerts": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "alertType": {
            "type": "string",
            "enum": [
              "weight_change",
              "reduced_activity",
              "eating_change",
              "drinking_change",
              "behavioral_concern"
            ]
          },
          "severity": {
            "type": "string",
            "enum": ["info", "low", "medium", "high", "urgent"]
          },
          "message": {"type": "string"},
          "recommendation": {"type": "string"}
        }
      }
    }
  }
}
```

### 4.2 Health Alert Triggers

| Alert Type | Trigger Condition | Severity | Recommended Action |
|------------|------------------|----------|-------------------|
| **Rapid Weight Loss** | >5% decrease in 7 days | High | Veterinary consultation |
| **Reduced Appetite** | <50% food consumed for 2+ meals | Medium | Monitor and contact vet if continues |
| **No Activity Detected** | <10 minutes activity in 24 hours | Medium | Check pet welfare |
| **Excessive Drinking** | >50% increase over baseline | Medium | Monitor for diabetes signs |
| **Eating Too Fast** | Meal consumed in <60 seconds | Low | Consider slow-feed bowl |
| **Weight Gain** | >10% increase in 30 days | Medium | Adjust portions and exercise |

---

## 5. Multi-Pet Management

### 5.1 Pet Profile Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PetProfile",
  "type": "object",
  "required": [
    "petId",
    "name",
    "species",
    "identification"
  ],
  "properties": {
    "petId": {
      "type": "string",
      "pattern": "^PET-[A-Z0-9]{12}$"
    },
    "name": {"type": "string"},
    "species": {
      "type": "string",
      "enum": ["dog", "cat", "rabbit", "bird", "other"]
    },
    "breed": {"type": "string"},
    "birthDate": {
      "type": "string",
      "format": "date"
    },
    "gender": {
      "type": "string",
      "enum": ["male", "female", "unknown"]
    },
    "neutered": {"type": "boolean"},
    "identification": {
      "type": "object",
      "properties": {
        "microchipId": {"type": "string"},
        "rfidTag": {"type": "string"},
        "facialRecognition": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "modelId": {"type": "string"},
            "confidence": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        },
        "collarTag": {"type": "string"}
      }
    },
    "dietaryRequirements": {
      "type": "object",
      "properties": {
        "foodType": {
          "type": "array",
          "items": {"type": "string"}
        },
        "portionSize": {
          "type": "number",
          "description": "Daily portion in grams"
        },
        "feedingFrequency": {
          "type": "number",
          "description": "Meals per day"
        },
        "allergies": {
          "type": "array",
          "items": {"type": "string"}
        },
        "restrictions": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "preferences": {
      "type": "object",
      "properties": {
        "favoritePlayType": {
          "type": "array",
          "items": {"type": "string"}
        },
        "playIntensity": {
          "type": "string",
          "enum": ["low", "medium", "high"]
        },
        "treatPreference": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "medicalConditions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "condition": {"type": "string"},
          "diagnosed": {
            "type": "string",
            "format": "date"
          },
          "medications": {
            "type": "array",
            "items": {"type": "string"}
          },
          "restrictions": {
            "type": "array",
            "items": {"type": "string"}
          }
        }
      }
    }
  }
}
```

### 5.2 Multi-Pet Household Scenarios

| Scenario | Challenge | Robot Solution | Implementation |
|----------|-----------|----------------|----------------|
| **Individual Feeding** | Pets steal each other's food | RFID-triggered feeding stations | Each pet has unique tag; feeder opens only for authorized pet |
| **Portion Control** | Different dietary needs | Pet-specific portion dispensing | Robot recognizes pet and dispenses correct amount |
| **Play Preferences** | Different energy levels | Adaptive play sessions | Robot adjusts intensity based on pet engagement |
| **Territory Management** | Resource guarding | Scheduled access windows | Time-based rotation for shared resources |
| **Medication Delivery** | One pet needs supplements | Targeted treat dispensing | Dispense medicated treats only to specific pet |

---

## 6. Code Examples

### 6.1 Robot Status Query

```python
import requests
import json
from datetime import datetime

class PetCareRobotClient:
    def __init__(self, base_url, api_key):
        self.base_url = base_url
        self.headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }

    def get_robot_status(self, robot_id):
        """Get current status of a pet care robot"""
        endpoint = f"{self.base_url}/api/v1/robots/{robot_id}/status"

        response = requests.get(endpoint, headers=self.headers)
        response.raise_for_status()

        return response.json()

    def check_supplies(self, robot_id):
        """Check food and water levels"""
        status = self.get_robot_status(robot_id)

        supplies = {
            'food_level': status['status']['foodLevel'],
            'water_level': status['status']['waterLevel'],
            'battery_level': status['status']['batteryLevel']
        }

        # Alert if any supply is low
        alerts = []
        if supplies['food_level'] < 20:
            alerts.append('Food level critically low')
        if supplies['water_level'] < 30:
            alerts.append('Water needs refilling')
        if supplies['battery_level'] < 15:
            alerts.append('Battery low - connect to charger')

        return {
            'supplies': supplies,
            'alerts': alerts,
            'timestamp': datetime.now().isoformat()
        }

# Usage
client = PetCareRobotClient('https://api.petcare.example.com', 'your-api-key')
status = client.check_supplies('PCR-ABC123456789')
print(json.dumps(status, indent=2))
```

### 6.2 Schedule Feeding Event

```javascript
const axios = require('axios');

class FeedingScheduler {
  constructor(apiUrl, apiKey) {
    this.apiUrl = apiUrl;
    this.apiKey = apiKey;
  }

  async scheduleFeedingEvent(robotId, petId, schedule) {
    const feedingEvent = {
      robotId: robotId,
      petId: petId,
      feedingType: 'scheduled',
      foodType: schedule.foodType || 'dry_kibble',
      portionSize: schedule.portionSize,
      schedule: {
        plannedTime: schedule.time,
        recurring: schedule.recurring || false,
        daysOfWeek: schedule.daysOfWeek || []
      }
    };

    try {
      const response = await axios.post(
        `${this.apiUrl}/api/v1/feeding/schedule`,
        feedingEvent,
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          }
        }
      );

      return {
        success: true,
        scheduleId: response.data.scheduleId,
        message: 'Feeding scheduled successfully'
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  }

  async createDailyFeedingSchedule(robotId, petId, meals) {
    const schedules = [];

    for (const meal of meals) {
      const result = await this.scheduleFeedingEvent(robotId, petId, {
        time: meal.time,
        portionSize: meal.portion,
        foodType: meal.type,
        recurring: true,
        daysOfWeek: ['MON', 'TUE', 'WED', 'THU', 'FRI', 'SAT', 'SUN']
      });

      schedules.push(result);
    }

    return schedules;
  }
}

// Usage example
const scheduler = new FeedingScheduler('https://api.petcare.example.com', 'api-key');

const dailyMeals = [
  { time: '07:00:00', portion: 100, type: 'dry_kibble' },
  { time: '12:00:00', portion: 50, type: 'wet_food' },
  { time: '18:00:00', portion: 100, type: 'dry_kibble' }
];

scheduler.createDailyFeedingSchedule('PCR-ABC123456789', 'PET-XYZ987654321', dailyMeals)
  .then(results => console.log('Feeding schedule created:', results));
```

### 6.3 Play Session Automation

```typescript
interface PlaySessionConfig {
  robotId: string;
  petId: string;
  playType: string;
  duration: number;
  intensity: 'low' | 'medium' | 'high' | 'adaptive';
  startTime?: Date;
}

class PlaySessionManager {
  private apiUrl: string;
  private apiKey: string;

  constructor(apiUrl: string, apiKey: string) {
    this.apiUrl = apiUrl;
    this.apiKey = apiKey;
  }

  async startPlaySession(config: PlaySessionConfig): Promise<any> {
    const sessionData = {
      robotId: config.robotId,
      petId: config.petId,
      playType: config.playType,
      intensity: config.intensity,
      maxDuration: config.duration,
      startTime: config.startTime || new Date()
    };

    const response = await fetch(`${this.apiUrl}/api/v1/play/session/start`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(sessionData)
    });

    if (!response.ok) {
      throw new Error(`Failed to start play session: ${response.statusText}`);
    }

    return await response.json();
  }

  async monitorPlaySession(sessionId: string): Promise<any> {
    const response = await fetch(
      `${this.apiUrl}/api/v1/play/session/${sessionId}`,
      {
        headers: {
          'Authorization': `Bearer ${this.apiKey}`
        }
      }
    );

    const sessionData = await response.json();

    // Check engagement level
    if (sessionData.engagement.level < 30) {
      console.warn('Low engagement detected - consider changing play type');
    }

    return sessionData;
  }

  async stopPlaySession(sessionId: string, reason: string): Promise<void> {
    await fetch(`${this.apiUrl}/api/v1/play/session/${sessionId}/stop`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ reason })
    });
  }
}

// Usage
const playManager = new PlaySessionManager('https://api.petcare.example.com', 'api-key');

const sessionConfig: PlaySessionConfig = {
  robotId: 'PCR-ABC123456789',
  petId: 'PET-XYZ987654321',
  playType: 'laser_chase',
  duration: 900, // 15 minutes
  intensity: 'adaptive'
};

playManager.startPlaySession(sessionConfig)
  .then(session => {
    console.log('Play session started:', session.sessionId);

    // Monitor every 30 seconds
    const monitorInterval = setInterval(async () => {
      const status = await playManager.monitorPlaySession(session.sessionId);
      console.log('Engagement:', status.engagement.level);

      if (status.endTime) {
        clearInterval(monitorInterval);
        console.log('Session completed');
      }
    }, 30000);
  });
```

### 6.4 Multi-Pet Recognition and Management

```python
from typing import List, Dict, Optional
import cv2
import numpy as np

class MultiPetManager:
    def __init__(self, robot_client):
        self.robot = robot_client
        self.pet_profiles = {}
        self.recognition_model = None

    def register_pet(self, pet_profile: Dict) -> str:
        """Register a pet with the system"""
        pet_id = pet_profile['petId']
        self.pet_profiles[pet_id] = pet_profile

        # Configure robot for this pet
        self.robot.add_pet_configuration(pet_id, {
            'feedingSchedule': pet_profile['dietaryRequirements'],
            'playPreferences': pet_profile['preferences'],
            'identificationMethod': pet_profile['identification']
        })

        return pet_id

    def identify_pet(self, image_data: np.ndarray) -> Optional[Dict]:
        """Identify which pet is present using facial recognition"""
        # Process image for pet detection
        detected_pets = []

        for pet_id, profile in self.pet_profiles.items():
            if profile['identification'].get('facialRecognition', {}).get('enabled'):
                # Run facial recognition
                confidence = self._run_recognition(image_data, pet_id)

                if confidence > 85:
                    detected_pets.append({
                        'petId': pet_id,
                        'name': profile['name'],
                        'confidence': confidence
                    })

        # Return highest confidence match
        if detected_pets:
            return max(detected_pets, key=lambda x: x['confidence'])

        return None

    def dispense_food_for_pet(self, robot_id: str, pet_id: str) -> Dict:
        """Dispense appropriate food portion for identified pet"""
        profile = self.pet_profiles.get(pet_id)

        if not profile:
            return {'error': 'Pet not registered'}

        # Get dietary requirements
        diet = profile['dietaryRequirements']
        portion = diet['portionSize'] / diet['feedingFrequency']

        # Dispense food
        result = self.robot.dispense_food({
            'robotId': robot_id,
            'petId': pet_id,
            'portionSize': portion,
            'foodType': diet['foodType'][0]
        })

        return result

    def manage_multi_pet_feeding(self, robot_id: str,
                                 detected_pet_ids: List[str]) -> List[Dict]:
        """Manage feeding when multiple pets are detected"""
        results = []

        for pet_id in detected_pet_ids:
            # Check if this pet has already eaten recently
            last_feeding = self.robot.get_last_feeding(pet_id)

            if last_feeding and self._is_recent(last_feeding['timestamp'], hours=4):
                results.append({
                    'petId': pet_id,
                    'action': 'skipped',
                    'reason': 'Already fed recently'
                })
                continue

            # Dispense food
            result = self.dispense_food_for_pet(robot_id, pet_id)
            results.append(result)

        return results

    def _run_recognition(self, image: np.ndarray, pet_id: str) -> float:
        """Run facial recognition model"""
        # Placeholder for actual recognition logic
        # In production, this would use a trained model
        return 90.0

    def _is_recent(self, timestamp: str, hours: int) -> bool:
        """Check if timestamp is within specified hours"""
        from datetime import datetime, timedelta

        event_time = datetime.fromisoformat(timestamp)
        time_diff = datetime.now() - event_time

        return time_diff < timedelta(hours=hours)

# Usage example
robot_client = PetCareRobotClient('https://api.petcare.example.com', 'api-key')
manager = MultiPetManager(robot_client)

# Register pets
pet1 = {
    'petId': 'PET-DOG001',
    'name': 'Max',
    'species': 'dog',
    'identification': {
        'rfidTag': 'RFID-12345',
        'facialRecognition': {'enabled': True}
    },
    'dietaryRequirements': {
        'foodType': ['dry_kibble'],
        'portionSize': 400,
        'feedingFrequency': 2
    },
    'preferences': {
        'favoritePlayType': ['ball_fetch'],
        'playIntensity': 'high'
    }
}

manager.register_pet(pet1)
```

### 6.5 Safety Protocol Implementation

```go
package petcare

import (
    "context"
    "fmt"
    "time"
)

type SafetyMonitor struct {
    robotID        string
    sensors        map[string]Sensor
    emergencyStop  chan bool
    alertChannel   chan Alert
}

type Sensor struct {
    Type     string
    Status   string
    LastRead time.Time
    Value    interface{}
}

type Alert struct {
    Severity    string
    Type        string
    Message     string
    Timestamp   time.Time
    Action      string
}

func NewSafetyMonitor(robotID string) *SafetyMonitor {
    return &SafetyMonitor{
        robotID:       robotID,
        sensors:       make(map[string]Sensor),
        emergencyStop: make(chan bool, 1),
        alertChannel:  make(chan Alert, 100),
    }
}

func (sm *SafetyMonitor) MonitorProximity(ctx context.Context) {
    ticker := time.NewTicker(100 * time.Millisecond)
    defer ticker.Stop()

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            // Read proximity sensors
            distance := sm.readProximitySensor()

            if distance < 0.1 { // Less than 10cm
                sm.alertChannel <- Alert{
                    Severity:  "critical",
                    Type:      "collision_imminent",
                    Message:   "Object detected within 10cm",
                    Timestamp: time.Now(),
                    Action:    "emergency_stop",
                }
                sm.emergencyStop <- true
            } else if distance < 0.3 { // Less than 30cm
                sm.alertChannel <- Alert{
                    Severity:  "warning",
                    Type:      "proximity_warning",
                    Message:   "Object detected within 30cm",
                    Timestamp: time.Now(),
                    Action:    "reduce_speed",
                }
            }
        }
    }
}

func (sm *SafetyMonitor) MonitorPetPresence(ctx context.Context) {
    ticker := time.NewTicker(500 * time.Millisecond)
    defer ticker.Stop()

    var petDetectedCount int

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            petDetected := sm.detectPet()

            if petDetected {
                petDetectedCount++

                // Confirm pet presence over multiple readings
                if petDetectedCount >= 3 {
                    sm.adjustOperationForPetPresence()
                }
            } else {
                petDetectedCount = 0
            }
        }
    }
}

func (sm *SafetyMonitor) EmergencyStopHandler(ctx context.Context) {
    for {
        select {
        case <-ctx.Done():
            return
        case <-sm.emergencyStop:
            fmt.Println("EMERGENCY STOP ACTIVATED")
            sm.executeEmergencyStop()

            // Wait for manual reset
            time.Sleep(5 * time.Second)

            // Check if safe to resume
            if sm.isSafeToResume() {
                sm.resumeOperation()
            }
        }
    }
}

func (sm *SafetyMonitor) readProximitySensor() float64 {
    // Simulate reading from ultrasonic sensor
    // Returns distance in meters
    return 0.5
}

func (sm *SafetyMonitor) detectPet() bool {
    // Simulate pet detection using camera/motion sensor
    return true
}

func (sm *SafetyMonitor) adjustOperationForPetPresence() {
    fmt.Println("Pet detected - adjusting operation parameters")
    // Reduce speed, increase sensor polling, etc.
}

func (sm *SafetyMonitor) executeEmergencyStop() {
    // Stop all motors
    // Disable dispensing mechanisms
    // Activate alert LED/sound
    fmt.Println("All systems stopped")
}

func (sm *SafetyMonitor) isSafeToResume() bool {
    // Check all safety conditions
    distance := sm.readProximitySensor()
    return distance > 0.5 // Safe distance threshold
}

func (sm *SafetyMonitor) resumeOperation() {
    fmt.Println("Resuming normal operation")
}

// Usage
func main() {
    ctx := context.Background()
    monitor := NewSafetyMonitor("PCR-ABC123456789")

    go monitor.MonitorProximity(ctx)
    go monitor.MonitorPetPresence(ctx)
    go monitor.EmergencyStopHandler(ctx)

    // Process alerts
    for alert := range monitor.alertChannel {
        fmt.Printf("[%s] %s: %s - Action: %s\n",
            alert.Severity, alert.Type, alert.Message, alert.Action)
    }
}
```

### 6.6 Smart Home Integration

```javascript
const { SmartHomeDevice } = require('alexa-smart-home-sdk');

class PetCareRobotAlexaSkill extends SmartHomeDevice {
  constructor(config) {
    super(config);
    this.robotClient = config.robotClient;
  }

  // Alexa Discovery
  async discover() {
    const robots = await this.robotClient.listRobots();

    return robots.map(robot => ({
      endpointId: robot.robotId,
      manufacturerName: robot.manufacturer.name,
      friendlyName: `Pet Feeder ${robot.location.room}`,
      description: `WIA Pet Care Robot - ${robot.deviceType}`,
      displayCategories: ['PET_FEEDER'],
      capabilities: [
        {
          type: 'AlexaInterface',
          interface: 'Alexa.PowerController',
          version: '3',
          properties: {
            supported: [{ name: 'powerState' }],
            retrievable: true
          }
        },
        {
          type: 'AlexaInterface',
          interface: 'Alexa.PercentageController',
          version: '3',
          properties: {
            supported: [{ name: 'percentage' }],
            retrievable: true
          }
        },
        {
          type: 'AlexaInterface',
          interface: 'Alexa.EndpointHealth',
          version: '3',
          properties: {
            supported: [{ name: 'connectivity' }],
            retrievable: true
          }
        }
      ]
    }));
  }

  // Handle "Alexa, feed my pet"
  async handleFeedPetCommand(endpointId, portion) {
    const result = await this.robotClient.dispenseFeed({
      robotId: endpointId,
      feedingType: 'manual',
      portionSize: portion || 100
    });

    return {
      namespace: 'Alexa',
      name: result.dispensed ? 'Response' : 'ErrorResponse',
      payload: {
        message: result.dispensed
          ? 'Pet has been fed'
          : 'Unable to dispense food - check robot status'
      }
    };
  }

  // Handle "Alexa, start play session"
  async handlePlayCommand(endpointId, duration) {
    const result = await this.robotClient.startPlaySession({
      robotId: endpointId,
      playType: 'laser_chase',
      duration: duration || 600,
      intensity: 'adaptive'
    });

    return {
      namespace: 'Alexa',
      name: 'Response',
      payload: {
        message: `Started ${duration || 10} minute play session`
      }
    };
  }

  // Handle "Alexa, what's the food level?"
  async handleStatusQuery(endpointId) {
    const status = await this.robotClient.getRobotStatus(endpointId);

    const message = `Food level is at ${status.status.foodLevel}%, ` +
                   `water level is at ${status.status.waterLevel}%, ` +
                   `and battery is at ${status.status.batteryLevel}%`;

    return {
      namespace: 'Alexa',
      name: 'Response',
      payload: { message }
    };
  }
}

// Google Home Integration
class PetCareRobotGoogleHome {
  constructor(robotClient) {
    this.robotClient = robotClient;
  }

  // Handle Google Home SYNC intent
  async onSync(body) {
    const robots = await this.robotClient.listRobots();

    return {
      requestId: body.requestId,
      payload: {
        agentUserId: body.agentUserId,
        devices: robots.map(robot => ({
          id: robot.robotId,
          type: 'action.devices.types.PETFEEDER',
          traits: [
            'action.devices.traits.OnOff',
            'action.devices.traits.Dispense',
            'action.devices.traits.SensorState'
          ],
          name: {
            name: `Pet Feeder ${robot.location.room}`
          },
          willReportState: true,
          attributes: {
            supportedDispenseItems: [
              { item_name: 'Food', item_name_synonyms: ['kibble', 'meal'] },
              { item_name: 'Treats', item_name_synonyms: ['snacks'] },
              { item_name: 'Water', item_name_synonyms: [] }
            ],
            supportedDispensePresets: [
              { preset_name: 'Small portion' },
              { preset_name: 'Regular portion' },
              { preset_name: 'Large portion' }
            ]
          },
          deviceInfo: {
            manufacturer: robot.manufacturer.name,
            model: robot.manufacturer.model,
            hwVersion: robot.manufacturer.hardwareRevision,
            swVersion: robot.manufacturer.firmwareVersion
          }
        }))
      }
    };
  }

  // Handle Google Home EXECUTE intent
  async onExecute(body) {
    const commands = [];

    for (const command of body.inputs[0].payload.commands) {
      for (const device of command.devices) {
        for (const execution of command.execution) {
          let result;

          switch (execution.command) {
            case 'action.devices.commands.Dispense':
              result = await this.handleDispense(device.id, execution.params);
              break;
            case 'action.devices.commands.OnOff':
              result = await this.handleOnOff(device.id, execution.params);
              break;
          }

          commands.push({
            ids: [device.id],
            status: result.success ? 'SUCCESS' : 'ERROR',
            states: result.states || {}
          });
        }
      }
    }

    return {
      requestId: body.requestId,
      payload: { commands }
    };
  }

  async handleDispense(robotId, params) {
    const portionMap = {
      'Small portion': 50,
      'Regular portion': 100,
      'Large portion': 150
    };

    const portion = portionMap[params.presetName] || params.amount || 100;

    const result = await this.robotClient.dispenseFeed({
      robotId: robotId,
      feedingType: 'manual',
      portionSize: portion
    });

    return {
      success: result.dispensed,
      states: {
        isDispensing: false,
        dispenseRemaining: 0
      }
    };
  }

  async handleOnOff(robotId, params) {
    // Toggle robot operational status
    const result = await this.robotClient.setRobotStatus(
      robotId,
      params.on ? 'online' : 'standby'
    );

    return {
      success: true,
      states: { on: params.on }
    };
  }
}

module.exports = {
  PetCareRobotAlexaSkill,
  PetCareRobotGoogleHome
};
```

---

## 7. Data Format Best Practices

### 7.1 Data Validation Rules

| Field Type | Validation | Example | Error Handling |
|------------|-----------|---------|----------------|
| **Robot ID** | Pattern: PCR-[A-Z0-9]{12} | PCR-ABC123456789 | Reject with 400 error |
| **Portion Size** | Range: 1-500 grams | 100 | Clamp to valid range |
| **Battery Level** | Range: 0-100 | 85 | Report sensor error if out of range |
| **Timestamp** | ISO 8601 format | 2025-12-18T10:30:00Z | Reject malformed dates |
| **Engagement Level** | Range: 0-100 | 75 | Default to 0 if invalid |

### 7.2 Data Retention Policies

| Data Type | Retention Period | Aggregation | Purpose |
|-----------|------------------|-------------|---------|
| **Real-time Status** | 7 days | None | Current monitoring |
| **Feeding Events** | 1 year | Daily summaries after 90 days | Health tracking |
| **Play Sessions** | 6 months | Weekly summaries after 30 days | Activity analysis |
| **Health Observations** | Permanent | Monthly summaries after 1 year | Long-term trends |
| **Error Logs** | 90 days | None | Troubleshooting |
| **Video Recordings** | 30 days | None | Privacy compliance |

### 7.3 Privacy and Security

| Aspect | Requirement | Implementation |
|--------|-------------|----------------|
| **Data Encryption** | AES-256 at rest, TLS 1.3 in transit | All data encrypted |
| **Access Control** | Role-based permissions | Owner, family, vet, service |
| **Video Privacy** | Opt-in recording only | Disabled by default |
| **Data Sharing** | Explicit consent required | Granular permissions |
| **Pet Identification** | Local processing preferred | Edge AI when possible |
| **API Authentication** | OAuth 2.0 + API keys | Multi-factor for admin |

---

## 8. Interoperability Standards

### 8.1 Compatible Standards

- **WIA-PET-HEALTH-PASSPORT**: Pet health record integration
- **WIA-PET-EMOTION**: Emotional state during interactions
- **WIA-PET-GENOME**: Breed-specific care recommendations
- **WIA-SMARTHOME**: Home automation integration
- **WIA-AI**: Machine learning model deployment
- **WIA-IOT**: Device connectivity protocols

### 8.2 External Protocol Support

| Protocol | Use Case | Implementation Status |
|----------|----------|----------------------|
| **MQTT** | IoT messaging | Required |
| **CoAP** | Constrained devices | Optional |
| **Zigbee** | Low-power sensors | Optional |
| **Thread** | Mesh networking | Optional |
| **Matter** | Smart home standard | Recommended |
| **WebRTC** | Video streaming | Required for camera robots |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
