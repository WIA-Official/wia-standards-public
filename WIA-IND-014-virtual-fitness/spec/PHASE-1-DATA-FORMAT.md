# WIA-IND-014: Virtual Fitness Standard
## Phase 1: Data Format Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-27
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document defines standardized data formats for virtual fitness applications, ensuring interoperability across VR/AR platforms, motion tracking systems, and fitness applications. The goal is to enable users to seamlessly move their workout data, preferences, and achievements between different platforms and devices.

## 2. Motion Tracking Data Schema

### 2.1 Skeletal Keypoint Model

The WIA-IND-014 standard defines a 33-keypoint skeletal model representing the human body:

```json
{
  "version": "1.0",
  "standard": "WIA-IND-014",
  "philosophy": "弘益人間",
  "skeletonModel": {
    "keypointCount": 33,
    "keypoints": [
      {"id": 0, "name": "nose", "parent": null},
      {"id": 1, "name": "leftEyeInner", "parent": 0},
      {"id": 2, "name": "leftEye", "parent": 0},
      {"id": 3, "name": "leftEyeOuter", "parent": 0},
      {"id": 4, "name": "rightEyeInner", "parent": 0},
      {"id": 5, "name": "rightEye", "parent": 0},
      {"id": 6, "name": "rightEyeOuter", "parent": 0},
      {"id": 7, "name": "leftEar", "parent": 0},
      {"id": 8, "name": "rightEar", "parent": 0},
      {"id": 9, "name": "mouthLeft", "parent": 0},
      {"id": 10, "name": "mouthRight", "parent": 0},
      {"id": 11, "name": "leftShoulder", "parent": null},
      {"id": 12, "name": "rightShoulder", "parent": null},
      {"id": 13, "name": "leftElbow", "parent": 11},
      {"id": 14, "name": "rightElbow", "parent": 12},
      {"id": 15, "name": "leftWrist", "parent": 13},
      {"id": 16, "name": "rightWrist", "parent": 14},
      {"id": 17, "name": "leftPinky", "parent": 15},
      {"id": 18, "name": "rightPinky", "parent": 16},
      {"id": 19, "name": "leftIndex", "parent": 15},
      {"id": 20, "name": "rightIndex", "parent": 16},
      {"id": 21, "name": "leftThumb", "parent": 15},
      {"id": 22, "name": "rightThumb", "parent": 16},
      {"id": 23, "name": "leftHip", "parent": null},
      {"id": 24, "name": "rightHip", "parent": null},
      {"id": 25, "name": "leftKnee", "parent": 23},
      {"id": 26, "name": "rightKnee", "parent": 24},
      {"id": 27, "name": "leftAnkle", "parent": 25},
      {"id": 28, "name": "rightAnkle", "parent": 26},
      {"id": 29, "name": "leftHeel", "parent": 27},
      {"id": 30, "name": "rightHeel", "parent": 28},
      {"id": 31, "name": "leftFootIndex", "parent": 27},
      {"id": 32, "name": "rightFootIndex", "parent": 28}
    ]
  }
}
```

### 2.2 Pose Frame Format

Each frame of motion capture data follows this structure:

```json
{
  "timestamp": 1706371200000,
  "frameNumber": 1234,
  "keypoints": [
    {
      "id": 0,
      "position": {
        "x": 0.5,
        "y": 0.8,
        "z": 0.0
      },
      "confidence": 0.95,
      "visibility": true
    }
  ],
  "worldCoordinates": {
    "referencePoint": {"x": 0, "y": 0, "z": 0},
    "scale": 1.0,
    "unit": "meters"
  }
}
```

**Field Specifications:**
- `timestamp`: Unix timestamp in milliseconds
- `frameNumber`: Sequential frame identifier
- `position.x/y/z`: Normalized coordinates (0-1) or world coordinates in meters
- `confidence`: Detection confidence (0.0-1.0), minimum 0.7 for valid data
- `visibility`: Boolean indicating if keypoint is visible to tracking system

## 3. Workout Session Metadata

### 3.1 Session Schema

```json
{
  "sessionId": "uuid-v4-string",
  "standard": "WIA-IND-014",
  "philosophy": "弘益人間",
  "user": {
    "userId": "anonymous-or-user-id",
    "age": 35,
    "weight": 70,
    "height": 175,
    "fitnessLevel": "intermediate"
  },
  "workout": {
    "type": "vr-boxing",
    "name": "Mountain Peak Combat",
    "duration": 1800,
    "intensity": "high",
    "environment": "virtual-mountain",
    "difficulty": "hard"
  },
  "tracking": {
    "mode": "full-body",
    "algorithm": "MediaPipe-Pose",
    "device": "Meta-Quest-3",
    "fps": 30,
    "totalFrames": 54000
  },
  "startTime": "2025-01-27T10:00:00Z",
  "endTime": "2025-01-27T10:30:00Z",
  "timezone": "UTC"
}
```

## 4. Performance Metrics

### 4.1 Real-time Metrics

```json
{
  "heartRate": {
    "current": 145,
    "average": 138,
    "max": 165,
    "min": 70,
    "zones": {
      "zone1": 120,
      "zone2": 420,
      "zone3": 780,
      "zone4": 420,
      "zone5": 60
    },
    "source": "apple-watch",
    "unit": "bpm"
  },
  "calories": {
    "total": 450,
    "active": 420,
    "resting": 30,
    "unit": "kcal"
  },
  "movement": {
    "punches": 1250,
    "dodges": 340,
    "squats": 85,
    "distance": 2.3,
    "distanceUnit": "km"
  },
  "performance": {
    "accuracy": 0.87,
    "formScore": 0.92,
    "consistency": 0.85,
    "overallScore": 89
  }
}
```

## 5. Form Analysis Data

### 5.1 Biomechanical Analysis

```json
{
  "exercise": "squat",
  "frameId": 12345,
  "timestamp": 1706371200000,
  "angles": {
    "leftKnee": 92,
    "rightKnee": 91,
    "leftHip": 95,
    "rightHip": 94,
    "spine": 5
  },
  "formCriteria": {
    "depth": {
      "status": "good",
      "value": "below-parallel",
      "score": 1.0
    },
    "kneeAlignment": {
      "status": "warning",
      "value": "slight-valgus-left",
      "score": 0.8
    },
    "spineNeutral": {
      "status": "good",
      "value": "neutral",
      "score": 1.0
    },
    "weightDistribution": {
      "status": "good",
      "value": "heels",
      "score": 1.0
    }
  },
  "overallFormScore": 0.95,
  "feedback": [
    "Excellent depth!",
    "Watch left knee tracking - slight inward movement"
  ]
}
```

## 6. Avatar and Customization Data

### 6.1 Avatar Configuration

```json
{
  "avatarId": "uuid-v4-string",
  "bodyType": {
    "height": 175,
    "weight": 70,
    "bodyFat": 18,
    "proportions": {
      "torsoLength": 1.0,
      "armLength": 1.0,
      "legLength": 1.0
    }
  },
  "appearance": {
    "skinTone": "#F5D4C1",
    "hairStyle": "short-curly",
    "hairColor": "#2C1B18",
    "gender": "neutral",
    "age": 35
  },
  "clothing": {
    "top": "compression-shirt-blue",
    "bottom": "athletic-shorts-black",
    "shoes": "running-shoes-white",
    "accessories": ["headband-red", "wristbands"]
  },
  "customization": {
    "unlocked": true,
    "premium": false
  }
}
```

## 7. Environment and Scene Configuration

### 7.1 Virtual Environment Data

```json
{
  "environmentId": "mountain-peak-sunset",
  "name": "Mountain Peak at Sunset",
  "category": "outdoor-nature",
  "timeOfDay": "sunset",
  "weather": "clear",
  "lighting": {
    "ambient": 0.3,
    "directional": 0.8,
    "colorTemperature": 3500
  },
  "audio": {
    "ambient": "mountain-wind",
    "music": "energetic-electronic",
    "volume": 0.7
  },
  "difficulty": "medium",
  "unlockRequirement": {
    "level": 10,
    "achievement": null
  }
}
```

## 8. Data Encoding and Transmission

### 8.1 Supported Formats

- **JSON**: Human-readable, web-friendly (default)
- **Protocol Buffers**: Binary, efficient for real-time streaming
- **MessagePack**: Binary JSON alternative, compact
- **CBOR**: Concise Binary Object Representation

### 8.2 Compression

- **Gzip**: Standard compression for JSON data
- **Brotli**: Higher compression ratios, recommended for large datasets
- **LZ4**: Fast compression for real-time streaming

### 8.3 Encryption

All biometric and personal data MUST be encrypted using:
- **TLS 1.3** for transmission
- **AES-256-GCM** for data at rest
- **End-to-end encryption** for peer-to-peer sessions

## 9. Data Portability

### 9.1 Export Format

Users can export their complete fitness data in standardized WIA-IND-014 format:

```json
{
  "export": {
    "version": "1.0",
    "standard": "WIA-IND-014",
    "philosophy": "弘益人間",
    "exportDate": "2025-01-27T12:00:00Z",
    "userId": "user-uuid",
    "dataType": "complete",
    "sessions": [...],
    "achievements": [...],
    "preferences": {...}
  }
}
```

## 10. Privacy and Consent

### 10.1 Data Classification

- **Public**: Username, public achievements
- **Friends-Only**: Workout history, leaderboard positions
- **Private**: Biometric data, health metrics, detailed performance
- **Encrypted**: Motion capture raw data, medical information

### 10.2 User Rights

Users have the right to:
1. **Access** all their data
2. **Export** data in standard formats
3. **Delete** data permanently
4. **Restrict** processing
5. **Object** to automated decision-making

## 11. Compliance and Certification

Applications implementing WIA-IND-014 Phase 1 Data Format must:
1. Support JSON format (minimum requirement)
2. Implement the 33-keypoint skeletal model
3. Provide data export functionality
4. Encrypt all biometric data
5. Respect user privacy controls
6. Display WIA-IND-014 certification badge

---

**Document Version:** 1.0
**Status:** Draft for Public Comment
**Contact:** standards@wia.org
**License:** CC BY-SA 4.0

弘益人間 · Benefit All Humanity
