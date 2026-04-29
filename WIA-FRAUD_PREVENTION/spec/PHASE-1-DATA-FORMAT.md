# WIA-FRAUD_PREVENTION v1.0
## PHASE 1: DATA FORMAT SPECIFICATION

**Status:** FULL Specification
**Version:** 1.0.0
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Executive Summary

WIA-FRAUD_PREVENTION Phase 1 defines comprehensive data formats for fraud prevention systems leveraging AI/ML technologies. This specification enables real-time fraud detection with proven 300% improvement in detection rates (Mastercard 2025) and supports the US Treasury's $4B fraud prevention initiative.

### Key Objectives
- Standardize fraud event data structures
- Enable real-time transaction monitoring
- Support behavioral analysis patterns
- Facilitate ML model training datasets
- Ensure interoperability across fraud prevention systems

---

## 2. Core Data Structures

### 2.1 Fraud Event Data Format

The fundamental data structure for capturing fraud events:

```json
{
  "fraudEvent": {
    "eventId": "string (UUID)",
    "eventType": "string (enum)",
    "timestamp": "string (ISO 8601)",
    "severity": "string (enum: low|medium|high|critical)",
    "status": "string (enum: detected|investigating|confirmed|false_positive|resolved)",
    "confidence": "number (0.0-1.0)",

    "transaction": {
      "transactionId": "string (UUID)",
      "amount": "number",
      "currency": "string (ISO 4217)",
      "merchantId": "string",
      "merchantCategory": "string (MCC code)",
      "paymentMethod": "string (enum)",
      "cardLast4": "string (optional)",
      "transactionTime": "string (ISO 8601)",
      "location": {
        "country": "string (ISO 3166-1)",
        "city": "string",
        "latitude": "number",
        "longitude": "number",
        "ipAddress": "string"
      }
    },

    "user": {
      "userId": "string (UUID)",
      "accountAge": "number (days)",
      "riskScore": "number (0-100)",
      "previousFraudCount": "number",
      "verificationLevel": "string (enum: none|basic|verified|premium)",
      "deviceFingerprint": "string",
      "behavioralProfile": {
        "avgTransactionAmount": "number",
        "transactionFrequency": "number (per day)",
        "typicalLocations": ["string"],
        "usualMerchantCategories": ["string"],
        "timeZonePattern": "string"
      }
    },

    "detectionMethod": {
      "method": "string (enum: rule_based|ml_model|behavioral|anomaly|hybrid)",
      "modelId": "string (optional)",
      "modelVersion": "string (optional)",
      "ruleIds": ["string"],
      "anomalyScore": "number (0.0-1.0)",
      "features": {
        "key": "value"
      }
    },

    "indicators": [
      {
        "indicatorType": "string (enum)",
        "description": "string",
        "weight": "number (0.0-1.0)",
        "value": "any"
      }
    ],

    "response": {
      "action": "string (enum: allow|block|challenge|review|alert)",
      "responseTime": "number (milliseconds)",
      "reviewerId": "string (optional)",
      "notes": "string (optional)",
      "falsePositiveReason": "string (optional)"
    },

    "metadata": {
      "source": "string",
      "version": "string",
      "tags": ["string"],
      "customFields": {}
    }
  }
}
```

### 2.2 Transaction Monitoring Data

```json
{
  "transactionMonitoring": {
    "sessionId": "string (UUID)",
    "monitoringStartTime": "string (ISO 8601)",
    "monitoringEndTime": "string (ISO 8601)",

    "realTimeMetrics": {
      "transactionCount": "number",
      "totalAmount": "number",
      "averageAmount": "number",
      "suspiciousCount": "number",
      "blockedCount": "number",
      "falsePositiveRate": "number (0.0-1.0)",
      "detectionRate": "number (0.0-1.0)",
      "averageResponseTime": "number (milliseconds)"
    },

    "velocityChecks": [
      {
        "checkType": "string (enum: amount|count|frequency|location)",
        "timeWindow": "number (seconds)",
        "threshold": "number",
        "currentValue": "number",
        "triggered": "boolean",
        "severity": "string (enum)"
      }
    ],

    "patternAnalysis": {
      "unusualPatterns": [
        {
          "patternType": "string",
          "description": "string",
          "confidence": "number (0.0-1.0)",
          "firstDetected": "string (ISO 8601)",
          "occurrences": "number"
        }
      ],
      "behavioralDeviations": [
        {
          "deviationType": "string",
          "baseline": "number",
          "current": "number",
          "deviationScore": "number (0.0-1.0)"
        }
      ]
    }
  }
}
```

### 2.3 ML Model Training Data Format

```json
{
  "trainingDataset": {
    "datasetId": "string (UUID)",
    "version": "string",
    "createdAt": "string (ISO 8601)",
    "datasetType": "string (enum: supervised|unsupervised|reinforcement)",

    "metadata": {
      "recordCount": "number",
      "fraudCount": "number",
      "legitimateCount": "number",
      "timeRange": {
        "start": "string (ISO 8601)",
        "end": "string (ISO 8601)"
      },
      "features": ["string"],
      "labels": ["string"],
      "balancingMethod": "string (enum: none|oversampling|undersampling|smote)"
    },

    "samples": [
      {
        "sampleId": "string (UUID)",
        "features": {
          "transaction_amount": "number",
          "time_since_last_transaction": "number (seconds)",
          "location_change_velocity": "number (km/h)",
          "device_reputation_score": "number (0-100)",
          "account_age_days": "number",
          "merchant_risk_score": "number (0-100)",
          "transaction_hour": "number (0-23)",
          "is_international": "boolean",
          "unusual_merchant_category": "boolean",
          "rapid_succession_flag": "boolean"
        },
        "label": "string (enum: fraud|legitimate)",
        "weight": "number (optional)",
        "timestamp": "string (ISO 8601)"
      }
    ],

    "validation": {
      "splitRatio": {
        "train": "number (0.0-1.0)",
        "validation": "number (0.0-1.0)",
        "test": "number (0.0-1.0)"
      },
      "crossValidationFolds": "number",
      "stratified": "boolean"
    }
  }
}
```

### 2.4 Anomaly Detection Data

```json
{
  "anomalyDetection": {
    "detectionId": "string (UUID)",
    "timestamp": "string (ISO 8601)",
    "algorithm": "string (enum: isolation_forest|one_class_svm|autoencoder|statistical)",

    "anomalyScore": "number (0.0-1.0)",
    "threshold": "number (0.0-1.0)",
    "isAnomaly": "boolean",

    "inputFeatures": {
      "feature_name": "number"
    },

    "reconstruction": {
      "originalValues": ["number"],
      "reconstructedValues": ["number"],
      "reconstructionError": "number"
    },

    "contextualInfo": {
      "similarPatterns": [
        {
          "patternId": "string",
          "similarity": "number (0.0-1.0)",
          "wasLegitimate": "boolean"
        }
      ],
      "historicalContext": {
        "averageBehavior": {},
        "deviationMagnitude": "number"
      }
    },

    "explanation": {
      "topContributingFeatures": [
        {
          "featureName": "string",
          "contribution": "number (0.0-1.0)",
          "expectedValue": "number",
          "actualValue": "number"
        }
      ],
      "humanReadableReason": "string"
    }
  }
}
```

### 2.5 Behavioral Analysis Profile

```json
{
  "behavioralProfile": {
    "profileId": "string (UUID)",
    "userId": "string (UUID)",
    "lastUpdated": "string (ISO 8601)",
    "profileVersion": "number",

    "transactionPatterns": {
      "typicalAmountRange": {
        "min": "number",
        "max": "number",
        "mean": "number",
        "median": "number",
        "stdDev": "number"
      },
      "frequencyPattern": {
        "daily": "number",
        "weekly": "number",
        "monthly": "number",
        "seasonalVariation": {}
      },
      "timeOfDayDistribution": {
        "0-6": "number (percentage)",
        "6-12": "number (percentage)",
        "12-18": "number (percentage)",
        "18-24": "number (percentage)"
      },
      "dayOfWeekDistribution": {
        "monday": "number (percentage)",
        "tuesday": "number (percentage)",
        "wednesday": "number (percentage)",
        "thursday": "number (percentage)",
        "friday": "number (percentage)",
        "saturday": "number (percentage)",
        "sunday": "number (percentage)"
      }
    },

    "locationPatterns": {
      "primaryLocations": [
        {
          "country": "string",
          "city": "string",
          "frequency": "number (percentage)"
        }
      ],
      "travelPatterns": {
        "averageDistanceBetweenTransactions": "number (km)",
        "maxPhysicallyPossibleVelocity": "number (km/h)",
        "internationalTransactionRate": "number (percentage)"
      }
    },

    "merchantPatterns": {
      "preferredCategories": [
        {
          "mccCode": "string",
          "categoryName": "string",
          "frequency": "number (percentage)"
        }
      ],
      "frequentMerchants": [
        {
          "merchantId": "string",
          "merchantName": "string",
          "transactionCount": "number"
        }
      ]
    },

    "devicePatterns": {
      "registeredDevices": [
        {
          "deviceId": "string",
          "deviceType": "string",
          "firstSeen": "string (ISO 8601)",
          "lastSeen": "string (ISO 8601)",
          "trustScore": "number (0-100)"
        }
      ],
      "unusualDeviceDetection": {
        "threshold": "number",
        "alertOnNewDevice": "boolean"
      }
    },

    "riskIndicators": {
      "overallRiskScore": "number (0-100)",
      "riskFactors": [
        {
          "factor": "string",
          "score": "number (0-100)",
          "weight": "number (0.0-1.0)"
        }
      ],
      "recentSuspiciousActivity": "number",
      "accountCompromiseIndicators": ["string"]
    }
  }
}
```

---

## 3. Alert and Notification Data

### 3.1 Fraud Alert Format

```json
{
  "fraudAlert": {
    "alertId": "string (UUID)",
    "triggeredAt": "string (ISO 8601)",
    "priority": "string (enum: low|medium|high|critical)",
    "category": "string (enum: suspicious_transaction|account_takeover|identity_theft|card_fraud|payment_fraud)",

    "subject": "string",
    "message": "string",

    "relatedEvents": [
      {
        "eventId": "string (UUID)",
        "eventType": "string",
        "timestamp": "string (ISO 8601)"
      }
    ],

    "recipients": [
      {
        "recipientType": "string (enum: user|admin|analyst|system)",
        "recipientId": "string",
        "channel": "string (enum: email|sms|push|webhook)",
        "deliveryStatus": "string (enum: pending|sent|delivered|failed)"
      }
    ],

    "actionRequired": {
      "requiresUserAction": "boolean",
      "actionType": "string (enum: verify_transaction|reset_password|contact_support|review_account)",
      "deadline": "string (ISO 8601)",
      "actionUrl": "string (optional)"
    },

    "resolution": {
      "resolved": "boolean",
      "resolvedAt": "string (ISO 8601)",
      "resolvedBy": "string (UUID)",
      "resolution": "string (enum: false_positive|confirmed_fraud|user_verified|system_error)",
      "notes": "string"
    }
  }
}
```

---

## 4. Reporting Data Structures

### 4.1 Fraud Prevention Report

```json
{
  "fraudReport": {
    "reportId": "string (UUID)",
    "reportType": "string (enum: daily|weekly|monthly|quarterly|annual|custom)",
    "generatedAt": "string (ISO 8601)",
    "reportPeriod": {
      "start": "string (ISO 8601)",
      "end": "string (ISO 8601)"
    },

    "summary": {
      "totalTransactions": "number",
      "totalAmount": "number",
      "fraudulentTransactions": "number",
      "fraudulentAmount": "number",
      "fraudRate": "number (percentage)",
      "preventedLoss": "number",
      "falsePositives": "number",
      "falseNegatives": "number",
      "detectionAccuracy": "number (percentage)"
    },

    "detectionMetrics": {
      "byMethod": [
        {
          "method": "string",
          "detectionCount": "number",
          "accuracy": "number (percentage)",
          "falsePositiveRate": "number (percentage)"
        }
      ],
      "byCategory": [
        {
          "category": "string",
          "count": "number",
          "amount": "number",
          "percentage": "number"
        }
      ],
      "responseTimeMetrics": {
        "average": "number (milliseconds)",
        "median": "number (milliseconds)",
        "p95": "number (milliseconds)",
        "p99": "number (milliseconds)"
      }
    },

    "trends": {
      "fraudRateTrend": "string (enum: increasing|decreasing|stable)",
      "comparisonToPreviousPeriod": {
        "fraudRateChange": "number (percentage)",
        "volumeChange": "number (percentage)",
        "accuracyChange": "number (percentage)"
      }
    },

    "topFraudPatterns": [
      {
        "patternId": "string",
        "description": "string",
        "occurrences": "number",
        "totalLoss": "number",
        "affectedUsers": "number"
      }
    ],

    "recommendations": [
      {
        "priority": "string (enum: high|medium|low)",
        "recommendation": "string",
        "expectedImpact": "string",
        "implementationEffort": "string (enum: low|medium|high)"
      }
    ]
  }
}
```

---

## 5. Configuration Data

### 5.1 Fraud Prevention Configuration

```json
{
  "fraudPreventionConfig": {
    "configId": "string (UUID)",
    "version": "string",
    "lastModified": "string (ISO 8601)",
    "modifiedBy": "string (UUID)",

    "detectionSettings": {
      "enableRealTimeDetection": "boolean",
      "enableBehavioralAnalysis": "boolean",
      "enableAnomalyDetection": "boolean",
      "enableMlModels": "boolean",

      "thresholds": {
        "fraudScoreThreshold": "number (0.0-1.0)",
        "anomalyThreshold": "number (0.0-1.0)",
        "velocityThreshold": {
          "amountPerHour": "number",
          "transactionsPerHour": "number",
          "amountPerDay": "number",
          "transactionsPerDay": "number"
        }
      },

      "autoActions": {
        "autoBlockScore": "number (0.0-1.0)",
        "autoChallengeScore": "number (0.0-1.0)",
        "autoReviewScore": "number (0.0-1.0)"
      }
    },

    "mlModelConfig": {
      "primaryModel": {
        "modelId": "string",
        "modelType": "string (enum: random_forest|neural_network|gradient_boosting|ensemble)",
        "version": "string",
        "updateFrequency": "string (cron expression)",
        "retrainingThreshold": "number (accuracy drop percentage)"
      },
      "backupModels": [
        {
          "modelId": "string",
          "activationCondition": "string"
        }
      ]
    },

    "alertSettings": {
      "alertChannels": ["string (enum: email|sms|push|webhook)"],
      "escalationRules": [
        {
          "condition": "string",
          "escalateTo": "string",
          "delayMinutes": "number"
        }
      ],
      "notificationPreferences": {
        "critical": {
          "immediate": "boolean",
          "channels": ["string"]
        },
        "high": {
          "immediate": "boolean",
          "channels": ["string"]
        },
        "medium": {
          "batched": "boolean",
          "batchInterval": "number (minutes)",
          "channels": ["string"]
        },
        "low": {
          "batched": "boolean",
          "batchInterval": "number (minutes)",
          "channels": ["string"]
        }
      }
    },

    "integrations": {
      "externalServices": [
        {
          "serviceId": "string",
          "serviceName": "string",
          "enabled": "boolean",
          "apiEndpoint": "string",
          "authentication": {
            "method": "string (enum: api_key|oauth2|jwt)",
            "credentialsId": "string (reference)"
          }
        }
      ]
    }
  }
}
```

---

## 6. Data Validation Rules

### 6.1 Field Validation

| Field Type | Validation Rule | Example |
|------------|----------------|---------|
| UUID | RFC 4122 compliant | `550e8400-e29b-41d4-a716-446655440000` |
| ISO 8601 Timestamp | YYYY-MM-DDTHH:mm:ss.sssZ | `2026-01-12T10:30:00.000Z` |
| Currency Code | ISO 4217 (3 characters) | `USD`, `EUR`, `GBP` |
| Country Code | ISO 3166-1 alpha-2 | `US`, `GB`, `CA` |
| Confidence Score | 0.0 ≤ value ≤ 1.0 | `0.85` |
| Risk Score | 0 ≤ value ≤ 100 | `75` |
| Email | RFC 5322 compliant | `user@example.com` |
| IP Address | IPv4 or IPv6 | `192.168.1.1` or `2001:db8::1` |

### 6.2 Data Integrity Constraints

- **Event IDs**: Must be globally unique across all systems
- **Timestamps**: Must use UTC timezone
- **Amount Fields**: Must be non-negative
- **Confidence Scores**: Required for ML-based detections
- **User IDs**: Must reference valid user accounts
- **Transaction IDs**: Must be unique within merchant system

---

## 7. Data Retention and Privacy

### 7.1 Retention Policies

| Data Type | Retention Period | Archive After | Delete After |
|-----------|------------------|---------------|--------------|
| Fraud Events | 7 years | 1 year | 7 years |
| Transaction Logs | 7 years | 1 year | 7 years |
| User Profiles | Active + 3 years | Account closure + 1 year | 3 years |
| ML Training Data | 5 years | 1 year | 5 years |
| Alerts | 3 years | 6 months | 3 years |
| Reports | 10 years | 2 years | 10 years |

### 7.2 Privacy Considerations

- **PII Protection**: Personal Identifiable Information must be encrypted at rest
- **Data Minimization**: Collect only necessary data for fraud prevention
- **Right to Erasure**: Support user data deletion requests (GDPR compliance)
- **Access Controls**: Implement role-based access to sensitive data
- **Audit Trails**: Maintain logs of data access and modifications
- **Anonymization**: Support data anonymization for analytics and ML training

---

## 8. Interoperability Standards

### 8.1 Supported Formats

- **Primary Format**: JSON (application/json)
- **Alternative Formats**:
  - XML (application/xml)
  - Protocol Buffers (application/protobuf)
  - Apache Avro (application/avro)

### 8.2 Character Encoding

- **Standard**: UTF-8
- **Alternative**: UTF-16 (must be explicitly specified)

### 8.3 Compression

- **Supported**: gzip, brotli, deflate
- **Recommended**: gzip for REST APIs, protobuf for high-volume streaming

---

## 9. Real-World Performance Benchmarks

Based on industry implementations (2025-2026):

| Metric | Industry Average | WIA Standard Target |
|--------|-----------------|---------------------|
| Detection Rate | 75% | 95%+ |
| False Positive Rate | 15% | <5% |
| Response Time | 500ms | <100ms |
| Transaction Throughput | 1,000 TPS | 10,000+ TPS |
| Model Accuracy | 85% | 95%+ |
| Data Processing Latency | 2 seconds | <500ms |

**Notable Implementations:**
- **Mastercard**: 300% improvement in fraud detection rates (2025)
- **US Treasury**: $4B fraud prevention initiative
- **PayPal**: 99.5% accuracy with AI-driven fraud detection
- **Stripe**: Real-time fraud detection processing 100,000+ TPS

---

## 10. Implementation Checklist

- [ ] Implement all core data structures (Section 2)
- [ ] Validate data against field validation rules (Section 6.1)
- [ ] Configure data retention policies (Section 7.1)
- [ ] Implement PII encryption and privacy controls (Section 7.2)
- [ ] Support primary JSON format with UTF-8 encoding
- [ ] Add compression support for high-volume data
- [ ] Implement data validation middleware
- [ ] Create data migration scripts for existing systems
- [ ] Document custom field extensions
- [ ] Test interoperability with external systems

---

## 11. Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2026-01-12 | Initial FULL specification | WIA Standards Committee |

---

## 12. References

- ISO 8601: Date and time format
- ISO 4217: Currency codes
- ISO 3166-1: Country codes
- RFC 4122: UUID specification
- RFC 5322: Email address specification
- GDPR: General Data Protection Regulation
- PCI DSS: Payment Card Industry Data Security Standard
- Mastercard 2025 Fraud Detection Report
- US Treasury Fraud Prevention Initiative 2025

---

**Document Classification:** Public Standard
**License:** Open Standard (Implementable without royalties)
**Maintenance:** WIA Standards Committee

---

© 2026 WIA (World Industry Association)
弘益人間 (홍익인간) - Benefit All Humanity
