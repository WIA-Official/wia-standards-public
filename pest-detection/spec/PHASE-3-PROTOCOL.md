# WIA-AGRI-013: Pest Detection Standard
## Phase 3: Alert Protocol Specification

### 3.1 Overview

Phase 3 defines the standardized protocols for pest outbreak alerts, early warning systems, and regional coordination mechanisms. This phase ensures timely communication of pest threats to farmers, extension services, and agricultural authorities.

**Duration**: 4-6 months
**Key Outcome**: Operational alert broadcasting system with multi-channel delivery

### 3.2 Alert Classification System

#### 3.2.1 Alert Levels

| Level | Code | Description | Response Time | Notification Channels |
|-------|------|-------------|---------------|----------------------|
| **Info** | L1 | Pest detected below economic threshold | 24-48 hours | App, Email |
| **Advisory** | L2 | Population approaching threshold | 12-24 hours | App, Email, SMS |
| **Warning** | L3 | Economic threshold exceeded | 6-12 hours | All channels + Voice |
| **Critical** | L4 | Widespread outbreak, immediate action required | 1-2 hours | All channels + Emergency broadcast |

#### 3.2.2 Alert Priority Matrix

```
Priority = f(Severity, Spread, Economic_Impact, Timing)

Where:
- Severity: [1-10] pest population density vs threshold
- Spread: [1-10] geographic distribution
- Economic_Impact: [1-10] potential crop loss
- Timing: [1-10] urgency based on pest life cycle
```

### 3.3 Alert Message Format

#### 3.3.1 Standard Alert Structure

```json
{
  "alert_id": "string (UUID)",
  "standard_version": "WIA-AGRI-013-v1.0",
  "alert_type": "enum [detection, outbreak, treatment, all_clear]",
  "priority": "enum [info, advisory, warning, critical]",
  "issue_timestamp": "ISO 8601 datetime",
  "effective_timestamp": "ISO 8601 datetime",
  "expires_timestamp": "ISO 8601 datetime",
  "issuer": {
    "organization": "string",
    "department": "string",
    "issuer_id": "string",
    "contact": {
      "phone": "string",
      "email": "string",
      "website": "string"
    }
  },
  "pest_information": {
    "species": "string (scientific name)",
    "common_name": {
      "en": "string",
      "ko": "string",
      "local": "string"
    },
    "pest_type": "enum [insect, fungal, bacterial, viral]",
    "life_stage": "string",
    "population_density": "number",
    "severity_level": "enum [low, medium, high, critical]"
  },
  "geographic_scope": {
    "alert_region": "string",
    "affected_areas": [
      {
        "region_code": "string",
        "region_name": "string",
        "area_ha": "number",
        "center_coordinates": {
          "latitude": "number",
          "longitude": "number"
        },
        "radius_km": "number",
        "boundary_polygon": "GeoJSON polygon"
      }
    ],
    "total_affected_area_ha": "number",
    "estimated_farmers_affected": "number"
  },
  "crop_impact": {
    "crops_affected": ["array of crop types"],
    "growth_stages_vulnerable": ["array of stages"],
    "estimated_yield_loss_percent": "number",
    "economic_impact_usd": "number"
  },
  "recommended_actions": {
    "immediate": ["array of action items"],
    "short_term": ["array of action items"],
    "preventive": ["array of action items"]
  },
  "message": {
    "headline": {
      "en": "string (max 140 chars)",
      "ko": "string (max 140 chars)"
    },
    "description": {
      "en": "string",
      "ko": "string"
    },
    "instruction": {
      "en": "string",
      "ko": "string"
    }
  },
  "contact_info": {
    "hotline": "string (phone number)",
    "emergency_contact": "string",
    "extension_office": "string",
    "website": "string"
  },
  "metadata": {
    "language": ["array of ISO 639-1 codes"],
    "category": ["array of categories"],
    "references": ["array of related alert IDs"]
  }
}
```

#### 3.3.2 Example Critical Alert

```json
{
  "alert_id": "WIA-PEST-2025-0615-001",
  "standard_version": "WIA-AGRI-013-v1.0",
  "alert_type": "outbreak",
  "priority": "critical",
  "issue_timestamp": "2025-06-15T14:30:00+09:00",
  "effective_timestamp": "2025-06-15T14:30:00+09:00",
  "expires_timestamp": "2025-06-22T23:59:59+09:00",
  "issuer": {
    "organization": "Rural Development Administration",
    "department": "농촌진흥청 병해충예찰팀",
    "issuer_id": "RDA-KR",
    "contact": {
      "phone": "1544-8572",
      "email": "pest@rda.go.kr",
      "website": "ncpms.rda.go.kr"
    }
  },
  "pest_information": {
    "species": "Nilaparvata lugens",
    "common_name": {
      "en": "Rice Planthopper",
      "ko": "벼멸구",
      "local": "멸구"
    },
    "pest_type": "insect",
    "life_stage": "adult",
    "population_density": 125,
    "severity_level": "critical"
  },
  "geographic_scope": {
    "alert_region": "Jeonbuk Province",
    "affected_areas": [
      {
        "region_code": "JB-01",
        "region_name": "김제시 (Gimje City)",
        "area_ha": 8500,
        "center_coordinates": {
          "latitude": 35.8019,
          "longitude": 126.8806
        },
        "radius_km": 25
      },
      {
        "region_code": "JB-02",
        "region_name": "정읍시 (Jeongeup City)",
        "area_ha": 6200,
        "center_coordinates": {
          "latitude": 35.5697,
          "longitude": 126.8561
        },
        "radius_km": 20
      }
    ],
    "total_affected_area_ha": 14700,
    "estimated_farmers_affected": 2840
  },
  "crop_impact": {
    "crops_affected": ["rice"],
    "growth_stages_vulnerable": ["tillering", "panicle_initiation", "heading"],
    "estimated_yield_loss_percent": 45,
    "economic_impact_usd": 8500000
  },
  "recommended_actions": {
    "immediate": [
      "Conduct field surveys within 24 hours",
      "Deploy pheromone traps for population monitoring",
      "Prepare spray equipment and pesticides",
      "Contact local extension office for guidance"
    ],
    "short_term": [
      "Apply registered insecticides if threshold exceeded (>10 adults per sweep)",
      "Monitor fields every 2-3 days during outbreak period",
      "Coordinate treatment timing with neighboring farmers",
      "Report treatment results to extension service"
    ],
    "preventive": [
      "Use resistant rice varieties in next season",
      "Maintain proper water management",
      "Avoid excessive nitrogen fertilization",
      "Preserve natural enemy populations"
    ]
  },
  "message": {
    "headline": {
      "en": "CRITICAL: Rice Planthopper Outbreak in Jeonbuk Province",
      "ko": "긴급: 전북지역 벼멸구 대발생 경보"
    },
    "description": {
      "en": "Severe rice planthopper infestation detected across 14,700 hectares in Gimje and Jeongeup areas. Population density exceeds economic threshold by 6x. Immediate treatment required to prevent significant yield losses.",
      "ko": "김제, 정읍 지역 14,700ha에서 벼멸구 심각한 발생이 확인되었습니다. 개체밀도가 경제적 피해 수준의 6배를 초과하였으며, 수확량 손실 방지를 위해 즉시 방제가 필요합니다."
    },
    "instruction": {
      "en": "1) Scout fields immediately, 2) Apply registered insecticides within 48 hours if threshold exceeded, 3) Contact extension office at 1544-8572 for technical support, 4) Report treatment to local authorities.",
      "ko": "1) 즉시 논 조사 실시, 2) 피해수준 초과시 48시간 내 등록약제 방제, 3) 기술지원은 1544-8572로 문의, 4) 방제실시 결과를 관할 기관에 보고하시기 바랍니다."
    }
  },
  "contact_info": {
    "hotline": "1544-8572",
    "emergency_contact": "063-238-1234 (전북농업기술원)",
    "extension_office": "각 시군 농업기술센터",
    "website": "ncpms.rda.go.kr"
  },
  "metadata": {
    "language": ["en", "ko"],
    "category": ["agriculture", "crop_protection", "insect_pest"],
    "references": []
  }
}
```

### 3.4 Alert Distribution Channels

#### 3.4.1 Channel Specifications

**1. SMS (Short Message Service)**
- Character limit: 160 chars (Korean), 70 chars (Unicode)
- Delivery time: < 60 seconds
- Priority: Warning and Critical alerts
- Format: Shortened URL for full details

```
[긴급] 벼멸구 대발생 김제/정읍 지역
즉시 논 조사 및 방제 필요
상세정보: https://wia.ag/p/WIA-001
문의: 1544-8572
```

**2. Mobile App Push Notifications**
- Title: 60 chars max
- Body: 240 chars max
- Delivery time: < 10 seconds
- Priority: All alert levels
- Rich media: Images, action buttons

**3. Email**
- Subject: 78 chars max
- Body: Full HTML with images
- Delivery time: < 5 minutes
- Priority: All alert levels
- Attachments: PDF reports, treatment guides

**4. Voice Call (Critical Only)**
- Automated voice message
- Duration: 30-60 seconds
- Language: Korean, English
- Delivery time: < 2 minutes
- Callback number provided

**5. Web Portal**
- Real-time alert dashboard
- Interactive map of affected areas
- Filterable by region, crop, pest
- Export to CSV, PDF

**6. IoT Device Display**
- LED warning lights on smart farms
- Digital signage at extension offices
- Alert tones for different priorities

### 3.5 Alert Workflow

#### 3.5.1 Detection to Alert Process

```
[Detection] → [Validation] → [Risk Assessment] → [Alert Generation] → [Distribution] → [Confirmation] → [Follow-up]

Timeline:
Detection (T+0): Pest detected via AI/manual inspection
Validation (T+15min): Expert verification or AI confidence check
Risk Assessment (T+30min): Calculate severity and priority
Alert Generation (T+45min): Create standardized alert message
Distribution (T+60min): Send via all applicable channels
Confirmation (T+90min): Track delivery status
Follow-up (T+24hrs): Monitor response and effectiveness
```

#### 3.5.2 Alert Escalation Protocol

```python
if population_density > threshold * 5:
    priority = "critical"
    channels = ["sms", "voice", "app", "email", "iot"]
    response_time = "1-2 hours"
elif population_density > threshold * 2:
    priority = "warning"
    channels = ["sms", "app", "email"]
    response_time = "6-12 hours"
elif population_density > threshold:
    priority = "advisory"
    channels = ["app", "email"]
    response_time = "12-24 hours"
else:
    priority = "info"
    channels = ["app"]
    response_time = "24-48 hours"
```

### 3.6 Regional Coordination

#### 3.6.1 Multi-Jurisdictional Alerts

When pest outbreak spans multiple administrative regions:

1. **Primary Issuer**: Organization with jurisdiction over epicenter
2. **Co-issuers**: Adjacent regional authorities
3. **Coordination**: Unified message with regional-specific instructions
4. **Contact Points**: Each region provides local hotline

Example:
```json
{
  "alert_id": "WIA-PEST-2025-0615-001",
  "issuer": {
    "primary": "Jeonbuk Agricultural Technology Center",
    "co_issuers": ["Jeonnam ATC", "Chungnam ATC"]
  },
  "regional_instructions": {
    "JB": "Contact 063-238-1234",
    "JN": "Contact 061-330-1234",
    "CN": "Contact 041-635-1234"
  }
}
```

#### 3.6.2 Cross-Border Coordination

For pests that may spread internationally:
- Notify national plant protection organizations (NPPO)
- Share data with neighboring countries via IPPC
- Coordinate with FAO regional offices

### 3.7 Alert Verification and Authentication

#### 3.7.1 Digital Signatures

All alerts MUST be digitally signed:

```json
{
  "signature": {
    "algorithm": "ES256",
    "issuer_public_key": "did:wia:rda-korea#key-1",
    "signature_value": "base64_encoded_signature",
    "timestamp": "2025-06-15T14:30:00Z"
  }
}
```

#### 3.7.2 Alert Verification Endpoint

**POST /alerts/verify**

Request:
```json
{
  "alert_id": "WIA-PEST-2025-0615-001",
  "signature": "base64_signature"
}
```

Response:
```json
{
  "valid": true,
  "issuer_verified": true,
  "timestamp_valid": true,
  "not_tampered": true,
  "issuer_info": {
    "organization": "RDA Korea",
    "verified_issuer": true,
    "trust_level": "government_authority"
  }
}
```

### 3.8 Alert Metrics and Reporting

#### 3.8.1 Key Performance Indicators

- **Alert Delivery Rate**: % of intended recipients reached
- **Delivery Time**: Time from issue to receipt
- **Response Rate**: % of farmers taking recommended action
- **False Positive Rate**: % of alerts not requiring action
- **Effectiveness**: Reduction in crop loss vs. no-alert scenario

#### 3.8.2 Alert Analytics

```json
{
  "alert_id": "WIA-PEST-2025-0615-001",
  "analytics": {
    "distribution": {
      "intended_recipients": 2840,
      "sms_delivered": 2701,
      "app_push_delivered": 1653,
      "email_delivered": 2398,
      "total_reach": 2748,
      "delivery_rate": 0.968
    },
    "engagement": {
      "alert_opened": 2103,
      "open_rate": 0.765,
      "link_clicks": 1247,
      "click_through_rate": 0.593
    },
    "response": {
      "farmers_surveyed": 487,
      "action_taken": 421,
      "action_rate": 0.865,
      "treatment_applied": 398,
      "monitoring_increased": 443
    },
    "outcome": {
      "estimated_yield_saved_percent": 38,
      "economic_benefit_usd": 6200000,
      "roi": 15.4
    }
  }
}
```

### 3.9 Alert Updates and Cancellations

#### 3.9.1 Alert Update

```json
{
  "update_id": "WIA-PEST-2025-0615-001-U1",
  "original_alert_id": "WIA-PEST-2025-0615-001",
  "update_type": "severity_change",
  "update_timestamp": "2025-06-17T10:00:00Z",
  "changes": {
    "severity_level": {
      "old": "critical",
      "new": "warning"
    },
    "population_density": {
      "old": 125,
      "new": 45
    },
    "reason": "Successful treatment reduced population density"
  }
}
```

#### 3.9.2 Alert Cancellation

```json
{
  "cancellation_id": "WIA-PEST-2025-0615-001-C1",
  "original_alert_id": "WIA-PEST-2025-0615-001",
  "cancellation_timestamp": "2025-06-20T14:00:00Z",
  "reason": "Pest outbreak successfully controlled, population below threshold",
  "all_clear": true
}
```

### 3.10 Internationalization (i18n)

#### 3.10.1 Supported Languages

- English (en)
- Korean (ko)
- Japanese (ja) - for export coordination
- Chinese (zh) - for regional coordination
- Spanish (es) - for technical references

#### 3.10.2 Korean Localization (한국화)

**Alert Terminology Mapping:**

| English | Korean (한글) | Notes |
|---------|------------|-------|
| Critical Alert | 긴급경보 | Highest priority |
| Warning | 경보 | High priority |
| Advisory | 주의보 | Medium priority |
| Info | 정보 | Low priority |
| Outbreak | 대발생 | Severe infestation |
| Economic Threshold | 경제적 피해수준 | Action level |
| Treatment | 방제 | Control measures |
| Monitoring | 예찰 | Surveillance |
| Extension Service | 농업기술센터 | Support office |

### 3.11 Compliance and Legal Requirements

#### 3.11.1 Korean Regulations

Compliance with:
- 식물방역법 (Plant Protection Act)
- 농약관리법 (Pesticide Control Act)
- 농촌진흥법 (Rural Development Act)
- 개인정보보호법 (Personal Information Protection Act)

#### 3.11.2 Data Privacy

- Opt-in for SMS/voice alerts
- Right to unsubscribe
- Location data protection
- Farmer identity confidentiality

### 3.12 Testing and Validation

#### 3.12.1 Alert System Testing

- Monthly test alerts (clearly marked as TEST)
- Annual full-scale drill
- Channel redundancy verification
- Failover testing

#### 3.12.2 Alert Message Validation

Required checks before sending:
- ✓ Valid alert structure (JSON schema)
- ✓ Digital signature present and valid
- ✓ Geographic coordinates valid
- ✓ Contact information current
- ✓ All required languages present
- ✓ No broken links in message
