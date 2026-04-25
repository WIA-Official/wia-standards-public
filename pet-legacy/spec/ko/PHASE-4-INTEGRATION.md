# WIA PET-LEGACY PHASE 4: 통합 명세서

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## 목차

1. [개요](#개요)
2. [동물병원 시스템 통합](#동물병원-시스템-통합)
3. [반려동물 묘지 통합](#반려동물-묘지-통합)
4. [소셜 미디어 통합](#소셜-미디어-통합)
5. [클라우드 저장소 통합](#클라우드-저장소-통합)
6. [결제 처리 통합](#결제-처리-통합)
7. [분석 통합](#분석-통합)
8. [이메일 및 통신 통합](#이메일-및-통신-통합)
9. [AI 서비스 통합](#ai-서비스-통합)
10. [타사 플랫폼 통합](#타사-플랫폼-통합)

---

## 1. 개요

### 1.1 목적

WIA PET-LEGACY Phase 4 명세서는 반려동물 추모 플랫폼을 외부 시스템, 서비스 및 플랫폼과 연결하기 위한 통합 표준을 정의합니다. 이러한 통합은 기능을 향상시키고, 워크플로를 간소화하며, 포괄적인 추모관 관리 기능을 제공합니다.

### 1.2 통합 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│              PET-LEGACY 플랫폼 코어                      │
│  ┌────────────────────────────────────────────────────┐  │
│  │         통합 게이트웨이                            │  │
│  │  - 인증                                           │  │
│  │  - 속도 제한                                      │  │
│  │  - 프로토콜 변환                                  │  │
│  │  - 오류 처리                                      │  │
│  └─────────────┬──────────────────────────────────────┘  │
└────────────────┼─────────────────────────────────────────┘
                 │
     ┌───────────┼───────────────────────┐
     │           │                       │
     ▼           ▼                       ▼
┌─────────┐ ┌──────────┐         ┌──────────────┐
│동물병원 │ │  묘지    │         │소셜 미디어   │
│ 시스템  │ │ 시스템   │         │  플랫폼      │
└─────────┘ └──────────┘         └──────────────┘
     ▼           ▼                       ▼
┌─────────┐ ┌──────────┐         ┌──────────────┐
│클라우드 │ │  결제    │         │    분석      │
│ 저장소  │ │프로세서  │         │   서비스     │
└─────────┘ └──────────┘         └──────────────┘
```

### 1.3 통합 유형

| 유형 | 설명 | 예시 |
|------|------|------|
| **데이터 동기화** | 양방향 데이터 동기화 | 동물병원 기록, 묘지 정보 |
| **API 통합** | 직접 API 호출 | 결제 처리, 분석 |
| **웹훅** | 이벤트 기반 알림 | 소셜 미디어, 이메일 트리거 |
| **OAuth** | 인증 위임 | Google Drive, Facebook 로그인 |
| **Import/Export** | 일괄 데이터 전송 | 사진 가져오기, 추모관 내보내기 |
| **임베디드** | 위젯/iframe 임베딩 | 웹사이트에 추모관 표시 |

### 1.4 통합 보안 표준

| 보안 조치 | 요구사항 | 구현 |
|----------|---------|------|
| **인증** | OAuth 2.0 또는 API 키 | 토큰 기반 인증 |
| **암호화** | 최소 TLS 1.3 | 모든 통신 |
| **데이터 검증** | 입력 삭제 | 서버 측 검증 |
| **속도 제한** | 통합별 제한 | API 게이트웨이 강제 |
| **감사 로깅** | 모든 통합 활동 | 중앙 집중식 로깅 |
| **오류 처리** | 우아한 저하 | 대체 메커니즘 |

---

## 2. 동물병원 시스템 통합

### 2.1 동물병원 진료 관리 시스템

#### 2.1.1 지원 시스템

| 시스템 | 통합 유형 | 기능 |
|-------|---------|------|
| **Avimark** | API, 파일 내보내기 | 의료 기록, 고객 정보 |
| **Cornerstone** | API, 파일 내보내기 | 환자 이력, 예약 |
| **eVetPractice** | API | 의료 기록, 청구서 |
| **VetView** | API, HL7 | 포괄적인 의료 데이터 |
| **ImproMed** | 파일 내보내기 | 환자 기록 |

#### 2.1.2 의료 기록 가져오기

```http
POST /v1/integrations/veterinary/import
Authorization: Bearer {token}
Content-Type: application/json

{
  "source": "avimark",
  "veterinaryClinic": {
    "clinicId": "clinic_5k7m9n2p4q",
    "name": "자비로운 동물병원",
    "veterinarian": "에밀리 첸 박사"
  },
  "petIdentification": {
    "clinicPatientId": "PAT-12345",
    "microchipId": "985112345678901",
    "name": "맥스"
  },
  "recordTypes": [
    "vaccinations",
    "medical_history",
    "prescriptions",
    "visit_notes"
  ],
  "dateRange": {
    "start": "2012-07-15",
    "end": "2024-03-10"
  },
  "linkToMemorial": "550e8400-e29b-41d4-a716-446655440000"
}
```

**응답:**
```json
{
  "success": true,
  "data": {
    "importId": "imp_7x9k2m4n6p",
    "status": "processing",
    "recordsFound": 87,
    "estimatedTime": "2분",
    "preview": {
      "vaccinations": 24,
      "visits": 52,
      "prescriptions": 11
    },
    "statusUrl": "/v1/integrations/veterinary/import/imp_7x9k2m4n6p"
  }
}
```

#### 2.1.3 의료 기록 구조

```json
{
  "medicalRecords": {
    "petId": "550e8400-e29b-41d4-a716-446655440000",
    "veterinaryCare": {
      "primaryClinic": {
        "clinicId": "clinic_5k7m9n2p4q",
        "name": "자비로운 동물병원",
        "address": "123 Veterinary Way, 포틀랜드, OR",
        "phone": "555-0123"
      },
      "primaryVeterinarian": {
        "name": "에밀리 첸 박사",
        "licenseNumber": "VET-OR-12345",
        "specialization": "일반 진료"
      }
    },
    "vaccinations": [
      {
        "vaccineName": "광견병",
        "date": "2023-05-20",
        "expirationDate": "2026-05-20",
        "manufacturer": "Merck",
        "administeredBy": "에밀리 첸 박사"
      }
    ],
    "medicalHistory": [
      {
        "visitDate": "2024-01-15",
        "visitType": "wellness_exam",
        "chiefComplaint": "연례 검진",
        "diagnosis": "건강함",
        "treatment": "치료 불필요",
        "notes": "나이에 비해 건강 상태 양호. 경미한 관절염 관찰됨.",
        "veterinarian": "에밀리 첸 박사",
        "weight": {
          "value": 32.5,
          "unit": "kg"
        }
      }
    ],
    "prescriptions": [
      {
        "medicationName": "카프로펜",
        "dosage": "75mg",
        "frequency": "하루 2회",
        "prescribedDate": "2023-08-10",
        "duration": "지속",
        "purpose": "관절염 통증 관리",
        "prescribedBy": "에밀리 첸 박사"
      }
    ]
  }
}
```

#### 2.1.4 안락사 기록 통합

```json
{
  "euthanasiaRecord": {
    "date": "2024-03-10",
    "time": "14:30:00",
    "location": "자택",
    "veterinarian": "에밀리 첸 박사",
    "clinic": "자비로운 동물병원",
    "reason": "삶의 질 - 노령 관련 쇠퇴",
    "guardianConsent": {
      "obtained": true,
      "consentForm": "CONSENT-2024-0310-MAX",
      "signedBy": "사라 존슨",
      "date": "2024-03-10"
    },
    "procedure": {
      "sedationGiven": true,
      "sedationMedication": "아세프로마진 + 부토르파놀",
      "euthanasiaAgent": "펜토바르비탈 나트륨",
      "dosage": "150mg/kg",
      "timeOfDeath": "14:45:00"
    },
    "finalDisposition": {
      "type": "cremation",
      "provider": "무지개 다리 반려동물 추모관",
      "certificateNumber": "CERT-2024-00127"
    }
  }
}
```

### 2.2 동물병원 데이터 프라이버시

```json
{
  "privacySettings": {
    "displayMedicalRecords": {
      "enabled": false,
      "visibleTo": "guardian_only",
      "redactedFields": ["cost_information", "insurance_details"]
    },
    "shareWithFamily": {
      "enabled": false,
      "requiresGuardianApproval": true
    },
    "includeInExport": {
      "enabled": true,
      "encrypted": true
    }
  }
}
```

### 2.3 동물병원 추모 프로그램

```json
{
  "clinicMemorialProgram": {
    "clinicId": "clinic_5k7m9n2p4q",
    "programName": "무지개 다리 추모 파트너십",
    "features": {
      "automaticMemorialCreation": {
        "enabled": true,
        "trigger": "euthanasia_record",
        "requiresGuardianConsent": true,
        "templateUsed": "compassionate_care_template"
      },
      "memorialPageWidget": {
        "enabled": true,
        "displayOnClinicWebsite": true
      },
      "tributeDonations": {
        "enabled": true,
        "beneficiary": "지역 동물 보호소",
        "donationUrl": "https://donate.example.org"
      }
    },
    "branding": {
      "includeClinicLogo": true,
      "customMessage": "우리의 소중한 환자들을 사랑으로 기억하며",
      "clinicColor": "#4A90E2"
    }
  }
}
```

---

## 3. 반려동물 묘지 통합

### 3.1 묘지 관리 시스템 통합

#### 3.1.1 구역 관리

```http
POST /v1/integrations/cemetery/reserve-plot
Authorization: Bearer {token}
Content-Type: application/json

{
  "cemeteryId": "cem_5k7m9n2p4q",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "burial",
  "plotSelection": {
    "section": "추억의 정원",
    "plotNumber": "A-127",
    "plotSize": "small",
    "plotType": "individual"
  },
  "intermentDetails": {
    "scheduledDate": "2024-03-12T14:00:00Z",
    "caretakerPresent": true
  },
  "markerDetails": {
    "markerType": "flat_bronze",
    "inscription": "맥스\n영원히 우리 마음속에\n2012 - 2024"
  }
}
```

**응답 (201 Created):**
```json
{
  "success": true,
  "data": {
    "reservationId": "res_3k5m7n9p2q",
    "plotNumber": "A-127",
    "status": "reserved",
    "intermentDate": "2024-03-12T14:00:00Z",
    "certificateNumber": "CERT-2024-00127",
    "totalCost": {
      "amount": 1500.00,
      "currency": "USD",
      "breakdown": {
        "plot": 800.00,
        "interment": 400.00,
        "marker": 300.00
      }
    }
  }
}
```

#### 3.1.2 화장 서비스

```json
{
  "cremationService": {
    "cemeteryId": "cem_5k7m9n2p4q",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "cremationType": "individual",
    "scheduledDate": "2024-03-11T10:00:00Z",
    "urnSelection": {
      "urnType": "wooden_personalized",
      "material": "cherry_wood",
      "size": "medium",
      "engraving": "맥스 - 2012-2024"
    },
    "ashesDisposition": {
      "type": "returned_to_family",
      "collectionDate": "2024-03-13T14:00:00Z",
      "collectedBy": "사라 존슨"
    },
    "memorialOptions": {
      "pawPrintKeepsake": true,
      "furClipping": true,
      "memorialCandle": true
    }
  }
}
```

#### 3.1.3 가상 묘지 통합

```json
{
  "virtualCemetery": {
    "enabled": true,
    "plotLocation": {
      "section": "추억의 정원",
      "plotNumber": "A-127",
      "coordinates": {
        "latitude": 45.523064,
        "longitude": -122.676483
      }
    },
    "virtualFeatures": {
      "3dPlotView": {
        "enabled": true,
        "url": "https://virtual.cemetery.com/plot/A-127"
      },
      "virtualVisitation": {
        "enabled": true,
        "activities": [
          "place_virtual_flowers",
          "light_candle",
          "leave_message"
        ]
      },
      "qrCodeMarker": {
        "enabled": true,
        "linkedUrl": "https://petlegacy.wia.org/memorial/550e8400"
      }
    }
  }
}
```

---

## 4. 소셜 미디어 통합

### 4.1 Facebook 통합

#### 4.1.1 Facebook 로그인 (OAuth)

```http
GET /v1/auth/facebook
응답: Facebook OAuth로 리다이렉트
```

**Facebook OAuth 흐름:**
```
1. 사용자가 "Facebook으로 로그인" 클릭
2. Facebook 인증으로 리다이렉트
3. 사용자가 권한 부여
4. Facebook이 인증 코드로 리다이렉트
5. 액세스 토큰으로 코드 교환
6. 사용자 계정 생성/연결
```

#### 4.1.2 Facebook에 추모관 공유

```http
POST /v1/integrations/facebook/share
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "shareType": "post",
  "content": {
    "message": "사랑하는 맥스를 기억하며. 추모관을 방문하여 당신의 추억을 공유하세요.",
    "link": "https://petlegacy.wia.org/memorial/550e8400",
    "privacy": "public"
  },
  "media": {
    "type": "photo",
    "url": "https://cdn.petlegacy.wia.org/share/550e8400-fb.jpg"
  }
}
```

### 4.2 Instagram 통합

```json
{
  "instagramShare": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "shareType": "story",
    "content": {
      "mediaType": "photo",
      "mediaUrl": "https://cdn.petlegacy.wia.org/share/550e8400-ig.jpg",
      "caption": "영원히 우리 마음속에 🐾 #PetMemorial #RainbowBridge",
      "link": "https://petlegacy.wia.org/memorial/550e8400"
    },
    "storySettings": {
      "duration": 24,
      "stickers": [
        {
          "type": "link",
          "url": "https://petlegacy.wia.org/memorial/550e8400",
          "text": "추모관 보기"
        }
      ]
    }
  }
}
```

### 4.3 소셜 미디어 분석

```json
{
  "socialAnalytics": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "period": {
      "start": "2024-03-15",
      "end": "2024-12-18"
    },
    "platforms": {
      "facebook": {
        "shares": 45,
        "reactions": 342,
        "comments": 67,
        "reach": 8920,
        "clicks": 234
      },
      "instagram": {
        "shares": 28,
        "likes": 489,
        "comments": 52,
        "reach": 6730
      },
      "twitter": {
        "retweets": 12,
        "likes": 98,
        "replies": 15,
        "impressions": 3450
      }
    },
    "totalEngagement": 1170,
    "totalReach": 19100
  }
}
```

---

## 5. 클라우드 저장소 통합

### 5.1 Google Drive 통합

```http
POST /v1/integrations/google-drive/backup
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "backupOptions": {
    "includeMedia": true,
    "includeTimeline": true,
    "includeComments": true,
    "organizationStructure": "by_year"
  },
  "googleDrive": {
    "folderId": "1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx",
    "folderName": "Pet Legacy - 맥스",
    "createSubfolders": true
  },
  "schedule": {
    "frequency": "weekly",
    "dayOfWeek": "sunday",
    "time": "02:00:00"
  }
}
```

**응답:**
```json
{
  "success": true,
  "data": {
    "backupId": "bkp_5k7m9n2p4q",
    "status": "scheduled",
    "nextBackup": "2024-12-22T02:00:00Z",
    "googleDriveFolder": {
      "folderId": "1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx",
      "folderUrl": "https://drive.google.com/drive/folders/1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx"
    }
  }
}
```

### 5.2 Dropbox 통합

```json
{
  "dropboxBackup": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "dropboxPath": "/Apps/PetLegacy/맥스",
    "syncOptions": {
      "bidirectional": false,
      "autoSync": true,
      "conflictResolution": "server_wins"
    },
    "fileOrganization": {
      "structure": "by_type",
      "folders": {
        "photos": "/사진",
        "videos": "/비디오",
        "documents": "/문서",
        "timeline": "/타임라인"
      }
    }
  }
}
```

### 5.3 iCloud 통합

```json
{
  "iCloudBackup": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "iCloudDrive": {
      "enabled": true,
      "folder": "PetLegacy/맥스"
    },
    "iCloudPhotos": {
      "enabled": true,
      "album": "맥스 추모관",
      "sharedAlbum": true,
      "subscribers": [
        "michael.j@icloud.com"
      ]
    },
    "syncFrequency": "real_time"
  }
}
```

---

## 6. 결제 처리 통합

### 6.1 Stripe 통합

#### 6.1.1 결제 처리

```http
POST /v1/integrations/stripe/create-payment
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "paymentType": "memorial_upgrade",
  "items": [
    {
      "sku": "memorial_pro_annual",
      "description": "프로 추모관 (연간)",
      "amount": 9900,
      "quantity": 1
    },
    {
      "sku": "storage_addon_50gb",
      "description": "추가 50GB 저장 공간",
      "amount": 1900,
      "quantity": 1
    }
  ],
  "currency": "USD",
  "customer": {
    "email": "sarah.j@example.com",
    "name": "사라 존슨"
  }
}
```

**응답:**
```json
{
  "success": true,
  "data": {
    "paymentIntentId": "pi_3k5m7n9p2q",
    "clientSecret": "pi_3k5m7n9p2q_secret_abc123",
    "amount": 11800,
    "currency": "USD",
    "status": "requires_payment_method",
    "checkoutUrl": "https://checkout.stripe.com/pay/cs_test_abc123"
  }
}
```

#### 6.1.2 구독 관리

```json
{
  "subscription": {
    "subscriptionId": "sub_5k7m9n2p4q",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "plan": {
      "id": "plan_pro_annual",
      "name": "프로 추모관",
      "interval": "year",
      "amount": 9900,
      "currency": "USD"
    },
    "status": "active",
    "currentPeriod": {
      "start": "2024-03-15T00:00:00Z",
      "end": "2025-03-15T00:00:00Z"
    },
    "cancelAtPeriodEnd": false,
    "defaultPaymentMethod": {
      "type": "card",
      "last4": "4242",
      "brand": "visa",
      "expMonth": 12,
      "expYear": 2026
    }
  }
}
```

### 6.2 PayPal 통합

```json
{
  "paypalPayment": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "orderId": "7X987654321",
    "orderType": "one_time_payment",
    "items": [
      {
        "name": "추모관 프로 업그레이드",
        "unitAmount": 99.00,
        "quantity": 1
      }
    ],
    "totalAmount": 99.00,
    "currency": "USD",
    "status": "completed",
    "transactionId": "8XY12345678",
    "completedAt": "2024-12-18T19:30:00Z"
  }
}
```

### 6.3 기부 처리

```json
{
  "tributeDonation": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "donationType": "in_memory_of",
    "amount": 50.00,
    "currency": "USD",
    "beneficiary": {
      "type": "charity",
      "name": "지역 동물 보호소",
      "ein": "12-3456789"
    },
    "donor": {
      "name": "제니퍼 스미스",
      "email": "jennifer@example.com",
      "isAnonymous": false,
      "includeMessage": true,
      "message": "맥스를 사랑으로 기억하며"
    },
    "paymentMethod": "stripe",
    "transactionId": "txn_9k2m5n7p8q",
    "taxDeductible": true
  }
}
```

---

## 7. 분석 통합

### 7.1 Google Analytics 통합

```javascript
// Google Analytics 4 구성
{
  "ga4": {
    "measurementId": "G-XXXXXXXXXX",
    "events": {
      "memorial_created": {
        "category": "engagement",
        "action": "create_memorial",
        "label": "memorial_id"
      },
      "media_uploaded": {
        "category": "content",
        "action": "upload_media",
        "label": "media_type"
      },
      "memorial_viewed": {
        "category": "engagement",
        "action": "view_memorial",
        "label": "memorial_id"
      }
    },
    "customDimensions": {
      "pet_species": "dimension1",
      "memorial_age": "dimension2",
      "user_role": "dimension3"
    },
    "customMetrics": {
      "photo_count": "metric1",
      "video_count": "metric2",
      "tribute_count": "metric3"
    }
  }
}
```

### 7.2 Mixpanel 통합

```json
{
  "mixpanel": {
    "projectToken": "abc123def456",
    "trackingEvents": [
      {
        "event": "추모관 생성됨",
        "properties": {
          "memorial_id": "550e8400",
          "pet_species": "dog",
          "pet_breed": "골든 리트리버",
          "memorial_type": "pro"
        }
      },
      {
        "event": "미디어 업로드됨",
        "properties": {
          "memorial_id": "550e8400",
          "media_type": "photo",
          "file_size": 3457280,
          "upload_source": "mobile_app"
        }
      }
    ],
    "userProperties": {
      "memorials_created": 1,
      "total_photos_uploaded": 324,
      "subscription_tier": "pro",
      "signup_date": "2024-03-15"
    }
  }
}
```

---

## 8. 이메일 및 통신 통합

### 8.1 SendGrid 통합

```http
POST /v1/integrations/sendgrid/send
Authorization: Bearer {token}
Content-Type: application/json

{
  "templateId": "memorial_anniversary",
  "to": {
    "email": "sarah.j@example.com",
    "name": "사라 존슨"
  },
  "dynamicData": {
    "petName": "맥스",
    "anniversaryType": "passing",
    "yearsSince": 1,
    "memorialUrl": "https://petlegacy.wia.org/memorial/550e8400",
    "photoUrl": "https://cdn.petlegacy.wia.org/photos/max-profile.jpg"
  },
  "categories": ["memorial", "anniversary"],
  "sendAt": "2025-03-10T09:00:00Z"
}
```

### 8.2 Twilio 통합 (SMS/전화)

```json
{
  "twilioSms": {
    "to": "+15551234567",
    "from": "+15559876543",
    "body": "맥스의 추모관에 새로운 헌사가 게시되었습니다. 여기에서 확인하세요: https://pet.wia/m/abc123",
    "statusCallback": "https://api.petlegacy.wia.org/v1/webhooks/twilio/status"
  }
}
```

---

## 9. AI 서비스 통합

### 9.1 OpenAI 통합

```http
POST /v1/integrations/openai/generate-summary
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "model": "gpt-4",
  "prompt": {
    "type": "memorial_summary",
    "tone": "warm_and_loving",
    "length": "medium",
    "context": {
      "petName": "맥스",
      "species": "dog",
      "breed": "골든 리트리버",
      "lifespan": "2012-2024",
      "personality": ["충성스러운", "온화한", "장난기 있는"],
      "favoriteActivities": ["수영", "공놀이", "하이킹"]
    }
  }
}
```

**응답:**
```json
{
  "success": true,
  "data": {
    "summary": "맥스는 단순한 반려동물이 아니라, 매일을 무조건적인 사랑과 무한한 기쁨으로 채워준 소중한 가족 구성원이었습니다. 2012년에 태어난 이 온화한 골든 리트리버는 평생 지속될 추억을 만들며 11년이라는 아름다운 시간을 보냈습니다. 수영을 위해 물에 뛰어들든, 좋아하는 테니스공을 쫓든, 가족 하이킹에서 새로운 길을 탐험하든, 맥스는 모든 순간을 열정과 꼬리 흔들기로 맞이했습니다. 그의 충성스럽고 장난기 있는 정신은 만나는 모든 사람을 감동시켰고, 수많은 마음에 발자국을 남겼습니다. 추모관에 담긴 324장의 사진과 52개의 소중한 순간들은 잘 살았고 깊이 사랑받은 삶의 이야기를 전합니다. 맥스가 2024년에 무지개 다리를 건넜지만, 그의 추억은 그를 알았던 모든 이에게 위로와 미소를 계속 가져다줍니다.",
    "wordCount": 142,
    "generatedAt": "2024-12-18T20:00:00Z"
  }
}
```

### 9.2 컴퓨터 비전 통합

```json
{
  "visionAI": {
    "provider": "google_cloud_vision",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "analysis": {
      "labels": [
        {"description": "개", "score": 0.99},
        {"description": "골든 리트리버", "score": 0.97},
        {"description": "해변", "score": 0.95},
        {"description": "바다", "score": 0.93},
        {"description": "행복", "score": 0.89}
      ],
      "faces": {
        "detected": 1,
        "emotions": {
          "joy": 0.92,
          "playful": 0.87
        }
      },
      "objects": [
        {"name": "동물", "score": 0.98},
        {"name": "물", "score": 0.94},
        {"name": "모래", "score": 0.91}
      ]
    },
    "suggestedTags": ["해변", "바다", "골든_리트리버", "행복", "여름"],
    "suggestedCaption": "아름다운 날 해변을 즐기는 맥스"
  }
}
```

### 9.3 비디오 처리 통합

```json
{
  "videoProcessing": {
    "provider": "cloudinary",
    "assetId": "770e8400-e29b-41d4-a716-446655440015",
    "operations": [
      {
        "type": "thumbnail_generation",
        "timestamps": [0, 5, 10, 15, 20],
        "outputFormat": "jpg",
        "quality": "auto:best"
      },
      {
        "type": "transcoding",
        "formats": [
          {
            "codec": "h264",
            "resolution": "1080p",
            "bitrate": "5000k"
          },
          {
            "codec": "h264",
            "resolution": "720p",
            "bitrate": "2500k"
          }
        ]
      },
      {
        "type": "highlight_reel",
        "duration": 30,
        "music": "gentle_piano",
        "transitions": "fade"
      }
    ]
  }
}
```

---

## 10. 타사 플랫폼 통합

### 10.1 Zapier 통합

```json
{
  "zapierIntegration": {
    "triggers": [
      {
        "event": "memorial_created",
        "zapUrl": "https://zapier.com/hooks/catch/123456/abc123/",
        "actions": [
          "Slack에 알림 보내기",
          "Google Sheets에 행 추가",
          "Trello 카드 생성"
        ]
      },
      {
        "event": "media_uploaded",
        "zapUrl": "https://zapier.com/hooks/catch/123456/def456/",
        "actions": [
          "Dropbox에 백업",
          "이메일 알림 보내기"
        ]
      }
    ]
  }
}
```

### 10.2 WordPress 통합

```json
{
  "wordpressPlugin": {
    "pluginName": "WIA Pet Legacy",
    "version": "1.0.0",
    "features": {
      "shortcode": {
        "enabled": true,
        "syntax": "[pet_memorial id='550e8400' display='timeline']",
        "attributes": {
          "id": "필수",
          "display": "timeline|gallery|tribute_wall",
          "theme": "light|dark|custom"
        }
      },
      "widget": {
        "enabled": true,
        "widgetAreas": ["sidebar", "footer"],
        "configuration": {
          "memorialId": "550e8400",
          "displayMode": "featured_photo",
          "showTributeCount": true
        }
      }
    }
  }
}
```

### 10.3 모바일 앱 통합 (iOS/Android)

```json
{
  "mobileSDK": {
    "platform": "ios",
    "version": "1.0.0",
    "features": {
      "authentication": {
        "oauth": true,
        "biometric": true,
        "appleSignIn": true
      },
      "mediaUpload": {
        "photoLibrary": true,
        "camera": true,
        "livePhoto": true,
        "backgroundUpload": true
      },
      "pushNotifications": {
        "enabled": true,
        "types": [
          "new_tribute",
          "memorial_anniversary",
          "family_member_activity"
        ]
      },
      "widgets": {
        "homeScreen": true,
        "lockScreen": true,
        "appleWatch": true
      }
    }
  }
}
```

---

## 부록 A: 통합 테스트 체크리스트

### 인증 및 권한 부여
- [ ] OAuth 2.0 흐름이 성공적으로 완료됨
- [ ] API 키가 올바르게 검증됨
- [ ] 토큰 갱신이 제대로 작동함
- [ ] 권한 범위가 강제됨
- [ ] 속도 제한이 예상대로 작동함

### 데이터 통합
- [ ] 데이터 동기화가 해당되는 경우 양방향임
- [ ] 충돌 해결이 올바르게 작동함
- [ ] 데이터 검증이 잘못된 입력을 방지함
- [ ] 오류 처리가 우아함
- [ ] 재시도 로직이 실패를 처리함

### 웹훅 통합
- [ ] 웹훅이 안정적으로 전달됨
- [ ] 서명 검증이 작동함
- [ ] 재시도 로직이 실패를 처리함
- [ ] 이벤트가 중복 제거됨
- [ ] 페이로드 형식이 올바름

---

## 부록 B: 통합 속도 제한

| 통합 | 요청/분 | 요청/시간 | 요청/일 |
|------|--------|----------|--------|
| 동물병원 시스템 | 60 | 1,000 | 10,000 |
| 묘지 시스템 | 30 | 500 | 5,000 |
| 소셜 미디어 | 100 | 3,000 | 25,000 |
| 클라우드 저장소 | 120 | 5,000 | 50,000 |
| 결제 처리 | 300 | 10,000 | 100,000 |
| 분석 | 600 | 20,000 | 200,000 |
| 이메일 서비스 | 1,000 | 30,000 | 250,000 |
| AI 서비스 | 60 | 1,000 | 10,000 |

---

## 부록 C: 통합 SLA 요구사항

| 서비스 유형 | 가동 시간 | 응답 시간 | 지원 수준 |
|-----------|---------|---------|---------|
| 중요 (결제, 인증) | 99.9% | < 200ms | 24/7 |
| 높은 우선순위 (미디어, API) | 99.5% | < 500ms | 업무 시간 |
| 표준 (분석, 소셜) | 99.0% | < 1000ms | 이메일 지원 |
| 백그라운드 (백업, 동기화) | 95.0% | < 5000ms | 최선 노력 |

---

**弘益人間 (홍익인간)** - 모든 인류에 이로움
© 2025 WIA
MIT License
