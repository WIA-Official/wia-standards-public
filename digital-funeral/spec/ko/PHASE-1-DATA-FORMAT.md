# WIA 디지털 장례 표준 - Phase 1: Data Format 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**Primary Color:** #64748B (Slate)
**시리즈:** Digital Death Services

---

## 목차

1. [소개](#소개)
2. [핵심 데이터 모델](#핵심-데이터-모델)
3. [장례 서비스 모델](#장례-서비스-모델)
4. [참석자 및 손님 관리](#참석자-및-손님-관리)
5. [추모 및 헌사 모델](#추모-및-헌사-모델)
6. [금융 및 기부 모델](#금융-및-기부-모델)
7. [묘지 및 안치 모델](#묘지-및-안치-모델)
8. [데이터 검증 규칙](#데이터-검증-규칙)
9. [참조 테이블](#참조-테이블)
10. [구현 예제](#구현-예제)

---

## 1. 소개

### 1.1 목적

본 명세서는 디지털 장례 서비스를 위한 표준화된 데이터 형식을 정의하여, 장례식장, 추모 플랫폼, 스트리밍 서비스 및 관련 서비스 제공업체 간의 상호 운용성을 가능하게 합니다. 이 표준은 전통적이고 현대적인 장례 관행 모두를 지원하며, 가상 예식, 라이브 스트리밍, 디지털 추모관을 포함합니다.

### 1.2 범위

데이터 형식 명세서는 다음을 포함합니다:
- 고인 정보 및 부고
- 장례 서비스 계획 및 일정
- 가상 및 하이브리드 예식 구성
- 손님 관리 및 RSVP 추적
- 추모 헌사 및 조문 메시지
- 금융 거래 및 기부
- 묘지 및 화장장 통합
- 사전 장례 계획

### 1.3 설계 원칙

- **Privacy First**: 민감한 정보는 적절한 액세스 제어로 보호
- **Cultural Sensitivity**: 다양한 종교 및 문화적 관행 지원
- **Accessibility**: 모든 사용자를 위한 다양한 형식의 정보 제공
- **Permanence**: 추모 목적의 장기 데이터 보존
- **Flexibility**: 다양한 장례 서비스 유형 및 전통에 적응 가능

---

## 2. 핵심 데이터 모델

### 2.1 고인 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DeceasedPerson",
  "type": "object",
  "required": [
    "id",
    "legalName",
    "dateOfBirth",
    "dateOfDeath",
    "privacy"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid",
      "description": "고인의 고유 식별자"
    },
    "legalName": {
      "type": "object",
      "required": ["firstName", "lastName"],
      "properties": {
        "prefix": {
          "type": "string",
          "examples": ["Mr.", "Mrs.", "Dr.", "Rev."]
        },
        "firstName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "middleName": {
          "type": "string",
          "maxLength": 100
        },
        "lastName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "suffix": {
          "type": "string",
          "examples": ["Jr.", "Sr.", "III", "PhD"]
        },
        "preferredName": {
          "type": "string",
          "description": "일반적으로 알려진 이름"
        }
      }
    },
    "dateOfBirth": {
      "type": "string",
      "format": "date",
      "description": "ISO 8601 날짜 형식"
    },
    "dateOfDeath": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 날짜-시간 형식"
    },
    "placeOfBirth": {
      "type": "object",
      "properties": {
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"}
      }
    },
    "placeOfDeath": {
      "type": "object",
      "properties": {
        "facility": {"type": "string"},
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"}
      }
    },
    "biography": {
      "type": "object",
      "properties": {
        "short": {
          "type": "string",
          "maxLength": 500,
          "description": "간략한 부고 요약"
        },
        "full": {
          "type": "string",
          "maxLength": 10000,
          "description": "전체 생애 이야기"
        },
        "achievements": {
          "type": "array",
          "items": {"type": "string"},
          "description": "주목할 만한 업적"
        },
        "hobbies": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "family": {
      "type": "object",
      "properties": {
        "spouse": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["current", "predeceased", "divorced"]
              }
            }
          }
        },
        "children": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        },
        "parents": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        },
        "siblings": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        }
      }
    },
    "photos": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "url": {"type": "string", "format": "uri"},
          "caption": {"type": "string"},
          "isPrimary": {"type": "boolean"},
          "yearTaken": {"type": "integer"},
          "uploadedAt": {"type": "string", "format": "date-time"}
        }
      }
    },
    "privacy": {
      "type": "object",
      "required": ["level"],
      "properties": {
        "level": {
          "type": "string",
          "enum": ["public", "family-only", "private"],
          "description": "고인 정보의 전체 공개 수준"
        },
        "showDateOfBirth": {"type": "boolean", "default": true},
        "showPlaceOfBirth": {"type": "boolean", "default": true},
        "showFullBiography": {"type": "boolean", "default": true},
        "showFamilyMembers": {"type": "boolean", "default": true},
        "allowPublicCondolences": {"type": "boolean", "default": true}
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"},
        "createdBy": {"type": "string", "description": "User ID"},
        "verificationStatus": {
          "type": "string",
          "enum": ["pending", "verified", "disputed"]
        }
      }
    }
  }
}
```

### 2.2 부고 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Obituary",
  "type": "object",
  "required": ["id", "deceasedId", "title", "content", "status"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid",
      "description": "DeceasedPerson 참조"
    },
    "title": {
      "type": "string",
      "maxLength": 200,
      "examples": ["고 김철수님을 추모하며"]
    },
    "content": {
      "type": "object",
      "properties": {
        "text": {
          "type": "string",
          "description": "일반 텍스트 부고"
        },
        "html": {
          "type": "string",
          "description": "HTML 형식 부고"
        },
        "markdown": {
          "type": "string",
          "description": "Markdown 형식 부고"
        }
      }
    },
    "publishedDate": {
      "type": "string",
      "format": "date-time"
    },
    "expiryDate": {
      "type": "string",
      "format": "date-time",
      "description": "부고 게시 중단 일시"
    },
    "newspapers": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "name": {"type": "string"},
          "publicationDate": {"type": "string", "format": "date"},
          "url": {"type": "string", "format": "uri"},
          "cost": {"type": "number"}
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["draft", "pending-approval", "published", "archived"]
    },
    "viewCount": {
      "type": "integer",
      "minimum": 0
    },
    "metadata": {
      "type": "object",
      "properties": {
        "language": {"type": "string", "pattern": "^[a-z]{2}$"},
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

---

## 3. 장례 서비스 모델

### 3.1 장례 서비스 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FuneralService",
  "type": "object",
  "required": [
    "id",
    "deceasedId",
    "serviceType",
    "startDateTime",
    "venue"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "serviceType": {
      "type": "string",
      "enum": [
        "viewing",
        "visitation",
        "wake",
        "funeral",
        "memorial",
        "celebration-of-life",
        "graveside",
        "committal",
        "virtual"
      ]
    },
    "religiousTradition": {
      "type": "string",
      "enum": [
        "christian",
        "catholic",
        "jewish",
        "muslim",
        "buddhist",
        "hindu",
        "non-religious",
        "interfaith",
        "other"
      ]
    },
    "startDateTime": {
      "type": "string",
      "format": "date-time"
    },
    "endDateTime": {
      "type": "string",
      "format": "date-time"
    },
    "timezone": {
      "type": "string",
      "examples": ["Asia/Seoul", "America/New_York"]
    },
    "venue": {
      "type": "object",
      "required": ["type"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["funeral-home", "church", "cemetery", "crematorium", "private-residence", "virtual", "other"]
        },
        "name": {"type": "string"},
        "address": {
          "type": "object",
          "properties": {
            "street": {"type": "string"},
            "city": {"type": "string"},
            "state": {"type": "string"},
            "postalCode": {"type": "string"},
            "country": {"type": "string"}
          }
        },
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "capacity": {"type": "integer"},
        "accessibility": {
          "type": "object",
          "properties": {
            "wheelchairAccessible": {"type": "boolean"},
            "parkingAvailable": {"type": "boolean"},
            "assistiveListeningDevices": {"type": "boolean"}
          }
        }
      }
    },
    "virtualService": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean", "default": false},
        "streamingUrl": {"type": "string", "format": "uri"},
        "platform": {
          "type": "string",
          "enum": ["zoom", "youtube", "facebook", "custom"]
        },
        "accessCode": {"type": "string"},
        "recordingEnabled": {"type": "boolean"},
        "recordingUrl": {"type": "string", "format": "uri"},
        "chatEnabled": {"type": "boolean"},
        "maxVirtualAttendees": {"type": "integer"}
      }
    },
    "program": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "order": {"type": "integer"},
          "type": {
            "type": "string",
            "enum": ["prayer", "eulogy", "music", "reading", "video-tribute", "open-mic", "other"]
          },
          "title": {"type": "string"},
          "description": {"type": "string"},
          "duration": {"type": "integer", "description": "소요 시간(분)"},
          "performer": {"type": "string"}
        }
      }
    },
    "officiant": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "title": {"type": "string"},
        "organization": {"type": "string"},
        "contact": {
          "type": "object",
          "properties": {
            "phone": {"type": "string"},
            "email": {"type": "string", "format": "email"}
          }
        }
      }
    },
    "dresscode": {
      "type": "string",
      "examples": ["formal", "business casual", "casual", "traditional mourning"]
    },
    "specialInstructions": {
      "type": "string",
      "maxLength": 1000
    },
    "rsvpRequired": {
      "type": "boolean",
      "default": false
    },
    "rsvpDeadline": {
      "type": "string",
      "format": "date-time"
    },
    "expectedAttendance": {
      "type": "integer"
    },
    "status": {
      "type": "string",
      "enum": ["planned", "confirmed", "in-progress", "completed", "cancelled", "postponed"]
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"},
        "createdBy": {"type": "string"}
      }
    }
  }
}
```

### 3.2 사전 장례 계획 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PreNeedPlan",
  "type": "object",
  "required": ["id", "planHolderId", "status"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "planHolderId": {
      "type": "string",
      "format": "uuid",
      "description": "계획이 수립된 개인"
    },
    "status": {
      "type": "string",
      "enum": ["draft", "active", "paid-in-full", "cancelled", "executed"]
    },
    "createdDate": {
      "type": "string",
      "format": "date-time"
    },
    "servicePreferences": {
      "type": "object",
      "properties": {
        "serviceType": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["viewing", "funeral", "memorial", "celebration-of-life", "graveside"]
          }
        },
        "religiousTradition": {"type": "string"},
        "preferredVenue": {"type": "string"},
        "musicSelections": {
          "type": "array",
          "items": {"type": "string"}
        },
        "readings": {
          "type": "array",
          "items": {"type": "string"}
        },
        "specialRequests": {"type": "string"}
      }
    },
    "dispositionPreferences": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["burial", "cremation", "natural-burial", "aquamation", "donation"]
        },
        "casketType": {"type": "string"},
        "urnType": {"type": "string"},
        "vaultRequired": {"type": "boolean"},
        "cemetery": {"type": "string"},
        "plotLocation": {"type": "string"}
      }
    },
    "financialPlan": {
      "type": "object",
      "properties": {
        "estimatedCost": {"type": "number"},
        "amountPaid": {"type": "number"},
        "paymentPlan": {
          "type": "string",
          "enum": ["lump-sum", "installments", "insurance-funded", "trust-funded"]
        },
        "insurancePolicy": {"type": "string"},
        "trustAccount": {"type": "string"}
      }
    },
    "documents": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {
            "type": "string",
            "enum": ["contract", "insurance-policy", "will", "advance-directive", "other"]
          },
          "url": {"type": "string", "format": "uri"},
          "uploadedAt": {"type": "string", "format": "date-time"}
        }
      }
    },
    "contacts": {
      "type": "object",
      "properties": {
        "primaryContact": {
          "type": "object",
          "properties": {
            "name": {"type": "string"},
            "relationship": {"type": "string"},
            "phone": {"type": "string"},
            "email": {"type": "string", "format": "email"}
          }
        },
        "alternateContacts": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "relationship": {"type": "string"},
              "phone": {"type": "string"},
              "email": {"type": "string", "format": "email"}
            }
          }
        }
      }
    }
  }
}
```

---

## 4. 참석자 및 손님 관리

### 4.1 손님 RSVP Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "GuestRSVP",
  "type": "object",
  "required": ["id", "serviceId", "attendeeInfo", "response"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "serviceId": {
      "type": "string",
      "format": "uuid"
    },
    "attendeeInfo": {
      "type": "object",
      "required": ["name", "email"],
      "properties": {
        "name": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "phone": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "response": {
      "type": "string",
      "enum": ["attending", "not-attending", "maybe", "virtual-only"]
    },
    "attendanceType": {
      "type": "string",
      "enum": ["in-person", "virtual", "both"]
    },
    "numberOfGuests": {
      "type": "integer",
      "minimum": 1,
      "maximum": 10
    },
    "guestNames": {
      "type": "array",
      "items": {"type": "string"}
    },
    "dietaryRestrictions": {
      "type": "array",
      "items": {"type": "string"}
    },
    "accessibilityNeeds": {
      "type": "string"
    },
    "submittedAt": {
      "type": "string",
      "format": "date-time"
    },
    "updatedAt": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

---

## 5. 추모 및 헌사 모델

### 5.1 조문 메시지 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "CondolenceMessage",
  "type": "object",
  "required": ["id", "deceasedId", "authorName", "message"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "authorName": {
      "type": "string",
      "maxLength": 100
    },
    "authorEmail": {
      "type": "string",
      "format": "email"
    },
    "relationship": {
      "type": "string",
      "examples": ["family", "friend", "colleague", "neighbor", "other"]
    },
    "message": {
      "type": "string",
      "minLength": 1,
      "maxLength": 5000
    },
    "isPublic": {
      "type": "boolean",
      "default": true
    },
    "moderationStatus": {
      "type": "string",
      "enum": ["pending", "approved", "rejected", "flagged"],
      "default": "pending"
    },
    "attachments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {"type": "string", "enum": ["photo", "video", "audio"]},
          "url": {"type": "string", "format": "uri"},
          "caption": {"type": "string"}
        }
      }
    },
    "postedAt": {
      "type": "string",
      "format": "date-time"
    },
    "metadata": {
      "type": "object",
      "properties": {
        "ipAddress": {"type": "string"},
        "userAgent": {"type": "string"}
      }
    }
  }
}
```

### 5.2 추모 헌사 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MemorialTribute",
  "type": "object",
  "required": ["id", "deceasedId", "type", "content"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "type": {
      "type": "string",
      "enum": ["story", "photo-album", "video-montage", "audio-recording", "poem", "artwork"]
    },
    "title": {
      "type": "string",
      "maxLength": 200
    },
    "content": {
      "type": "object",
      "properties": {
        "text": {"type": "string"},
        "mediaUrls": {
          "type": "array",
          "items": {"type": "string", "format": "uri"}
        },
        "embedCode": {"type": "string"}
      }
    },
    "author": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "isPublic": {
      "type": "boolean",
      "default": true
    },
    "viewCount": {
      "type": "integer",
      "minimum": 0
    }
  }
}
```

---

## 6. 금융 및 기부 모델

### 6.1 기부 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Donation",
  "type": "object",
  "required": ["id", "deceasedId", "amount", "currency", "donorInfo"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "amount": {
      "type": "number",
      "minimum": 0
    },
    "currency": {
      "type": "string",
      "pattern": "^[A-Z]{3}$",
      "default": "KRW"
    },
    "donorInfo": {
      "type": "object",
      "required": ["name"],
      "properties": {
        "name": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "isAnonymous": {"type": "boolean", "default": false}
      }
    },
    "recipient": {
      "type": "object",
      "properties": {
        "type": {
          "type": "string",
          "enum": ["charity", "family", "memorial-fund", "other"]
        },
        "name": {"type": "string"},
        "taxId": {"type": "string"}
      }
    },
    "inMemoryOf": {
      "type": "string",
      "description": "고인의 성함"
    },
    "message": {
      "type": "string",
      "maxLength": 500
    },
    "paymentMethod": {
      "type": "string",
      "enum": ["credit-card", "debit-card", "paypal", "venmo", "bank-transfer", "check"]
    },
    "transactionId": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "completed", "failed", "refunded"]
    },
    "donatedAt": {
      "type": "string",
      "format": "date-time"
    },
    "receiptUrl": {
      "type": "string",
      "format": "uri"
    }
  }
}
```

### 6.2 화환 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FloralArrangement",
  "type": "object",
  "required": ["id", "serviceId", "type", "sender"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "serviceId": {
      "type": "string",
      "format": "uuid"
    },
    "type": {
      "type": "string",
      "examples": ["standing-spray", "casket-spray", "wreath", "basket", "bouquet"]
    },
    "description": {
      "type": "string"
    },
    "florist": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "phone": {"type": "string"},
        "orderNumber": {"type": "string"}
      }
    },
    "sender": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "organization": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "cardMessage": {
      "type": "string",
      "maxLength": 200
    },
    "deliveryDate": {
      "type": "string",
      "format": "date"
    },
    "deliveryLocation": {
      "type": "string",
      "enum": ["funeral-home", "church", "cemetery", "residence"]
    },
    "cost": {
      "type": "number"
    },
    "photoUrl": {
      "type": "string",
      "format": "uri"
    }
  }
}
```

---

## 7. 묘지 및 안치 모델

### 7.1 안치 기록 Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "IntermentRecord",
  "type": "object",
  "required": ["id", "deceasedId", "cemetery", "intermentDate"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "cemetery": {
      "type": "object",
      "required": ["name"],
      "properties": {
        "id": {"type": "string", "format": "uuid"},
        "name": {"type": "string"},
        "address": {
          "type": "object",
          "properties": {
            "street": {"type": "string"},
            "city": {"type": "string"},
            "state": {"type": "string"},
            "postalCode": {"type": "string"},
            "country": {"type": "string"}
          }
        },
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "plotLocation": {
      "type": "object",
      "properties": {
        "section": {"type": "string"},
        "lot": {"type": "string"},
        "grave": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "intermentType": {
      "type": "string",
      "enum": ["full-body", "cremains", "above-ground", "mausoleum", "columbarium"]
    },
    "intermentDate": {
      "type": "string",
      "format": "date-time"
    },
    "casketVault": {
      "type": "object",
      "properties": {
        "casketType": {"type": "string"},
        "vaultType": {"type": "string"},
        "vaultRequired": {"type": "boolean"}
      }
    },
    "monument": {
      "type": "object",
      "properties": {
        "type": {"type": "string", "enum": ["headstone", "marker", "monument", "plaque", "none"]},
        "material": {"type": "string"},
        "inscription": {"type": "string"},
        "unveilingDate": {"type": "string", "format": "date"}
      }
    },
    "perpetualCare": {
      "type": "boolean"
    },
    "deedHolder": {
      "type": "string"
    }
  }
}
```

---

## 8. 데이터 검증 규칙

### 8.1 날짜 및 시간 검증

| 필드 | 규칙 | 설명 |
|-----|------|-----|
| dateOfDeath | 현재 날짜 이하여야 함 | 미래 날짜 불가 |
| dateOfBirth | dateOfDeath보다 작아야 함 | 사망일 이전이어야 함 |
| serviceStartDateTime | dateOfDeath 이후여야 함 | 일반적으로 사망 이후 |
| rsvpDeadline | serviceStartDateTime 이전이어야 함 | 서비스 전 RSVP 마감 |
| expiryDate | publishedDate 이후여야 함 | 게시일 이후 만료 |

### 8.2 개인정보 보호 및 액세스 제어

| Privacy Level | 접근 가능 대상 | 제한사항 |
|--------------|---------------|---------|
| public | 링크가 있는 모든 사람 | 모든 정보 공개 |
| family-only | 인증된 가족 구성원 | 인증 필요 |
| private | 승인된 사용자만 | 완전한 액세스 제어 필요 |

### 8.3 콘텐츠 검토 규칙

| 콘텐츠 유형 | 검토 수준 | 자동 승인 기준 |
|-----------|---------|--------------|
| 조문 메시지 | 보통 | 길이 < 500자, 금지어 없음 |
| 사진 | 엄격 | 파일 크기 < 10MB, 승인된 형식 |
| 비디오 헌사 | 엄격 | 재생 시간 < 10분, 승인된 형식 |
| 부고 | 검토 | 검증된 사용자가 제출 |

---

## 9. 참조 테이블

### 9.1 서비스 유형 범주

| 범주 | 서비스 유형 | 일반적 소요 시간 |
|-----|-----------|----------------|
| 사전 서비스 | Viewing, Visitation, Wake | 2-4시간 |
| 주요 서비스 | Funeral, Memorial, Celebration of Life | 1-2시간 |
| 사후 서비스 | Graveside, Committal, Reception | 30-60분 |
| 가상 서비스 | Live-Streamed Service | 대면 서비스와 동일 |

### 9.2 문화 및 종교 전통

| 전통 | 일반적 관행 | 일정 |
|-----|----------|-----|
| 기독교 | 빈소, 장례식, 매장 | 사망 후 3-7일 |
| 가톨릭 | 빈소, 장례미사, 매장 | 사망 후 3-7일 |
| 유대교 | 즉시 매장, 시바, 제막식 | 24시간 내, 7일, 1년 |
| 이슬람교 | 의식 세정, 매장(방부 없음) | 24시간 내 |
| 불교 | 염불, 화장, 추모 의식 | 전통에 따라 다름 |
| 힌두교 | 화장, 유골 산골 | 가능하면 24시간 내 |

### 9.3 안치 방법

| 방법 | 과정 | 환경 영향 |
|-----|-----|---------|
| 전통 매장 | 방부 처리, 관, 봉안실 | 보통~높음 |
| 화장 | 고온 연소 | 보통 |
| 자연 매장 | 방부 없음, 생분해 관 | 낮음 |
| 수장 | 물 기반 화장 | 낮음 |
| 시신 기증 | 의학/과학 연구 | 해당 없음 |

### 9.4 비용 범위 (KRW)

| 서비스 구성요소 | 하위 범위 | 중간 범위 | 상위 범위 |
|---------------|---------|---------|---------|
| 장례 서비스 | 2,000,000 | 7,000,000 | 15,000,000+ |
| 화장만 | 800,000 | 2,500,000 | 5,000,000 |
| 관 | 500,000 | 2,500,000 | 10,000,000+ |
| 묘지 | 1,000,000 | 3,500,000 | 10,000,000+ |
| 비석 | 500,000 | 2,000,000 | 8,000,000+ |
| 가상 스트리밍 | 200,000 | 500,000 | 1,500,000 |

---

## 10. 구현 예제

### 10.1 완전한 고인 기록

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "legalName": {
    "prefix": "Dr.",
    "firstName": "Margaret",
    "middleName": "Anne",
    "lastName": "Johnson",
    "suffix": "PhD",
    "preferredName": "Maggie"
  },
  "dateOfBirth": "1945-03-15",
  "dateOfDeath": "2025-12-10T08:30:00Z",
  "placeOfBirth": {
    "city": "Boston",
    "state": "Massachusetts",
    "country": "US"
  },
  "placeOfDeath": {
    "facility": "St. Mary's Hospital",
    "city": "Portland",
    "state": "Oregon",
    "country": "US"
  },
  "biography": {
    "short": "사랑하는 어머니이자 할머니, 은퇴한 문학 교수인 마가렛 존슨 박사님이 80세를 일기로 평화롭게 별세하셨습니다.",
    "full": "마가렛 '매기' 존슨 박사님은 교육과 가족을 위해 평생을 헌신하셨습니다. 포틀랜드 주립대학교에서 40년간 영문학을 가르치며 시와 고전 소설에 대한 열정으로 수천 명의 학생들에게 영감을 주셨습니다...",
    "achievements": [
      "포틀랜드 주립대학교 명예교수",
      "미국 시에 관한 세 권의 저서 저자",
      "Distinguished Teaching Award 수상 (1998)",
      "포틀랜드 시 협회 설립"
    ],
    "hobbies": [
      "독서",
      "정원 가꾸기",
      "여행",
      "지역 도서관 자원봉사"
    ]
  },
  "family": {
    "spouse": [
      {
        "name": "Robert Johnson",
        "status": "current"
      }
    ],
    "children": [
      {"name": "Sarah Martinez", "status": "surviving"},
      {"name": "Michael Johnson", "status": "surviving"},
      {"name": "Jennifer Chen", "status": "surviving"}
    ]
  },
  "privacy": {
    "level": "public",
    "showDateOfBirth": true,
    "showPlaceOfBirth": true,
    "showFullBiography": true,
    "showFamilyMembers": true,
    "allowPublicCondolences": true
  }
}
```

### 10.2 하이브리드 장례 서비스

```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "memorial",
  "religiousTradition": "non-religious",
  "startDateTime": "2025-12-18T14:00:00-08:00",
  "endDateTime": "2025-12-18T16:00:00-08:00",
  "timezone": "America/Los_Angeles",
  "venue": {
    "type": "funeral-home",
    "name": "Peaceful Rest Memorial Chapel",
    "address": {
      "street": "1234 Oak Street",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97201",
      "country": "US"
    },
    "capacity": 150,
    "accessibility": {
      "wheelchairAccessible": true,
      "parkingAvailable": true,
      "assistiveListeningDevices": true
    }
  },
  "virtualService": {
    "enabled": true,
    "streamingUrl": "https://stream.example.com/service/660e8400",
    "platform": "custom",
    "accessCode": "MAGGIE2025",
    "recordingEnabled": true,
    "chatEnabled": true,
    "maxVirtualAttendees": 500
  },
  "program": [
    {
      "order": 1,
      "type": "music",
      "title": "전주곡",
      "description": "클래식 피아노 연주",
      "duration": 10,
      "performer": "Emily Chen, pianist"
    },
    {
      "order": 2,
      "type": "eulogy",
      "title": "추도사",
      "description": "존슨 박사님의 생애와 업적에 대한 회상",
      "duration": 15,
      "performer": "Dr. James Wilson, 동료"
    },
    {
      "order": 3,
      "type": "video-tribute",
      "title": "사진으로 보는 생애",
      "description": "사진 및 비디오 몽타주",
      "duration": 10
    }
  ],
  "status": "confirmed"
}
```

### 10.3 손님 RSVP 응답

```json
{
  "id": "770e8400-e29b-41d4-a716-446655440002",
  "serviceId": "660e8400-e29b-41d4-a716-446655440001",
  "attendeeInfo": {
    "name": "김민수",
    "email": "minsoo.kim@email.com",
    "phone": "+82-10-1234-5678",
    "relationship": "former student"
  },
  "response": "attending",
  "attendanceType": "in-person",
  "numberOfGuests": 2,
  "guestNames": ["김민수", "이영희"],
  "dietaryRestrictions": ["vegetarian"],
  "submittedAt": "2025-12-14T10:30:00Z"
}
```

### 10.4 조문 메시지

```json
{
  "id": "880e8400-e29b-41d4-a716-446655440003",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "authorName": "박지연",
  "authorEmail": "jiyeon.park@email.com",
  "relationship": "former student",
  "message": "존슨 박사님은 제가 만난 가장 영감을 주는 교수님이셨습니다. 문학에 대한 열정이 전염성이 있었고, 각 학생을 진심으로 아끼셨습니다. 교실 밖에서도 배운 교훈들이 있습니다. 깊이 그리워할 것입니다. 이 어려운 시기에 존슨 가족에게 위로를 전합니다.",
  "isPublic": true,
  "moderationStatus": "approved",
  "postedAt": "2025-12-12T16:45:00Z"
}
```

### 10.5 추모 기부

```json
{
  "id": "990e8400-e29b-41d4-a716-446655440004",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "amount": 100000,
  "currency": "KRW",
  "donorInfo": {
    "name": "포틀랜드 주립대학교 동문회",
    "email": "alumni@psu.edu",
    "isAnonymous": false
  },
  "recipient": {
    "type": "charity",
    "name": "포틀랜드 공공도서관 재단",
    "taxId": "12-3456789"
  },
  "inMemoryOf": "Dr. Margaret Johnson",
  "message": "문맹 퇴치와 교육에 평생을 바친 존슨 박사님을 기리며.",
  "paymentMethod": "credit-card",
  "transactionId": "txn_1234567890",
  "status": "completed",
  "donatedAt": "2025-12-13T09:15:00Z"
}
```

---

## 결론

이 Phase 1 명세서는 디지털 장례 서비스를 위한 포괄적인 데이터 형식을 설정합니다. Schema는 다양한 문화적 전통, 서비스 유형 및 가상 참석과 디지털 추모관 같은 현대적 기능에 유연성을 제공합니다. 이러한 형식의 구현은 개인정보 보호와 문화적 민감성을 존중하면서 장례 서비스 업계 전반의 상호 운용성을 가능하게 합니다.

**다음 단계:** Phase 2는 이러한 표준화된 형식을 사용하여 장례 서비스 데이터를 생성, 검색, 업데이트 및 관리하기 위한 API interface를 정의합니다.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
