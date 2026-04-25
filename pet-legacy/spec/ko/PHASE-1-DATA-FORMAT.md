# WIA PET-LEGACY PHASE 1: 데이터 포맷 명세서

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## 목차

1. [개요](#개요)
2. [핵심 데이터 모델](#핵심-데이터-모델)
3. [추모 프로필 스키마](#추모-프로필-스키마)
4. [미디어 자산 스키마](#미디어-자산-스키마)
5. [타임라인 이벤트 스키마](#타임라인-이벤트-스키마)
6. [가족 구성원 스키마](#가족-구성원-스키마)
7. [데이터 검증 규칙](#데이터-검증-규칙)
8. [파일 형식 명세](#파일-형식-명세)
9. [메타데이터 표준](#메타데이터-표준)
10. [보안 및 프라이버시](#보안-및-프라이버시)

---

## 1. 개요

### 1.1 목적

WIA PET-LEGACY Phase 1 명세서는 사망한 반려동물의 디지털 추모 및 유산 보존을 위한 표준화된 데이터 포맷을 정의합니다. 이 표준은 추모 플랫폼, 동물병원 시스템, 묘지/화장 서비스 간의 상호 운용성을 보장하면서 사랑하는 반려동물의 존엄성과 추억을 보존합니다.

### 1.2 적용 범위

이 명세서는 다음을 포함합니다:
- 반려동물 추모 프로필 데이터 구조
- 미디어 자산 관리 및 메타데이터
- 타임라인 및 생애 역사 문서화
- 가족 구성원 접근 및 관계 모델링
- 애도 지원 리소스 통합
- 크로스 플랫폼 데이터 교환 형식

### 1.3 핵심 원칙

| 원칙 | 설명 | 구현 |
|------|------|------|
| **존엄성** | 반려동물 추억을 존중과 경외심으로 다룸 | 콘텐츠 정책 강제, 품질 표준 |
| **영속성** | 장기 데이터 보존 보장 | 안정적인 형식, 버전 관리, 마이그레이션 경로 |
| **프라이버시** | 민감한 가족 정보 보호 | 암호화, 접근 제어, 동의 관리 |
| **접근성** | 추억에 대한 보편적 접근 가능 | 다중 형식 지원, 보조 기술 호환성 |
| **상호운용성** | 원활한 데이터 교환 허용 | 표준 JSON 스키마, API 호환성 |

### 1.4 용어

| 용어 | 정의 |
|------|------|
| **Memorial Profile** | 사망한 반려동물의 생애와 유산에 대한 완전한 디지털 표현 |
| **Guardian** | 추모관을 책임지는 주 소유자 또는 보호자 |
| **Contributor** | 추억을 추가할 권한이 있는 가족 구성원 또는 친구 |
| **Timeline Event** | 관련 미디어가 있는 반려동물 생애의 중요한 순간 |
| **Memorial Asset** | 추억을 보존하는 사진, 비디오, 문서 또는 기타 미디어 |
| **Legacy Package** | 모든 추모 데이터 및 자산의 내보내기 가능한 아카이브 |

---

## 2. 핵심 데이터 모델

### 2.1 데이터 모델 계층 구조

```
Memorial Profile (루트)
├── Pet Identity (반려동물 신원)
│   ├── Basic Information (기본 정보)
│   ├── Physical Characteristics (신체 특성)
│   └── Identification Records (식별 기록)
├── Life Timeline (생애 타임라인)
│   ├── Birth/Adoption Events (출생/입양 이벤트)
│   ├── Milestone Events (이정표 이벤트)
│   ├── Medical Events (의료 이벤트)
│   └── Passing Information (별세 정보)
├── Media Library (미디어 라이브러리)
│   ├── Photos (사진)
│   ├── Videos (비디오)
│   ├── Audio Recordings (오디오 녹음)
│   └── Documents (문서)
├── Stories & Memories (이야기와 추억)
│   ├── Written Tributes (헌사)
│   ├── Favorite Moments (좋아하는 순간)
│   └── Personality Traits (성격 특성)
├── Family Network (가족 네트워크)
│   ├── Guardians (보호자)
│   ├── Contributors (기여자)
│   └── Viewers (조회자)
└── Memorial Settings (추모 설정)
    ├── Privacy Controls (프라이버시 제어)
    ├── Sharing Preferences (공유 환경설정)
    └── Display Customization (표시 사용자 지정)
```

### 2.2 엔티티 관계 다이어그램

| 엔티티 | 관계 | 카디널리티 |
|--------|------|-----------|
| Memorial Profile | has many Timeline Events | 1:N |
| Memorial Profile | has many Media Assets | 1:N |
| Memorial Profile | has many Family Members | 1:N |
| Memorial Profile | has one Guardian | 1:1 |
| Timeline Event | has many Media Assets | 1:N |
| Media Asset | belongs to many Timeline Events | N:N |
| Family Member | creates many Timeline Events | 1:N |
| Family Member | uploads many Media Assets | 1:N |

### 2.3 데이터 저장 요구사항

| 데이터 유형 | 최소 크기 | 최대 크기 | 보존 기간 | 백업 빈도 |
|------------|---------|---------|----------|---------|
| Memorial Profile | 1 KB | 100 MB | 영구 | 매일 |
| Photo Asset | 100 KB | 50 MB | 영구 | 매일 |
| Video Asset | 1 MB | 500 MB | 영구 | 매일 |
| Audio Asset | 100 KB | 100 MB | 영구 | 매일 |
| Document Asset | 10 KB | 25 MB | 영구 | 매일 |
| Timeline Event | 500 B | 10 KB | 영구 | 매일 |

---

## 3. 추모 프로필 스키마

### 3.1 완전한 Memorial Profile JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "PetMemorialProfile",
  "type": "object",
  "required": ["profileId", "petIdentity", "guardianInfo", "memorialStatus"],
  "properties": {
    "profileId": {
      "type": "string",
      "format": "uuid",
      "description": "추모 프로필의 고유 식별자"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "default": "1.0.0",
      "description": "데이터 마이그레이션을 위한 스키마 버전"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time",
      "description": "추모관 생성 시간"
    },
    "lastModified": {
      "type": "string",
      "format": "date-time",
      "description": "마지막 수정 시간"
    },
    "petIdentity": {
      "type": "object",
      "required": ["name", "species"],
      "properties": {
        "name": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100,
          "description": "반려동물 이름"
        },
        "nickname": {
          "type": "array",
          "items": {"type": "string"},
          "description": "별명 또는 애칭"
        },
        "species": {
          "type": "string",
          "enum": ["dog", "cat", "bird", "rabbit", "hamster", "guinea_pig", "reptile", "fish", "horse", "other"],
          "description": "종 분류"
        },
        "breed": {
          "type": "string",
          "maxLength": 100,
          "description": "품종 또는 종 변형"
        },
        "gender": {
          "type": "string",
          "enum": ["male", "female", "unknown"],
          "description": "생물학적 성별"
        },
        "birthDate": {
          "type": "string",
          "format": "date",
          "description": "출생일 또는 추정 출생일"
        },
        "birthDateEstimated": {
          "type": "boolean",
          "default": false,
          "description": "출생일이 추정값인지 여부"
        },
        "adoptionDate": {
          "type": "string",
          "format": "date",
          "description": "보호자가 입양한 날짜"
        },
        "passingDate": {
          "type": "string",
          "format": "date",
          "description": "별세 날짜"
        },
        "ageAtPassing": {
          "type": "object",
          "properties": {
            "years": {"type": "integer", "minimum": 0},
            "months": {"type": "integer", "minimum": 0, "maximum": 11},
            "days": {"type": "integer", "minimum": 0, "maximum": 30}
          }
        },
        "microchipId": {
          "type": "string",
          "pattern": "^[0-9]{15}$",
          "description": "15자리 마이크로칩 식별 번호"
        },
        "registrationIds": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {"type": "string"},
              "number": {"type": "string"},
              "organization": {"type": "string"}
            }
          }
        }
      }
    },
    "physicalCharacteristics": {
      "type": "object",
      "properties": {
        "furColor": {
          "type": "array",
          "items": {"type": "string"},
          "description": "주요 및 보조 털/깃털/비늘 색상"
        },
        "eyeColor": {
          "type": "string",
          "description": "눈 색깔 설명"
        },
        "weight": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["kg", "lb", "g", "oz"]}
          }
        },
        "height": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["cm", "in", "m"]}
          }
        },
        "distinguishingMarks": {
          "type": "array",
          "items": {"type": "string"},
          "description": "고유한 신체적 특징 또는 표시"
        }
      }
    },
    "personality": {
      "type": "object",
      "properties": {
        "traits": {
          "type": "array",
          "items": {"type": "string"},
          "description": "성격 특성"
        },
        "favoriteActivities": {
          "type": "array",
          "items": {"type": "string"}
        },
        "favoriteToys": {
          "type": "array",
          "items": {"type": "string"}
        },
        "favoriteFoods": {
          "type": "array",
          "items": {"type": "string"}
        },
        "quirks": {
          "type": "array",
          "items": {"type": "string"},
          "description": "독특한 행동이나 습관"
        }
      }
    },
    "guardianInfo": {
      "type": "object",
      "required": ["userId", "role"],
      "properties": {
        "userId": {
          "type": "string",
          "format": "uuid",
          "description": "주 보호자 사용자 ID"
        },
        "name": {
          "type": "string",
          "description": "보호자 이름"
        },
        "role": {
          "type": "string",
          "enum": ["owner", "guardian", "caretaker"],
          "default": "owner"
        },
        "relationship": {
          "type": "string",
          "description": "반려동물과의 관계 설명"
        },
        "contactEmail": {
          "type": "string",
          "format": "email"
        }
      }
    },
    "memorialStatus": {
      "type": "object",
      "required": ["isPublic", "status"],
      "properties": {
        "status": {
          "type": "string",
          "enum": ["draft", "active", "archived", "private"],
          "default": "draft"
        },
        "isPublic": {
          "type": "boolean",
          "default": false,
          "description": "추모관이 공개 조회 가능한지 여부"
        },
        "publishedAt": {
          "type": "string",
          "format": "date-time"
        },
        "viewCount": {
          "type": "integer",
          "minimum": 0,
          "default": 0
        },
        "tributeCount": {
          "type": "integer",
          "minimum": 0,
          "default": 0
        }
      }
    },
    "passingInformation": {
      "type": "object",
      "properties": {
        "cause": {
          "type": "string",
          "description": "별세 원인 (보호자가 공유하기로 선택한 경우)"
        },
        "location": {
          "type": "string",
          "description": "반려동물이 별세한 장소"
        },
        "wasEuthanized": {
          "type": "boolean"
        },
        "veterinarianInfo": {
          "type": "object",
          "properties": {
            "clinicName": {"type": "string"},
            "veterinarianName": {"type": "string"},
            "contactInfo": {"type": "string"}
          }
        },
        "funeralArrangements": {
          "type": "object",
          "properties": {
            "type": {
              "type": "string",
              "enum": ["burial", "cremation", "other", "none"]
            },
            "location": {"type": "string"},
            "providerName": {"type": "string"},
            "providerContact": {"type": "string"},
            "ceremonyDate": {"type": "string", "format": "date-time"},
            "plotNumber": {"type": "string"},
            "urnDescription": {"type": "string"}
          }
        }
      }
    },
    "memorialCustomization": {
      "type": "object",
      "properties": {
        "themeColor": {
          "type": "string",
          "pattern": "^#[0-9A-Fa-f]{6}$",
          "default": "#F59E0B"
        },
        "coverPhoto": {
          "type": "string",
          "format": "uri",
          "description": "커버 사진 URL"
        },
        "profilePhoto": {
          "type": "string",
          "format": "uri",
          "description": "메인 프로필 사진 URL"
        },
        "epitaph": {
          "type": "string",
          "maxLength": 500,
          "description": "추모 비문 또는 헌정 메시지"
        },
        "musicUrl": {
          "type": "string",
          "format": "uri",
          "description": "추모 페이지 배경 음악"
        },
        "displayLayout": {
          "type": "string",
          "enum": ["timeline", "gallery", "story", "mixed"],
          "default": "mixed"
        }
      }
    },
    "statistics": {
      "type": "object",
      "properties": {
        "totalPhotos": {"type": "integer", "minimum": 0},
        "totalVideos": {"type": "integer", "minimum": 0},
        "totalStories": {"type": "integer", "minimum": 0},
        "totalTimelineEvents": {"type": "integer", "minimum": 0},
        "totalContributors": {"type": "integer", "minimum": 0},
        "lastActivityDate": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

### 3.2 Memorial Profile 예시

```json
{
  "profileId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "createdAt": "2024-03-15T10:30:00Z",
  "lastModified": "2024-12-18T14:22:00Z",
  "petIdentity": {
    "name": "맥스",
    "nickname": ["맥시", "맥스 보이"],
    "species": "dog",
    "breed": "골든 리트리버",
    "gender": "male",
    "birthDate": "2012-05-20",
    "birthDateEstimated": false,
    "adoptionDate": "2012-07-15",
    "passingDate": "2024-03-10",
    "ageAtPassing": {
      "years": 11,
      "months": 9,
      "days": 20
    },
    "microchipId": "985112345678901"
  },
  "physicalCharacteristics": {
    "furColor": ["황금색", "크림색"],
    "eyeColor": "갈색",
    "weight": {
      "value": 32.5,
      "unit": "kg"
    }
  },
  "personality": {
    "traits": ["충성스러운", "온화한", "장난기 있는", "사랑스러운"],
    "favoriteActivities": ["수영", "공놀이", "하이킹"],
    "favoriteToys": ["테니스공", "로프 장난감"],
    "favoriteFoods": ["닭고기 간식", "땅콩버터"]
  },
  "memorialStatus": {
    "status": "active",
    "isPublic": true,
    "viewCount": 1247,
    "tributeCount": 89
  }
}
```

---

## 4. 미디어 자산 스키마

### 4.1 Media Asset JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "MediaAsset",
  "type": "object",
  "required": ["assetId", "memorialId", "type", "url", "uploadedBy"],
  "properties": {
    "assetId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid",
      "description": "상위 추모 프로필 참조"
    },
    "type": {
      "type": "string",
      "enum": ["photo", "video", "audio", "document"]
    },
    "url": {
      "type": "string",
      "format": "uri",
      "description": "자산에 대한 CDN URL"
    },
    "thumbnailUrl": {
      "type": "string",
      "format": "uri"
    },
    "fileName": {
      "type": "string"
    },
    "fileSize": {
      "type": "integer",
      "minimum": 0,
      "description": "파일 크기(바이트)"
    },
    "mimeType": {
      "type": "string"
    },
    "uploadedBy": {
      "type": "string",
      "format": "uuid",
      "description": "자산을 업로드한 사용자 ID"
    },
    "uploadedAt": {
      "type": "string",
      "format": "date-time"
    },
    "capturedAt": {
      "type": "string",
      "format": "date-time",
      "description": "사진/비디오 촬영 시간"
    },
    "title": {
      "type": "string",
      "maxLength": 200
    },
    "description": {
      "type": "string",
      "maxLength": 2000
    },
    "tags": {
      "type": "array",
      "items": {"type": "string"}
    },
    "location": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number"},
        "longitude": {"type": "number"},
        "placeName": {"type": "string"}
      }
    },
    "mediaMetadata": {
      "type": "object",
      "properties": {
        "width": {"type": "integer"},
        "height": {"type": "integer"},
        "duration": {"type": "number", "description": "비디오/오디오 길이(초)"},
        "bitrate": {"type": "integer"},
        "codec": {"type": "string"},
        "orientation": {"type": "integer"},
        "cameraMake": {"type": "string"},
        "cameraModel": {"type": "string"}
      }
    },
    "processingStatus": {
      "type": "string",
      "enum": ["uploading", "processing", "ready", "failed"],
      "default": "uploading"
    },
    "privacyLevel": {
      "type": "string",
      "enum": ["public", "family_only", "private"],
      "default": "family_only"
    },
    "isFeatured": {
      "type": "boolean",
      "default": false
    },
    "linkedTimelineEvents": {
      "type": "array",
      "items": {"type": "string", "format": "uuid"}
    }
  }
}
```

### 4.2 지원되는 미디어 형식

| 미디어 유형 | 지원 형식 | 최대 크기 | 권장 해상도 |
|-----------|---------|---------|-----------|
| Photo | JPEG, PNG, HEIC, WebP | 50 MB | 4096x4096 px |
| Video | MP4, MOV, AVI, WebM | 500 MB | 1920x1080 (1080p) |
| Audio | MP3, WAV, AAC, OGG | 100 MB | 320 kbps |
| Document | PDF, DOCX, TXT | 25 MB | N/A |

### 4.3 사진 자산 예시

```json
{
  "assetId": "770e8400-e29b-41d4-a716-446655440002",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "photo",
  "url": "https://cdn.example.com/assets/max-birthday-2020.jpg",
  "thumbnailUrl": "https://cdn.example.com/thumbs/max-birthday-2020-thumb.jpg",
  "fileName": "max-birthday-2020.jpg",
  "fileSize": 3457280,
  "mimeType": "image/jpeg",
  "uploadedAt": "2024-03-15T11:00:00Z",
  "capturedAt": "2020-05-20T15:30:00Z",
  "title": "맥스의 8번째 생일 축하",
  "description": "강아지 간식으로 만든 생일 케이크를 즐기는 맥스. 너무 행복해 했어요!",
  "tags": ["생일", "축하", "케이크", "행복"],
  "location": {
    "placeName": "우리 집 뒷마당"
  },
  "processingStatus": "ready",
  "privacyLevel": "public",
  "isFeatured": true
}
```

---

## 5. 타임라인 이벤트 스키마

### 5.1 Timeline Event JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "TimelineEvent",
  "type": "object",
  "required": ["eventId", "memorialId", "eventType", "date", "title"],
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid"
    },
    "eventType": {
      "type": "string",
      "enum": [
        "birth",
        "adoption",
        "medical",
        "milestone",
        "travel",
        "celebration",
        "achievement",
        "loss",
        "memorial_service",
        "other"
      ]
    },
    "date": {
      "type": "string",
      "format": "date-time"
    },
    "dateEstimated": {
      "type": "boolean",
      "default": false
    },
    "title": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200
    },
    "description": {
      "type": "string",
      "maxLength": 5000
    },
    "location": {
      "type": "object",
      "properties": {
        "placeName": {"type": "string"},
        "address": {"type": "string"},
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "attachedMedia": {
      "type": "array",
      "items": {"type": "string", "format": "uuid"},
      "description": "미디어 자산 ID 배열"
    },
    "createdBy": {
      "type": "string",
      "format": "uuid"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "visibility": {
      "type": "string",
      "enum": ["public", "family_only", "private"],
      "default": "family_only"
    },
    "medicalInfo": {
      "type": "object",
      "properties": {
        "veterinarianName": {"type": "string"},
        "clinicName": {"type": "string"},
        "diagnosis": {"type": "string"},
        "treatment": {"type": "string"},
        "medications": {
          "type": "array",
          "items": {"type": "string"}
        },
        "outcome": {"type": "string"}
      }
    },
    "celebrationInfo": {
      "type": "object",
      "properties": {
        "occasionType": {
          "type": "string",
          "enum": ["birthday", "adoption_anniversary", "holiday", "achievement", "other"]
        },
        "attendees": {
          "type": "array",
          "items": {"type": "string"}
        },
        "gifts": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    }
  }
}
```

### 5.2 이벤트 유형 분류

| 이벤트 유형 | 아이콘 | 색상 | 일반적인 용도 |
|-----------|------|------|-------------|
| Birth | 🐾 | #10B981 | 반려동물의 출생 또는 추정 출생 |
| Adoption | ❤️ | #F59E0B | 가족에 합류한 날 |
| Medical | 🏥 | #EF4444 | 동물병원 방문, 치료 |
| Milestone | ⭐ | #8B5CF6 | 첫 발걸음, 훈련 성취 |
| Travel | ✈️ | #3B82F6 | 여행, 반려동물과의 휴가 |
| Celebration | 🎉 | #F59E0B | 생일, 공휴일 |
| Achievement | 🏆 | #FBBF24 | 수상, 자격증 |
| Loss | 🕊️ | #6B7280 | 별세 날짜 |
| Memorial Service | 🌹 | #A855F7 | 장례식 또는 추도식 |

### 5.3 타임라인 이벤트 예시

```json
[
  {
    "eventId": "880e8400-e29b-41d4-a716-446655440003",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "adoption",
    "date": "2012-07-15T10:00:00Z",
    "title": "맥스, 우리 집에 온 날!",
    "description": "보호소에서 맥스를 데려온 날. 생후 8주였고 내 팔에 쏙 들어왔어요. 우리가 만난 순간부터, 맥스가 우리 가족의 일원이 될 운명이라는 걸 알았어요.",
    "location": {
      "placeName": "시립 동물 보호소",
      "city": "포틀랜드",
      "state": "OR",
      "country": "USA"
    },
    "visibility": "public"
  },
  {
    "eventId": "880e8400-e29b-41d4-a716-446655440006",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "milestone",
    "date": "2013-01-10T14:00:00Z",
    "title": "복종 훈련 통과!",
    "description": "맥스가 초급 복종 과정을 우수한 성적으로 마쳤어요. 최고의 학생이었고 새로운 기술을 보여주는 걸 좋아했어요.",
    "visibility": "public",
    "celebrationInfo": {
      "occasionType": "achievement",
      "gifts": ["수료증", "새 목줄"]
    }
  }
]
```

---

## 6. 가족 구성원 스키마

### 6.1 Family Member JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "FamilyMember",
  "type": "object",
  "required": ["memberId", "memorialId", "userId", "role", "permissions"],
  "properties": {
    "memberId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid"
    },
    "userId": {
      "type": "string",
      "format": "uuid"
    },
    "displayName": {
      "type": "string",
      "maxLength": 100
    },
    "relationshipToPet": {
      "type": "string",
      "maxLength": 200,
      "description": "반려동물과의 관계"
    },
    "role": {
      "type": "string",
      "enum": ["guardian", "contributor", "viewer"],
      "description": "접근 수준 역할"
    },
    "permissions": {
      "type": "object",
      "required": ["canView", "canComment", "canUpload", "canEdit"],
      "properties": {
        "canView": {"type": "boolean", "default": true},
        "canComment": {"type": "boolean", "default": false},
        "canUpload": {"type": "boolean", "default": false},
        "canEdit": {"type": "boolean", "default": false},
        "canDelete": {"type": "boolean", "default": false},
        "canManageMembers": {"type": "boolean", "default": false},
        "canManageSettings": {"type": "boolean", "default": false}
      }
    },
    "invitedBy": {
      "type": "string",
      "format": "uuid"
    },
    "invitedAt": {
      "type": "string",
      "format": "date-time"
    },
    "joinedAt": {
      "type": "string",
      "format": "date-time"
    },
    "status": {
      "type": "string",
      "enum": ["invited", "active", "suspended", "removed"],
      "default": "invited"
    },
    "contributionStats": {
      "type": "object",
      "properties": {
        "photosUploaded": {"type": "integer", "minimum": 0},
        "videosUploaded": {"type": "integer", "minimum": 0},
        "storiesWritten": {"type": "integer", "minimum": 0},
        "eventsCreated": {"type": "integer", "minimum": 0},
        "commentsPosted": {"type": "integer", "minimum": 0},
        "lastContribution": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

### 6.2 권한 매트릭스

| 역할 | 보기 | 댓글 | 미디어 업로드 | 이벤트 생성 | 추모관 편집 | 콘텐츠 삭제 | 구성원 관리 | 설정 관리 |
|------|-----|------|------------|-----------|-----------|-----------|-----------|---------|
| **Guardian** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| **Contributor** | ✓ | ✓ | ✓ | ✓ | 본인 것만 | 본인 것만 | ✗ | ✗ |
| **Viewer** | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |

---

## 7. 데이터 검증 규칙

### 7.1 필수 필드 검증

| 필드 | 검증 규칙 | 오류 메시지 |
|------|---------|-----------|
| Pet Name | 최소 1자, 최대 100자 | "반려동물 이름은 필수이며 1-100자여야 합니다" |
| Species | enum 목록에서 선택 | "유효한 종을 선택하세요" |
| Birth Date | 유효한 날짜, 미래 불가 | "출생일은 유효한 과거 날짜여야 합니다" |
| Passing Date | Birth Date >= | "별세 날짜는 출생일 이전일 수 없습니다" |
| Guardian ID | 유효한 UUID, 사용자 존재 | "잘못된 보호자 참조" |
| Media URL | 유효한 HTTPS URL | "미디어 URL은 유효한 보안 URL이어야 합니다" |
| File Size | 유형 제한 초과 불가 | "파일 크기가 이 유형에 허용된 최대값을 초과합니다" |

### 7.2 데이터 일관성 규칙

```javascript
// 검증 함수 예시
function validateMemorialProfile(profile) {
  const errors = [];

  // 날짜 일관성
  if (profile.petIdentity.passingDate && profile.petIdentity.birthDate) {
    const birthDate = new Date(profile.petIdentity.birthDate);
    const passingDate = new Date(profile.petIdentity.passingDate);

    if (passingDate < birthDate) {
      errors.push("별세 날짜는 출생일 이전일 수 없습니다");
    }
  }

  // 나이 계산 검증
  if (profile.petIdentity.ageAtPassing) {
    const calculated = calculateAge(
      profile.petIdentity.birthDate,
      profile.petIdentity.passingDate
    );

    if (!agesMatch(calculated, profile.petIdentity.ageAtPassing)) {
      errors.push("별세 시 나이가 날짜와 일치하지 않습니다");
    }
  }

  // 추모 상태 논리
  if (profile.memorialStatus.status === 'active' && !profile.memorialStatus.publishedAt) {
    errors.push("활성 추모관에는 게시 날짜가 있어야 합니다");
  }

  // 보호자 검증
  if (!profile.guardianInfo || !profile.guardianInfo.userId) {
    errors.push("추모관에는 지정된 보호자가 있어야 합니다");
  }

  return errors;
}
```

### 7.3 콘텐츠 조정 규칙

| 콘텐츠 유형 | 조정 요구사항 | 위반 시 조치 |
|-----------|-------------|------------|
| Profile Photos | 공격적인 콘텐츠 없음, 반려동물 포함 필수 | 검토 플래그, 승인될 때까지 숨김 |
| Descriptions | 욕설, 스팸, 홍보 콘텐츠 없음 | 자동 필터, 사용자 알림 |
| Comments | 존중, 주제 관련, 괴롭힘 없음 | 제거, 사용자 경고, 잠재적 차단 |
| External Links | HTTPS 필수, 멀웨어/피싱 없음 | 차단, 보안 스캔 |
| File Uploads | 바이러스 스캔, 형식 검증 | 업로드 거부, 사용자 알림 |

---

## 8. 파일 형식 명세

### 8.1 내보내기 패키지 형식

추모 프로필은 백업 또는 마이그레이션을 위해 완전한 패키지로 내보낼 수 있습니다:

```json
{
  "exportFormat": "WIA-PET-LEGACY-1.0",
  "exportDate": "2024-12-18T14:22:00Z",
  "exportedBy": "660e8400-e29b-41d4-a716-446655440001",
  "contents": {
    "memorialProfile": "profile.json",
    "timelineEvents": "timeline/",
    "mediaAssets": "media/",
    "familyMembers": "family.json",
    "comments": "comments.json",
    "metadata": "export-metadata.json"
  },
  "totalSize": 2457280000,
  "fileCount": {
    "photos": 324,
    "videos": 47,
    "documents": 12,
    "jsonFiles": 8
  }
}
```

### 8.2 디렉토리 구조

```
memorial-export-max-20241218/
├── export-manifest.json
├── profile.json
├── family.json
├── comments.json
├── timeline/
│   ├── events.json
│   └── events-by-year/
│       ├── 2012.json
│       ├── 2013.json
│       └── ...
├── media/
│   ├── photos/
│   │   ├── 2012/
│   │   ├── 2013/
│   │   └── ...
│   ├── videos/
│   ├── audio/
│   └── documents/
└── thumbnails/
    ├── photos/
    └── videos/
```

---

## 9. 메타데이터 표준

### 9.1 Dublin Core 메타데이터

모든 추모 프로필에는 아카이브 호환성을 위한 Dublin Core 메타데이터가 포함됩니다:

```json
{
  "dublinCore": {
    "title": "맥스 추모관 - 골든 리트리버",
    "creator": "사라 존슨",
    "subject": ["반려동물 추모", "개", "골든 리트리버", "반려동물"],
    "description": "사랑받은 골든 리트리버 맥스의 생애와 유산을 보존하는 디지털 추모관 (2012-2024)",
    "publisher": "WIA Pet Legacy Platform",
    "contributor": ["사라 존슨", "마이클 존슨", "에밀리 첸"],
    "date": "2024-03-15",
    "type": "InteractiveResource",
    "format": "application/json",
    "identifier": "wia:pet-legacy:550e8400-e29b-41d4-a716-446655440000",
    "language": "ko",
    "coverage": "2012-2024",
    "rights": "© 2024 사라 존슨. 추모관 접근 권한에 따라 공유됨."
  }
}
```

---

## 10. 보안 및 프라이버시

### 10.1 데이터 암호화 표준

| 데이터 상태 | 암호화 방법 | 키 관리 |
|-----------|-----------|--------|
| 저장 시 | AES-256 | AWS KMS / Azure Key Vault |
| 전송 중 | TLS 1.3 | 90일마다 인증서 교체 |
| 백업 | AES-256 + 압축 | 별도 키 계층 구조 |
| 내보내기 | 선택적 비밀번호 보호 (AES-256) | 사용자 제공 암호 |

### 10.2 프라이버시 제어

```json
{
  "privacySettings": {
    "memorialVisibility": {
      "type": "string",
      "enum": ["public", "unlisted", "private"],
      "default": "unlisted"
    },
    "searchEngineIndexing": {
      "type": "boolean",
      "default": false,
      "description": "검색 엔진이 추모관을 색인할 수 있도록 허용"
    },
    "socialSharing": {
      "type": "boolean",
      "default": true,
      "description": "소셜 미디어 공유 활성화"
    },
    "requireLoginToView": {
      "type": "boolean",
      "default": false
    },
    "allowPublicComments": {
      "type": "boolean",
      "default": false
    },
    "showGuardianInfo": {
      "type": "boolean",
      "default": true
    },
    "showStatistics": {
      "type": "boolean",
      "default": true
    },
    "geolocationSharing": {
      "type": "string",
      "enum": ["none", "city_only", "full"],
      "default": "city_only"
    }
  }
}
```

### 10.3 데이터 보존 정책

| 데이터 유형 | 활성 기간 | 아카이브 기간 | 삭제 정책 |
|-----------|---------|-------------|---------|
| Memorial Profiles | 무기한 | N/A | 보호자 요청 시에만 |
| Media Assets | 무기한 | N/A | 보호자 요청 시에만 |
| Comments | 무기한 | N/A | 작성자/보호자가 삭제 가능 |
| Activity Logs | 2년 | 5년 | 7년 후 자동 삭제 |
| Analytics Data | 1년 | 2년 | 3년 후 익명화 |
| Deleted Content | 30일 (복구 가능) | N/A | 30일 후 영구 삭제 |

### 10.4 GDPR 준수

```javascript
// 데이터 주체 권리 구현 예시
const dataSubjectRights = {
  rightToAccess: {
    endpoint: '/api/v1/memorials/{id}/export',
    format: 'JSON 또는 PDF',
    deliveryTime: '30일'
  },
  rightToRectification: {
    endpoint: '/api/v1/memorials/{id}',
    method: 'PATCH',
    allowedFields: ['모든 비시스템 필드']
  },
  rightToErasure: {
    endpoint: '/api/v1/memorials/{id}',
    method: 'DELETE',
    confirmation: '필수',
    graceperiod: '30일'
  },
  rightToDataPortability: {
    endpoint: '/api/v1/memorials/{id}/export',
    format: 'JSON (기계 판독 가능)',
    includesAllData: true
  },
  rightToObject: {
    endpoint: '/api/v1/memorials/{id}/privacy',
    options: ['분석 거부', '처리 제한']
  }
};
```

---

## 부록 A: 버전 이력

| Version | Date | 변경 사항 | 작성자 |
|---------|------|---------|-------|
| 1.0.0 | 2025-12-18 | 초기 명세서 작성 | WIA Standards Committee |

---

## 부록 B: 참고 문헌

- ISO 8601: 날짜 및 시간 형식
- RFC 3986: URI 일반 구문
- RFC 7946: GeoJSON 형식
- Dublin Core Metadata Initiative
- GDPR (EU) 2016/679
- JSON Schema Draft 2020-12

---

## 부록 C: 감사의 글

이 명세서는 다음의 의견을 받아 개발되었습니다:
- 반려동물 추모 서비스 제공업체
- 수의학 전문가
- 반려동물 상실 애도 상담사
- 동물 복지 단체
- 반려동물 묘지 및 화장 운영자
- 유가족 및 반려동물 보호자

---

**弘益人間 (홍익인간)** - 모든 인류에 이로움
© 2025 WIA
MIT License
