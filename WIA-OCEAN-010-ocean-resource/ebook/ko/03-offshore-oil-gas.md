# 제3장: 해양 석유 및 가스

## 해저에서 나오는 에너지

해양 석유 및 가스 생산은 **전 세계 석유 공급의 30%**를 제공하며, 일일 **2,700만 배럴의 석유**와 연간 **수조 입방미터의 천연가스**를 생산합니다. 이 **3조 달러 산업**은 수십억 명의 운송, 난방, 제조 및 발전에 동력을 제공합니다.

그러나 해양 추출은 지구상에서 가장 도전적인 환경에서 작동하며, 재앙적인 기름 유출, 메탄 누출 및 생태계 파괴의 위험이 있습니다. 에너지 수요와 환경 보호의 균형을 맞추려면 전례 없는 엔지니어링, 모니터링 및 거버넌스가 필요합니다.

### 한국의 해양 석유·가스 현황

한국은 석유·가스의 99%를 수입에 의존하지만, 해양 탐사를 통해 에너지 자급률을 높이려 노력하고 있습니다:

```typescript
interface KoreaOffshorePetroleum {
  domesticProduction: {
    동해가스전: {
      위치: "울산 동남쪽 58km 해상",
      수심: 80,                       // 미터
      생산개시: 2004,
      최대생산: 350000000,            // 입방미터/년 (천연가스)
      현상태: "생산 감소 중",
      매장량: "초기 매장량의 90% 생산 완료"
    },

    서해가스전: {
      위치: "충남 태안반도 서쪽",
      현황: "탐사 및 평가 단계",
      예상매장량: "미확정",
      개발전망: "경제성 평가 중"
    }
  };

  exploration: {
    탐사해역: {
      동해: {
        유망구조: 47,
        탐사활동: "2D/3D 탄성파 탐사",
        잠재력: "심부 가스전 가능성"
      },
      서해: {
        탐사: "중국과 공동 탐사",
        과제: "중첩 EEZ 협상"
      }
    },

    국제협력: {
      해외자원개발: {
        참여국: ["UAE", "미국", "호주", "베트남"],
        투자: 5000000000,             // USD
        목표: "에너지 자주개발률 제고"
      }
    }
  };

  infrastructure: {
    천연가스인수기지: {
      인천: { capacity: 2600000 },   // 톤/년
      평택: { capacity: 2900000 },
      통영: { capacity: 3300000 },
      삼척: { capacity: 2800000 },
      보령: { capacity: 3000000 }
    },

    가스공급망: {
      총길이: 5000,                   // 킬로미터
      공급지역: "전국",
      비축일수: 60                    // 일
    }
  };

  transition: {
    탄소중립목표: 2050,
    천연가스역할: "과도기 브릿지 연료",
    해상풍력전환: "플랫폼 재활용 검토",
    수소경제: "천연가스 개질 수소 생산"
  };
}
```

### 해양 플랫폼 기술

```typescript
interface OffshorePlatform {
  platformId: string;
  type: PlatformType;
  location: GeographicCoordinate;
  waterDepth: number;                 // 미터
  reservoirDepth: number;             // 해저 아래 미터
  production: ProductionData;
  safety: SafetySystem;
  environmental: EnvironmentalMonitoring;
}

enum PlatformType {
  FIXED = "고정식플랫폼",              // <500m 수심
  COMPLIANT_TOWER = "유연탑식",      // 500-1000m
  SPAR = "스파플랫폼",                // 600-3000m
  TLP = "긴장계류식",                 // 1000-2000m
  SEMI = "반잠수식",                  // 500-3000m
  FPSO = "부유식생산저장하역설비",    // 모든 수심
  SUBSEA = "해저생산시스템"           // >3000m
}
```

### 생산 모니터링

```typescript
interface ProductionMonitoring {
  platform: string;
  timestamp: Date;

  wells: {
    wellId: string;
    status: "생산" | "폐쇄" | "주입" | "점검";

    downhole: {
      pressure: number;               // PSI
      temperature: number;            // 섭씨
      oilRate: number;                // 배럴/일
      gasRate: number;                // 백만 입방피트/일
      waterCut: number;               // 퍼센트
    };

    integrity: {
      casingPressure: number;
      leakDetection: boolean;
      corrosionRate: number;          // 밀리미터/년
    };
  }[];

  processing: {
    oilProduction: number;            // 배럴/일
    gasProduction: number;            // 백만 입방피트/일
    waterDisposal: number;            // 배럴/일
    flaringRate: number;              // 백만 입방피트/일
  };

  환경모니터링: {
    배출수질: {
      oilContent: number;             // mg/L (법적 기준 <30)
      compliance: boolean;
    };

    대기배출: {
      co2: number;                    // 톤/일
      methane: number;
      nox: number;
      voc: number;
    };

    해양생물: {
      소음모니터링: boolean;
      고래관찰: ObservationRecord[];
    };
  };
}
```

### 환경 영향과 대응

#### 기름 유출 대응

```typescript
interface OilSpillResponse {
  준비태세: {
    최악시나리오: number;            // 배럴/일
    대응시간: number;                // 시간

    1차대응: {
      오일붐: number,                 // 미터
      회수기: number,
      저장용량: number,               // 배럴
      분산제: number                  // 갤런
    };

    2차대응: {
      지역협력: "인근 시설과 공동 대응",
      선박: number,
      항공기: number,
      인력: number
    };

    3차대응: {
      국제지원: "ITOPF, OSRL",
      구호정: "해저 누출 차단 장비",
      구제시추: "소요 일수"
    };
  };

  예방조치: {
    폭발방지기: {
      type: "해저 BOP",
      rams: "블라인드 전단 램",
      testingFrequency: 14,           // 일
      backup: "이중 안전 시스템"
    };

    환경민감지역: {
      최근거리해안선: number,         // 킬로미터
      해양보호구역: string[],
      어장: string[],
      관광지: string[]
    };

    훈련: {
      빈도: 4,                        // 회/년
      시나리오: "다양한 유출 시나리오",
      관계기관협력: "해양경찰, 지자체"
    };
  };

  한국사례: {
    허베이스피리트호: {
      연도: 2007,
      위치: "태안 앞바다",
      유출량: 12547,                  // 톤 (국내 최대)
      피해: {
        해안선: 375,                  // 킬로미터
        어업손실: 500000000000,       // 원
        생태계피해: "김 양식장, 갯벌"
      },
      정화: {
        기간: 7,                      // 년
        비용: 500000000000,
        자원봉사: 1200000             // 명
      },
      교훈: [
        "유조선 항로 규제 강화",
        "해상교통관제 확대",
        "방제 장비 확충",
        "보상 체계 개선"
      ]
    };
  };
}
```

### 안전 시스템

```typescript
interface SafetySystem {
  blowoutPreventer: {
    설치위치: "해저" | "표면",
    rams: {
      blindShearRam: number,          // 파이프 절단 및 밀봉
      pipeRam: number,
      blindRam: number
    },
    테스트: 14,                       // 일마다
    백업: "이중 유압 시스템"
  };

  화재가스: {
    감지기: {
      smoke: number,
      heat: number,
      flame: number,
      hydrocarbon: number,
      h2s: number
    },
    진압: {
      waterDeluge: boolean,
      foamSystem: boolean,
      coverage: 100                   // 퍼센트
    };
  };

  대피: {
    구명정: {
      수: number,
      수용인원: number,
      자유낙하식: boolean,
      대피시간: number                // 분
    },
    헬리패드: boolean,
    탈출캡슐: boolean
  };

  인력안전: {
    h2s모니터링: boolean,
    호흡기: boolean,
    집합: {
      1차집결지: string,
      2차집결지: string,
      인원확인시스템: string
    };
  };
}
```

### 해체 및 복원

플랫폼은 결국 제거되어야 합니다:

```typescript
interface Decommissioning {
  platform: string;
  설치연도: Date;
  예상제거연도: Date;

  plugAndAbandonment: {
    유정수: number,
    시멘트플러그: number,            // 유정당
    유정당비용: number,               // 백만 USD
    유정당기간: number                // 일
  };

  플랫폼제거: {
    방법: "완전제거" | "부분제거" | "인공어초전환",
    중량: number,                     // 톤
    절단깊이: number,                 // 해저면 아래 미터
    비용: number,                     // 백만 USD
    기간: number                      // 개월
  };

  환경복원: {
    현장평가: boolean,
    모니터링기간: number,             // 년
    생태계회복목표: string
  };

  인공어초프로그램: {
    적격여부: boolean,
    해양생물서식: "산호, 어류",
    비용절감: number,                 // 퍼센트
    환경편익: "생물다양성 증가"
  };
}
```

### 저탄소 전환

석유·가스 산업은 탈탄소화 압력에 직면:

```typescript
interface LowCarbonTransition {
  현황: {
    한국천연가스발전: {
      비중: 30,                       // 퍼센트 (발전량)
      co2배출: 100000000,             // 톤/년
      석탄대비: "절반 수준 배출"
    };
  };

  감축전략: {
    메탄누출감소: {
      감지기술: "광학가스이미징, 위성",
      누출감지: "조기 발견 및 보수",
      목표: "메탄 배출 90% 감소"
    };

    플레어링감소: {
      현재플레어링: number,           // 백만 입방피트/일
      목표: "제로 일상 플레어링",
      방법: [
        "가스 재주입",
        "발전 활용",
        "액화"
      ]
    };

    탄소포집저장: {
      CCS잠재력: {
        동해: "대용량 저장층",
        저장용량: 100000000,          // 톤 CO2
        활용: "발전소, 산업시설 CO2 저장"
      },

      pilot프로젝트: {
        위치: "동해가스전",
        규모: 400000,                 // 톤 CO2/년
        기술: "고갈 가스전 활용",
        시작: 2025
      };
    };

    해상수소: {
      천연가스개질: "블루수소 생산",
      CCS결합: "저탄소 수소",
      해상풍력연계: "그린수소 생산",
      전망: "에너지 전환 핵심"
    };
  };

  플랫폼재활용: {
    해상풍력전환: {
      concept: "석유 플랫폼을 풍력 기초로 전환",
      advantage: "기존 인프라 활용",
      북해사례: "수십 개 플랫폼 전환 계획",
      한국검토: "동해가스전 종료 후 활용"
    };
  };
}
```

### 철학: 弘益人間과 해양 에너지

弘益人間의 원칙은 책임 있는 해양 개발을 요구합니다:

**모두를 위한 에너지:** 해양자원이 현대 생활에 동력을 제공하지만:
- 가능한 한 빨리 **재생에너지로 전환**
- 전환 기간 동안 **환경 피해 최소화**
- 연안 지역사회와 **혜택 공유**
- 수산업과 관광을 지원하는 **생태계 보호**

**미래 세대에 대한 책임:**
- 강력한 안전 시스템을 통한 **무사고**
- 해양 환경 복원을 위한 **완전한 해체**
- 기후변화 대응을 위한 **탄소 감축**
- 글로벌 관행 개선을 위한 **지식 공유**

---

**다음 장:** 해양 재생에너지 - 풍력, 파력, 조력 발전 - 해양 에너지의 지속가능한 미래를 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
