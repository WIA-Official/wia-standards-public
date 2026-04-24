# Phase 4 사전 조사 결과: WIA 생태계 통합

## 1. 미션 시각화 기술

### NASA OpenMCT

NASA Ames Research Center에서 개발한 오픈소스 미션 컨트롤 프레임워크.

**현재 사용 미션:**
- ASTERIA
- Cold Atom Laboratory (ISS)
- Mars Cube One (InSight 지원)
- Mars 2020, Jason 3 (VISTA를 통해)

**주요 특징:**
| 기능 | 설명 |
|-----|------|
| 웹 기반 | 데스크톱/모바일 지원 |
| 플러그인 | API 기반 확장 |
| 실시간 | 스트리밍 및 히스토리 데이터 |
| 다양한 시각화 | 이미지, 타임라인, 프로시저 |

**장점:**
- NASA 검증된 프레임워크
- 오픈소스 (Apache 2.0)
- 모듈화 및 확장성
- 다중 미션 지원

**단점:**
- 학습 곡선
- Node.js 환경 필요
- 커스터마이징 복잡

**GitHub**: https://github.com/nasa/openmct

### CesiumJS

WebGL 기반 3D 글로브 및 지도 라이브러리.

**주요 특징:**
| 기능 | 설명 |
|-----|------|
| WGS84 글로브 | 정밀한 지구 모델 |
| 시간 동적 시각화 | 위성 궤도, 날씨, 드론 |
| 좌표계 | ICRF, ECI, ECEF 지원 |
| 커스텀 셰이더 | GLSL/WGSL 지원 |

**장점:**
- 플러그인 불필요 (순수 WebGL)
- 고위도 왜곡 없음
- Three.js 통합 가능
- AGI/STK 호환

**단점:**
- 파일 크기 큼
- 복잡한 씬에서 성능 이슈

**GitHub**: https://github.com/CesiumGS/cesium

### GMAT (General Mission Analysis Tool)

NASA의 오픈소스 미션 설계/최적화/항법 시스템.

**지원 미션:**
- LCROSS
- Lunar Reconnaissance Orbiter
- OSIRIS-REx
- TESS
- Artemis/Lunar Gateway

**인터페이스:**
| 타입 | 설명 |
|-----|------|
| GUI | 그래픽 인터페이스 |
| Script | 텍스트 스크립트 |
| MATLAB | MATLAB 연동 |

**장점:**
- 유일한 다중 미션 오픈소스 시스템
- LEO ~ 심우주 지원
- STK ephemeris 출력
- 플러그인 지원

**최신 버전**: R2025a

---

## 2. 데이터 내보내기/가져오기 기술

### CCSDS 데이터 형식

**주요 형식:**

| 형식 | 용도 | 설명 |
|-----|------|------|
| **OPM** | Orbit Parameter Message | 단일 궤도 파라미터 |
| **OEM** | Orbit Ephemeris Message | 궤도 시계열 |
| **OMM** | Orbit Mean-Elements Message | 평균 요소 |
| **XTCE** | Telemetric & Command Exchange | 텔레메트리/명령 정의 |
| **CFDP** | File Delivery Protocol | 파일 전송 |

### CCSDSPy

CCSDS 바이너리 데이터용 Python 라이브러리.

**사용처:**
- NASA
- NOAA
- SWRI

**기능:**
- 바이너리 패킷 파싱
- 텔레메트리 디코딩
- I/O 유틸리티

**GitHub**: https://github.com/CCSDSPy

### NASA cFS (Core Flight System)

범용 비행 소프트웨어 아키텍처 프레임워크.

**적용 범위:**
- 플래그십 우주선
- 유인 우주선
- CubeSat
- Raspberry Pi

**라이선스**: Apache 2.0

**관련 도구:**
| 도구 | 설명 |
|-----|------|
| CCDD | Command and Data Dictionary Tool |
| CryptoLib | SDLS-EP 보안 프로토콜 |
| CFDP | 파일 전송 프로토콜 |

**GitHub**: https://github.com/nasa/cFS

### NASA CCDD

CFS 명령 및 데이터 딕셔너리 도구.

**지원 형식:**
| 형식 | 가져오기 | 내보내기 |
|-----|:-------:|:-------:|
| CSV | ✓ | ✓ |
| JSON | ✓ | ✓ |
| EDS | ✓ | ✓ |
| XTCE | ✓ | ✓ |

**저장소**: PostgreSQL 데이터베이스

---

## 3. 실시간 모니터링 기술

### Lightstreamer (NASA ISS)

NASA Mission Control Center에서 ISS 텔레메트리 스트리밍에 사용.

**아키텍처:**
```
ISS → Radio → MCC Broker → Public Broker → Web/Mobile
```

**특징:**
- ISSLive! 애플리케이션
- ETHOS 콘솔 실시간 환경 데이터
- 개발자/공개 API

### 텔레메트리 모니터링 시스템

**Dewesoft NASA 시스템:**
- 수십만 파라미터 실시간 모니터링
- Firing Room 콘솔 지원
- 발사체/우주선/페이로드 독립 검증

**기능:**
| 기능 | 설명 |
|-----|------|
| 실시간 플로팅 | 라이브 데이터 시각화 |
| 검색 | 과거 데이터 조회 |
| 이상 감지 | 자동 알림 |
| 상태 모니터링 | 우주선 건강 상태 |

---

## 4. 웹 기반 시각화

### Three.js + WebGL

3D 그래픽 렌더링 라이브러리.

**WebGL-Orbiter 예시:**
- 실시간 궤도 시뮬레이터
- Euler/Runge-Kutta 수치적분
- 태양계 실제 스케일
- Kerbal Space Program 스타일 조작

**URL**: https://msakuta.github.io/WebGL-Orbiter/index.html

### Cesium + Three.js 통합

**통합 방식:**
1. 별도 Canvas 레이어
2. 동일 좌표계 컨트롤러 결합
3. z-up (Three.js) ↔ y-up (Cesium) 변환

**Asterank:**
- 소행성 궤도 3D 시각화
- WebGL 기반
- 실시간 시뮬레이션

### ZERUA

웹 기반 우주 궤도 시뮬레이터 (HOMA 후속).

**특징:**
- 고정밀 위성 궤도 모델링
- 3D 인터랙티브 환경
- 일일 업데이트 TLE 데이터베이스
- 실제 위성 데이터 활용

**URL**: https://www.zerua.space/

---

## 5. WIA Space Standard 적용 방안

### 출력 계층 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                   WIA Space Standard                         │
│              (Phase 1-3 Data + Protocol)                    │
└─────────────────────────────────────────────────────────────┘
                              │
                      [OutputManager]
                              │
         ┌────────────┬───────┴───────┬────────────┐
         ▼            ▼               ▼            ▼
    ┌─────────┐  ┌─────────┐   ┌─────────┐  ┌─────────┐
    │Visualizer│  │Exporter │   │Dashboard │  │ Alert   │
    │ Adapter │  │ Adapter │   │ Adapter  │  │ Adapter │
    └────┬────┘  └────┬────┘   └────┬────┘  └────┬────┘
         │            │              │            │
         ▼            ▼              ▼            ▼
    ┌─────────┐  ┌─────────┐   ┌─────────┐  ┌─────────┐
    │CesiumJS │  │ CCSDS   │   │ OpenMCT │  │ WebHook │
    │Three.js │  │ JSON    │   │ VISTA   │  │ Email   │
    │ GMAT    │  │ NASA    │   │ Custom  │  │ Slack   │
    └─────────┘  └─────────┘   └─────────┘  └─────────┘
```

### 출력 어댑터 인터페이스

```rust
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// 출력 타입
    fn output_type(&self) -> OutputType;

    /// 초기화
    async fn initialize(&mut self, options: OutputOptions) -> Result<()>;

    /// 데이터 출력
    async fn output(&self, data: &OutputData) -> Result<()>;

    /// 상태 확인
    fn is_available(&self) -> bool;

    /// 정리
    async fn dispose(&mut self) -> Result<()>;
}
```

### 지원 출력 타입

| 타입 | 어댑터 | 용도 |
|-----|-------|------|
| `visualization` | VisualizerAdapter | 3D 시각화 |
| `export` | ExporterAdapter | 데이터 내보내기 |
| `dashboard` | DashboardAdapter | 대시보드 연동 |
| `alert` | AlertAdapter | 알림 전송 |

### 데이터 내보내기 형식

| 형식 | 확장자 | 용도 |
|-----|-------|------|
| WIA JSON | `.wia.json` | WIA 표준 형식 |
| CCSDS OEM | `.oem` | 궤도 시계열 |
| CCSDS OPM | `.opm` | 궤도 파라미터 |
| NASA SPICE | `.bsp` | 행성력표 |
| STK | `.e` | AGI STK 호환 |
| GMAT Script | `.script` | GMAT 연동 |
| CSV | `.csv` | 범용 테이블 |

---

## 6. 결론

### 권장 시각화 방식

1. **웹 기반**: CesiumJS + Three.js 조합
2. **데스크톱**: GMAT 연동
3. **미션 컨트롤**: OpenMCT 프레임워크

### 데이터 내보내기 설계

1. **표준 형식**: CCSDS OEM/OPM 우선 지원
2. **WIA 형식**: Phase 1 JSON 스키마 유지
3. **확장성**: 플러그인 기반 내보내기 추가

### 대시보드 연동 설계

1. **OpenMCT 플러그인**: 텔레메트리 피드 연동
2. **WebSocket**: Phase 3 프로토콜 활용
3. **REST API**: 히스토리 데이터 조회

### 알림 시스템 설계

1. **WebHook**: 범용 알림
2. **이메일**: 중요 이벤트
3. **Slack/Discord**: 팀 커뮤니케이션

---

## 참조 문헌

### NASA 도구
- [OpenMCT](https://github.com/nasa/openmct) - 미션 컨트롤 프레임워크
- [GMAT](https://sourceforge.net/projects/gmat/) - 미션 분석 도구
- [NASA cFS](https://github.com/nasa/cFS) - 비행 소프트웨어 시스템

### 시각화 라이브러리
- [CesiumJS](https://github.com/CesiumGS/cesium) - 3D 글로브 및 지도
- [Three.js](https://threejs.org/) - WebGL 3D 그래픽

### CCSDS 표준
- [CCSDS.org](https://ccsds.org/) - 우주 데이터 시스템 표준
- [CCSDSPy](https://github.com/CCSDSPy) - Python 라이브러리

### 실시간 데이터
- [Lightstreamer](https://lightstreamer.com/) - 실시간 스트리밍
- [ZERUA](https://www.zerua.space/) - 웹 궤도 시뮬레이터

---

*작성일: 2025-12-14*
*WIA Space Standard Phase 4 Research*
