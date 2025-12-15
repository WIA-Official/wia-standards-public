# Phase 2: Security API Interface Standard
## Claude Code 작업 프롬프트

---

**Phase**: 2 of 4
**목표**: 사이버보안 SDK 및 API 인터페이스 표준화
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + TypeScript SDK + Python SDK + Rust SDK

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 보안 데이터 형식을 정의했다.

 이제 개발자가 보안 이벤트를 생성, 분석, 관리할 수 있는
 표준 API를 어떻게 설계할 것인가?

 - 위협 탐지 및 분석
 - 취약점 스캔 및 관리
 - 사고 대응 자동화
 - SIEM/EDR 연동

 모든 기능을 언어에 관계없이 일관된 인터페이스로 제공할 수 있을까?"
```

### 목표
```
Phase 1의 데이터 형식을 활용하여
보안 분석/탐지/대응 기능을 제공하는
표준 API Interface를 정의한다.

지원 언어: TypeScript, Python, Rust
```

---

## 📋 Phase 1 결과물 활용

| Phase 1 산출물 | Phase 2 활용 |
|---------------|-------------|
| Security Event Schema | 이벤트 생성/파싱 API |
| Threat Intel Schema | 위협 인텔리전스 API |
| Vulnerability Schema | 취약점 관리 API |
| Incident Schema | 사고 대응 API |
| MITRE ATT&CK Mapping | 공격 기법 분류 API |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 보안 API 조사

| 서비스 | 조사 대상 | 웹서치 키워드 |
|-------|----------|--------------|
| **VirusTotal** | 위협 분석 API | "VirusTotal API v3 documentation" |
| **Shodan** | 인터넷 스캐닝 API | "Shodan API Python SDK" |
| **MISP** | 위협 공유 플랫폼 API | "MISP API PyMISP documentation" |
| **TheHive** | 사고 대응 API | "TheHive4py API documentation" |
| **OpenCTI** | CTI 플랫폼 API | "OpenCTI API GraphQL" |
| **MITRE ATT&CK** | 공격 기법 API | "MITRE ATT&CK Python library" |

### 2단계: 보안 라이브러리 조사

| 분야 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **IOC 추출** | ioc-fanger, iocextract | "Python IOC extraction library" |
| **YARA** | yara-python | "YARA Python rule matching" |
| **Sigma** | sigmac, pySigma | "Sigma rule Python library" |
| **STIX** | stix2 Python library | "STIX2 Python library tutorial" |
| **해시 분석** | hashlib, ssdeep | "Python file hash analysis" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-2.md`에 다음을 정리:

```markdown
# Phase 2 사전 조사 결과

## 1. 보안 API 분석

### VirusTotal API
- 인증 방식: [API Key / OAuth]
- 주요 엔드포인트: [files, URLs, domains, ...]
- Rate Limiting: [조사 내용]
- SDK 구조: [조사 내용]

### Shodan API
- 인증 방식: [조사 내용]
- 주요 기능: [조사 내용]
- Python SDK: [조사 내용]

### MISP/TheHive
- REST API 구조: [조사 내용]
- 이벤트 생성: [조사 내용]

## 2. 보안 라이브러리 분석

### STIX2 Python
- 객체 생성 방식: [조사 내용]
- 번들 관리: [조사 내용]

### YARA Python
- 룰 매칭 방식: [조사 내용]
- 성능 고려: [조사 내용]

## 3. 공통 패턴 분석
- 인증 방식: [API Key, OAuth, JWT 등]
- 에러 처리: [표준 패턴]
- 페이지네이션: [표준 패턴]
- 비동기 처리: [패턴]

## 4. 결론
- API 설계 방향: [제안]
- SDK 구조: [제안]
```

---

## 🏗️ API 설계

### 1. 핵심 인터페이스

#### SecurityClient (메인 클라이언트)
```typescript
interface ISecurityClient {
  // 설정
  configure(options: SecurityOptions): void;

  // 서브 클라이언트
  readonly events: IEventManager;
  readonly threats: IThreatIntelManager;
  readonly vulnerabilities: IVulnerabilityManager;
  readonly incidents: IIncidentManager;
  readonly detection: IDetectionEngine;
  readonly analysis: IAnalysisEngine;

  // 연결
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;
}

interface SecurityOptions {
  apiKey?: string;
  baseUrl?: string;
  timeout?: number;
  retryConfig?: RetryConfig;
  logging?: LoggingConfig;
}
```

### 2. 이벤트 관리 (Event Manager)

```typescript
interface IEventManager {
  // 이벤트 생성
  create(event: SecurityEvent): Promise<SecurityEvent>;
  createBatch(events: SecurityEvent[]): Promise<BatchResult>;

  // 이벤트 조회
  get(id: string): Promise<SecurityEvent>;
  list(filter?: EventFilter): Promise<PaginatedResult<SecurityEvent>>;
  search(query: SearchQuery): Promise<SearchResult>;

  // 이벤트 업데이트
  update(id: string, updates: Partial<SecurityEvent>): Promise<SecurityEvent>;
  updateStatus(id: string, status: EventStatus): Promise<void>;

  // 이벤트 삭제
  delete(id: string): Promise<void>;
  deleteBatch(ids: string[]): Promise<BatchResult>;

  // 스트리밍
  subscribe(filter?: EventFilter): EventStream;
  unsubscribe(streamId: string): void;
}

interface EventFilter {
  types?: EventType[];
  severity?: SeverityRange;
  timeRange?: TimeRange;
  sources?: string[];
  tags?: string[];
  mitreTactics?: string[];
  mitreTechniques?: string[];
}

interface SearchQuery {
  query: string;
  fields?: string[];
  sort?: SortOptions;
  page?: PageOptions;
}
```

### 3. 위협 인텔리전스 (Threat Intel Manager)

```typescript
interface IThreatIntelManager {
  // IOC 관리
  addIndicator(indicator: ThreatIndicator): Promise<ThreatIndicator>;
  getIndicator(type: IndicatorType, value: string): Promise<ThreatIndicator | null>;
  searchIndicators(query: IndicatorQuery): Promise<ThreatIndicator[]>;
  enrichIndicator(indicator: ThreatIndicator): Promise<EnrichedIndicator>;

  // 위협 피드 관리
  addFeed(feed: ThreatFeed): Promise<ThreatFeed>;
  getFeed(id: string): Promise<ThreatFeed>;
  listFeeds(): Promise<ThreatFeed[]>;
  syncFeed(id: string): Promise<SyncResult>;

  // 위협 분석
  checkIP(ip: string): Promise<IPReputation>;
  checkDomain(domain: string): Promise<DomainReputation>;
  checkHash(hash: string): Promise<FileReputation>;
  checkURL(url: string): Promise<URLReputation>;

  // STIX 변환
  toSTIX(indicators: ThreatIndicator[]): STIXBundle;
  fromSTIX(bundle: STIXBundle): ThreatIndicator[];

  // MITRE ATT&CK
  mapToATTACK(indicator: ThreatIndicator): ATTACKMapping[];
  getTechnique(id: string): ATTACKTechnique;
  getTactic(id: string): ATTACKTactic;
}

interface ThreatIndicator {
  type: IndicatorType;
  value: string;
  confidence: number;
  firstSeen?: Date;
  lastSeen?: Date;
  tags?: string[];
  context?: Record<string, any>;
}

type IndicatorType =
  | 'ip'
  | 'domain'
  | 'url'
  | 'email'
  | 'file_hash_md5'
  | 'file_hash_sha1'
  | 'file_hash_sha256'
  | 'file_hash_ssdeep'
  | 'mutex'
  | 'registry'
  | 'user_agent';
```

### 4. 취약점 관리 (Vulnerability Manager)

```typescript
interface IVulnerabilityManager {
  // CVE 조회
  getCVE(id: string): Promise<Vulnerability>;
  searchCVE(query: CVEQuery): Promise<Vulnerability[]>;

  // 취약점 스캔
  scanHost(target: ScanTarget): Promise<ScanResult>;
  scanNetwork(targets: ScanTarget[]): Promise<ScanResult>;
  scheduleScan(config: ScanSchedule): Promise<string>;
  getScanStatus(scanId: string): Promise<ScanStatus>;

  // 자산 취약점 관리
  getAssetVulnerabilities(assetId: string): Promise<AssetVulnerability[]>;
  prioritizeVulnerabilities(
    vulnerabilities: Vulnerability[]
  ): Promise<PrioritizedVulnerability[]>;

  // CVSS 계산
  calculateCVSS(vector: string): CVSSScore;
  calculateRiskScore(vuln: Vulnerability, context: AssetContext): number;

  // 패치 관리
  getRemediations(vulnId: string): Promise<Remediation[]>;
  trackRemediation(vulnId: string, status: RemediationStatus): Promise<void>;
}

interface Vulnerability {
  id: string;  // CVE-YYYY-NNNNN
  title: string;
  description: string;
  cvss: CVSSScore;
  cwe: string[];
  affectedProducts: AffectedProduct[];
  exploitAvailable: boolean;
  patchAvailable: boolean;
  references: string[];
  published: Date;
  modified: Date;
}

interface CVSSScore {
  version: '3.0' | '3.1' | '4.0';
  score: number;
  vector: string;
  severity: 'none' | 'low' | 'medium' | 'high' | 'critical';
}
```

### 5. 사고 대응 (Incident Manager)

```typescript
interface IIncidentManager {
  // 사고 생성/관리
  createIncident(incident: IncidentCreate): Promise<Incident>;
  getIncident(id: string): Promise<Incident>;
  listIncidents(filter?: IncidentFilter): Promise<PaginatedResult<Incident>>;
  updateIncident(id: string, updates: IncidentUpdate): Promise<Incident>;
  closeIncident(id: string, resolution: Resolution): Promise<void>;

  // 타임라인 관리
  addTimelineEntry(incidentId: string, entry: TimelineEntry): Promise<void>;
  getTimeline(incidentId: string): Promise<TimelineEntry[]>;

  // IOC 연결
  linkIOC(incidentId: string, ioc: ThreatIndicator): Promise<void>;
  getLinkedIOCs(incidentId: string): Promise<ThreatIndicator[]>;

  // 영향받은 자산
  linkAsset(incidentId: string, asset: Asset): Promise<void>;
  getAffectedAssets(incidentId: string): Promise<Asset[]>;

  // 대응 액션
  executeAction(incidentId: string, action: ResponseAction): Promise<ActionResult>;
  getAvailableActions(incidentId: string): Promise<ResponseAction[]>;
  getActionHistory(incidentId: string): Promise<ActionResult[]>;

  // 플레이북
  runPlaybook(incidentId: string, playbookId: string): Promise<PlaybookResult>;
  getPlaybooks(): Promise<Playbook[]>;
}

interface Incident {
  id: string;
  title: string;
  description: string;
  category: IncidentCategory;
  severity: Severity;
  status: IncidentStatus;
  assignee?: string;
  createdAt: Date;
  updatedAt: Date;
  closedAt?: Date;
  timeline: TimelineEntry[];
  iocs: ThreatIndicator[];
  affectedAssets: Asset[];
}

type IncidentCategory =
  | 'malware'
  | 'phishing'
  | 'ransomware'
  | 'data_breach'
  | 'ddos'
  | 'unauthorized_access'
  | 'insider_threat'
  | 'apt'
  | 'other';

type ResponseAction =
  | { type: 'isolate_host'; hostId: string }
  | { type: 'block_ip'; ip: string }
  | { type: 'block_domain'; domain: string }
  | { type: 'disable_user'; userId: string }
  | { type: 'quarantine_file'; fileHash: string }
  | { type: 'collect_forensics'; hostId: string }
  | { type: 'custom'; command: string };
```

### 6. 탐지 엔진 (Detection Engine)

```typescript
interface IDetectionEngine {
  // YARA 룰
  loadYaraRules(rules: string | YaraRule[]): Promise<void>;
  matchYara(data: Buffer | string, ruleset?: string): Promise<YaraMatch[]>;

  // Sigma 룰
  loadSigmaRules(rules: SigmaRule[]): Promise<void>;
  matchSigma(event: SecurityEvent): Promise<SigmaMatch[]>;
  compileSigma(rule: SigmaRule, target: SigmaTarget): string;

  // 커스텀 탐지 룰
  createRule(rule: DetectionRule): Promise<DetectionRule>;
  updateRule(id: string, rule: Partial<DetectionRule>): Promise<DetectionRule>;
  deleteRule(id: string): Promise<void>;
  listRules(): Promise<DetectionRule[]>;
  testRule(rule: DetectionRule, events: SecurityEvent[]): Promise<RuleTestResult>;

  // 실시간 탐지
  startDetection(config: DetectionConfig): Promise<void>;
  stopDetection(): Promise<void>;
  onDetection(handler: (alert: Alert) => void): void;

  // 머신러닝 기반 탐지
  trainModel(data: TrainingData): Promise<MLModel>;
  detectAnomaly(event: SecurityEvent): Promise<AnomalyScore>;
  classifyThreat(event: SecurityEvent): Promise<ThreatClassification>;
}

interface DetectionRule {
  id: string;
  name: string;
  description: string;
  severity: Severity;
  mitre?: {
    tactic: string;
    technique: string;
  };
  condition: RuleCondition;
  enabled: boolean;
}

interface YaraMatch {
  rule: string;
  namespace: string;
  tags: string[];
  meta: Record<string, string>;
  strings: MatchedString[];
}

interface SigmaRule {
  title: string;
  status: 'experimental' | 'test' | 'stable';
  description: string;
  logsource: LogSource;
  detection: SigmaDetection;
  level: 'informational' | 'low' | 'medium' | 'high' | 'critical';
}
```

### 7. 분석 엔진 (Analysis Engine)

```typescript
interface IAnalysisEngine {
  // 파일 분석
  analyzeFile(file: Buffer | string, options?: FileAnalysisOptions): Promise<FileAnalysis>;
  analyzePE(file: Buffer): Promise<PEAnalysis>;
  analyzeELF(file: Buffer): Promise<ELFAnalysis>;
  analyzePDF(file: Buffer): Promise<PDFAnalysis>;
  analyzeOffice(file: Buffer): Promise<OfficeAnalysis>;

  // 네트워크 분석
  analyzePCAP(pcap: Buffer): Promise<PcapAnalysis>;
  analyzeNetflow(netflow: NetflowData[]): Promise<NetworkAnalysis>;
  detectC2(traffic: NetworkTraffic[]): Promise<C2Detection>;

  // 로그 분석
  parseLogs(logs: string[], format: LogFormat): Promise<ParsedLog[]>;
  correlateEvents(events: SecurityEvent[]): Promise<CorrelatedEvents>;
  detectPatterns(events: SecurityEvent[]): Promise<Pattern[]>;

  // 메모리 분석
  analyzeMemoryDump(dump: Buffer): Promise<MemoryAnalysis>;
  extractArtifacts(dump: Buffer): Promise<MemoryArtifact[]>;

  // 포렌식
  collectForensics(target: ForensicTarget): Promise<ForensicBundle>;
  analyzeTimeline(events: TimelineEvent[]): Promise<TimelineAnalysis>;
}

interface FileAnalysis {
  hash: {
    md5: string;
    sha1: string;
    sha256: string;
    ssdeep?: string;
  };
  type: FileType;
  size: number;
  entropy: number;
  strings: ExtractedString[];
  yaraMatches: YaraMatch[];
  iocs: ThreatIndicator[];
  verdict: 'clean' | 'suspicious' | 'malicious';
  confidence: number;
}

interface PEAnalysis extends FileAnalysis {
  imports: ImportedFunction[];
  exports: ExportedFunction[];
  sections: PESection[];
  resources: PEResource[];
  signature?: DigitalSignature;
  packer?: string;
}
```

---

## 📁 산출물 목록

Phase 2 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-2.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-2-API-INTERFACE.md

내용:
1. 개요 (Overview)
2. 아키텍처 (Architecture)
3. 인증 및 권한 (Authentication & Authorization)
4. 핵심 인터페이스 (Core Interfaces)
   - SecurityClient
   - EventManager
   - ThreatIntelManager
   - VulnerabilityManager
   - IncidentManager
   - DetectionEngine
   - AnalysisEngine
5. 데이터 타입 (Data Types)
6. 에러 처리 (Error Handling)
7. 비동기 처리 (Async Patterns)
8. 이벤트 및 스트리밍 (Events & Streaming)
9. 예제 (Examples)
10. 참고문헌 (References)
```

### 3. TypeScript SDK
```
/api/typescript/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts
│   ├── client.ts                # SecurityClient
│   ├── types/
│   │   ├── index.ts
│   │   ├── events.ts
│   │   ├── threats.ts
│   │   ├── vulnerabilities.ts
│   │   └── incidents.ts
│   ├── managers/
│   │   ├── EventManager.ts
│   │   ├── ThreatIntelManager.ts
│   │   ├── VulnerabilityManager.ts
│   │   └── IncidentManager.ts
│   ├── engines/
│   │   ├── DetectionEngine.ts
│   │   └── AnalysisEngine.ts
│   ├── utils/
│   │   ├── ioc-extractor.ts
│   │   ├── hash.ts
│   │   └── validators.ts
│   └── errors/
│       └── SecurityError.ts
└── tests/
    └── *.test.ts
```

### 4. Python SDK
```
/api/python/
├── pyproject.toml
├── wia_security/
│   ├── __init__.py
│   ├── client.py                # SecurityClient
│   ├── types/
│   │   ├── __init__.py
│   │   ├── events.py
│   │   ├── threats.py
│   │   ├── vulnerabilities.py
│   │   └── incidents.py
│   ├── managers/
│   │   ├── __init__.py
│   │   ├── event_manager.py
│   │   ├── threat_intel_manager.py
│   │   ├── vulnerability_manager.py
│   │   └── incident_manager.py
│   ├── engines/
│   │   ├── __init__.py
│   │   ├── detection_engine.py
│   │   └── analysis_engine.py
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── ioc_extractor.py
│   │   └── validators.py
│   └── errors.py
└── tests/
    └── test_*.py
```

### 5. Rust SDK
```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── client.rs
│   ├── types/
│   │   ├── mod.rs
│   │   ├── events.rs
│   │   ├── threats.rs
│   │   ├── vulnerabilities.rs
│   │   └── incidents.rs
│   ├── managers/
│   │   ├── mod.rs
│   │   ├── event_manager.rs
│   │   ├── threat_intel_manager.rs
│   │   ├── vulnerability_manager.rs
│   │   └── incident_manager.rs
│   ├── engines/
│   │   ├── mod.rs
│   │   ├── detection_engine.rs
│   │   └── analysis_engine.rs
│   ├── utils/
│   │   ├── mod.rs
│   │   └── ioc_extractor.rs
│   └── error.rs
└── tests/
    └── *.rs
```

---

## ✅ 완료 체크리스트

Phase 2 완료 전 확인:

```
□ 웹서치로 보안 API(VirusTotal, Shodan 등) 조사 완료
□ 웹서치로 보안 라이브러리(YARA, Sigma, STIX) 조사 완료
□ /spec/RESEARCH-PHASE-2.md 작성 완료
□ /spec/PHASE-2-API-INTERFACE.md 작성 완료
□ TypeScript SDK 구현 완료
□ Python SDK 구현 완료
□ Rust SDK 구현 완료
□ IOC 추출 기능 구현 (IP, Domain, Hash, Email 등)
□ MITRE ATT&CK 매핑 기능 구현
□ 단위 테스트 작성 완료
□ 모든 테스트 통과
□ README 업데이트 (Phase 2 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 보안 API/라이브러리 조사
   ↓
2. /spec/RESEARCH-PHASE-2.md 작성
   ↓
3. API 인터페이스 설계
   ↓
4. /spec/PHASE-2-API-INTERFACE.md 작성
   ↓
5. TypeScript SDK 구현
   ↓
6. TypeScript 테스트 작성 및 통과
   ↓
7. Python SDK 구현
   ↓
8. Python 테스트 작성 및 통과
   ↓
9. Rust SDK 구현
   ↓
10. Rust 테스트 작성 및 통과
   ↓
11. 완료 체크리스트 확인
   ↓
12. Phase 3 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ Phase 1 데이터 형식과 완벽히 호환
✅ 비동기 처리 (async/await) 필수
✅ 타입 안전성 보장 (TypeScript, Rust)
✅ 포괄적인 에러 처리
✅ STIX 2.1 변환 기능 포함
✅ IOC 추출 기능 포함
✅ 단위 테스트 커버리지 80% 이상
```

### DON'T (하지 말 것)

```
❌ 동기 블로킹 API 설계
❌ 타입 미정의 (any 남용)
❌ 에러 무시 처리
❌ Phase 1 형식과 불일치
❌ 보안 취약점 있는 코드
❌ 하드코딩된 자격 증명
```

---

## 🔐 보안 코딩 가이드

```
보안 SDK 개발 시 주의사항:

1. API Key 관리
   - 환경변수 또는 설정 파일에서 로드
   - 절대 코드에 하드코딩 금지
   - 로그에 API Key 노출 금지

2. 입력 검증
   - 모든 외부 입력 검증
   - SQL Injection, XSS 방지
   - 경로 순회 공격 방지

3. 암호화
   - 민감 데이터 전송 시 TLS 사용
   - 저장 시 암호화 권장

4. 로깅
   - 민감 정보 로그 제외
   - 감사 로그 지원

5. 의존성
   - 알려진 취약점 있는 라이브러리 사용 금지
   - 정기적 의존성 업데이트
```

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **웹서치로 VirusTotal API 조사**

```
검색 키워드: "VirusTotal API v3 documentation"
```

화이팅! 🤟🔒

---

<div align="center">

**Phase 2 of 4**

Security API Interface Standard

🎯 목표: 보안 분석/탐지/대응 API 표준화

</div>
