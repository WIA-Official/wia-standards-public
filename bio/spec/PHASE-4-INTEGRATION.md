# WIA Biotechnology Standard - Phase 4: Ecosystem Integration

**버전**: 1.0.0
**상태**: Draft
**최종 수정**: 2025-12-14

---

## 목차

1. [개요](#1-개요)
2. [통합 아키텍처](#2-통합-아키텍처)
3. [어댑터 인터페이스](#3-어댑터-인터페이스)
4. [GA4GH 통합](#4-ga4gh-통합)
5. [FHIR Genomics 통합](#5-fhir-genomics-통합)
6. [LIMS 통합](#6-lims-통합)
7. [ELN 통합](#7-eln-통합)
8. [데이터베이스 통합](#8-데이터베이스-통합)
9. [에러 처리](#9-에러-처리)
10. [보안](#10-보안)
11. [예제](#11-예제)

---

## 1. 개요

### 1.1 목적

Phase 4는 WIA Biotechnology Standard를 외부 생태계와 통합하여:
- 기존 연구실 인프라와 상호운용성 확보
- 국제 표준(GA4GH, FHIR) 준수
- 데이터 교환 및 공유 지원

### 1.2 Phase 1-3 연계

```
Phase 1: Data Format     → 데이터 정의
Phase 2: API Interface   → 내부 API
Phase 3: Protocol        → 통신 방식
Phase 4: Integration     → 외부 시스템 연동
```

### 1.3 지원 통합 대상

| 카테고리 | 대상 시스템 |
|----------|------------|
| **국제 표준** | GA4GH, HL7 FHIR Genomics |
| **연구실 시스템** | LIMS, ELN |
| **데이터베이스** | NCBI, UniProt, Ensembl |
| **WIA 내부** | WIA Healthcare, WIA AI |

---

## 2. 통합 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Bio Application                       │
├─────────────────────────────────────────────────────────────┤
│                    EcosystemManager                          │
│  ┌─────────────────────────────────────────────────────────┐│
│  │                   Adapter Registry                       ││
│  │  ┌─────────┬─────────┬─────────┬─────────┬─────────┐   ││
│  │  │ GA4GH   │  FHIR   │  LIMS   │   ELN   │Database │   ││
│  │  │ Adapter │ Adapter │ Adapter │ Adapter │ Adapter │   ││
│  │  └────┬────┴────┬────┴────┬────┴────┬────┴────┬────┘   ││
│  └───────┼─────────┼─────────┼─────────┼─────────┼────────┘│
├──────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
│          │         │         │         │         │         │
│  ┌───────┴─────────┴─────────┴─────────┴─────────┴───────┐ │
│  │               Service Client Layer                    │ │
│  │  - HTTP Client (reqwest)                              │ │
│  │  - Authentication Handler                             │ │
│  │  - Retry Logic                                        │ │
│  │  - Response Cache                                     │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         ▼                    ▼                    ▼
    ┌─────────┐          ┌─────────┐          ┌─────────┐
    │ GA4GH   │          │  LIMS   │          │ UniProt │
    │Services │          │ Systems │          │   API   │
    └─────────┘          └─────────┘          └─────────┘
```

### 2.2 계층 구조

| 계층 | 역할 | 구현 |
|------|------|------|
| **Application** | 비즈니스 로직 | lib.rs |
| **EcosystemManager** | 어댑터 관리 | ecosystem/manager.rs |
| **Adapter** | 외부 시스템 연동 | ecosystem/adapters/*.rs |
| **Service Client** | HTTP 통신 | ecosystem/client.rs |

---

## 3. 어댑터 인터페이스

### 3.1 기본 어댑터 트레이트

```rust
use async_trait::async_trait;
use serde::{Deserialize, Serialize};

/// 외부 시스템 어댑터 인터페이스
#[async_trait]
pub trait IEcosystemAdapter: Send + Sync {
    /// 어댑터 타입
    fn adapter_type(&self) -> AdapterType;

    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 초기화
    async fn initialize(&mut self, config: &AdapterConfig) -> Result<(), AdapterError>;

    /// 연결 상태 확인
    async fn health_check(&self) -> Result<HealthStatus, AdapterError>;

    /// 연결 종료
    async fn shutdown(&mut self) -> Result<(), AdapterError>;
}

/// 어댑터 타입
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AdapterType {
    Ga4gh,
    Fhir,
    Lims,
    Eln,
    Database,
    Custom,
}

/// 어댑터 설정
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdapterConfig {
    pub base_url: String,
    pub api_key: Option<String>,
    pub auth_type: AuthType,
    pub timeout_ms: u64,
    pub retry_count: u32,
    pub cache_ttl_sec: u64,
}

/// 인증 타입
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AuthType {
    None,
    ApiKey,
    OAuth2 { client_id: String, client_secret: String },
    Bearer { token: String },
}

/// 헬스 상태
#[derive(Debug, Clone)]
pub struct HealthStatus {
    pub healthy: bool,
    pub latency_ms: u64,
    pub message: Option<String>,
}
```

### 3.2 데이터 익스포트 트레이트

```rust
#[async_trait]
pub trait IDataExporter: IEcosystemAdapter {
    /// 시퀀스 내보내기
    async fn export_sequence(&self, sequence: &Sequence) -> Result<ExportResult, AdapterError>;

    /// 실험 내보내기
    async fn export_experiment(&self, experiment: &CrisprExperiment) -> Result<ExportResult, AdapterError>;

    /// 구조 내보내기
    async fn export_structure(&self, structure: &ProteinStructure) -> Result<ExportResult, AdapterError>;

    /// 배치 내보내기
    async fn export_batch(&self, items: Vec<ExportItem>) -> Result<Vec<ExportResult>, AdapterError>;
}

#[derive(Debug, Clone)]
pub struct ExportResult {
    pub success: bool,
    pub external_id: Option<String>,
    pub url: Option<String>,
    pub message: Option<String>,
}

#[derive(Debug, Clone)]
pub enum ExportItem {
    Sequence(Sequence),
    Experiment(CrisprExperiment),
    Structure(ProteinStructure),
    Part(BioPart),
}
```

### 3.3 데이터 임포트 트레이트

```rust
#[async_trait]
pub trait IDataImporter: IEcosystemAdapter {
    /// 외부 ID로 시퀀스 가져오기
    async fn import_sequence(&self, external_id: &str) -> Result<Sequence, AdapterError>;

    /// 검색으로 시퀀스 가져오기
    async fn search_sequences(&self, query: &SearchQuery) -> Result<Vec<Sequence>, AdapterError>;

    /// 단백질 구조 가져오기
    async fn import_structure(&self, external_id: &str) -> Result<ProteinStructure, AdapterError>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SearchQuery {
    pub text: Option<String>,
    pub organism: Option<String>,
    pub sequence_type: Option<SequenceType>,
    pub limit: u32,
    pub offset: u32,
}
```

---

## 4. GA4GH 통합

### 4.1 지원 API

| API | 용도 | 구현 |
|-----|------|------|
| **DRS** | 데이터 저장소 접근 | Ga4ghDrsAdapter |
| **WES** | 워크플로우 실행 | Ga4ghWesAdapter |
| **TES** | 태스크 실행 | Ga4ghTesAdapter |
| **Beacon** | 변이 검색 | Ga4ghBeaconAdapter |

### 4.2 DRS (Data Repository Service) 어댑터

```rust
/// GA4GH DRS 어댑터
pub struct Ga4ghDrsAdapter {
    config: AdapterConfig,
    client: HttpClient,
}

#[async_trait]
impl IEcosystemAdapter for Ga4ghDrsAdapter {
    fn adapter_type(&self) -> AdapterType { AdapterType::Ga4gh }
    fn name(&self) -> &str { "GA4GH DRS" }
    // ...
}

impl Ga4ghDrsAdapter {
    /// DRS 객체 조회
    pub async fn get_object(&self, object_id: &str) -> Result<DrsObject, AdapterError>;

    /// 데이터 URL 획득
    pub async fn get_access_url(&self, object_id: &str, access_id: &str) -> Result<String, AdapterError>;

    /// 번들 조회
    pub async fn get_bundle(&self, bundle_id: &str) -> Result<DrsBundle, AdapterError>;
}

/// DRS 객체
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrsObject {
    pub id: String,
    pub name: Option<String>,
    pub self_uri: String,
    pub size: u64,
    pub created_time: String,
    pub checksums: Vec<DrsChecksum>,
    pub access_methods: Vec<DrsAccessMethod>,
}
```

### 4.3 Beacon API 어댑터

```rust
/// GA4GH Beacon 어댑터
pub struct Ga4ghBeaconAdapter {
    config: AdapterConfig,
    client: HttpClient,
}

impl Ga4ghBeaconAdapter {
    /// 변이 검색
    pub async fn query_variant(
        &self,
        chromosome: &str,
        position: u64,
        reference: &str,
        alternate: &str,
    ) -> Result<BeaconResponse, AdapterError>;

    /// 범위 검색
    pub async fn query_range(
        &self,
        chromosome: &str,
        start: u64,
        end: u64,
    ) -> Result<BeaconResponse, AdapterError>;
}

/// Beacon 응답
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconResponse {
    pub exists: bool,
    pub allele_count: Option<u32>,
    pub allele_frequency: Option<f64>,
    pub datasets: Vec<BeaconDataset>,
}
```

---

## 5. FHIR Genomics 통합

### 5.1 지원 리소스

| FHIR 리소스 | WIA Bio 타입 | 변환 |
|------------|-------------|------|
| MolecularSequence | Sequence | 양방향 |
| GenomicStudy | CrisprExperiment | 양방향 |
| Observation | CrisprResults | 양방향 |
| MolecularDefinition | BioPart | 양방향 |

### 5.2 FHIR 어댑터

```rust
/// FHIR Genomics 어댑터
pub struct FhirGenomicsAdapter {
    config: AdapterConfig,
    client: HttpClient,
    fhir_version: FhirVersion,
}

#[derive(Debug, Clone)]
pub enum FhirVersion {
    R4,
    R5,
    R6,
}

impl FhirGenomicsAdapter {
    /// WIA Sequence → FHIR MolecularSequence
    pub fn to_molecular_sequence(&self, sequence: &Sequence) -> FhirMolecularSequence;

    /// FHIR MolecularSequence → WIA Sequence
    pub fn from_molecular_sequence(&self, fhir: &FhirMolecularSequence) -> Result<Sequence, AdapterError>;

    /// FHIR 서버에 리소스 생성
    pub async fn create_resource<T: FhirResource>(&self, resource: &T) -> Result<String, AdapterError>;

    /// FHIR 서버에서 리소스 조회
    pub async fn get_resource<T: FhirResource>(&self, id: &str) -> Result<T, AdapterError>;

    /// FHIR 검색
    pub async fn search<T: FhirResource>(&self, params: &SearchParams) -> Result<Bundle<T>, AdapterError>;
}

/// FHIR MolecularSequence
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirMolecularSequence {
    pub resource_type: String,
    pub id: Option<String>,
    pub r#type: String,  // dna, rna, aa
    pub coordinate_system: i32,
    pub patient: Option<FhirReference>,
    pub specimen: Option<FhirReference>,
    pub literal: Option<String>,
}
```

### 5.3 FHIR Operations

```rust
impl FhirGenomicsAdapter {
    /// $find-subject-variants 오퍼레이션
    pub async fn find_subject_variants(
        &self,
        patient_id: &str,
        gene: Option<&str>,
    ) -> Result<Vec<Variant>, AdapterError>;

    /// $match 오퍼레이션 (임상시험 매칭)
    pub async fn match_clinical_trials(
        &self,
        patient_id: &str,
    ) -> Result<Vec<ClinicalTrial>, AdapterError>;
}
```

---

## 6. LIMS 통합

### 6.1 LIMS 어댑터

```rust
/// 범용 LIMS 어댑터
pub struct LimsAdapter {
    config: AdapterConfig,
    client: HttpClient,
    lims_type: LimsType,
}

#[derive(Debug, Clone)]
pub enum LimsType {
    Generic,
    LabWare,
    Starlims,
    LabVantage,
    Custom(String),
}

#[async_trait]
impl IDataExporter for LimsAdapter {
    async fn export_sequence(&self, sequence: &Sequence) -> Result<ExportResult, AdapterError> {
        let sample = self.to_lims_sample(sequence);
        self.create_sample(&sample).await
    }
    // ...
}

impl LimsAdapter {
    /// 샘플 생성
    pub async fn create_sample(&self, sample: &LimsSample) -> Result<ExportResult, AdapterError>;

    /// 샘플 조회
    pub async fn get_sample(&self, sample_id: &str) -> Result<LimsSample, AdapterError>;

    /// 실험 생성
    pub async fn create_experiment(&self, experiment: &LimsExperiment) -> Result<ExportResult, AdapterError>;

    /// 결과 등록
    pub async fn submit_results(&self, results: &LimsResults) -> Result<ExportResult, AdapterError>;

    /// 워크플로우 시작
    pub async fn start_workflow(&self, workflow_id: &str, samples: Vec<&str>) -> Result<String, AdapterError>;
}

/// LIMS 샘플
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LimsSample {
    pub sample_id: String,
    pub sample_type: String,
    pub name: String,
    pub description: Option<String>,
    pub properties: HashMap<String, serde_json::Value>,
    pub created_at: DateTime<Utc>,
}
```

---

## 7. ELN 통합

### 7.1 ELN 어댑터

```rust
/// ELN 어댑터 (Benchling 호환)
pub struct ElnAdapter {
    config: AdapterConfig,
    client: HttpClient,
    eln_type: ElnType,
}

#[derive(Debug, Clone)]
pub enum ElnType {
    Benchling,
    Scispot,
    SignalsNotebook,
    SciNote,
    Custom(String),
}

impl ElnAdapter {
    /// 노트북 엔트리 생성
    pub async fn create_entry(&self, entry: &ElnEntry) -> Result<ExportResult, AdapterError>;

    /// 시퀀스 등록
    pub async fn register_sequence(&self, sequence: &Sequence) -> Result<ExportResult, AdapterError>;

    /// 실험 기록
    pub async fn log_experiment(&self, experiment: &CrisprExperiment) -> Result<ExportResult, AdapterError>;

    /// 첨부파일 업로드
    pub async fn upload_attachment(&self, entry_id: &str, file: &[u8], filename: &str) -> Result<String, AdapterError>;

    /// 엔트리 검색
    pub async fn search_entries(&self, query: &str) -> Result<Vec<ElnEntry>, AdapterError>;
}

/// ELN 엔트리
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElnEntry {
    pub entry_id: Option<String>,
    pub name: String,
    pub folder_id: Option<String>,
    pub content: String,
    pub schema_id: Option<String>,
    pub fields: HashMap<String, serde_json::Value>,
    pub authors: Vec<String>,
    pub created_at: Option<DateTime<Utc>>,
}
```

---

## 8. 데이터베이스 통합

### 8.1 NCBI 어댑터

```rust
/// NCBI E-utilities 어댑터
pub struct NcbiAdapter {
    config: AdapterConfig,
    client: HttpClient,
}

impl NcbiAdapter {
    /// 검색 (esearch)
    pub async fn search(&self, database: NcbiDatabase, query: &str) -> Result<Vec<String>, AdapterError>;

    /// 조회 (efetch)
    pub async fn fetch(&self, database: NcbiDatabase, ids: &[&str]) -> Result<String, AdapterError>;

    /// GenBank 시퀀스 가져오기
    pub async fn get_genbank_sequence(&self, accession: &str) -> Result<Sequence, AdapterError>;

    /// PubMed 검색
    pub async fn search_pubmed(&self, query: &str) -> Result<Vec<PubmedArticle>, AdapterError>;
}

#[derive(Debug, Clone, Copy)]
pub enum NcbiDatabase {
    Nucleotide,
    Protein,
    Gene,
    Pubmed,
    Structure,
}
```

### 8.2 UniProt 어댑터

```rust
/// UniProt/EBI Proteins API 어댑터
pub struct UniProtAdapter {
    config: AdapterConfig,
    client: HttpClient,
}

impl UniProtAdapter {
    /// 단백질 조회
    pub async fn get_protein(&self, accession: &str) -> Result<UniProtEntry, AdapterError>;

    /// 검색
    pub async fn search(&self, query: &str, limit: u32) -> Result<Vec<UniProtEntry>, AdapterError>;

    /// WIA ProteinStructure로 변환
    pub async fn to_protein_structure(&self, accession: &str) -> Result<ProteinStructure, AdapterError>;

    /// 변이 정보 조회
    pub async fn get_variants(&self, accession: &str) -> Result<Vec<UniProtVariant>, AdapterError>;
}

/// UniProt 엔트리
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniProtEntry {
    pub accession: String,
    pub name: String,
    pub protein_name: String,
    pub organism: String,
    pub sequence: String,
    pub length: u32,
    pub features: Vec<UniProtFeature>,
}
```

---

## 9. 에러 처리

### 9.1 어댑터 에러

```rust
#[derive(Debug, Clone, thiserror::Error)]
pub enum AdapterError {
    /// 연결 실패
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    /// 인증 실패
    #[error("Authentication failed: {0}")]
    AuthenticationFailed(String),

    /// 요청 타임아웃
    #[error("Request timeout after {0}ms")]
    Timeout(u64),

    /// 리소스 없음
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// 변환 실패
    #[error("Conversion failed: {0}")]
    ConversionFailed(String),

    /// 레이트 제한
    #[error("Rate limited: retry after {0}s")]
    RateLimited(u64),

    /// 서버 에러
    #[error("Server error ({code}): {message}")]
    ServerError { code: u16, message: String },

    /// 검증 실패
    #[error("Validation failed: {0}")]
    ValidationFailed(String),
}
```

### 9.2 재시도 정책

```rust
/// 재시도 정책
#[derive(Debug, Clone)]
pub struct RetryPolicy {
    pub max_retries: u32,
    pub initial_delay_ms: u64,
    pub max_delay_ms: u64,
    pub exponential_base: f64,
    pub retryable_errors: Vec<ErrorKind>,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_retries: 3,
            initial_delay_ms: 1000,
            max_delay_ms: 30000,
            exponential_base: 2.0,
            retryable_errors: vec![
                ErrorKind::Timeout,
                ErrorKind::RateLimited,
                ErrorKind::ServerError,
            ],
        }
    }
}
```

---

## 10. 보안

### 10.1 인증 처리

```rust
/// OAuth2 인증 핸들러
pub struct OAuth2Handler {
    client_id: String,
    client_secret: String,
    token_url: String,
    cached_token: RwLock<Option<OAuth2Token>>,
}

impl OAuth2Handler {
    pub async fn get_access_token(&self) -> Result<String, AdapterError>;
    pub async fn refresh_token(&self) -> Result<(), AdapterError>;
}

/// API Key 인증 핸들러
pub struct ApiKeyHandler {
    api_key: String,
    header_name: String,
}
```

### 10.2 데이터 보안

- 전송 중 암호화: TLS 1.3 필수
- 저장 시 암호화: AES-256
- API 키/토큰: 환경 변수 또는 시크릿 매니저
- 감사 로깅: 모든 외부 API 호출 기록

---

## 11. 예제

### 11.1 GA4GH DRS 사용

```rust
use wia_bio::ecosystem::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 어댑터 생성
    let config = AdapterConfig {
        base_url: "https://drs.example.org".to_string(),
        api_key: Some("your-api-key".to_string()),
        auth_type: AuthType::ApiKey,
        timeout_ms: 30000,
        retry_count: 3,
        cache_ttl_sec: 300,
    };

    let mut adapter = Ga4ghDrsAdapter::new();
    adapter.initialize(&config).await?;

    // 객체 조회
    let object = adapter.get_object("drs://example.org/abc123").await?;
    println!("Object: {} ({} bytes)", object.name.unwrap_or_default(), object.size);

    // 다운로드 URL 획득
    let url = adapter.get_access_url(&object.id, "https").await?;
    println!("Download URL: {}", url);

    Ok(())
}
```

### 11.2 FHIR Genomics 변환

```rust
use wia_bio::ecosystem::*;

// WIA Sequence를 FHIR로 변환
let sequence = create_sequence("BRCA1", "ATCGATCGATCG", SequenceType::Dna)?;

let fhir_adapter = FhirGenomicsAdapter::new(FhirVersion::R5);
fhir_adapter.initialize(&config).await?;

// FHIR 서버에 등록
let fhir_seq = fhir_adapter.to_molecular_sequence(&sequence);
let fhir_id = fhir_adapter.create_resource(&fhir_seq).await?;
println!("Created FHIR resource: {}", fhir_id);
```

### 11.3 멀티 어댑터 사용

```rust
use wia_bio::ecosystem::*;

// EcosystemManager 생성
let mut manager = EcosystemManager::new();

// 여러 어댑터 등록
manager.register(Ga4ghDrsAdapter::new());
manager.register(FhirGenomicsAdapter::new(FhirVersion::R5));
manager.register(NcbiAdapter::new());

// 초기화
manager.initialize_all(&configs).await?;

// 헬스 체크
let statuses = manager.health_check_all().await;
for (adapter_type, status) in statuses {
    println!("{:?}: {} ({}ms)", adapter_type, status.healthy, status.latency_ms);
}

// 시퀀스를 여러 시스템에 내보내기
let sequence = create_sequence("MyGene", "ATCGATCG", SequenceType::Dna)?;
let results = manager.export_to_all(&ExportItem::Sequence(sequence)).await;

for (adapter_type, result) in results {
    if result.success {
        println!("{:?}: Exported as {}", adapter_type, result.external_id.unwrap_or_default());
    }
}
```

---

## 참조

1. [GA4GH Specifications](https://www.ga4gh.org/our-products/)
2. [HL7 FHIR Genomics](https://www.hl7.org/fhir/genomics.html)
3. [NCBI E-utilities](https://www.ncbi.nlm.nih.gov/home/develop/api/)
4. [UniProt REST API](https://www.uniprot.org/help/api)
5. [Benchling API](https://benchling.com/api)

---

*WIA Biotechnology Standard v1.0.0*
*Phase 4: Ecosystem Integration*
