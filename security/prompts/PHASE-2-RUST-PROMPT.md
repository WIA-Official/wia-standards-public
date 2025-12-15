# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Security (Cybersecurity Standards)
**Phase**: 2 of 4
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 사이버보안 API 구현
**난이도**: ★★★★★
**예상 작업량**: Rust 라이브러리 + 암호화 모듈 + 테스트 + 예제

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 Security Data Format을 정의했다.

 이제 이 데이터를 프로그래밍 방식으로 어떻게 다룰 것인가?

 - 취약점 스캔 결과 파싱 및 분석?
 - CVSS 점수 자동 계산?
 - 양자내성 암호화 구현?
 - 침투테스트 보고서 생성?
 - 위협 인텔리전스 분석?

 모든 기능을 안전하고 빠른 API로 제공할 수 있을까?"
```

### 목표
```
사이버보안 데이터를 처리하는 Rust API 구현

- 데이터 타입 정의 (Phase 1 스키마 기반)
- 암호화 프리미티브 (PQC 포함)
- 취약점 평가 엔진
- CVSS 계산기
- 보안 스캐너 인터페이스
- WASM/Python 바인딩 지원
```

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (암호화 연산, 대용량 로그 분석)
2. 메모리 안전: 보안 도구에 필수적인 메모리 안전성 보장
3. 동시성: 병렬 스캔 및 분석에 유리
4. 암호화 생태계: ring, rustls, RustCrypto 등 강력한 크레이트
5. Zero-cost Abstractions: 안전성과 성능을 동시에
6. 크로스 플랫폼: WASM으로 브라우저에서도 실행 가능
7. 일관성: WIA 표준 전체에서 Rust 사용
```

---

## 📦 프로젝트 구조

```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs                    # 메인 라이브러리
│   ├── types.rs                  # 타입 정의
│   ├── error.rs                  # 에러 타입
│   ├── crypto/
│   │   ├── mod.rs
│   │   ├── pqc.rs                # Post-Quantum Crypto
│   │   ├── hash.rs               # 해시 함수
│   │   ├── symmetric.rs          # 대칭키 암호
│   │   └── asymmetric.rs         # 비대칭키 암호
│   ├── scanner/
│   │   ├── mod.rs
│   │   ├── vulnerability.rs      # 취약점 스캐너
│   │   ├── port.rs               # 포트 스캐너
│   │   └── web.rs                # 웹 스캐너
│   ├── assessment/
│   │   ├── mod.rs
│   │   ├── cvss.rs               # CVSS 계산
│   │   ├── pentest.rs            # 침투 테스트
│   │   ├── zero_trust.rs         # Zero Trust 평가
│   │   └── ai_security.rs        # AI 보안
│   ├── threat_intel/
│   │   ├── mod.rs
│   │   ├── ioc.rs                # IOC 분석
│   │   ├── stix.rs               # STIX 파싱
│   │   └── mitre_attack.rs       # MITRE ATT&CK
│   ├── reporting/
│   │   ├── mod.rs
│   │   ├── generator.rs          # 보고서 생성
│   │   └── export.rs             # 다양한 형식 출력
│   └── prelude.rs                # 편의 re-exports
├── tests/
│   ├── integration_test.rs
│   ├── crypto_test.rs
│   └── scanner_test.rs
├── benches/
│   └── crypto_bench.rs
└── examples/
    ├── basic_usage.rs
    ├── vulnerability_scan.rs
    ├── cvss_calculator.rs
    ├── pqc_crypto.rs
    └── threat_analysis.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)

```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SecurityError {
    #[error("Cryptographic error: {0}")]
    CryptoError(String),

    #[error("Invalid CVSS vector: {0}")]
    InvalidCvssVector(String),

    #[error("Scan error: {0}")]
    ScanError(String),

    #[error("Invalid vulnerability data: {0}")]
    InvalidVulnerability(String),

    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parsing error: {0}")]
    ParsingError(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("TLS error: {0}")]
    TlsError(String),

    #[error("Authentication failed: {0}")]
    AuthenticationError(String),
}

pub type SecurityResult<T> = std::result::Result<T, SecurityError>;
```

### 기본 타입 정의 (types.rs)

```rust
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// 보안 평가 유형
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentType {
    PostQuantumCrypto,
    PenetrationTesting,
    ZeroTrust,
    AiSecurity,
    ThreatIntelligence,
    VulnerabilityManagement,
}

/// 심각도 레벨
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

impl Severity {
    pub fn from_cvss_score(score: f64) -> Self {
        match score {
            s if s >= 9.0 => Severity::Critical,
            s if s >= 7.0 => Severity::High,
            s if s >= 4.0 => Severity::Medium,
            s if s >= 0.1 => Severity::Low,
            _ => Severity::Info,
        }
    }

    pub fn color_code(&self) -> &'static str {
        match self {
            Severity::Critical => "#8B0000",  // Dark Red
            Severity::High => "#FF4500",      // Orange Red
            Severity::Medium => "#FFA500",    // Orange
            Severity::Low => "#FFD700",       // Gold
            Severity::Info => "#4682B4",      // Steel Blue
        }
    }
}

/// 보안 평가 상태
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AssessmentStatus {
    Planned,
    InProgress,
    Completed,
    Remediated,
    Accepted,
    Rejected,
}

/// 기본 평가 구조
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurityAssessment {
    pub id: Uuid,
    pub assessment_type: AssessmentType,
    pub name: String,
    pub status: AssessmentStatus,
    pub severity: Severity,
    pub target: Target,
    pub findings: Vec<Finding>,
    pub timeline: Timeline,
    pub compliance: Compliance,
    pub meta: Metadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Target {
    pub organization: String,
    pub systems: Vec<String>,
    pub scope: String,
    pub environment: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Finding {
    pub id: String,
    pub category: String,
    pub title: String,
    pub description: String,
    pub severity: Severity,
    pub cvss_score: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timeline {
    pub started_at: DateTime<Utc>,
    pub completed_at: Option<DateTime<Utc>>,
    pub duration_hours: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Compliance {
    pub frameworks: Vec<String>,
    pub controls: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub assessor: String,
    pub tool_used: String,
    pub methodology: String,
}
```

### CVSS 계산기 (assessment/cvss.rs)

```rust
use crate::{SecurityResult, SecurityError, Severity};
use serde::{Deserialize, Serialize};

/// CVSS v3.1 메트릭
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV3 {
    pub vector_string: String,
    pub attack_vector: AttackVector,
    pub attack_complexity: AttackComplexity,
    pub privileges_required: PrivilegesRequired,
    pub user_interaction: UserInteraction,
    pub scope: Scope,
    pub confidentiality_impact: Impact,
    pub integrity_impact: Impact,
    pub availability_impact: Impact,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum AttackVector {
    Network,       // AV:N
    Adjacent,      // AV:A
    Local,         // AV:L
    Physical,      // AV:P
}

impl AttackVector {
    fn metric_value(&self) -> f64 {
        match self {
            AttackVector::Network => 0.85,
            AttackVector::Adjacent => 0.62,
            AttackVector::Local => 0.55,
            AttackVector::Physical => 0.2,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum AttackComplexity {
    Low,   // AC:L
    High,  // AC:H
}

impl AttackComplexity {
    fn metric_value(&self) -> f64 {
        match self {
            AttackComplexity::Low => 0.77,
            AttackComplexity::High => 0.44,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PrivilegesRequired {
    None,   // PR:N
    Low,    // PR:L
    High,   // PR:H
}

impl PrivilegesRequired {
    fn metric_value(&self, scope_changed: bool) -> f64 {
        match (self, scope_changed) {
            (PrivilegesRequired::None, _) => 0.85,
            (PrivilegesRequired::Low, false) => 0.62,
            (PrivilegesRequired::Low, true) => 0.68,
            (PrivilegesRequired::High, false) => 0.27,
            (PrivilegesRequired::High, true) => 0.50,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum UserInteraction {
    None,      // UI:N
    Required,  // UI:R
}

impl UserInteraction {
    fn metric_value(&self) -> f64 {
        match self {
            UserInteraction::None => 0.85,
            UserInteraction::Required => 0.62,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Scope {
    Unchanged,  // S:U
    Changed,    // S:C
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Impact {
    None,   // C:N, I:N, A:N
    Low,    // C:L, I:L, A:L
    High,   // C:H, I:H, A:H
}

impl Impact {
    fn metric_value(&self) -> f64 {
        match self {
            Impact::None => 0.0,
            Impact::Low => 0.22,
            Impact::High => 0.56,
        }
    }
}

impl CvssV3 {
    /// CVSS Base Score 계산 (0.0 ~ 10.0)
    pub fn calculate_base_score(&self) -> f64 {
        let scope_changed = matches!(self.scope, Scope::Changed);

        // Impact Sub-Score
        let isc_base = 1.0 - (
            (1.0 - self.confidentiality_impact.metric_value()) *
            (1.0 - self.integrity_impact.metric_value()) *
            (1.0 - self.availability_impact.metric_value())
        );

        let impact = if scope_changed {
            7.52 * (isc_base - 0.029) - 3.25 * (isc_base - 0.02).powi(15)
        } else {
            6.42 * isc_base
        };

        // Exploitability Sub-Score
        let exploitability = 8.22 *
            self.attack_vector.metric_value() *
            self.attack_complexity.metric_value() *
            self.privileges_required.metric_value(scope_changed) *
            self.user_interaction.metric_value();

        // Base Score
        let score = if impact <= 0.0 {
            0.0
        } else if scope_changed {
            roundup((1.08 * (impact + exploitability)).min(10.0))
        } else {
            roundup((impact + exploitability).min(10.0))
        };

        score
    }

    /// CVSS 벡터 문자열에서 파싱
    pub fn from_vector(vector: &str) -> SecurityResult<Self> {
        // 예: "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H"

        if !vector.starts_with("CVSS:3.1/") && !vector.starts_with("CVSS:3.0/") {
            return Err(SecurityError::InvalidCvssVector(
                "Must start with CVSS:3.1/ or CVSS:3.0/".into()
            ));
        }

        let parts: Vec<&str> = vector.split('/').skip(1).collect();

        let mut av = None;
        let mut ac = None;
        let mut pr = None;
        let mut ui = None;
        let mut s = None;
        let mut c = None;
        let mut i = None;
        let mut a = None;

        for part in parts {
            let kv: Vec<&str> = part.split(':').collect();
            if kv.len() != 2 {
                return Err(SecurityError::InvalidCvssVector(format!("Invalid metric: {}", part)));
            }

            match kv[0] {
                "AV" => av = Some(match kv[1] {
                    "N" => AttackVector::Network,
                    "A" => AttackVector::Adjacent,
                    "L" => AttackVector::Local,
                    "P" => AttackVector::Physical,
                    _ => return Err(SecurityError::InvalidCvssVector(format!("Invalid AV: {}", kv[1]))),
                }),
                "AC" => ac = Some(match kv[1] {
                    "L" => AttackComplexity::Low,
                    "H" => AttackComplexity::High,
                    _ => return Err(SecurityError::InvalidCvssVector(format!("Invalid AC: {}", kv[1]))),
                }),
                "PR" => pr = Some(match kv[1] {
                    "N" => PrivilegesRequired::None,
                    "L" => PrivilegesRequired::Low,
                    "H" => PrivilegesRequired::High,
                    _ => return Err(SecurityError::InvalidCvssVector(format!("Invalid PR: {}", kv[1]))),
                }),
                "UI" => ui = Some(match kv[1] {
                    "N" => UserInteraction::None,
                    "R" => UserInteraction::Required,
                    _ => return Err(SecurityError::InvalidCvssVector(format!("Invalid UI: {}", kv[1]))),
                }),
                "S" => s = Some(match kv[1] {
                    "U" => Scope::Unchanged,
                    "C" => Scope::Changed,
                    _ => return Err(SecurityError::InvalidCvssVector(format!("Invalid S: {}", kv[1]))),
                }),
                "C" => c = Some(parse_impact(kv[1])?),
                "I" => i = Some(parse_impact(kv[1])?),
                "A" => a = Some(parse_impact(kv[1])?),
                _ => {},
            }
        }

        Ok(CvssV3 {
            vector_string: vector.to_string(),
            attack_vector: av.ok_or_else(|| SecurityError::InvalidCvssVector("Missing AV".into()))?,
            attack_complexity: ac.ok_or_else(|| SecurityError::InvalidCvssVector("Missing AC".into()))?,
            privileges_required: pr.ok_or_else(|| SecurityError::InvalidCvssVector("Missing PR".into()))?,
            user_interaction: ui.ok_or_else(|| SecurityError::InvalidCvssVector("Missing UI".into()))?,
            scope: s.ok_or_else(|| SecurityError::InvalidCvssVector("Missing S".into()))?,
            confidentiality_impact: c.ok_or_else(|| SecurityError::InvalidCvssVector("Missing C".into()))?,
            integrity_impact: i.ok_or_else(|| SecurityError::InvalidCvssVector("Missing I".into()))?,
            availability_impact: a.ok_or_else(|| SecurityError::InvalidCvssVector("Missing A".into()))?,
        })
    }

    pub fn severity(&self) -> Severity {
        Severity::from_cvss_score(self.calculate_base_score())
    }
}

fn parse_impact(s: &str) -> SecurityResult<Impact> {
    match s {
        "N" => Ok(Impact::None),
        "L" => Ok(Impact::Low),
        "H" => Ok(Impact::High),
        _ => Err(SecurityError::InvalidCvssVector(format!("Invalid impact: {}", s))),
    }
}

fn roundup(value: f64) -> f64 {
    (value * 10.0).ceil() / 10.0
}
```

### Post-Quantum Cryptography (crypto/pqc.rs)

```rust
use crate::{SecurityResult, SecurityError};
use serde::{Deserialize, Serialize};

/// NIST PQC 알고리즘
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PqcAlgorithm {
    /// Key Encapsulation Mechanism
    CrystalsKyber,

    /// Digital Signatures
    CrystalsDilithium,
    Falcon,
    Sphincs,
}

/// NIST 보안 레벨
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum NistSecurityLevel {
    Level1,  // AES-128 상당
    Level2,  // SHA-256 collision 상당
    Level3,  // AES-192 상당
    Level4,  // SHA-384 collision 상당
    Level5,  // AES-256 상당
}

impl NistSecurityLevel {
    pub fn classical_bits(&self) -> u32 {
        match self {
            NistSecurityLevel::Level1 => 128,
            NistSecurityLevel::Level2 => 128,
            NistSecurityLevel::Level3 => 192,
            NistSecurityLevel::Level4 => 192,
            NistSecurityLevel::Level5 => 256,
        }
    }

    pub fn quantum_bits(&self) -> u32 {
        match self {
            NistSecurityLevel::Level1 => 128,
            NistSecurityLevel::Level2 => 256,
            NistSecurityLevel::Level3 => 192,
            NistSecurityLevel::Level4 => 384,
            NistSecurityLevel::Level5 => 256,
        }
    }
}

/// PQC 키 교환 데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqcKeyExchange {
    pub algorithm: PqcAlgorithm,
    pub security_level: NistSecurityLevel,
    pub public_key: Vec<u8>,
    pub ciphertext: Option<Vec<u8>>,
    pub shared_secret: Option<Vec<u8>>,
}

/// 양자 위협 평가
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumThreatAssessment {
    pub current_algorithm: String,
    pub key_size: usize,
    pub vulnerable_to_quantum: bool,
    pub estimated_break_time: String,
    pub recommended_pqc: PqcAlgorithm,
    pub migration_complexity: MigrationComplexity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MigrationComplexity {
    Low,
    Medium,
    High,
    Critical,
}

impl QuantumThreatAssessment {
    /// RSA/ECC 양자 취약성 평가
    pub fn assess_classical_crypto(algorithm: &str, key_size: usize) -> Self {
        let vulnerable = matches!(algorithm, "RSA" | "ECDSA" | "ECDH" | "DSA" | "DH");

        let (break_time, complexity) = match algorithm {
            "RSA" | "DSA" | "DH" if key_size <= 2048 => ("Minutes (Shor's algorithm)", MigrationComplexity::Critical),
            "RSA" | "DSA" | "DH" if key_size <= 4096 => ("Minutes (Shor's algorithm)", MigrationComplexity::High),
            "ECDSA" | "ECDH" if key_size <= 256 => ("Minutes (Shor's algorithm)", MigrationComplexity::High),
            "AES" if key_size == 128 => ("Reduced to 64-bit (Grover)", MigrationComplexity::Medium),
            "AES" if key_size == 256 => ("Reduced to 128-bit (Grover)", MigrationComplexity::Low),
            _ => ("Not vulnerable", MigrationComplexity::Low),
        };

        QuantumThreatAssessment {
            current_algorithm: algorithm.to_string(),
            key_size,
            vulnerable_to_quantum: vulnerable,
            estimated_break_time: break_time.to_string(),
            recommended_pqc: PqcAlgorithm::CrystalsKyber,
            migration_complexity: complexity,
        }
    }
}
```

### 취약점 스캐너 (scanner/vulnerability.rs)

```rust
use crate::{SecurityResult, SecurityError, Severity};
use serde::{Deserialize, Serialize};
use std::net::IpAddr;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VulnerabilityScan {
    pub target: ScanTarget,
    pub scan_type: ScanType,
    pub vulnerabilities: Vec<Vulnerability>,
    pub scan_duration_seconds: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanTarget {
    pub ip: IpAddr,
    pub hostname: Option<String>,
    pub ports: Vec<u16>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScanType {
    Full,
    Quick,
    Compliance,
    WebApp,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vulnerability {
    pub id: String,           // CVE ID or custom
    pub name: String,
    pub severity: Severity,
    pub cvss_score: f64,
    pub description: String,
    pub affected_component: String,
    pub port: Option<u16>,
    pub protocol: Option<String>,
    pub exploit_available: bool,
    pub patch_available: bool,
    pub remediation: String,
    pub references: Vec<String>,
}

impl VulnerabilityScan {
    pub fn critical_count(&self) -> usize {
        self.vulnerabilities.iter()
            .filter(|v| v.severity == Severity::Critical)
            .count()
    }

    pub fn high_count(&self) -> usize {
        self.vulnerabilities.iter()
            .filter(|v| v.severity == Severity::High)
            .count()
    }

    pub fn risk_score(&self) -> f64 {
        let weights = [
            (Severity::Critical, 10.0),
            (Severity::High, 7.0),
            (Severity::Medium, 4.0),
            (Severity::Low, 1.0),
            (Severity::Info, 0.1),
        ];

        self.vulnerabilities.iter()
            .map(|v| {
                weights.iter()
                    .find(|(s, _)| s == &v.severity)
                    .map(|(_, w)| w)
                    .unwrap_or(&0.0)
            })
            .sum()
    }
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-security"
version = "1.0.0"
edition = "2021"
description = "WIA Security Standards - Rust SDK for Cybersecurity"
license = "MIT"
repository = "https://github.com/WIA-Official/wia-standards"
keywords = ["security", "cybersecurity", "cryptography", "pqc", "vulnerability"]
categories = ["cryptography", "network-programming", "web-programming"]

[dependencies]
# Async runtime
tokio = { version = "1", features = ["full"] }
async-trait = "0.1"

# Serialization
serde = { version = "1", features = ["derive"] }
serde_json = "1"

# Error handling
thiserror = "1"
anyhow = "1"

# Date/Time
chrono = { version = "0.4", features = ["serde"] }

# UUID
uuid = { version = "1", features = ["v4", "serde"] }

# Cryptography
ring = "0.17"                    # 암호화 프리미티브
rustls = "0.23"                  # TLS
sha2 = "0.10"                    # SHA-256/384/512
sha3 = "0.10"                    # SHA-3
blake3 = "1.5"                   # BLAKE3
ed25519-dalek = "2"              # Ed25519 서명
x25519-dalek = "2"               # X25519 키 교환
aes-gcm = "0.10"                 # AES-GCM
chacha20poly1305 = "0.10"        # ChaCha20-Poly1305

# Post-Quantum Cryptography (experimental)
# oqs = { version = "0.9", optional = true }  # liboqs wrapper

# Network
reqwest = { version = "0.12", features = ["json", "rustls-tls"] }
tokio-rustls = "0.26"

# Web Assembly support
wasm-bindgen = { version = "0.2", optional = true }
wasm-bindgen-futures = { version = "0.4", optional = true }

# Python bindings
pyo3 = { version = "0.20", features = ["extension-module"], optional = true }

# Logging
tracing = "0.1"
tracing-subscriber = "0.3"

[dev-dependencies]
tokio-test = "0.4"
criterion = "0.5"
proptest = "1"

[features]
default = ["crypto"]
crypto = []
pqc = []  # Post-Quantum Crypto (experimental)
wasm = ["wasm-bindgen", "wasm-bindgen-futures"]
python = ["pyo3"]

[[bench]]
name = "crypto_bench"
harness = false
```

---

## 🚀 사용 예시

### Basic Usage

```rust
use wia_security::prelude::*;

#[tokio::main]
async fn main() -> SecurityResult<()> {
    // CVSS 점수 계산
    let cvss = CvssV3::from_vector(
        "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H"
    )?;

    let score = cvss.calculate_base_score();
    println!("CVSS Score: {:.1}", score);
    println!("Severity: {:?}", cvss.severity());

    // 양자 위협 평가
    let threat = QuantumThreatAssessment::assess_classical_crypto("RSA", 2048);
    println!("Quantum Vulnerable: {}", threat.vulnerable_to_quantum);
    println!("Recommended: {:?}", threat.recommended_pqc);

    Ok(())
}
```

### Vulnerability Scanning

```rust
use wia_security::{VulnerabilityScan, ScanTarget, Severity};
use std::net::IpAddr;

async fn scan_example() -> SecurityResult<()> {
    let target = ScanTarget {
        ip: "192.168.1.100".parse::<IpAddr>().unwrap(),
        hostname: Some("web-server-01".into()),
        ports: vec![80, 443, 22, 3306],
    };

    let scan = perform_scan(target).await?;

    println!("Critical: {}", scan.critical_count());
    println!("High: {}", scan.high_count());
    println!("Risk Score: {:.2}", scan.risk_score());

    for vuln in scan.vulnerabilities.iter().filter(|v| v.severity == Severity::Critical) {
        println!("🚨 {}: {} (CVSS: {})", vuln.id, vuln.name, vuln.cvss_score);
    }

    Ok(())
}
```

---

## 📁 산출물 목록

```
/api/rust/Cargo.toml
/api/rust/src/lib.rs
/api/rust/src/types.rs
/api/rust/src/error.rs
/api/rust/src/prelude.rs
/api/rust/src/crypto/mod.rs
/api/rust/src/crypto/pqc.rs
/api/rust/src/crypto/hash.rs
/api/rust/src/crypto/symmetric.rs
/api/rust/src/crypto/asymmetric.rs
/api/rust/src/scanner/mod.rs
/api/rust/src/scanner/vulnerability.rs
/api/rust/src/scanner/port.rs
/api/rust/src/scanner/web.rs
/api/rust/src/assessment/mod.rs
/api/rust/src/assessment/cvss.rs
/api/rust/src/assessment/pentest.rs
/api/rust/src/assessment/zero_trust.rs
/api/rust/src/assessment/ai_security.rs
/api/rust/src/threat_intel/mod.rs
/api/rust/src/threat_intel/ioc.rs
/api/rust/src/threat_intel/stix.rs
/api/rust/src/threat_intel/mitre_attack.rs
/api/rust/src/reporting/mod.rs
/api/rust/src/reporting/generator.rs
/api/rust/src/reporting/export.rs
/api/rust/tests/integration_test.rs
/api/rust/examples/*.rs
/api/rust/README.md
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성 (암호화 의존성 포함)
□ Error 타입 정의
□ 기본 타입 정의 (Severity, AssessmentType 등)
□ CVSS 계산기 구현 (v3.1 표준)
□ Post-Quantum Crypto 모듈 구현
□ 암호화 프리미티브 구현 (해시, 대칭/비대칭키)
□ 취약점 스캐너 인터페이스 구현
□ 위협 인텔리전스 파서 구현 (STIX, IOC)
□ MITRE ATT&CK 매핑 구현
□ 보고서 생성기 구현
□ 단위 테스트 작성
□ 통합 테스트 작성
□ 벤치마크 작성 (암호화 성능)
□ 예제 코드 작성 (5개 이상)
□ cargo test 통과
□ cargo clippy 경고 없음
□ cargo bench 실행
□ README 업데이트
```

---

## 🔄 작업 순서

```
1. Cargo.toml 생성 (암호화 크레이트 포함)
   ↓
2. error.rs - 에러 타입 정의
   ↓
3. types.rs - 기본 타입 정의
   ↓
4. assessment/cvss.rs - CVSS 계산기
   ↓
5. crypto/pqc.rs - Post-Quantum Crypto
   ↓
6. crypto/hash.rs - 해시 함수
   ↓
7. crypto/symmetric.rs - 대칭키 암호
   ↓
8. crypto/asymmetric.rs - 비대칭키 암호
   ↓
9. scanner/vulnerability.rs - 취약점 스캐너
   ↓
10. threat_intel/stix.rs - STIX 파서
   ↓
11. threat_intel/mitre_attack.rs - MITRE 매핑
   ↓
12. reporting/generator.rs - 보고서 생성
   ↓
13. 테스트 작성 및 실행
   ↓
14. 예제 코드 작성
   ↓
15. 벤치마크 실행
   ↓
16. 완료 체크리스트 확인
   ↓
17. Phase 3 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 스키마와 1:1 대응되는 타입 정의
✅ CVSS 3.1 공식을 정확히 구현
✅ 암호화는 검증된 크레이트 사용 (ring, RustCrypto)
✅ 모든 민감 데이터는 zeroize로 메모리 클리어
✅ 타이밍 공격 방지 (constant-time 비교)
✅ Result 타입으로 에러 처리
✅ async/await로 비동기 처리
✅ 철저한 테스트 (단위 + 통합 + 속성 기반)
✅ 벤치마크로 성능 측정
```

### DON'T (하지 말 것)

```
❌ 자체 암호화 알고리즘 구현
❌ panic! 사용 (Result 반환)
❌ unwrap() 남용 (? 연산자 사용)
❌ 민감 정보 로그에 출력
❌ 하드코딩된 시크릿/키
❌ 동기 블로킹 코드 (async 사용)
❌ unsafe 남용 (반드시 필요한 경우만)
```

---

## 🔗 참고 자료

### Rust Cryptography
- **RustCrypto**: https://github.com/RustCrypto
- **ring**: https://github.com/briansmith/ring
- **rustls**: https://github.com/rustls/rustls
- **liboqs (PQC)**: https://github.com/open-quantum-safe/liboqs

### Security Standards
- **CVSS Calculator**: https://www.first.org/cvss/calculator/3.1
- **NIST PQC**: https://csrc.nist.gov/projects/post-quantum-cryptography
- **OWASP**: https://owasp.org/

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **Cargo.toml 생성 후 error.rs 구현**

```bash
cd /home/user/wia-standards/security
cargo new --lib api/rust
cd api/rust
```

사이버보안의 안전한 구현을 위해! 🦀🔐

---

<div align="center">

**Phase 2 of 4**

Rust API Implementation

🦀 Safe, Secure, Fast 🔐

🛡️ 弘益人間 - Benefit All Humanity 🛡️

</div>
