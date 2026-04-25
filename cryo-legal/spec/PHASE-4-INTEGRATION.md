# CRYO-LEGAL Phase 4: Integration Specification

## Overview

This document defines the system integration architecture for legal status management, including Rust implementation, PostgreSQL schema, and deployment configuration.

## Rust Implementation

### Core Service

```rust
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::PgPool;
use thiserror::Error;
use uuid::Uuid;

#[derive(Error, Debug)]
pub enum LegalError {
    #[error("Record not found: {0}")]
    RecordNotFound(String),

    #[error("Invalid transition: {from} -> {to}")]
    InvalidTransition { from: String, to: String },

    #[error("Document not found: {0}")]
    DocumentNotFound(String),

    #[error("Signature validation failed: {0}")]
    SignatureValidationFailed(String),

    #[error("Jurisdiction not recognized: {0}")]
    JurisdictionNotRecognized(String),

    #[error("Authorization denied: {0}")]
    AuthorizationDenied(String),

    #[error("Database error: {0}")]
    DatabaseError(#[from] sqlx::Error),
}

pub type Result<T> = std::result::Result<T, LegalError>;

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "legal_status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum LegalStatus {
    Living,
    LegallyDeceased,
    PreservedStatus,
    RevivalPending,
    LegallyRestored,
    PermanentDeath,
}

impl LegalStatus {
    pub fn valid_transitions(&self) -> Vec<LegalStatus> {
        match self {
            Self::Living => vec![Self::LegallyDeceased],
            Self::LegallyDeceased => vec![Self::PreservedStatus],
            Self::PreservedStatus => vec![Self::RevivalPending, Self::PermanentDeath],
            Self::RevivalPending => vec![
                Self::LegallyRestored,
                Self::PreservedStatus,
                Self::PermanentDeath
            ],
            Self::LegallyRestored => vec![],
            Self::PermanentDeath => vec![],
        }
    }

    pub fn can_transition_to(&self, target: &LegalStatus) -> bool {
        self.valid_transitions().contains(target)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegalStatusRecord {
    pub record_id: String,
    pub subject_id: String,
    pub identity_id: String,
    pub current_status: LegalStatus,
    pub primary_jurisdiction: String,
    pub death_certificate_id: Option<String>,
    pub preservation_declaration_id: Option<String>,
    pub restoration_certificate_id: Option<String>,
    pub guardianship_id: Option<String>,
    pub created_at: DateTime<Utc>,
    pub last_updated: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusTransition {
    pub transition_id: String,
    pub record_id: String,
    pub from_status: LegalStatus,
    pub to_status: LegalStatus,
    pub transition_date: DateTime<Utc>,
    pub authority_id: String,
    pub document_ref: String,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "document_type", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DocumentType {
    DeathCertificate,
    PreservationDeclaration,
    RestorationCertificate,
    CourtOrder,
    GuardianshipOrder,
    TrustDocument,
    PowerOfAttorney,
    IdentityVerification,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "document_status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DocumentStatus {
    Draft,
    PendingSignature,
    Active,
    Superseded,
    Revoked,
    Expired,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegalDocument {
    pub document_id: String,
    pub document_type: DocumentType,
    pub title: String,
    pub content_hash: String,
    pub status: DocumentStatus,
    pub jurisdiction: String,
    pub effective_date: DateTime<Utc>,
    pub expiration_date: Option<DateTime<Utc>>,
    pub notarized: bool,
    pub apostille: bool,
    pub created_at: DateTime<Utc>,
}
```

### Legal Status Service

```rust
pub struct LegalStatusService {
    pool: PgPool,
    document_service: DocumentService,
    jurisdiction_service: JurisdictionService,
}

impl LegalStatusService {
    pub fn new(
        pool: PgPool,
        document_service: DocumentService,
        jurisdiction_service: JurisdictionService,
    ) -> Self {
        Self {
            pool,
            document_service,
            jurisdiction_service,
        }
    }

    pub async fn create_record(
        &self,
        request: CreateRecordRequest,
    ) -> Result<LegalStatusRecord> {
        let year = Utc::now().format("%Y");
        let sequence: i32 = sqlx::query_scalar(
            "SELECT COALESCE(MAX(sequence_number), 0) + 1
             FROM legal_status_records
             WHERE EXTRACT(YEAR FROM created_at) = EXTRACT(YEAR FROM NOW())"
        )
        .fetch_one(&self.pool)
        .await?;

        let record_id = format!("LEGAL-{}-{:06}", year, sequence);

        let record = sqlx::query_as::<_, LegalStatusRecord>(
            r#"
            INSERT INTO legal_status_records (
                record_id, subject_id, identity_id, current_status,
                primary_jurisdiction, sequence_number
            ) VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING *
            "#
        )
        .bind(&record_id)
        .bind(&request.subject_id)
        .bind(&request.identity_id)
        .bind(LegalStatus::Living)
        .bind(&request.primary_jurisdiction)
        .bind(sequence)
        .fetch_one(&self.pool)
        .await?;

        Ok(record)
    }

    pub async fn get_record(&self, record_id: &str) -> Result<LegalStatusRecord> {
        sqlx::query_as::<_, LegalStatusRecord>(
            "SELECT * FROM legal_status_records WHERE record_id = $1"
        )
        .bind(record_id)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| LegalError::RecordNotFound(record_id.to_string()))
    }

    pub async fn transition_status(
        &self,
        record_id: &str,
        request: TransitionRequest,
    ) -> Result<StatusTransition> {
        let record = self.get_record(record_id).await?;

        if !record.current_status.can_transition_to(&request.to_status) {
            return Err(LegalError::InvalidTransition {
                from: format!("{:?}", record.current_status),
                to: format!("{:?}", request.to_status),
            });
        }

        // Verify document exists
        self.document_service
            .verify_document(&request.document_ref)
            .await?;

        let transition_id = format!(
            "TRANS-{}-{:03}",
            record_id,
            self.count_transitions(record_id).await? + 1
        );

        let transition = sqlx::query_as::<_, StatusTransition>(
            r#"
            INSERT INTO status_transitions (
                transition_id, record_id, from_status, to_status,
                authority_id, document_ref, reason
            ) VALUES ($1, $2, $3, $4, $5, $6, $7)
            RETURNING *
            "#
        )
        .bind(&transition_id)
        .bind(record_id)
        .bind(&record.current_status)
        .bind(&request.to_status)
        .bind(&request.authority_id)
        .bind(&request.document_ref)
        .bind(&request.reason)
        .fetch_one(&self.pool)
        .await?;

        // Update record status
        sqlx::query(
            "UPDATE legal_status_records SET current_status = $2, last_updated = NOW() WHERE record_id = $1"
        )
        .bind(record_id)
        .bind(&request.to_status)
        .execute(&self.pool)
        .await?;

        Ok(transition)
    }

    async fn count_transitions(&self, record_id: &str) -> Result<i64> {
        let count: (i64,) = sqlx::query_as(
            "SELECT COUNT(*) FROM status_transitions WHERE record_id = $1"
        )
        .bind(record_id)
        .fetch_one(&self.pool)
        .await?;

        Ok(count.0)
    }

    pub async fn get_transition_history(
        &self,
        record_id: &str,
    ) -> Result<Vec<StatusTransition>> {
        let transitions = sqlx::query_as::<_, StatusTransition>(
            "SELECT * FROM status_transitions WHERE record_id = $1 ORDER BY transition_date"
        )
        .bind(record_id)
        .fetch_all(&self.pool)
        .await?;

        Ok(transitions)
    }
}

#[derive(Debug, Deserialize)]
pub struct CreateRecordRequest {
    pub subject_id: String,
    pub identity_id: String,
    pub primary_jurisdiction: String,
}

#[derive(Debug, Deserialize)]
pub struct TransitionRequest {
    pub to_status: LegalStatus,
    pub authority_id: String,
    pub document_ref: String,
    pub reason: String,
}
```

### Document Service

```rust
use sha2::{Sha256, Digest};

pub struct DocumentService {
    pool: PgPool,
    storage: DocumentStorage,
}

impl DocumentService {
    pub async fn create_document(
        &self,
        request: CreateDocumentRequest,
    ) -> Result<LegalDocument> {
        let content_hash = self.hash_content(&request.content);

        let doc_type_prefix = match request.document_type {
            DocumentType::DeathCertificate => "DEATH",
            DocumentType::PreservationDeclaration => "PRES",
            DocumentType::RestorationCertificate => "REST",
            DocumentType::CourtOrder => "COURT",
            DocumentType::GuardianshipOrder => "GUARD",
            _ => "DOC",
        };

        let sequence: i32 = sqlx::query_scalar(
            "SELECT COALESCE(MAX(sequence_number), 0) + 1 FROM legal_documents"
        )
        .fetch_one(&self.pool)
        .await?;

        let document_id = format!("DOC-{}-{:06}", doc_type_prefix, sequence);

        // Store content
        self.storage.store(&document_id, &request.content).await?;

        let document = sqlx::query_as::<_, LegalDocument>(
            r#"
            INSERT INTO legal_documents (
                document_id, document_type, title, content_hash,
                status, jurisdiction, effective_date, sequence_number
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            RETURNING *
            "#
        )
        .bind(&document_id)
        .bind(&request.document_type)
        .bind(&request.title)
        .bind(&content_hash)
        .bind(DocumentStatus::Draft)
        .bind(&request.jurisdiction)
        .bind(&request.effective_date)
        .bind(sequence)
        .fetch_one(&self.pool)
        .await?;

        Ok(document)
    }

    pub async fn add_signature(
        &self,
        document_id: &str,
        signature: SignatureRequest,
    ) -> Result<()> {
        let document = self.get_document(document_id).await?;

        if !matches!(document.status, DocumentStatus::Draft | DocumentStatus::PendingSignature) {
            return Err(LegalError::SignatureValidationFailed(
                "Document not in signable state".to_string()
            ));
        }

        sqlx::query(
            r#"
            INSERT INTO document_signatures (
                document_id, signer_id, signer_name, signer_role,
                signature_type, digital_signature
            ) VALUES ($1, $2, $3, $4, $5, $6)
            "#
        )
        .bind(document_id)
        .bind(&signature.signer_id)
        .bind(&signature.signer_name)
        .bind(&signature.signer_role)
        .bind(&signature.signature_type)
        .bind(&signature.digital_signature)
        .execute(&self.pool)
        .await?;

        // Update document status
        sqlx::query(
            "UPDATE legal_documents SET status = 'PENDING_SIGNATURE' WHERE document_id = $1"
        )
        .bind(document_id)
        .execute(&self.pool)
        .await?;

        Ok(())
    }

    pub async fn finalize_document(&self, document_id: &str) -> Result<LegalDocument> {
        let missing = self.check_required_signatures(document_id).await?;

        if !missing.is_empty() {
            return Err(LegalError::SignatureValidationFailed(
                format!("Missing signatures: {:?}", missing)
            ));
        }

        sqlx::query(
            "UPDATE legal_documents SET status = 'ACTIVE' WHERE document_id = $1"
        )
        .bind(document_id)
        .execute(&self.pool)
        .await?;

        self.get_document(document_id).await
    }

    pub async fn verify_document(&self, document_id: &str) -> Result<bool> {
        let document = self.get_document(document_id).await?;

        if document.status != DocumentStatus::Active {
            return Ok(false);
        }

        if let Some(exp) = document.expiration_date {
            if Utc::now() > exp {
                // Mark as expired
                sqlx::query(
                    "UPDATE legal_documents SET status = 'EXPIRED' WHERE document_id = $1"
                )
                .bind(document_id)
                .execute(&self.pool)
                .await?;
                return Ok(false);
            }
        }

        Ok(true)
    }

    async fn get_document(&self, document_id: &str) -> Result<LegalDocument> {
        sqlx::query_as::<_, LegalDocument>(
            "SELECT * FROM legal_documents WHERE document_id = $1"
        )
        .bind(document_id)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| LegalError::DocumentNotFound(document_id.to_string()))
    }

    async fn check_required_signatures(&self, document_id: &str) -> Result<Vec<String>> {
        // Implementation to check required signatures based on document type
        Ok(vec![])
    }

    fn hash_content(&self, content: &[u8]) -> String {
        let mut hasher = Sha256::new();
        hasher.update(content);
        format!("{:x}", hasher.finalize())
    }
}
```

### Restoration Service

```rust
#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "restoration_stage", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RestorationStage {
    Initiated,
    IdentityVerification,
    MedicalClearance,
    LegalReview,
    CourtPetition,
    CourtHearing,
    RestorationGranted,
    RightsRestoration,
    Completed,
    Denied,
}

pub struct RestorationService {
    pool: PgPool,
    legal_status_service: LegalStatusService,
    identity_client: IdentityServiceClient,
}

impl RestorationService {
    pub async fn initiate_restoration(
        &self,
        subject_id: &str,
        revival_procedure_id: &str,
    ) -> Result<RestorationCase> {
        let year = Utc::now().format("%Y");
        let sequence: i32 = sqlx::query_scalar(
            "SELECT COALESCE(MAX(sequence_number), 0) + 1 FROM restoration_cases"
        )
        .fetch_one(&self.pool)
        .await?;

        let case_id = format!("RESTORE-{}-{:04}", year, sequence);

        let case = sqlx::query_as::<_, RestorationCase>(
            r#"
            INSERT INTO restoration_cases (
                case_id, subject_id, revival_procedure_id, current_stage, sequence_number
            ) VALUES ($1, $2, $3, $4, $5)
            RETURNING *
            "#
        )
        .bind(&case_id)
        .bind(subject_id)
        .bind(revival_procedure_id)
        .bind(RestorationStage::Initiated)
        .bind(sequence)
        .fetch_one(&self.pool)
        .await?;

        // Generate initial tasks
        self.generate_stage_tasks(&case_id, RestorationStage::IdentityVerification).await?;

        Ok(case)
    }

    pub async fn complete_task(
        &self,
        case_id: &str,
        task_id: &str,
        completed_by: &str,
        notes: &str,
    ) -> Result<()> {
        sqlx::query(
            r#"
            UPDATE restoration_tasks
            SET completed = true, completion_date = NOW(), completed_by = $3, notes = $4
            WHERE case_id = $1 AND task_id = $2
            "#
        )
        .bind(case_id)
        .bind(task_id)
        .bind(completed_by)
        .bind(notes)
        .execute(&self.pool)
        .await?;

        // Check if stage is complete
        self.check_stage_completion(case_id).await?;

        Ok(())
    }

    pub async fn record_court_decision(
        &self,
        case_id: &str,
        decision: &str,
        judge_name: &str,
        order_text: &str,
    ) -> Result<RestorationCase> {
        let new_stage = match decision {
            "GRANTED" => RestorationStage::RestorationGranted,
            "DENIED" => RestorationStage::Denied,
            _ => return Err(LegalError::InvalidTransition {
                from: "COURT_HEARING".to_string(),
                to: decision.to_string(),
            }),
        };

        sqlx::query(
            r#"
            UPDATE restoration_cases
            SET current_stage = $2, decision = $3, judge_assigned = $4, decision_text = $5
            WHERE case_id = $1
            "#
        )
        .bind(case_id)
        .bind(&new_stage)
        .bind(decision)
        .bind(judge_name)
        .bind(order_text)
        .execute(&self.pool)
        .await?;

        if decision == "GRANTED" {
            self.generate_stage_tasks(case_id, RestorationStage::RightsRestoration).await?;
        }

        self.get_case(case_id).await
    }

    async fn get_case(&self, case_id: &str) -> Result<RestorationCase> {
        sqlx::query_as::<_, RestorationCase>(
            "SELECT * FROM restoration_cases WHERE case_id = $1"
        )
        .bind(case_id)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| LegalError::RecordNotFound(case_id.to_string()))
    }

    async fn generate_stage_tasks(&self, case_id: &str, stage: RestorationStage) -> Result<()> {
        let tasks = match stage {
            RestorationStage::IdentityVerification => vec![
                "Verify biometric identity (DNA)",
                "Verify biometric identity (fingerprint)",
                "Cross-reference CRYO-IDENTITY records",
                "Obtain witness statements",
            ],
            RestorationStage::MedicalClearance => vec![
                "Complete medical examination",
                "Assess mental capacity",
                "Document functional status",
                "Obtain medical clearance certificate",
            ],
            RestorationStage::RightsRestoration => vec![
                "Obtain new/restored identification",
                "Restore social security status",
                "Restore voting rights",
                "Restore professional licenses",
                "Update family records",
            ],
            _ => vec![],
        };

        for (i, description) in tasks.iter().enumerate() {
            let task_id = format!("{}-{}-{:02}", case_id, &format!("{:?}", stage)[..4], i + 1);
            sqlx::query(
                r#"
                INSERT INTO restoration_tasks (task_id, case_id, stage, description)
                VALUES ($1, $2, $3, $4)
                "#
            )
            .bind(&task_id)
            .bind(case_id)
            .bind(&stage)
            .bind(description)
            .execute(&self.pool)
            .await?;
        }

        Ok(())
    }

    async fn check_stage_completion(&self, case_id: &str) -> Result<()> {
        // Check if all tasks for current stage are complete and advance if so
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestorationCase {
    pub case_id: String,
    pub subject_id: String,
    pub revival_procedure_id: String,
    pub current_stage: RestorationStage,
    pub court_case_number: Option<String>,
    pub judge_assigned: Option<String>,
    pub decision: Option<String>,
    pub created_at: DateTime<Utc>,
    pub last_updated: DateTime<Utc>,
}
```

## PostgreSQL Schema

```sql
-- Enable extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Enum types
CREATE TYPE legal_status AS ENUM (
    'LIVING', 'LEGALLY_DECEASED', 'PRESERVED_STATUS',
    'REVIVAL_PENDING', 'LEGALLY_RESTORED', 'PERMANENT_DEATH'
);

CREATE TYPE document_type AS ENUM (
    'DEATH_CERTIFICATE', 'PRESERVATION_DECLARATION', 'RESTORATION_CERTIFICATE',
    'COURT_ORDER', 'GUARDIANSHIP_ORDER', 'TRUST_DOCUMENT',
    'POWER_OF_ATTORNEY', 'IDENTITY_VERIFICATION'
);

CREATE TYPE document_status AS ENUM (
    'DRAFT', 'PENDING_SIGNATURE', 'ACTIVE', 'SUPERSEDED', 'REVOKED', 'EXPIRED'
);

CREATE TYPE restoration_stage AS ENUM (
    'INITIATED', 'IDENTITY_VERIFICATION', 'MEDICAL_CLEARANCE', 'LEGAL_REVIEW',
    'COURT_PETITION', 'COURT_HEARING', 'RESTORATION_GRANTED',
    'RIGHTS_RESTORATION', 'COMPLETED', 'DENIED'
);

CREATE TYPE guardianship_status AS ENUM (
    'ACTIVE', 'SUSPENDED', 'TERMINATED', 'TRANSFERRED'
);

CREATE TYPE recognition_status AS ENUM (
    'FULL_RECOGNITION', 'PARTIAL_RECOGNITION', 'PENDING_RECOGNITION',
    'NOT_RECOGNIZED', 'TREATY_BASED'
);

-- Legal status records
CREATE TABLE legal_status_records (
    record_id VARCHAR(20) PRIMARY KEY,
    subject_id VARCHAR(50) NOT NULL,
    identity_id VARCHAR(50) NOT NULL,
    current_status legal_status NOT NULL DEFAULT 'LIVING',
    primary_jurisdiction VARCHAR(10) NOT NULL,
    death_certificate_id VARCHAR(30),
    preservation_declaration_id VARCHAR(30),
    restoration_certificate_id VARCHAR(30),
    guardianship_id VARCHAR(30),
    sequence_number INTEGER NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_updated TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_record_id CHECK (record_id ~ '^LEGAL-[0-9]{4}-[0-9]{6}$')
);

CREATE INDEX idx_legal_records_subject ON legal_status_records(subject_id);
CREATE INDEX idx_legal_records_status ON legal_status_records(current_status);
CREATE INDEX idx_legal_records_jurisdiction ON legal_status_records(primary_jurisdiction);

-- Status transitions
CREATE TABLE status_transitions (
    transition_id VARCHAR(30) PRIMARY KEY,
    record_id VARCHAR(20) NOT NULL REFERENCES legal_status_records(record_id),
    from_status legal_status NOT NULL,
    to_status legal_status NOT NULL,
    transition_date TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    authority_id VARCHAR(50) NOT NULL,
    document_ref VARCHAR(30) NOT NULL,
    reason TEXT NOT NULL,

    CONSTRAINT valid_transition CHECK (from_status != to_status)
);

CREATE INDEX idx_transitions_record ON status_transitions(record_id);

-- Legal documents
CREATE TABLE legal_documents (
    document_id VARCHAR(30) PRIMARY KEY,
    document_type document_type NOT NULL,
    title VARCHAR(255) NOT NULL,
    content_hash VARCHAR(64) NOT NULL,
    status document_status NOT NULL DEFAULT 'DRAFT',
    jurisdiction VARCHAR(10) NOT NULL,
    effective_date TIMESTAMPTZ NOT NULL,
    expiration_date TIMESTAMPTZ,
    notarized BOOLEAN DEFAULT FALSE,
    apostille BOOLEAN DEFAULT FALSE,
    sequence_number INTEGER NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_updated TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_documents_type ON legal_documents(document_type);
CREATE INDEX idx_documents_status ON legal_documents(status);

-- Document signatures
CREATE TABLE document_signatures (
    signature_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(30) NOT NULL REFERENCES legal_documents(document_id),
    signer_id VARCHAR(50) NOT NULL,
    signer_name VARCHAR(100) NOT NULL,
    signer_role VARCHAR(50) NOT NULL,
    signature_type VARCHAR(20) NOT NULL,
    digital_signature TEXT,
    signature_date TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    verified BOOLEAN DEFAULT FALSE,
    verified_at TIMESTAMPTZ,
    verified_by VARCHAR(50)
);

CREATE INDEX idx_signatures_document ON document_signatures(document_id);

-- Notarization records
CREATE TABLE notarization_records (
    notarization_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(30) NOT NULL REFERENCES legal_documents(document_id),
    notary_id VARCHAR(50) NOT NULL,
    notary_name VARCHAR(100) NOT NULL,
    notary_jurisdiction VARCHAR(10) NOT NULL,
    notary_commission VARCHAR(50) NOT NULL,
    commission_expiration DATE NOT NULL,
    seal_number VARCHAR(50),
    notarization_date TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Apostille records
CREATE TABLE apostille_records (
    apostille_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(30) NOT NULL REFERENCES legal_documents(document_id),
    apostille_number VARCHAR(50) NOT NULL,
    issuing_country VARCHAR(10) NOT NULL,
    issuing_authority VARCHAR(100) NOT NULL,
    issue_date DATE NOT NULL,
    target_country VARCHAR(10)
);

-- Jurisdictions
CREATE TABLE jurisdictions (
    country_code VARCHAR(10) PRIMARY KEY,
    country_name VARCHAR(100) NOT NULL,
    region VARCHAR(100),
    legal_system VARCHAR(20) NOT NULL,
    treaty_member BOOLEAN DEFAULT FALSE,
    cryopreservation_legal BOOLEAN,
    recognition_requirements JSONB DEFAULT '[]',
    local_authorities JSONB DEFAULT '[]',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Bilateral agreements
CREATE TABLE bilateral_agreements (
    agreement_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    country_a VARCHAR(10) NOT NULL,
    country_b VARCHAR(10) NOT NULL,
    agreement_name VARCHAR(255) NOT NULL,
    effective_date DATE NOT NULL,
    expiration_date DATE,
    agreement_text TEXT,
    recognition_scope JSONB DEFAULT '{}',

    UNIQUE(country_a, country_b)
);

-- Jurisdiction recognitions for records
CREATE TABLE jurisdiction_recognitions (
    recognition_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    record_id VARCHAR(20) NOT NULL REFERENCES legal_status_records(record_id),
    country_code VARCHAR(10) NOT NULL,
    recognition_status recognition_status NOT NULL,
    local_status_equivalent VARCHAR(100),
    registration_id VARCHAR(50),
    registration_date DATE,
    verified BOOLEAN DEFAULT FALSE,
    verified_date DATE,
    documents JSONB DEFAULT '[]',

    UNIQUE(record_id, country_code)
);

CREATE INDEX idx_recognitions_record ON jurisdiction_recognitions(record_id);

-- Restoration cases
CREATE TABLE restoration_cases (
    case_id VARCHAR(20) PRIMARY KEY,
    subject_id VARCHAR(50) NOT NULL,
    revival_procedure_id VARCHAR(20) NOT NULL,
    current_stage restoration_stage NOT NULL DEFAULT 'INITIATED',
    court_case_number VARCHAR(50),
    judge_assigned VARCHAR(100),
    hearing_date DATE,
    decision VARCHAR(20),
    decision_text TEXT,
    sequence_number INTEGER NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_updated TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_case_id CHECK (case_id ~ '^RESTORE-[0-9]{4}-[0-9]{4}$')
);

CREATE INDEX idx_restoration_subject ON restoration_cases(subject_id);
CREATE INDEX idx_restoration_stage ON restoration_cases(current_stage);

-- Restoration tasks
CREATE TABLE restoration_tasks (
    task_id VARCHAR(30) PRIMARY KEY,
    case_id VARCHAR(20) NOT NULL REFERENCES restoration_cases(case_id),
    stage restoration_stage NOT NULL,
    description TEXT NOT NULL,
    assigned_to VARCHAR(50),
    due_date DATE,
    completed BOOLEAN DEFAULT FALSE,
    completion_date TIMESTAMPTZ,
    completed_by VARCHAR(50),
    notes TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_tasks_case ON restoration_tasks(case_id);
CREATE INDEX idx_tasks_stage ON restoration_tasks(case_id, stage);

-- Guardianships
CREATE TABLE guardianships (
    guardianship_id VARCHAR(30) PRIMARY KEY,
    subject_id VARCHAR(50) NOT NULL,
    record_id VARCHAR(20) REFERENCES legal_status_records(record_id),
    status guardianship_status NOT NULL DEFAULT 'ACTIVE',
    scope JSONB NOT NULL DEFAULT '{}',
    appointment_authority VARCHAR(100) NOT NULL,
    court_order_ref VARCHAR(50),
    review_date DATE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    terminated_at TIMESTAMPTZ,
    termination_reason TEXT
);

CREATE INDEX idx_guardianship_subject ON guardianships(subject_id);
CREATE INDEX idx_guardianship_status ON guardianships(status);

-- Guardians
CREATE TABLE guardians (
    guardian_id VARCHAR(30) PRIMARY KEY,
    guardianship_id VARCHAR(30) NOT NULL REFERENCES guardianships(guardianship_id),
    name VARCHAR(100) NOT NULL,
    relationship VARCHAR(50) NOT NULL,
    priority INTEGER NOT NULL DEFAULT 1,
    contact_info JSONB DEFAULT '{}',
    appointment_date DATE NOT NULL,
    active BOOLEAN DEFAULT TRUE,
    bond_amount DECIMAL(15, 2),
    bond_provider VARCHAR(100)
);

CREATE INDEX idx_guardians_guardianship ON guardians(guardianship_id);

-- Audit log
CREATE TABLE legal_audit_log (
    log_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    record_id VARCHAR(20),
    document_id VARCHAR(30),
    action VARCHAR(50) NOT NULL,
    entity_type VARCHAR(50) NOT NULL,
    entity_id VARCHAR(50),
    actor_id VARCHAR(50) NOT NULL,
    actor_role VARCHAR(50),
    old_values JSONB,
    new_values JSONB,
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    ip_address INET,
    user_agent TEXT
);

CREATE INDEX idx_audit_record ON legal_audit_log(record_id);
CREATE INDEX idx_audit_timestamp ON legal_audit_log(timestamp DESC);

-- Views
CREATE VIEW active_preservation_records AS
SELECT r.*, g.guardianship_id as active_guardianship
FROM legal_status_records r
LEFT JOIN guardianships g ON r.guardianship_id = g.guardianship_id AND g.status = 'ACTIVE'
WHERE r.current_status = 'PRESERVED_STATUS';

CREATE VIEW pending_restorations AS
SELECT c.*, r.subject_id, r.current_status as legal_status
FROM restoration_cases c
JOIN legal_status_records r ON c.subject_id = r.subject_id
WHERE c.current_stage NOT IN ('COMPLETED', 'DENIED');

-- Functions
CREATE OR REPLACE FUNCTION update_legal_record_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.last_updated = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trigger_update_legal_record_timestamp
    BEFORE UPDATE ON legal_status_records
    FOR EACH ROW
    EXECUTE FUNCTION update_legal_record_timestamp();
```

## Docker Configuration

```yaml
version: '3.8'

services:
  legal-api:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://legal:${DB_PASSWORD}@db:5432/cryo_legal
      - REDIS_URL=redis://redis:6379
      - IDENTITY_SERVICE_URL=http://identity-service:8080
      - REVIVAL_SERVICE_URL=http://revival-service:8080
      - JWT_SECRET=${JWT_SECRET}
      - RUST_LOG=info
    depends_on:
      - db
      - redis
    networks:
      - legal-network

  db:
    image: postgres:15-alpine
    environment:
      - POSTGRES_DB=cryo_legal
      - POSTGRES_USER=legal
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init.sql:/docker-entrypoint-initdb.d/init.sql
    networks:
      - legal-network

  redis:
    image: redis:7-alpine
    command: redis-server --appendonly yes
    volumes:
      - redis_data:/data
    networks:
      - legal-network

volumes:
  postgres_data:
  redis_data:

networks:
  legal-network:
    driver: bridge
```

---

*WIA Technical Committee - Cryopreservation Working Group*
