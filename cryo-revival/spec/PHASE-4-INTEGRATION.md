# CRYO-REVIVAL Phase 4: Integration Specification

## Overview

This document defines the system integration architecture for revival procedure management, including Rust implementation, PostgreSQL schema, and deployment configuration.

## Rust Implementation

### Core Service

```rust
// src/lib.rs
use chrono::{DateTime, Utc, Duration};
use serde::{Deserialize, Serialize};
use sqlx::PgPool;
use thiserror::Error;
use uuid::Uuid;

#[derive(Error, Debug)]
pub enum RevivalError {
    #[error("Procedure not found: {0}")]
    ProcedureNotFound(String),

    #[error("Invalid stage transition: {from} -> {to}")]
    InvalidTransition { from: String, to: String },

    #[error("Transition criteria not met: {0}")]
    CriteriaNotMet(String),

    #[error("Authorization required: {0}")]
    AuthorizationRequired(String),

    #[error("Consent verification failed: {0}")]
    ConsentVerificationFailed(String),

    #[error("Database error: {0}")]
    DatabaseError(#[from] sqlx::Error),

    #[error("External service error: {0}")]
    ExternalServiceError(String),
}

pub type Result<T> = std::result::Result<T, RevivalError>;

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "revival_status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RevivalStatus {
    Scheduled,
    Preparing,
    InProgress,
    Paused,
    Completed,
    Failed,
    Aborted,
    RePreserved,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type, PartialEq, Eq, PartialOrd, Ord)]
#[sqlx(type_name = "revival_stage", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RevivalStage {
    PreRevival,
    Warming,
    CryoprotectantRemoval,
    Rehydration,
    CardiovascularRestart,
    RespiratoryActivation,
    NeurologicalRestoration,
    Stabilization,
    PostRevivalCare,
}

impl RevivalStage {
    pub fn order(&self) -> u8 {
        match self {
            Self::PreRevival => 0,
            Self::Warming => 1,
            Self::CryoprotectantRemoval => 2,
            Self::Rehydration => 3,
            Self::CardiovascularRestart => 4,
            Self::RespiratoryActivation => 5,
            Self::NeurologicalRestoration => 6,
            Self::Stabilization => 7,
            Self::PostRevivalCare => 8,
        }
    }

    pub fn next(&self) -> Option<Self> {
        match self {
            Self::PreRevival => Some(Self::Warming),
            Self::Warming => Some(Self::CryoprotectantRemoval),
            Self::CryoprotectantRemoval => Some(Self::Rehydration),
            Self::Rehydration => Some(Self::CardiovascularRestart),
            Self::CardiovascularRestart => Some(Self::RespiratoryActivation),
            Self::RespiratoryActivation => Some(Self::NeurologicalRestoration),
            Self::NeurologicalRestoration => Some(Self::Stabilization),
            Self::Stabilization => Some(Self::PostRevivalCare),
            Self::PostRevivalCare => None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RevivalProcedure {
    pub procedure_id: String,
    pub subject_id: String,
    pub identity_id: String,
    pub consent_id: String,
    pub status: RevivalStatus,
    pub current_stage: RevivalStage,
    pub facility_id: String,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub last_updated: DateTime<Utc>,
    pub created_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StageRecord {
    pub record_id: Uuid,
    pub procedure_id: String,
    pub stage: RevivalStage,
    pub stage_number: i32,
    pub status: StageStatus,
    pub started_at: DateTime<Utc>,
    pub completed_at: Option<DateTime<Utc>>,
    pub parameters: serde_json::Value,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "stage_status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum StageStatus {
    Pending,
    InProgress,
    Completed,
    Failed,
    Skipped,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VitalSigns {
    pub reading_id: Uuid,
    pub procedure_id: String,
    pub timestamp: DateTime<Utc>,
    pub temperature: Option<f64>,
    pub heart_rate: Option<i32>,
    pub systolic_bp: Option<i32>,
    pub diastolic_bp: Option<i32>,
    pub respiratory_rate: Option<i32>,
    pub oxygen_saturation: Option<f64>,
    pub end_tidal_co2: Option<f64>,
    pub cvp: Option<i32>,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alert {
    pub alert_id: String,
    pub procedure_id: String,
    pub timestamp: DateTime<Utc>,
    pub severity: AlertSeverity,
    pub category: String,
    pub source: String,
    pub message: String,
    pub acknowledged: bool,
    pub acknowledged_by: Option<String>,
    pub acknowledged_at: Option<DateTime<Utc>>,
    pub resolved: bool,
    pub resolved_by: Option<String>,
    pub resolution: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "alert_severity", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
    Emergency,
}
```

### Procedure Service

```rust
// src/services/procedure.rs
use crate::*;
use sqlx::PgPool;

pub struct ProcedureService {
    pool: PgPool,
    consent_client: ConsentServiceClient,
    identity_client: IdentityServiceClient,
}

impl ProcedureService {
    pub fn new(
        pool: PgPool,
        consent_client: ConsentServiceClient,
        identity_client: IdentityServiceClient,
    ) -> Self {
        Self {
            pool,
            consent_client,
            identity_client,
        }
    }

    pub async fn create_procedure(
        &self,
        request: CreateProcedureRequest,
    ) -> Result<RevivalProcedure> {
        // Verify consent
        let consent = self.consent_client
            .verify_consent(&request.consent_id)
            .await
            .map_err(|e| RevivalError::ConsentVerificationFailed(e.to_string()))?;

        if consent.status != "ACTIVE" {
            return Err(RevivalError::ConsentVerificationFailed(
                "Consent is not active".to_string()
            ));
        }

        // Check revival conditions
        let conditions_met = self.consent_client
            .check_revival_conditions(&request.consent_id)
            .await
            .map_err(|e| RevivalError::ExternalServiceError(e.to_string()))?;

        if !conditions_met {
            return Err(RevivalError::ConsentVerificationFailed(
                "Revival conditions not met".to_string()
            ));
        }

        // Verify identity
        let _identity = self.identity_client
            .verify_identity(&request.identity_id)
            .await
            .map_err(|e| RevivalError::ExternalServiceError(e.to_string()))?;

        // Generate procedure ID
        let year = Utc::now().format("%Y");
        let sequence: i32 = sqlx::query_scalar(
            "SELECT COALESCE(MAX(sequence_number), 0) + 1
             FROM revival_procedures
             WHERE EXTRACT(YEAR FROM created_at) = EXTRACT(YEAR FROM NOW())"
        )
        .fetch_one(&self.pool)
        .await?;

        let procedure_id = format!("REVIVAL-{}-{:03}", year, sequence);

        // Create procedure
        let procedure = sqlx::query_as::<_, RevivalProcedure>(
            r#"
            INSERT INTO revival_procedures (
                procedure_id, subject_id, identity_id, consent_id,
                status, current_stage, facility_id, sequence_number
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            RETURNING *
            "#
        )
        .bind(&procedure_id)
        .bind(&request.subject_id)
        .bind(&request.identity_id)
        .bind(&request.consent_id)
        .bind(RevivalStatus::Scheduled)
        .bind(RevivalStage::PreRevival)
        .bind(&request.facility_id)
        .bind(sequence)
        .fetch_one(&self.pool)
        .await?;

        // Create initial stage record
        self.create_stage_record(&procedure_id, RevivalStage::PreRevival, 1).await?;

        Ok(procedure)
    }

    pub async fn get_procedure(&self, procedure_id: &str) -> Result<RevivalProcedure> {
        sqlx::query_as::<_, RevivalProcedure>(
            "SELECT * FROM revival_procedures WHERE procedure_id = $1"
        )
        .bind(procedure_id)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| RevivalError::ProcedureNotFound(procedure_id.to_string()))
    }

    pub async fn update_status(
        &self,
        procedure_id: &str,
        status: RevivalStatus,
    ) -> Result<RevivalProcedure> {
        let procedure = sqlx::query_as::<_, RevivalProcedure>(
            r#"
            UPDATE revival_procedures
            SET status = $2, last_updated = NOW()
            WHERE procedure_id = $1
            RETURNING *
            "#
        )
        .bind(procedure_id)
        .bind(status)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| RevivalError::ProcedureNotFound(procedure_id.to_string()))?;

        Ok(procedure)
    }

    pub async fn start_procedure(&self, procedure_id: &str) -> Result<RevivalProcedure> {
        let procedure = self.get_procedure(procedure_id).await?;

        if procedure.status != RevivalStatus::Scheduled {
            return Err(RevivalError::InvalidTransition {
                from: format!("{:?}", procedure.status),
                to: "InProgress".to_string(),
            });
        }

        let updated = sqlx::query_as::<_, RevivalProcedure>(
            r#"
            UPDATE revival_procedures
            SET status = $2, started_at = NOW(), last_updated = NOW()
            WHERE procedure_id = $1
            RETURNING *
            "#
        )
        .bind(procedure_id)
        .bind(RevivalStatus::InProgress)
        .fetch_one(&self.pool)
        .await?;

        // Update initial stage to in_progress
        sqlx::query(
            r#"
            UPDATE stage_records
            SET status = 'IN_PROGRESS', started_at = NOW()
            WHERE procedure_id = $1 AND stage = $2
            "#
        )
        .bind(procedure_id)
        .bind(RevivalStage::PreRevival)
        .execute(&self.pool)
        .await?;

        Ok(updated)
    }

    async fn create_stage_record(
        &self,
        procedure_id: &str,
        stage: RevivalStage,
        stage_number: i32,
    ) -> Result<StageRecord> {
        let record = sqlx::query_as::<_, StageRecord>(
            r#"
            INSERT INTO stage_records (
                procedure_id, stage, stage_number, status, parameters
            ) VALUES ($1, $2, $3, 'PENDING', '{}')
            RETURNING *
            "#
        )
        .bind(procedure_id)
        .bind(stage)
        .bind(stage_number)
        .fetch_one(&self.pool)
        .await?;

        Ok(record)
    }
}

#[derive(Debug, Deserialize)]
pub struct CreateProcedureRequest {
    pub subject_id: String,
    pub identity_id: String,
    pub consent_id: String,
    pub facility_id: String,
    pub scheduled_date: DateTime<Utc>,
}
```

### Stage Transition Service

```rust
// src/services/stage.rs
use crate::*;
use sqlx::PgPool;

pub struct StageTransitionService {
    pool: PgPool,
    alert_service: AlertService,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitionCriteria {
    pub required_vitals: HashMap<String, (f64, f64)>,
    pub required_labs: HashMap<String, (f64, f64)>,
    pub minimum_duration: Duration,
    pub required_approvals: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitionRequest {
    pub current_vitals: VitalSigns,
    pub current_labs: Option<LabResults>,
    pub approvals: Vec<Approval>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Approval {
    pub role: String,
    pub member_id: String,
    pub approved_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitionResult {
    pub success: bool,
    pub previous_stage: RevivalStage,
    pub new_stage: Option<RevivalStage>,
    pub issues: Vec<String>,
    pub transitioned_at: Option<DateTime<Utc>>,
}

impl StageTransitionService {
    pub fn new(pool: PgPool, alert_service: AlertService) -> Self {
        Self { pool, alert_service }
    }

    pub async fn request_transition(
        &self,
        procedure_id: &str,
        request: TransitionRequest,
    ) -> Result<TransitionResult> {
        // Get current procedure state
        let procedure: RevivalProcedure = sqlx::query_as(
            "SELECT * FROM revival_procedures WHERE procedure_id = $1"
        )
        .bind(procedure_id)
        .fetch_optional(&self.pool)
        .await?
        .ok_or_else(|| RevivalError::ProcedureNotFound(procedure_id.to_string()))?;

        // Check if procedure is in progress
        if procedure.status != RevivalStatus::InProgress {
            return Err(RevivalError::InvalidTransition {
                from: format!("{:?}", procedure.status),
                to: "Next stage".to_string(),
            });
        }

        let current_stage = procedure.current_stage.clone();

        // Get next stage
        let next_stage = match current_stage.next() {
            Some(stage) => stage,
            None => {
                return Ok(TransitionResult {
                    success: true,
                    previous_stage: current_stage,
                    new_stage: None,
                    issues: vec!["Procedure complete - no more stages".to_string()],
                    transitioned_at: None,
                });
            }
        };

        // Check transition criteria
        let issues = self.check_criteria(
            &current_stage,
            &request,
            procedure_id,
        ).await?;

        if !issues.is_empty() {
            return Ok(TransitionResult {
                success: false,
                previous_stage: current_stage,
                new_stage: None,
                issues,
                transitioned_at: None,
            });
        }

        // Perform transition
        let now = Utc::now();

        // Complete current stage
        sqlx::query(
            r#"
            UPDATE stage_records
            SET status = 'COMPLETED', completed_at = $3
            WHERE procedure_id = $1 AND stage = $2
            "#
        )
        .bind(procedure_id)
        .bind(&current_stage)
        .bind(now)
        .execute(&self.pool)
        .await?;

        // Get next stage number
        let stage_number: i32 = sqlx::query_scalar(
            "SELECT COALESCE(MAX(stage_number), 0) + 1 FROM stage_records WHERE procedure_id = $1"
        )
        .bind(procedure_id)
        .fetch_one(&self.pool)
        .await?;

        // Create new stage record
        sqlx::query(
            r#"
            INSERT INTO stage_records (
                procedure_id, stage, stage_number, status, started_at, parameters
            ) VALUES ($1, $2, $3, 'IN_PROGRESS', $4, '{}')
            "#
        )
        .bind(procedure_id)
        .bind(&next_stage)
        .bind(stage_number)
        .bind(now)
        .execute(&self.pool)
        .await?;

        // Update procedure
        sqlx::query(
            r#"
            UPDATE revival_procedures
            SET current_stage = $2, last_updated = $3
            WHERE procedure_id = $1
            "#
        )
        .bind(procedure_id)
        .bind(&next_stage)
        .bind(now)
        .execute(&self.pool)
        .await?;

        // Generate transition alert
        self.alert_service.create_alert(
            procedure_id,
            AlertSeverity::Info,
            "STAGE_TRANSITION",
            &format!("Transitioned from {:?} to {:?}", current_stage, next_stage),
        ).await?;

        Ok(TransitionResult {
            success: true,
            previous_stage: current_stage,
            new_stage: Some(next_stage),
            issues: vec![],
            transitioned_at: Some(now),
        })
    }

    async fn check_criteria(
        &self,
        stage: &RevivalStage,
        request: &TransitionRequest,
        procedure_id: &str,
    ) -> Result<Vec<String>> {
        let mut issues = vec![];

        // Get stage-specific criteria
        let criteria = self.get_stage_criteria(stage);

        // Check vital signs
        for (vital, (min, max)) in &criteria.required_vitals {
            let value = match vital.as_str() {
                "temperature" => request.current_vitals.temperature,
                "heart_rate" => request.current_vitals.heart_rate.map(|v| v as f64),
                "systolic_bp" => request.current_vitals.systolic_bp.map(|v| v as f64),
                "diastolic_bp" => request.current_vitals.diastolic_bp.map(|v| v as f64),
                "oxygen_saturation" => request.current_vitals.oxygen_saturation,
                _ => None,
            };

            match value {
                Some(v) if v < *min || v > *max => {
                    issues.push(format!(
                        "{} out of range: {:.1} (required: {:.1}-{:.1})",
                        vital, v, min, max
                    ));
                }
                None => {
                    issues.push(format!("Missing vital: {}", vital));
                }
                _ => {}
            }
        }

        // Check minimum duration
        let stage_record: Option<StageRecord> = sqlx::query_as(
            "SELECT * FROM stage_records WHERE procedure_id = $1 AND stage = $2"
        )
        .bind(procedure_id)
        .bind(stage)
        .fetch_optional(&self.pool)
        .await?;

        if let Some(record) = stage_record {
            let elapsed = Utc::now() - record.started_at;
            if elapsed < criteria.minimum_duration {
                let remaining = criteria.minimum_duration - elapsed;
                issues.push(format!(
                    "Minimum duration not met: {} remaining",
                    format_duration(remaining)
                ));
            }
        }

        // Check approvals
        let provided_roles: HashSet<_> = request.approvals
            .iter()
            .map(|a| a.role.clone())
            .collect();

        for required_role in &criteria.required_approvals {
            if !provided_roles.contains(required_role) {
                issues.push(format!("Missing approval from: {}", required_role));
            }
        }

        Ok(issues)
    }

    fn get_stage_criteria(&self, stage: &RevivalStage) -> TransitionCriteria {
        match stage {
            RevivalStage::Warming => TransitionCriteria {
                required_vitals: [
                    ("temperature".to_string(), (35.0, 37.5)),
                ].into_iter().collect(),
                required_labs: HashMap::new(),
                minimum_duration: Duration::hours(4),
                required_approvals: vec!["CRYONICS_SPECIALIST".to_string()],
            },
            RevivalStage::CardiovascularRestart => TransitionCriteria {
                required_vitals: [
                    ("heart_rate".to_string(), (60.0, 100.0)),
                    ("systolic_bp".to_string(), (90.0, 140.0)),
                ].into_iter().collect(),
                required_labs: [
                    ("lactate".to_string(), (0.0, 2.0)),
                ].into_iter().collect(),
                minimum_duration: Duration::minutes(30),
                required_approvals: vec!["CARDIOVASCULAR_SURGEON".to_string()],
            },
            RevivalStage::NeurologicalRestoration => TransitionCriteria {
                required_vitals: [
                    ("oxygen_saturation".to_string(), (94.0, 100.0)),
                ].into_iter().collect(),
                required_labs: HashMap::new(),
                minimum_duration: Duration::hours(6),
                required_approvals: vec!["NEUROLOGIST".to_string()],
            },
            _ => TransitionCriteria {
                required_vitals: HashMap::new(),
                required_labs: HashMap::new(),
                minimum_duration: Duration::hours(1),
                required_approvals: vec![],
            },
        }
    }
}

fn format_duration(d: Duration) -> String {
    let hours = d.num_hours();
    let minutes = d.num_minutes() % 60;
    format!("{}h {}m", hours, minutes)
}
```

### Monitoring Service

```rust
// src/services/monitoring.rs
use crate::*;
use sqlx::PgPool;
use tokio::sync::broadcast;

pub struct MonitoringService {
    pool: PgPool,
    alert_service: AlertService,
    event_tx: broadcast::Sender<MonitoringEvent>,
}

#[derive(Debug, Clone, Serialize)]
pub enum MonitoringEvent {
    VitalsRecorded { procedure_id: String, vitals: VitalSigns },
    AlertGenerated { procedure_id: String, alert: Alert },
    ThresholdExceeded { procedure_id: String, parameter: String, value: f64 },
}

#[derive(Debug, Clone)]
pub struct AlertRule {
    pub rule_id: String,
    pub parameter: String,
    pub min_threshold: Option<f64>,
    pub max_threshold: Option<f64>,
    pub severity: AlertSeverity,
    pub message_template: String,
}

impl MonitoringService {
    pub fn new(
        pool: PgPool,
        alert_service: AlertService,
    ) -> (Self, broadcast::Receiver<MonitoringEvent>) {
        let (event_tx, event_rx) = broadcast::channel(1000);
        (
            Self { pool, alert_service, event_tx },
            event_rx,
        )
    }

    pub async fn record_vitals(
        &self,
        procedure_id: &str,
        vitals: RecordVitalsRequest,
    ) -> Result<VitalsRecordResult> {
        // Insert vital signs
        let recorded: VitalSigns = sqlx::query_as(
            r#"
            INSERT INTO vital_signs (
                procedure_id, timestamp, temperature, heart_rate,
                systolic_bp, diastolic_bp, respiratory_rate,
                oxygen_saturation, end_tidal_co2, cvp, source
            ) VALUES ($1, NOW(), $2, $3, $4, $5, $6, $7, $8, $9, $10)
            RETURNING *
            "#
        )
        .bind(procedure_id)
        .bind(vitals.temperature)
        .bind(vitals.heart_rate)
        .bind(vitals.systolic_bp)
        .bind(vitals.diastolic_bp)
        .bind(vitals.respiratory_rate)
        .bind(vitals.oxygen_saturation)
        .bind(vitals.end_tidal_co2)
        .bind(vitals.cvp)
        .bind(&vitals.source)
        .fetch_one(&self.pool)
        .await?;

        // Check alert rules
        let alerts = self.check_alert_rules(procedure_id, &recorded).await?;

        // Broadcast event
        let _ = self.event_tx.send(MonitoringEvent::VitalsRecorded {
            procedure_id: procedure_id.to_string(),
            vitals: recorded.clone(),
        });

        Ok(VitalsRecordResult {
            recorded: true,
            reading_id: recorded.reading_id,
            alerts,
        })
    }

    async fn check_alert_rules(
        &self,
        procedure_id: &str,
        vitals: &VitalSigns,
    ) -> Result<Vec<Alert>> {
        let rules = self.get_default_rules();
        let mut alerts = vec![];

        for rule in rules {
            let value = match rule.parameter.as_str() {
                "heart_rate" => vitals.heart_rate.map(|v| v as f64),
                "systolic_bp" => vitals.systolic_bp.map(|v| v as f64),
                "diastolic_bp" => vitals.diastolic_bp.map(|v| v as f64),
                "temperature" => vitals.temperature,
                "oxygen_saturation" => vitals.oxygen_saturation,
                "respiratory_rate" => vitals.respiratory_rate.map(|v| v as f64),
                _ => None,
            };

            if let Some(v) = value {
                let triggered = match (rule.min_threshold, rule.max_threshold) {
                    (Some(min), _) if v < min => true,
                    (_, Some(max)) if v > max => true,
                    _ => false,
                };

                if triggered {
                    let alert = self.alert_service.create_alert(
                        procedure_id,
                        rule.severity.clone(),
                        &rule.parameter,
                        &rule.message_template.replace("{value}", &format!("{:.1}", v)),
                    ).await?;

                    alerts.push(alert);

                    let _ = self.event_tx.send(MonitoringEvent::ThresholdExceeded {
                        procedure_id: procedure_id.to_string(),
                        parameter: rule.parameter.clone(),
                        value: v,
                    });
                }
            }
        }

        Ok(alerts)
    }

    fn get_default_rules(&self) -> Vec<AlertRule> {
        vec![
            AlertRule {
                rule_id: "HR_LOW".to_string(),
                parameter: "heart_rate".to_string(),
                min_threshold: Some(40.0),
                max_threshold: None,
                severity: AlertSeverity::Critical,
                message_template: "Heart rate critically low: {value} bpm".to_string(),
            },
            AlertRule {
                rule_id: "HR_HIGH".to_string(),
                parameter: "heart_rate".to_string(),
                min_threshold: None,
                max_threshold: Some(150.0),
                severity: AlertSeverity::Warning,
                message_template: "Heart rate elevated: {value} bpm".to_string(),
            },
            AlertRule {
                rule_id: "BP_LOW".to_string(),
                parameter: "systolic_bp".to_string(),
                min_threshold: Some(80.0),
                max_threshold: None,
                severity: AlertSeverity::Critical,
                message_template: "Systolic BP critically low: {value} mmHg".to_string(),
            },
            AlertRule {
                rule_id: "SPO2_LOW".to_string(),
                parameter: "oxygen_saturation".to_string(),
                min_threshold: Some(90.0),
                max_threshold: None,
                severity: AlertSeverity::Critical,
                message_template: "SpO2 low: {value}%".to_string(),
            },
            AlertRule {
                rule_id: "TEMP_HIGH".to_string(),
                parameter: "temperature".to_string(),
                min_threshold: None,
                max_threshold: Some(39.0),
                severity: AlertSeverity::Warning,
                message_template: "Temperature elevated: {value}C".to_string(),
            },
        ]
    }

    pub async fn get_vitals_history(
        &self,
        procedure_id: &str,
        from: Option<DateTime<Utc>>,
        to: Option<DateTime<Utc>>,
        limit: i64,
    ) -> Result<Vec<VitalSigns>> {
        let vitals = sqlx::query_as::<_, VitalSigns>(
            r#"
            SELECT * FROM vital_signs
            WHERE procedure_id = $1
              AND ($2::timestamptz IS NULL OR timestamp >= $2)
              AND ($3::timestamptz IS NULL OR timestamp <= $3)
            ORDER BY timestamp DESC
            LIMIT $4
            "#
        )
        .bind(procedure_id)
        .bind(from)
        .bind(to)
        .bind(limit)
        .fetch_all(&self.pool)
        .await?;

        Ok(vitals)
    }
}

#[derive(Debug, Deserialize)]
pub struct RecordVitalsRequest {
    pub temperature: Option<f64>,
    pub heart_rate: Option<i32>,
    pub systolic_bp: Option<i32>,
    pub diastolic_bp: Option<i32>,
    pub respiratory_rate: Option<i32>,
    pub oxygen_saturation: Option<f64>,
    pub end_tidal_co2: Option<f64>,
    pub cvp: Option<i32>,
    pub source: String,
}

#[derive(Debug, Serialize)]
pub struct VitalsRecordResult {
    pub recorded: bool,
    pub reading_id: Uuid,
    pub alerts: Vec<Alert>,
}
```

## PostgreSQL Schema

```sql
-- Database: cryo_revival

-- Enable extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Enum types
CREATE TYPE revival_status AS ENUM (
    'SCHEDULED', 'PREPARING', 'IN_PROGRESS', 'PAUSED',
    'COMPLETED', 'FAILED', 'ABORTED', 'RE_PRESERVED'
);

CREATE TYPE revival_stage AS ENUM (
    'PRE_REVIVAL', 'WARMING', 'CRYOPROTECTANT_REMOVAL', 'REHYDRATION',
    'CARDIOVASCULAR_RESTART', 'RESPIRATORY_ACTIVATION',
    'NEUROLOGICAL_RESTORATION', 'STABILIZATION', 'POST_REVIVAL_CARE'
);

CREATE TYPE stage_status AS ENUM (
    'PENDING', 'IN_PROGRESS', 'COMPLETED', 'FAILED', 'SKIPPED'
);

CREATE TYPE alert_severity AS ENUM (
    'INFO', 'WARNING', 'CRITICAL', 'EMERGENCY'
);

CREATE TYPE team_role AS ENUM (
    'REVIVAL_DIRECTOR', 'CRYONICS_SPECIALIST', 'CARDIOVASCULAR_SURGEON',
    'NEUROLOGIST', 'ANESTHESIOLOGIST', 'PERFUSIONIST',
    'ICU_NURSE', 'MONITORING_SPECIALIST', 'SUPPORT_STAFF'
);

CREATE TYPE complication_severity AS ENUM (
    'MILD', 'MODERATE', 'SEVERE', 'LIFE_THREATENING'
);

CREATE TYPE cardiac_rhythm AS ENUM (
    'ASYSTOLE', 'VENTRICULAR_FIBRILLATION', 'VENTRICULAR_TACHYCARDIA',
    'PULSELESS_ELECTRICAL_ACTIVITY', 'BRADYCARDIA', 'SINUS_RHYTHM',
    'ATRIAL_FIBRILLATION', 'PACED'
);

-- Main procedures table
CREATE TABLE revival_procedures (
    procedure_id VARCHAR(20) PRIMARY KEY,
    subject_id VARCHAR(50) NOT NULL,
    identity_id VARCHAR(50) NOT NULL,
    consent_id VARCHAR(50) NOT NULL,
    status revival_status NOT NULL DEFAULT 'SCHEDULED',
    current_stage revival_stage NOT NULL DEFAULT 'PRE_REVIVAL',
    facility_id VARCHAR(50) NOT NULL,
    sequence_number INTEGER NOT NULL,
    started_at TIMESTAMPTZ,
    completed_at TIMESTAMPTZ,
    last_updated TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_procedure_id CHECK (procedure_id ~ '^REVIVAL-[0-9]{4}-[0-9]{3}$')
);

CREATE INDEX idx_procedures_status ON revival_procedures(status);
CREATE INDEX idx_procedures_facility ON revival_procedures(facility_id);
CREATE INDEX idx_procedures_subject ON revival_procedures(subject_id);

-- Stage records
CREATE TABLE stage_records (
    record_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    stage revival_stage NOT NULL,
    stage_number INTEGER NOT NULL,
    status stage_status NOT NULL DEFAULT 'PENDING',
    started_at TIMESTAMPTZ,
    completed_at TIMESTAMPTZ,
    parameters JSONB NOT NULL DEFAULT '{}',
    notes TEXT[] DEFAULT '{}',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    UNIQUE(procedure_id, stage)
);

CREATE INDEX idx_stage_records_procedure ON stage_records(procedure_id);
CREATE INDEX idx_stage_records_status ON stage_records(status);

-- Vital signs
CREATE TABLE vital_signs (
    reading_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    temperature DECIMAL(4,2),
    heart_rate INTEGER,
    systolic_bp INTEGER,
    diastolic_bp INTEGER,
    mean_arterial_pressure INTEGER GENERATED ALWAYS AS (
        CASE WHEN systolic_bp IS NOT NULL AND diastolic_bp IS NOT NULL
        THEN diastolic_bp + (systolic_bp - diastolic_bp) / 3
        ELSE NULL END
    ) STORED,
    respiratory_rate INTEGER,
    oxygen_saturation DECIMAL(5,2),
    end_tidal_co2 DECIMAL(4,1),
    cvp INTEGER,
    cardiac_output DECIMAL(4,2),
    source VARCHAR(20) NOT NULL DEFAULT 'AUTOMATED',
    verified BOOLEAN DEFAULT FALSE,
    verified_by VARCHAR(50),

    CONSTRAINT valid_temp CHECK (temperature IS NULL OR temperature BETWEEN -200 AND 50),
    CONSTRAINT valid_hr CHECK (heart_rate IS NULL OR heart_rate BETWEEN 0 AND 300),
    CONSTRAINT valid_spo2 CHECK (oxygen_saturation IS NULL OR oxygen_saturation BETWEEN 0 AND 100)
);

CREATE INDEX idx_vitals_procedure ON vital_signs(procedure_id);
CREATE INDEX idx_vitals_timestamp ON vital_signs(procedure_id, timestamp DESC);

-- Temperature readings (for warming phase)
CREATE TABLE temperature_readings (
    reading_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    temperature DECIMAL(5,2) NOT NULL,
    location VARCHAR(20) NOT NULL, -- CORE, BRAIN, PERIPHERAL
    sensor_id VARCHAR(50),

    CONSTRAINT valid_location CHECK (location IN ('CORE', 'BRAIN', 'PERIPHERAL'))
);

CREATE INDEX idx_temp_procedure ON temperature_readings(procedure_id, timestamp DESC);

-- Cardiovascular data
CREATE TABLE cardiovascular_records (
    record_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    rhythm cardiac_rhythm NOT NULL,
    heart_rate INTEGER,
    systolic_bp INTEGER,
    diastolic_bp INTEGER,
    cardiac_output DECIMAL(4,2),
    cardiac_index DECIMAL(3,2),
    stroke_volume INTEGER,
    cvp INTEGER,
    pulmonary_artery_pressure INTEGER,
    notes TEXT
);

CREATE INDEX idx_cv_procedure ON cardiovascular_records(procedure_id, timestamp DESC);

-- Defibrillation attempts
CREATE TABLE defibrillation_attempts (
    attempt_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    attempt_number INTEGER NOT NULL,
    energy_joules INTEGER NOT NULL,
    rhythm_before cardiac_rhythm NOT NULL,
    rhythm_after cardiac_rhythm NOT NULL,
    success BOOLEAN NOT NULL,
    performed_by VARCHAR(50) NOT NULL,
    notes TEXT
);

CREATE INDEX idx_defib_procedure ON defibrillation_attempts(procedure_id);

-- EEG readings
CREATE TABLE eeg_readings (
    reading_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    pattern VARCHAR(30) NOT NULL, -- FLAT, BURST_SUPPRESSION, SLOW_WAVE, NORMAL
    frequency_hz DECIMAL(4,2),
    amplitude_uv DECIMAL(6,2),
    seizure_activity BOOLEAN DEFAULT FALSE,
    interpretation TEXT,
    recorded_by VARCHAR(50)
);

CREATE INDEX idx_eeg_procedure ON eeg_readings(procedure_id, timestamp DESC);

-- Neurological assessments
CREATE TABLE neurological_assessments (
    assessment_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    gcs_eye INTEGER NOT NULL CHECK (gcs_eye BETWEEN 1 AND 4),
    gcs_verbal INTEGER NOT NULL CHECK (gcs_verbal BETWEEN 1 AND 5),
    gcs_motor INTEGER NOT NULL CHECK (gcs_motor BETWEEN 1 AND 6),
    gcs_total INTEGER GENERATED ALWAYS AS (gcs_eye + gcs_verbal + gcs_motor) STORED,
    pupil_left_size DECIMAL(3,1),
    pupil_right_size DECIMAL(3,1),
    pupil_left_response VARCHAR(20),
    pupil_right_response VARCHAR(20),
    corneal_reflex BOOLEAN,
    gag_reflex BOOLEAN,
    cough_reflex BOOLEAN,
    intracranial_pressure DECIMAL(4,1),
    cerebral_perfusion_pressure DECIMAL(4,1),
    assessed_by VARCHAR(50) NOT NULL
);

CREATE INDEX idx_neuro_procedure ON neurological_assessments(procedure_id, timestamp DESC);

-- Laboratory results
CREATE TABLE laboratory_results (
    result_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    category VARCHAR(30) NOT NULL,
    test_name VARCHAR(50) NOT NULL,
    value DECIMAL(10,4),
    value_text VARCHAR(100),
    unit VARCHAR(20),
    reference_low DECIMAL(10,4),
    reference_high DECIMAL(10,4),
    flag VARCHAR(20),
    critical BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_lab_procedure ON laboratory_results(procedure_id, timestamp DESC);
CREATE INDEX idx_lab_critical ON laboratory_results(procedure_id) WHERE critical = TRUE;

-- Alerts
CREATE TABLE alerts (
    alert_id VARCHAR(30) PRIMARY KEY,
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    severity alert_severity NOT NULL,
    category VARCHAR(30) NOT NULL,
    source VARCHAR(50) NOT NULL,
    message TEXT NOT NULL,
    data JSONB DEFAULT '{}',
    acknowledged BOOLEAN DEFAULT FALSE,
    acknowledged_by VARCHAR(50),
    acknowledged_at TIMESTAMPTZ,
    resolved BOOLEAN DEFAULT FALSE,
    resolved_by VARCHAR(50),
    resolved_at TIMESTAMPTZ,
    resolution TEXT
);

CREATE INDEX idx_alerts_procedure ON alerts(procedure_id);
CREATE INDEX idx_alerts_active ON alerts(procedure_id) WHERE resolved = FALSE;
CREATE INDEX idx_alerts_severity ON alerts(procedure_id, severity) WHERE resolved = FALSE;

-- Complications
CREATE TABLE complications (
    complication_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    stage_record_id UUID REFERENCES stage_records(record_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    type VARCHAR(50) NOT NULL,
    severity complication_severity NOT NULL,
    description TEXT NOT NULL,
    detected_by VARCHAR(50) NOT NULL,
    resolved BOOLEAN DEFAULT FALSE,
    resolution_time TIMESTAMPTZ,
    residual_effects TEXT[],
    response_actions JSONB DEFAULT '[]'
);

CREATE INDEX idx_complications_procedure ON complications(procedure_id);
CREATE INDEX idx_complications_unresolved ON complications(procedure_id) WHERE resolved = FALSE;

-- Interventions
CREATE TABLE interventions (
    intervention_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    stage_record_id UUID REFERENCES stage_records(record_id),
    complication_id UUID REFERENCES complications(complication_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    type VARCHAR(50) NOT NULL,
    indication TEXT NOT NULL,
    description TEXT NOT NULL,
    performed_by VARCHAR(50) NOT NULL,
    assisted_by TEXT[],
    outcome VARCHAR(30) NOT NULL,
    complications TEXT[],
    notes TEXT
);

CREATE INDEX idx_interventions_procedure ON interventions(procedure_id);

-- Team members
CREATE TABLE team_members (
    member_id VARCHAR(50) PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    role team_role NOT NULL,
    specialization VARCHAR(100),
    license_number VARCHAR(50),
    wia_certification VARCHAR(50),
    certification_expiry DATE,
    contact_info_encrypted BYTEA,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Procedure team assignments
CREATE TABLE procedure_team (
    assignment_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    member_id VARCHAR(50) NOT NULL REFERENCES team_members(member_id),
    role team_role NOT NULL,
    responsibilities TEXT[],
    assigned_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    removed_at TIMESTAMPTZ,
    is_director BOOLEAN DEFAULT FALSE,

    UNIQUE(procedure_id, member_id)
);

CREATE INDEX idx_team_procedure ON procedure_team(procedure_id);

-- Shift schedules
CREATE TABLE shift_schedules (
    shift_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    shift_number INTEGER NOT NULL,
    start_time TIMESTAMPTZ NOT NULL,
    end_time TIMESTAMPTZ NOT NULL,
    handover_notes TEXT,
    incidents TEXT[]
);

CREATE INDEX idx_shifts_procedure ON shift_schedules(procedure_id);

-- Shift assignments
CREATE TABLE shift_assignments (
    assignment_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    shift_id UUID NOT NULL REFERENCES shift_schedules(shift_id),
    member_id VARCHAR(50) NOT NULL REFERENCES team_members(member_id),
    role team_role NOT NULL,
    responsibilities TEXT[]
);

-- Re-preservation decisions
CREATE TABLE re_preservation_decisions (
    decision_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    consent_id VARCHAR(50) NOT NULL,
    trigger VARCHAR(50) NOT NULL,
    criteria_evaluations JSONB NOT NULL DEFAULT '[]',
    criteria_score DECIMAL(4,3),
    votes JSONB NOT NULL DEFAULT '[]',
    decision VARCHAR(20), -- REPRESERVE, CONTINUE, REVIEW
    rationale TEXT,
    guardian_notified BOOLEAN DEFAULT FALSE,
    guardian_notified_at TIMESTAMPTZ,
    guardian_response TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    decided_at TIMESTAMPTZ
);

CREATE INDEX idx_repres_procedure ON re_preservation_decisions(procedure_id);

-- Re-preservation executions
CREATE TABLE re_preservation_executions (
    execution_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    decision_id UUID NOT NULL REFERENCES re_preservation_decisions(decision_id),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id),
    started_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    completed_at TIMESTAMPTZ,
    cooling_method VARCHAR(50) NOT NULL,
    cooling_rate DECIMAL(4,2),
    target_temperature DECIMAL(5,2),
    cryoprotectant_type VARCHAR(50),
    cryoprotectant_concentration DECIMAL(4,2),
    outcome VARCHAR(20),
    notes TEXT
);

-- Revival outcomes
CREATE TABLE revival_outcomes (
    outcome_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) NOT NULL REFERENCES revival_procedures(procedure_id) UNIQUE,
    result VARCHAR(30) NOT NULL,
    survival_status VARCHAR(30) NOT NULL,
    functional_assessment JSONB NOT NULL DEFAULT '{}',
    complications_summary JSONB NOT NULL DEFAULT '[]',
    interventions_summary JSONB NOT NULL DEFAULT '[]',
    recovery_prognosis JSONB NOT NULL DEFAULT '{}',
    follow_up_plan JSONB NOT NULL DEFAULT '{}',
    quality_metrics JSONB NOT NULL DEFAULT '{}',
    documented_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    documented_by VARCHAR(50) NOT NULL
);

-- Audit log
CREATE TABLE audit_log (
    log_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    procedure_id VARCHAR(20) REFERENCES revival_procedures(procedure_id),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    action VARCHAR(50) NOT NULL,
    entity_type VARCHAR(50) NOT NULL,
    entity_id VARCHAR(50),
    actor_id VARCHAR(50) NOT NULL,
    actor_role VARCHAR(50),
    old_values JSONB,
    new_values JSONB,
    ip_address INET,
    user_agent TEXT
);

CREATE INDEX idx_audit_procedure ON audit_log(procedure_id);
CREATE INDEX idx_audit_timestamp ON audit_log(timestamp DESC);
CREATE INDEX idx_audit_actor ON audit_log(actor_id);

-- Views
CREATE VIEW active_procedures AS
SELECT p.*,
       s.started_at as current_stage_started,
       t.member_count,
       a.active_alerts
FROM revival_procedures p
LEFT JOIN stage_records s ON p.procedure_id = s.procedure_id
    AND p.current_stage = s.stage
LEFT JOIN (
    SELECT procedure_id, COUNT(*) as member_count
    FROM procedure_team WHERE removed_at IS NULL
    GROUP BY procedure_id
) t ON p.procedure_id = t.procedure_id
LEFT JOIN (
    SELECT procedure_id, COUNT(*) as active_alerts
    FROM alerts WHERE resolved = FALSE
    GROUP BY procedure_id
) a ON p.procedure_id = a.procedure_id
WHERE p.status IN ('SCHEDULED', 'PREPARING', 'IN_PROGRESS', 'PAUSED');

CREATE VIEW procedure_summary AS
SELECT
    p.procedure_id,
    p.subject_id,
    p.status,
    p.current_stage,
    p.started_at,
    p.completed_at,
    EXTRACT(EPOCH FROM (COALESCE(p.completed_at, NOW()) - p.started_at))/3600 as duration_hours,
    (SELECT COUNT(*) FROM stage_records WHERE procedure_id = p.procedure_id AND status = 'COMPLETED') as stages_completed,
    (SELECT COUNT(*) FROM complications WHERE procedure_id = p.procedure_id) as complications_count,
    (SELECT COUNT(*) FROM interventions WHERE procedure_id = p.procedure_id) as interventions_count,
    (SELECT COUNT(*) FROM alerts WHERE procedure_id = p.procedure_id AND severity IN ('CRITICAL', 'EMERGENCY')) as critical_alerts
FROM revival_procedures p;

-- Functions
CREATE OR REPLACE FUNCTION update_procedure_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.last_updated = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trigger_update_procedure_timestamp
    BEFORE UPDATE ON revival_procedures
    FOR EACH ROW
    EXECUTE FUNCTION update_procedure_timestamp();

CREATE OR REPLACE FUNCTION generate_alert_id(procedure_id VARCHAR)
RETURNS VARCHAR AS $$
DECLARE
    seq INTEGER;
BEGIN
    SELECT COALESCE(MAX(CAST(SPLIT_PART(alert_id, '-', 4) AS INTEGER)), 0) + 1
    INTO seq
    FROM alerts WHERE alerts.procedure_id = generate_alert_id.procedure_id;

    RETURN 'ALERT-' || procedure_id || '-' || LPAD(seq::TEXT, 4, '0');
END;
$$ LANGUAGE plpgsql;
```

## Docker Configuration

```yaml
# docker-compose.yml
version: '3.8'

services:
  revival-api:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://revival:${DB_PASSWORD}@db:5432/cryo_revival
      - REDIS_URL=redis://redis:6379
      - CONSENT_SERVICE_URL=http://consent-service:8080
      - IDENTITY_SERVICE_URL=http://identity-service:8080
      - JWT_SECRET=${JWT_SECRET}
      - RUST_LOG=info
    depends_on:
      - db
      - redis
    networks:
      - revival-network
    deploy:
      replicas: 3
      resources:
        limits:
          cpus: '2'
          memory: 2G

  db:
    image: postgres:15-alpine
    environment:
      - POSTGRES_DB=cryo_revival
      - POSTGRES_USER=revival
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init.sql:/docker-entrypoint-initdb.d/init.sql
    networks:
      - revival-network
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 8G

  redis:
    image: redis:7-alpine
    command: redis-server --appendonly yes
    volumes:
      - redis_data:/data
    networks:
      - revival-network

  monitoring-worker:
    build:
      context: .
      dockerfile: Dockerfile.worker
    environment:
      - DATABASE_URL=postgres://revival:${DB_PASSWORD}@db:5432/cryo_revival
      - REDIS_URL=redis://redis:6379
    depends_on:
      - db
      - redis
    networks:
      - revival-network
    deploy:
      replicas: 2

  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=${GRAFANA_PASSWORD}
    volumes:
      - grafana_data:/var/lib/grafana
      - ./grafana/dashboards:/etc/grafana/provisioning/dashboards
    networks:
      - revival-network

  prometheus:
    image: prom/prometheus:latest
    ports:
      - "9090:9090"
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus_data:/prometheus
    networks:
      - revival-network

volumes:
  postgres_data:
  redis_data:
  grafana_data:
  prometheus_data:

networks:
  revival-network:
    driver: overlay
```

## Kubernetes Deployment

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: revival-api
  namespace: cryo-revival
spec:
  replicas: 3
  selector:
    matchLabels:
      app: revival-api
  template:
    metadata:
      labels:
        app: revival-api
    spec:
      containers:
      - name: revival-api
        image: wia/cryo-revival-api:latest
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: revival-secrets
              key: database-url
        - name: REDIS_URL
          valueFrom:
            configMapKeyRef:
              name: revival-config
              key: redis-url
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: revival-secrets
              key: jwt-secret
        resources:
          requests:
            cpu: "500m"
            memory: "512Mi"
          limits:
            cpu: "2000m"
            memory: "2Gi"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: revival-api
  namespace: cryo-revival
spec:
  selector:
    app: revival-api
  ports:
  - port: 80
    targetPort: 8080
  type: ClusterIP
---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: revival-api-hpa
  namespace: cryo-revival
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: revival-api
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

## Integration Points

### CRYO-CONSENT Integration

```rust
// Integration with CRYO-CONSENT service
pub struct ConsentServiceClient {
    base_url: String,
    client: reqwest::Client,
}

impl ConsentServiceClient {
    pub async fn verify_consent(&self, consent_id: &str) -> Result<ConsentVerification> {
        let response = self.client
            .get(&format!("{}/consents/{}/verify", self.base_url, consent_id))
            .send()
            .await?;

        response.json().await.map_err(Into::into)
    }

    pub async fn check_revival_conditions(&self, consent_id: &str) -> Result<bool> {
        let response: RevivalConditionCheck = self.client
            .get(&format!("{}/consents/{}/revival-conditions", self.base_url, consent_id))
            .send()
            .await?
            .json()
            .await?;

        Ok(response.all_conditions_met)
    }

    pub async fn notify_revival_start(&self, consent_id: &str, procedure_id: &str) -> Result<()> {
        self.client
            .post(&format!("{}/consents/{}/revival-notification", self.base_url, consent_id))
            .json(&serde_json::json!({
                "procedure_id": procedure_id,
                "event": "REVIVAL_STARTED"
            }))
            .send()
            .await?;

        Ok(())
    }
}
```

### CRYO-IDENTITY Integration

```rust
// Integration with CRYO-IDENTITY service
pub struct IdentityServiceClient {
    base_url: String,
    client: reqwest::Client,
}

impl IdentityServiceClient {
    pub async fn verify_identity(&self, identity_id: &str) -> Result<IdentityVerification> {
        let response = self.client
            .get(&format!("{}/identities/{}/verify", self.base_url, identity_id))
            .send()
            .await?;

        response.json().await.map_err(Into::into)
    }

    pub async fn initiate_restoration(&self, identity_id: &str, procedure_id: &str) -> Result<String> {
        let response: RestorationResponse = self.client
            .post(&format!("{}/identities/{}/restore", self.base_url, identity_id))
            .json(&serde_json::json!({
                "procedure_id": procedure_id,
                "restoration_type": "REVIVAL"
            }))
            .send()
            .await?
            .json()
            .await?;

        Ok(response.restoration_id)
    }
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
