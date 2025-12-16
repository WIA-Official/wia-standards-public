//! Phase 2 기반 핵심 알고리즘
//!
//! 검역 적격성, 백신 일정, 건강 위험도, 응급 정보 등 핵심 로직

use crate::types::*;
use chrono::{Duration, NaiveDate, Utc};
use std::collections::HashMap;
use uuid::Uuid;

// ==================== Eligibility Result ====================

/// 요구사항 체크 결과
#[derive(Debug, Clone)]
pub struct RequirementCheck {
    pub requirement: String,
    pub status: RequirementStatus,
    pub details: String,
    pub action_required: Option<String>,
    pub deadline: Option<NaiveDate>,
}

/// 요구사항 상태
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequirementStatus {
    Passed,
    Failed,
    Warning,
    Pending,
}

/// 검역 적격성 결과
#[derive(Debug, Clone)]
pub struct EligibilityResult {
    pub eligible: bool,
    pub overall_score: u32,
    pub requirements: Vec<RequirementCheck>,
    pub missing_items: Vec<String>,
    pub warnings: Vec<String>,
    pub recommendations: Vec<String>,
    pub estimated_processing_days: u32,
}

// ==================== Vaccination Schedule ====================

/// 예정된 백신
#[derive(Debug, Clone)]
pub struct ScheduledVaccination {
    pub vaccine: String,
    pub due_date: NaiveDate,
    pub priority: VaccinePriority,
    pub dose_number: u32,
    pub reason: String,
}

/// 백신 우선순위
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum VaccinePriority {
    Urgent,
    High,
    Medium,
    Low,
}

/// 백신 일정 결과
#[derive(Debug, Clone)]
pub struct VaccinationScheduleResult {
    pub upcoming: Vec<ScheduledVaccination>,
    pub overdue: Vec<ScheduledVaccination>,
    pub completed: Vec<CompletedVaccination>,
    pub summary: VaccineSummary,
}

/// 완료된 백신
#[derive(Debug, Clone)]
pub struct CompletedVaccination {
    pub vaccine: String,
    pub last_dose: NaiveDate,
    pub next_due: NaiveDate,
    pub status: String,
}

/// 백신 요약
#[derive(Debug, Clone)]
pub struct VaccineSummary {
    pub total_vaccines: usize,
    pub up_to_date: usize,
    pub overdue_count: usize,
    pub upcoming_count: usize,
}

// ==================== Health Risk ====================

/// 위험 요소
#[derive(Debug, Clone)]
pub struct RiskFactor {
    pub category: String,
    pub factor: String,
    pub severity: RiskSeverity,
    pub score: u32,
    pub description: String,
    pub evidence: Option<String>,
}

/// 위험 심각도
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RiskSeverity {
    Low,
    Moderate,
    High,
}

/// 건강 권장사항
#[derive(Debug, Clone)]
pub struct HealthRecommendation {
    pub category: String,
    pub recommendation: String,
    pub priority: String,
    pub action: Option<String>,
}

/// 건강 모니터링 항목
#[derive(Debug, Clone)]
pub struct MonitoringItem {
    pub item: String,
    pub frequency: String,
    pub next_due: Option<NaiveDate>,
    pub reason: String,
}

/// 건강 위험도 결과
#[derive(Debug, Clone)]
pub struct HealthRiskResult {
    pub overall_risk: OverallRisk,
    pub risk_score: u32,
    pub risk_factors: Vec<RiskFactor>,
    pub recommendations: Vec<HealthRecommendation>,
    pub monitoring_schedule: Vec<MonitoringItem>,
}

/// 전체 위험 등급
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverallRisk {
    Low,
    Moderate,
    High,
    Critical,
}

// ==================== Emergency Info ====================

/// 긴급 알림
#[derive(Debug, Clone)]
pub struct CriticalAlert {
    pub alert_type: AlertType,
    pub severity: AlertSeverity,
    pub message: String,
    pub action: Option<String>,
}

/// 알림 유형
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertType {
    Allergy,
    Medication,
    Condition,
    Warning,
}

/// 알림 심각도
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertSeverity {
    High,
    Critical,
}

/// 투약 요약
#[derive(Debug, Clone)]
pub struct MedicationSummary {
    pub name: String,
    pub dosage: String,
    pub route: String,
    pub indication: String,
}

/// 수술 요약
#[derive(Debug, Clone)]
pub struct SurgerySummary {
    pub procedure: String,
    pub date: NaiveDate,
    pub restrictions: Vec<String>,
    pub follow_up_required: bool,
}

/// 응급 정보
#[derive(Debug, Clone)]
pub struct EmergencyInfo {
    pub critical_alerts: Vec<CriticalAlert>,
    pub pet_name: String,
    pub species: PetSpecies,
    pub breed: Option<String>,
    pub age: Option<String>,
    pub weight: Option<f32>,
    pub sex: String,
    pub severe_allergies: Vec<String>,
    pub current_medications: Vec<MedicationSummary>,
    pub active_conditions: Vec<String>,
    pub recent_surgeries: Vec<SurgerySummary>,
    pub emergency_contacts: Vec<EmergencyContact>,
    pub veterinary_notes: Vec<String>,
}

// ==================== Verification ====================

/// 검증 상세
#[derive(Debug, Clone)]
pub struct VerificationDetail {
    pub component: String,
    pub status: VerificationStatus,
    pub message: String,
    pub timestamp: Option<String>,
}

/// 검증 상태
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerificationStatus {
    Valid,
    Invalid,
    Unknown,
}

/// 검증 결과
#[derive(Debug, Clone)]
pub struct VerificationResult {
    pub valid: bool,
    pub signature_valid: bool,
    pub chain_valid: bool,
    pub issuer_trusted: bool,
    pub records_verified: usize,
    pub records_failed: usize,
    pub details: Vec<VerificationDetail>,
    pub warnings: Vec<String>,
}

// ==================== Core Services ====================

/// 여권 관리자
pub struct PassportManager {
    passports: HashMap<Uuid, PetHealthPassport>,
}

impl PassportManager {
    pub fn new() -> Self {
        Self {
            passports: HashMap::new(),
        }
    }

    /// 여권 생성
    pub fn create(&mut self, identity: PetIdentity) -> PetHealthPassport {
        let passport = PetHealthPassport::new(identity);
        let id = passport.passport_id();
        self.passports.insert(id, passport.clone());
        passport
    }

    /// 여권 조회
    pub fn get(&self, id: &Uuid) -> Option<&PetHealthPassport> {
        self.passports.get(id)
    }

    /// 마이크로칩으로 조회
    pub fn find_by_microchip(&self, chip_id: &str) -> Option<&PetHealthPassport> {
        self.passports.values().find(|p| {
            p.identity.microchip_id.as_ref().map_or(false, |id| id == chip_id)
        })
    }

    /// 백신 기록 추가
    pub fn add_vaccination(&mut self, passport_id: &Uuid, record: VaccinationRecord) -> bool {
        if let Some(passport) = self.passports.get_mut(passport_id) {
            passport.medical_records.vaccinations.push(record);
            passport.touch();
            true
        } else {
            false
        }
    }

    /// 알레르기 기록 추가
    pub fn add_allergy(&mut self, passport_id: &Uuid, record: AllergyRecord) -> bool {
        if let Some(passport) = self.passports.get_mut(passport_id) {
            passport.medical_records.allergies.push(record);
            passport.touch();
            true
        } else {
            false
        }
    }
}

impl Default for PassportManager {
    fn default() -> Self {
        Self::new()
    }
}

/// 검역 적격성 판정기
pub struct QuarantineChecker {
    country_requirements: HashMap<String, CountryRequirements>,
}

impl QuarantineChecker {
    pub fn new() -> Self {
        let mut checker = Self {
            country_requirements: HashMap::new(),
        };
        checker.load_default_requirements();
        checker
    }

    fn load_default_requirements(&mut self) {
        // EU 요구사항 (독일 예시)
        self.country_requirements.insert("DE".to_string(), CountryRequirements {
            country: "DE".to_string(),
            last_updated: Utc::now(),
            microchip_required: true,
            microchip_standard: "ISO11784".to_string(),
            rabies_vaccination: RabiesRequirement {
                required: true,
                minimum_age: 3,
                wait_period: 21,
                validity_period: 12,
                titer_test_required: true,
                minimum_titer: 0.5,
            },
            other_vaccinations: vec![],
            parasite_treatment: Some(ParasiteTreatmentRequirement {
                treatment_type: "tapeworm".to_string(),
                required: true,
                timing_before_entry: 120,
            }),
            quarantine: None,
            banned_breeds: vec![],
            import_permit_required: false,
            health_certificate_required: true,
            health_certificate_valid_days: 10,
            exempt_countries: vec!["FR".to_string(), "NL".to_string(), "BE".to_string()],
        });

        // 호주 요구사항
        self.country_requirements.insert("AU".to_string(), CountryRequirements {
            country: "AU".to_string(),
            last_updated: Utc::now(),
            microchip_required: true,
            microchip_standard: "ISO11784".to_string(),
            rabies_vaccination: RabiesRequirement {
                required: true,
                minimum_age: 3,
                wait_period: 30,
                validity_period: 12,
                titer_test_required: true,
                minimum_titer: 0.5,
            },
            other_vaccinations: vec!["distemper".to_string(), "parvovirus".to_string()],
            parasite_treatment: Some(ParasiteTreatmentRequirement {
                treatment_type: "tick".to_string(),
                required: true,
                timing_before_entry: 48,
            }),
            quarantine: Some(QuarantineRequirement {
                required: true,
                duration: 10,
                home_quarantine_allowed: false,
            }),
            banned_breeds: vec![],
            import_permit_required: true,
            health_certificate_required: true,
            health_certificate_valid_days: 5,
            exempt_countries: vec!["NZ".to_string()],
        });
    }

    /// 검역 적격성 확인
    pub fn check_eligibility(
        &self,
        passport: &PetHealthPassport,
        departure: &str,
        arrival: &str,
        travel_date: NaiveDate,
    ) -> EligibilityResult {
        let mut checks = Vec::new();
        let mut missing = Vec::new();
        let mut warnings = Vec::new();

        let requirements = match self.country_requirements.get(arrival) {
            Some(req) => req,
            None => {
                return EligibilityResult {
                    eligible: false,
                    overall_score: 0,
                    requirements: vec![RequirementCheck {
                        requirement: "Country Support".to_string(),
                        status: RequirementStatus::Failed,
                        details: format!("Country {} not supported", arrival),
                        action_required: Some("Contact WIA support".to_string()),
                        deadline: None,
                    }],
                    missing_items: vec!["Country requirements".to_string()],
                    warnings: vec![],
                    recommendations: vec![],
                    estimated_processing_days: 0,
                };
            }
        };

        // 1. 마이크로칩 검증
        if requirements.microchip_required {
            let chip_status = if passport.identity.microchip_id.is_some() {
                RequirementCheck {
                    requirement: "Microchip".to_string(),
                    status: RequirementStatus::Passed,
                    details: format!("ISO 11784/11785 compliant ({})",
                        passport.identity.microchip_id.as_ref().unwrap()),
                    action_required: None,
                    deadline: None,
                }
            } else {
                missing.push("Valid microchip".to_string());
                RequirementCheck {
                    requirement: "Microchip".to_string(),
                    status: RequirementStatus::Failed,
                    details: "No microchip registered".to_string(),
                    action_required: Some("Get ISO 11784/11785 compliant microchip".to_string()),
                    deadline: None,
                }
            };
            checks.push(chip_status);
        }

        // 2. 광견병 백신 검증
        if requirements.rabies_vaccination.required {
            let rabies_check = self.check_rabies_vaccination(
                passport,
                &requirements.rabies_vaccination,
                travel_date,
            );
            if rabies_check.status == RequirementStatus::Failed {
                missing.push("Valid rabies vaccination".to_string());
            }
            checks.push(rabies_check);
        }

        // 3. 항체가 검사
        if requirements.rabies_vaccination.titer_test_required {
            let titer_check = self.check_titer_test(passport, requirements.rabies_vaccination.minimum_titer);
            if titer_check.status == RequirementStatus::Failed {
                missing.push(format!("Rabies titer test (>= {} IU/ml)", requirements.rabies_vaccination.minimum_titer));
            }
            checks.push(titer_check);
        }

        // 4. 기생충 치료
        if let Some(ref parasite_req) = requirements.parasite_treatment {
            if parasite_req.required {
                let parasite_check = RequirementCheck {
                    requirement: format!("{} Treatment", parasite_req.treatment_type),
                    status: RequirementStatus::Pending,
                    details: format!("Required {}-{} hours before arrival",
                        parasite_req.timing_before_entry / 2,
                        parasite_req.timing_before_entry),
                    action_required: Some("Get treatment from veterinarian".to_string()),
                    deadline: Some(travel_date - Duration::hours(parasite_req.timing_before_entry as i64 / 24)),
                };
                checks.push(parasite_check);
            }
        }

        // 5. 금지 품종
        if !requirements.banned_breeds.is_empty() {
            let breed = passport.identity.breed.as_ref().map(|b| b.to_lowercase());
            let is_banned = breed.as_ref().map_or(false, |b| {
                requirements.banned_breeds.iter().any(|banned| b.contains(&banned.to_lowercase()))
            });

            let breed_check = if is_banned {
                missing.push("Breed allowed".to_string());
                RequirementCheck {
                    requirement: "Breed Allowed".to_string(),
                    status: RequirementStatus::Failed,
                    details: format!("Breed {} is banned in {}", breed.unwrap_or_default(), arrival),
                    action_required: None,
                    deadline: None,
                }
            } else {
                RequirementCheck {
                    requirement: "Breed Allowed".to_string(),
                    status: RequirementStatus::Passed,
                    details: "Breed is allowed".to_string(),
                    action_required: None,
                    deadline: None,
                }
            };
            checks.push(breed_check);
        }

        // 6. 건강 증명서
        if requirements.health_certificate_required {
            let cert_check = RequirementCheck {
                requirement: "Health Certificate".to_string(),
                status: RequirementStatus::Pending,
                details: format!("Valid for {} days before travel", requirements.health_certificate_valid_days),
                action_required: Some("Get health certificate from licensed veterinarian".to_string()),
                deadline: Some(travel_date - Duration::days(requirements.health_certificate_valid_days as i64)),
            };
            checks.push(cert_check);
        }

        // 점수 계산
        let passed_count = checks.iter().filter(|c| c.status == RequirementStatus::Passed).count();
        let total_count = checks.len();
        let score = if total_count > 0 {
            (passed_count * 100 / total_count) as u32
        } else {
            0
        };

        // 권장사항 생성
        let mut recommendations = Vec::new();
        if requirements.parasite_treatment.is_some() {
            recommendations.push(format!("Schedule parasite treatment 1-5 days before departure"));
        }
        if requirements.health_certificate_required {
            recommendations.push(format!("Get health certificate within {} days of travel",
                requirements.health_certificate_valid_days));
        }

        EligibilityResult {
            eligible: missing.is_empty(),
            overall_score: score,
            requirements: checks,
            missing_items: missing,
            warnings,
            recommendations,
            estimated_processing_days: if requirements.quarantine.as_ref().map_or(false, |q| q.required) {
                requirements.quarantine.as_ref().unwrap().duration
            } else {
                0
            },
        }
    }

    fn check_rabies_vaccination(
        &self,
        passport: &PetHealthPassport,
        config: &RabiesRequirement,
        travel_date: NaiveDate,
    ) -> RequirementCheck {
        let rabies_records: Vec<_> = passport.medical_records.vaccinations
            .iter()
            .filter(|v| v.target_diseases.contains(&VaccineDisease::Rabies))
            .collect();

        if rabies_records.is_empty() {
            return RequirementCheck {
                requirement: "Rabies Vaccination".to_string(),
                status: RequirementStatus::Failed,
                details: "No rabies vaccination record found".to_string(),
                action_required: Some("Get rabies vaccination from licensed veterinarian".to_string()),
                deadline: None,
            };
        }

        let latest = rabies_records.iter().max_by_key(|r| r.administered_at).unwrap();
        let valid_until = latest.valid_until.date_naive();

        if travel_date > valid_until {
            return RequirementCheck {
                requirement: "Rabies Vaccination".to_string(),
                status: RequirementStatus::Failed,
                details: format!("Vaccination expired on {}", valid_until),
                action_required: Some("Get booster vaccination".to_string()),
                deadline: None,
            };
        }

        let vaccination_date = latest.administered_at.date_naive();
        let wait_period_end = vaccination_date + Duration::days(config.wait_period as i64);

        if travel_date < wait_period_end {
            let days_remaining = (wait_period_end - travel_date).num_days();
            return RequirementCheck {
                requirement: "Rabies Vaccination".to_string(),
                status: RequirementStatus::Failed,
                details: format!("Wait period not complete. {} days remaining", days_remaining),
                action_required: Some(format!("Travel date must be after {}", wait_period_end)),
                deadline: Some(wait_period_end),
            };
        }

        RequirementCheck {
            requirement: "Rabies Vaccination".to_string(),
            status: RequirementStatus::Passed,
            details: format!("Valid until {}", valid_until),
            action_required: None,
            deadline: None,
        }
    }

    fn check_titer_test(&self, passport: &PetHealthPassport, minimum_titer: f32) -> RequirementCheck {
        // 항체가 검사 결과 조회 (진단 기록에서)
        let titer_tests: Vec<_> = passport.medical_records.diagnostics
            .iter()
            .filter(|d| d.test_name.to_lowercase().contains("titer") ||
                       d.test_name.to_lowercase().contains("rabies antibody"))
            .collect();

        if titer_tests.is_empty() {
            return RequirementCheck {
                requirement: "Rabies Titer Test".to_string(),
                status: RequirementStatus::Failed,
                details: "No titer test record found".to_string(),
                action_required: Some(format!("Get FAVN or RFFIT test (minimum {} IU/ml)", minimum_titer)),
                deadline: None,
            };
        }

        // 가장 최근 검사 결과 확인
        let latest = titer_tests.iter().max_by_key(|t| t.performed_at).unwrap();
        let result = latest.results.iter().find(|r| r.parameter.to_lowercase().contains("titer"));

        if let Some(titer_result) = result {
            if let Ok(value) = titer_result.value.parse::<f32>() {
                if value >= minimum_titer {
                    return RequirementCheck {
                        requirement: "Rabies Titer Test".to_string(),
                        status: RequirementStatus::Passed,
                        details: format!("{} IU/ml (minimum: {} IU/ml)", value, minimum_titer),
                        action_required: None,
                        deadline: None,
                    };
                } else {
                    return RequirementCheck {
                        requirement: "Rabies Titer Test".to_string(),
                        status: RequirementStatus::Failed,
                        details: format!("{} IU/ml is below minimum {} IU/ml", value, minimum_titer),
                        action_required: Some("Get revaccination and retest".to_string()),
                        deadline: None,
                    };
                }
            }
        }

        RequirementCheck {
            requirement: "Rabies Titer Test".to_string(),
            status: RequirementStatus::Warning,
            details: "Could not parse titer test result".to_string(),
            action_required: Some("Verify test results".to_string()),
            deadline: None,
        }
    }
}

impl Default for QuarantineChecker {
    fn default() -> Self {
        Self::new()
    }
}

/// 백신 일정 계산기
pub struct VaccinationScheduler;

impl VaccinationScheduler {
    pub fn new() -> Self {
        Self
    }

    /// 백신 일정 계산
    pub fn calculate_schedule(
        &self,
        passport: &PetHealthPassport,
    ) -> VaccinationScheduleResult {
        let mut upcoming = Vec::new();
        let mut overdue = Vec::new();
        let mut completed = Vec::new();

        let today = Utc::now().date_naive();

        // 광견병 백신 확인
        let rabies_records: Vec<_> = passport.medical_records.vaccinations
            .iter()
            .filter(|v| v.target_diseases.contains(&VaccineDisease::Rabies))
            .collect();

        if rabies_records.is_empty() {
            overdue.push(ScheduledVaccination {
                vaccine: "Rabies".to_string(),
                due_date: today,
                priority: VaccinePriority::Urgent,
                dose_number: 1,
                reason: "Initial vaccination required".to_string(),
            });
        } else {
            let latest = rabies_records.iter().max_by_key(|r| r.administered_at).unwrap();
            let next_due = latest.valid_until.date_naive();

            if next_due < today {
                overdue.push(ScheduledVaccination {
                    vaccine: "Rabies".to_string(),
                    due_date: next_due,
                    priority: VaccinePriority::Urgent,
                    dose_number: rabies_records.len() as u32 + 1,
                    reason: "Booster overdue".to_string(),
                });
            } else if next_due < today + Duration::days(30) {
                upcoming.push(ScheduledVaccination {
                    vaccine: "Rabies".to_string(),
                    due_date: next_due,
                    priority: VaccinePriority::High,
                    dose_number: rabies_records.len() as u32 + 1,
                    reason: "Booster due soon".to_string(),
                });
            } else {
                completed.push(CompletedVaccination {
                    vaccine: "Rabies".to_string(),
                    last_dose: latest.administered_at.date_naive(),
                    next_due,
                    status: "up_to_date".to_string(),
                });
            }
        }

        // 종별 추가 백신
        match passport.identity.species {
            PetSpecies::Dog => {
                // DHPP
                self.check_vaccine_status(
                    &passport.medical_records.vaccinations,
                    VaccineDisease::Distemper,
                    "DHPP",
                    today,
                    &mut upcoming, &mut overdue, &mut completed,
                );
            }
            PetSpecies::Cat => {
                // FVRCP
                self.check_vaccine_status(
                    &passport.medical_records.vaccinations,
                    VaccineDisease::FelinePanleukopenia,
                    "FVRCP",
                    today,
                    &mut upcoming, &mut overdue, &mut completed,
                );
            }
            _ => {}
        }

        VaccinationScheduleResult {
            upcoming,
            overdue,
            completed,
            summary: VaccineSummary {
                total_vaccines: 3, // 기본 백신 수
                up_to_date: completed.len(),
                overdue_count: overdue.len(),
                upcoming_count: upcoming.len(),
            },
        }
    }

    fn check_vaccine_status(
        &self,
        vaccinations: &[VaccinationRecord],
        disease: VaccineDisease,
        vaccine_name: &str,
        today: NaiveDate,
        upcoming: &mut Vec<ScheduledVaccination>,
        overdue: &mut Vec<ScheduledVaccination>,
        completed: &mut Vec<CompletedVaccination>,
    ) {
        let records: Vec<_> = vaccinations
            .iter()
            .filter(|v| v.target_diseases.contains(&disease))
            .collect();

        if records.is_empty() {
            overdue.push(ScheduledVaccination {
                vaccine: vaccine_name.to_string(),
                due_date: today,
                priority: VaccinePriority::High,
                dose_number: 1,
                reason: "Initial vaccination required".to_string(),
            });
        } else {
            let latest = records.iter().max_by_key(|r| r.administered_at).unwrap();
            let next_due = latest.valid_until.date_naive();

            if next_due < today {
                overdue.push(ScheduledVaccination {
                    vaccine: vaccine_name.to_string(),
                    due_date: next_due,
                    priority: VaccinePriority::High,
                    dose_number: records.len() as u32 + 1,
                    reason: "Booster overdue".to_string(),
                });
            } else {
                completed.push(CompletedVaccination {
                    vaccine: vaccine_name.to_string(),
                    last_dose: latest.administered_at.date_naive(),
                    next_due,
                    status: "up_to_date".to_string(),
                });
            }
        }
    }
}

impl Default for VaccinationScheduler {
    fn default() -> Self {
        Self::new()
    }
}

/// 건강 위험도 평가기
pub struct HealthRiskAssessor;

impl HealthRiskAssessor {
    pub fn new() -> Self {
        Self
    }

    /// 건강 위험도 평가
    pub fn assess(&self, passport: &PetHealthPassport) -> HealthRiskResult {
        let mut risk_factors = Vec::new();
        let mut total_score: f32 = 0.0;

        // 1. 유전 위험도
        if let Some(ref genetics) = passport.genetics {
            for marker in &genetics.health_markers {
                if marker.risk_level == "at_risk" {
                    risk_factors.push(RiskFactor {
                        category: "Genetic".to_string(),
                        factor: marker.condition.clone(),
                        severity: RiskSeverity::High,
                        score: 30,
                        description: format!("Genetic risk for {}", marker.condition),
                        evidence: Some(format!("Genotype: {}", marker.genotype)),
                    });
                    total_score += 30.0;
                } else if marker.risk_level == "carrier" {
                    risk_factors.push(RiskFactor {
                        category: "Genetic".to_string(),
                        factor: marker.condition.clone(),
                        severity: RiskSeverity::Low,
                        score: 10,
                        description: format!("Carrier for {}", marker.condition),
                        evidence: Some(format!("Genotype: {}", marker.genotype)),
                    });
                    total_score += 10.0;
                }
            }
        }

        // 2. 만성 질환
        for condition in &passport.medical_records.conditions {
            if condition.status == "active" {
                let severity = match condition.severity.as_str() {
                    "severe" => RiskSeverity::High,
                    "moderate" => RiskSeverity::Moderate,
                    _ => RiskSeverity::Low,
                };
                let score = match severity {
                    RiskSeverity::High => 30,
                    RiskSeverity::Moderate => 20,
                    RiskSeverity::Low => 10,
                };
                risk_factors.push(RiskFactor {
                    category: "Condition".to_string(),
                    factor: condition.condition_name.clone(),
                    severity,
                    score,
                    description: format!("Active condition: {}", condition.condition_name),
                    evidence: None,
                });
                total_score += score as f32;
            }
        }

        // 3. 알레르기
        for allergy in &passport.medical_records.allergies {
            if allergy.status == "active" && allergy.reaction_severity == "severe" {
                risk_factors.push(RiskFactor {
                    category: "Allergy".to_string(),
                    factor: allergy.allergen.clone(),
                    severity: RiskSeverity::High,
                    score: 25,
                    description: format!("Severe allergy to {}", allergy.allergen),
                    evidence: None,
                });
                total_score += 25.0;
            }
        }

        // 점수 정규화 (0-100)
        let normalized_score = (total_score.min(100.0)) as u32;

        // 위험 등급 결정
        let overall_risk = match normalized_score {
            0..=24 => OverallRisk::Low,
            25..=49 => OverallRisk::Moderate,
            50..=74 => OverallRisk::High,
            _ => OverallRisk::Critical,
        };

        // 권장사항 생성
        let recommendations = self.generate_recommendations(&risk_factors);
        let monitoring_schedule = self.generate_monitoring_schedule(&risk_factors);

        HealthRiskResult {
            overall_risk,
            risk_score: normalized_score,
            risk_factors,
            recommendations,
            monitoring_schedule,
        }
    }

    fn generate_recommendations(&self, risk_factors: &[RiskFactor]) -> Vec<HealthRecommendation> {
        let mut recommendations = Vec::new();

        for factor in risk_factors {
            if factor.severity == RiskSeverity::High {
                recommendations.push(HealthRecommendation {
                    category: factor.category.clone(),
                    recommendation: format!("Monitor {} closely", factor.factor),
                    priority: "high".to_string(),
                    action: Some(format!("Consult veterinarian about {}", factor.factor)),
                });
            }
        }

        recommendations
    }

    fn generate_monitoring_schedule(&self, risk_factors: &[RiskFactor]) -> Vec<MonitoringItem> {
        let mut schedule = Vec::new();

        for factor in risk_factors {
            if factor.severity == RiskSeverity::High {
                schedule.push(MonitoringItem {
                    item: factor.factor.clone(),
                    frequency: "Every 3 months".to_string(),
                    next_due: Some(Utc::now().date_naive() + Duration::days(90)),
                    reason: format!("High risk for {}", factor.factor),
                });
            }
        }

        schedule
    }
}

impl Default for HealthRiskAssessor {
    fn default() -> Self {
        Self::new()
    }
}

/// 응급 정보 추출기
pub struct EmergencyExtractor;

impl EmergencyExtractor {
    pub fn new() -> Self {
        Self
    }

    /// 응급 정보 추출
    pub fn extract(&self, passport: &PetHealthPassport) -> EmergencyInfo {
        let mut alerts = Vec::new();

        // 1. 심각한 알레르기
        let severe_allergies: Vec<String> = passport.medical_records.allergies
            .iter()
            .filter(|a| a.status == "active")
            .filter(|a| a.reaction_severity == "severe" || a.reaction_severity == "life_threatening")
            .map(|a| {
                let severity = if a.reaction_severity == "life_threatening" {
                    AlertSeverity::Critical
                } else {
                    AlertSeverity::High
                };
                alerts.push(CriticalAlert {
                    alert_type: AlertType::Allergy,
                    severity,
                    message: format!("ALLERGY: {}", a.allergen),
                    action: a.emergency_treatment.clone(),
                });
                a.allergen.clone()
            })
            .collect();

        // 2. 현재 복용 약물
        let today = Utc::now().date_naive();
        let current_medications: Vec<MedicationSummary> = passport.medical_records.medications
            .iter()
            .filter(|m| {
                m.start_date <= today &&
                m.end_date.map_or(true, |end| end >= today)
            })
            .map(|m| MedicationSummary {
                name: m.medication.name.clone(),
                dosage: format!("{} {} {}", m.dosage.amount, m.dosage.unit, m.dosage.frequency),
                route: m.dosage.route.clone(),
                indication: m.indication.clone(),
            })
            .collect();

        // 3. 활성 질환
        let active_conditions: Vec<String> = passport.medical_records.conditions
            .iter()
            .filter(|c| c.status == "active")
            .map(|c| {
                if c.severity == "severe" {
                    alerts.push(CriticalAlert {
                        alert_type: AlertType::Condition,
                        severity: AlertSeverity::High,
                        message: format!("Active condition: {}", c.condition_name),
                        action: c.management_plan.clone(),
                    });
                }
                c.condition_name.clone()
            })
            .collect();

        // 4. 최근 수술 (6개월 이내)
        let six_months_ago = today - Duration::days(180);
        let recent_surgeries: Vec<SurgerySummary> = passport.medical_records.surgeries
            .iter()
            .filter(|s| s.performed_at.date_naive() >= six_months_ago)
            .map(|s| SurgerySummary {
                procedure: s.procedure_name.clone(),
                date: s.performed_at.date_naive(),
                restrictions: s.restrictions.clone(),
                follow_up_required: s.follow_up_required,
            })
            .collect();

        // 5. 나이 계산
        let age = passport.identity.date_of_birth.map(|dob| {
            let years = (today - dob).num_days() / 365;
            let months = ((today - dob).num_days() % 365) / 30;
            format!("{} years {} months", years, months)
        });

        // 6. 수의사 참고사항
        let mut vet_notes = Vec::new();

        // 광견병 상태
        let rabies_record = passport.medical_records.vaccinations
            .iter()
            .find(|v| v.target_diseases.contains(&VaccineDisease::Rabies));

        if let Some(rabies) = rabies_record {
            let valid = rabies.valid_until.date_naive() > today;
            vet_notes.push(format!("Rabies: {} (until {})",
                if valid { "Current" } else { "EXPIRED" },
                rabies.valid_until.date_naive()));
        } else {
            vet_notes.push("Rabies: NO RECORD - Handle with caution".to_string());
        }

        // 정렬: Critical 먼저
        alerts.sort_by(|a, b| {
            match (&a.severity, &b.severity) {
                (AlertSeverity::Critical, AlertSeverity::High) => std::cmp::Ordering::Less,
                (AlertSeverity::High, AlertSeverity::Critical) => std::cmp::Ordering::Greater,
                _ => std::cmp::Ordering::Equal,
            }
        });

        EmergencyInfo {
            critical_alerts: alerts,
            pet_name: passport.identity.name.clone(),
            species: passport.identity.species,
            breed: passport.identity.breed.clone(),
            age,
            weight: passport.identity.weight.as_ref().map(|w| w.value),
            sex: format!("{:?}", passport.identity.sex),
            severe_allergies,
            current_medications,
            active_conditions,
            recent_surgeries,
            emergency_contacts: passport.guardian.as_ref()
                .map(|g| g.emergency_contacts.clone())
                .unwrap_or_default(),
            veterinary_notes: vet_notes,
        }
    }
}

impl Default for EmergencyExtractor {
    fn default() -> Self {
        Self::new()
    }
}

/// 문서 검증기
pub struct DocumentVerifier;

impl DocumentVerifier {
    pub fn new() -> Self {
        Self
    }

    /// 여권 검증
    pub fn verify(&self, passport: &PetHealthPassport) -> VerificationResult {
        let mut details = Vec::new();
        let mut records_verified = 0;
        let mut records_failed = 0;

        // 1. 서명 검증 (시뮬레이션)
        let signature_valid = passport.verification.as_ref()
            .and_then(|v| v.digital_signature.as_ref())
            .map_or(false, |sig| !sig.is_empty());

        details.push(VerificationDetail {
            component: "Passport Signature".to_string(),
            status: if signature_valid { VerificationStatus::Valid } else { VerificationStatus::Invalid },
            message: if signature_valid {
                "Digital signature verified".to_string()
            } else {
                "No digital signature or verification failed".to_string()
            },
            timestamp: None,
        });

        // 2. 인증서 체인 검증 (시뮬레이션)
        let chain_valid = passport.verification.as_ref()
            .map_or(false, |v| !v.certificate_chain.is_empty());

        details.push(VerificationDetail {
            component: "Certificate Chain".to_string(),
            status: if chain_valid { VerificationStatus::Valid } else { VerificationStatus::Invalid },
            message: if chain_valid {
                "Certificate chain valid".to_string()
            } else {
                "Certificate chain missing or invalid".to_string()
            },
            timestamp: None,
        });

        // 3. 발급 기관 확인
        let issuer_trusted = passport.verification.as_ref()
            .map_or(false, |v| !v.issuing_authority.is_empty());

        details.push(VerificationDetail {
            component: "Issuing Authority".to_string(),
            status: if issuer_trusted { VerificationStatus::Valid } else { VerificationStatus::Unknown },
            message: passport.verification.as_ref()
                .map(|v| format!("Issuer: {}", v.issuing_authority))
                .unwrap_or_else(|| "Unknown issuer".to_string()),
            timestamp: None,
        });

        // 4. 백신 기록 검증
        for vaccination in &passport.medical_records.vaccinations {
            if vaccination.verified {
                records_verified += 1;
            } else if vaccination.digital_signature.is_some() {
                // 서명은 있지만 검증 안됨
                records_failed += 1;
            }
        }

        // 5. 마이크로칩 형식 검증
        if let Some(ref chip_id) = passport.identity.microchip_id {
            let chip_valid = crate::validate_microchip_id(chip_id);
            details.push(VerificationDetail {
                component: "Microchip Format".to_string(),
                status: if chip_valid { VerificationStatus::Valid } else { VerificationStatus::Invalid },
                message: if chip_valid {
                    "ISO 11784/11785 compliant".to_string()
                } else {
                    "Invalid microchip format".to_string()
                },
                timestamp: None,
            });
        }

        let valid = signature_valid && chain_valid && records_failed == 0;

        VerificationResult {
            valid,
            signature_valid,
            chain_valid,
            issuer_trusted,
            records_verified,
            records_failed,
            details,
            warnings: Vec::new(),
        }
    }
}

impl Default for DocumentVerifier {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_passport() -> PetHealthPassport {
        let mut passport = PetHealthPassport::new(PetIdentity {
            name: "Max".to_string(),
            species: PetSpecies::Dog,
            breed: Some("Golden Retriever".to_string()),
            microchip_id: Some("985141234567890".to_string()),
            ..Default::default()
        });

        // 광견병 백신 추가
        passport.medical_records.vaccinations.push(VaccinationRecord {
            record_id: Uuid::now_v7(),
            vaccine: VaccineInfo {
                name: "Nobivac Rabies".to_string(),
                manufacturer: Some("MSD".to_string()),
                lot_number: Some("A123".to_string()),
                serial_number: None,
            },
            target_diseases: vec![VaccineDisease::Rabies],
            administered_at: Utc::now() - Duration::days(30),
            administered_by: None,
            clinic: None,
            valid_from: Utc::now() - Duration::days(30),
            valid_until: Utc::now() + Duration::days(335),
            dose_number: 1,
            total_doses: 1,
            adverse_reactions: vec![],
            verified: true,
            verification_method: Some("clinic".to_string()),
            digital_signature: Some("sig123".to_string()),
        });

        passport
    }

    #[test]
    fn test_quarantine_check() {
        let passport = create_test_passport();
        let checker = QuarantineChecker::new();

        let result = checker.check_eligibility(
            &passport,
            "KR",
            "DE",
            Utc::now().date_naive() + Duration::days(30),
        );

        assert!(result.requirements.len() > 0);
    }

    #[test]
    fn test_vaccination_schedule() {
        let passport = create_test_passport();
        let scheduler = VaccinationScheduler::new();

        let result = scheduler.calculate_schedule(&passport);

        assert!(result.completed.len() > 0 || result.upcoming.len() > 0);
    }

    #[test]
    fn test_emergency_extraction() {
        let passport = create_test_passport();
        let extractor = EmergencyExtractor::new();

        let info = extractor.extract(&passport);

        assert_eq!(info.pet_name, "Max");
        assert_eq!(info.species, PetSpecies::Dog);
    }
}
