//! WIA Refugee Credential - Core Algorithms
//!
//! 국가가 무너져도, 사람의 가치는 무너지지 않습니다.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

use std::collections::HashMap;
use chrono::{DateTime, Utc};
use uuid::Uuid;

use crate::types::*;
use crate::{get_base_confidence, fuzzy_match, calculate_year_overlap, classify_verification_level};

// ============================================================================
// Credential Manager
// ============================================================================

/// Credential Manager - CRUD operations
pub struct CredentialManager {
    credentials: HashMap<String, RefugeeCredential>,
}

impl CredentialManager {
    pub fn new() -> Self {
        Self {
            credentials: HashMap::new(),
        }
    }

    pub fn create(&mut self, credential: RefugeeCredential) -> Result<(), CredentialError> {
        if self.credentials.contains_key(&credential.id) {
            return Err(CredentialError::AlreadyExists(credential.id.clone()));
        }
        self.credentials.insert(credential.id.clone(), credential);
        Ok(())
    }

    pub fn get(&self, id: &str) -> Option<&RefugeeCredential> {
        self.credentials.get(id)
    }

    pub fn get_by_did(&self, did: &str) -> Option<&RefugeeCredential> {
        self.credentials.values().find(|c| c.identity.holder_did == did)
    }

    pub fn update(&mut self, credential: RefugeeCredential) -> Result<(), CredentialError> {
        if !self.credentials.contains_key(&credential.id) {
            return Err(CredentialError::NotFound(credential.id.clone()));
        }
        self.credentials.insert(credential.id.clone(), credential);
        Ok(())
    }

    pub fn delete(&mut self, id: &str) -> Result<(), CredentialError> {
        self.credentials.remove(id)
            .map(|_| ())
            .ok_or_else(|| CredentialError::NotFound(id.to_string()))
    }

    pub fn list(&self) -> Vec<&RefugeeCredential> {
        self.credentials.values().collect()
    }

    pub fn add_verification(&mut self, id: &str, verification: Verification) -> Result<(), CredentialError> {
        let credential = self.credentials.get_mut(id)
            .ok_or_else(|| CredentialError::NotFound(id.to_string()))?;

        credential.verifications.push(verification);

        // Update verification level if new one is higher
        let max_level = credential.verifications.iter()
            .map(|v| v.level)
            .max()
            .unwrap_or(1);

        if max_level > credential.verification_level {
            credential.verification_level = max_level;
        }

        credential.updated_at = Utc::now();
        Ok(())
    }
}

impl Default for CredentialManager {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum CredentialError {
    #[error("Credential already exists: {0}")]
    AlreadyExists(String),
    #[error("Credential not found: {0}")]
    NotFound(String),
    #[error("Invalid data: {0}")]
    InvalidData(String),
}

// ============================================================================
// Confidence Calculator
// ============================================================================

/// Confidence Calculator
pub struct ConfidenceCalculator;

impl ConfidenceCalculator {
    pub fn new() -> Self {
        Self
    }

    /// Calculate overall confidence score
    pub fn calculate(&self, credential: &RefugeeCredential) -> ConfidenceResult {
        // Base confidence from verification level
        let base_confidence = get_base_confidence(credential.verification_level);

        // Component scores
        let education_confidence = self.calculate_education_confidence(&credential.education);
        let competency_confidence = self.calculate_competency_confidence(&credential.competencies);
        let experience_confidence = self.calculate_experience_confidence(&credential.experience);

        // Weighted combination
        let component_score = education_confidence * 0.35
            + competency_confidence * 0.40
            + experience_confidence * 0.25;

        // Combine base and component
        let mut overall = base_confidence * 0.6 + component_score * 0.4;

        // Apply bonuses
        overall += self.calculate_bonus(credential);
        overall = overall.min(0.99);

        // Apply penalties
        overall -= self.calculate_penalty(credential);
        overall = overall.max(0.10);

        let classification = classify_verification_level(overall);

        ConfidenceResult {
            overall_confidence: (overall * 100.0).round() / 100.0,
            base_confidence,
            education_confidence: (education_confidence * 100.0).round() / 100.0,
            competency_confidence: (competency_confidence * 100.0).round() / 100.0,
            experience_confidence: (experience_confidence * 100.0).round() / 100.0,
            classification: format!("{:?}", classification),
        }
    }

    fn calculate_education_confidence(&self, education: &[EducationRecord]) -> f64 {
        if education.is_empty() {
            return 0.0;
        }

        let scores: Vec<f64> = education.iter().map(|edu| {
            let mut score = edu.verification.confidence_score;

            // Bonus for higher education
            match edu.level {
                EducationLevel::Doctorate | EducationLevel::PostDoctorate => score += 0.10,
                EducationLevel::Masters | EducationLevel::Professional => score += 0.08,
                EducationLevel::Bachelors => score += 0.05,
                _ => {}
            }

            // Bonus for evidence
            let evidence_bonus = (edu.evidence.len() as f64 * 0.05).min(0.15);
            score += evidence_bonus;

            score.min(1.0)
        }).collect();

        scores.iter().sum::<f64>() / scores.len() as f64
    }

    fn calculate_competency_confidence(&self, competencies: &[CompetencyRecord]) -> f64 {
        if competencies.is_empty() {
            return 0.0;
        }

        let scores: Vec<f64> = competencies.iter().map(|comp| {
            let mut score = comp.verification.confidence_score;

            // Bonus for formal assessment
            if let Some(ref assessment) = comp.assessment {
                match assessment.assessment_type {
                    AssessmentType::PracticalTest | AssessmentType::Simulation => score += 0.10,
                    _ => {}
                }

                if assessment.assessor_type == AssessorType::WIACertified {
                    score += 0.10;
                }

                if let Some(s) = assessment.score {
                    if s > 80.0 {
                        score += 0.05;
                    }
                }
            }

            score.min(1.0)
        }).collect();

        scores.iter().sum::<f64>() / scores.len() as f64
    }

    fn calculate_experience_confidence(&self, experience: &[WorkExperience]) -> f64 {
        if experience.is_empty() {
            return 0.0;
        }

        let scores: Vec<f64> = experience.iter().map(|exp| {
            let mut score = exp.verification.confidence_score;

            // Bonus for longer experience
            if exp.total_months > 60 {
                score += 0.10;
            } else if exp.total_months > 36 {
                score += 0.05;
            }

            // Bonus for evidence
            let evidence_bonus = (exp.evidence.len() as f64 * 0.05).min(0.15);
            score += evidence_bonus;

            score.min(1.0)
        }).collect();

        scores.iter().sum::<f64>() / scores.len() as f64
    }

    fn calculate_bonus(&self, credential: &RefugeeCredential) -> f64 {
        let mut bonus = 0.0;

        // Multiple verification methods
        let methods: std::collections::HashSet<_> = credential.verifications.iter()
            .map(|v| format!("{:?}", v.verification_type))
            .collect();
        if methods.len() >= 3 {
            bonus += 0.05;
        }

        // Multiple peer verifiers
        let peer_count = credential.verifications.iter()
            .filter(|v| matches!(v.verification_type, VerificationType::PeerVerification))
            .count();
        if peer_count >= 3 {
            bonus += 0.05;
        }

        bonus
    }

    fn calculate_penalty(&self, credential: &RefugeeCredential) -> f64 {
        let mut penalty = 0.0;

        // No evidence at all
        let has_any_evidence = credential.education.iter().any(|e| !e.evidence.is_empty())
            || credential.competencies.iter().any(|c| !c.evidence.is_empty());

        if !has_any_evidence {
            penalty += 0.10;
        }

        penalty
    }
}

impl Default for ConfidenceCalculator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Peer Matcher
// ============================================================================

/// Peer Matcher - Find potential verifiers
pub struct PeerMatcher {
    peer_profiles: Vec<PeerProfile>,
}

#[derive(Debug, Clone)]
pub struct PeerProfile {
    pub id: String,
    pub institutions: Vec<String>,
    pub employers: Vec<String>,
    pub fields: Vec<String>,
    pub domains: Vec<CompetencyDomain>,
    pub country: String,
    pub education_years: Vec<(u32, Option<u32>)>,
    pub verification_level: u8,
}

impl PeerMatcher {
    pub fn new() -> Self {
        Self {
            peer_profiles: Vec::new(),
        }
    }

    pub fn add_peer(&mut self, profile: PeerProfile) {
        self.peer_profiles.push(profile);
    }

    /// Find peer matches for credential verification
    pub fn find_matches(&self, credential: &RefugeeCredential) -> Vec<PeerMatch> {
        let mut matches = Vec::new();

        // Match by education
        for edu in &credential.education {
            for peer in &self.peer_profiles {
                if peer.id == credential.identity.holder_did {
                    continue; // Skip self
                }

                let match_score = self.calculate_education_match(edu, peer);
                if match_score >= 0.5 {
                    matches.push(PeerMatch {
                        peer_id: peer.id.clone(),
                        match_type: "education".to_string(),
                        match_score,
                        shared_context: PeerContext {
                            institution: Some(edu.institution_name.clone()),
                            employer: None,
                            years: Some(format!("{}-{}", edu.year_start, edu.year_end.unwrap_or(edu.year_start))),
                            field: Some(edu.field_of_study.clone()),
                        },
                    });
                }
            }
        }

        // Match by work experience
        for exp in &credential.experience {
            for peer in &self.peer_profiles {
                if peer.id == credential.identity.holder_did {
                    continue;
                }

                let match_score = self.calculate_work_match(exp, peer);
                if match_score >= 0.5 {
                    matches.push(PeerMatch {
                        peer_id: peer.id.clone(),
                        match_type: "employment".to_string(),
                        match_score,
                        shared_context: PeerContext {
                            institution: None,
                            employer: Some(exp.employer_name.clone()),
                            years: Some(format!("{}-{:?}", exp.start_date, exp.end_date)),
                            field: Some(exp.industry.clone()),
                        },
                    });
                }
            }
        }

        // Deduplicate and sort
        matches.sort_by(|a, b| b.match_score.partial_cmp(&a.match_score).unwrap());
        matches.truncate(50);
        matches
    }

    fn calculate_education_match(&self, education: &EducationRecord, peer: &PeerProfile) -> f64 {
        let mut score = 0.0;

        // Same institution (fuzzy match)
        for inst in &peer.institutions {
            if fuzzy_match(&education.institution_name, inst) > 0.8 {
                score += 0.4;
                break;
            }
        }

        // Overlapping years
        for years in &peer.education_years {
            let overlap = calculate_year_overlap(
                (education.year_start, education.year_end),
                *years
            );
            if overlap > 0.3 {
                score += overlap * 0.3;
                break;
            }
        }

        // Same field
        if peer.fields.iter().any(|f| f.to_lowercase() == education.field_of_study.to_lowercase()) {
            score += 0.2;
        }

        // Same country
        if peer.country == education.institution_country {
            score += 0.1;
        }

        score.min(1.0)
    }

    fn calculate_work_match(&self, experience: &WorkExperience, peer: &PeerProfile) -> f64 {
        let mut score = 0.0;

        // Same employer (fuzzy match)
        for emp in &peer.employers {
            if fuzzy_match(&experience.employer_name, emp) > 0.8 {
                score += 0.5;
                break;
            }
        }

        // Same country
        if peer.country == experience.employer_country {
            score += 0.2;
        }

        // Same industry/domain
        for domain in &peer.domains {
            if matches_job_to_domain(&experience.job_category, domain) {
                score += 0.3;
                break;
            }
        }

        score.min(1.0)
    }
}

impl Default for PeerMatcher {
    fn default() -> Self {
        Self::new()
    }
}

fn matches_job_to_domain(job: &JobCategory, domain: &CompetencyDomain) -> bool {
    matches!(
        (job, domain),
        (JobCategory::Medical, CompetencyDomain::Medicine) |
        (JobCategory::Medical, CompetencyDomain::Nursing) |
        (JobCategory::Academic, CompetencyDomain::Teaching) |
        (JobCategory::Legal, CompetencyDomain::Law) |
        (JobCategory::Technical, CompetencyDomain::SoftwareEngineering) |
        (JobCategory::Technical, CompetencyDomain::IT)
    )
}

// ============================================================================
// Assessment Evaluator
// ============================================================================

/// Assessment Evaluator
pub struct AssessmentEvaluator;

impl AssessmentEvaluator {
    pub fn new() -> Self {
        Self
    }

    /// Evaluate assessment result
    pub fn evaluate(
        &self,
        raw_score: f64,
        max_score: f64,
        claimed_level: ProficiencyLevel
    ) -> AssessmentResult {
        let normalized_score = (raw_score / max_score) * 100.0;

        // Determine achieved level
        let achieved_level = if normalized_score >= 95.0 {
            ProficiencyLevel::Master
        } else if normalized_score >= 85.0 {
            ProficiencyLevel::Expert
        } else if normalized_score >= 70.0 {
            ProficiencyLevel::Proficient
        } else if normalized_score >= 55.0 {
            ProficiencyLevel::Competent
        } else if normalized_score >= 40.0 {
            ProficiencyLevel::Beginner
        } else {
            ProficiencyLevel::Novice
        };

        // Compare with claimed
        let claimed_idx = level_to_index(&claimed_level);
        let achieved_idx = level_to_index(&achieved_level);

        let (verification_status, confidence) = if achieved_idx >= claimed_idx {
            ("verified".to_string(), 0.8 + (normalized_score - 70.0) / 100.0 * 0.15)
        } else if achieved_idx == claimed_idx - 1 {
            ("partially_verified".to_string(), 0.6)
        } else {
            ("not_verified".to_string(), 0.3)
        };

        AssessmentResult {
            raw_score,
            normalized_score,
            claimed_level,
            achieved_level,
            verification_status,
            confidence: confidence.min(0.95),
        }
    }
}

impl Default for AssessmentEvaluator {
    fn default() -> Self {
        Self::new()
    }
}

fn level_to_index(level: &ProficiencyLevel) -> i32 {
    match level {
        ProficiencyLevel::Novice => 0,
        ProficiencyLevel::Beginner => 1,
        ProficiencyLevel::Competent => 2,
        ProficiencyLevel::Proficient => 3,
        ProficiencyLevel::Expert => 4,
        ProficiencyLevel::Master => 5,
    }
}

// ============================================================================
// Document Verifier
// ============================================================================

/// Document Verifier (placeholder for AI verification)
pub struct DocumentVerifier;

impl DocumentVerifier {
    pub fn new() -> Self {
        Self
    }

    /// Verify document authenticity
    pub fn verify(&self, evidence: &Evidence) -> DocumentVerificationResult {
        // Placeholder - in production would use AI models

        let authenticity_score = evidence.authenticity_score.unwrap_or(0.5);
        let tampering_detected = authenticity_score < 0.3;
        let text_verified = evidence.evidence_type != EvidenceType::Other;

        let confidence = if tampering_detected {
            0.1
        } else if text_verified {
            authenticity_score * 0.9
        } else {
            authenticity_score * 0.5
        };

        DocumentVerificationResult {
            document_id: evidence.id.clone(),
            authenticity_score,
            tampering_detected,
            text_verified,
            confidence,
        }
    }
}

impl Default for DocumentVerifier {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_credential_manager() {
        let mut manager = CredentialManager::new();

        let credential = create_test_credential();
        let id = credential.id.clone();

        assert!(manager.create(credential).is_ok());
        assert!(manager.get(&id).is_some());
        assert!(manager.delete(&id).is_ok());
        assert!(manager.get(&id).is_none());
    }

    fn create_test_credential() -> RefugeeCredential {
        RefugeeCredential {
            id: Uuid::now_v7().to_string(),
            version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: Utc::now(),
            identity: HolderIdentity {
                holder_did: "did:wia:test123".to_string(),
                name_hash: "hash".to_string(),
                display_name: Some("Test User".to_string()),
                photo_hash: None,
                birth_year: 1985,
                gender: Some(Gender::Male),
                nationality_claimed: "SY".to_string(),
                refugee_status: RefugeeStatus::SelfDeclared,
                displacement_date: None,
                displacement_reason: Some(DisplacementReason::War),
                current_country: "DE".to_string(),
                anonymous_mode: false,
            },
            education: vec![],
            competencies: vec![],
            languages: vec![],
            experience: vec![],
            verification_level: 1,
            verifications: vec![],
            signature: None,
        }
    }
}
