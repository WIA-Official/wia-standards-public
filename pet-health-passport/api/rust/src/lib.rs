//! # WIA Pet Health Passport
//!
//! 전 세계 어디서든 인정되는 반려동물 건강 기록 표준
//!
//! ## 철학
//!
//! **홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라
//!
//! ## 주요 기능
//!
//! - 반려동물 신원 및 건강 기록 관리
//! - 백신 접종 일정 계산
//! - 국제 여행 검역 적격성 판정
//! - 응급 정보 빠른 추출
//! - REST/WebSocket API
//!
//! ## Example
//!
//! ```rust
//! use wia_pet_passport::{PetPassport, PetIdentity, PetSpecies};
//!
//! let passport = PetPassport::new(PetIdentity {
//!     name: "Max".to_string(),
//!     species: PetSpecies::Dog,
//!     breed: Some("Golden Retriever".to_string()),
//!     ..Default::default()
//! });
//!
//! // 검역 적격성 확인
//! let eligibility = passport.check_travel_eligibility("KR", "DE", "2025-06-01");
//! println!("Travel eligible: {}", eligibility.eligible);
//! ```

pub mod types;
pub mod core;
pub mod server;

// Re-exports
pub use types::{
    PetHealthPassport, PetIdentity, PetSpecies, PetSex,
    MicrochipInfo, GuardianInfo, EmergencyContact,
    VaccinationRecord, VaccineDisease, SurgeryRecord, ProcedureType,
    DiagnosticRecord, DiagnosticTestType, AllergyRecord, AllergenType,
    ChronicCondition, MedicationRecord,
    GeneticProfile, GeneticHealthMarker,
    TravelRecord, TravelDocument, CountryRequirements,
};
pub use core::{
    PassportManager, QuarantineChecker, VaccinationScheduler,
    HealthRiskAssessor, EmergencyExtractor, DocumentVerifier,
    EligibilityResult, RequirementCheck, VaccinationScheduleResult,
    HealthRiskResult, EmergencyInfo, VerificationResult,
};
pub use server::{create_app, run_server, ServerConfig};

/// WIA Pet Health Passport 버전
pub const VERSION: &str = "1.0.0";

/// 지원하는 마이크로칩 표준
pub const MICROCHIP_STANDARD: &str = "ISO 11784/11785";

/// 지원 국가 수
pub const SUPPORTED_COUNTRIES: usize = 193;

/// 마이크로칩 ID 유효성 검사
///
/// ISO 11784/11785 표준: 15자리 숫자
pub fn validate_microchip_id(chip_id: &str) -> bool {
    if chip_id.len() != 15 {
        return false;
    }
    chip_id.chars().all(|c| c.is_ascii_digit())
}

/// ISO 3166-1 alpha-2 국가 코드 유효성 검사
pub fn validate_country_code(code: &str) -> bool {
    code.len() == 2 && code.chars().all(|c| c.is_ascii_uppercase())
}

/// 백신 만료까지 남은 일수 계산
pub fn days_until_expiry(valid_until: &str) -> Option<i64> {
    use chrono::{NaiveDate, Utc};

    let expiry = NaiveDate::parse_from_str(valid_until, "%Y-%m-%d").ok()?;
    let today = Utc::now().date_naive();
    Some((expiry - today).num_days())
}

/// 반려동물 나이 계산 (년/월)
pub fn calculate_pet_age(date_of_birth: &str) -> Option<(u32, u32)> {
    use chrono::{NaiveDate, Utc};

    let birth = NaiveDate::parse_from_str(date_of_birth, "%Y-%m-%d").ok()?;
    let today = Utc::now().date_naive();

    let years = (today.year() - birth.year()) as u32;
    let months = if today.month() >= birth.month() {
        today.month() - birth.month()
    } else {
        12 - (birth.month() - today.month())
    };

    Some((years, months))
}

/// 품종별 평균 수명 (년)
pub fn get_breed_life_expectancy(species: PetSpecies, breed: &str) -> Option<u32> {
    match species {
        PetSpecies::Dog => {
            match breed.to_lowercase().as_str() {
                "golden retriever" => Some(12),
                "german shepherd" => Some(11),
                "labrador retriever" => Some(12),
                "bulldog" => Some(8),
                "poodle" => Some(14),
                "chihuahua" => Some(16),
                "beagle" => Some(13),
                _ => Some(12), // 평균
            }
        }
        PetSpecies::Cat => {
            match breed.to_lowercase().as_str() {
                "persian" => Some(15),
                "siamese" => Some(15),
                "maine coon" => Some(13),
                "ragdoll" => Some(15),
                "british shorthair" => Some(15),
                _ => Some(15), // 평균
            }
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_microchip_id() {
        assert!(validate_microchip_id("985141234567890"));
        assert!(!validate_microchip_id("12345"));           // 너무 짧음
        assert!(!validate_microchip_id("98514123456789A")); // 문자 포함
    }

    #[test]
    fn test_validate_country_code() {
        assert!(validate_country_code("KR"));
        assert!(validate_country_code("US"));
        assert!(validate_country_code("DE"));
        assert!(!validate_country_code("Korea")); // 너무 김
        assert!(!validate_country_code("kr"));    // 소문자
    }

    #[test]
    fn test_days_until_expiry() {
        // 미래 날짜
        let days = days_until_expiry("2030-12-31");
        assert!(days.is_some());
        assert!(days.unwrap() > 0);
    }

    #[test]
    fn test_calculate_pet_age() {
        let age = calculate_pet_age("2022-03-15");
        assert!(age.is_some());
        let (years, _months) = age.unwrap();
        assert!(years >= 2);
    }

    #[test]
    fn test_breed_life_expectancy() {
        assert_eq!(
            get_breed_life_expectancy(PetSpecies::Dog, "Golden Retriever"),
            Some(12)
        );
        assert_eq!(
            get_breed_life_expectancy(PetSpecies::Cat, "Persian"),
            Some(15)
        );
    }
}
