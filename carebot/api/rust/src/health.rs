//! Health monitoring and vital signs tracking

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Health monitoring data for a care recipient
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthMonitoring {
    /// Recipient ID
    pub recipient_id: String,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Vital signs
    pub vital_signs: VitalSigns,
    /// Medication adherence
    pub medication_adherence: MedicationAdherence,
    /// Activity metrics
    pub activity: ActivityMetrics,
    /// Sleep data
    pub sleep: Option<SleepData>,
    /// Nutrition data
    pub nutrition: Option<NutritionData>,
}

impl HealthMonitoring {
    /// Create new health monitoring instance
    pub fn new(recipient_id: &str) -> Self {
        Self {
            recipient_id: recipient_id.to_string(),
            timestamp: Timestamp::now(),
            vital_signs: VitalSigns::default(),
            medication_adherence: MedicationAdherence::default(),
            activity: ActivityMetrics::default(),
            sleep: None,
            nutrition: None,
        }
    }

    /// Record heart rate
    pub fn record_heart_rate(&mut self, bpm: u16) {
        self.vital_signs.heart_rate = Some(HeartRate {
            bpm,
            timestamp: Timestamp::now(),
            source: VitalSource::Wearable,
        });
    }

    /// Record blood pressure
    pub fn record_blood_pressure(&mut self, systolic: u16, diastolic: u16) {
        self.vital_signs.blood_pressure = Some(BloodPressure {
            systolic,
            diastolic,
            timestamp: Timestamp::now(),
            source: VitalSource::Device,
        });
    }

    /// Record body temperature
    pub fn record_temperature(&mut self, celsius: f32) {
        self.vital_signs.temperature = Some(Temperature {
            celsius,
            timestamp: Timestamp::now(),
            source: VitalSource::Device,
        });
    }

    /// Record blood oxygen saturation
    pub fn record_spo2(&mut self, percentage: u8) {
        self.vital_signs.spo2 = Some(SpO2 {
            percentage,
            timestamp: Timestamp::now(),
            source: VitalSource::Wearable,
        });
    }

    /// Mark medication as taken
    pub fn mark_medication_taken(&mut self, time_slot: &str) {
        match time_slot {
            "morning" => self.medication_adherence.morning_taken = true,
            "afternoon" => self.medication_adherence.afternoon_taken = true,
            "evening" => self.medication_adherence.evening_taken = true,
            "night" => self.medication_adherence.night_taken = true,
            _ => {}
        }
    }

    /// Check if all vital signs are within normal range
    pub fn check_vitals_normal(&self) -> bool {
        let hr_ok = self
            .vital_signs
            .heart_rate
            .as_ref()
            .map(|hr| hr.bpm >= 60 && hr.bpm <= 100)
            .unwrap_or(true);

        let bp_ok = self
            .vital_signs
            .blood_pressure
            .as_ref()
            .map(|bp| bp.systolic <= 140 && bp.diastolic <= 90)
            .unwrap_or(true);

        let temp_ok = self
            .vital_signs
            .temperature
            .as_ref()
            .map(|t| t.celsius >= 36.0 && t.celsius <= 37.5)
            .unwrap_or(true);

        let spo2_ok = self
            .vital_signs
            .spo2
            .as_ref()
            .map(|s| s.percentage >= 95)
            .unwrap_or(true);

        hr_ok && bp_ok && temp_ok && spo2_ok
    }

    /// Get abnormal vital signs
    pub fn get_abnormal_vitals(&self) -> Vec<String> {
        let mut abnormal = Vec::new();

        if let Some(hr) = &self.vital_signs.heart_rate {
            if hr.bpm < 60 {
                abnormal.push(format!("서맥: {}bpm", hr.bpm));
            } else if hr.bpm > 100 {
                abnormal.push(format!("빈맥: {}bpm", hr.bpm));
            }
        }

        if let Some(bp) = &self.vital_signs.blood_pressure {
            if bp.systolic > 140 || bp.diastolic > 90 {
                abnormal.push(format!("고혈압: {}/{} mmHg", bp.systolic, bp.diastolic));
            } else if bp.systolic < 90 || bp.diastolic < 60 {
                abnormal.push(format!("저혈압: {}/{} mmHg", bp.systolic, bp.diastolic));
            }
        }

        if let Some(t) = &self.vital_signs.temperature {
            if t.celsius > 37.5 {
                abnormal.push(format!("발열: {}°C", t.celsius));
            } else if t.celsius < 36.0 {
                abnormal.push(format!("저체온: {}°C", t.celsius));
            }
        }

        if let Some(s) = &self.vital_signs.spo2 {
            if s.percentage < 95 {
                abnormal.push(format!("저산소증: {}%", s.percentage));
            }
        }

        abnormal
    }
}

/// Vital signs collection
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VitalSigns {
    /// Heart rate
    pub heart_rate: Option<HeartRate>,
    /// Blood pressure
    pub blood_pressure: Option<BloodPressure>,
    /// Body temperature
    pub temperature: Option<Temperature>,
    /// Blood oxygen saturation
    pub spo2: Option<SpO2>,
    /// Blood glucose
    pub blood_glucose: Option<BloodGlucose>,
}

/// Heart rate measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeartRate {
    /// Beats per minute
    pub bpm: u16,
    /// Measurement timestamp
    pub timestamp: Timestamp,
    /// Measurement source
    pub source: VitalSource,
}

/// Blood pressure measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BloodPressure {
    /// Systolic pressure (mmHg)
    pub systolic: u16,
    /// Diastolic pressure (mmHg)
    pub diastolic: u16,
    /// Measurement timestamp
    pub timestamp: Timestamp,
    /// Measurement source
    pub source: VitalSource,
}

/// Temperature measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Temperature {
    /// Temperature in Celsius
    pub celsius: f32,
    /// Measurement timestamp
    pub timestamp: Timestamp,
    /// Measurement source
    pub source: VitalSource,
}

/// Blood oxygen saturation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpO2 {
    /// Saturation percentage
    pub percentage: u8,
    /// Measurement timestamp
    pub timestamp: Timestamp,
    /// Measurement source
    pub source: VitalSource,
}

/// Blood glucose measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BloodGlucose {
    /// Glucose level (mg/dL)
    pub mg_dl: u16,
    /// Measurement timestamp
    pub timestamp: Timestamp,
    /// Fasting measurement
    pub fasting: bool,
    /// Measurement source
    pub source: VitalSource,
}

/// Source of vital sign measurement
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VitalSource {
    /// Wearable device
    Wearable,
    /// Dedicated medical device
    Device,
    /// Manual entry
    Manual,
    /// Camera-based (rPPG)
    Camera,
}

/// Medication adherence tracking
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MedicationAdherence {
    /// Morning medication taken
    pub morning_taken: bool,
    /// Afternoon medication taken
    pub afternoon_taken: bool,
    /// Evening medication taken
    pub evening_taken: bool,
    /// Night medication taken
    pub night_taken: bool,
    /// Notes
    pub notes: Vec<String>,
}

/// Activity metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ActivityMetrics {
    /// Steps taken today
    pub steps: u32,
    /// Distance walked (meters)
    pub distance_meters: f32,
    /// Active minutes
    pub active_minutes: u32,
    /// Sedentary minutes
    pub sedentary_minutes: u32,
    /// Calories burned
    pub calories_burned: u32,
}

/// Sleep data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SleepData {
    /// Time went to bed
    pub bed_time: String,
    /// Time woke up
    pub wake_time: String,
    /// Total sleep duration (minutes)
    pub duration_minutes: u32,
    /// Sleep quality score (0.0-1.0)
    pub quality_score: f64,
    /// Number of wake-ups during night
    pub wake_ups: u32,
    /// Deep sleep minutes
    pub deep_sleep_minutes: u32,
    /// Light sleep minutes
    pub light_sleep_minutes: u32,
    /// REM sleep minutes
    pub rem_sleep_minutes: u32,
}

/// Nutrition data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NutritionData {
    /// Meals eaten today
    pub meals_eaten: u8,
    /// Water intake (ml)
    pub water_ml: u32,
    /// Appetite level (0.0-1.0)
    pub appetite_level: f64,
    /// Notes about meals
    pub notes: Vec<String>,
}
