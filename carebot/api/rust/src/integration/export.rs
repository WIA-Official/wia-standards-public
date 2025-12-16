//! Data export and reporting

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Export format options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExportFormat {
    Json,
    Csv,
    Pdf,
    Html,
    FhirJson,
    Xml,
}

/// Daily summary report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DailySummary {
    /// Report date
    pub date: String,
    /// Recipient ID
    pub recipient_id: String,
    /// Recipient name
    pub recipient_name: String,
    /// Activity summary
    pub activities: ActivitySummary,
    /// Health summary
    pub health: HealthSummary,
    /// Emotion summary
    pub emotion: EmotionSummary,
    /// Medication summary
    pub medication: MedicationSummary,
    /// Conversation highlights
    pub conversation_highlights: Vec<String>,
    /// Notable events
    pub events: Vec<NotableEvent>,
    /// AI recommendations
    pub recommendations: Vec<String>,
    /// Generated timestamp
    pub generated_at: Timestamp,
}

impl DailySummary {
    /// Create a new daily summary
    pub fn new(date: &str, recipient_id: &str, recipient_name: &str) -> Self {
        Self {
            date: date.to_string(),
            recipient_id: recipient_id.to_string(),
            recipient_name: recipient_name.to_string(),
            activities: ActivitySummary::default(),
            health: HealthSummary::default(),
            emotion: EmotionSummary::default(),
            medication: MedicationSummary::default(),
            conversation_highlights: Vec::new(),
            events: Vec::new(),
            recommendations: Vec::new(),
            generated_at: Timestamp::now(),
        }
    }

    /// Add a conversation highlight
    pub fn add_highlight(&mut self, highlight: &str) {
        self.conversation_highlights.push(highlight.to_string());
    }

    /// Add a notable event
    pub fn add_event(&mut self, event: NotableEvent) {
        self.events.push(event);
    }

    /// Add a recommendation
    pub fn add_recommendation(&mut self, recommendation: &str) {
        self.recommendations.push(recommendation.to_string());
    }

    /// Generate simple text summary
    pub fn to_text(&self) -> String {
        format!(
            r#"{} 돌봄 리포트
======================
이름: {}
날짜: {}

📊 활동 요약
- 걸음 수: {} 걸음
- 활동 시간: {} 분
- 수면 시간: {} 시간

💊 약 복용
- 복용률: {}%
- 미복용: {}회

❤️ 건강 상태
- 평균 심박수: {} bpm
- 혈압: {}/{}
- 산소포화도: {}%

😊 감정 상태
- 긍정 비율: {}%
- 주요 감정: {}

💬 대화 하이라이트
{}

📋 권장 사항
{}

생성 시간: {}
"#,
            self.recipient_name,
            self.recipient_name,
            self.date,
            self.activities.total_steps,
            self.activities.active_minutes,
            self.activities.sleep_hours,
            self.medication.adherence_rate,
            self.medication.missed_doses,
            self.health.avg_heart_rate.unwrap_or(0),
            self.health.avg_systolic.unwrap_or(0),
            self.health.avg_diastolic.unwrap_or(0),
            self.health.avg_spo2.unwrap_or(0),
            self.emotion.positive_ratio,
            self.emotion.dominant_emotion.as_deref().unwrap_or("보통"),
            self.conversation_highlights
                .iter()
                .map(|h| format!("  - {}", h))
                .collect::<Vec<_>>()
                .join("\n"),
            self.recommendations
                .iter()
                .map(|r| format!("  - {}", r))
                .collect::<Vec<_>>()
                .join("\n"),
            chrono::Utc::now().format("%Y-%m-%d %H:%M")
        )
    }
}

/// Activity summary
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ActivitySummary {
    /// Total steps
    pub total_steps: u32,
    /// Active minutes
    pub active_minutes: u32,
    /// Sedentary minutes
    pub sedentary_minutes: u32,
    /// Sleep hours
    pub sleep_hours: f32,
    /// Sleep quality (0-100)
    pub sleep_quality: u8,
    /// Outdoor time in minutes
    pub outdoor_minutes: u32,
    /// Social interactions count
    pub social_interactions: u32,
    /// Completed routine activities
    pub completed_activities: u32,
    /// Total scheduled activities
    pub total_activities: u32,
}

impl ActivitySummary {
    /// Calculate activity completion rate
    pub fn completion_rate(&self) -> f64 {
        if self.total_activities == 0 {
            return 0.0;
        }
        (self.completed_activities as f64 / self.total_activities as f64) * 100.0
    }
}

/// Health summary
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HealthSummary {
    /// Average heart rate
    pub avg_heart_rate: Option<u16>,
    /// Min heart rate
    pub min_heart_rate: Option<u16>,
    /// Max heart rate
    pub max_heart_rate: Option<u16>,
    /// Average systolic BP
    pub avg_systolic: Option<u16>,
    /// Average diastolic BP
    pub avg_diastolic: Option<u16>,
    /// Average SpO2
    pub avg_spo2: Option<u8>,
    /// Average temperature
    pub avg_temperature: Option<f32>,
    /// Abnormal readings count
    pub abnormal_readings: u32,
    /// Health alerts triggered
    pub health_alerts: u32,
}

/// Emotion summary
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmotionSummary {
    /// Dominant emotion
    pub dominant_emotion: Option<String>,
    /// Positive emotion ratio (0-100)
    pub positive_ratio: u8,
    /// Negative emotion ratio (0-100)
    pub negative_ratio: u8,
    /// Emotion distribution
    pub distribution: Vec<EmotionCount>,
    /// Emotional concerns detected
    pub concerns_detected: u32,
}

/// Emotion count for distribution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionCount {
    pub emotion: String,
    pub count: u32,
    pub percentage: f64,
}

/// Medication summary
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MedicationSummary {
    /// Total scheduled doses
    pub total_doses: u32,
    /// Taken doses
    pub taken_doses: u32,
    /// Missed doses
    pub missed_doses: u32,
    /// Adherence rate (0-100)
    pub adherence_rate: u8,
    /// Late doses
    pub late_doses: u32,
}

impl MedicationSummary {
    /// Calculate adherence rate
    pub fn calculate_adherence(&mut self) {
        if self.total_doses == 0 {
            self.adherence_rate = 100;
        } else {
            self.adherence_rate = ((self.taken_doses as f64 / self.total_doses as f64) * 100.0) as u8;
        }
    }
}

/// Notable event in the day
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotableEvent {
    /// Event time
    pub time: String,
    /// Event type
    pub event_type: String,
    /// Description
    pub description: String,
    /// Severity
    pub severity: String,
    /// Resolution
    pub resolution: Option<String>,
}

impl NotableEvent {
    /// Create a fall event
    pub fn fall(time: &str, location: &str, resolved: bool) -> Self {
        Self {
            time: time.to_string(),
            event_type: "낙상".to_string(),
            description: format!("{}에서 낙상 감지", location),
            severity: "긴급".to_string(),
            resolution: if resolved {
                Some("자력 회복 확인".to_string())
            } else {
                None
            },
        }
    }

    /// Create a medication missed event
    pub fn medication_missed(time: &str, medication: &str) -> Self {
        Self {
            time: time.to_string(),
            event_type: "약 미복용".to_string(),
            description: format!("{} 복용 누락", medication),
            severity: "주의".to_string(),
            resolution: None,
        }
    }

    /// Create an emotional concern event
    pub fn emotional_concern(time: &str, emotion: &str) -> Self {
        Self {
            time: time.to_string(),
            event_type: "감정 우려".to_string(),
            description: format!("{}한 감정 상태 감지", emotion),
            severity: "알림".to_string(),
            resolution: None,
        }
    }
}

/// Weekly report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeeklyReport {
    /// Report week (YYYY-WXX format)
    pub week: String,
    /// Start date
    pub start_date: String,
    /// End date
    pub end_date: String,
    /// Recipient info
    pub recipient_id: String,
    pub recipient_name: String,
    /// Daily summaries
    pub daily_summaries: Vec<DailySummary>,
    /// Weekly trends
    pub trends: WeeklyTrends,
    /// Weekly recommendations
    pub recommendations: Vec<String>,
    /// Generated timestamp
    pub generated_at: Timestamp,
}

/// Weekly trends
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WeeklyTrends {
    /// Activity trend (increasing, decreasing, stable)
    pub activity_trend: String,
    /// Average daily steps
    pub avg_daily_steps: u32,
    /// Sleep quality trend
    pub sleep_trend: String,
    /// Average sleep hours
    pub avg_sleep_hours: f32,
    /// Medication adherence trend
    pub medication_trend: String,
    /// Average adherence rate
    pub avg_adherence_rate: u8,
    /// Emotional wellbeing trend
    pub emotion_trend: String,
    /// Average positive ratio
    pub avg_positive_ratio: u8,
    /// Health stability
    pub health_stability: String,
}

/// Export request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportRequest {
    /// Export format
    pub format: ExportFormat,
    /// Date range start
    pub start_date: String,
    /// Date range end
    pub end_date: String,
    /// Recipient ID
    pub recipient_id: String,
    /// Data types to include
    pub include: Vec<ExportDataType>,
    /// Requested by
    pub requested_by: String,
    /// Delivery method
    pub delivery: DeliveryMethod,
}

/// Data types that can be exported
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExportDataType {
    Vitals,
    Activities,
    Medications,
    Emotions,
    Conversations,
    SafetyEvents,
    CognitiveAssessments,
}

/// Delivery method for exports
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMethod {
    Download,
    Email { address: String },
    FhirServer { url: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_daily_summary_creation() {
        let mut summary = DailySummary::new("2024-01-15", "recipient-001", "김영희");
        summary.add_highlight("오늘 날씨가 좋다고 기분이 좋으셨습니다");
        summary.add_recommendation("오후 산책을 권장합니다");

        assert_eq!(summary.recipient_name, "김영희");
        assert_eq!(summary.conversation_highlights.len(), 1);
        assert_eq!(summary.recommendations.len(), 1);
    }

    #[test]
    fn test_activity_completion_rate() {
        let mut activity = ActivitySummary::default();
        activity.completed_activities = 8;
        activity.total_activities = 10;

        assert!((activity.completion_rate() - 80.0).abs() < 0.01);
    }

    #[test]
    fn test_medication_adherence() {
        let mut med = MedicationSummary::default();
        med.total_doses = 4;
        med.taken_doses = 3;
        med.missed_doses = 1;
        med.calculate_adherence();

        assert_eq!(med.adherence_rate, 75);
    }

    #[test]
    fn test_notable_events() {
        let fall = NotableEvent::fall("10:30", "화장실", true);
        assert_eq!(fall.event_type, "낙상");
        assert!(fall.resolution.is_some());

        let missed = NotableEvent::medication_missed("08:00", "혈압약");
        assert_eq!(missed.severity, "주의");
    }
}
