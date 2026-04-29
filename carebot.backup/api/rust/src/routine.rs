//! Daily routine management

use serde::{Deserialize, Serialize};
use crate::types::{ActivityType, Timestamp};

/// Daily routine for a care recipient
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DailyRoutine {
    /// Recipient ID
    pub recipient_id: String,
    /// Date
    pub date: String,
    /// Scheduled activities
    pub activities: Vec<ScheduledActivity>,
    /// Routine notes
    pub notes: Vec<String>,
}

impl DailyRoutine {
    /// Create a new daily routine
    pub fn new(recipient_id: &str) -> Self {
        Self {
            recipient_id: recipient_id.to_string(),
            date: chrono::Utc::now().format("%Y-%m-%d").to_string(),
            activities: Vec::new(),
            notes: Vec::new(),
        }
    }

    /// Add an activity to the routine
    pub fn add_activity(&mut self, time: &str, name: &str, activity_type: ActivityType) {
        self.activities.push(ScheduledActivity {
            time: time.to_string(),
            name: name.to_string(),
            activity_type,
            status: ActivityStatus::Pending,
            completed_at: None,
            notes: None,
        });
        self.activities.sort_by(|a, b| a.time.cmp(&b.time));
    }

    /// Complete an activity by time
    pub fn complete_activity(&mut self, time: &str) {
        if let Some(activity) = self.activities.iter_mut().find(|a| a.time == time) {
            activity.status = ActivityStatus::Completed;
            activity.completed_at = Some(Timestamp::now());
        }
    }

    /// Skip an activity with reason
    pub fn skip_activity(&mut self, time: &str, reason: &str) {
        if let Some(activity) = self.activities.iter_mut().find(|a| a.time == time) {
            activity.status = ActivityStatus::Skipped;
            activity.notes = Some(reason.to_string());
        }
    }

    /// Get count of completed activities
    pub fn completed_count(&self) -> usize {
        self.activities
            .iter()
            .filter(|a| a.status == ActivityStatus::Completed)
            .count()
    }

    /// Get count of pending activities
    pub fn pending_count(&self) -> usize {
        self.activities
            .iter()
            .filter(|a| a.status == ActivityStatus::Pending)
            .count()
    }

    /// Get completion percentage
    pub fn completion_percentage(&self) -> f64 {
        if self.activities.is_empty() {
            return 0.0;
        }
        (self.completed_count() as f64 / self.activities.len() as f64) * 100.0
    }

    /// Get next pending activity
    pub fn next_pending(&self) -> Option<&ScheduledActivity> {
        self.activities
            .iter()
            .find(|a| a.status == ActivityStatus::Pending)
    }

    /// Get activities by type
    pub fn activities_by_type(&self, activity_type: ActivityType) -> Vec<&ScheduledActivity> {
        self.activities
            .iter()
            .filter(|a| a.activity_type == activity_type)
            .collect()
    }
}

/// Scheduled activity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScheduledActivity {
    /// Scheduled time (HH:MM format)
    pub time: String,
    /// Activity name
    pub name: String,
    /// Activity type
    pub activity_type: ActivityType,
    /// Status
    pub status: ActivityStatus,
    /// Completion timestamp
    pub completed_at: Option<Timestamp>,
    /// Notes
    pub notes: Option<String>,
}

/// Activity status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActivityStatus {
    Pending,
    InProgress,
    Completed,
    Skipped,
    Delayed,
}

/// Routine template for creating daily routines
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RoutineTemplate {
    /// Template name
    pub name: String,
    /// Description
    pub description: Option<String>,
    /// Template activities
    pub activities: Vec<TemplateActivity>,
    /// Days this template applies to (0 = Sunday)
    pub applicable_days: Vec<u8>,
}

impl RoutineTemplate {
    /// Create a new routine template
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            description: None,
            activities: Vec::new(),
            applicable_days: vec![0, 1, 2, 3, 4, 5, 6], // All days
        }
    }

    /// Add template activity
    pub fn add_activity(&mut self, time: &str, name: &str, activity_type: ActivityType) {
        self.activities.push(TemplateActivity {
            time: time.to_string(),
            name: name.to_string(),
            activity_type,
            duration_minutes: None,
            reminder_minutes_before: Some(15),
        });
    }

    /// Generate daily routine from template
    pub fn generate_routine(&self, recipient_id: &str) -> DailyRoutine {
        let mut routine = DailyRoutine::new(recipient_id);
        for template_activity in &self.activities {
            routine.add_activity(
                &template_activity.time,
                &template_activity.name,
                template_activity.activity_type,
            );
        }
        routine
    }
}

/// Template activity definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemplateActivity {
    /// Scheduled time
    pub time: String,
    /// Activity name
    pub name: String,
    /// Activity type
    pub activity_type: ActivityType,
    /// Expected duration in minutes
    pub duration_minutes: Option<u32>,
    /// Reminder before activity (minutes)
    pub reminder_minutes_before: Option<u32>,
}

/// Default routine templates
pub fn default_elderly_routine() -> RoutineTemplate {
    let mut template = RoutineTemplate::new("기본 일과");
    template.description = Some("어르신을 위한 기본 일과표".to_string());

    template.add_activity("07:00", "기상 및 세면", ActivityType::PersonalCare);
    template.add_activity("07:30", "아침 약 복용", ActivityType::Medication);
    template.add_activity("08:00", "아침 식사", ActivityType::Meal);
    template.add_activity("09:00", "가벼운 스트레칭", ActivityType::Exercise);
    template.add_activity("10:00", "두뇌 운동", ActivityType::Cognitive);
    template.add_activity("12:00", "점심 식사", ActivityType::Meal);
    template.add_activity("12:30", "점심 약 복용", ActivityType::Medication);
    template.add_activity("14:00", "낮잠 또는 휴식", ActivityType::Rest);
    template.add_activity("15:00", "산책", ActivityType::Exercise);
    template.add_activity("16:00", "가족 영상통화", ActivityType::Social);
    template.add_activity("18:00", "저녁 식사", ActivityType::Meal);
    template.add_activity("18:30", "저녁 약 복용", ActivityType::Medication);
    template.add_activity("20:00", "TV 시청 또는 독서", ActivityType::Rest);
    template.add_activity("21:00", "취침 준비", ActivityType::PersonalCare);
    template.add_activity("21:30", "취침", ActivityType::Rest);

    template
}
