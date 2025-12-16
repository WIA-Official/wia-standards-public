//! Analytics and Telemetry
//!
//! Privacy-preserving analytics for accessibility feature usage.

use super::GameAction;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};
use uuid::Uuid;

/// Analytics collector for accessibility usage
#[derive(Debug)]
pub struct AnalyticsCollector {
    config: AnalyticsConfig,
    current_session: Option<SessionData>,
    feature_usage: HashMap<AccessibilityFeature, FeatureMetrics>,
    action_history: Vec<ActionRecord>,
}

/// Analytics configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalyticsConfig {
    /// Consent for anonymous usage
    pub anonymous_usage: bool,
    /// Consent for feature effectiveness tracking
    pub feature_effectiveness: bool,
    /// Consent for crash reports
    pub crash_reports: bool,
    /// Consent for detailed input logs
    pub detailed_input_logs: bool,
    /// Share with game developers
    pub share_with_developers: bool,
    /// Data retention period (days)
    pub data_retention_days: u32,
    /// Allow deletion on request
    pub deletion_on_request: bool,
}

/// Session data
#[derive(Debug, Clone)]
struct SessionData {
    id: Uuid,
    start_time: Instant,
    profile_id: Option<Uuid>,
    game_id: Option<String>,
    input_sources: HashSet<InputSourceMetric>,
    features_used: HashSet<AccessibilityFeature>,
    action_count: u32,
    success_count: u32,
    failure_count: u32,
    fatigue_events: u32,
    pause_reasons: Vec<String>,
}

/// Accessibility features for tracking
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AccessibilityFeature {
    EyeGazeAiming,
    EyeGazeDwell,
    BCIMotorImagery,
    BCISSVEP,
    BCIP300,
    VoiceCommands,
    SymbolSelection,
    WheelchairTilt,
    WheelchairComfortSync,
    HapticFeedback,
    SpatialHaptic,
    SmartHomeLighting,
    Copilot,
    SwitchAccess,
    MacroSystem,
    AimAssist,
    AutoAim,
    OneHandedMode,
    ColorBlindMode,
    ScreenReader,
    Subtitles,
    SlowMotion,
}

/// Input source for metrics
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputSourceMetric {
    EyeTracker,
    BCI,
    Voice,
    Symbol,
    Wheelchair,
    XboxController,
    XboxAdaptive,
    PlayStationAccess,
    Switch,
    Keyboard,
    Mouse,
}

/// Metrics for a feature
#[derive(Debug, Clone, Default)]
struct FeatureMetrics {
    total_usage_time: Duration,
    usage_count: u32,
    success_count: u32,
    adjustment_count: u32,
    last_used: Option<Instant>,
}

/// Record of an action
#[derive(Debug, Clone)]
struct ActionRecord {
    timestamp: Instant,
    action_type: String,
    source: Option<InputSourceMetric>,
    success: bool,
    latency_ms: Option<u32>,
}

/// Summary of analytics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalyticsSummary {
    pub session_count: u32,
    pub total_play_time_secs: u64,
    pub most_used_features: Vec<(String, u32)>,
    pub input_source_distribution: HashMap<String, f32>,
    pub average_success_rate: f32,
    pub average_latency_ms: f32,
    pub feature_effectiveness: Vec<FeatureEffectiveness>,
}

/// Feature effectiveness data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureEffectiveness {
    pub feature: String,
    pub usage_time_percent: f32,
    pub success_rate: f32,
    pub user_adjustments: u32,
}

/// Session metrics for export
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionMetrics {
    pub session_id: String,
    pub duration_secs: u64,
    pub input_sources: Vec<String>,
    pub features_used: Vec<String>,
    pub action_count: u32,
    pub success_rate: f32,
    pub fatigue_events: u32,
}

impl AnalyticsCollector {
    /// Create a new analytics collector
    pub fn new() -> Self {
        Self {
            config: AnalyticsConfig::default(),
            current_session: None,
            feature_usage: HashMap::new(),
            action_history: Vec::new(),
        }
    }

    /// Configure analytics
    pub fn configure(&mut self, config: AnalyticsConfig) {
        self.config = config;
    }

    /// Start a new session
    pub fn start_session(&mut self, profile_id: Option<Uuid>, game_id: Option<String>) {
        self.current_session = Some(SessionData {
            id: Uuid::new_v4(),
            start_time: Instant::now(),
            profile_id,
            game_id,
            input_sources: HashSet::new(),
            features_used: HashSet::new(),
            action_count: 0,
            success_count: 0,
            failure_count: 0,
            fatigue_events: 0,
            pause_reasons: Vec::new(),
        });
    }

    /// End current session
    pub fn end_session(&mut self) -> Option<SessionMetrics> {
        let session = self.current_session.take()?;

        let duration = session.start_time.elapsed();
        let total_actions = session.success_count + session.failure_count;
        let success_rate = if total_actions > 0 {
            session.success_count as f32 / total_actions as f32
        } else {
            0.0
        };

        Some(SessionMetrics {
            session_id: session.id.to_string(),
            duration_secs: duration.as_secs(),
            input_sources: session.input_sources.iter()
                .map(|s| format!("{:?}", s))
                .collect(),
            features_used: session.features_used.iter()
                .map(|f| format!("{:?}", f))
                .collect(),
            action_count: session.action_count,
            success_rate,
            fatigue_events: session.fatigue_events,
        })
    }

    /// Record feature usage
    pub fn record_feature_use(&mut self, feature: AccessibilityFeature) {
        if !self.config.anonymous_usage {
            return;
        }

        if let Some(ref mut session) = self.current_session {
            session.features_used.insert(feature);
        }

        let metrics = self.feature_usage
            .entry(feature)
            .or_insert_with(FeatureMetrics::default);
        metrics.usage_count += 1;
        metrics.last_used = Some(Instant::now());
    }

    /// Record input source usage
    pub fn record_input_source(&mut self, source: InputSourceMetric) {
        if let Some(ref mut session) = self.current_session {
            session.input_sources.insert(source);
        }
    }

    /// Record an action
    pub fn record_action(&mut self, action: GameAction) {
        if !self.config.anonymous_usage {
            return;
        }

        if let Some(ref mut session) = self.current_session {
            session.action_count += 1;
            session.success_count += 1; // Assume success for now
        }

        if self.config.detailed_input_logs {
            self.action_history.push(ActionRecord {
                timestamp: Instant::now(),
                action_type: format!("{:?}", action),
                source: None,
                success: true,
                latency_ms: None,
            });

            // Limit history size
            if self.action_history.len() > 1000 {
                self.action_history.drain(0..500);
            }
        }
    }

    /// Record action result
    pub fn record_action_result(&mut self, success: bool, latency_ms: Option<u32>) {
        if let Some(ref mut session) = self.current_session {
            if success {
                session.success_count += 1;
            } else {
                session.failure_count += 1;
            }
        }

        if let Some(last) = self.action_history.last_mut() {
            last.success = success;
            last.latency_ms = latency_ms;
        }
    }

    /// Record fatigue event
    pub fn record_fatigue(&mut self) {
        if let Some(ref mut session) = self.current_session {
            session.fatigue_events += 1;
        }
    }

    /// Record pause reason
    pub fn record_pause(&mut self, reason: String) {
        if let Some(ref mut session) = self.current_session {
            session.pause_reasons.push(reason);
        }
    }

    /// Record feature adjustment
    pub fn record_adjustment(&mut self, feature: AccessibilityFeature) {
        if let Some(metrics) = self.feature_usage.get_mut(&feature) {
            metrics.adjustment_count += 1;
        }
    }

    /// Get analytics summary
    pub fn get_summary(&self) -> AnalyticsSummary {
        let mut most_used: Vec<_> = self.feature_usage.iter()
            .map(|(f, m)| (format!("{:?}", f), m.usage_count))
            .collect();
        most_used.sort_by(|a, b| b.1.cmp(&a.1));
        most_used.truncate(10);

        let feature_effectiveness: Vec<_> = self.feature_usage.iter()
            .map(|(f, m)| {
                let total = m.usage_count.max(1);
                FeatureEffectiveness {
                    feature: format!("{:?}", f),
                    usage_time_percent: m.total_usage_time.as_secs_f32() / 3600.0 * 100.0,
                    success_rate: m.success_count as f32 / total as f32,
                    user_adjustments: m.adjustment_count,
                }
            })
            .collect();

        AnalyticsSummary {
            session_count: 1,
            total_play_time_secs: self.current_session
                .as_ref()
                .map(|s| s.start_time.elapsed().as_secs())
                .unwrap_or(0),
            most_used_features: most_used,
            input_source_distribution: HashMap::new(),
            average_success_rate: self.current_session
                .as_ref()
                .map(|s| {
                    let total = (s.success_count + s.failure_count).max(1);
                    s.success_count as f32 / total as f32
                })
                .unwrap_or(0.0),
            average_latency_ms: 0.0,
            feature_effectiveness,
        }
    }

    /// Export data for deletion request
    pub fn export_user_data(&self) -> serde_json::Value {
        serde_json::json!({
            "feature_usage": self.feature_usage.iter()
                .map(|(f, m)| (format!("{:?}", f), m.usage_count))
                .collect::<HashMap<_, _>>(),
            "action_count": self.action_history.len(),
            "consent": {
                "anonymous_usage": self.config.anonymous_usage,
                "feature_effectiveness": self.config.feature_effectiveness,
                "detailed_logs": self.config.detailed_input_logs,
            }
        })
    }

    /// Delete all user data
    pub fn delete_user_data(&mut self) {
        self.feature_usage.clear();
        self.action_history.clear();
        self.current_session = None;
    }

    /// Check if data collection is enabled
    pub fn is_enabled(&self) -> bool {
        self.config.anonymous_usage
    }
}

impl Default for AnalyticsCollector {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for AnalyticsConfig {
    fn default() -> Self {
        Self {
            anonymous_usage: true,
            feature_effectiveness: true,
            crash_reports: true,
            detailed_input_logs: false,
            share_with_developers: false,
            data_retention_days: 30,
            deletion_on_request: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analytics_creation() {
        let collector = AnalyticsCollector::new();
        assert!(collector.is_enabled());
    }

    #[test]
    fn test_start_session() {
        let mut collector = AnalyticsCollector::new();
        collector.start_session(None, Some("test_game".to_string()));

        assert!(collector.current_session.is_some());
    }

    #[test]
    fn test_record_feature() {
        let mut collector = AnalyticsCollector::new();
        collector.start_session(None, None);
        collector.record_feature_use(AccessibilityFeature::EyeGazeAiming);

        let session = collector.current_session.as_ref().unwrap();
        assert!(session.features_used.contains(&AccessibilityFeature::EyeGazeAiming));
    }

    #[test]
    fn test_end_session() {
        let mut collector = AnalyticsCollector::new();
        collector.start_session(None, None);
        collector.record_feature_use(AccessibilityFeature::VoiceCommands);

        let metrics = collector.end_session();
        assert!(metrics.is_some());
        assert!(collector.current_session.is_none());
    }

    #[test]
    fn test_delete_data() {
        let mut collector = AnalyticsCollector::new();
        collector.start_session(None, None);
        collector.record_feature_use(AccessibilityFeature::HapticFeedback);

        collector.delete_user_data();
        assert!(collector.feature_usage.is_empty());
    }

    #[test]
    fn test_disabled_collection() {
        let mut collector = AnalyticsCollector::new();
        collector.config.anonymous_usage = false;
        collector.start_session(None, None);
        collector.record_feature_use(AccessibilityFeature::AimAssist);

        // Should not record when disabled
        assert!(collector.feature_usage.is_empty());
    }
}
