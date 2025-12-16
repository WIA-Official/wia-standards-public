//! Session Management
//!
//! Manages XR session timing, breaks, and health monitoring.

use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, RwLock};
use tokio::time::interval;

use crate::types::*;

/// Session state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    /// Session not started
    NotStarted,
    /// Session is active
    Active,
    /// Session is paused
    Paused,
    /// On mandatory break
    OnBreak,
    /// Session ended
    Ended,
}

/// Session event
#[derive(Debug, Clone)]
pub enum SessionEvent {
    Started,
    Paused,
    Resumed,
    BreakStarted { duration_minutes: u32 },
    BreakEnded,
    ReminderTriggered { minutes_active: u32 },
    WarningTriggered { minutes_remaining: u32 },
    LimitReached,
    Ended { total_minutes: u32 },
    SafeSpaceEntered,
    SafeSpaceExited,
}

/// Session statistics
#[derive(Debug, Clone, Default)]
pub struct SessionStats {
    /// Total session time in seconds
    pub total_time_seconds: u64,
    /// Active time in seconds
    pub active_time_seconds: u64,
    /// Number of breaks taken
    pub breaks_taken: u32,
    /// Total break time in seconds
    pub break_time_seconds: u64,
    /// Number of safe space visits
    pub safe_space_visits: u32,
    /// Number of rest reminders shown
    pub reminders_shown: u32,
}

/// Session manager for tracking XR usage
pub struct SessionManager {
    settings: SessionSettings,
    state: Arc<RwLock<SessionState>>,
    stats: Arc<RwLock<SessionStats>>,
    start_time: Arc<RwLock<Option<Instant>>>,
    active_start: Arc<RwLock<Option<Instant>>>,
    last_reminder: Arc<RwLock<Option<Instant>>>,
    event_sender: Option<mpsc::Sender<SessionEvent>>,
    reminder_interval_minutes: u32,
}

impl SessionManager {
    /// Create a new session manager
    pub fn new(settings: SessionSettings) -> Self {
        Self {
            settings,
            state: Arc::new(RwLock::new(SessionState::NotStarted)),
            stats: Arc::new(RwLock::new(SessionStats::default())),
            start_time: Arc::new(RwLock::new(None)),
            active_start: Arc::new(RwLock::new(None)),
            last_reminder: Arc::new(RwLock::new(None)),
            event_sender: None,
            reminder_interval_minutes: 30,
        }
    }

    /// Create with event channel
    pub fn with_events(settings: SessionSettings) -> (Self, mpsc::Receiver<SessionEvent>) {
        let (tx, rx) = mpsc::channel(100);
        let mut manager = Self::new(settings);
        manager.event_sender = Some(tx);
        (manager, rx)
    }

    /// Set reminder interval
    pub fn set_reminder_interval(&mut self, minutes: u32) {
        self.reminder_interval_minutes = minutes;
    }

    /// Start a new session
    pub async fn start(&self) -> Result<(), SessionError> {
        let mut state = self.state.write().await;

        if *state != SessionState::NotStarted && *state != SessionState::Ended {
            return Err(SessionError::AlreadyActive);
        }

        let now = Instant::now();
        *state = SessionState::Active;
        *self.start_time.write().await = Some(now);
        *self.active_start.write().await = Some(now);
        *self.stats.write().await = SessionStats::default();

        self.send_event(SessionEvent::Started).await;
        Ok(())
    }

    /// Pause the session
    pub async fn pause(&self) -> Result<(), SessionError> {
        let mut state = self.state.write().await;

        if *state != SessionState::Active {
            return Err(SessionError::NotActive);
        }

        // Update active time
        if let Some(start) = *self.active_start.read().await {
            let mut stats = self.stats.write().await;
            stats.active_time_seconds += start.elapsed().as_secs();
        }

        *state = SessionState::Paused;
        *self.active_start.write().await = None;

        self.send_event(SessionEvent::Paused).await;
        Ok(())
    }

    /// Resume the session
    pub async fn resume(&self) -> Result<(), SessionError> {
        let mut state = self.state.write().await;

        if *state != SessionState::Paused && *state != SessionState::OnBreak {
            return Err(SessionError::NotPaused);
        }

        *state = SessionState::Active;
        *self.active_start.write().await = Some(Instant::now());

        if *self.state.read().await == SessionState::OnBreak {
            self.send_event(SessionEvent::BreakEnded).await;
        }

        self.send_event(SessionEvent::Resumed).await;
        Ok(())
    }

    /// Start a break
    pub async fn start_break(&self, duration_minutes: u32) -> Result<(), SessionError> {
        let mut state = self.state.write().await;

        if *state != SessionState::Active && *state != SessionState::Paused {
            return Err(SessionError::InvalidState);
        }

        // Update active time
        if let Some(start) = *self.active_start.read().await {
            let mut stats = self.stats.write().await;
            stats.active_time_seconds += start.elapsed().as_secs();
            stats.breaks_taken += 1;
        }

        *state = SessionState::OnBreak;
        *self.active_start.write().await = None;

        self.send_event(SessionEvent::BreakStarted { duration_minutes }).await;
        Ok(())
    }

    /// End the session
    pub async fn end(&self) -> Result<SessionStats, SessionError> {
        let mut state = self.state.write().await;

        if *state == SessionState::NotStarted || *state == SessionState::Ended {
            return Err(SessionError::NotActive);
        }

        // Update final stats
        if let Some(start) = *self.active_start.read().await {
            let mut stats = self.stats.write().await;
            stats.active_time_seconds += start.elapsed().as_secs();
        }

        if let Some(session_start) = *self.start_time.read().await {
            let mut stats = self.stats.write().await;
            stats.total_time_seconds = session_start.elapsed().as_secs();
        }

        *state = SessionState::Ended;
        let stats = self.stats.read().await.clone();

        self.send_event(SessionEvent::Ended {
            total_minutes: (stats.total_time_seconds / 60) as u32,
        })
        .await;

        Ok(stats)
    }

    /// Enter safe space
    pub async fn enter_safe_space(&self) -> Result<(), SessionError> {
        let state = self.state.read().await;
        if *state != SessionState::Active {
            return Err(SessionError::NotActive);
        }

        let mut stats = self.stats.write().await;
        stats.safe_space_visits += 1;

        self.send_event(SessionEvent::SafeSpaceEntered).await;
        Ok(())
    }

    /// Exit safe space
    pub async fn exit_safe_space(&self) -> Result<(), SessionError> {
        self.send_event(SessionEvent::SafeSpaceExited).await;
        Ok(())
    }

    /// Get current session state
    pub async fn get_state(&self) -> SessionState {
        *self.state.read().await
    }

    /// Get current statistics
    pub async fn get_stats(&self) -> SessionStats {
        let mut stats = self.stats.read().await.clone();

        // Add current active time if session is active
        if *self.state.read().await == SessionState::Active {
            if let Some(start) = *self.active_start.read().await {
                stats.active_time_seconds += start.elapsed().as_secs();
            }
        }

        if let Some(session_start) = *self.start_time.read().await {
            stats.total_time_seconds = session_start.elapsed().as_secs();
        }

        stats
    }

    /// Get remaining session time (if limit set)
    pub async fn get_remaining_time(&self) -> Option<Duration> {
        let max_duration = self.settings.max_session_duration_minutes?;
        let start = (*self.start_time.read().await)?;
        let elapsed = start.elapsed();
        let max = Duration::from_secs(max_duration as u64 * 60);

        if elapsed >= max {
            Some(Duration::ZERO)
        } else {
            Some(max - elapsed)
        }
    }

    /// Check if session should show warning
    pub async fn check_time_warning(&self) -> Option<u32> {
        if let Some(remaining) = self.get_remaining_time().await {
            let minutes_remaining = remaining.as_secs() / 60;
            if minutes_remaining <= self.settings.warning_before_limit_minutes as u64 {
                return Some(minutes_remaining as u32);
            }
        }
        None
    }

    /// Check if rest reminder should be shown
    pub async fn check_rest_reminder(&self) -> bool {
        if *self.state.read().await != SessionState::Active {
            return false;
        }

        let last = *self.last_reminder.read().await;
        let interval = Duration::from_secs(self.reminder_interval_minutes as u64 * 60);

        match last {
            None => {
                if let Some(start) = *self.active_start.read().await {
                    start.elapsed() >= interval
                } else {
                    false
                }
            }
            Some(last_time) => last_time.elapsed() >= interval,
        }
    }

    /// Trigger rest reminder
    pub async fn trigger_reminder(&self) {
        let stats = self.get_stats().await;
        *self.last_reminder.write().await = Some(Instant::now());

        let mut stats_write = self.stats.write().await;
        stats_write.reminders_shown += 1;

        self.send_event(SessionEvent::ReminderTriggered {
            minutes_active: (stats.active_time_seconds / 60) as u32,
        })
        .await;
    }

    /// Start monitoring task
    pub fn start_monitoring(self: Arc<Self>) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_secs(60));

            loop {
                ticker.tick().await;

                // Check state
                let state = self.get_state().await;
                if state == SessionState::Ended || state == SessionState::NotStarted {
                    break;
                }

                // Check for time warning
                if let Some(minutes) = self.check_time_warning().await {
                    self.send_event(SessionEvent::WarningTriggered {
                        minutes_remaining: minutes,
                    })
                    .await;

                    if minutes == 0 {
                        self.send_event(SessionEvent::LimitReached).await;
                    }
                }

                // Check for rest reminder
                if self.check_rest_reminder().await {
                    self.trigger_reminder().await;
                }
            }
        })
    }

    async fn send_event(&self, event: SessionEvent) {
        if let Some(ref sender) = self.event_sender {
            let _ = sender.send(event).await;
        }
    }
}

/// Session error types
#[derive(Debug, Clone, thiserror::Error)]
pub enum SessionError {
    #[error("Session is already active")]
    AlreadyActive,

    #[error("Session is not active")]
    NotActive,

    #[error("Session is not paused")]
    NotPaused,

    #[error("Invalid session state for operation")]
    InvalidState,
}

/// Break scheduler for mandatory breaks
pub struct BreakScheduler {
    active_duration: Duration,
    break_duration: Duration,
    last_break: Option<Instant>,
}

impl BreakScheduler {
    pub fn new(active_minutes: u32, break_minutes: u32) -> Self {
        Self {
            active_duration: Duration::from_secs(active_minutes as u64 * 60),
            break_duration: Duration::from_secs(break_minutes as u64 * 60),
            last_break: None,
        }
    }

    pub fn should_break(&self, active_time: Duration) -> bool {
        match self.last_break {
            None => active_time >= self.active_duration,
            Some(last) => last.elapsed() >= self.active_duration,
        }
    }

    pub fn record_break(&mut self) {
        self.last_break = Some(Instant::now());
    }

    pub fn get_break_duration(&self) -> Duration {
        self.break_duration
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_settings() -> SessionSettings {
        SessionSettings {
            max_session_duration_minutes: Some(120),
            required_break_duration_minutes: Some(10),
            warning_before_limit_minutes: 5,
        }
    }

    #[tokio::test]
    async fn test_session_lifecycle() {
        let manager = SessionManager::new(default_settings());

        // Start session
        assert!(manager.start().await.is_ok());
        assert_eq!(manager.get_state().await, SessionState::Active);

        // Pause
        assert!(manager.pause().await.is_ok());
        assert_eq!(manager.get_state().await, SessionState::Paused);

        // Resume
        assert!(manager.resume().await.is_ok());
        assert_eq!(manager.get_state().await, SessionState::Active);

        // End
        let stats = manager.end().await.unwrap();
        assert_eq!(manager.get_state().await, SessionState::Ended);
        assert!(stats.total_time_seconds >= 0);
    }

    #[tokio::test]
    async fn test_session_with_events() {
        let (manager, mut rx) = SessionManager::with_events(default_settings());

        manager.start().await.unwrap();

        // Should receive start event
        if let Some(event) = rx.recv().await {
            assert!(matches!(event, SessionEvent::Started));
        }
    }

    #[tokio::test]
    async fn test_break_scheduler() {
        let mut scheduler = BreakScheduler::new(30, 5);

        // Should not break initially
        assert!(!scheduler.should_break(Duration::from_secs(10 * 60)));

        // Should break after 30 minutes
        assert!(scheduler.should_break(Duration::from_secs(31 * 60)));

        // Record break
        scheduler.record_break();

        // Should not need break immediately after
        assert!(!scheduler.should_break(Duration::from_secs(31 * 60)));
    }

    #[tokio::test]
    async fn test_safe_space() {
        let manager = SessionManager::new(default_settings());
        manager.start().await.unwrap();

        assert!(manager.enter_safe_space().await.is_ok());

        let stats = manager.get_stats().await;
        assert_eq!(stats.safe_space_visits, 1);
    }
}
