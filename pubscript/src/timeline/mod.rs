//! Timeline synchronization module
//!
//! Enables synchronized playback of multiple representation streams:
//! - Visual (video, text)
//! - Auditory (audio, speech)
//! - Tactile (braille, haptics)
//! - Spatial (3D positioning)
//! - Gestural (motion capture)
//!
//! ## Philosophy
//!
//! All representations are synchronized in time, enabling:
//! - Video with synchronized captions and audio description
//! - Haptic feedback synchronized with audio
//! - Spatial audio synchronized with visual position
//! - Multi-modal experiences with perfect timing
//!
//! ## Example
//!
//! ```rust
//! use wia_pubscript::timeline::{Timeline, Track, TrackKind, TimeEvent};
//! use std::time::Duration;
//!
//! let mut timeline = Timeline::new(Duration::from_secs(10));
//!
//! // Add visual track
//! let visual_track = Track::new(TrackKind::Visual);
//! timeline.add_track(visual_track);
//!
//! // Sync at specific timestamp
//! let synced = timeline.sync(Duration::from_secs(5));
//! ```

use std::collections::HashMap;
use std::time::Duration;

/// Track kind corresponding to the five representations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrackKind {
    /// Visual representation (text, video, images)
    Visual,
    /// Auditory representation (audio, speech, music)
    Auditory,
    /// Tactile representation (braille, haptics)
    Tactile,
    /// Spatial representation (3D position, surround)
    Spatial,
    /// Gestural representation (sign language, motion)
    Gestural,
}

impl TrackKind {
    /// Get all track kinds
    pub fn all() -> Vec<TrackKind> {
        vec![
            TrackKind::Visual,
            TrackKind::Auditory,
            TrackKind::Tactile,
            TrackKind::Spatial,
            TrackKind::Gestural,
        ]
    }

    /// Get track kind name
    pub fn name(&self) -> &'static str {
        match self {
            TrackKind::Visual => "Visual",
            TrackKind::Auditory => "Auditory",
            TrackKind::Tactile => "Tactile",
            TrackKind::Spatial => "Spatial",
            TrackKind::Gestural => "Gestural",
        }
    }
}

/// Time event on a track
#[derive(Debug, Clone)]
pub struct TimeEvent {
    /// Start time of the event
    pub start: Duration,
    /// End time of the event
    pub end: Duration,
    /// Content node reference (from IR)
    pub content_id: String,
    /// Optional metadata
    pub metadata: HashMap<String, String>,
}

impl TimeEvent {
    /// Create a new time event
    pub fn new(start: Duration, end: Duration, content_id: String) -> Self {
        Self {
            start,
            end,
            content_id,
            metadata: HashMap::new(),
        }
    }

    /// Check if this event is active at given time
    pub fn is_active(&self, time: Duration) -> bool {
        time >= self.start && time < self.end
    }

    /// Get event duration
    pub fn duration(&self) -> Duration {
        self.end.saturating_sub(self.start)
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }
}

/// Timeline track containing time-ordered events
#[derive(Debug, Clone)]
pub struct Track {
    /// Track kind (which representation)
    pub kind: TrackKind,
    /// Ordered list of time events
    pub events: Vec<TimeEvent>,
    /// Track metadata
    pub metadata: HashMap<String, String>,
}

impl Track {
    /// Create a new track
    pub fn new(kind: TrackKind) -> Self {
        Self {
            kind,
            events: Vec::new(),
            metadata: HashMap::new(),
        }
    }

    /// Add an event to the track
    pub fn add_event(&mut self, event: TimeEvent) {
        self.events.push(event);
        // Keep events sorted by start time
        self.events.sort_by_key(|e| e.start);
    }

    /// Get active event at given time
    pub fn get_active_event(&self, time: Duration) -> Option<&TimeEvent> {
        self.events.iter().find(|e| e.is_active(time))
    }

    /// Get all events in time range
    pub fn get_events_in_range(&self, start: Duration, end: Duration) -> Vec<&TimeEvent> {
        self.events
            .iter()
            .filter(|e| e.start < end && e.end > start)
            .collect()
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }
}

/// Synchronized output at a specific timestamp
#[derive(Debug, Clone)]
pub struct SyncedOutput {
    /// Current timestamp
    pub timestamp: Duration,
    /// Active events per track kind
    pub active_events: HashMap<TrackKind, String>,
    /// Metadata
    pub metadata: HashMap<String, String>,
}

impl SyncedOutput {
    /// Create new synced output
    pub fn new(timestamp: Duration) -> Self {
        Self {
            timestamp,
            active_events: HashMap::new(),
            metadata: HashMap::new(),
        }
    }

    /// Add active event
    pub fn add_event(&mut self, kind: TrackKind, content_id: String) {
        self.active_events.insert(kind, content_id);
    }

    /// Get visual content
    pub fn visual(&self) -> Option<&String> {
        self.active_events.get(&TrackKind::Visual)
    }

    /// Get auditory content
    pub fn auditory(&self) -> Option<&String> {
        self.active_events.get(&TrackKind::Auditory)
    }

    /// Get tactile content
    pub fn tactile(&self) -> Option<&String> {
        self.active_events.get(&TrackKind::Tactile)
    }

    /// Get spatial content
    pub fn spatial(&self) -> Option<&String> {
        self.active_events.get(&TrackKind::Spatial)
    }

    /// Get gestural content
    pub fn gestural(&self) -> Option<&String> {
        self.active_events.get(&TrackKind::Gestural)
    }
}

/// Timeline for synchronized multi-modal playback
#[derive(Debug, Clone)]
pub struct Timeline {
    /// Total duration of the timeline
    pub duration: Duration,
    /// Tracks organized by kind
    pub tracks: HashMap<TrackKind, Track>,
    /// Timeline metadata
    pub metadata: HashMap<String, String>,
}

impl Timeline {
    /// Create a new timeline
    pub fn new(duration: Duration) -> Self {
        Self {
            duration,
            tracks: HashMap::new(),
            metadata: HashMap::new(),
        }
    }

    /// Add a track to the timeline
    pub fn add_track(&mut self, track: Track) {
        self.tracks.insert(track.kind, track);
    }

    /// Get track by kind
    pub fn get_track(&self, kind: TrackKind) -> Option<&Track> {
        self.tracks.get(&kind)
    }

    /// Get mutable track by kind
    pub fn get_track_mut(&mut self, kind: TrackKind) -> Option<&mut Track> {
        self.tracks.get_mut(&kind)
    }

    /// Synchronize all tracks at given timestamp
    ///
    /// Returns the active content for each representation at that time
    pub fn sync(&self, timestamp: Duration) -> SyncedOutput {
        let mut output = SyncedOutput::new(timestamp);

        // Get active event from each track
        for (kind, track) in &self.tracks {
            if let Some(event) = track.get_active_event(timestamp) {
                output.add_event(*kind, event.content_id.clone());
            }
        }

        output
    }

    /// Get all active events at timestamp across all tracks
    pub fn get_active_events(&self, timestamp: Duration) -> Vec<(&TrackKind, &TimeEvent)> {
        self.tracks
            .iter()
            .filter_map(|(kind, track)| {
                track.get_active_event(timestamp).map(|event| (kind, event))
            })
            .collect()
    }

    /// Validate timeline (check for overlaps, gaps, etc.)
    pub fn validate(&self) -> Result<(), String> {
        for (kind, track) in &self.tracks {
            // Check events don't exceed timeline duration
            for event in &track.events {
                if event.end > self.duration {
                    return Err(format!(
                        "{} track has event ending at {:?} beyond timeline duration {:?}",
                        kind.name(),
                        event.end,
                        self.duration
                    ));
                }
            }

            // Check for overlapping events in same track
            for i in 0..track.events.len() {
                for j in i + 1..track.events.len() {
                    let e1 = &track.events[i];
                    let e2 = &track.events[j];
                    if e1.start < e2.end && e2.start < e1.end {
                        return Err(format!(
                            "{} track has overlapping events: {:?} and {:?}",
                            kind.name(),
                            e1.start,
                            e2.start
                        ));
                    }
                }
            }
        }

        Ok(())
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_track_kind_all() {
        let kinds = TrackKind::all();
        assert_eq!(kinds.len(), 5);
        assert!(kinds.contains(&TrackKind::Visual));
        assert!(kinds.contains(&TrackKind::Auditory));
        assert!(kinds.contains(&TrackKind::Tactile));
    }

    #[test]
    fn test_time_event_active() {
        let event = TimeEvent::new(
            Duration::from_secs(1),
            Duration::from_secs(5),
            "event1".to_string(),
        );

        assert!(!event.is_active(Duration::from_secs(0)));
        assert!(event.is_active(Duration::from_secs(1)));
        assert!(event.is_active(Duration::from_secs(3)));
        assert!(!event.is_active(Duration::from_secs(5)));
    }

    #[test]
    fn test_track_add_event() {
        let mut track = Track::new(TrackKind::Visual);

        track.add_event(TimeEvent::new(
            Duration::from_secs(5),
            Duration::from_secs(10),
            "event2".to_string(),
        ));

        track.add_event(TimeEvent::new(
            Duration::from_secs(0),
            Duration::from_secs(5),
            "event1".to_string(),
        ));

        // Events should be sorted by start time
        assert_eq!(track.events[0].content_id, "event1");
        assert_eq!(track.events[1].content_id, "event2");
    }

    #[test]
    fn test_timeline_sync() {
        let mut timeline = Timeline::new(Duration::from_secs(10));

        let mut visual = Track::new(TrackKind::Visual);
        visual.add_event(TimeEvent::new(
            Duration::from_secs(0),
            Duration::from_secs(5),
            "visual1".to_string(),
        ));
        timeline.add_track(visual);

        let mut auditory = Track::new(TrackKind::Auditory);
        auditory.add_event(TimeEvent::new(
            Duration::from_secs(0),
            Duration::from_secs(10),
            "audio1".to_string(),
        ));
        timeline.add_track(auditory);

        // Sync at t=2s
        let synced = timeline.sync(Duration::from_secs(2));
        assert_eq!(synced.visual(), Some(&"visual1".to_string()));
        assert_eq!(synced.auditory(), Some(&"audio1".to_string()));

        // Sync at t=7s (visual event ended)
        let synced = timeline.sync(Duration::from_secs(7));
        assert_eq!(synced.visual(), None);
        assert_eq!(synced.auditory(), Some(&"audio1".to_string()));
    }

    #[test]
    fn test_timeline_validate() {
        let mut timeline = Timeline::new(Duration::from_secs(10));

        let mut track = Track::new(TrackKind::Visual);
        track.add_event(TimeEvent::new(
            Duration::from_secs(0),
            Duration::from_secs(5),
            "event1".to_string(),
        ));
        track.add_event(TimeEvent::new(
            Duration::from_secs(5),
            Duration::from_secs(10),
            "event2".to_string(),
        ));
        timeline.add_track(track);

        assert!(timeline.validate().is_ok());

        // Test overlapping events
        let mut bad_timeline = Timeline::new(Duration::from_secs(10));
        let mut bad_track = Track::new(TrackKind::Visual);
        bad_track.add_event(TimeEvent::new(
            Duration::from_secs(0),
            Duration::from_secs(5),
            "event1".to_string(),
        ));
        bad_track.add_event(TimeEvent::new(
            Duration::from_secs(3),
            Duration::from_secs(7),
            "event2".to_string(),
        ));
        bad_timeline.add_track(bad_track);

        assert!(bad_timeline.validate().is_err());
    }
}
