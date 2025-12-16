//! Event Stream for Real-time Accessibility Events
//! 弘益人間 - Event streaming and handling

use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use uuid::Uuid;

use super::accessibility::{AccessibilityEvent, EventType};

/// Event handler trait
pub trait EventHandler: Send + Sync {
    /// Handle an accessibility event
    fn handle(&self, event: &AccessibilityEvent);

    /// Get the event types this handler is interested in
    fn event_types(&self) -> Vec<EventType>;

    /// Get handler name (for debugging)
    fn name(&self) -> &str;
}

/// Event subscription
#[derive(Debug, Clone)]
pub struct EventSubscription {
    /// Subscription ID
    pub subscription_id: Uuid,
    /// Subscribed event types
    pub event_types: Vec<EventType>,
    /// Profile filter (if any)
    pub profile_filter: Option<Uuid>,
    /// Source filter (if any)
    pub source_filter: Option<String>,
    /// Active status
    pub active: bool,
}

impl EventSubscription {
    /// Create a new subscription
    pub fn new(event_types: Vec<EventType>) -> Self {
        Self {
            subscription_id: Uuid::new_v4(),
            event_types,
            profile_filter: None,
            source_filter: None,
            active: true,
        }
    }

    /// Filter by profile
    pub fn with_profile_filter(mut self, profile_id: Uuid) -> Self {
        self.profile_filter = Some(profile_id);
        self
    }

    /// Filter by source
    pub fn with_source_filter(mut self, source: &str) -> Self {
        self.source_filter = Some(source.to_string());
        self
    }

    /// Check if event matches this subscription
    pub fn matches(&self, event: &AccessibilityEvent) -> bool {
        if !self.active {
            return false;
        }

        // Check event type
        if !self.event_types.contains(&event.event_type) {
            return false;
        }

        // Check profile filter
        if let Some(ref profile_id) = self.profile_filter {
            if event.profile_id.as_ref() != Some(profile_id) {
                return false;
            }
        }

        // Check source filter
        if let Some(ref source) = self.source_filter {
            if &event.source != source {
                return false;
            }
        }

        true
    }

    /// Deactivate subscription
    pub fn deactivate(&mut self) {
        self.active = false;
    }
}

/// Callback-based event handler
pub struct CallbackHandler {
    name: String,
    event_types: Vec<EventType>,
    callback: Box<dyn Fn(&AccessibilityEvent) + Send + Sync>,
}

impl CallbackHandler {
    /// Create a new callback handler
    pub fn new<F>(name: &str, event_types: Vec<EventType>, callback: F) -> Self
    where
        F: Fn(&AccessibilityEvent) + Send + Sync + 'static,
    {
        Self {
            name: name.to_string(),
            event_types,
            callback: Box::new(callback),
        }
    }
}

impl EventHandler for CallbackHandler {
    fn handle(&self, event: &AccessibilityEvent) {
        (self.callback)(event);
    }

    fn event_types(&self) -> Vec<EventType> {
        self.event_types.clone()
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Event stream for managing accessibility events
pub struct EventStream {
    /// Event handlers
    handlers: Arc<RwLock<Vec<Arc<dyn EventHandler>>>>,
    /// Subscriptions
    subscriptions: Arc<RwLock<HashMap<Uuid, EventSubscription>>>,
    /// Event buffer
    buffer: Arc<RwLock<Vec<AccessibilityEvent>>>,
    /// Maximum buffer size
    max_buffer_size: usize,
    /// Stream source identifier
    source: String,
}

impl EventStream {
    /// Create a new event stream
    pub fn new(source: &str) -> Self {
        Self {
            handlers: Arc::new(RwLock::new(vec![])),
            subscriptions: Arc::new(RwLock::new(HashMap::new())),
            buffer: Arc::new(RwLock::new(vec![])),
            max_buffer_size: 1000,
            source: source.to_string(),
        }
    }

    /// Set maximum buffer size
    pub fn with_buffer_size(mut self, size: usize) -> Self {
        self.max_buffer_size = size;
        self
    }

    /// Register an event handler
    pub fn register_handler(&self, handler: Arc<dyn EventHandler>) {
        let mut handlers = self.handlers.write().unwrap();
        handlers.push(handler);
    }

    /// Register a callback handler
    pub fn on_event<F>(&self, name: &str, event_types: Vec<EventType>, callback: F)
    where
        F: Fn(&AccessibilityEvent) + Send + Sync + 'static,
    {
        let handler = Arc::new(CallbackHandler::new(name, event_types, callback));
        self.register_handler(handler);
    }

    /// Subscribe to events
    pub fn subscribe(&self, subscription: EventSubscription) -> Uuid {
        let id = subscription.subscription_id;
        let mut subs = self.subscriptions.write().unwrap();
        subs.insert(id, subscription);
        id
    }

    /// Unsubscribe
    pub fn unsubscribe(&self, subscription_id: &Uuid) -> bool {
        let mut subs = self.subscriptions.write().unwrap();
        subs.remove(subscription_id).is_some()
    }

    /// Emit an event
    pub fn emit(&self, event: AccessibilityEvent) {
        // Buffer the event
        {
            let mut buffer = self.buffer.write().unwrap();
            buffer.push(event.clone());

            // Trim buffer if too large
            if buffer.len() > self.max_buffer_size {
                let drain_count = buffer.len() - self.max_buffer_size;
                buffer.drain(0..drain_count);
            }
        }

        // Notify handlers
        let handlers = self.handlers.read().unwrap();
        for handler in handlers.iter() {
            if handler.event_types().contains(&event.event_type) {
                handler.handle(&event);
            }
        }
    }

    /// Emit multiple events
    pub fn emit_batch(&self, events: Vec<AccessibilityEvent>) {
        for event in events {
            self.emit(event);
        }
    }

    /// Get buffered events matching a subscription
    pub fn get_events(&self, subscription_id: &Uuid) -> Vec<AccessibilityEvent> {
        let subs = self.subscriptions.read().unwrap();
        let subscription = match subs.get(subscription_id) {
            Some(s) => s,
            None => return vec![],
        };

        let buffer = self.buffer.read().unwrap();
        buffer
            .iter()
            .filter(|e| subscription.matches(e))
            .cloned()
            .collect()
    }

    /// Get recent events of specific types
    pub fn get_recent(&self, event_types: &[EventType], limit: usize) -> Vec<AccessibilityEvent> {
        let buffer = self.buffer.read().unwrap();
        buffer
            .iter()
            .rev()
            .filter(|e| event_types.contains(&e.event_type))
            .take(limit)
            .cloned()
            .collect()
    }

    /// Get events for a specific profile
    pub fn get_profile_events(&self, profile_id: &Uuid, limit: usize) -> Vec<AccessibilityEvent> {
        let buffer = self.buffer.read().unwrap();
        buffer
            .iter()
            .rev()
            .filter(|e| e.is_for_profile(profile_id))
            .take(limit)
            .cloned()
            .collect()
    }

    /// Clear buffer
    pub fn clear_buffer(&self) {
        let mut buffer = self.buffer.write().unwrap();
        buffer.clear();
    }

    /// Get buffer size
    pub fn buffer_size(&self) -> usize {
        let buffer = self.buffer.read().unwrap();
        buffer.len()
    }

    /// Get handler count
    pub fn handler_count(&self) -> usize {
        let handlers = self.handlers.read().unwrap();
        handlers.len()
    }

    /// Get subscription count
    pub fn subscription_count(&self) -> usize {
        let subs = self.subscriptions.read().unwrap();
        subs.len()
    }

    /// Get stream source
    pub fn source(&self) -> &str {
        &self.source
    }
}

impl Clone for EventStream {
    fn clone(&self) -> Self {
        Self {
            handlers: Arc::clone(&self.handlers),
            subscriptions: Arc::clone(&self.subscriptions),
            buffer: Arc::clone(&self.buffer),
            max_buffer_size: self.max_buffer_size,
            source: self.source.clone(),
        }
    }
}

/// Event stream builder
pub struct EventStreamBuilder {
    source: String,
    buffer_size: usize,
    handlers: Vec<Arc<dyn EventHandler>>,
}

impl EventStreamBuilder {
    /// Create a new builder
    pub fn new(source: &str) -> Self {
        Self {
            source: source.to_string(),
            buffer_size: 1000,
            handlers: vec![],
        }
    }

    /// Set buffer size
    pub fn buffer_size(mut self, size: usize) -> Self {
        self.buffer_size = size;
        self
    }

    /// Add handler
    pub fn handler(mut self, handler: Arc<dyn EventHandler>) -> Self {
        self.handlers.push(handler);
        self
    }

    /// Build the event stream
    pub fn build(self) -> EventStream {
        let stream = EventStream::new(&self.source).with_buffer_size(self.buffer_size);

        for handler in self.handlers {
            stream.register_handler(handler);
        }

        stream
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::accessibility::{EventPayload, AdaptationRequest, AdaptationType};
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn test_create_event_stream() {
        let stream = EventStream::new("test-platform");
        assert_eq!(stream.source(), "test-platform");
        assert_eq!(stream.buffer_size(), 0);
    }

    #[test]
    fn test_emit_event() {
        let stream = EventStream::new("test");

        let request = AdaptationRequest::new(AdaptationType::ExtendedTime);
        let event = AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "test",
            EventPayload::Adaptation(request),
        );

        stream.emit(event);
        assert_eq!(stream.buffer_size(), 1);
    }

    #[test]
    fn test_subscription() {
        let stream = EventStream::new("test");

        let subscription = EventSubscription::new(vec![EventType::AdaptationRequest]);
        let sub_id = stream.subscribe(subscription);

        assert_eq!(stream.subscription_count(), 1);

        let request = AdaptationRequest::new(AdaptationType::Caption);
        let event = AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "test",
            EventPayload::Adaptation(request),
        );

        stream.emit(event);

        let events = stream.get_events(&sub_id);
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn test_subscription_filter() {
        let stream = EventStream::new("test");
        let profile_id = Uuid::new_v4();

        let subscription = EventSubscription::new(vec![EventType::AdaptationRequest])
            .with_profile_filter(profile_id);
        let sub_id = stream.subscribe(subscription);

        // Event without profile - should not match
        let request1 = AdaptationRequest::new(AdaptationType::Caption);
        let event1 = AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "test",
            EventPayload::Adaptation(request1),
        );
        stream.emit(event1);

        // Event with matching profile - should match
        let request2 = AdaptationRequest::new(AdaptationType::ExtendedTime);
        let event2 = AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "test",
            EventPayload::Adaptation(request2),
        ).with_profile(profile_id);
        stream.emit(event2);

        let events = stream.get_events(&sub_id);
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn test_callback_handler() {
        let stream = EventStream::new("test");
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_clone = Arc::clone(&counter);

        stream.on_event(
            "test-handler",
            vec![EventType::SettingChange],
            move |_event| {
                counter_clone.fetch_add(1, Ordering::SeqCst);
            },
        );

        // This event type doesn't match the handler
        let request = AdaptationRequest::new(AdaptationType::Caption);
        stream.emit(AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "test",
            EventPayload::Adaptation(request),
        ));

        assert_eq!(counter.load(Ordering::SeqCst), 0);

        // This event type matches
        use super::super::accessibility::{SettingChangeEvent, SettingChangeSource};
        let setting = SettingChangeEvent::new(
            "screen_reader.enabled",
            None,
            serde_json::json!(true),
            SettingChangeSource::User,
        );
        stream.emit(AccessibilityEvent::new(
            EventType::SettingChange,
            "test",
            EventPayload::Setting(setting),
        ));

        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_buffer_limit() {
        let stream = EventStream::new("test").with_buffer_size(5);

        for i in 0..10 {
            let request = AdaptationRequest::new(AdaptationType::Caption)
                .with_reason(&format!("Request {}", i));
            stream.emit(AccessibilityEvent::new(
                EventType::AdaptationRequest,
                "test",
                EventPayload::Adaptation(request),
            ));
        }

        assert_eq!(stream.buffer_size(), 5);
    }

    #[test]
    fn test_get_recent() {
        let stream = EventStream::new("test");

        for _ in 0..5 {
            let request = AdaptationRequest::new(AdaptationType::Caption);
            stream.emit(AccessibilityEvent::new(
                EventType::AdaptationRequest,
                "test",
                EventPayload::Adaptation(request),
            ));
        }

        let recent = stream.get_recent(&[EventType::AdaptationRequest], 3);
        assert_eq!(recent.len(), 3);
    }

    #[test]
    fn test_builder() {
        let stream = EventStreamBuilder::new("builder-test")
            .buffer_size(500)
            .build();

        assert_eq!(stream.source(), "builder-test");
    }
}
