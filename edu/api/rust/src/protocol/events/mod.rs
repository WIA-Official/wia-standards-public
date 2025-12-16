//! Accessibility Events Module
//! 弘益人間 - Real-time accessibility adaptation events

mod accessibility;
mod stream;

pub use accessibility::{
    AccessibilityEvent, EventType, AdaptationRequest, AdaptationResponse,
    ContentAlternativeRequest, SettingChangeEvent,
};
pub use stream::{EventStream, EventHandler, EventSubscription};
