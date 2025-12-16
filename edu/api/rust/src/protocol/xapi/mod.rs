//! xAPI (Experience API) Module
//! 弘益人間 - Learning activity tracking with accessibility support

mod statements;
mod verbs;
mod generator;

pub use statements::{
    XapiStatement, XapiActor, XapiVerb, XapiObject, XapiResult,
    XapiContext, XapiExtensions, ObjectType, ActorType,
};
pub use verbs::AccessibilityVerb;
pub use generator::StatementGenerator;
