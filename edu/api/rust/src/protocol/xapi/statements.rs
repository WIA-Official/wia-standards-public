//! xAPI Statement Structures
//! 弘益人間 - Experience API statement definitions

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};
use std::collections::HashMap;

/// xAPI Statement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XapiStatement {
    /// Statement ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    /// Actor (who did the action)
    pub actor: XapiActor,
    /// Verb (what was done)
    pub verb: XapiVerb,
    /// Object (what the action was done to)
    pub object: XapiObject,
    /// Result (outcome of the action)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<XapiResult>,
    /// Context (additional information)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<XapiContext>,
    /// Timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
    /// Stored timestamp (set by LRS)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stored: Option<DateTime<Utc>>,
    /// Authority (who is asserting this statement)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub authority: Option<XapiActor>,
    /// Statement version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

impl XapiStatement {
    /// Create a new statement
    pub fn new(actor: XapiActor, verb: XapiVerb, object: XapiObject) -> Self {
        Self {
            id: Some(Uuid::new_v4()),
            actor,
            verb,
            object,
            result: None,
            context: None,
            timestamp: Some(Utc::now()),
            stored: None,
            authority: None,
            version: Some("1.0.3".to_string()),
        }
    }

    /// Add result to statement
    pub fn with_result(mut self, result: XapiResult) -> Self {
        self.result = Some(result);
        self
    }

    /// Add context to statement
    pub fn with_context(mut self, context: XapiContext) -> Self {
        self.context = Some(context);
        self
    }
}

/// xAPI Actor (Agent or Group)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum XapiActor {
    Agent(Agent),
    Group(Group),
}

/// Actor type identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActorType {
    Agent,
    Group,
}

/// xAPI Agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Agent {
    /// Object type (always "Agent")
    #[serde(rename = "objectType")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub object_type: Option<ActorType>,
    /// Agent name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Mailto IFI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mbox: Option<String>,
    /// SHA1 hash of mailto IFI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mbox_sha1sum: Option<String>,
    /// OpenID IFI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub openid: Option<String>,
    /// Account IFI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub account: Option<Account>,
}

impl Agent {
    /// Create agent with email
    pub fn with_email(email: &str, name: Option<String>) -> Self {
        Self {
            object_type: Some(ActorType::Agent),
            name,
            mbox: Some(format!("mailto:{}", email)),
            mbox_sha1sum: None,
            openid: None,
            account: None,
        }
    }

    /// Create agent with account
    pub fn with_account(home_page: &str, account_name: &str, name: Option<String>) -> Self {
        Self {
            object_type: Some(ActorType::Agent),
            name,
            mbox: None,
            mbox_sha1sum: None,
            openid: None,
            account: Some(Account {
                home_page: home_page.to_string(),
                name: account_name.to_string(),
            }),
        }
    }
}

/// xAPI Group
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Group {
    /// Object type (always "Group")
    #[serde(rename = "objectType")]
    pub object_type: ActorType,
    /// Group name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Group members
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub member: Vec<Agent>,
    /// Account IFI (for identified groups)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub account: Option<Account>,
}

/// Account identifier
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Account {
    /// Home page URL
    #[serde(rename = "homePage")]
    pub home_page: String,
    /// Account name
    pub name: String,
}

/// xAPI Verb
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XapiVerb {
    /// Verb IRI
    pub id: String,
    /// Display name in various languages
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<HashMap<String, String>>,
}

impl XapiVerb {
    /// Create a new verb
    pub fn new(id: &str) -> Self {
        Self {
            id: id.to_string(),
            display: None,
        }
    }

    /// Add display name
    pub fn with_display(mut self, lang: &str, name: &str) -> Self {
        let display = self.display.get_or_insert_with(HashMap::new);
        display.insert(lang.to_string(), name.to_string());
        self
    }
}

/// xAPI Object
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum XapiObject {
    Activity(Activity),
    Agent(Agent),
    StatementRef(StatementRef),
    SubStatement(Box<SubStatement>),
}

/// Object type identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObjectType {
    Activity,
    Agent,
    Group,
    StatementRef,
    SubStatement,
}

/// xAPI Activity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Activity {
    /// Object type (always "Activity" for activities)
    #[serde(rename = "objectType")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub object_type: Option<ObjectType>,
    /// Activity IRI
    pub id: String,
    /// Activity definition
    #[serde(skip_serializing_if = "Option::is_none")]
    pub definition: Option<ActivityDefinition>,
}

impl Activity {
    /// Create a new activity
    pub fn new(id: &str) -> Self {
        Self {
            object_type: Some(ObjectType::Activity),
            id: id.to_string(),
            definition: None,
        }
    }

    /// Add definition
    pub fn with_definition(mut self, definition: ActivityDefinition) -> Self {
        self.definition = Some(definition);
        self
    }
}

/// Activity Definition
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ActivityDefinition {
    /// Activity type IRI
    #[serde(rename = "type")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub activity_type: Option<String>,
    /// Name in various languages
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<HashMap<String, String>>,
    /// Description in various languages
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<HashMap<String, String>>,
    /// More info URL
    #[serde(rename = "moreInfo")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub more_info: Option<String>,
    /// Extensions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extensions: Option<XapiExtensions>,
}

impl ActivityDefinition {
    /// Create with type
    pub fn with_type(activity_type: &str) -> Self {
        Self {
            activity_type: Some(activity_type.to_string()),
            ..Default::default()
        }
    }

    /// Add name
    pub fn with_name(mut self, lang: &str, name: &str) -> Self {
        let names = self.name.get_or_insert_with(HashMap::new);
        names.insert(lang.to_string(), name.to_string());
        self
    }

    /// Add description
    pub fn with_description(mut self, lang: &str, description: &str) -> Self {
        let descriptions = self.description.get_or_insert_with(HashMap::new);
        descriptions.insert(lang.to_string(), description.to_string());
        self
    }
}

/// Statement Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatementRef {
    /// Object type
    #[serde(rename = "objectType")]
    pub object_type: ObjectType,
    /// Statement ID
    pub id: Uuid,
}

/// Sub-Statement (Statement within a Statement)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubStatement {
    /// Object type
    #[serde(rename = "objectType")]
    pub object_type: ObjectType,
    /// Actor
    pub actor: XapiActor,
    /// Verb
    pub verb: XapiVerb,
    /// Object
    pub object: XapiObject,
    /// Result
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<XapiResult>,
    /// Context
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<XapiContext>,
}

/// xAPI Result
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct XapiResult {
    /// Score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub score: Option<Score>,
    /// Success indicator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub success: Option<bool>,
    /// Completion indicator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub completion: Option<bool>,
    /// Response string
    #[serde(skip_serializing_if = "Option::is_none")]
    pub response: Option<String>,
    /// Duration (ISO 8601)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
    /// Extensions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extensions: Option<XapiExtensions>,
}

impl XapiResult {
    /// Create a completion result
    pub fn completed(success: bool) -> Self {
        Self {
            completion: Some(true),
            success: Some(success),
            ..Default::default()
        }
    }

    /// Add score
    pub fn with_score(mut self, score: Score) -> Self {
        self.score = Some(score);
        self
    }

    /// Add duration
    pub fn with_duration(mut self, duration: &str) -> Self {
        self.duration = Some(duration.to_string());
        self
    }
}

/// Score object
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Score {
    /// Scaled score (-1 to 1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scaled: Option<f64>,
    /// Raw score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw: Option<f64>,
    /// Minimum possible score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    /// Maximum possible score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
}

impl Score {
    /// Create a scaled score
    pub fn scaled(value: f64) -> Self {
        Self {
            scaled: Some(value.clamp(-1.0, 1.0)),
            ..Default::default()
        }
    }

    /// Create a raw score with range
    pub fn raw(value: f64, min: f64, max: f64) -> Self {
        Self {
            raw: Some(value),
            min: Some(min),
            max: Some(max),
            scaled: Some((value - min) / (max - min)),
        }
    }
}

/// xAPI Context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct XapiContext {
    /// Registration UUID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub registration: Option<Uuid>,
    /// Instructor
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instructor: Option<XapiActor>,
    /// Team
    #[serde(skip_serializing_if = "Option::is_none")]
    pub team: Option<Group>,
    /// Context activities
    #[serde(rename = "contextActivities")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context_activities: Option<ContextActivities>,
    /// Revision
    #[serde(skip_serializing_if = "Option::is_none")]
    pub revision: Option<String>,
    /// Platform
    #[serde(skip_serializing_if = "Option::is_none")]
    pub platform: Option<String>,
    /// Language
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
    /// Statement reference
    #[serde(skip_serializing_if = "Option::is_none")]
    pub statement: Option<StatementRef>,
    /// Extensions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extensions: Option<XapiExtensions>,
}

impl XapiContext {
    /// Add registration
    pub fn with_registration(mut self, registration: Uuid) -> Self {
        self.registration = Some(registration);
        self
    }

    /// Add platform
    pub fn with_platform(mut self, platform: &str) -> Self {
        self.platform = Some(platform.to_string());
        self
    }

    /// Add language
    pub fn with_language(mut self, language: &str) -> Self {
        self.language = Some(language.to_string());
        self
    }

    /// Add extensions
    pub fn with_extensions(mut self, extensions: XapiExtensions) -> Self {
        self.extensions = Some(extensions);
        self
    }
}

/// Context Activities
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ContextActivities {
    /// Parent activities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parent: Option<Vec<Activity>>,
    /// Grouping activities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grouping: Option<Vec<Activity>>,
    /// Category activities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<Vec<Activity>>,
    /// Other activities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub other: Option<Vec<Activity>>,
}

/// xAPI Extensions (key-value map)
pub type XapiExtensions = HashMap<String, serde_json::Value>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_statement() {
        let actor = XapiActor::Agent(Agent::with_email(
            "learner@example.com",
            Some("Test Learner".to_string()),
        ));

        let verb = XapiVerb::new("http://adlnet.gov/expapi/verbs/completed")
            .with_display("en-US", "completed");

        let object = XapiObject::Activity(
            Activity::new("https://example.com/course/123")
                .with_definition(
                    ActivityDefinition::with_type("http://adlnet.gov/expapi/activities/course")
                        .with_name("en-US", "Accessible Learning Course")
                )
        );

        let statement = XapiStatement::new(actor, verb, object);

        assert!(statement.id.is_some());
        assert_eq!(statement.version, Some("1.0.3".to_string()));
    }

    #[test]
    fn test_score() {
        let score = Score::raw(85.0, 0.0, 100.0);
        assert_eq!(score.raw, Some(85.0));
        assert_eq!(score.scaled, Some(0.85));
    }

    #[test]
    fn test_result() {
        let result = XapiResult::completed(true)
            .with_score(Score::scaled(0.9))
            .with_duration("PT30M");

        assert_eq!(result.success, Some(true));
        assert_eq!(result.completion, Some(true));
        assert_eq!(result.duration, Some("PT30M".to_string()));
    }
}
