//! Project management types

use serde::{Deserialize, Serialize};
use crate::types::*;

/// Space project
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpaceProject {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// WIA version
    pub wia_version: String,

    /// Format version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub format_version: Option<String>,

    /// Project ID
    pub project_id: String,

    /// Project info
    pub project_info: ProjectInfo,

    /// Organization
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organization: Option<Organization>,

    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<ProjectMetadata>,
}

impl SpaceProject {
    /// Create a new project
    pub fn new(id: impl Into<String>, name: impl Into<String>, category: TechnologyCategory) -> Self {
        Self {
            schema: Some("https://wia.live/schemas/space/project.schema.json".to_string()),
            wia_version: crate::WIA_VERSION.to_string(),
            format_version: Some("1.0.0".to_string()),
            project_id: id.into(),
            project_info: ProjectInfo {
                name: name.into(),
                description: None,
                start_date: None,
                status: ProjectStatus::Planned,
                technology_category: category,
            },
            organization: None,
            metadata: Some(ProjectMetadata::default()),
        }
    }

    /// Set description
    pub fn with_description(mut self, desc: impl Into<String>) -> Self {
        self.project_info.description = Some(desc.into());
        self
    }

    /// Set organization
    pub fn with_organization(mut self, org: Organization) -> Self {
        self.organization = Some(org);
        self
    }

    /// Set status
    pub fn with_status(mut self, status: ProjectStatus) -> Self {
        self.project_info.status = status;
        self
    }
}

/// Project information
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectInfo {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_date: Option<String>,
    pub status: ProjectStatus,
    pub technology_category: TechnologyCategory,
}

/// Organization
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Organization {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub country: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contact: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub website: Option<String>,
}

impl Organization {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            country: None,
            contact: None,
            website: None,
        }
    }

    pub fn with_country(mut self, country: impl Into<String>) -> Self {
        self.country = Some(country.into());
        self
    }

    pub fn with_contact(mut self, contact: impl Into<String>) -> Self {
        self.contact = Some(contact.into());
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_creation() {
        let project = SpaceProject::new(
            "proj-001",
            "Alpha Centauri Mission",
            TechnologyCategory::InterstellarTravel,
        )
        .with_description("First interstellar probe mission")
        .with_status(ProjectStatus::Active);

        assert_eq!(project.project_id, "proj-001");
        assert_eq!(project.project_info.status, ProjectStatus::Active);
    }

    #[test]
    fn test_project_serialization() {
        let project = SpaceProject::new(
            "proj-test",
            "Test Project",
            TechnologyCategory::AsteroidMining,
        );

        let json = serde_json::to_string_pretty(&project).unwrap();
        assert!(json.contains("asteroid_mining"));
    }
}
