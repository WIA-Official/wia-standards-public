//! Course Manager
//! 弘益人間 - Education for Everyone

use std::collections::HashMap;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::*;

/// Manages courses and their accessibility metadata
#[derive(Debug, Default)]
pub struct CourseManager {
    courses: HashMap<CourseId, Course>,
}

impl CourseManager {
    /// Create a new course manager
    pub fn new() -> Self {
        Self {
            courses: HashMap::new(),
        }
    }

    /// Create a new course
    pub fn create_course(&mut self, title: String) -> Course {
        let course = Course {
            course_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            title,
            description: None,
            created_at: Utc::now(),
            updated_at: None,
            course_info: CourseInfo::default(),
            accessibility_statement: AccessibilityStatement::default(),
            modules: Vec::new(),
            learning_outcomes: Vec::new(),
            accessibility_features: CourseAccessibilityFeatures::default(),
            accommodations_available: CourseAccommodations::default(),
        };

        self.courses.insert(course.course_id, course.clone());
        course
    }

    /// Get a course by ID
    pub fn get_course(&self, course_id: CourseId) -> Result<&Course> {
        self.courses
            .get(&course_id)
            .ok_or(EduError::CourseNotFound(course_id))
    }

    /// Get a mutable course by ID
    pub fn get_course_mut(&mut self, course_id: CourseId) -> Result<&mut Course> {
        self.courses
            .get_mut(&course_id)
            .ok_or(EduError::CourseNotFound(course_id))
    }

    /// Update a course
    pub fn update_course(&mut self, mut course: Course) -> Result<()> {
        if !self.courses.contains_key(&course.course_id) {
            return Err(EduError::CourseNotFound(course.course_id));
        }
        course.updated_at = Some(Utc::now());
        self.courses.insert(course.course_id, course);
        Ok(())
    }

    /// Delete a course
    pub fn delete_course(&mut self, course_id: CourseId) -> Result<Course> {
        self.courses
            .remove(&course_id)
            .ok_or(EduError::CourseNotFound(course_id))
    }

    /// List all courses
    pub fn list_courses(&self) -> Vec<&Course> {
        self.courses.values().collect()
    }

    /// Add a module to a course
    pub fn add_module(&mut self, course_id: CourseId, title: String) -> Result<Module> {
        let course = self.get_course_mut(course_id)?;
        let sequence = course.modules.len() as u32 + 1;

        let module = Module {
            module_id: Uuid::new_v4(),
            title,
            description: None,
            sequence,
            duration_minutes: None,
            learning_objectives: Vec::new(),
            content_items: Vec::new(),
            assessments: Vec::new(),
            udl_options: UDLOptions::default(),
            prerequisites_modules: Vec::new(),
        };

        course.modules.push(module.clone());
        course.updated_at = Some(Utc::now());

        Ok(module)
    }

    /// Get a module from a course
    pub fn get_module(&self, course_id: CourseId, module_id: ModuleId) -> Result<&Module> {
        let course = self.get_course(course_id)?;
        course
            .modules
            .iter()
            .find(|m| m.module_id == module_id)
            .ok_or(EduError::ModuleNotFound(module_id))
    }

    /// Add content to a module
    pub fn add_content_to_module(
        &mut self,
        course_id: CourseId,
        module_id: ModuleId,
        content: ContentItem,
    ) -> Result<()> {
        let course = self.get_course_mut(course_id)?;
        let module = course
            .modules
            .iter_mut()
            .find(|m| m.module_id == module_id)
            .ok_or(EduError::ModuleNotFound(module_id))?;

        module.content_items.push(content);
        course.updated_at = Some(Utc::now());

        Ok(())
    }

    /// Set accessibility statement for a course
    pub fn set_accessibility_statement(
        &mut self,
        course_id: CourseId,
        statement: AccessibilityStatement,
    ) -> Result<()> {
        let course = self.get_course_mut(course_id)?;
        course.accessibility_statement = statement;
        course.updated_at = Some(Utc::now());
        Ok(())
    }

    /// Add accessibility feature to a course
    pub fn add_accessibility_feature(
        &mut self,
        course_id: CourseId,
        feature: CourseFeature,
    ) -> Result<()> {
        let course = self.get_course_mut(course_id)?;
        if !course.accessibility_features.features.contains(&feature) {
            course.accessibility_features.features.push(feature);
            course.updated_at = Some(Utc::now());
        }
        Ok(())
    }

    /// Find courses by accessibility feature
    pub fn find_by_feature(&self, feature: CourseFeature) -> Vec<&Course> {
        self.courses
            .values()
            .filter(|c| c.accessibility_features.features.contains(&feature))
            .collect()
    }

    /// Find courses by WCAG conformance level
    pub fn find_by_wcag_level(&self, level: WCAGLevel) -> Vec<&Course> {
        self.courses
            .values()
            .filter(|c| c.accessibility_statement.wcag_conformance == Some(level))
            .collect()
    }

    /// Check if course has all videos captioned
    pub fn has_all_captions(&self, course_id: CourseId) -> Result<bool> {
        let course = self.get_course(course_id)?;

        for module in &course.modules {
            for content in &module.content_items {
                if content.content_type == ContentType::Video
                    && !content.accessibility_metadata.has_captions
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// Get accessibility report for a course
    pub fn get_accessibility_report(&self, course_id: CourseId) -> Result<CourseAccessibilityReport> {
        let course = self.get_course(course_id)?;

        let mut total_content = 0;
        let mut captioned_videos = 0;
        let mut total_videos = 0;
        let mut transcribed_audio = 0;
        let mut total_audio = 0;
        let mut keyboard_accessible = 0;
        let mut screen_reader_compatible = 0;

        for module in &course.modules {
            for content in &module.content_items {
                total_content += 1;

                if content.content_type == ContentType::Video {
                    total_videos += 1;
                    if content.accessibility_metadata.has_captions {
                        captioned_videos += 1;
                    }
                }

                if content.content_type == ContentType::Audio {
                    total_audio += 1;
                    if content.accessibility_metadata.has_transcript {
                        transcribed_audio += 1;
                    }
                }

                if content.accessibility_metadata.keyboard_accessible {
                    keyboard_accessible += 1;
                }

                if content.accessibility_metadata.screen_reader_compatible {
                    screen_reader_compatible += 1;
                }
            }
        }

        Ok(CourseAccessibilityReport {
            course_id,
            total_content,
            captioned_videos,
            total_videos,
            transcribed_audio,
            total_audio,
            keyboard_accessible,
            screen_reader_compatible,
            wcag_conformance: course.accessibility_statement.wcag_conformance,
            known_issues: course.accessibility_statement.known_issues.len(),
        })
    }

    /// Export course to JSON
    pub fn export_course(&self, course_id: CourseId) -> Result<String> {
        let course = self.get_course(course_id)?;
        serde_json::to_string_pretty(course).map_err(EduError::from)
    }

    /// Import course from JSON
    pub fn import_course(&mut self, json: &str) -> Result<Course> {
        let mut course: Course = serde_json::from_str(json)?;
        course.course_id = Uuid::new_v4();
        course.created_at = Utc::now();
        course.updated_at = None;
        self.courses.insert(course.course_id, course.clone());
        Ok(course)
    }
}

/// Accessibility report for a course
#[derive(Debug, Clone)]
pub struct CourseAccessibilityReport {
    pub course_id: CourseId,
    pub total_content: usize,
    pub captioned_videos: usize,
    pub total_videos: usize,
    pub transcribed_audio: usize,
    pub total_audio: usize,
    pub keyboard_accessible: usize,
    pub screen_reader_compatible: usize,
    pub wcag_conformance: Option<WCAGLevel>,
    pub known_issues: usize,
}

impl CourseAccessibilityReport {
    /// Calculate caption coverage percentage
    pub fn caption_coverage(&self) -> f32 {
        if self.total_videos == 0 {
            100.0
        } else {
            (self.captioned_videos as f32 / self.total_videos as f32) * 100.0
        }
    }

    /// Calculate transcript coverage percentage
    pub fn transcript_coverage(&self) -> f32 {
        if self.total_audio == 0 {
            100.0
        } else {
            (self.transcribed_audio as f32 / self.total_audio as f32) * 100.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_course() {
        let mut manager = CourseManager::new();
        let course = manager.create_course("Introduction to Accessibility".to_string());

        assert!(!course.course_id.is_nil());
        assert_eq!(course.title, "Introduction to Accessibility");
    }

    #[test]
    fn test_add_module() {
        let mut manager = CourseManager::new();
        let course = manager.create_course("Test Course".to_string());
        let module = manager.add_module(course.course_id, "Module 1".to_string()).unwrap();

        assert_eq!(module.sequence, 1);
        assert_eq!(module.title, "Module 1");
    }

    #[test]
    fn test_accessibility_features() {
        let mut manager = CourseManager::new();
        let course = manager.create_course("Accessible Course".to_string());

        manager.add_accessibility_feature(course.course_id, CourseFeature::CaptionsAllVideos).unwrap();
        manager.add_accessibility_feature(course.course_id, CourseFeature::KeyboardNavigation).unwrap();

        let updated = manager.get_course(course.course_id).unwrap();
        assert!(updated.accessibility_features.features.contains(&CourseFeature::CaptionsAllVideos));
        assert!(updated.accessibility_features.features.contains(&CourseFeature::KeyboardNavigation));
    }
}
