//! Accessibility compliance module

use serde::{Deserialize, Serialize};

/// WCAG compliance level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WcagLevel {
    /// WCAG 2.1 Level A
    A,
    /// WCAG 2.1 Level AA
    AA,
    /// WCAG 2.1 Level AAA (target for sign language)
    AAA,
}

/// Accessibility configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityConfig {
    /// Target WCAG level
    pub wcag_level: WcagLevel,

    /// Avatar display settings
    pub avatar: AvatarAccessibilityConfig,

    /// Caption settings
    pub captions: CaptionConfig,

    /// Playback controls
    pub playback: PlaybackConfig,

    /// Regional compliance
    pub regional: RegionalCompliance,
}

impl Default for AccessibilityConfig {
    fn default() -> Self {
        Self {
            wcag_level: WcagLevel::AAA,
            avatar: AvatarAccessibilityConfig::default(),
            captions: CaptionConfig::default(),
            playback: PlaybackConfig::default(),
            regional: RegionalCompliance::default(),
        }
    }
}

/// Avatar accessibility configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AvatarAccessibilityConfig {
    /// Minimum width in pixels
    pub min_width: u32,
    /// Minimum height in pixels
    pub min_height: u32,
    /// Minimum contrast ratio
    pub min_contrast_ratio: f32,
    /// Must show full signing space
    pub full_signing_space: bool,
    /// Face must be visible
    pub face_visible: bool,
    /// Hands must be visible
    pub hands_visible: bool,
    /// User can resize
    pub resizable: bool,
    /// User can reposition
    pub movable: bool,
}

impl Default for AvatarAccessibilityConfig {
    fn default() -> Self {
        Self {
            min_width: 320,
            min_height: 240,
            min_contrast_ratio: 4.5,
            full_signing_space: true,
            face_visible: true,
            hands_visible: true,
            resizable: true,
            movable: true,
        }
    }
}

/// Caption configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaptionConfig {
    /// Captions enabled
    pub enabled: bool,
    /// Synchronize with signs
    pub synchronized: bool,
    /// Font size options
    pub font_sizes: Vec<String>,
    /// Customizable colors
    pub customizable_colors: bool,
    /// Position options
    pub positions: Vec<String>,
}

impl Default for CaptionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            synchronized: true,
            font_sizes: vec![
                "small".to_string(),
                "medium".to_string(),
                "large".to_string(),
                "extra-large".to_string(),
            ],
            customizable_colors: true,
            positions: vec!["top".to_string(), "bottom".to_string()],
        }
    }
}

/// Playback configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlaybackConfig {
    /// Play/pause control
    pub play_pause: bool,
    /// Speed control
    pub speed_control: bool,
    /// Available speeds
    pub speed_options: Vec<f32>,
    /// Seek bar
    pub seek_bar: bool,
    /// Repeat functionality
    pub repeat: bool,
    /// Keyboard accessible
    pub keyboard_accessible: bool,
}

impl Default for PlaybackConfig {
    fn default() -> Self {
        Self {
            play_pause: true,
            speed_control: true,
            speed_options: vec![0.5, 0.75, 1.0, 1.25, 1.5],
            seek_bar: true,
            repeat: true,
            keyboard_accessible: true,
        }
    }
}

/// Regional compliance settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegionalCompliance {
    /// EU EN 301 549 compliance
    pub en301549: bool,
    /// US Section 508 compliance
    pub section508: bool,
    /// Korean accessibility compliance
    pub korean: bool,
}

impl Default for RegionalCompliance {
    fn default() -> Self {
        Self {
            en301549: true,
            section508: true,
            korean: true,
        }
    }
}

/// Accessibility check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityCheckResult {
    /// Overall compliance
    pub compliant: bool,

    /// WCAG level achieved
    pub wcag_level_achieved: WcagLevel,

    /// Individual checks
    pub checks: Vec<AccessibilityCheck>,

    /// Violations found
    pub violations: Vec<AccessibilityViolation>,

    /// Warnings
    pub warnings: Vec<String>,

    /// Recommendations
    pub recommendations: Vec<String>,
}

/// Individual accessibility check
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityCheck {
    pub name: String,
    pub category: String,
    pub passed: bool,
    pub wcag_criterion: String,
}

/// Accessibility violation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityViolation {
    pub code: String,
    pub message: String,
    pub wcag_criterion: String,
    pub severity: ViolationSeverity,
    pub recommendation: String,
}

/// Violation severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViolationSeverity {
    Minor,
    Moderate,
    Serious,
    Critical,
}

/// Accessibility checker
pub struct AccessibilityChecker {
    target_level: WcagLevel,
}

impl AccessibilityChecker {
    /// Create a new accessibility checker
    pub fn new() -> Self {
        Self {
            target_level: WcagLevel::AAA,
        }
    }

    /// Create with specific target level
    pub fn with_level(level: WcagLevel) -> Self {
        Self { target_level: level }
    }

    /// Check accessibility compliance
    pub fn check(&self, config: &AccessibilityConfig) -> AccessibilityCheckResult {
        let mut checks = Vec::new();
        let mut violations = Vec::new();
        let mut warnings = Vec::new();
        let mut recommendations = Vec::new();

        // Check avatar size (WCAG 1.2.6)
        let avatar_size_check = config.avatar.min_width >= 320 && config.avatar.min_height >= 240;
        checks.push(AccessibilityCheck {
            name: "Avatar Minimum Size".to_string(),
            category: "Sign Language".to_string(),
            passed: avatar_size_check,
            wcag_criterion: "1.2.6".to_string(),
        });

        if !avatar_size_check {
            violations.push(AccessibilityViolation {
                code: "AVATAR_SIZE".to_string(),
                message: "Avatar size is below minimum recommended dimensions".to_string(),
                wcag_criterion: "1.2.6".to_string(),
                severity: ViolationSeverity::Serious,
                recommendation: "Ensure avatar display is at least 320x240 pixels".to_string(),
            });
        }

        // Check contrast ratio (WCAG 1.4.3)
        let contrast_check = config.avatar.min_contrast_ratio >= 4.5;
        checks.push(AccessibilityCheck {
            name: "Contrast Ratio".to_string(),
            category: "Visual".to_string(),
            passed: contrast_check,
            wcag_criterion: "1.4.3".to_string(),
        });

        if !contrast_check {
            violations.push(AccessibilityViolation {
                code: "CONTRAST".to_string(),
                message: "Contrast ratio is below minimum".to_string(),
                wcag_criterion: "1.4.3".to_string(),
                severity: ViolationSeverity::Moderate,
                recommendation: "Ensure contrast ratio is at least 4.5:1".to_string(),
            });
        }

        // Check keyboard accessibility (WCAG 2.1.1)
        let keyboard_check = config.playback.keyboard_accessible;
        checks.push(AccessibilityCheck {
            name: "Keyboard Accessibility".to_string(),
            category: "Operable".to_string(),
            passed: keyboard_check,
            wcag_criterion: "2.1.1".to_string(),
        });

        if !keyboard_check {
            violations.push(AccessibilityViolation {
                code: "KEYBOARD".to_string(),
                message: "Keyboard accessibility is not enabled".to_string(),
                wcag_criterion: "2.1.1".to_string(),
                severity: ViolationSeverity::Critical,
                recommendation: "Enable keyboard controls for all functionality".to_string(),
            });
        }

        // Check pause control (WCAG 2.2.2)
        let pause_check = config.playback.play_pause;
        checks.push(AccessibilityCheck {
            name: "Pause Control".to_string(),
            category: "Operable".to_string(),
            passed: pause_check,
            wcag_criterion: "2.2.2".to_string(),
        });

        // Add recommendations
        if config.avatar.resizable {
            recommendations.push("User can resize avatar - good for personalization".to_string());
        }

        if config.playback.speed_control {
            recommendations.push("Speed control available - helpful for learning".to_string());
        } else {
            warnings.push("Consider adding speed control for better accessibility".to_string());
        }

        // Determine compliance level
        let has_critical = violations.iter().any(|v| v.severity == ViolationSeverity::Critical);
        let has_serious = violations.iter().any(|v| v.severity == ViolationSeverity::Serious);

        let level_achieved = if has_critical {
            WcagLevel::A // Failed basic requirements
        } else if has_serious {
            WcagLevel::AA
        } else {
            WcagLevel::AAA
        };

        let compliant = violations.is_empty()
            || (level_achieved as u8 >= self.target_level as u8);

        AccessibilityCheckResult {
            compliant,
            wcag_level_achieved: level_achieved,
            checks,
            violations,
            warnings,
            recommendations,
        }
    }
}

impl Default for AccessibilityChecker {
    fn default() -> Self {
        Self::new()
    }
}

/// Keyboard shortcut definitions
pub struct KeyboardShortcuts;

impl KeyboardShortcuts {
    /// Get standard keyboard shortcuts
    pub fn standard() -> Vec<KeyboardShortcut> {
        vec![
            KeyboardShortcut {
                key: "Space".to_string(),
                action: "Play/Pause".to_string(),
            },
            KeyboardShortcut {
                key: "Enter".to_string(),
                action: "Play/Pause".to_string(),
            },
            KeyboardShortcut {
                key: "ArrowLeft".to_string(),
                action: "Skip back 5s".to_string(),
            },
            KeyboardShortcut {
                key: "ArrowRight".to_string(),
                action: "Skip forward 5s".to_string(),
            },
            KeyboardShortcut {
                key: "ArrowUp".to_string(),
                action: "Volume up".to_string(),
            },
            KeyboardShortcut {
                key: "ArrowDown".to_string(),
                action: "Volume down".to_string(),
            },
            KeyboardShortcut {
                key: "M".to_string(),
                action: "Mute/Unmute".to_string(),
            },
            KeyboardShortcut {
                key: "F".to_string(),
                action: "Fullscreen".to_string(),
            },
            KeyboardShortcut {
                key: "Escape".to_string(),
                action: "Exit fullscreen".to_string(),
            },
            KeyboardShortcut {
                key: "<".to_string(),
                action: "Slower".to_string(),
            },
            KeyboardShortcut {
                key: ">".to_string(),
                action: "Faster".to_string(),
            },
            KeyboardShortcut {
                key: "R".to_string(),
                action: "Repeat current sign".to_string(),
            },
        ]
    }
}

/// Keyboard shortcut definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyboardShortcut {
    pub key: String,
    pub action: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config_compliant() {
        let checker = AccessibilityChecker::new();
        let config = AccessibilityConfig::default();
        let result = checker.check(&config);
        assert!(result.compliant);
        assert_eq!(result.wcag_level_achieved, WcagLevel::AAA);
    }

    #[test]
    fn test_small_avatar_violation() {
        let checker = AccessibilityChecker::new();
        let mut config = AccessibilityConfig::default();
        config.avatar.min_width = 200;
        config.avatar.min_height = 150;

        let result = checker.check(&config);
        assert!(!result.violations.is_empty());
    }

    #[test]
    fn test_keyboard_shortcuts() {
        let shortcuts = KeyboardShortcuts::standard();
        assert!(!shortcuts.is_empty());
        assert!(shortcuts.iter().any(|s| s.key == "Space"));
    }
}
