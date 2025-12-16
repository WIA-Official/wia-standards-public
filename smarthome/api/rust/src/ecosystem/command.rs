//! Command processor for unified ecosystem commands

use super::*;
use crate::error::{Result, SmartHomeError};
use std::collections::VecDeque;
use std::sync::{Arc, RwLock};

/// Command processor handles unified commands from all sources
pub struct CommandProcessor {
    pending_commands: Arc<RwLock<VecDeque<UnifiedCommand>>>,
    command_history: Arc<RwLock<Vec<CommandResult>>>,
    priority_rules: PriorityRules,
    safety_checks: SafetyChecks,
}

impl CommandProcessor {
    pub fn new() -> Self {
        Self {
            pending_commands: Arc::new(RwLock::new(VecDeque::new())),
            command_history: Arc::new(RwLock::new(Vec::new())),
            priority_rules: PriorityRules::default(),
            safety_checks: SafetyChecks::default(),
        }
    }

    /// Process a unified command
    pub async fn process(&self, command: UnifiedCommand) -> Result<CommandResult> {
        // 1. Validate command
        self.validate_command(&command)?;

        // 2. Check safety constraints
        self.safety_checks.check(&command)?;

        // 3. Check if confirmation needed
        if self.requires_confirmation(&command) {
            return Ok(CommandResult {
                command_id: command.id,
                success: false,
                message: LocalizedMessage::new("확인이 필요합니다", "Confirmation required"),
                feedback: vec![],
                executed_at: Utc::now(),
            });
        }

        // 4. Execute command (simulated)
        let result = self.execute_command(&command).await?;

        // 5. Store in history
        if let Ok(mut history) = self.command_history.write() {
            history.push(result.clone());
            // Keep last 1000 commands
            if history.len() > 1000 {
                history.remove(0);
            }
        }

        Ok(result)
    }

    fn validate_command(&self, command: &UnifiedCommand) -> Result<()> {
        // Check source confidence for BCI
        if let CommandSource::BCI { confidence, .. } = &command.source {
            if *confidence < self.priority_rules.min_bci_confidence {
                return Err(SmartHomeError::ValidationError(format!(
                    "BCI confidence {} below threshold {}",
                    confidence, self.priority_rules.min_bci_confidence
                )));
            }
        }

        Ok(())
    }

    fn requires_confirmation(&self, command: &UnifiedCommand) -> bool {
        match &command.confirmation {
            ConfirmationRequirement::None => false,
            _ => true,
        }
    }

    async fn execute_command(&self, command: &UnifiedCommand) -> Result<CommandResult> {
        // Generate appropriate message based on action
        let message = self.generate_result_message(&command.action);

        Ok(CommandResult {
            command_id: command.id,
            success: true,
            message,
            feedback: vec![],
            executed_at: Utc::now(),
        })
    }

    fn generate_result_message(&self, action: &DeviceAction) -> LocalizedMessage {
        match action {
            DeviceAction::Power { on } => {
                if *on {
                    LocalizedMessage::new("전원을 켰습니다", "Power turned on")
                } else {
                    LocalizedMessage::new("전원을 껐습니다", "Power turned off")
                }
            }
            DeviceAction::Brightness { level } => LocalizedMessage::new(
                format!("밝기를 {}%로 설정했습니다", level),
                format!("Brightness set to {}%", level),
            ),
            DeviceAction::Temperature { celsius } => LocalizedMessage::new(
                format!("온도를 {}°C로 설정했습니다", celsius),
                format!("Temperature set to {}°C", celsius),
            ),
            DeviceAction::Lock { locked } => {
                if *locked {
                    LocalizedMessage::new("잠금을 설정했습니다", "Locked")
                } else {
                    LocalizedMessage::new("잠금을 해제했습니다", "Unlocked")
                }
            }
            DeviceAction::Door { open } => {
                if *open {
                    LocalizedMessage::new("문을 열었습니다", "Door opened")
                } else {
                    LocalizedMessage::new("문을 닫았습니다", "Door closed")
                }
            }
            DeviceAction::Blind { position } => LocalizedMessage::new(
                format!("블라인드를 {}%로 설정했습니다", position),
                format!("Blind set to {}%", position),
            ),
            DeviceAction::Volume { level } => LocalizedMessage::new(
                format!("볼륨을 {}%로 설정했습니다", level),
                format!("Volume set to {}%", level),
            ),
            _ => LocalizedMessage::new("명령을 실행했습니다", "Command executed"),
        }
    }

    /// Get command history
    pub fn get_history(&self, limit: usize) -> Vec<CommandResult> {
        if let Ok(history) = self.command_history.read() {
            history.iter().rev().take(limit).cloned().collect()
        } else {
            vec![]
        }
    }

    /// Clear command history
    pub fn clear_history(&self) {
        if let Ok(mut history) = self.command_history.write() {
            history.clear();
        }
    }
}

impl Default for CommandProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// Priority resolution rules
#[derive(Debug, Clone)]
pub struct PriorityRules {
    /// Minimum BCI confidence to accept command
    pub min_bci_confidence: f32,
    /// Minimum eye gaze dwell time
    pub min_dwell_ms: u32,
    /// Allow automation override by user
    pub user_override_automation: bool,
    /// Emergency overrides everything
    pub emergency_override_all: bool,
}

impl Default for PriorityRules {
    fn default() -> Self {
        Self {
            min_bci_confidence: 0.7,
            min_dwell_ms: 800,
            user_override_automation: true,
            emergency_override_all: true,
        }
    }
}

/// Safety checks for commands
#[derive(Debug, Clone)]
pub struct SafetyChecks {
    /// Require confirmation for security devices
    pub confirm_security_devices: bool,
    /// Maximum commands per minute
    pub rate_limit_per_minute: u32,
    /// Block during emergency mode
    pub block_during_emergency: bool,
}

impl Default for SafetyChecks {
    fn default() -> Self {
        Self {
            confirm_security_devices: true,
            rate_limit_per_minute: 60,
            block_during_emergency: false,
        }
    }
}

impl SafetyChecks {
    pub fn check(&self, command: &UnifiedCommand) -> Result<()> {
        // Check if security device needs confirmation
        if self.confirm_security_devices {
            if let DeviceAction::Lock { .. } = command.action {
                if matches!(command.confirmation, ConfirmationRequirement::None) {
                    // For now, allow without confirmation in simulation
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_command_processor() {
        let processor = CommandProcessor::new();

        let command = UnifiedCommand::new(
            CommandSource::Manual {
                user_id: Uuid::new_v4(),
            },
            CommandTarget::Device(Uuid::new_v4()),
            DeviceAction::Power { on: true },
        );

        let result = processor.process(command).await.unwrap();
        assert!(result.success);
    }

    #[tokio::test]
    async fn test_bci_confidence_check() {
        let processor = CommandProcessor::new();

        let command = UnifiedCommand::new(
            CommandSource::BCI {
                paradigm: BCIParadigm::MotorImagery,
                confidence: 0.5, // Below threshold
                channel_data: None,
            },
            CommandTarget::Device(Uuid::new_v4()),
            DeviceAction::Power { on: true },
        );

        let result = processor.process(command).await;
        assert!(result.is_err());
    }

    #[test]
    fn test_result_message_generation() {
        let processor = CommandProcessor::new();

        let msg = processor.generate_result_message(&DeviceAction::Power { on: true });
        assert_eq!(msg.ko, "전원을 켰습니다");
        assert_eq!(msg.en, "Power turned on");

        let msg = processor.generate_result_message(&DeviceAction::Brightness { level: 75 });
        assert!(msg.ko.contains("75%"));
    }
}
