//! Integration Manager for coordinating multiple adapters

use super::traits::*;
use std::collections::HashMap;

/// Integration preferences
#[derive(Debug, Clone)]
pub struct IntegrationPreferences {
    /// Enabled integrations
    pub enabled_integrations: Vec<IntegrationType>,

    /// Primary archive type
    pub primary_archive: IntegrationType,

    /// Auto-export enabled
    pub auto_export: bool,

    /// Export interval in milliseconds
    pub export_interval_ms: u64,
}

impl Default for IntegrationPreferences {
    fn default() -> Self {
        Self {
            enabled_integrations: vec![IntegrationType::Hdf5],
            primary_archive: IntegrationType::Hdf5,
            auto_export: false,
            export_interval_ms: 1000,
        }
    }
}

/// Integration manager for coordinating multiple adapters
pub struct IntegrationManager {
    /// Registered adapters
    adapters: HashMap<IntegrationType, Box<dyn IntegrationAdapter>>,

    /// Preferences
    preferences: IntegrationPreferences,

    /// Event handlers
    event_handlers: Vec<Box<dyn Fn(&IntegrationEventData) + Send + Sync>>,
}

impl IntegrationManager {
    /// Create new integration manager
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
            preferences: IntegrationPreferences::default(),
            event_handlers: Vec::new(),
        }
    }

    /// Create with preferences
    pub fn with_preferences(preferences: IntegrationPreferences) -> Self {
        Self {
            adapters: HashMap::new(),
            preferences,
            event_handlers: Vec::new(),
        }
    }

    /// Register an adapter
    pub fn register(&mut self, adapter: Box<dyn IntegrationAdapter>) {
        let integration_type = adapter.integration_type();
        self.adapters.insert(integration_type, adapter);
        self.emit_event(IntegrationEvent::AdapterRegistered, integration_type, None);
    }

    /// Unregister an adapter
    pub fn unregister(&mut self, integration_type: IntegrationType) -> Option<Box<dyn IntegrationAdapter>> {
        let adapter = self.adapters.remove(&integration_type);
        if adapter.is_some() {
            self.emit_event(IntegrationEvent::AdapterUnregistered, integration_type, None);
        }
        adapter
    }

    /// Get adapter reference
    pub fn get(&self, integration_type: IntegrationType) -> Option<&dyn IntegrationAdapter> {
        self.adapters.get(&integration_type).map(|a| a.as_ref())
    }

    /// Get mutable adapter reference
    pub fn get_mut(&mut self, integration_type: IntegrationType) -> Option<&mut Box<dyn IntegrationAdapter>> {
        self.adapters.get_mut(&integration_type)
    }

    /// Get all registered adapters
    pub fn get_all(&self) -> Vec<&dyn IntegrationAdapter> {
        self.adapters.values().map(|a| a.as_ref()).collect()
    }

    /// Get available adapters
    pub fn get_available(&self) -> Vec<&dyn IntegrationAdapter> {
        self.adapters
            .values()
            .filter(|a| a.is_available())
            .map(|a| a.as_ref())
            .collect()
    }

    /// Initialize all adapters
    pub fn initialize_all(&mut self, options: HashMap<IntegrationType, IntegrationOptions>) -> Result<(), IntegrationError> {
        for (integration_type, opts) in options {
            if let Some(adapter) = self.adapters.get_mut(&integration_type) {
                adapter.initialize(opts)?;
            }
        }
        Ok(())
    }

    /// Connect all adapters
    pub fn connect_all(&mut self) -> Vec<Result<(), IntegrationError>> {
        let mut results = Vec::new();
        let types: Vec<_> = self.adapters.keys().cloned().collect();

        for integration_type in types {
            if let Some(adapter) = self.adapters.get_mut(&integration_type) {
                let result = adapter.connect();
                if result.is_ok() {
                    self.emit_event(IntegrationEvent::Connected, integration_type, None);
                }
                results.push(result);
            }
        }

        results
    }

    /// Disconnect all adapters
    pub fn disconnect_all(&mut self) -> Vec<Result<(), IntegrationError>> {
        let mut results = Vec::new();
        let types: Vec<_> = self.adapters.keys().cloned().collect();

        for integration_type in types {
            if let Some(adapter) = self.adapters.get_mut(&integration_type) {
                let result = adapter.disconnect();
                if result.is_ok() {
                    self.emit_event(IntegrationEvent::Disconnected, integration_type, None);
                }
                results.push(result);
            }
        }

        results
    }

    /// Export to all enabled integrations
    pub fn export_all(&self, data: &PhysicsData) -> Vec<ExportResult> {
        let mut results = Vec::new();

        for integration_type in &self.preferences.enabled_integrations {
            if let Some(adapter) = self.adapters.get(integration_type) {
                if adapter.is_available() {
                    self.emit_event(IntegrationEvent::ExportStarted, *integration_type, None);

                    match adapter.export(data) {
                        Ok(result) => {
                            self.emit_event(
                                IntegrationEvent::ExportCompleted,
                                *integration_type,
                                Some(format!("{} records", result.records_exported)),
                            );
                            results.push(result);
                        }
                        Err(e) => {
                            self.emit_event(
                                IntegrationEvent::ExportFailed,
                                *integration_type,
                                Some(e.to_string()),
                            );
                        }
                    }
                }
            }
        }

        results
    }

    /// Export to specific integration
    pub fn export_to(&self, integration_type: IntegrationType, data: &PhysicsData) -> Result<ExportResult, IntegrationError> {
        let adapter = self.adapters.get(&integration_type)
            .ok_or(IntegrationError::AdapterNotFound(integration_type))?;

        if !adapter.is_available() {
            return Err(IntegrationError::AdapterNotAvailable(integration_type));
        }

        self.emit_event(IntegrationEvent::ExportStarted, integration_type, None);

        match adapter.export(data) {
            Ok(result) => {
                self.emit_event(
                    IntegrationEvent::ExportCompleted,
                    integration_type,
                    Some(format!("{} records", result.records_exported)),
                );
                Ok(result)
            }
            Err(e) => {
                self.emit_event(
                    IntegrationEvent::ExportFailed,
                    integration_type,
                    Some(e.to_string()),
                );
                Err(e)
            }
        }
    }

    /// Import from specific integration
    pub fn import_from(&self, integration_type: IntegrationType, source: &str) -> Result<PhysicsData, IntegrationError> {
        let adapter = self.adapters.get(&integration_type)
            .ok_or(IntegrationError::AdapterNotFound(integration_type))?;

        if !adapter.is_available() {
            return Err(IntegrationError::AdapterNotAvailable(integration_type));
        }

        self.emit_event(IntegrationEvent::ImportStarted, integration_type, None);

        match adapter.import(source) {
            Ok(data) => {
                self.emit_event(IntegrationEvent::ImportCompleted, integration_type, None);
                Ok(data)
            }
            Err(e) => {
                self.emit_event(
                    IntegrationEvent::ImportFailed,
                    integration_type,
                    Some(e.to_string()),
                );
                Err(e)
            }
        }
    }

    /// Register event handler
    pub fn on_event<F>(&mut self, handler: F)
    where
        F: Fn(&IntegrationEventData) + Send + Sync + 'static,
    {
        self.event_handlers.push(Box::new(handler));
    }

    /// Set preferences
    pub fn set_preferences(&mut self, preferences: IntegrationPreferences) {
        self.preferences = preferences;
    }

    /// Get preferences
    pub fn preferences(&self) -> &IntegrationPreferences {
        &self.preferences
    }

    /// Dispose all adapters
    pub fn dispose(&mut self) -> Vec<Result<(), IntegrationError>> {
        let mut results = Vec::new();
        let types: Vec<_> = self.adapters.keys().cloned().collect();

        for integration_type in types {
            if let Some(adapter) = self.adapters.get_mut(&integration_type) {
                results.push(adapter.dispose());
            }
        }

        self.adapters.clear();
        results
    }

    /// Emit event to handlers
    fn emit_event(&self, event: IntegrationEvent, integration_type: IntegrationType, details: Option<String>) {
        let event_data = IntegrationEventData {
            event,
            integration_type,
            timestamp: chrono::Utc::now().timestamp_millis(),
            details,
        };

        for handler in &self.event_handlers {
            handler(&event_data);
        }
    }
}

impl Default for IntegrationManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::hdf5::HDF5Adapter;

    #[test]
    fn test_manager_creation() {
        let manager = IntegrationManager::new();
        assert!(manager.get_all().is_empty());
    }

    #[test]
    fn test_adapter_registration() {
        let mut manager = IntegrationManager::new();
        manager.register(Box::new(HDF5Adapter::new()));

        assert!(manager.get(IntegrationType::Hdf5).is_some());
        assert_eq!(manager.get_all().len(), 1);
    }

    #[test]
    fn test_adapter_unregistration() {
        let mut manager = IntegrationManager::new();
        manager.register(Box::new(HDF5Adapter::new()));

        let removed = manager.unregister(IntegrationType::Hdf5);
        assert!(removed.is_some());
        assert!(manager.get(IntegrationType::Hdf5).is_none());
    }

    #[test]
    fn test_preferences() {
        let prefs = IntegrationPreferences {
            enabled_integrations: vec![IntegrationType::Hdf5, IntegrationType::InfluxDb],
            primary_archive: IntegrationType::Hdf5,
            auto_export: true,
            export_interval_ms: 5000,
        };

        let manager = IntegrationManager::with_preferences(prefs.clone());
        assert!(manager.preferences().auto_export);
        assert_eq!(manager.preferences().export_interval_ms, 5000);
    }
}
