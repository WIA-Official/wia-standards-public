//! WIA-AI Integration Hub

use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{broadcast, RwLock};

use crate::error::WiaAiError;
use super::{
    AiInput, AiOutput, HubEvent, WiaMessage, WiaStandardType,
    connector::{AiInputAdapter, AiOutputAdapter, WiaConnector},
};

/// WIA-AI Integration Hub
///
/// The hub manages connections between WIA AI and other WIA standards,
/// routing messages between input adapters, AI processing, and output adapters.
///
/// # Architecture
///
/// ```text
/// WIA Connectors → Input Adapters → AI Processing → Output Adapters → WIA Connectors
/// ```
///
/// # Example
///
/// ```rust,no_run
/// use wia_ai::integration::*;
///
/// #[tokio::main]
/// async fn main() {
///     let hub = WiaAiHub::new();
///
///     // Register adapters
///     hub.register_input_adapter(Box::new(MockInputAdapter::new(WiaStandardType::Aac))).await;
///     hub.register_output_adapter(Box::new(MockOutputAdapter::new(WiaStandardType::Tts))).await;
///
///     // Process a message
///     let message = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "Hello");
///     let input = hub.process_to_input(message).await.unwrap();
/// }
/// ```
pub struct WiaAiHub {
    /// Registered connectors
    connectors: RwLock<HashMap<String, Arc<dyn WiaConnector>>>,
    /// Input adapters
    input_adapters: RwLock<Vec<Box<dyn AiInputAdapter>>>,
    /// Output adapters
    output_adapters: RwLock<HashMap<WiaStandardType, Box<dyn AiOutputAdapter>>>,
    /// Event broadcaster
    event_tx: broadcast::Sender<HubEvent>,
    /// AI response handler (optional callback)
    response_handler: RwLock<Option<Arc<dyn Fn(AiInput) -> AiOutput + Send + Sync>>>,
}

impl WiaAiHub {
    /// Create a new WIA-AI Hub
    pub fn new() -> Self {
        let (event_tx, _) = broadcast::channel(1024);
        Self {
            connectors: RwLock::new(HashMap::new()),
            input_adapters: RwLock::new(Vec::new()),
            output_adapters: RwLock::new(HashMap::new()),
            event_tx,
            response_handler: RwLock::new(None),
        }
    }

    /// Register a WIA connector
    pub async fn register_connector(&self, connector: Arc<dyn WiaConnector>) {
        let id = connector.id().to_string();
        let standard = connector.standard_type();

        let mut connectors = self.connectors.write().await;
        connectors.insert(id.clone(), connector);

        let _ = self.event_tx.send(HubEvent::ConnectorConnected { id, standard });
    }

    /// Unregister a connector
    pub async fn unregister_connector(&self, id: &str) {
        let mut connectors = self.connectors.write().await;
        if connectors.remove(id).is_some() {
            let _ = self.event_tx.send(HubEvent::ConnectorDisconnected {
                id: id.to_string(),
            });
        }
    }

    /// Register an input adapter
    pub async fn register_input_adapter(&self, adapter: Box<dyn AiInputAdapter>) {
        let mut adapters = self.input_adapters.write().await;
        adapters.push(adapter);
    }

    /// Register an output adapter
    pub async fn register_output_adapter(&self, adapter: Box<dyn AiOutputAdapter>) {
        let mut adapters = self.output_adapters.write().await;
        adapters.insert(adapter.target_standard(), adapter);
    }

    /// Set the AI response handler
    pub async fn set_response_handler<F>(&self, handler: F)
    where
        F: Fn(AiInput) -> AiOutput + Send + Sync + 'static,
    {
        let mut rh = self.response_handler.write().await;
        *rh = Some(Arc::new(handler));
    }

    /// Process a WIA message to AI input
    pub async fn process_to_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        let adapters = self.input_adapters.read().await;

        // Find an adapter that can handle this message
        let adapter = adapters
            .iter()
            .find(|a| a.can_handle(&message))
            .ok_or_else(|| {
                WiaAiError::NotSupported(format!(
                    "No input adapter for {:?}",
                    message.source
                ))
            })?;

        // Convert to AI input
        let input = adapter.to_ai_input(message.clone()).await?;

        // Emit event
        let _ = self.event_tx.send(HubEvent::MessageReceived { message });

        Ok(input)
    }

    /// Convert AI output to WIA messages for specified targets
    pub async fn process_to_output(
        &self,
        output: AiOutput,
        targets: Vec<WiaStandardType>,
    ) -> Result<Vec<WiaMessage>, WiaAiError> {
        let adapters = self.output_adapters.read().await;
        let mut messages = Vec::new();

        for target in targets {
            if let Some(adapter) = adapters.get(&target) {
                let message = adapter.from_ai_output(output.clone()).await?;
                messages.push(message.clone());

                // Emit event
                let _ = self.event_tx.send(HubEvent::MessageSent { message });
            }
        }

        if messages.is_empty() {
            return Err(WiaAiError::NotSupported(
                "No output adapters found for targets".into(),
            ));
        }

        Ok(messages)
    }

    /// Full conversation pipeline: input → AI → output
    pub async fn conversation(
        &self,
        message: WiaMessage,
        output_targets: Vec<WiaStandardType>,
    ) -> Result<Vec<WiaMessage>, WiaAiError> {
        // 1. Process input
        let ai_input = self.process_to_input(message).await?;

        // 2. Get AI response
        let ai_output = {
            let handler = self.response_handler.read().await;
            match &*handler {
                Some(h) => h(ai_input),
                None => {
                    // Default echo response
                    AiOutput::text(
                        ai_input.text.unwrap_or_else(|| "[no text input]".into())
                    )
                }
            }
        };

        // 3. Process output
        let messages = self.process_to_output(ai_output, output_targets).await?;

        Ok(messages)
    }

    /// Send a message through a connector
    pub async fn send_via_connector(
        &self,
        connector_id: &str,
        message: WiaMessage,
    ) -> Result<(), WiaAiError> {
        let connectors = self.connectors.read().await;

        let connector = connectors
            .get(connector_id)
            .ok_or_else(|| WiaAiError::NotFound(format!("Connector: {}", connector_id)))?;

        connector.send(message).await
    }

    /// Broadcast a message to all connectors of a specific standard type
    pub async fn broadcast(
        &self,
        standard: WiaStandardType,
        message: WiaMessage,
    ) -> Result<usize, WiaAiError> {
        let connectors = self.connectors.read().await;
        let mut sent_count = 0;

        for connector in connectors.values() {
            if connector.standard_type() == standard {
                if let Ok(()) = connector.send(message.clone()).await {
                    sent_count += 1;
                }
            }
        }

        Ok(sent_count)
    }

    /// Get a list of registered connector IDs
    pub async fn list_connectors(&self) -> Vec<(String, WiaStandardType)> {
        let connectors = self.connectors.read().await;
        connectors
            .iter()
            .map(|(id, c)| (id.clone(), c.standard_type()))
            .collect()
    }

    /// Get a list of registered input adapter names
    pub async fn list_input_adapters(&self) -> Vec<String> {
        let adapters = self.input_adapters.read().await;
        adapters.iter().map(|a| a.name().to_string()).collect()
    }

    /// Get a list of registered output adapter targets
    pub async fn list_output_adapters(&self) -> Vec<WiaStandardType> {
        let adapters = self.output_adapters.read().await;
        adapters.keys().cloned().collect()
    }

    /// Subscribe to hub events
    pub fn subscribe(&self) -> broadcast::Receiver<HubEvent> {
        self.event_tx.subscribe()
    }

    /// Get the number of active connectors
    pub async fn connector_count(&self) -> usize {
        self.connectors.read().await.len()
    }

    /// Check if a connector exists
    pub async fn has_connector(&self, id: &str) -> bool {
        self.connectors.read().await.contains_key(id)
    }
}

impl Default for WiaAiHub {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::mock::*;

    #[tokio::test]
    async fn test_hub_creation() {
        let hub = WiaAiHub::new();
        assert_eq!(hub.connector_count().await, 0);
    }

    #[tokio::test]
    async fn test_register_input_adapter() {
        let hub = WiaAiHub::new();
        let adapter = MockInputAdapter::new(WiaStandardType::Aac);

        hub.register_input_adapter(Box::new(adapter)).await;

        let adapters = hub.list_input_adapters().await;
        assert_eq!(adapters.len(), 1);
    }

    #[tokio::test]
    async fn test_register_output_adapter() {
        let hub = WiaAiHub::new();
        let adapter = MockOutputAdapter::new(WiaStandardType::Tts);

        hub.register_output_adapter(Box::new(adapter)).await;

        let adapters = hub.list_output_adapters().await;
        assert!(adapters.contains(&WiaStandardType::Tts));
    }

    #[tokio::test]
    async fn test_process_to_input() {
        let hub = WiaAiHub::new();
        let adapter = MockInputAdapter::new(WiaStandardType::Aac);
        hub.register_input_adapter(Box::new(adapter)).await;

        let message = WiaMessage::text(
            WiaStandardType::Aac,
            WiaStandardType::Ai,
            "Hello",
        );

        let input = hub.process_to_input(message).await.unwrap();
        assert!(input.text.is_some());
    }

    #[tokio::test]
    async fn test_process_to_output() {
        let hub = WiaAiHub::new();
        let adapter = MockOutputAdapter::new(WiaStandardType::Tts);
        hub.register_output_adapter(Box::new(adapter)).await;

        let output = AiOutput::text("Hello");
        let messages = hub
            .process_to_output(output, vec![WiaStandardType::Tts])
            .await
            .unwrap();

        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].target, WiaStandardType::Tts);
    }

    #[tokio::test]
    async fn test_conversation() {
        let hub = WiaAiHub::new();

        // Register input adapter
        hub.register_input_adapter(Box::new(MockInputAdapter::new(WiaStandardType::Aac)))
            .await;

        // Register output adapter
        hub.register_output_adapter(Box::new(MockOutputAdapter::new(WiaStandardType::Tts)))
            .await;

        // Set response handler
        hub.set_response_handler(|input| {
            AiOutput::text(format!(
                "Response to: {}",
                input.text.unwrap_or_default()
            ))
        })
        .await;

        // Run conversation
        let message = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "Hello");
        let responses = hub
            .conversation(message, vec![WiaStandardType::Tts])
            .await
            .unwrap();

        assert_eq!(responses.len(), 1);
    }
}
