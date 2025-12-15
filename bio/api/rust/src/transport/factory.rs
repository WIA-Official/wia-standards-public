//! Transport factory for creating transport instances

use super::base::*;
use super::mock::MockTransport;
use super::websocket::WebSocketTransport;

/// Transport types available
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportType {
    /// WebSocket transport
    WebSocket,
    /// Mock transport for testing
    Mock,
    /// USB transport (future)
    Usb,
    /// Bluetooth LE transport (future)
    BluetoothLe,
    /// Serial port transport (future)
    Serial,
}

impl TransportType {
    /// Get the type name
    pub fn name(&self) -> &str {
        match self {
            TransportType::WebSocket => "websocket",
            TransportType::Mock => "mock",
            TransportType::Usb => "usb",
            TransportType::BluetoothLe => "bluetooth_le",
            TransportType::Serial => "serial",
        }
    }

    /// Check if this transport type is available
    pub fn is_available(&self) -> bool {
        matches!(self, TransportType::WebSocket | TransportType::Mock)
    }
}

/// Factory for creating transport instances
pub struct TransportFactory;

impl TransportFactory {
    /// Create a transport of the specified type
    pub fn create(transport_type: TransportType) -> Box<dyn ITransport> {
        match transport_type {
            TransportType::WebSocket => Box::new(WebSocketTransport::new()),
            TransportType::Mock => Box::new(MockTransport::new()),
            _ => {
                // Return mock for unsupported types
                // In a real implementation, this would return an error
                tracing::warn!(
                    "Transport type {:?} not yet implemented, using mock",
                    transport_type
                );
                Box::new(MockTransport::new())
            }
        }
    }

    /// Create a WebSocket transport
    pub fn websocket() -> WebSocketTransport {
        WebSocketTransport::new()
    }

    /// Create a mock transport
    pub fn mock() -> MockTransport {
        MockTransport::new()
    }

    /// Get list of available transport types
    pub fn available_types() -> Vec<TransportType> {
        vec![TransportType::WebSocket, TransportType::Mock]
    }

    /// Check if a transport type is supported
    pub fn is_supported(transport_type: TransportType) -> bool {
        transport_type.is_available()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_type_names() {
        assert_eq!(TransportType::WebSocket.name(), "websocket");
        assert_eq!(TransportType::Mock.name(), "mock");
        assert_eq!(TransportType::Usb.name(), "usb");
    }

    #[test]
    fn test_transport_availability() {
        assert!(TransportType::WebSocket.is_available());
        assert!(TransportType::Mock.is_available());
        assert!(!TransportType::Usb.is_available());
        assert!(!TransportType::BluetoothLe.is_available());
    }

    #[test]
    fn test_factory_create() {
        let ws = TransportFactory::create(TransportType::WebSocket);
        assert_eq!(ws.transport_type(), "websocket");

        let mock = TransportFactory::create(TransportType::Mock);
        assert_eq!(mock.transport_type(), "mock");
    }

    #[test]
    fn test_factory_helpers() {
        let ws = TransportFactory::websocket();
        assert_eq!(ws.transport_type(), "websocket");

        let mock = TransportFactory::mock();
        assert_eq!(mock.transport_type(), "mock");
    }

    #[test]
    fn test_available_types() {
        let types = TransportFactory::available_types();
        assert!(types.contains(&TransportType::WebSocket));
        assert!(types.contains(&TransportType::Mock));
        assert!(!types.contains(&TransportType::Usb));
    }

    #[test]
    fn test_is_supported() {
        assert!(TransportFactory::is_supported(TransportType::WebSocket));
        assert!(TransportFactory::is_supported(TransportType::Mock));
        assert!(!TransportFactory::is_supported(TransportType::Usb));
    }
}
