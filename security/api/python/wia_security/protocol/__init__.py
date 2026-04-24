"""
WIA Security Communication Protocol
Phase 3: WebSocket, TAXII 2.1, SIEM adapters
"""

from .websocket import (
    WebSocketConfig,
    WebSocketMessage,
    SubscriptionFilter,
    WiaWebSocketClient,
    create_websocket_client,
)

from .taxii import (
    TaxiiConfig,
    TaxiiDiscovery,
    TaxiiApiRoot,
    TaxiiCollection,
    TaxiiStatus,
    TaxiiManifestEntry,
    TaxiiClient,
    create_taxii_client,
)

from .siem import (
    SplunkConfig,
    SplunkAdapter,
    ElasticConfig,
    ElasticAdapter,
    QRadarConfig,
    QRadarAdapter,
    create_splunk_adapter,
    create_elastic_adapter,
    create_qradar_adapter,
)

__all__ = [
    # WebSocket
    "WebSocketConfig",
    "WebSocketMessage",
    "SubscriptionFilter",
    "WiaWebSocketClient",
    "create_websocket_client",
    # TAXII
    "TaxiiConfig",
    "TaxiiDiscovery",
    "TaxiiApiRoot",
    "TaxiiCollection",
    "TaxiiStatus",
    "TaxiiManifestEntry",
    "TaxiiClient",
    "create_taxii_client",
    # SIEM
    "SplunkConfig",
    "SplunkAdapter",
    "ElasticConfig",
    "ElasticAdapter",
    "QRadarConfig",
    "QRadarAdapter",
    "create_splunk_adapter",
    "create_elastic_adapter",
    "create_qradar_adapter",
]
