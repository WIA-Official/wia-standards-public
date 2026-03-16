# WIA-UNI-007 - Phase 2: API Interface

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 2 defines RESTful APIs for power grid management, energy trading, and real-time monitoring. All endpoints follow REST principles with JSON payloads conforming to Phase 1 data formats.

## 2. Base URL

```
https://api.powergrid.wia/{region}/v1/
```

Regions: `kr-south`, `kr-north`, `unified`

## 3. Authentication

### 3.1 OAuth 2.0

All API requests require OAuth 2.0 Bearer tokens:

```
Authorization: Bearer {access_token}
```

### 3.2 Client Credentials Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id={client_id}&
client_secret={client_secret}&
scope=grid.read grid.write
```

## 4. Core Endpoints

### 4.1 Grid Nodes

**List Grid Nodes**
```http
GET /nodes?type={node_type}&region={region}&status={status}
```

**Get Node Details**
```http
GET /nodes/{node_id}
```

**Create Node** (requires `grid.write` scope)
```http
POST /nodes
Content-Type: application/json

{PowerGridNode JSON from Phase 1}
```

### 4.2 Power Flow

**Real-time Power Flow**
```http
GET /power-flow/{node_id}
```

**Historical Data**
```http
GET /power-flow/{node_id}/history?start={ISO8601}&end={ISO8601}&interval=5s
```

### 4.3 Energy Trading

**Submit Trade Order**
```http
POST /trading/orders
{
  "orderType": "buy|sell",
  "quantity": 500,
  "unit": "MWh",
  "price": 50.00,
  "currency": "USD",
  "deliveryTime": "ISO 8601",
  "source": "renewable|conventional"
}
```

**Get Market Prices**
```http
GET /trading/prices?interval=hourly&start={date}
```

### 4.4 Forecasting

**Load Forecast**
```http
GET /forecasts/load?region={region}&horizon=24h
```

**Renewable Generation Forecast**
```http
GET /forecasts/renewable/{source_id}?horizon=24h
```

## 5. WebSocket Streaming

Real-time updates via WebSocket:

```
wss://stream.powergrid.wia/v1/grid-status
```

Subscribe to specific nodes:
```json
{
  "action": "subscribe",
  "channels": ["power-flow:HVDC-WS-001", "alerts:all"]
}
```

## 6. Rate Limits

- Standard: 1,000 requests/hour
- Premium: 10,000 requests/hour
- Real-time streaming: No limits

## 7. Error Handling

Standard HTTP status codes with detailed error responses:

```json
{
  "error": {
    "code": "INVALID_NODE_ID",
    "message": "Node ID not found",
    "details": {}
  }
}
```

