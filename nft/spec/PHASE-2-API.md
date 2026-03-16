# WIA-FIN-009: NFT Standard - Phase 2: API

## Overview

Phase 2 defines RESTful APIs, SDKs, and integration patterns for NFT operations including minting, transferring, querying, and marketplace interactions.

## REST API Specification

### Authentication

```http
Authorization: Bearer {access_token}
X-API-Key: {api_key}
```

### Endpoints

#### Get NFT Metadata

```http
GET /api/v1/nft/{contractAddress}/{tokenId}

Response:
{
  "contract": "0x...",
  "tokenId": "1",
  "owner": "0x...",
  "metadata": {
    "name": "NFT #1",
    "image": "ipfs://...",
    "attributes": []
  },
  "tokenURI": "ipfs://..."
}
```

#### Mint NFT

```http
POST /api/v1/nft/mint

Request:
{
  "recipient": "0x...",
  "metadata": {},
  "royaltyPercentage": 10
}

Response:
{
  "tokenId": "1",
  "transactionHash": "0x...",
  "blockNumber": 12345,
  "gasUsed": "150000"
}
```

#### Transfer NFT

```http
POST /api/v1/nft/transfer

Request:
{
  "from": "0x...",
  "to": "0x...",
  "tokenId": "1",
  "contractAddress": "0x..."
}
```

#### List for Sale

```http
POST /api/v1/marketplace/list

Request:
{
  "contractAddress": "0x...",
  "tokenId": "1",
  "price": "1.5",
  "currency": "ETH",
  "duration": 86400
}
```

## SDK Reference

### TypeScript SDK

```typescript
import { NFTClient } from '@wia/nft-sdk';

const client = new NFTClient({
  apiKey: 'your-api-key',
  network: 'mainnet'
});

// Mint NFT
const result = await client.mint({
  recipient: '0x...',
  metadata: {
    name: 'My NFT',
    image: 'ipfs://...'
  },
  royaltyPercentage: 10
});

// Get NFT
const nft = await client.getNFT('0xContract...', '1');

// Transfer
await client.transfer({
  from: '0x...',
  to: '0x...',
  tokenId: '1'
});
```

### Python SDK

```python
from wia_nft import NFTClient

client = NFTClient(api_key='your-api-key', network='mainnet')

# Mint NFT
result = client.mint(
    recipient='0x...',
    metadata={'name': 'My NFT', 'image': 'ipfs://...'},
    royalty_percentage=10
)

# Get NFT
nft = client.get_nft('0xContract...', '1')
```

## Marketplace API

### Listing Management

```http
GET /api/v1/marketplace/listings
POST /api/v1/marketplace/buy/{listingId}
DELETE /api/v1/marketplace/cancel/{listingId}
```

### Auction System

```http
POST /api/v1/auction/create
POST /api/v1/auction/{auctionId}/bid
GET /api/v1/auction/{auctionId}/bids
POST /api/v1/auction/{auctionId}/settle
```

## Metadata API

### Upload to IPFS

```http
POST /api/v1/ipfs/upload

Content-Type: multipart/form-data

Response:
{
  "cid": "QmHash...",
  "url": "ipfs://QmHash...",
  "size": 1234567,
  "pinned": true
}
```

### Query Metadata

```http
GET /api/v1/metadata/{cid}

Response:
{
  "name": "NFT Name",
  "description": "Description",
  "image": "ipfs://...",
  "attributes": []
}
```

## Royalty API

### Calculate Royalty

```http
POST /api/v1/royalty/calculate

Request:
{
  "contractAddress": "0x...",
  "tokenId": "1",
  "salePrice": "10.0"
}

Response:
{
  "receiver": "0x...",
  "royaltyAmount": "0.75",
  "royaltyPercentage": 7.5
}
```

## Analytics API

### Collection Stats

```http
GET /api/v1/analytics/collection/{contractAddress}

Response:
{
  "totalSupply": 10000,
  "uniqueOwners": 3456,
  "floorPrice": "0.5",
  "volumeTraded": "1234.5",
  "totalSales": 5678
}
```

## WebSocket API

### Real-time Updates

```javascript
const ws = new WebSocket('wss://api.wia.org/v1/ws');

ws.on('connect', () => {
  ws.subscribe('nft:transfer', {
    contractAddress: '0x...'
  });
});

ws.on('nft:transfer', (event) => {
  console.log('Transfer:', event);
});
```

## Rate Limiting

- **Free Tier**: 100 requests/minute
- **Pro Tier**: 1000 requests/minute
- **Enterprise**: Custom limits

## Error Handling

```json
{
  "error": {
    "code": "INSUFFICIENT_BALANCE",
    "message": "Wallet has insufficient funds",
    "details": {
      "required": "0.1 ETH",
      "available": "0.05 ETH"
    }
  }
}
```

## Best Practices

1. **Caching**: Implement client-side caching
2. **Retry Logic**: Exponential backoff for failures
3. **Pagination**: Use cursor-based pagination
4. **Webhooks**: Subscribe to events vs polling
5. **Idempotency**: Use idempotency keys for mutations

---

**Status**: Draft
**Version**: 1.0.0
**Last Updated**: 2025-01-15

© 2025 SmileStory Inc. / WIA
