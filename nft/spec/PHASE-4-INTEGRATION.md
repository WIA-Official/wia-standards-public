# WIA-FIN-009: NFT Standard - Phase 4: Integration

## Overview

Phase 4 defines integration patterns with marketplaces, wallets, metaverse platforms, and enterprise systems for comprehensive NFT ecosystem connectivity.

## Marketplace Integration

### OpenSea Integration

#### Contract Metadata

```solidity
contract OpenSeaCompatible is ERC721 {
    function contractURI() public pure returns (string memory) {
        return "ipfs://QmCollectionMetadata...";
    }

    // Gasless listing support
    function isApprovedForAll(address owner, address operator)
        public view override returns (bool)
    {
        // OpenSea proxy registry
        if (operator == openSeaProxyRegistry) {
            return true;
        }
        return super.isApprovedForAll(owner, operator);
    }
}
```

#### Collection Metadata

```json
{
  "name": "Collection Name",
  "description": "Collection description",
  "image": "ipfs://QmImage...",
  "external_link": "https://project.com",
  "seller_fee_basis_points": 750,
  "fee_recipient": "0xCreator..."
}
```

### Rarible Integration

```javascript
import { RaribleSdk } from "@rarible/sdk";

const sdk = RaribleSdk.create({
    connector: walletConnector,
    environment: "mainnet"
});

// Create sell order
const order = await sdk.order.sell({
    itemId: `ETHEREUM:${contractAddress}:${tokenId}`,
    amount: 1,
    price: "1.5",
    currency: "ETH"
});
```

### Blur Integration

```javascript
// Blur API integration
const listing = await fetch('https://api.blur.io/v1/listings', {
    method: 'POST',
    headers: {
        'Authorization': `Bearer ${apiKey}`
    },
    body: JSON.stringify({
        contractAddress,
        tokenId,
        price: "1.5 ETH"
    })
});
```

## Wallet Integration

### MetaMask

```javascript
import Web3 from 'web3';

// Connect wallet
const web3 = new Web3(window.ethereum);
await window.ethereum.request({ method: 'eth_requestAccounts' });

// Mint NFT
const contract = new web3.eth.Contract(ABI, contractAddress);
const receipt = await contract.methods.mint(recipient, tokenURI).send({
    from: userAddress,
    value: web3.utils.toWei('0.08', 'ether')
});
```

### WalletConnect

```javascript
import WalletConnect from "@walletconnect/client";

const connector = new WalletConnect({
    bridge: "https://bridge.walletconnect.org",
    qrcodeModal: QRCodeModal
});

// Connect
if (!connector.connected) {
    await connector.createSession();
}

// Send transaction
const result = await connector.sendTransaction({
    from: address,
    to: contractAddress,
    data: mintData,
    value: "0x..."
});
```

### Coinbase Wallet

```javascript
import CoinbaseWalletSDK from '@coinbase/wallet-sdk';

const coinbaseWallet = new CoinbaseWalletSDK({
    appName: 'My NFT App',
    appLogoUrl: 'https://...'
});

const provider = coinbaseWallet.makeWeb3Provider();
await provider.request({ method: 'eth_requestAccounts' });
```

## Metaverse Integration

### Decentraland

```javascript
// Wearable NFT metadata
{
  "id": "urn:decentraland:ethereum:collections-v2:0x...:1",
  "name": "Sneakers",
  "description": "Limited edition sneakers",
  "i18n": {
    "en": { "name": "Sneakers" },
    "es": { "name": "Zapatillas" }
  },
  "data": {
    "replaces": [],
    "hides": [],
    "tags": ["footwear", "sneakers"],
    "category": "feet",
    "representations": [{
      "bodyShapes": ["urn:decentraland:off-chain:base-avatars:BaseFemale"],
      "mainFile": "model.glb",
      "contents": ["model.glb", "texture.png"]
    }]
  }
}
```

### The Sandbox

```javascript
// LAND metadata for The Sandbox
{
  "name": "Premium LAND",
  "description": "Prime location near center",
  "image": "ipfs://...",
  "attributes": [{
    "trait_type": "Size",
    "value": "3x3"
  }, {
    "trait_type": "District",
    "value": "Center"
  }],
  "properties": {
    "coordinates": "100,150",
    "elevation": "10"
  }
}
```

### VRChat

```csharp
// Unity integration for VRChat
public class NFTAvatar : MonoBehaviour
{
    private string nftContractAddress;
    private string tokenId;

    async void LoadNFT()
    {
        var metadata = await NFTClient.GetMetadata(nftContractAddress, tokenId);
        var texture = await LoadTextureFromIPFS(metadata.image);
        ApplyTextureToAvatar(texture);
    }
}
```

## Gaming Integration

### Unity SDK

```csharp
using WIA.NFT;

public class NFTInventory : MonoBehaviour
{
    private NFTClient nftClient;

    void Start()
    {
        nftClient = new NFTClient("your-api-key");
    }

    public async Task<List<NFT>> GetPlayerNFTs(string walletAddress)
    {
        return await nftClient.GetNFTsByOwner(walletAddress);
    }

    public async Task UseNFTItem(string tokenId)
    {
        var nft = await nftClient.GetNFT(contractAddress, tokenId);
        ApplyNFTBoost(nft.attributes);
    }
}
```

### Unreal Engine

```cpp
// Unreal Engine C++ integration
#include "NFTClient.h"

void APlayerCharacter::LoadNFTSkin(FString TokenId)
{
    UNFTClient* Client = UNFTClient::Create();
    Client->GetNFT(ContractAddress, TokenId,
        [this](FNFTMetadata Metadata) {
            LoadTextureFromURL(Metadata.Image);
        }
    );
}
```

### Web3.js Gaming

```javascript
class GameNFT {
    constructor(web3, contractAddress) {
        this.contract = new web3.eth.Contract(ABI, contractAddress);
    }

    async equipItem(tokenId) {
        const metadata = await this.contract.methods
            .tokenURI(tokenId)
            .call();
        const stats = await this.parseStats(metadata);
        this.applyStats(stats);
    }
}
```

## Social Media Integration

### Twitter NFT Verification

```html
<meta name="twitter:card" content="summary_large_image">
<meta name="twitter:title" content="NFT Title">
<meta name="twitter:image" content="ipfs://...">
<meta property="eth:nft:collection" content="0x...">
<meta property="eth:nft:contract_address" content="0x...">
<meta property="eth:nft:token_id" content="1">
<meta property="eth:nft:chain" content="ethereum">
```

### Instagram NFT Support

```json
{
  "platform": "instagram",
  "nft": {
    "contract_address": "0x...",
    "token_id": "1",
    "chain": "ethereum",
    "creator": "@username"
  }
}
```

## Enterprise Integration

### ERP Systems

```javascript
// SAP integration example
class NFTInventoryModule {
    async syncWithSAP(nftData) {
        await sapClient.post('/inventory/create', {
            material_number: nftData.tokenId,
            description: nftData.metadata.name,
            value: nftData.lastSalePrice,
            blockchain_ref: {
                contract: nftData.contract,
                tokenId: nftData.tokenId,
                network: 'ethereum'
            }
        });
    }
}
```

### CRM Integration

```javascript
// Salesforce integration
const createNFTOwnerRecord = async (ownerAddress, nfts) => {
    await sfClient.sobject('NFT_Owner__c').create({
        Wallet_Address__c: ownerAddress,
        NFT_Count__c: nfts.length,
        Total_Value__c: calculateTotalValue(nfts),
        Collection_Names__c: getCollectionNames(nfts)
    });
};
```

### Supply Chain

```solidity
// Supply chain tracking with NFTs
contract SupplyChainNFT is ERC721 {
    struct ProductJourney {
        string origin;
        uint256 manufactureDate;
        string[] checkpoints;
        mapping(string => uint256) timestamps;
    }

    mapping(uint256 => ProductJourney) public journeys;

    function addCheckpoint(uint256 tokenId, string memory checkpoint)
        external
    {
        journeys[tokenId].checkpoints.push(checkpoint);
        journeys[tokenId].timestamps[checkpoint] = block.timestamp;
    }
}
```

## Analytics Integration

### Google Analytics

```javascript
// Track NFT events
gtag('event', 'nft_mint', {
    'contract_address': contractAddress,
    'token_id': tokenId,
    'price': mintPrice,
    'currency': 'ETH'
});

gtag('event', 'nft_sale', {
    'contract_address': contractAddress,
    'token_id': tokenId,
    'sale_price': salePrice,
    'marketplace': 'OpenSea'
});
```

### Dune Analytics

```sql
-- Custom SQL query for NFT analytics
SELECT
    date_trunc('day', evt_block_time) as day,
    COUNT(*) as daily_mints,
    SUM(value) as total_volume
FROM nft_mints
WHERE contract_address = '0x...'
GROUP BY day
ORDER BY day DESC;
```

## Payment Integration

### Stripe

```javascript
import Stripe from 'stripe';

const stripe = new Stripe(process.env.STRIPE_SECRET_KEY);

// Create payment intent for NFT purchase
const paymentIntent = await stripe.paymentIntents.create({
    amount: nftPrice * 100, // Convert to cents
    currency: 'usd',
    metadata: {
        contractAddress: '0x...',
        tokenId: '1',
        buyer: buyerEmail
    }
});
```

### Crypto Payment Processors

```javascript
// Coinbase Commerce integration
import { Client } from 'coinbase-commerce-node';

const client = Client.init(process.env.COINBASE_API_KEY);

const charge = await client.charges.create({
    name: 'NFT Purchase',
    description: `Token #${tokenId}`,
    pricing_type: 'fixed_price',
    local_price: {
        amount: '1.5',
        currency: 'ETH'
    },
    metadata: {
        tokenId,
        contractAddress
    }
});
```

## Email & Notification Integration

### SendGrid

```javascript
const sendMintConfirmation = async (email, nftData) => {
    await sgMail.send({
        to: email,
        from: 'noreply@mynft.com',
        subject: 'NFT Minted Successfully!',
        html: `
            <h1>Congratulations!</h1>
            <p>You've minted NFT #${nftData.tokenId}</p>
            <img src="${nftData.image}" alt="Your NFT">
            <a href="https://opensea.io/assets/${contractAddress}/${tokenId}">
                View on OpenSea
            </a>
        `
    });
};
```

### Push Notifications

```javascript
// OneSignal integration
const notifyNFTSale = async (userId, saleData) => {
    await OneSignal.createNotification({
        contents: {
            en: `Your NFT sold for ${saleData.price} ETH!`
        },
        include_player_ids: [userId],
        data: {
            type: 'nft_sale',
            tokenId: saleData.tokenId,
            price: saleData.price
        }
    });
};
```

## IPFS Pinning Services

### Pinata

```javascript
import pinataSDK from '@pinata/sdk';

const pinata = pinataSDK(apiKey, apiSecret);

// Upload to IPFS
const result = await pinata.pinJSONToIPFS({
    name: "NFT Metadata",
    description: "...",
    image: "ipfs://..."
});

console.log(`ipfs://${result.IpfsHash}`);
```

### NFT.Storage

```javascript
import { NFTStorage, File } from 'nft.storage';

const client = new NFTStorage({ token: apiKey });

// Store NFT data
const metadata = await client.store({
    name: 'My NFT',
    description: '...',
    image: new File([imageData], 'image.png', { type: 'image/png' })
});
```

## Testing & Monitoring

### E2E Testing

```javascript
describe('Full NFT Flow', () => {
    it('should complete mint to sale journey', async () => {
        // Mint
        const mintTx = await nft.mint(user.address, metadata);
        await mintTx.wait();

        // List on marketplace
        await marketplace.list(nft.address, tokenId, price);

        // Purchase
        await marketplace.buy(listingId, { value: price });

        // Verify transfer
        expect(await nft.ownerOf(tokenId)).to.equal(buyer.address);
    });
});
```

### Monitoring

```javascript
// Datadog integration
const dd = require('dd-trace').init();

dd.trace('nft.mint', async (span) => {
    span.setTag('contract', contractAddress);
    span.setTag('tokenId', tokenId);

    const result = await mintNFT();

    span.setTag('gasUsed', result.gasUsed);
    return result;
});
```

## Best Practices

1. **Error Handling**: Implement comprehensive error handling
2. **Rate Limiting**: Respect API rate limits
3. **Caching**: Cache NFT metadata appropriately
4. **Webhooks**: Use webhooks over polling
5. **Security**: Validate all external data
6. **Testing**: Test integrations in sandbox environments
7. **Documentation**: Maintain integration documentation
8. **Monitoring**: Set up alerts for integration failures

---

**Status**: Draft
**Version**: 1.0.0
**Last Updated**: 2025-01-15

© 2025 SmileStory Inc. / WIA
