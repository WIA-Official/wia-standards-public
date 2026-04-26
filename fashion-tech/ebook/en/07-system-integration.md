# Chapter 7: System Integration

## Learning Objectives

By the end of this chapter, you will understand:
- E-commerce platform integration strategies
- AR/VR platform compatibility
- Blockchain and NFT marketplace integration
- Payment and inventory system connections
- Third-party tool integration (Shopify, WooCommerce, etc.)

---

## 7.1 E-commerce Platform Integration

### 7.1.1 Shopify Integration

```typescript
// Shopify App Structure
interface ShopifyWIAFashionApp {
  // Product sync
  syncProducts(): Promise<void>;

  // Virtual try-on widget
  injectTryOnWidget(productId: string): void;

  // Size recommendation
  addSizeRecommendation(productId: string): void;

  // Sustainability badge
  displaySustainabilityScore(productId: string): void;
}

// Example: Shopify Product Metafield Integration
async function addWIAMetafieldToProduct(
  productId: string,
  garmentData: WIAFashionGarment
): Promise<void> {
  const shopify = new Shopify({
    shopName: process.env.SHOP_NAME,
    accessToken: process.env.SHOPIFY_ACCESS_TOKEN
  });

  // Store WIA data as product metafield
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: 'garment_data',
    value: JSON.stringify(garmentData),
    type: 'json',
    owner_id: productId,
    owner_resource: 'product'
  });

  // Add 3D model metafield
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: '3d_model_url',
    value: garmentData.garment.assets_3d.models[0].url,
    type: 'url',
    owner_id: productId,
    owner_resource: 'product'
  });

  // Add sustainability score for filtering
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: 'sustainability_score',
    value: garmentData.garment.sustainability.totalScore.toString(),
    type: 'number_integer',
    owner_id: productId,
    owner_resource: 'product'
  });
}

// Shopify Theme Liquid Template
const shopifyProductTemplate = `
<!-- Product Page: Virtual Try-On Button -->
{% if product.metafields.wia_fashion.garment_data %}
  <div id="wia-tryon-widget"
       data-garment-id="{{ product.metafields.wia_fashion.garment_data.garment.id }}">
    <button class="wia-tryon-button">
      Try On Virtually
    </button>
  </div>

  <!-- Sustainability Score Badge -->
  <div class="wia-sustainability-badge">
    <span class="score">
      {{ product.metafields.wia_fashion.sustainability_score }}
    </span>
    <span class="label">Sustainability Score</span>
  </div>

  <!-- Size Recommendation Tool -->
  <div id="wia-size-recommend"></div>

  <script src="https://cdn.wiastandards.com/widgets/fashion/v1/all.js"></script>
  <script>
    WIAFashion.init({
      apiKey: '{{ shop.metafields.wia.api_key }}',
      garmentId: '{{ product.metafields.wia_fashion.garment_data.garment.id }}'
    });
  </script>
{% endif %}
`;

// JavaScript Widget Initialization
class WIAShopifyWidget {
  constructor(config: { apiKey: string; garmentId: string }) {
    this.apiKey = config.apiKey;
    this.garmentId = config.garmentId;
    this.init();
  }

  private init(): void {
    // Inject try-on button handler
    document.querySelector('.wia-tryon-button')
      ?.addEventListener('click', () => this.openTryOn());

    // Initialize size recommendation
    this.initSizeRecommendation();

    // Load 3D model preview
    this.load3DPreview();
  }

  private async openTryOn(): Promise<void> {
    // Open virtual try-on modal
    const modal = document.createElement('div');
    modal.id = 'wia-tryon-modal';
    modal.innerHTML = `
      <div class="modal-content">
        <span class="close">&times;</span>
        <iframe src="https://tryon.wiastandards.com/session?garment=${this.garmentId}&key=${this.apiKey}"
                width="100%" height="600px"></iframe>
      </div>
    `;
    document.body.appendChild(modal);

    // Close button handler
    modal.querySelector('.close')?.addEventListener('click', () => {
      modal.remove();
    });
  }

  private async initSizeRecommendation(): Promise<void> {
    const container = document.getElementById('wia-size-recommend');
    if (!container) return;

    // Render size recommendation form
    container.innerHTML = `
      <div class="wia-size-form">
        <h3>Get Your Perfect Size</h3>
        <input type="number" placeholder="Height (cm)" id="wia-height" />
        <input type="number" placeholder="Chest (cm)" id="wia-chest" />
        <input type="number" placeholder="Waist (cm)" id="wia-waist" />
        <input type="number" placeholder="Hips (cm)" id="wia-hips" />
        <button id="wia-recommend-btn">Recommend Size</button>
        <div id="wia-recommendation-result"></div>
      </div>
    `;

    // Button handler
    document.getElementById('wia-recommend-btn')
      ?.addEventListener('click', async () => {
        const measurements = {
          height: parseFloat((document.getElementById('wia-height') as HTMLInputElement).value),
          chest: parseFloat((document.getElementById('wia-chest') as HTMLInputElement).value),
          waist: parseFloat((document.getElementById('wia-waist') as HTMLInputElement).value),
          hips: parseFloat((document.getElementById('wia-hips') as HTMLInputElement).value)
        };

        const recommendation = await this.getSizeRecommendation(measurements);
        this.displayRecommendation(recommendation);
      });
  }

  private async getSizeRecommendation(
    measurements: BodyMeasurements
  ): Promise<SizeRecommendation> {
    const response = await fetch(
      'https://api.wiastandards.com/fashion/v1/size-recommend',
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          garmentId: this.garmentId,
          userMeasurements: measurements
        })
      }
    );

    const data = await response.json();
    return data.recommendation;
  }

  private displayRecommendation(rec: SizeRecommendation): void {
    const resultDiv = document.getElementById('wia-recommendation-result');
    if (!resultDiv) return;

    resultDiv.innerHTML = `
      <div class="recommendation">
        <h4>Recommended Size: <strong>${rec.size}</strong></h4>
        <p class="confidence">Confidence: ${(rec.confidence * 100).toFixed(0)}%</p>
        <p class="fit">${rec.fitPrediction.overall}</p>
        <p class="reasoning">${rec.reasoning}</p>
      </div>
    `;

    // Auto-select size variant
    const sizeSelector = document.querySelector(`input[value="${rec.size}"]`) as HTMLInputElement;
    if (sizeSelector) {
      sizeSelector.checked = true;
      sizeSelector.dispatchEvent(new Event('change'));
    }
  }
}
```

### 7.1.2 WooCommerce Integration

```php
<?php
/**
 * WIA Fashion WordPress Plugin
 */

// Register custom post meta for WIA data
add_action('init', 'wia_register_product_meta');
function wia_register_product_meta() {
    register_post_meta('product', 'wia_garment_data', array(
        'type' => 'string',
        'description' => 'WIA Fashion garment data (JSON)',
        'single' => true,
        'show_in_rest' => true,
    ));

    register_post_meta('product', 'wia_sustainability_score', array(
        'type' => 'number',
        'description' => 'Sustainability score (0-100)',
        'single' => true,
        'show_in_rest' => true,
    ));
}

// Add WIA metabox to product edit page
add_action('add_meta_boxes', 'wia_add_product_metabox');
function wia_add_product_metabox() {
    add_meta_box(
        'wia_fashion_data',
        'WIA Fashion Technology',
        'wia_product_metabox_html',
        'product',
        'normal',
        'high'
    );
}

function wia_product_metabox_html($post) {
    $garment_data = get_post_meta($post->ID, 'wia_garment_data', true);
    $sustainability_score = get_post_meta($post->ID, 'wia_sustainability_score', true);

    ?>
    <div class="wia-fashion-metabox">
        <h4>WIA Garment ID</h4>
        <input type="text" name="wia_garment_id"
               value="<?php echo esc_attr($garment_data['id'] ?? ''); ?>"
               placeholder="WIA-DRESS-2026-12345"
               style="width: 100%;" />

        <h4>Sustainability Score</h4>
        <input type="number" name="wia_sustainability_score"
               value="<?php echo esc_attr($sustainability_score); ?>"
               min="0" max="100" />

        <h4>Sync from WIA API</h4>
        <button type="button" class="button" id="wia-sync-btn">
            Sync Garment Data
        </button>

        <div id="wia-sync-result"></div>
    </div>

    <script>
    jQuery('#wia-sync-btn').click(function() {
        const garmentId = jQuery('input[name="wia_garment_id"]').val();

        jQuery.ajax({
            url: '<?php echo admin_url('admin-ajax.php'); ?>',
            method: 'POST',
            data: {
                action: 'wia_sync_garment',
                garment_id: garmentId,
                product_id: <?php echo $post->ID; ?>
            },
            success: function(response) {
                if (response.success) {
                    jQuery('#wia-sync-result').html(
                        '<p style="color: green;">Synced successfully!</p>'
                    );
                    location.reload();
                }
            }
        });
    });
    </script>
    <?php
}

// AJAX handler for syncing garment data
add_action('wp_ajax_wia_sync_garment', 'wia_sync_garment_ajax');
function wia_sync_garment_ajax() {
    $garment_id = $_POST['garment_id'];
    $product_id = $_POST['product_id'];

    // Call WIA API
    $api_key = get_option('wia_api_key');
    $response = wp_remote_get(
        "https://api.wiastandards.com/fashion/v1/garments/{$garment_id}",
        array(
            'headers' => array(
                'Authorization' => "Bearer {$api_key}"
            )
        )
    );

    if (is_wp_error($response)) {
        wp_send_json_error();
        return;
    }

    $garment_data = json_decode(wp_remote_retrieve_body($response), true);

    // Update product meta
    update_post_meta($product_id, 'wia_garment_data', json_encode($garment_data));
    update_post_meta($product_id, 'wia_sustainability_score',
        $garment_data['garment']['sustainability']['totalScore']);

    wp_send_json_success();
}

// Add virtual try-on button to product page
add_action('woocommerce_after_add_to_cart_button', 'wia_add_tryon_button');
function wia_add_tryon_button() {
    global $post;
    $garment_data = get_post_meta($post->ID, 'wia_garment_data', true);

    if (empty($garment_data)) return;

    ?>
    <button type="button" class="wia-tryon-button"
            data-garment-id="<?php echo esc_attr($garment_data['garment']['id']); ?>">
        🥽 Try On Virtually
    </button>
    <?php
}
?>
```

---

## 7.2 AR/VR Platform Integration

### 7.2.1 Decentraland Integration

```typescript
// Decentraland Wearable Integration
import { wearable } from '@dcl/schemas';

interface DecentralandWearable {
  id: string;
  name: string;
  description: string;
  collectionAddress: string;
  rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary' | 'mythic' | 'unique';
  category: 'eyebrows' | 'eyes' | 'facial_hair' | 'hair' | 'mouth' | 'upper_body' | 'lower_body' | 'feet' | 'earring' | 'eyewear' | 'hat' | 'helmet' | 'mask' | 'tiara' | 'top_head' | 'skin' | 'hands_wear';

  // 3D model
  data: {
    representations: Array<{
      bodyShapes: string[];  // ['urn:decentraland:off-chain:base-avatars:BaseMale']
      mainFile: string;      // 'male/dress.glb'
      contents: string[];    // ['male/dress.glb', 'male/dress_texture.png']
      overrideHides: string[];
      overrideReplaces: string[];
    }>;
  };

  // Metadata
  thumbnail: string;
  image: string;
}

// Convert WIA garment to Decentraland wearable
async function convertToDecentralandWearable(
  wiaGarment: WIAFashionGarment
): Promise<DecentralandWearable> {
  // Optimize 3D model for Decentraland (max 5000 triangles)
  const optimizedModel = await optimizeModelForDecentraland(
    wiaGarment.garment.assets_3d.models.find(m => m.lod === 'medium')?.url
  );

  return {
    id: `wia-${wiaGarment.garment.id}`,
    name: wiaGarment.garment.name,
    description: wiaGarment.garment.description,
    collectionAddress: '0x...', // Your collection smart contract
    rarity: mapWIARarityToDecentraland(wiaGarment.garment.rarity),
    category: mapWIACategoryToDecentraland(wiaGarment.garment.category),

    data: {
      representations: [
        {
          bodyShapes: ['urn:decentraland:off-chain:base-avatars:BaseMale',
                       'urn:decentraland:off-chain:base-avatars:BaseFemale'],
          mainFile: 'dress.glb',
          contents: ['dress.glb', 'texture.png'],
          overrideHides: ['upper_body', 'lower_body'],
          overrideReplaces: []
        }
      ]
    },

    thumbnail: wiaGarment.garment.assets_2d.images[0].url,
    image: wiaGarment.garment.assets_2d.images[0].url
  };
}

// Upload to Decentraland Builder
async function uploadToDecentralandBuilder(
  wearable: DecentralandWearable
): Promise<string> {
  const builderAPI = 'https://builder-api.decentraland.org';

  // Upload 3D model and textures
  const uploadedFiles = await uploadFilesToIPFS(wearable.data.representations[0].contents);

  // Create wearable in Builder
  const response = await fetch(`${builderAPI}/v1/items`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${DECENTRALAND_AUTH_TOKEN}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      ...wearable,
      data: {
        ...wearable.data,
        representations: wearable.data.representations.map(rep => ({
          ...rep,
          contents: uploadedFiles
        }))
      }
    })
  });

  const data = await response.json();
  return data.id;
}
```

### 7.2.2 The Sandbox Integration

```typescript
// The Sandbox Voxel Asset
interface SandboxAsset {
  name: string;
  description: string;
  type: 'WEARABLE';
  category: 'CLOTHING' | 'ACCESSORY';

  // Voxel model (.vox format)
  voxelModel: {
    file: string;        // .vox file URL
    size: number;        // Voxel count
    palette: string[];   // Color palette
  };

  // Metadata
  rarity: number;        // 1-5
  tags: string[];
}

// Convert WIA garment to Sandbox voxel asset
async function convertToSandboxVoxel(
  wiaGarment: WIAFashionGarment
): Promise<SandboxAsset> {
  // Convert 3D mesh to voxels using MagicaVoxel or similar
  const voxelFile = await meshToVoxel(
    wiaGarment.garment.assets_3d.models[2].url, // Use low-poly model
    {
      voxelSize: 32,  // 32x32x32 resolution
      maxVoxels: 3000
    }
  );

  // Extract color palette from materials
  const palette = extractColorPalette(wiaGarment.garment.colors);

  return {
    name: wiaGarment.garment.name,
    description: wiaGarment.garment.description,
    type: 'WEARABLE',
    category: wiaGarment.garment.type === 'dress' ? 'CLOTHING' : 'ACCESSORY',

    voxelModel: {
      file: voxelFile.url,
      size: voxelFile.voxelCount,
      palette
    },

    rarity: mapWIAScoreToRarity(wiaGarment.garment.sustainability.totalScore),
    tags: wiaGarment.garment.tags
  };
}
```

---

## 7.3 Blockchain Integration

### 7.3.1 NFT Smart Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/token/ERC721/extensions/ERC721URIStorage.sol";
import "@openzeppelin/contracts/access/Ownable.sol";

/**
 * @title WIAFashionNFT
 * @dev ERC-721 NFT contract for WIA Fashion items
 */
contract WIAFashionNFT is ERC721URIStorage, Ownable {
    uint256 private _tokenIdCounter;

    // Mapping from token ID to WIA garment ID
    mapping(uint256 => string) public wiaGarmentIds;

    // Mapping from token ID to sustainability score
    mapping(uint256 => uint8) public sustainabilityScores;

    // Mapping from token ID to physical redemption status
    mapping(uint256 => bool) public physicalRedeemed;

    // Events
    event FashionNFTMinted(uint256 indexed tokenId, string wiaGarmentId, address owner);
    event PhysicalRedeemed(uint256 indexed tokenId, address redeemer);

    constructor() ERC721("WIA Fashion", "WIAF") {}

    /**
     * @dev Mint new fashion NFT
     */
    function mintFashionNFT(
        address to,
        string memory wiaGarmentId,
        string memory tokenURI,
        uint8 sustainabilityScore
    ) public onlyOwner returns (uint256) {
        uint256 tokenId = _tokenIdCounter;
        _tokenIdCounter++;

        _safeMint(to, tokenId);
        _setTokenURI(tokenId, tokenURI);

        wiaGarmentIds[tokenId] = wiaGarmentId;
        sustainabilityScores[tokenId] = sustainabilityScore;
        physicalRedeemed[tokenId] = false;

        emit FashionNFTMinted(tokenId, wiaGarmentId, to);

        return tokenId;
    }

    /**
     * @dev Redeem physical version (one-time only for phygital NFTs)
     */
    function redeemPhysical(uint256 tokenId) public {
        require(ownerOf(tokenId) == msg.sender, "Not token owner");
        require(!physicalRedeemed[tokenId], "Already redeemed");

        physicalRedeemed[tokenId] = true;

        emit PhysicalRedeemed(tokenId, msg.sender);

        // Off-chain system will fulfill physical order
    }

    /**
     * @dev Get fashion metadata
     */
    function getFashionMetadata(uint256 tokenId) public view returns (
        string memory wiaGarmentId,
        uint8 sustainabilityScore,
        bool isPhysicalRedeemed
    ) {
        require(_exists(tokenId), "Token does not exist");

        return (
            wiaGarmentIds[tokenId],
            sustainabilityScores[tokenId],
            physicalRedeemed[tokenId]
        );
    }
}
```

### 7.3.2 Minting NFTs from WIA Data

```typescript
import { ethers } from 'ethers';

class WIAFashionNFTMinter {
  private contract: ethers.Contract;
  private provider: ethers.providers.Provider;
  private signer: ethers.Signer;

  constructor(
    contractAddress: string,
    privateKey: string
  ) {
    this.provider = new ethers.providers.JsonRpcProvider(process.env.RPC_URL);
    this.signer = new ethers.Wallet(privateKey, this.provider);
    this.contract = new ethers.Contract(
      contractAddress,
      WIA_FASHION_NFT_ABI,
      this.signer
    );
  }

  async mintFromWIAGarment(
    recipientAddress: string,
    wiaGarment: WIAFashionGarment
  ): Promise<string> {
    // Upload metadata to IPFS
    const metadataURI = await this.uploadMetadataToIPFS(wiaGarment);

    // Mint NFT
    const tx = await this.contract.mintFashionNFT(
      recipientAddress,
      wiaGarment.garment.id,
      metadataURI,
      wiaGarment.garment.sustainability.totalScore
    );

    // Wait for confirmation
    const receipt = await tx.wait();

    // Get token ID from event
    const event = receipt.events?.find((e: any) => e.event === 'FashionNFTMinted');
    const tokenId = event?.args?.tokenId.toString();

    console.log(`Minted NFT #${tokenId} for ${wiaGarment.garment.name}`);

    return tokenId;
  }

  private async uploadMetadataToIPFS(
    wiaGarment: WIAFashionGarment
  ): Promise<string> {
    // Create NFT metadata following WIA standard
    const nftMetadata: WIAFashionNFT = {
      name: wiaGarment.garment.name,
      description: wiaGarment.garment.description,
      image: wiaGarment.garment.assets_2d.images[0].url,
      animation_url: wiaGarment.garment.assets_3d.models[0].url,

      attributes: [
        { trait_type: 'Brand', value: wiaGarment.garment.brand },
        { trait_type: 'Category', value: wiaGarment.garment.category },
        { trait_type: 'Sustainability Score',
          value: wiaGarment.garment.sustainability.totalScore,
          max_value: 100 },
        { trait_type: 'Carbon Footprint',
          value: `${wiaGarment.garment.sustainability.carbonFootprint.total} kg CO₂e` }
      ],

      wia_fashion: {
        standard: 'WIA-IND-001',
        version: '1.0.0',
        type: wiaGarment.garment.blockchain?.nftEnabled ? 'phygital' : 'digital_only',
        models_3d: {
          web: wiaGarment.garment.assets_3d.models[0].url,
          // ... other platform models
        },
        garmentData: '', // Will be filled with IPFS hash
        sustainability: {
          digitalOnly: !wiaGarment.garment.blockchain?.nftEnabled,
          carbonOffset: true,
          sustainableDesign: wiaGarment.garment.sustainability.totalScore > 70,
          score: wiaGarment.garment.sustainability.totalScore
        }
      }
    };

    // Upload full garment data to IPFS
    const garmentDataHash = await this.uploadToIPFS(wiaGarment);
    nftMetadata.wia_fashion.garmentData = `ipfs://${garmentDataHash}`;

    // Upload metadata to IPFS
    const metadataHash = await this.uploadToIPFS(nftMetadata);

    return `ipfs://${metadataHash}`;
  }

  private async uploadToIPFS(data: any): Promise<string> {
    // Using Pinata or similar IPFS service
    const response = await fetch('https://api.pinata.cloud/pinning/pinJSONToIPFS', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${process.env.PINATA_JWT}`
      },
      body: JSON.stringify(data)
    });

    const result = await response.json();
    return result.IpfsHash;
  }
}

// Usage
const minter = new WIAFashionNFTMinter(
  '0x742d35Cc6634C0532925a3b8',  // Contract address
  process.env.PRIVATE_KEY!
);

const tokenId = await minter.mintFromWIAGarment(
  '0xRecipientAddress',
  dressGarmentData
);
```

---

## Review Questions

1. **Which two e-commerce platforms are covered in the integration examples?**
   <details>
   <summary>Answer</summary>
   Shopify and WooCommerce (WordPress).
   </details>

2. **What is the maximum polygon count for Decentraland wearables?**
   <details>
   <summary>Answer</summary>
   5,000 triangles (polygons) for optimal performance.
   </details>

3. **What file format does The Sandbox use for 3D assets?**
   <details>
   <summary>Answer</summary>
   .vox (voxel) format from MagicaVoxel or similar tools.
   </details>

4. **What is the ERC standard used for fashion NFTs?**
   <details>
   <summary>Answer</summary>
   ERC-721 with URI storage extension (ERC721URIStorage).
   </details>

5. **Where is NFT metadata stored in the WIA standard?**
   <details>
   <summary>Answer</summary>
   On IPFS (InterPlanetary File System) for decentralized, permanent storage.
   </details>

---

## Next Steps

Complete your learning journey with [**Chapter 8: Implementation**](08-implementation.md), which provides a practical guide to building fashion tech systems.

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
