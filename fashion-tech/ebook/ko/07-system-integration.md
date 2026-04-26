# 07장: 시스템 통합

## 학습 목표

이 장을 마치면 다음을 이해하게 됩니다:
- 전자상거래 플랫폼 통합 전략
- AR/VR 플랫폼 호환성
- 블록체인 및 NFT 마켓플레이스 통합
- 결제 및 재고 시스템 연결
- 서드파티 도구 통합 (Shopify, WooCommerce 등)

---

## 7.1 전자상거래 플랫폼 통합

### 7.1.1 Shopify 통합

```typescript
// Shopify 앱 구조
interface ShopifyWIAFashionApp {
  // 제품 동기화
  syncProducts(): Promise<void>;

  // 가상 피팅 위젯
  injectTryOnWidget(productId: string): void;

  // 사이즈 추천
  addSizeRecommendation(productId: string): void;

  // 지속가능성 배지
  displaySustainabilityScore(productId: string): void;
}

// 예제: Shopify 제품 메타필드 통합
async function addWIAMetafieldToProduct(
  productId: string,
  garmentData: WIAFashionGarment
): Promise<void> {
  const shopify = new Shopify({
    shopName: process.env.SHOP_NAME,
    accessToken: process.env.SHOPIFY_ACCESS_TOKEN
  });

  // WIA 데이터를 제품 메타필드로 저장
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: 'garment_data',
    value: JSON.stringify(garmentData),
    type: 'json',
    owner_id: productId,
    owner_resource: 'product'
  });

  // 3D 모델 메타필드 추가
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: '3d_model_url',
    value: garmentData.garment.assets_3d.models[0].url,
    type: 'url',
    owner_id: productId,
    owner_resource: 'product'
  });

  // 필터링을 위한 지속가능성 점수 추가
  await shopify.metafield.create({
    namespace: 'wia_fashion',
    key: 'sustainability_score',
    value: garmentData.garment.sustainability.totalScore.toString(),
    type: 'number_integer',
    owner_id: productId,
    owner_resource: 'product'
  });
}

// Shopify 테마 Liquid 템플릿
const shopifyProductTemplate = `
<!-- 제품 페이지: 가상 피팅 버튼 -->
{% if product.metafields.wia_fashion.garment_data %}
  <div id="wia-tryon-widget"
       data-garment-id="{{ product.metafields.wia_fashion.garment_data.garment.id }}">
    <button class="wia-tryon-button">
      가상으로 입어보기
    </button>
  </div>

  <!-- 지속가능성 점수 배지 -->
  <div class="wia-sustainability-badge">
    <span class="score">
      {{ product.metafields.wia_fashion.sustainability_score }}
    </span>
    <span class="label">지속가능성 점수</span>
  </div>

  <!-- 사이즈 추천 도구 -->
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

// JavaScript 위젯 초기화
class WIAShopifyWidget {
  constructor(config: { apiKey: string; garmentId: string }) {
    this.apiKey = config.apiKey;
    this.garmentId = config.garmentId;
    this.init();
  }

  private init(): void {
    // 피팅 버튼 핸들러 주입
    document.querySelector('.wia-tryon-button')
      ?.addEventListener('click', () => this.openTryOn());

    // 사이즈 추천 초기화
    this.initSizeRecommendation();

    // 3D 모델 프리뷰 로드
    this.load3DPreview();
  }

  private async openTryOn(): Promise<void> {
    // 가상 피팅 모달 열기
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

    // 닫기 버튼 핸들러
    modal.querySelector('.close')?.addEventListener('click', () => {
      modal.remove();
    });
  }

  private async initSizeRecommendation(): Promise<void> {
    const container = document.getElementById('wia-size-recommend');
    if (!container) return;

    // 사이즈 추천 폼 렌더링
    container.innerHTML = `
      <div class="wia-size-form">
        <h3>완벽한 사이즈 찾기</h3>
        <input type="number" placeholder="키 (cm)" id="wia-height" />
        <input type="number" placeholder="가슴둘레 (cm)" id="wia-chest" />
        <input type="number" placeholder="허리둘레 (cm)" id="wia-waist" />
        <input type="number" placeholder="엉덩이둘레 (cm)" id="wia-hips" />
        <button id="wia-recommend-btn">사이즈 추천받기</button>
        <div id="wia-recommendation-result"></div>
      </div>
    `;

    // 버튼 핸들러
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
        <h4>추천 사이즈: <strong>${rec.size}</strong></h4>
        <p class="confidence">신뢰도: ${(rec.confidence * 100).toFixed(0)}%</p>
        <p class="fit">${rec.fitPrediction.overall}</p>
        <p class="reasoning">${rec.reasoning}</p>
      </div>
    `;

    // 사이즈 변형 자동 선택
    const sizeSelector = document.querySelector(`input[value="${rec.size}"]`) as HTMLInputElement;
    if (sizeSelector) {
      sizeSelector.checked = true;
      sizeSelector.dispatchEvent(new Event('change'));
    }
  }
}
```

### 7.1.2 WooCommerce 통합

```php
<?php
/**
 * WIA 패션 WordPress 플러그인
 */

// WIA 데이터를 위한 커스텀 포스트 메타 등록
add_action('init', 'wia_register_product_meta');
function wia_register_product_meta() {
    register_post_meta('product', 'wia_garment_data', array(
        'type' => 'string',
        'description' => 'WIA 패션 의류 데이터 (JSON)',
        'single' => true,
        'show_in_rest' => true,
    ));

    register_post_meta('product', 'wia_sustainability_score', array(
        'type' => 'number',
        'description' => '지속가능성 점수 (0-100)',
        'single' => true,
        'show_in_rest' => true,
    ));
}

// 제품 편집 페이지에 WIA 메타박스 추가
add_action('add_meta_boxes', 'wia_add_product_metabox');
function wia_add_product_metabox() {
    add_meta_box(
        'wia_fashion_data',
        'WIA 패션 기술',
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
        <h4>WIA 의류 ID</h4>
        <input type="text" name="wia_garment_id"
               value="<?php echo esc_attr($garment_data['id'] ?? ''); ?>"
               placeholder="WIA-DRESS-2026-12345"
               style="width: 100%;" />

        <h4>지속가능성 점수</h4>
        <input type="number" name="wia_sustainability_score"
               value="<?php echo esc_attr($sustainability_score); ?>"
               min="0" max="100" />

        <h4>WIA API에서 동기화</h4>
        <button type="button" class="button" id="wia-sync-btn">
            의류 데이터 동기화
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
                        '<p style="color: green;">동기화 완료!</p>'
                    );
                    location.reload();
                }
            }
        });
    });
    </script>
    <?php
}

// 의류 데이터 동기화를 위한 AJAX 핸들러
add_action('wp_ajax_wia_sync_garment', 'wia_sync_garment_ajax');
function wia_sync_garment_ajax() {
    $garment_id = $_POST['garment_id'];
    $product_id = $_POST['product_id'];

    // WIA API 호출
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

    // 제품 메타 업데이트
    update_post_meta($product_id, 'wia_garment_data', json_encode($garment_data));
    update_post_meta($product_id, 'wia_sustainability_score',
        $garment_data['garment']['sustainability']['totalScore']);

    wp_send_json_success();
}

// 제품 페이지에 가상 피팅 버튼 추가
add_action('woocommerce_after_add_to_cart_button', 'wia_add_tryon_button');
function wia_add_tryon_button() {
    global $post;
    $garment_data = get_post_meta($post->ID, 'wia_garment_data', true);

    if (empty($garment_data)) return;

    ?>
    <button type="button" class="wia-tryon-button"
            data-garment-id="<?php echo esc_attr($garment_data['garment']['id']); ?>">
        🥽 가상으로 입어보기
    </button>
    <?php
}
?>
```

---

## 7.2 AR/VR 플랫폼 통합

### 7.2.1 Decentraland 통합

```typescript
// Decentraland 웨어러블 통합
import { wearable } from '@dcl/schemas';

interface DecentralandWearable {
  id: string;
  name: string;
  description: string;
  collectionAddress: string;
  rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary' | 'mythic' | 'unique';
  category: 'eyebrows' | 'eyes' | 'facial_hair' | 'hair' | 'mouth' | 'upper_body' | 'lower_body' | 'feet' | 'earring' | 'eyewear' | 'hat' | 'helmet' | 'mask' | 'tiara' | 'top_head' | 'skin' | 'hands_wear';

  // 3D 모델
  data: {
    representations: Array<{
      bodyShapes: string[];  // ['urn:decentraland:off-chain:base-avatars:BaseMale']
      mainFile: string;      // 'male/dress.glb'
      contents: string[];    // ['male/dress.glb', 'male/dress_texture.png']
      overrideHides: string[];
      overrideReplaces: string[];
    }>;
  };

  // 메타데이터
  thumbnail: string;
  image: string;
}

// WIA 의류를 Decentraland 웨어러블로 변환
async function convertToDecentralandWearable(
  wiaGarment: WIAFashionGarment
): Promise<DecentralandWearable> {
  // Decentraland용 3D 모델 최적화 (최대 5000 삼각형)
  const optimizedModel = await optimizeModelForDecentraland(
    wiaGarment.garment.assets_3d.models.find(m => m.lod === 'medium')?.url
  );

  return {
    id: `wia-${wiaGarment.garment.id}`,
    name: wiaGarment.garment.name,
    description: wiaGarment.garment.description,
    collectionAddress: '0x...', // 컬렉션 스마트 컨트랙트 주소
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

// Decentraland Builder에 업로드
async function uploadToDecentralandBuilder(
  wearable: DecentralandWearable
): Promise<string> {
  const builderAPI = 'https://builder-api.decentraland.org';

  // 3D 모델과 텍스처를 IPFS에 업로드
  const uploadedFiles = await uploadFilesToIPFS(wearable.data.representations[0].contents);

  // Builder에 웨어러블 생성
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

### 7.2.2 The Sandbox 통합

```typescript
// The Sandbox 복셀 에셋
interface SandboxAsset {
  name: string;
  description: string;
  type: 'WEARABLE';
  category: 'CLOTHING' | 'ACCESSORY';

  // 복셀 모델 (.vox 형식)
  voxelModel: {
    file: string;        // .vox 파일 URL
    size: number;        // 복셀 개수
    palette: string[];   // 색상 팔레트
  };

  // 메타데이터
  rarity: number;        // 1-5
  tags: string[];
}

// WIA 의류를 Sandbox 복셀 에셋으로 변환
async function convertToSandboxVoxel(
  wiaGarment: WIAFashionGarment
): Promise<SandboxAsset> {
  // MagicaVoxel 등을 사용하여 3D 메시를 복셀로 변환
  const voxelFile = await meshToVoxel(
    wiaGarment.garment.assets_3d.models[2].url, // 로우폴리 모델 사용
    {
      voxelSize: 32,  // 32x32x32 해상도
      maxVoxels: 3000
    }
  );

  // 소재에서 색상 팔레트 추출
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

## 7.3 블록체인 통합

### 7.3.1 NFT 스마트 컨트랙트

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/token/ERC721/extensions/ERC721URIStorage.sol";
import "@openzeppelin/contracts/access/Ownable.sol";

/**
 * @title WIAFashionNFT
 * @dev WIA 패션 아이템용 ERC-721 NFT 컨트랙트
 */
contract WIAFashionNFT is ERC721URIStorage, Ownable {
    uint256 private _tokenIdCounter;

    // 토큰 ID에서 WIA 의류 ID로 매핑
    mapping(uint256 => string) public wiaGarmentIds;

    // 토큰 ID에서 지속가능성 점수로 매핑
    mapping(uint256 => uint8) public sustainabilityScores;

    // 토큰 ID에서 실물 교환 상태로 매핑
    mapping(uint256 => bool) public physicalRedeemed;

    // 이벤트
    event FashionNFTMinted(uint256 indexed tokenId, string wiaGarmentId, address owner);
    event PhysicalRedeemed(uint256 indexed tokenId, address redeemer);

    constructor() ERC721("WIA Fashion", "WIAF") {}

    /**
     * @dev 새 패션 NFT 발행
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
     * @dev 실물 버전 교환 (피지털 NFT의 경우 1회만 가능)
     */
    function redeemPhysical(uint256 tokenId) public {
        require(ownerOf(tokenId) == msg.sender, "토큰 소유자가 아닙니다");
        require(!physicalRedeemed[tokenId], "이미 교환되었습니다");

        physicalRedeemed[tokenId] = true;

        emit PhysicalRedeemed(tokenId, msg.sender);

        // 오프체인 시스템이 실물 주문을 처리합니다
    }

    /**
     * @dev 패션 메타데이터 가져오기
     */
    function getFashionMetadata(uint256 tokenId) public view returns (
        string memory wiaGarmentId,
        uint8 sustainabilityScore,
        bool isPhysicalRedeemed
    ) {
        require(_exists(tokenId), "토큰이 존재하지 않습니다");

        return (
            wiaGarmentIds[tokenId],
            sustainabilityScores[tokenId],
            physicalRedeemed[tokenId]
        );
    }
}
```

### 7.3.2 WIA 데이터로 NFT 발행

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
    // 메타데이터를 IPFS에 업로드
    const metadataURI = await this.uploadMetadataToIPFS(wiaGarment);

    // NFT 발행
    const tx = await this.contract.mintFashionNFT(
      recipientAddress,
      wiaGarment.garment.id,
      metadataURI,
      wiaGarment.garment.sustainability.totalScore
    );

    // 확인 대기
    const receipt = await tx.wait();

    // 이벤트에서 토큰 ID 가져오기
    const event = receipt.events?.find((e: any) => e.event === 'FashionNFTMinted');
    const tokenId = event?.args?.tokenId.toString();

    console.log(`${wiaGarment.garment.name}에 대한 NFT #${tokenId} 발행`);

    return tokenId;
  }

  private async uploadMetadataToIPFS(
    wiaGarment: WIAFashionGarment
  ): Promise<string> {
    // WIA 표준을 따르는 NFT 메타데이터 생성
    const nftMetadata: WIAFashionNFT = {
      name: wiaGarment.garment.name,
      description: wiaGarment.garment.description,
      image: wiaGarment.garment.assets_2d.images[0].url,
      animation_url: wiaGarment.garment.assets_3d.models[0].url,

      attributes: [
        { trait_type: '브랜드', value: wiaGarment.garment.brand },
        { trait_type: '카테고리', value: wiaGarment.garment.category },
        { trait_type: '지속가능성 점수',
          value: wiaGarment.garment.sustainability.totalScore,
          max_value: 100 },
        { trait_type: '탄소 발자국',
          value: `${wiaGarment.garment.sustainability.carbonFootprint.total} kg CO₂e` }
      ],

      wia_fashion: {
        standard: 'WIA-IND-001',
        version: '1.0.0',
        type: wiaGarment.garment.blockchain?.nftEnabled ? 'phygital' : 'digital_only',
        models_3d: {
          web: wiaGarment.garment.assets_3d.models[0].url,
          // ... 다른 플랫폼 모델
        },
        garmentData: '', // IPFS 해시로 채워질 것임
        sustainability: {
          digitalOnly: !wiaGarment.garment.blockchain?.nftEnabled,
          carbonOffset: true,
          sustainableDesign: wiaGarment.garment.sustainability.totalScore > 70,
          score: wiaGarment.garment.sustainability.totalScore
        }
      }
    };

    // 전체 의류 데이터를 IPFS에 업로드
    const garmentDataHash = await this.uploadToIPFS(wiaGarment);
    nftMetadata.wia_fashion.garmentData = `ipfs://${garmentDataHash}`;

    // 메타데이터를 IPFS에 업로드
    const metadataHash = await this.uploadToIPFS(nftMetadata);

    return `ipfs://${metadataHash}`;
  }

  private async uploadToIPFS(data: any): Promise<string> {
    // Pinata 또는 유사한 IPFS 서비스 사용
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

// 사용 예제
const minter = new WIAFashionNFTMinter(
  '0x742d35Cc6634C0532925a3b8',  // 컨트랙트 주소
  process.env.PRIVATE_KEY!
);

const tokenId = await minter.mintFromWIAGarment(
  '0xRecipientAddress',
  dressGarmentData
);
```

---

## 7.4 결제 시스템 통합

### 7.4.1 암호화폐 결제

```typescript
// Web3 결제 통합
import { ethers } from 'ethers';

class CryptoPaymentProcessor {
  private provider: ethers.providers.Web3Provider;

  constructor() {
    // MetaMask 등의 지갑 연결
    this.provider = new ethers.providers.Web3Provider(window.ethereum);
  }

  async processPayment(
    amount: number,
    currency: 'ETH' | 'USDC',
    merchantAddress: string
  ): Promise<string> {
    const signer = this.provider.getSigner();

    if (currency === 'ETH') {
      // ETH 직접 전송
      const tx = await signer.sendTransaction({
        to: merchantAddress,
        value: ethers.utils.parseEther(amount.toString())
      });

      await tx.wait();
      return tx.hash;

    } else if (currency === 'USDC') {
      // ERC-20 토큰 전송
      const usdcContract = new ethers.Contract(
        USDC_CONTRACT_ADDRESS,
        ERC20_ABI,
        signer
      );

      const tx = await usdcContract.transfer(
        merchantAddress,
        ethers.utils.parseUnits(amount.toString(), 6) // USDC는 6 decimals
      );

      await tx.wait();
      return tx.hash;
    }

    throw new Error('지원하지 않는 통화입니다');
  }

  async connectWallet(): Promise<string> {
    await this.provider.send('eth_requestAccounts', []);
    const signer = this.provider.getSigner();
    return await signer.getAddress();
  }
}

// 사용 예제
const paymentProcessor = new CryptoPaymentProcessor();
const walletAddress = await paymentProcessor.connectWallet();

const txHash = await paymentProcessor.processPayment(
  0.05,  // 0.05 ETH
  'ETH',
  '0xMerchantAddress'
);

console.log(`결제 완료: ${txHash}`);
```

### 7.4.2 전통적인 결제 게이트웨이

```typescript
// Stripe 통합
import Stripe from 'stripe';

const stripe = new Stripe(process.env.STRIPE_SECRET_KEY!, {
  apiVersion: '2023-10-16'
});

// WIA 의류 구매를 위한 결제 인텐트 생성
async function createPaymentIntent(
  garmentId: string,
  amount: number,
  currency: string = 'usd'
): Promise<string> {
  const paymentIntent = await stripe.paymentIntents.create({
    amount: amount * 100, // 센트 단위
    currency,
    metadata: {
      garment_id: garmentId,
      platform: 'WIA Fashion Tech'
    },
    automatic_payment_methods: {
      enabled: true
    }
  });

  return paymentIntent.client_secret!;
}

// 프론트엔드에서 결제 확인
async function confirmPayment(clientSecret: string) {
  const { error, paymentIntent } = await stripe.confirmPayment({
    clientSecret,
    confirmParams: {
      return_url: 'https://yoursite.com/order-confirmation'
    }
  });

  if (error) {
    console.error('결제 실패:', error.message);
  } else if (paymentIntent.status === 'succeeded') {
    console.log('결제 성공!');
  }
}
```

---

## 7.5 재고 관리 시스템 통합

### 7.5.1 실시간 재고 동기화

```typescript
// 재고 관리 시스템 통합
interface InventorySystem {
  getStock(garmentId: string, size: string): Promise<number>;
  reserveStock(garmentId: string, size: string, quantity: number): Promise<boolean>;
  releaseStock(garmentId: string, size: string, quantity: number): Promise<void>;
  updateStock(garmentId: string, size: string, quantity: number): Promise<void>;
}

class WIAInventoryManager implements InventorySystem {
  private apiUrl = 'https://api.wiastandards.com/inventory/v1';

  async getStock(garmentId: string, size: string): Promise<number> {
    const response = await fetch(
      `${this.apiUrl}/stock/${garmentId}/${size}`,
      {
        headers: {
          'Authorization': `Bearer ${process.env.WIA_API_KEY}`
        }
      }
    );

    const data = await response.json();
    return data.quantity;
  }

  async reserveStock(
    garmentId: string,
    size: string,
    quantity: number
  ): Promise<boolean> {
    const response = await fetch(
      `${this.apiUrl}/reserve`,
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${process.env.WIA_API_KEY}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          garmentId,
          size,
          quantity,
          ttl: 600 // 10분 동안 예약
        })
      }
    );

    const data = await response.json();
    return data.success;
  }

  async releaseStock(
    garmentId: string,
    size: string,
    quantity: number
  ): Promise<void> {
    await fetch(
      `${this.apiUrl}/release`,
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${process.env.WIA_API_KEY}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ garmentId, size, quantity })
      }
    );
  }

  async updateStock(
    garmentId: string,
    size: string,
    quantity: number
  ): Promise<void> {
    await fetch(
      `${this.apiUrl}/stock/${garmentId}/${size}`,
      {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${process.env.WIA_API_KEY}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ quantity })
      }
    );
  }
}

// 구매 워크플로우에서 사용
const inventory = new WIAInventoryManager();

// 1. 재고 확인
const stock = await inventory.getStock('WIA-DRESS-2026-12345', 'M');
if (stock > 0) {
  // 2. 재고 예약 (장바구니 추가 시)
  await inventory.reserveStock('WIA-DRESS-2026-12345', 'M', 1);

  // 3. 결제 성공 시 재고 업데이트
  await inventory.updateStock('WIA-DRESS-2026-12345', 'M', stock - 1);
} else {
  console.log('재고 없음');
}
```

---

## 7.6 분석 및 모니터링

### 7.6.1 사용자 행동 추적

```typescript
// Google Analytics 4 통합
import { gtag } from 'ga-gtag';

class FashionTechAnalytics {
  trackVirtualTryOn(garmentId: string, duration: number): void {
    gtag('event', 'virtual_tryon', {
      garment_id: garmentId,
      session_duration: duration,
      event_category: 'engagement'
    });
  }

  trackSizeRecommendation(
    garmentId: string,
    recommendedSize: string,
    selectedSize: string
  ): void {
    gtag('event', 'size_recommendation', {
      garment_id: garmentId,
      recommended: recommendedSize,
      selected: selectedSize,
      match: recommendedSize === selectedSize,
      event_category: 'engagement'
    });
  }

  trackPurchase(
    garmentId: string,
    size: string,
    price: number,
    usedTryOn: boolean
  ): void {
    gtag('event', 'purchase', {
      garment_id: garmentId,
      size,
      value: price,
      currency: 'USD',
      used_virtual_tryon: usedTryOn,
      event_category: 'ecommerce'
    });
  }

  trackSustainabilityView(garmentId: string, score: number): void {
    gtag('event', 'view_sustainability', {
      garment_id: garmentId,
      sustainability_score: score,
      event_category: 'engagement'
    });
  }
}

// 사용 예제
const analytics = new FashionTechAnalytics();

// 가상 피팅 세션 추적
analytics.trackVirtualTryOn('WIA-DRESS-2026-12345', 120); // 120초

// 사이즈 추천 추적
analytics.trackSizeRecommendation('WIA-DRESS-2026-12345', 'M', 'M');

// 구매 추적
analytics.trackPurchase('WIA-DRESS-2026-12345', 'M', 89.99, true);
```

### 7.6.2 성능 모니터링

```typescript
// New Relic 또는 유사한 APM 도구와 통합
class PerformanceMonitor {
  trackModelLoadTime(garmentId: string, loadTime: number): void {
    // 커스텀 메트릭 전송
    newrelic.recordMetric('3DModel/LoadTime', loadTime);
    newrelic.addCustomAttribute('garment_id', garmentId);
  }

  trackAPILatency(endpoint: string, latency: number): void {
    newrelic.recordMetric(`API/${endpoint}/Latency`, latency);
  }

  trackError(error: Error, context: any): void {
    newrelic.noticeError(error, context);
  }
}

// 사용 예제
const perfMonitor = new PerformanceMonitor();

const startTime = performance.now();
await loadGarmentModel('WIA-DRESS-2026-12345');
const loadTime = performance.now() - startTime;

perfMonitor.trackModelLoadTime('WIA-DRESS-2026-12345', loadTime);
```

---

## 복습 질문

1. **통합 예제에서 다루는 두 가지 전자상거래 플랫폼은 무엇입니까?**
   <details>
   <summary>답변</summary>
   Shopify와 WooCommerce (WordPress).
   </details>

2. **Decentraland 웨어러블의 최대 폴리곤 수는 얼마입니까?**
   <details>
   <summary>답변</summary>
   최적 성능을 위해 5,000 삼각형 (폴리곤).
   </details>

3. **The Sandbox는 3D 에셋에 어떤 파일 형식을 사용합니까?**
   <details>
   <summary>답변</summary>
   MagicaVoxel 또는 유사한 도구의 .vox (복셀) 형식.
   </details>

4. **패션 NFT에 사용되는 ERC 표준은 무엇입니까?**
   <details>
   <summary>답변</summary>
   URI 저장 확장을 가진 ERC-721 (ERC721URIStorage).
   </details>

5. **WIA 표준에서 NFT 메타데이터는 어디에 저장됩니까?**
   <details>
   <summary>답변</summary>
   탈중앙화되고 영구적인 저장을 위해 IPFS (InterPlanetary File System)에 저장됩니다.
   </details>

6. **재고 예약의 기본 TTL (Time To Live)은 얼마입니까?**
   <details>
   <summary>답변</summary>
   600초 (10분) - 사용자가 결제를 완료할 충분한 시간을 제공합니다.
   </details>

7. **암호화폐 결제에서 USDC는 몇 개의 소수점을 사용합니까?**
   <details>
   <summary>답변</summary>
   6 decimals (ETH는 18 decimals를 사용하지만 USDC는 6 decimals).
   </details>

8. **Shopify에서 WIA 데이터를 저장하는 데 사용되는 메커니즘은 무엇입니까?**
   <details>
   <summary>답변</summary>
   메타필드 (Metafields) - 제품에 커스텀 데이터를 첨부할 수 있습니다.
   </details>

---

## 실습 과제

### 과제 1: Shopify 통합
Shopify 스토어에 WIA 패션 위젯을 통합하세요:
1. WIA API 키 획득
2. 제품에 WIA 메타필드 추가
3. 제품 페이지에 가상 피팅 버튼 추가
4. 사이즈 추천 위젯 구현

### 과제 2: NFT 발행
WIA 의류 데이터로 패션 NFT를 발행하세요:
1. 테스트넷에 스마트 컨트랙트 배포 (Goerli 또는 Mumbai)
2. IPFS에 의류 데이터 업로드
3. NFT 메타데이터 생성
4. NFT 발행 및 OpenSea에서 확인

### 과제 3: 결제 통합
Stripe를 사용하여 결제 플로우를 구현하세요:
1. Stripe 계정 설정
2. 결제 인텐트 API 통합
3. 웹훅으로 결제 확인 처리
4. 주문 이행 워크플로우 구현

---

## 다음 단계

[**08장: 구현 가이드**](08-implementation.md)에서 패션 테크 시스템 구축에 대한 실용적인 가이드로 학습 여정을 완료하세요.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
