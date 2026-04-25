# 🧴 WIA-IND-005: Cosmetics Data Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-005
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industry
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-IND-005 standard defines the comprehensive framework for cosmetics and personal care product data management, covering ingredient databases, safety assessments, efficacy tracking, regulatory compliance, and supply chain transparency. This standard provides a unified interface for product formulation, safety analysis, and consumer information systems.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to ensure transparency, safety, and efficacy in cosmetics products worldwide, protecting consumers and enabling informed choices while fostering innovation in the beauty and personal care industry.

## 🎯 Key Features

- **Ingredient Database**: Comprehensive INCI nomenclature and chemical composition
- **Safety Assessment**: Toxicological data, allergen information, and risk evaluation
- **Efficacy Tracking**: Clinical trial data, performance metrics, and claims substantiation
- **Regulatory Compliance**: Global regulatory frameworks (FDA, EU, K-Beauty, J-Beauty)
- **Supply Chain Transparency**: Ingredient sourcing, sustainability, and ethical tracking
- **Product Formulation**: Recipe management, stability testing, and batch tracking
- **Consumer Safety**: Allergen alerts, sensitivity testing, and adverse event reporting
- **Quality Control**: Manufacturing standards, testing protocols, and certification

## 📊 Core Concepts

### 1. Ingredient Safety Score

```
Safety Score = (Toxicity × 0.3) + (Allergenicity × 0.3) + (Irritation × 0.2) + (Regulatory × 0.2)
```

Where each factor is scored 0-100 (100 = safest)

### 2. Product Stability Index

```
Stability Index = exp(-k × t)
```

Where:
- `k` = degradation rate constant
- `t` = time (months)

### 3. Efficacy Score

```
Efficacy Score = (Clinical Results × 0.5) + (Consumer Satisfaction × 0.3) + (Durability × 0.2)
```

### 4. Sustainability Rating

```
Sustainability = (Ingredient Sourcing × 0.4) + (Packaging × 0.3) + (Manufacturing × 0.3)
```

### 5. Concentration Calculator

```
Final Concentration (%) = (Ingredient Mass / Total Formula Mass) × 100
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  validateIngredient,
  calculateSafetyScore,
  checkRegulatory,
  analyzeFormulation,
  trackBatch
} from '@wia/ind-005';

// Validate ingredient safety
const ingredient = await validateIngredient({
  inci_name: 'Sodium Hyaluronate',
  cas_number: '9067-32-7',
  concentration_percent: 1.5,
  function: 'Humectant'
});

// Calculate product safety score
const safetyScore = calculateSafetyScore({
  ingredients: formulation.ingredients,
  productType: 'facial_serum',
  targetDemographic: 'adult'
});

// Check regulatory compliance
const compliance = await checkRegulatory({
  productType: 'sunscreen',
  ingredients: formulation.ingredients,
  markets: ['US', 'EU', 'KR', 'JP']
});

console.log(`Safety Score: ${safetyScore.overall}/100`);
console.log(`Compliant in: ${compliance.approvedMarkets.join(', ')}`);
```

### CLI Tool

```bash
# Validate ingredient
wia-ind-005 validate-ingredient --inci "Retinol" --concentration 0.5

# Calculate safety score
wia-ind-005 calc-safety --formula product.json --type serum

# Check regulatory compliance
wia-ind-005 check-regulatory --product sunscreen.json --markets US,EU,KR

# Analyze formulation
wia-ind-005 analyze-formula --file moisturizer.json

# Track batch
wia-ind-005 track-batch --batch-id B2025001 --product-id PRD-001

# Generate safety data sheet
wia-ind-005 generate-sds --formula product.json --output sds.pdf
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-005-v1.0.md](./spec/WIA-IND-005-v1.0.md) | Complete specification with technical details |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-005.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/cosmetics-data

# Run installation script
./install.sh

# Verify installation
wia-ind-005 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-005

# Or yarn
yarn add @wia/ind-005
```

```typescript
import { CosmeticsSDK } from '@wia/ind-005';

const cosmetics = new CosmeticsSDK({
  apiKey: 'your-api-key',
  region: 'US'
});

// Create product formulation
const formula = cosmetics.createFormulation({
  productType: 'facial_cream',
  targetSkin: 'dry',
  ingredients: [
    {
      inci_name: 'Aqua',
      concentration_percent: 65.0,
      function: 'Solvent'
    },
    {
      inci_name: 'Glycerin',
      concentration_percent: 5.0,
      function: 'Humectant'
    },
    {
      inci_name: 'Niacinamide',
      concentration_percent: 3.0,
      function: 'Active'
    }
  ]
});

// Validate formulation
const validation = await cosmetics.validateFormulation(formula);
console.log(`Safety: ${validation.safety.score}/100`);
console.log(`Stability: ${validation.stability.rating}`);
console.log(`Regulatory: ${validation.regulatory.status}`);
```

## 🧪 Ingredient Categories

### Active Ingredients
- **Anti-aging**: Retinol, Peptides, Vitamin C, AHAs, BHAs
- **Moisturizing**: Hyaluronic Acid, Glycerin, Ceramides, Squalane
- **Brightening**: Niacinamide, Arbutin, Kojic Acid, Vitamin C
- **Sun Protection**: UV Filters (Chemical and Physical)

### Functional Ingredients
- **Emollients**: Natural oils, Butters, Silicones
- **Preservatives**: Phenoxyethanol, Parabens, Natural alternatives
- **Emulsifiers**: Polysorbates, Lecithin, Cetearyl Alcohol
- **Thickeners**: Xanthan Gum, Carbomer, Cellulose derivatives

### Specialty Ingredients
- **Antioxidants**: Vitamin E, Green Tea, Resveratrol
- **Botanicals**: Plant extracts, Essential oils, Herbal compounds
- **Peptides**: Signal peptides, Carrier peptides, Neurotransmitter peptides
- **Probiotics**: Fermented ingredients, Microbiome-friendly compounds

## 📋 Regulatory Frameworks

### United States (FDA)
- **Category**: Cosmetics vs. Drugs
- **Requirements**: Safety substantiation, labeling, adverse events
- **SPF Products**: OTC drug classification
- **Color Additives**: Pre-market approval required

### European Union (EU)
- **Regulation**: EC 1223/2009
- **CPSR**: Cosmetic Product Safety Report required
- **CPNP**: Cosmetic Products Notification Portal
- **Restricted Substances**: Annex II, III, IV lists

### South Korea (K-Beauty)
- **Agency**: MFDS (Ministry of Food and Drug Safety)
- **Functional Cosmetics**: Special approval required
- **Labeling**: Korean language mandatory
- **Testing**: Some animal testing required

### Japan (J-Beauty)
- **Law**: Pharmaceutical and Medical Device Act
- **Categories**: Cosmetics vs. Quasi-drugs
- **Standards**: Japanese Cosmetic Ingredient Codex
- **Approval**: Pre-market notification system

### China
- **NMPA**: National Medical Products Administration
- **Registration**: Mandatory for imported cosmetics
- **Animal Testing**: Required for certain categories
- **IECIC**: Inventory of Existing Cosmetic Ingredients in China

## 🔬 Safety Assessment

### Toxicological Endpoints
1. **Acute Toxicity**: LD50, LC50 values
2. **Skin Irritation**: Primary irritation index
3. **Eye Irritation**: Draize test alternatives
4. **Sensitization**: LLNA, HRIPT protocols
5. **Phototoxicity**: UV exposure response
6. **Genotoxicity**: Ames test, micronucleus test
7. **Carcinogenicity**: Long-term exposure studies
8. **Reproductive Toxicity**: DART studies

### Risk Assessment Formula

```
MoS (Margin of Safety) = NOAEL / SED
```

Where:
- `NOAEL` = No Observed Adverse Effect Level
- `SED` = Systemic Exposure Dose
- MoS ≥ 100 is generally considered safe

## 🌱 Sustainability Metrics

### Ingredient Sourcing
- **Natural Origin**: % of ingredients from natural sources
- **Organic Certification**: USDA, COSMOS, ECOCERT standards
- **Fair Trade**: Ethical sourcing verification
- **Biodegradability**: Environmental impact assessment

### Packaging
- **Recyclability**: % recyclable materials
- **PCR Content**: Post-consumer recycled content
- **Refillable**: Refill system availability
- **Zero Waste**: Plastic-free options

### Manufacturing
- **Carbon Footprint**: CO2 emissions per unit
- **Water Usage**: Liters per production batch
- **Energy Source**: Renewable energy percentage
- **Waste Reduction**: Manufacturing waste metrics

## 📊 Product Types

| Category | Subcategory | pH Range | Key Functions |
|----------|-------------|----------|---------------|
| Skincare | Cleanser | 4.5-7.0 | Surfactants, Mild acids |
| Skincare | Toner | 4.5-6.0 | Hydration, pH adjustment |
| Skincare | Serum | 5.0-7.0 | Active delivery, Penetration |
| Skincare | Moisturizer | 5.0-7.0 | Occlusion, Humectancy |
| Skincare | Sunscreen | 6.0-8.0 | UV protection, Water resistance |
| Haircare | Shampoo | 4.5-6.5 | Cleansing, Scalp care |
| Haircare | Conditioner | 3.5-5.0 | Detangling, Smoothing |
| Makeup | Foundation | 6.0-8.0 | Coverage, Longevity |
| Makeup | Lipstick | 5.0-7.0 | Color, Moisture |

## 🔍 Quality Control

### Testing Protocols
1. **Stability Testing**
   - Accelerated (40°C/75% RH, 3 months)
   - Long-term (25°C/60% RH, 12+ months)
   - Freeze-thaw cycles
   - Photostability

2. **Microbiological Testing**
   - Total aerobic count
   - Yeast and mold count
   - Pathogen screening (E. coli, S. aureus, P. aeruginosa)
   - Preservative efficacy (Challenge test)

3. **Physical/Chemical Testing**
   - pH measurement
   - Viscosity
   - Specific gravity
   - Color and odor
   - Active ingredient assay

4. **Safety Testing**
   - HRIPT (Human Repeat Insult Patch Test)
   - RIPT (Repeat Insult Patch Test)
   - Ocular irritation alternatives
   - Photopatch testing

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language product queries and recommendations
- **WIA-OMNI-API**: Universal API for retail and e-commerce integration
- **WIA-SOCIAL**: Product reviews, ingredient education, community sharing
- **WIA-HEALTH**: Skin condition tracking, allergy management
- **WIA-SUPPLY**: Supply chain transparency and traceability
- **WIA-CERT**: Product certification and compliance verification

## 📖 Use Cases

1. **Product Development**: Formulation design and optimization
2. **Regulatory Affairs**: Compliance verification and documentation
3. **Quality Control**: Manufacturing quality assurance
4. **Consumer Safety**: Allergen alerts and sensitivity matching
5. **E-commerce**: Product information and transparency
6. **Supply Chain**: Ingredient traceability and sustainability
7. **Clinical Research**: Efficacy studies and claims substantiation
8. **Market Analysis**: Ingredient trends and consumer preferences

## ⚠️ Common Allergens

### EU 26 Allergens (Fragrance)
Must be labeled if concentration exceeds:
- Leave-on products: > 0.001%
- Rinse-off products: > 0.01%

Examples:
- Limonene, Linalool, Geraniol
- Citronellol, Eugenol, Coumarin
- Benzyl alcohol, Cinnamal, Citral

### Other Common Allergens
- Preservatives: Formaldehyde releasers, Isothiazolinones
- Surfactants: Cocamidopropyl betaine
- UV Filters: Oxybenzone, Octinoxate
- Proteins: Wheat, Soy, Nut derivatives

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
