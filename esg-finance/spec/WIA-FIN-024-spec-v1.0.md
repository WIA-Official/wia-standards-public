# WIA-FIN-024: ESG Finance Standard

**Version:** 1.0
**Status:** Stable
**Published:** January 2024
**Category:** Finance (FIN)

---

## 1. Overview

The WIA-FIN-024 ESG Finance Standard defines requirements and best practices for integrating Environmental, Social, and Governance (ESG) principles into financial systems, investment analysis, and corporate reporting.

### 1.1 Purpose

This standard aims to:
- Establish consistent ESG measurement and reporting frameworks
- Enable transparent ESG data collection and disclosure
- Facilitate ESG-integrated investment decision-making
- Support regulatory compliance across jurisdictions
- Promote sustainable finance and capital allocation

### 1.2 Scope

This standard covers:
- ESG scoring and rating methodologies
- Data collection and management processes
- Disclosure and reporting requirements
- Green finance instruments (bonds, loans)
- Impact measurement and verification

## 2. Normative References

- GHG Protocol Corporate Accounting and Reporting Standard
- TCFD Recommendations
- SASB Standards
- GRI Standards
- ISO 14064 (GHG quantification)
- ISO 26000 (Social responsibility)

## 3. Terms and Definitions

**ESG**: Environmental, Social, and Governance factors integrated into business and investment analysis

**Materiality**: ESG issues that have substantial impact on financial performance or stakeholder value

**Scope 1 Emissions**: Direct GHG emissions from owned or controlled sources

**Scope 2 Emissions**: Indirect GHG emissions from purchased energy

**Scope 3 Emissions**: All other indirect emissions in the value chain

**Green Bond**: Fixed-income instrument specifically earmarked for climate and environmental projects

## 4. ESG Scoring Framework

### 4.1 Environmental Score (E)

**Weight:** 35%

**Components:**
- Carbon emissions intensity (15%)
- Renewable energy adoption (10%)
- Water management (5%)
- Waste and circular economy (5%)

**Calculation:**
```
E_score = (carbon_score * 0.15) + (renewable_score * 0.10) +
          (water_score * 0.05) + (waste_score * 0.05)
```

### 4.2 Social Score (S)

**Weight:** 30%

**Components:**
- Diversity & inclusion (10%)
- Employee well-being (8%)
- Labor practices (7%)
- Community impact (5%)

**Calculation:**
```
S_score = (diversity_score * 0.10) + (wellbeing_score * 0.08) +
          (labor_score * 0.07) + (community_score * 0.05)
```

### 4.3 Governance Score (G)

**Weight:** 35%

**Components:**
- Board independence (12%)
- Executive compensation (8%)
- Shareholder rights (8%)
- Ethics & compliance (7%)

**Calculation:**
```
G_score = (board_score * 0.12) + (compensation_score * 0.08) +
          (shareholder_score * 0.08) + (ethics_score * 0.07)
```

### 4.4 Overall ESG Score

```
ESG_score = (E_score * 0.35) + (S_score * 0.30) + (G_score * 0.35)
```

**Scale:** 0-100

**Ratings:**
- 90-100: AAA (Leader)
- 80-89: AA (Advanced)
- 70-79: A (Good)
- 60-69: BBB (Average)
- 50-59: BB (Laggard)
- Below 50: B (Critical)

## 5. Data Collection Requirements

### 5.1 Environmental Data

**REQUIRED:**
- Annual GHG emissions (Scope 1, 2, 3)
- Energy consumption by source
- Water withdrawal and consumption
- Waste generated and disposal method

**RECOMMENDED:**
- Biodiversity impact assessment
- Pollution emissions (air, water, soil)
- Climate risk scenario analysis

### 5.2 Social Data

**REQUIRED:**
- Employee demographics (gender, ethnicity)
- Workplace safety incidents (TRIR, LTIFR)
- Employee turnover rate
- Training hours per employee

**RECOMMENDED:**
- Pay equity analysis
- Employee engagement scores
- Supply chain labor audits
- Community investment amount

### 5.3 Governance Data

**REQUIRED:**
- Board composition and independence
- Executive compensation structure
- Shareholder voting results
- Ethics violations and resolutions

**RECOMMENDED:**
- Political spending disclosure
- Lobbying expenditures
- Cybersecurity incidents
- Tax transparency

## 6. Disclosure Requirements

### 6.1 Minimum Disclosure

Organizations MUST disclose:
- ESG governance structure
- Material ESG risks and opportunities
- Key ESG metrics and performance
- Progress against stated targets
- Assurance statement (if applicable)

### 6.2 Reporting Frameworks

Align disclosure with at least ONE of:
- TCFD (climate-focused)
- SASB (industry-specific)
- GRI (stakeholder-focused)

### 6.3 Reporting Frequency

- Annual sustainability report (REQUIRED)
- Quarterly ESG metrics update (RECOMMENDED)

## 7. Green Finance Instruments

### 7.1 Green Bonds

**Eligible Use of Proceeds:**
- Renewable energy projects
- Energy efficiency improvements
- Clean transportation
- Sustainable water management
- Climate change adaptation
- Circular economy initiatives

**Requirements:**
- Independent verification of use of proceeds
- Annual impact reporting
- Alignment with Green Bond Principles

### 7.2 Sustainability-Linked Loans

**Structure:**
- Interest rate tied to ESG performance
- Predefined sustainability performance targets (SPTs)
- Independent verification of target achievement

## 8. Compliance and Assurance

### 8.1 Internal Controls

Organizations SHALL:
- Establish ESG data governance policies
- Assign clear data ownership and accountability
- Implement validation and quality checks
- Maintain audit trail for all data

### 8.2 External Assurance

RECOMMENDED for:
- GHG emissions data (Scope 1, 2)
- Key ESG metrics included in public reporting
- Green bond proceeds allocation

**Assurance Levels:**
- Limited assurance (minimum)
- Reasonable assurance (preferred for material metrics)

## 9. Implementation Requirements

### 9.1 Technology Requirements

**ESG Data Management System SHALL support:**
- Multi-source data aggregation
- Automated calculations and conversions
- Historical trend analysis
- Regulatory reporting formats
- API integration for third-party platforms

### 9.2 Personnel Requirements

**Organizations SHOULD appoint:**
- Chief Sustainability Officer (or equivalent)
- ESG steering committee
- Board oversight committee
- Dedicated ESG data analysts

## 10. Conformance

### 10.1 Conformance Levels

**Level 1 - Basic:**
- ESG scoring implementation
- Annual ESG disclosure
- Minimum data collection

**Level 2 - Intermediate:**
- Level 1 requirements
- TCFD/SASB/GRI alignment
- Green finance instruments
- Limited assurance

**Level 3 - Advanced:**
- Level 2 requirements
- Comprehensive Scope 3 reporting
- Science-based targets
- Reasonable assurance
- Real-time ESG dashboard

### 10.2 Certification

Organizations may seek third-party certification of conformance to this standard.

---

## Appendix A: Environmental Metrics Calculation

### A.1 Carbon Intensity

```
Carbon_Intensity = Total_GHG_Emissions_tCO2e / Revenue_Million_USD
```

### A.2 Renewable Energy Percentage

```
Renewable_Energy_% = Renewable_Energy_Consumed / Total_Energy_Consumed * 100
```

---

## Appendix B: Social Metrics Calculation

### B.1 Gender Diversity Ratio

```
Gender_Diversity_% = Min(Women_Count, Men_Count) / Total_Employees * 100
```

### B.2 Total Recordable Incident Rate (TRIR)

```
TRIR = (Number_of_Recordable_Incidents * 200,000) / Total_Hours_Worked
```

---

**© 2024 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
