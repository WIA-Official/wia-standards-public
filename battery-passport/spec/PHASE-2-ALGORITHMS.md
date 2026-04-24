# WIA-BATTERY-PASSPORT: Phase 2 - Algorithms

**EU 배터리 여권 알고리즘**
*Battery health, carbon, and lifecycle algorithms*

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document specifies the algorithms used for:
1. State of Health (SOH) calculation
2. Remaining Useful Life (RUL) prediction
3. Carbon footprint calculation
4. Second-life eligibility assessment
5. Recycling efficiency calculation
6. Supply chain verification

---

## 2. State of Health (SOH) Calculator

### 2.1 Algorithm Overview

```
SOH = (Current_Capacity / Original_Capacity) × 100%
```

### 2.2 Detailed Implementation

```python
def calculate_soh(battery_data: BatteryHealth) -> SOHResult:
    """
    Calculate State of Health using multiple indicators.

    Inputs:
        - current_capacity_ah: Current measured capacity
        - original_capacity_ah: Nameplate capacity
        - internal_resistance_mohm: Current internal resistance
        - original_resistance_mohm: Initial internal resistance
        - full_cycle_equivalents: Total full cycle count
        - expected_cycle_life: Rated cycle life

    Returns:
        - soh_percent: Overall SOH (0-100%)
        - capacity_soh: Capacity-based SOH
        - resistance_soh: Resistance-based SOH
        - cycle_soh: Cycle-based SOH
        - confidence: Confidence level
    """

    # 1. Capacity-based SOH (primary metric)
    capacity_soh = (current_capacity_ah / original_capacity_ah) * 100

    # 2. Resistance-based SOH
    # Resistance increases as battery degrades
    resistance_increase = (internal_resistance_mohm - original_resistance_mohm) / original_resistance_mohm
    resistance_soh = max(0, 100 - (resistance_increase * 200))  # 50% increase = 0% SOH

    # 3. Cycle-based SOH (remaining cycle capacity)
    cycle_soh = max(0, (1 - (full_cycle_equivalents / expected_cycle_life)) * 100)

    # 4. Weighted combination
    # Capacity is the most reliable indicator
    weights = {
        'capacity': 0.6,
        'resistance': 0.25,
        'cycle': 0.15
    }

    overall_soh = (
        capacity_soh * weights['capacity'] +
        resistance_soh * weights['resistance'] +
        cycle_soh * weights['cycle']
    )

    # 5. Confidence calculation
    confidence = calculate_confidence(battery_data)

    return SOHResult(
        soh_percent=round(overall_soh, 1),
        capacity_soh=round(capacity_soh, 1),
        resistance_soh=round(resistance_soh, 1),
        cycle_soh=round(cycle_soh, 1),
        confidence=confidence,
        measurement_date=datetime.utcnow()
    )


def calculate_confidence(data: BatteryHealth) -> ConfidenceLevel:
    """Determine confidence level based on data quality."""

    if data.data_source == HealthDataSource.BMS:
        if data.last_bms_sync > datetime.utcnow() - timedelta(days=7):
            return ConfidenceLevel.HIGH
        return ConfidenceLevel.MEDIUM

    if data.data_source == HealthDataSource.DIAGNOSTIC:
        return ConfidenceLevel.HIGH

    return ConfidenceLevel.LOW
```

### 2.3 SOH Classification

| SOH Range | Status | Recommendation |
|-----------|--------|----------------|
| 100-90% | Excellent | Normal operation |
| 90-80% | Good | Monitor closely |
| 80-70% | Fair | Consider second-life |
| 70-60% | Degraded | Second-life application |
| <60% | Poor | Recycling recommended |

---

## 3. Remaining Useful Life (RUL) Predictor

### 3.1 Algorithm Overview

Predicts when battery will reach End-of-Life (typically 80% SOH).

### 3.2 Implementation

```python
def predict_remaining_life(
    battery: BatteryPassport,
    usage_profile: UsageProfile
) -> RULResult:
    """
    Predict remaining useful life using degradation modeling.

    Methods:
        1. Linear extrapolation (baseline)
        2. Capacity fade model (Arrhenius)
        3. Machine learning (if sufficient data)
    """

    health = battery.health
    specs = battery.specifications

    # Historical data points
    soh_history = get_soh_history(battery.id)

    if len(soh_history) < 3:
        # Not enough data - use empirical model
        return empirical_rul_model(battery, usage_profile)

    # 1. Linear regression on SOH history
    linear_rul = linear_extrapolation(soh_history, target_soh=80.0)

    # 2. Capacity fade model (considering temperature)
    arrhenius_rul = arrhenius_degradation_model(
        current_soh=health.state_of_health_percent,
        avg_temperature=usage_profile.avg_operating_temp_c,
        cycles_per_year=usage_profile.annual_cycles,
        chemistry=battery.identity.chemistry
    )

    # 3. Stress factor adjustments
    stress_factor = calculate_stress_factor(
        fast_charge_ratio=health.fast_charge_count / (health.fast_charge_count + health.slow_charge_count),
        time_at_high_temp=health.time_above_45c_hours,
        time_at_low_temp=health.time_below_0c_hours,
        avg_dod=usage_profile.avg_depth_of_discharge
    )

    # Apply stress factor
    adjusted_rul = arrhenius_rul / stress_factor

    # 4. Calculate end-of-life date
    eol_date = datetime.utcnow() + timedelta(days=adjusted_rul * 30)

    return RULResult(
        remaining_months=round(adjusted_rul),
        remaining_cycles=estimate_remaining_cycles(battery),
        expected_eol_date=eol_date,
        confidence=calculate_rul_confidence(soh_history),
        methodology="arrhenius_adjusted"
    )


def arrhenius_degradation_model(
    current_soh: float,
    avg_temperature: float,
    cycles_per_year: int,
    chemistry: BatteryChemistry
) -> float:
    """
    Arrhenius-based capacity fade model.

    Degradation rate doubles for every 10°C above 25°C.
    """

    # Base degradation rates by chemistry (% per 1000 cycles at 25°C)
    base_rates = {
        BatteryChemistry.LFP: 2.0,      # Most stable
        BatteryChemistry.NMC: 3.5,
        BatteryChemistry.NCA: 4.0,
        BatteryChemistry.LCO: 5.0,
        BatteryChemistry.LTO: 1.5,      # Very stable
    }

    base_rate = base_rates.get(chemistry, 3.5)

    # Temperature acceleration factor
    # Arrhenius equation: k = A * exp(-Ea/RT)
    # Simplified: factor = 2^((T-25)/10)
    temp_factor = 2 ** ((avg_temperature - 25) / 10)

    # Adjusted degradation rate
    adjusted_rate = base_rate * temp_factor

    # Calculate remaining SOH capacity to lose (until 80%)
    soh_to_lose = current_soh - 80.0

    if soh_to_lose <= 0:
        return 0  # Already at or below EOL

    # Cycles remaining
    cycles_remaining = (soh_to_lose / adjusted_rate) * 1000

    # Convert to months
    months_remaining = (cycles_remaining / cycles_per_year) * 12

    return months_remaining


def calculate_stress_factor(
    fast_charge_ratio: float,
    time_at_high_temp: float,
    time_at_low_temp: float,
    avg_dod: float
) -> float:
    """
    Calculate stress factor that accelerates degradation.

    Returns factor > 1 for higher stress conditions.
    """

    factor = 1.0

    # Fast charging stress (DC fast charging accelerates degradation)
    if fast_charge_ratio > 0.5:
        factor *= 1.0 + (fast_charge_ratio - 0.5) * 0.3  # Up to 15% increase

    # High temperature stress
    if time_at_high_temp > 100:  # hours
        factor *= 1.0 + min(time_at_high_temp / 1000, 0.3)  # Up to 30% increase

    # Low temperature stress (charging at low temp is damaging)
    if time_at_low_temp > 50:
        factor *= 1.0 + min(time_at_low_temp / 500, 0.2)  # Up to 20% increase

    # Deep discharge stress
    if avg_dod > 0.8:
        factor *= 1.0 + (avg_dod - 0.8) * 0.5  # Up to 10% increase

    return factor
```

---

## 4. Carbon Footprint Calculator

### 4.1 Algorithm Overview

Calculates lifecycle carbon footprint following ISO 14067 / EU PEF methodology.

### 4.2 Implementation

```python
def calculate_carbon_footprint(
    battery: BatteryPassport,
    supply_chain: List[SupplyChainRecord]
) -> CarbonFootprint:
    """
    Calculate total lifecycle carbon footprint.

    Phases:
        1. Raw material acquisition & processing
        2. Manufacturing
        3. Transport (cradle-to-gate)
    """

    specs = battery.specifications
    materials = battery.materials

    # 1. Raw material emissions
    material_emissions = calculate_material_emissions(materials)

    # 2. Manufacturing emissions
    manufacturing_emissions = calculate_manufacturing_emissions(
        capacity_wh=specs.rated_capacity_wh,
        chemistry=battery.identity.chemistry,
        facility_country=battery.identity.production_country
    )

    # 3. Transport emissions
    transport_emissions = calculate_transport_emissions(supply_chain)

    # Total
    total_emissions = (
        material_emissions +
        manufacturing_emissions +
        transport_emissions
    )

    # Per kWh
    per_kwh = total_emissions / (specs.rated_capacity_wh / 1000)

    # Determine class
    carbon_class = determine_carbon_class(per_kwh)

    return CarbonFootprint(
        total_kg_co2e=round(total_emissions, 2),
        per_kwh_kg_co2e=round(per_kwh, 2),
        raw_material_acquisition=round(material_emissions, 2),
        manufacturing=round(manufacturing_emissions, 2),
        transport=round(transport_emissions, 2),
        performance_class=carbon_class,
        methodology="ISO 14067 / EU PEF",
        calculation_date=datetime.utcnow(),
        third_party_verified=False
    )


def calculate_material_emissions(materials: MaterialComposition) -> float:
    """
    Calculate emissions from raw material extraction and processing.
    """

    # Emission factors (kg CO2e per kg material)
    emission_factors = {
        'lithium': 15.0,          # Lithium carbonate
        'cobalt': 35.0,           # Refined cobalt
        'nickel': 12.0,           # Class 1 nickel
        'manganese': 3.0,
        'graphite': 4.5,          # Synthetic graphite
        'aluminum': 12.0,         # Primary aluminum
        'copper': 4.0,
        'steel': 2.0,
        'plastics': 3.5
    }

    # Recycled content reduces emissions
    recycled_reduction = {
        'lithium': 0.7,           # 70% reduction if recycled
        'cobalt': 0.8,
        'nickel': 0.75,
        'aluminum': 0.95,         # Recycled Al is much cleaner
        'copper': 0.85
    }

    total_emissions = 0.0

    # Calculate for each critical material
    for material_name, info in [
        ('lithium', materials.lithium),
        ('cobalt', materials.cobalt),
        ('nickel', materials.nickel),
        ('manganese', materials.manganese),
        ('graphite', materials.graphite)
    ]:
        base_factor = emission_factors.get(material_name, 5.0)
        recycled_pct = info.recycled_content_percent / 100

        # Weighted average of virgin and recycled
        effective_factor = (
            base_factor * (1 - recycled_pct) +
            base_factor * (1 - recycled_reduction.get(material_name, 0.7)) * recycled_pct
        )

        total_emissions += info.weight_kg * effective_factor

    return total_emissions


def calculate_manufacturing_emissions(
    capacity_wh: float,
    chemistry: BatteryChemistry,
    facility_country: str
) -> float:
    """
    Calculate manufacturing emissions based on energy consumption.
    """

    # Energy consumption per kWh capacity (kWh electricity)
    energy_intensity = {
        BatteryChemistry.LFP: 45,
        BatteryChemistry.NMC: 55,
        BatteryChemistry.NCA: 60,
        BatteryChemistry.LTO: 70,
    }

    # Grid emission factors by country (kg CO2e / kWh)
    grid_factors = {
        'CN': 0.58,    # China
        'KR': 0.42,    # South Korea
        'JP': 0.45,    # Japan
        'DE': 0.35,    # Germany
        'US': 0.40,    # USA average
        'SE': 0.02,    # Sweden (hydro/nuclear)
        'NO': 0.01,    # Norway (hydro)
        'PL': 0.65,    # Poland (coal)
    }

    energy_per_kwh = energy_intensity.get(chemistry, 50)
    grid_factor = grid_factors.get(facility_country, 0.50)

    capacity_kwh = capacity_wh / 1000
    electricity_consumed = capacity_kwh * energy_per_kwh
    emissions = electricity_consumed * grid_factor

    return emissions


def calculate_transport_emissions(supply_chain: List[SupplyChainRecord]) -> float:
    """
    Calculate transport emissions across supply chain.
    """

    # Emission factors (kg CO2e per ton-km)
    transport_factors = {
        'sea': 0.016,
        'rail': 0.028,
        'road': 0.096,
        'air': 0.602
    }

    total_emissions = 0.0

    for record in supply_chain:
        if record.transport_mode and record.distance_km and record.weight_kg:
            factor = transport_factors.get(record.transport_mode, 0.05)
            ton_km = (record.weight_kg / 1000) * record.distance_km
            total_emissions += ton_km * factor

    return total_emissions


def determine_carbon_class(per_kwh_kg_co2e: float) -> CarbonClass:
    """
    Determine EU carbon performance class.
    """

    if per_kwh_kg_co2e < 50:
        return CarbonClass.A
    elif per_kwh_kg_co2e < 65:
        return CarbonClass.B
    elif per_kwh_kg_co2e < 80:
        return CarbonClass.C
    elif per_kwh_kg_co2e < 95:
        return CarbonClass.D
    else:
        return CarbonClass.E
```

---

## 5. Second-Life Eligibility Assessor

### 5.1 Algorithm Overview

Determines if a battery is suitable for second-life applications (e.g., EV → stationary storage).

### 5.2 Implementation

```python
def assess_second_life_eligibility(
    battery: BatteryPassport
) -> SecondLifeResult:
    """
    Assess battery suitability for second-life applications.

    Criteria:
        - SOH between 70-80% (typical EV EOL)
        - No safety issues
        - Sufficient remaining capacity
        - Economic viability
    """

    health = battery.health
    specs = battery.specifications

    checks = []
    score = 0

    # 1. SOH check (sweet spot: 70-80%)
    soh = health.state_of_health_percent
    if 65 <= soh <= 85:
        checks.append(Check("soh_range", True, f"SOH {soh}% in acceptable range"))
        score += 30
    elif 60 <= soh < 65:
        checks.append(Check("soh_range", True, f"SOH {soh}% marginal but acceptable"))
        score += 15
    else:
        checks.append(Check("soh_range", False, f"SOH {soh}% outside range"))

    # 2. Capacity check (minimum useful capacity)
    remaining_capacity_wh = specs.rated_capacity_wh * (soh / 100)
    if remaining_capacity_wh >= 10000:  # 10 kWh minimum for stationary
        checks.append(Check("capacity", True, f"{remaining_capacity_wh/1000:.1f} kWh remaining"))
        score += 20
    else:
        checks.append(Check("capacity", False, f"Only {remaining_capacity_wh/1000:.1f} kWh remaining"))

    # 3. Remaining useful life
    rul_result = predict_remaining_life(battery, UsageProfile.STATIONARY_DEFAULT)
    if rul_result.remaining_months >= 36:  # 3+ years remaining
        checks.append(Check("rul", True, f"{rul_result.remaining_months} months remaining"))
        score += 25
    elif rul_result.remaining_months >= 24:
        checks.append(Check("rul", True, f"{rul_result.remaining_months} months remaining (marginal)"))
        score += 15
    else:
        checks.append(Check("rul", False, f"Only {rul_result.remaining_months} months remaining"))

    # 4. Safety check (no thermal events, balanced cells)
    safety_ok = (
        health.max_temperature_reached_c < 60 and
        health.resistance_increase_percent < 50
    )
    if safety_ok:
        checks.append(Check("safety", True, "No safety concerns"))
        score += 25
    else:
        checks.append(Check("safety", False, "Safety review required"))

    # Determine eligibility
    eligible = score >= 70

    # Suggest applications
    applications = suggest_applications(battery, score)

    return SecondLifeResult(
        eligible=eligible,
        score=score,
        checks=checks,
        suggested_applications=applications,
        estimated_remaining_value=estimate_residual_value(battery, score),
        recertification_required=True
    )


def suggest_applications(battery: BatteryPassport, score: int) -> List[str]:
    """
    Suggest suitable second-life applications.
    """

    applications = []
    capacity_kwh = battery.specifications.rated_capacity_wh / 1000

    if score >= 80:
        applications.append("Grid-scale energy storage")
        applications.append("Commercial/industrial backup")

    if score >= 70:
        applications.append("Residential energy storage")
        applications.append("EV charging station buffer")

    if score >= 60:
        applications.append("Low-power backup systems")
        applications.append("Agricultural applications")

    if capacity_kwh < 20:
        applications.append("Small-scale solar storage")
        applications.append("Off-grid applications")

    return applications


def estimate_residual_value(battery: BatteryPassport, score: int) -> ResidualValue:
    """
    Estimate residual economic value.
    """

    specs = battery.specifications
    original_value_per_kwh = 150  # USD/kWh (assumed original cost)

    # Value depreciation curve
    soh = battery.health.state_of_health_percent
    depreciation = 1 - ((100 - soh) / 100) ** 1.5  # Non-linear depreciation

    # Score adjustment
    score_factor = score / 100

    # Calculate
    original_value = specs.rated_capacity_wh / 1000 * original_value_per_kwh
    residual_value = original_value * depreciation * score_factor

    return ResidualValue(
        estimated_usd=round(residual_value, 2),
        per_kwh_usd=round(residual_value / (specs.rated_capacity_wh / 1000), 2),
        confidence="medium"
    )
```

---

## 6. Recycling Efficiency Calculator

### 6.1 Algorithm Overview

Calculates material recovery efficiency and compliance with EU targets.

### 6.2 Implementation

```python
def calculate_recycling_efficiency(
    battery: BatteryPassport,
    recycling_output: RecycledData
) -> RecyclingEfficiencyResult:
    """
    Calculate recycling efficiency and EU compliance.

    EU Targets (by 2031):
        - Cobalt: 95%
        - Lithium: 80%
        - Nickel: 95%
        - Copper: 95%
    """

    materials = battery.materials
    recovered = recycling_output.materials_recovered

    # EU 2031 targets
    targets = {
        'cobalt': 0.95,
        'lithium': 0.80,
        'nickel': 0.95,
        'copper': 0.95,
        'lead': 0.95  # For lead-acid
    }

    results = {}
    compliant = True

    for material_name in ['cobalt', 'lithium', 'nickel', 'copper']:
        original = getattr(materials, material_name, None)
        if original is None or original.weight_kg == 0:
            continue

        recovered_weight = next(
            (r.weight_kg for r in recovered if r.material.lower() == material_name),
            0
        )

        efficiency = recovered_weight / original.weight_kg
        target = targets.get(material_name, 0.80)

        results[material_name] = MaterialRecoveryResult(
            original_kg=original.weight_kg,
            recovered_kg=recovered_weight,
            efficiency_percent=round(efficiency * 100, 1),
            target_percent=round(target * 100, 1),
            meets_target=efficiency >= target
        )

        if efficiency < target:
            compliant = False

    # Overall efficiency
    total_original = sum(getattr(materials, m).weight_kg for m in ['cobalt', 'lithium', 'nickel'] if hasattr(materials, m))
    total_recovered = sum(r.weight_kg for r in recovered if r.material.lower() in ['cobalt', 'lithium', 'nickel'])
    overall_efficiency = total_recovered / total_original if total_original > 0 else 0

    return RecyclingEfficiencyResult(
        material_results=results,
        overall_efficiency_percent=round(overall_efficiency * 100, 1),
        eu_compliant=compliant,
        process_type=recycling_output.process_type,
        recommendations=generate_recycling_recommendations(results)
    )


def generate_recycling_recommendations(results: Dict[str, MaterialRecoveryResult]) -> List[str]:
    """
    Generate recommendations to improve recycling efficiency.
    """

    recommendations = []

    for material, result in results.items():
        if not result.meets_target:
            gap = result.target_percent - result.efficiency_percent
            recommendations.append(
                f"Improve {material} recovery by {gap:.1f}% to meet EU target"
            )

    if not any(r.meets_target for r in results.values()):
        recommendations.append("Consider switching to hydrometallurgical process for better recovery rates")

    return recommendations
```

---

## 7. Supply Chain Verification

### 7.1 Algorithm Overview

Verifies responsible sourcing claims and due diligence compliance.

### 7.2 Implementation

```python
def verify_supply_chain(
    battery: BatteryPassport,
    supply_chain: List[SupplyChainRecord]
) -> SupplyChainVerificationResult:
    """
    Verify supply chain compliance with:
        - EU Battery Regulation due diligence
        - OECD Due Diligence Guidance
        - Responsible sourcing standards (RMI, IRMA, etc.)
    """

    materials = battery.materials
    verifications = []
    risk_score = 0
    max_risk = 0

    # Check each critical material
    for material_name, info in [
        ('cobalt', materials.cobalt),
        ('lithium', materials.lithium),
        ('nickel', materials.nickel)
    ]:
        max_risk += 100

        # Source country risk assessment
        country_risk = assess_country_risk(info.source_country)
        risk_score += country_risk

        # Certification verification
        cert_valid = verify_certification(info.responsible_sourcing)

        verifications.append(MaterialVerification(
            material=material_name,
            source_countries=info.source_country,
            country_risk_level=country_risk_level(country_risk),
            certification_scheme=info.responsible_sourcing.certification_scheme,
            certification_valid=cert_valid,
            due_diligence_complete=info.responsible_sourcing.due_diligence_report_url is not None,
            audit_date=info.responsible_sourcing.audit_date
        ))

    # Supply chain traceability check
    traceability_score = calculate_traceability(supply_chain)

    # Overall compliance
    overall_risk = (risk_score / max_risk) * 100 if max_risk > 0 else 0
    compliant = overall_risk < 30 and all(v.certification_valid for v in verifications)

    return SupplyChainVerificationResult(
        material_verifications=verifications,
        overall_risk_score=round(overall_risk, 1),
        traceability_score=traceability_score,
        eu_due_diligence_compliant=compliant,
        oecd_guidance_compliant=compliant,
        recommendations=generate_supply_chain_recommendations(verifications, overall_risk)
    )


def assess_country_risk(countries: List[str]) -> int:
    """
    Assess sourcing risk based on country of origin.

    High-risk countries for conflict minerals.
    """

    # Risk scores (0-100)
    country_risks = {
        'CD': 90,    # DRC - high conflict mineral risk
        'RW': 70,
        'UG': 60,
        'ZM': 40,    # Zambia
        'AU': 10,    # Australia - low risk
        'CL': 15,    # Chile
        'AR': 20,    # Argentina
        'CN': 35,    # China
        'ID': 30,    # Indonesia
        'PH': 35,    # Philippines
    }

    if not countries:
        return 50  # Unknown = medium risk

    # Return highest risk among source countries
    return max(country_risks.get(c, 50) for c in countries)


def country_risk_level(score: int) -> str:
    if score >= 70:
        return "HIGH"
    elif score >= 40:
        return "MEDIUM"
    else:
        return "LOW"


def verify_certification(sourcing: ResponsibleSourcing) -> bool:
    """
    Verify that certification is valid and current.
    """

    if not sourcing.certified:
        return False

    # Check if audit is recent (within 2 years)
    if sourcing.audit_date:
        audit_age = datetime.utcnow() - sourcing.audit_date
        if audit_age.days > 730:  # 2 years
            return False

    # Verify certification scheme is recognized
    recognized_schemes = ['RMI', 'IRMA', 'ASI', 'LBMA', 'RJC']
    if sourcing.certification_scheme not in recognized_schemes:
        return False

    return True


def calculate_traceability(supply_chain: List[SupplyChainRecord]) -> int:
    """
    Calculate supply chain traceability score (0-100).
    """

    if not supply_chain:
        return 0

    total_score = 0

    for record in supply_chain:
        record_score = 0

        # Has origin information
        if record.origin_country:
            record_score += 25

        # Has supplier identification
        if record.supplier_id:
            record_score += 25

        # Has timestamp
        if record.timestamp:
            record_score += 25

        # Has verification
        if record.verified:
            record_score += 25

        total_score += record_score

    return min(100, total_score // len(supply_chain))
```

---

## 8. Data Quality Scoring

### 8.1 Implementation

```python
def calculate_data_quality_score(battery: BatteryPassport) -> DataQualityResult:
    """
    Assess overall data quality of battery passport.
    """

    scores = {}

    # Identity completeness
    identity = battery.identity
    identity_fields = ['unique_identifier', 'chemistry', 'model', 'serial_number', 'production_date']
    identity_score = sum(1 for f in identity_fields if getattr(identity, f, None)) / len(identity_fields) * 100
    scores['identity'] = identity_score

    # Health data freshness
    health = battery.health
    if health.last_bms_sync:
        days_old = (datetime.utcnow() - health.last_bms_sync).days
        if days_old < 7:
            scores['health'] = 100
        elif days_old < 30:
            scores['health'] = 80
        elif days_old < 90:
            scores['health'] = 50
        else:
            scores['health'] = 20
    else:
        scores['health'] = 0

    # Carbon footprint verification
    carbon = battery.carbon_footprint
    if carbon.third_party_verified:
        scores['carbon'] = 100
    elif carbon.methodology:
        scores['carbon'] = 60
    else:
        scores['carbon'] = 20

    # Supply chain traceability
    if battery.supply_chain:
        scores['supply_chain'] = calculate_traceability(battery.supply_chain)
    else:
        scores['supply_chain'] = 0

    # Overall score
    weights = {'identity': 0.2, 'health': 0.3, 'carbon': 0.25, 'supply_chain': 0.25}
    overall = sum(scores[k] * weights[k] for k in weights)

    return DataQualityResult(
        overall_score=round(overall, 1),
        category_scores=scores,
        grade=data_quality_grade(overall),
        recommendations=generate_data_quality_recommendations(scores)
    )


def data_quality_grade(score: float) -> str:
    if score >= 90:
        return "A"
    elif score >= 75:
        return "B"
    elif score >= 60:
        return "C"
    elif score >= 40:
        return "D"
    else:
        return "F"
```

---

**Document ID**: WIA-BATTERY-PASSPORT-PHASE-2
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
