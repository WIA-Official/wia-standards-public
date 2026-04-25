# CRYO-ASSET Phase 2: Algorithm Specification

## 1. Overview

This document defines the core algorithms for managing assets of cryopreserved individuals.

## 2. Asset Registry Manager

```python
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple
from enum import Enum
from decimal import Decimal
import hashlib
import json


class AssetRegistryStatus(Enum):
    ACTIVE = "ACTIVE"
    FROZEN = "FROZEN"
    TRUST_MANAGED = "TRUST_MANAGED"
    RESTORATION_PENDING = "RESTORATION_PENDING"
    RESTORED = "RESTORED"
    ESTATE_DISTRIBUTED = "ESTATE_DISTRIBUTED"


class AssetStatus(Enum):
    ACTIVE = "ACTIVE"
    FROZEN = "FROZEN"
    UNDER_MANAGEMENT = "UNDER_MANAGEMENT"
    LIQUIDATING = "LIQUIDATING"
    TRANSFERRED = "TRANSFERRED"
    DISPOSED = "DISPOSED"


@dataclass
class MonetaryValue:
    amount: Decimal
    currency: str
    as_of: datetime


@dataclass
class StatusTransition:
    from_status: AssetRegistryStatus
    to_status: AssetRegistryStatus
    timestamp: datetime
    triggered_by: str
    legal_record_id: Optional[str]
    reason: str


class AssetRegistryManager:
    """Manages asset registry lifecycle for cryopreserved individuals."""

    VALID_TRANSITIONS = {
        AssetRegistryStatus.ACTIVE: [
            AssetRegistryStatus.FROZEN
        ],
        AssetRegistryStatus.FROZEN: [
            AssetRegistryStatus.TRUST_MANAGED
        ],
        AssetRegistryStatus.TRUST_MANAGED: [
            AssetRegistryStatus.RESTORATION_PENDING,
            AssetRegistryStatus.ESTATE_DISTRIBUTED
        ],
        AssetRegistryStatus.RESTORATION_PENDING: [
            AssetRegistryStatus.RESTORED,
            AssetRegistryStatus.TRUST_MANAGED
        ],
        AssetRegistryStatus.RESTORED: [],
        AssetRegistryStatus.ESTATE_DISTRIBUTED: []
    }

    def __init__(self, registry_id: str):
        self.registry_id = registry_id
        self.status = AssetRegistryStatus.ACTIVE
        self.status_history: List[StatusTransition] = []
        self.assets: Dict[str, 'Asset'] = {}
        self.trusts: Dict[str, 'TrustRecord'] = {}

    def can_transition_to(self, new_status: AssetRegistryStatus) -> bool:
        """Check if transition to new status is valid."""
        return new_status in self.VALID_TRANSITIONS.get(self.status, [])

    def transition_status(
        self,
        new_status: AssetRegistryStatus,
        triggered_by: str,
        legal_record_id: Optional[str] = None,
        reason: str = ""
    ) -> StatusTransition:
        """Execute status transition with validation."""
        if not self.can_transition_to(new_status):
            raise ValueError(
                f"Invalid transition from {self.status.value} to {new_status.value}"
            )

        transition = StatusTransition(
            from_status=self.status,
            to_status=new_status,
            timestamp=datetime.utcnow(),
            triggered_by=triggered_by,
            legal_record_id=legal_record_id,
            reason=reason
        )

        self.status_history.append(transition)
        old_status = self.status
        self.status = new_status

        # Execute status-specific actions
        self._execute_transition_actions(old_status, new_status)

        return transition

    def _execute_transition_actions(
        self,
        old_status: AssetRegistryStatus,
        new_status: AssetRegistryStatus
    ):
        """Execute actions required for specific transitions."""
        if new_status == AssetRegistryStatus.FROZEN:
            self._freeze_all_assets()
        elif new_status == AssetRegistryStatus.TRUST_MANAGED:
            self._transfer_to_trust_management()
        elif new_status == AssetRegistryStatus.RESTORATION_PENDING:
            self._initiate_restoration_preparation()
        elif new_status == AssetRegistryStatus.RESTORED:
            self._complete_restoration()
        elif new_status == AssetRegistryStatus.ESTATE_DISTRIBUTED:
            self._initiate_estate_distribution()

    def _freeze_all_assets(self):
        """Freeze all assets upon legal death declaration."""
        for asset in self.assets.values():
            asset.freeze()

    def _transfer_to_trust_management(self):
        """Transfer assets to trust management."""
        for asset in self.assets.values():
            asset.set_status(AssetStatus.UNDER_MANAGEMENT)

    def _initiate_restoration_preparation(self):
        """Prepare assets for restoration to revived individual."""
        pass  # Detailed preparation logic

    def _complete_restoration(self):
        """Complete asset restoration to individual."""
        for asset in self.assets.values():
            asset.set_status(AssetStatus.TRANSFERRED)

    def _initiate_estate_distribution(self):
        """Begin estate distribution process."""
        for asset in self.assets.values():
            if asset.status != AssetStatus.DISPOSED:
                asset.set_status(AssetStatus.LIQUIDATING)

    def add_asset(self, asset: 'Asset'):
        """Add asset to registry."""
        self.assets[asset.asset_id] = asset

    def remove_asset(self, asset_id: str) -> Optional['Asset']:
        """Remove asset from registry."""
        return self.assets.pop(asset_id, None)

    def get_total_value(self, currency: str = "USD") -> MonetaryValue:
        """Calculate total value of all assets."""
        total = Decimal(0)
        for asset in self.assets.values():
            converted = self._convert_currency(
                asset.value, asset.value.currency, currency
            )
            total += converted

        return MonetaryValue(
            amount=total,
            currency=currency,
            as_of=datetime.utcnow()
        )

    def _convert_currency(
        self,
        value: MonetaryValue,
        from_currency: str,
        to_currency: str
    ) -> Decimal:
        """Convert currency using current exchange rates."""
        if from_currency == to_currency:
            return value.amount
        # Placeholder for actual exchange rate lookup
        rate = self._get_exchange_rate(from_currency, to_currency)
        return value.amount * rate

    def _get_exchange_rate(self, from_currency: str, to_currency: str) -> Decimal:
        """Get current exchange rate."""
        # Placeholder - would integrate with exchange rate service
        return Decimal("1.0")
```

## 3. Trust Management System

```python
class TrustType(Enum):
    PRESERVATION_TRUST = "PRESERVATION_TRUST"
    REVOCABLE_LIVING = "REVOCABLE_LIVING"
    IRREVOCABLE = "IRREVOCABLE"
    DYNASTY = "DYNASTY"
    CHARITABLE = "CHARITABLE"
    SPECIAL_NEEDS = "SPECIAL_NEEDS"
    ASSET_PROTECTION = "ASSET_PROTECTION"


class TrustStatus(Enum):
    ACTIVE = "ACTIVE"
    SUSPENDED = "SUSPENDED"
    TERMINATED = "TERMINATED"
    DISSOLVED = "DISSOLVED"


@dataclass
class TrusteeRecord:
    trustee_id: str
    name: str
    trustee_type: str  # INDIVIDUAL, CORPORATE, PROFESSIONAL
    role: str  # PRIMARY, SUCCESSOR, CO_TRUSTEE
    appointment_date: datetime
    powers: List[str]
    compensation_type: str
    compensation_rate: Decimal


@dataclass
class BeneficiaryRecord:
    beneficiary_id: str
    name: str
    relationship: str
    benefit_type: str  # INCOME, PRINCIPAL, BOTH, REMAINDER
    share: Decimal
    contingent: bool


@dataclass
class Distribution:
    distribution_id: str
    date: datetime
    beneficiary_id: str
    amount: MonetaryValue
    distribution_type: str
    reason: str
    approved_by: str


class TrustManagementSystem:
    """Manages preservation trusts for cryopreserved individuals."""

    def __init__(self, trust_id: str, trust_type: TrustType):
        self.trust_id = trust_id
        self.trust_type = trust_type
        self.status = TrustStatus.ACTIVE

        self.trustees: List[TrusteeRecord] = []
        self.beneficiaries: List[BeneficiaryRecord] = []
        self.assets: List[str] = []  # Asset IDs

        self.distributions: List[Distribution] = []

        self.preservation_reserve: MonetaryValue = MonetaryValue(
            amount=Decimal(0), currency="USD", as_of=datetime.utcnow()
        )

        self.investment_policy: Optional[Dict] = None

    def appoint_trustee(
        self,
        trustee: TrusteeRecord,
        appointing_authority: str
    ) -> bool:
        """Appoint a new trustee."""
        # Validate trustee qualifications
        if not self._validate_trustee_qualifications(trustee):
            raise ValueError("Trustee does not meet qualification requirements")

        # Check for conflicts
        if self._has_conflict_of_interest(trustee):
            raise ValueError("Trustee has conflict of interest")

        self.trustees.append(trustee)
        return True

    def _validate_trustee_qualifications(self, trustee: TrusteeRecord) -> bool:
        """Validate trustee meets requirements."""
        required_qualifications = {
            "CORPORATE": ["licensed", "bonded", "insured"],
            "PROFESSIONAL": ["certified", "bonded"],
            "INDIVIDUAL": ["adult", "competent"]
        }
        # Placeholder for actual validation
        return True

    def _has_conflict_of_interest(self, trustee: TrusteeRecord) -> bool:
        """Check for conflicts of interest."""
        # Check against existing trustees and beneficiaries
        for existing in self.trustees:
            if existing.trustee_id == trustee.trustee_id:
                return True
        return False

    def add_beneficiary(self, beneficiary: BeneficiaryRecord):
        """Add beneficiary to trust."""
        total_share = sum(b.share for b in self.beneficiaries if not b.contingent)
        if total_share + beneficiary.share > Decimal("1.0") and not beneficiary.contingent:
            raise ValueError("Total beneficiary shares exceed 100%")

        self.beneficiaries.append(beneficiary)

    def calculate_distribution(
        self,
        distribution_type: str,
        period_start: datetime,
        period_end: datetime
    ) -> List[Distribution]:
        """Calculate distributions for beneficiaries."""
        distributions = []

        if distribution_type == "INCOME":
            income = self._calculate_trust_income(period_start, period_end)
            for beneficiary in self.beneficiaries:
                if beneficiary.benefit_type in ["INCOME", "BOTH"]:
                    amount = income.amount * beneficiary.share
                    distributions.append(Distribution(
                        distribution_id=self._generate_distribution_id(),
                        date=datetime.utcnow(),
                        beneficiary_id=beneficiary.beneficiary_id,
                        amount=MonetaryValue(amount, income.currency, datetime.utcnow()),
                        distribution_type="INCOME",
                        reason=f"Income distribution for {period_start} to {period_end}",
                        approved_by=""
                    ))

        return distributions

    def _calculate_trust_income(
        self,
        period_start: datetime,
        period_end: datetime
    ) -> MonetaryValue:
        """Calculate trust income for period."""
        # Placeholder for actual income calculation
        return MonetaryValue(
            amount=Decimal("10000"),
            currency="USD",
            as_of=datetime.utcnow()
        )

    def _generate_distribution_id(self) -> str:
        return f"DST-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}"

    def approve_distribution(
        self,
        distribution_id: str,
        approver_id: str
    ) -> bool:
        """Approve a pending distribution."""
        for dist in self.distributions:
            if dist.distribution_id == distribution_id:
                dist.approved_by = approver_id
                return True
        return False

    def fund_preservation_reserve(self, amount: MonetaryValue):
        """Add funds to preservation reserve."""
        if amount.currency != self.preservation_reserve.currency:
            # Convert currency
            pass
        self.preservation_reserve.amount += amount.amount
        self.preservation_reserve.as_of = datetime.utcnow()

    def check_preservation_funding(
        self,
        minimum_required: MonetaryValue
    ) -> Tuple[bool, MonetaryValue]:
        """Check if preservation reserve meets minimum."""
        is_sufficient = self.preservation_reserve.amount >= minimum_required.amount
        shortfall = MonetaryValue(
            amount=max(Decimal(0), minimum_required.amount - self.preservation_reserve.amount),
            currency=minimum_required.currency,
            as_of=datetime.utcnow()
        )
        return is_sufficient, shortfall

    def set_investment_policy(self, policy: Dict):
        """Set investment policy for trust assets."""
        required_fields = ["objectives", "risk_tolerance", "asset_allocation", "constraints"]
        for field in required_fields:
            if field not in policy:
                raise ValueError(f"Investment policy missing required field: {field}")

        self.investment_policy = policy

    def terminate_trust(self, reason: str, authorized_by: str) -> bool:
        """Terminate the trust."""
        if self.status in [TrustStatus.TERMINATED, TrustStatus.DISSOLVED]:
            raise ValueError("Trust already terminated")

        self.status = TrustStatus.TERMINATED
        return True

    def dissolve_trust(
        self,
        distribution_plan: Dict[str, Decimal],
        authorized_by: str
    ) -> List[Distribution]:
        """Dissolve trust and distribute remaining assets."""
        if self.status != TrustStatus.TERMINATED:
            raise ValueError("Trust must be terminated before dissolution")

        distributions = []
        total_value = self._calculate_total_trust_value()

        for beneficiary_id, share in distribution_plan.items():
            amount = total_value.amount * share
            distributions.append(Distribution(
                distribution_id=self._generate_distribution_id(),
                date=datetime.utcnow(),
                beneficiary_id=beneficiary_id,
                amount=MonetaryValue(amount, total_value.currency, datetime.utcnow()),
                distribution_type="PRINCIPAL",
                reason="Trust dissolution distribution",
                approved_by=authorized_by
            ))

        self.status = TrustStatus.DISSOLVED
        return distributions

    def _calculate_total_trust_value(self) -> MonetaryValue:
        """Calculate total value of trust assets."""
        # Placeholder
        return MonetaryValue(
            amount=Decimal("1000000"),
            currency="USD",
            as_of=datetime.utcnow()
        )
```

## 4. Investment Strategy Engine

```python
class RiskTolerance(Enum):
    CONSERVATIVE = "CONSERVATIVE"
    MODERATE = "MODERATE"
    AGGRESSIVE = "AGGRESSIVE"


@dataclass
class AssetAllocation:
    asset_class: str
    target_percentage: Decimal
    current_percentage: Decimal
    min_percentage: Decimal
    max_percentage: Decimal


@dataclass
class RebalancingAction:
    action_id: str
    asset_class: str
    action_type: str  # BUY, SELL
    amount: MonetaryValue
    reason: str


class InvestmentStrategyEngine:
    """Manages long-term investment strategy for preservation trusts."""

    # Default allocations by risk tolerance
    DEFAULT_ALLOCATIONS = {
        RiskTolerance.CONSERVATIVE: {
            "FIXED_INCOME": Decimal("0.60"),
            "EQUITIES": Decimal("0.20"),
            "ALTERNATIVES": Decimal("0.10"),
            "CASH": Decimal("0.10")
        },
        RiskTolerance.MODERATE: {
            "FIXED_INCOME": Decimal("0.40"),
            "EQUITIES": Decimal("0.40"),
            "ALTERNATIVES": Decimal("0.15"),
            "CASH": Decimal("0.05")
        },
        RiskTolerance.AGGRESSIVE: {
            "FIXED_INCOME": Decimal("0.20"),
            "EQUITIES": Decimal("0.55"),
            "ALTERNATIVES": Decimal("0.20"),
            "CASH": Decimal("0.05")
        }
    }

    def __init__(
        self,
        trust_id: str,
        risk_tolerance: RiskTolerance,
        time_horizon_years: int
    ):
        self.trust_id = trust_id
        self.risk_tolerance = risk_tolerance
        self.time_horizon_years = time_horizon_years

        self.target_allocation = self._build_target_allocation()
        self.current_holdings: Dict[str, MonetaryValue] = {}
        self.rebalancing_threshold = Decimal("0.05")  # 5% drift

    def _build_target_allocation(self) -> List[AssetAllocation]:
        """Build target allocation based on risk tolerance."""
        allocations = []
        default = self.DEFAULT_ALLOCATIONS[self.risk_tolerance]

        for asset_class, target in default.items():
            allocations.append(AssetAllocation(
                asset_class=asset_class,
                target_percentage=target,
                current_percentage=Decimal("0"),
                min_percentage=target - Decimal("0.10"),
                max_percentage=target + Decimal("0.10")
            ))

        return allocations

    def analyze_drift(self) -> Dict[str, Decimal]:
        """Analyze allocation drift from targets."""
        total_value = sum(h.amount for h in self.current_holdings.values())
        if total_value == 0:
            return {}

        drift = {}
        for allocation in self.target_allocation:
            current = self.current_holdings.get(
                allocation.asset_class,
                MonetaryValue(Decimal(0), "USD", datetime.utcnow())
            )
            current_pct = current.amount / total_value if total_value > 0 else Decimal(0)
            allocation.current_percentage = current_pct
            drift[allocation.asset_class] = current_pct - allocation.target_percentage

        return drift

    def needs_rebalancing(self) -> bool:
        """Check if portfolio needs rebalancing."""
        drift = self.analyze_drift()
        return any(abs(d) > self.rebalancing_threshold for d in drift.values())

    def generate_rebalancing_plan(self) -> List[RebalancingAction]:
        """Generate rebalancing actions."""
        actions = []
        drift = self.analyze_drift()
        total_value = sum(h.amount for h in self.current_holdings.values())

        for asset_class, drift_pct in drift.items():
            if abs(drift_pct) > self.rebalancing_threshold:
                amount = abs(drift_pct) * total_value
                action_type = "SELL" if drift_pct > 0 else "BUY"

                actions.append(RebalancingAction(
                    action_id=f"REB-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}",
                    asset_class=asset_class,
                    action_type=action_type,
                    amount=MonetaryValue(amount, "USD", datetime.utcnow()),
                    reason=f"Rebalance: {drift_pct:.2%} drift from target"
                ))

        return actions

    def project_growth(
        self,
        initial_value: MonetaryValue,
        years: int,
        inflation_rate: Decimal = Decimal("0.025")
    ) -> List[Dict]:
        """Project portfolio growth over time."""
        # Expected returns by risk tolerance
        expected_returns = {
            RiskTolerance.CONSERVATIVE: Decimal("0.04"),
            RiskTolerance.MODERATE: Decimal("0.06"),
            RiskTolerance.AGGRESSIVE: Decimal("0.08")
        }

        annual_return = expected_returns[self.risk_tolerance]
        real_return = annual_return - inflation_rate

        projections = []
        value = initial_value.amount

        for year in range(1, years + 1):
            value = value * (1 + real_return)
            projections.append({
                "year": year,
                "nominal_value": value * ((1 + inflation_rate) ** year),
                "real_value": value,
                "inflation_adjusted": value
            })

        return projections

    def calculate_sustainable_withdrawal(
        self,
        portfolio_value: MonetaryValue,
        preservation_years: int
    ) -> MonetaryValue:
        """Calculate sustainable annual withdrawal rate."""
        # Conservative 3% rule for long-term preservation
        base_rate = Decimal("0.03")

        # Adjust for time horizon
        if preservation_years > 100:
            rate = Decimal("0.02")
        elif preservation_years > 50:
            rate = Decimal("0.025")
        else:
            rate = base_rate

        annual_amount = portfolio_value.amount * rate

        return MonetaryValue(
            amount=annual_amount,
            currency=portfolio_value.currency,
            as_of=datetime.utcnow()
        )

    def evaluate_preservation_sustainability(
        self,
        portfolio_value: MonetaryValue,
        annual_costs: MonetaryValue,
        target_years: int
    ) -> Dict:
        """Evaluate if portfolio can sustain preservation costs."""
        projections = self.project_growth(portfolio_value, target_years)

        # Calculate year when funds would be exhausted
        remaining = portfolio_value.amount
        exhaustion_year = None

        for year in range(1, target_years + 1):
            expected_return = remaining * Decimal("0.04")  # Conservative estimate
            remaining = remaining + expected_return - annual_costs.amount

            if remaining <= 0:
                exhaustion_year = year
                break

        is_sustainable = exhaustion_year is None

        return {
            "is_sustainable": is_sustainable,
            "exhaustion_year": exhaustion_year,
            "final_value": remaining if is_sustainable else Decimal(0),
            "funding_gap": annual_costs.amount - self.calculate_sustainable_withdrawal(
                portfolio_value, target_years
            ).amount if not is_sustainable else Decimal(0)
        }
```

## 5. Asset Valuation Engine

```python
class ValuationMethod(Enum):
    MARKET_VALUE = "MARKET_VALUE"
    APPRAISAL = "APPRAISAL"
    BOOK_VALUE = "BOOK_VALUE"
    COST_BASIS = "COST_BASIS"
    DISCOUNTED_CASH_FLOW = "DISCOUNTED_CASH_FLOW"
    COMPARABLE_SALES = "COMPARABLE_SALES"
    ALGORITHMIC = "ALGORITHMIC"


@dataclass
class ValuationResult:
    valuation_id: str
    asset_id: str
    method: ValuationMethod
    value: MonetaryValue
    confidence: Decimal  # 0-1
    valid_until: datetime
    performed_by: str
    supporting_data: Dict


class AssetValuationEngine:
    """Handles asset valuation for various asset types."""

    def __init__(self):
        self.valuations: Dict[str, List[ValuationResult]] = {}
        self.valuation_frequency: Dict[str, timedelta] = {
            "REAL_PROPERTY": timedelta(days=365),
            "FINANCIAL_ACCOUNT": timedelta(days=1),
            "INVESTMENT_PORTFOLIO": timedelta(days=1),
            "CRYPTOCURRENCY": timedelta(hours=1),
            "INTELLECTUAL_PROPERTY": timedelta(days=365),
            "BUSINESS_INTEREST": timedelta(days=180),
            "PERSONAL_PROPERTY": timedelta(days=730)
        }

    def value_asset(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict,
        method: Optional[ValuationMethod] = None
    ) -> ValuationResult:
        """Value an asset using appropriate method."""
        if method is None:
            method = self._select_valuation_method(asset_type)

        valuation_func = {
            ValuationMethod.MARKET_VALUE: self._market_value,
            ValuationMethod.APPRAISAL: self._appraisal_value,
            ValuationMethod.BOOK_VALUE: self._book_value,
            ValuationMethod.DISCOUNTED_CASH_FLOW: self._dcf_value,
            ValuationMethod.COMPARABLE_SALES: self._comparable_sales,
            ValuationMethod.ALGORITHMIC: self._algorithmic_value
        }

        result = valuation_func[method](asset_id, asset_type, asset_data)

        if asset_id not in self.valuations:
            self.valuations[asset_id] = []
        self.valuations[asset_id].append(result)

        return result

    def _select_valuation_method(self, asset_type: str) -> ValuationMethod:
        """Select appropriate valuation method for asset type."""
        method_mapping = {
            "REAL_PROPERTY": ValuationMethod.COMPARABLE_SALES,
            "FINANCIAL_ACCOUNT": ValuationMethod.BOOK_VALUE,
            "INVESTMENT_PORTFOLIO": ValuationMethod.MARKET_VALUE,
            "CRYPTOCURRENCY": ValuationMethod.MARKET_VALUE,
            "INTELLECTUAL_PROPERTY": ValuationMethod.DISCOUNTED_CASH_FLOW,
            "BUSINESS_INTEREST": ValuationMethod.DISCOUNTED_CASH_FLOW,
            "PERSONAL_PROPERTY": ValuationMethod.APPRAISAL
        }
        return method_mapping.get(asset_type, ValuationMethod.BOOK_VALUE)

    def _market_value(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Calculate market value for tradeable assets."""
        # Get current market price
        if asset_type == "INVESTMENT_PORTFOLIO":
            total_value = Decimal(0)
            for holding in asset_data.get("holdings", []):
                quantity = Decimal(str(holding.get("quantity", 0)))
                price = Decimal(str(holding.get("current_price", 0)))
                total_value += quantity * price

            return ValuationResult(
                valuation_id=self._generate_valuation_id(),
                asset_id=asset_id,
                method=ValuationMethod.MARKET_VALUE,
                value=MonetaryValue(total_value, "USD", datetime.utcnow()),
                confidence=Decimal("0.99"),
                valid_until=datetime.utcnow() + timedelta(days=1),
                performed_by="SYSTEM",
                supporting_data={"holdings_count": len(asset_data.get("holdings", []))}
            )

        elif asset_type == "CRYPTOCURRENCY":
            balance = Decimal(str(asset_data.get("balance", 0)))
            price = self._get_crypto_price(asset_data.get("symbol", ""))

            return ValuationResult(
                valuation_id=self._generate_valuation_id(),
                asset_id=asset_id,
                method=ValuationMethod.MARKET_VALUE,
                value=MonetaryValue(balance * price, "USD", datetime.utcnow()),
                confidence=Decimal("0.95"),
                valid_until=datetime.utcnow() + timedelta(hours=1),
                performed_by="SYSTEM",
                supporting_data={"symbol": asset_data.get("symbol"), "price": str(price)}
            )

        return self._default_valuation(asset_id)

    def _appraisal_value(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Use appraisal-based valuation."""
        appraisal = asset_data.get("last_appraisal", {})
        value = Decimal(str(appraisal.get("value", 0)))
        appraisal_date = appraisal.get("date", datetime.utcnow())

        # Adjust for age of appraisal
        age_days = (datetime.utcnow() - appraisal_date).days if isinstance(appraisal_date, datetime) else 0
        confidence = max(Decimal("0.5"), Decimal("0.95") - (age_days / 365 * Decimal("0.1")))

        return ValuationResult(
            valuation_id=self._generate_valuation_id(),
            asset_id=asset_id,
            method=ValuationMethod.APPRAISAL,
            value=MonetaryValue(value, "USD", datetime.utcnow()),
            confidence=confidence,
            valid_until=datetime.utcnow() + timedelta(days=180),
            performed_by=appraisal.get("appraiser", "UNKNOWN"),
            supporting_data={"appraisal_date": str(appraisal_date)}
        )

    def _book_value(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Calculate book value."""
        balance = Decimal(str(asset_data.get("balance", 0)))

        return ValuationResult(
            valuation_id=self._generate_valuation_id(),
            asset_id=asset_id,
            method=ValuationMethod.BOOK_VALUE,
            value=MonetaryValue(balance, asset_data.get("currency", "USD"), datetime.utcnow()),
            confidence=Decimal("1.0"),
            valid_until=datetime.utcnow() + timedelta(days=1),
            performed_by="SYSTEM",
            supporting_data={}
        )

    def _dcf_value(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Calculate discounted cash flow value."""
        cash_flows = asset_data.get("projected_cash_flows", [])
        discount_rate = Decimal(str(asset_data.get("discount_rate", "0.10")))
        terminal_growth = Decimal(str(asset_data.get("terminal_growth", "0.02")))

        # Calculate NPV of cash flows
        npv = Decimal(0)
        for i, cf in enumerate(cash_flows):
            cf_value = Decimal(str(cf))
            npv += cf_value / ((1 + discount_rate) ** (i + 1))

        # Terminal value
        if cash_flows:
            terminal_cf = Decimal(str(cash_flows[-1])) * (1 + terminal_growth)
            terminal_value = terminal_cf / (discount_rate - terminal_growth)
            npv += terminal_value / ((1 + discount_rate) ** len(cash_flows))

        return ValuationResult(
            valuation_id=self._generate_valuation_id(),
            asset_id=asset_id,
            method=ValuationMethod.DISCOUNTED_CASH_FLOW,
            value=MonetaryValue(npv, "USD", datetime.utcnow()),
            confidence=Decimal("0.75"),
            valid_until=datetime.utcnow() + timedelta(days=90),
            performed_by="SYSTEM",
            supporting_data={
                "discount_rate": str(discount_rate),
                "terminal_growth": str(terminal_growth),
                "projection_years": len(cash_flows)
            }
        )

    def _comparable_sales(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Value using comparable sales method."""
        comparables = asset_data.get("comparables", [])
        if not comparables:
            return self._default_valuation(asset_id)

        # Calculate adjusted average
        adjusted_values = []
        for comp in comparables:
            base_value = Decimal(str(comp.get("sale_price", 0)))
            adjustments = comp.get("adjustments", {})

            adjusted = base_value
            for adj_name, adj_value in adjustments.items():
                adjusted += Decimal(str(adj_value))

            adjusted_values.append(adjusted)

        avg_value = sum(adjusted_values) / len(adjusted_values)

        return ValuationResult(
            valuation_id=self._generate_valuation_id(),
            asset_id=asset_id,
            method=ValuationMethod.COMPARABLE_SALES,
            value=MonetaryValue(avg_value, "USD", datetime.utcnow()),
            confidence=Decimal("0.85"),
            valid_until=datetime.utcnow() + timedelta(days=180),
            performed_by="SYSTEM",
            supporting_data={"comparables_used": len(comparables)}
        )

    def _algorithmic_value(
        self,
        asset_id: str,
        asset_type: str,
        asset_data: Dict
    ) -> ValuationResult:
        """Use algorithmic/ML-based valuation."""
        # Placeholder for ML model integration
        return self._default_valuation(asset_id)

    def _default_valuation(self, asset_id: str) -> ValuationResult:
        """Return default valuation when specific method unavailable."""
        return ValuationResult(
            valuation_id=self._generate_valuation_id(),
            asset_id=asset_id,
            method=ValuationMethod.BOOK_VALUE,
            value=MonetaryValue(Decimal(0), "USD", datetime.utcnow()),
            confidence=Decimal("0.0"),
            valid_until=datetime.utcnow(),
            performed_by="SYSTEM",
            supporting_data={"error": "Unable to value asset"}
        )

    def _get_crypto_price(self, symbol: str) -> Decimal:
        """Get current cryptocurrency price."""
        # Placeholder - would integrate with price feed
        return Decimal("50000") if symbol == "BTC" else Decimal("3000")

    def _generate_valuation_id(self) -> str:
        return f"VAL-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}"

    def get_latest_valuation(self, asset_id: str) -> Optional[ValuationResult]:
        """Get most recent valuation for asset."""
        valuations = self.valuations.get(asset_id, [])
        if not valuations:
            return None
        return max(valuations, key=lambda v: v.value.as_of)
```

## 6. Asset Restoration Engine

```python
@dataclass
class RestorationTask:
    task_id: str
    asset_id: str
    task_type: str
    status: str  # PENDING, IN_PROGRESS, COMPLETED, FAILED
    description: str
    assigned_to: str
    due_date: datetime
    completed_date: Optional[datetime] = None


@dataclass
class RestorationPlan:
    plan_id: str
    registry_id: str
    legal_restoration_id: str
    status: str
    tasks: List[RestorationTask]
    estimated_completion: datetime
    created_at: datetime


class AssetRestorationEngine:
    """Manages asset restoration upon individual revival."""

    def __init__(self, registry_id: str):
        self.registry_id = registry_id
        self.restoration_plans: List[RestorationPlan] = []

    def initiate_restoration(
        self,
        legal_restoration_id: str,
        registry: 'AssetRegistryManager'
    ) -> RestorationPlan:
        """Initiate asset restoration process."""
        tasks = []

        # Generate tasks for each asset type
        for asset_id, asset in registry.assets.items():
            asset_tasks = self._generate_asset_tasks(asset)
            tasks.extend(asset_tasks)

        # Add trust dissolution tasks
        for trust_id, trust in registry.trusts.items():
            trust_tasks = self._generate_trust_tasks(trust)
            tasks.extend(trust_tasks)

        # Add administrative tasks
        admin_tasks = self._generate_admin_tasks()
        tasks.extend(admin_tasks)

        plan = RestorationPlan(
            plan_id=f"RST-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}",
            registry_id=self.registry_id,
            legal_restoration_id=legal_restoration_id,
            status="INITIATED",
            tasks=tasks,
            estimated_completion=self._estimate_completion(tasks),
            created_at=datetime.utcnow()
        )

        self.restoration_plans.append(plan)
        return plan

    def _generate_asset_tasks(self, asset) -> List[RestorationTask]:
        """Generate restoration tasks for an asset."""
        tasks = []
        asset_id = asset.asset_id
        asset_type = asset.type

        base_tasks = [
            ("VERIFY_OWNERSHIP", "Verify current ownership status"),
            ("CLEAR_ENCUMBRANCES", "Clear any temporary encumbrances"),
            ("UPDATE_RECORDS", "Update ownership records"),
            ("TRANSFER_TITLE", "Transfer title to restored individual")
        ]

        for task_type, description in base_tasks:
            tasks.append(RestorationTask(
                task_id=f"TSK-{asset_id}-{task_type}",
                asset_id=asset_id,
                task_type=task_type,
                status="PENDING",
                description=f"{description} for {asset_type}",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=30)
            ))

        # Asset-type specific tasks
        if asset_type == "REAL_PROPERTY":
            tasks.append(RestorationTask(
                task_id=f"TSK-{asset_id}-DEED_TRANSFER",
                asset_id=asset_id,
                task_type="DEED_TRANSFER",
                status="PENDING",
                description="Execute deed transfer",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=60)
            ))

        elif asset_type == "FINANCIAL_ACCOUNT":
            tasks.append(RestorationTask(
                task_id=f"TSK-{asset_id}-ACCOUNT_TRANSFER",
                asset_id=asset_id,
                task_type="ACCOUNT_TRANSFER",
                status="PENDING",
                description="Transfer account ownership",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=14)
            ))

        elif asset_type == "CRYPTOCURRENCY":
            tasks.append(RestorationTask(
                task_id=f"TSK-{asset_id}-WALLET_TRANSFER",
                asset_id=asset_id,
                task_type="WALLET_TRANSFER",
                status="PENDING",
                description="Transfer wallet access/keys",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=7)
            ))

        return tasks

    def _generate_trust_tasks(self, trust) -> List[RestorationTask]:
        """Generate tasks for trust dissolution."""
        trust_id = trust.trust_id

        return [
            RestorationTask(
                task_id=f"TSK-{trust_id}-TERMINATE",
                asset_id=trust_id,
                task_type="TRUST_TERMINATE",
                status="PENDING",
                description="Terminate preservation trust",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=30)
            ),
            RestorationTask(
                task_id=f"TSK-{trust_id}-DISTRIBUTE",
                asset_id=trust_id,
                task_type="TRUST_DISTRIBUTE",
                status="PENDING",
                description="Distribute trust assets to restored individual",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=45)
            ),
            RestorationTask(
                task_id=f"TSK-{trust_id}-DISSOLVE",
                asset_id=trust_id,
                task_type="TRUST_DISSOLVE",
                status="PENDING",
                description="Dissolve trust entity",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=60)
            )
        ]

    def _generate_admin_tasks(self) -> List[RestorationTask]:
        """Generate administrative tasks."""
        return [
            RestorationTask(
                task_id=f"TSK-ADMIN-TAX_REVIEW",
                asset_id="ADMIN",
                task_type="TAX_REVIEW",
                status="PENDING",
                description="Review tax implications of restoration",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=30)
            ),
            RestorationTask(
                task_id=f"TSK-ADMIN-INSURANCE_UPDATE",
                asset_id="ADMIN",
                task_type="INSURANCE_UPDATE",
                status="PENDING",
                description="Update insurance policies",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=14)
            ),
            RestorationTask(
                task_id=f"TSK-ADMIN-FINAL_ACCOUNTING",
                asset_id="ADMIN",
                task_type="FINAL_ACCOUNTING",
                status="PENDING",
                description="Prepare final accounting report",
                assigned_to="",
                due_date=datetime.utcnow() + timedelta(days=90)
            )
        ]

    def _estimate_completion(self, tasks: List[RestorationTask]) -> datetime:
        """Estimate completion date based on tasks."""
        if not tasks:
            return datetime.utcnow()
        return max(task.due_date for task in tasks)

    def complete_task(
        self,
        plan_id: str,
        task_id: str,
        completed_by: str
    ) -> bool:
        """Mark a task as completed."""
        for plan in self.restoration_plans:
            if plan.plan_id == plan_id:
                for task in plan.tasks:
                    if task.task_id == task_id:
                        task.status = "COMPLETED"
                        task.completed_date = datetime.utcnow()
                        return True
        return False

    def get_progress(self, plan_id: str) -> Dict:
        """Get restoration progress."""
        for plan in self.restoration_plans:
            if plan.plan_id == plan_id:
                total = len(plan.tasks)
                completed = sum(1 for t in plan.tasks if t.status == "COMPLETED")
                in_progress = sum(1 for t in plan.tasks if t.status == "IN_PROGRESS")
                pending = sum(1 for t in plan.tasks if t.status == "PENDING")
                failed = sum(1 for t in plan.tasks if t.status == "FAILED")

                return {
                    "total_tasks": total,
                    "completed": completed,
                    "in_progress": in_progress,
                    "pending": pending,
                    "failed": failed,
                    "completion_percentage": (completed / total * 100) if total > 0 else 0
                }
        return {}

    def finalize_restoration(self, plan_id: str) -> bool:
        """Finalize restoration after all tasks complete."""
        for plan in self.restoration_plans:
            if plan.plan_id == plan_id:
                incomplete = [t for t in plan.tasks if t.status != "COMPLETED"]
                if incomplete:
                    return False
                plan.status = "COMPLETED"
                return True
        return False
```

## 7. Estate Distribution Engine

```python
@dataclass
class Heir:
    heir_id: str
    name: str
    relationship: str
    share: Decimal
    priority: int
    contact_info: Dict


@dataclass
class DistributionRecord:
    record_id: str
    heir_id: str
    asset_id: str
    distribution_type: str  # IN_KIND, CASH
    value: MonetaryValue
    date: datetime
    status: str


class EstateDistributionEngine:
    """Manages estate distribution upon permanent death."""

    def __init__(self, registry_id: str):
        self.registry_id = registry_id
        self.heirs: List[Heir] = []
        self.distributions: List[DistributionRecord] = []
        self.will_provisions: Optional[Dict] = None

    def load_will_provisions(self, will_document: Dict):
        """Load provisions from will document."""
        self.will_provisions = will_document
        self._extract_heirs_from_will()

    def _extract_heirs_from_will(self):
        """Extract heir information from will."""
        if not self.will_provisions:
            return

        beneficiaries = self.will_provisions.get("beneficiaries", [])
        for i, ben in enumerate(beneficiaries):
            self.heirs.append(Heir(
                heir_id=ben.get("id", f"HEIR-{i}"),
                name=ben.get("name", ""),
                relationship=ben.get("relationship", ""),
                share=Decimal(str(ben.get("share", 0))),
                priority=ben.get("priority", i),
                contact_info=ben.get("contact", {})
            ))

    def calculate_distribution_plan(
        self,
        assets: Dict[str, 'Asset'],
        debts: List[Dict],
        expenses: List[Dict]
    ) -> Dict:
        """Calculate distribution plan for estate."""
        # Calculate gross estate
        gross_value = sum(a.value.amount for a in assets.values())

        # Subtract debts
        total_debts = sum(Decimal(str(d.get("amount", 0))) for d in debts)

        # Subtract expenses
        total_expenses = sum(Decimal(str(e.get("amount", 0))) for e in expenses)

        # Net estate
        net_estate = gross_value - total_debts - total_expenses

        # Calculate each heir's share
        distribution_plan = {
            "gross_estate": gross_value,
            "total_debts": total_debts,
            "total_expenses": total_expenses,
            "net_estate": net_estate,
            "heir_distributions": []
        }

        for heir in sorted(self.heirs, key=lambda h: h.priority):
            heir_amount = net_estate * heir.share
            distribution_plan["heir_distributions"].append({
                "heir_id": heir.heir_id,
                "heir_name": heir.name,
                "share_percentage": float(heir.share * 100),
                "distribution_amount": float(heir_amount)
            })

        return distribution_plan

    def execute_distribution(
        self,
        distribution_plan: Dict,
        assets: Dict[str, 'Asset']
    ) -> List[DistributionRecord]:
        """Execute the distribution plan."""
        records = []

        for heir_dist in distribution_plan["heir_distributions"]:
            heir_id = heir_dist["heir_id"]
            amount = Decimal(str(heir_dist["distribution_amount"]))

            # Distribute assets
            record = DistributionRecord(
                record_id=f"DST-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}-{heir_id}",
                heir_id=heir_id,
                asset_id="CASH",
                distribution_type="CASH",
                value=MonetaryValue(amount, "USD", datetime.utcnow()),
                date=datetime.utcnow(),
                status="PENDING"
            )
            records.append(record)
            self.distributions.append(record)

        return records

    def complete_distribution(self, record_id: str) -> bool:
        """Mark distribution as completed."""
        for dist in self.distributions:
            if dist.record_id == record_id:
                dist.status = "COMPLETED"
                return True
        return False
```
