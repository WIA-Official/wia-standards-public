# CRYO-REVIVAL Phase 2: Algorithm Specification

## Overview

This document defines the algorithms for revival stage management, warming control, cardiovascular restart, neurological restoration, and re-preservation decision making.

## Core Algorithms

### Revival Stage Manager

```python
from enum import Enum
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
import logging

class RevivalStage(Enum):
    PRE_REVIVAL = "PRE_REVIVAL"
    WARMING = "WARMING"
    CRYOPROTECTANT_REMOVAL = "CRYOPROTECTANT_REMOVAL"
    REHYDRATION = "REHYDRATION"
    CARDIOVASCULAR_RESTART = "CARDIOVASCULAR_RESTART"
    RESPIRATORY_ACTIVATION = "RESPIRATORY_ACTIVATION"
    NEUROLOGICAL_RESTORATION = "NEUROLOGICAL_RESTORATION"
    STABILIZATION = "STABILIZATION"
    POST_REVIVAL_CARE = "POST_REVIVAL_CARE"

class StageStatus(Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    SKIPPED = "SKIPPED"

@dataclass
class StageTransitionCriteria:
    stage: RevivalStage
    required_vitals: Dict[str, tuple]  # parameter -> (min, max)
    required_labs: Dict[str, tuple]
    minimum_duration: timedelta
    maximum_duration: timedelta
    required_approvals: List[str]

@dataclass
class StageRecord:
    stage: RevivalStage
    status: StageStatus
    started_at: Optional[datetime]
    completed_at: Optional[datetime]
    parameters: Dict[str, Any]
    complications: List[str]
    interventions: List[str]

class RevivalStageManager:
    """Manages revival procedure stages and transitions."""

    STAGE_ORDER = [
        RevivalStage.PRE_REVIVAL,
        RevivalStage.WARMING,
        RevivalStage.CRYOPROTECTANT_REMOVAL,
        RevivalStage.REHYDRATION,
        RevivalStage.CARDIOVASCULAR_RESTART,
        RevivalStage.RESPIRATORY_ACTIVATION,
        RevivalStage.NEUROLOGICAL_RESTORATION,
        RevivalStage.STABILIZATION,
        RevivalStage.POST_REVIVAL_CARE
    ]

    TRANSITION_CRITERIA = {
        RevivalStage.PRE_REVIVAL: StageTransitionCriteria(
            stage=RevivalStage.PRE_REVIVAL,
            required_vitals={},
            required_labs={},
            minimum_duration=timedelta(hours=2),
            maximum_duration=timedelta(hours=24),
            required_approvals=["REVIVAL_DIRECTOR", "MEDICAL_DIRECTOR"]
        ),
        RevivalStage.WARMING: StageTransitionCriteria(
            stage=RevivalStage.WARMING,
            required_vitals={"temperature": (20.0, 37.0)},
            required_labs={},
            minimum_duration=timedelta(hours=4),
            maximum_duration=timedelta(hours=12),
            required_approvals=["CRYONICS_SPECIALIST"]
        ),
        RevivalStage.CRYOPROTECTANT_REMOVAL: StageTransitionCriteria(
            stage=RevivalStage.CRYOPROTECTANT_REMOVAL,
            required_vitals={"temperature": (35.0, 37.5)},
            required_labs={"cryoprotectant_level": (0, 0.5)},
            minimum_duration=timedelta(hours=2),
            maximum_duration=timedelta(hours=8),
            required_approvals=["PERFUSIONIST"]
        ),
        RevivalStage.CARDIOVASCULAR_RESTART: StageTransitionCriteria(
            stage=RevivalStage.CARDIOVASCULAR_RESTART,
            required_vitals={
                "heart_rate": (60, 100),
                "blood_pressure_systolic": (90, 140),
                "blood_pressure_diastolic": (60, 90)
            },
            required_labs={"lactate": (0, 2.0)},
            minimum_duration=timedelta(minutes=30),
            maximum_duration=timedelta(hours=4),
            required_approvals=["CARDIOVASCULAR_SURGEON"]
        ),
        RevivalStage.NEUROLOGICAL_RESTORATION: StageTransitionCriteria(
            stage=RevivalStage.NEUROLOGICAL_RESTORATION,
            required_vitals={
                "heart_rate": (60, 100),
                "oxygen_saturation": (94, 100)
            },
            required_labs={"glucose": (70, 140)},
            minimum_duration=timedelta(hours=6),
            maximum_duration=timedelta(hours=72),
            required_approvals=["NEUROLOGIST"]
        )
    }

    def __init__(self, procedure_id: str):
        self.procedure_id = procedure_id
        self.current_stage = RevivalStage.PRE_REVIVAL
        self.stages: List[StageRecord] = []
        self.logger = logging.getLogger(f"RevivalStageManager:{procedure_id}")

    def initialize_procedure(self) -> StageRecord:
        """Initialize the revival procedure at PRE_REVIVAL stage."""
        record = StageRecord(
            stage=RevivalStage.PRE_REVIVAL,
            status=StageStatus.IN_PROGRESS,
            started_at=datetime.utcnow(),
            completed_at=None,
            parameters={},
            complications=[],
            interventions=[]
        )
        self.stages.append(record)
        self.logger.info(f"Procedure initialized at {record.started_at}")
        return record

    def can_transition(
        self,
        vitals: Dict[str, float],
        labs: Dict[str, float],
        approvals: List[str]
    ) -> tuple[bool, List[str]]:
        """Check if transition to next stage is possible."""
        criteria = self.TRANSITION_CRITERIA.get(self.current_stage)
        if not criteria:
            return True, []

        issues = []
        current_record = self._get_current_record()

        # Check minimum duration
        if current_record and current_record.started_at:
            elapsed = datetime.utcnow() - current_record.started_at
            if elapsed < criteria.minimum_duration:
                remaining = criteria.minimum_duration - elapsed
                issues.append(f"Minimum duration not met: {remaining} remaining")

        # Check vital signs
        for vital, (min_val, max_val) in criteria.required_vitals.items():
            if vital not in vitals:
                issues.append(f"Missing vital: {vital}")
            elif not min_val <= vitals[vital] <= max_val:
                issues.append(
                    f"{vital} out of range: {vitals[vital]} "
                    f"(required: {min_val}-{max_val})"
                )

        # Check lab results
        for lab, (min_val, max_val) in criteria.required_labs.items():
            if lab not in labs:
                issues.append(f"Missing lab: {lab}")
            elif not min_val <= labs[lab] <= max_val:
                issues.append(
                    f"{lab} out of range: {labs[lab]} "
                    f"(required: {min_val}-{max_val})"
                )

        # Check approvals
        missing_approvals = set(criteria.required_approvals) - set(approvals)
        if missing_approvals:
            issues.append(f"Missing approvals: {missing_approvals}")

        return len(issues) == 0, issues

    def transition_to_next_stage(
        self,
        vitals: Dict[str, float],
        labs: Dict[str, float],
        approvals: List[str]
    ) -> tuple[bool, Optional[StageRecord], List[str]]:
        """Attempt to transition to the next stage."""
        can_transition, issues = self.can_transition(vitals, labs, approvals)

        if not can_transition:
            return False, None, issues

        # Complete current stage
        current_record = self._get_current_record()
        if current_record:
            current_record.status = StageStatus.COMPLETED
            current_record.completed_at = datetime.utcnow()

        # Get next stage
        current_idx = self.STAGE_ORDER.index(self.current_stage)
        if current_idx >= len(self.STAGE_ORDER) - 1:
            return True, current_record, ["Procedure complete"]

        next_stage = self.STAGE_ORDER[current_idx + 1]
        self.current_stage = next_stage

        # Create new stage record
        new_record = StageRecord(
            stage=next_stage,
            status=StageStatus.IN_PROGRESS,
            started_at=datetime.utcnow(),
            completed_at=None,
            parameters={},
            complications=[],
            interventions=[]
        )
        self.stages.append(new_record)

        self.logger.info(f"Transitioned to {next_stage.value}")
        return True, new_record, []

    def record_complication(
        self,
        complication_type: str,
        severity: str,
        description: str
    ) -> None:
        """Record a complication in the current stage."""
        current_record = self._get_current_record()
        if current_record:
            complication = f"{severity}:{complication_type}:{description}"
            current_record.complications.append(complication)
            self.logger.warning(f"Complication recorded: {complication}")

    def record_intervention(
        self,
        intervention_type: str,
        description: str,
        performed_by: str
    ) -> None:
        """Record an intervention in the current stage."""
        current_record = self._get_current_record()
        if current_record:
            intervention = f"{intervention_type}:{description}:{performed_by}"
            current_record.interventions.append(intervention)
            self.logger.info(f"Intervention recorded: {intervention}")

    def _get_current_record(self) -> Optional[StageRecord]:
        """Get the current stage record."""
        for record in reversed(self.stages):
            if record.status == StageStatus.IN_PROGRESS:
                return record
        return None

    def get_stage_summary(self) -> Dict[str, Any]:
        """Get summary of all stages."""
        return {
            "procedure_id": self.procedure_id,
            "current_stage": self.current_stage.value,
            "stages": [
                {
                    "stage": r.stage.value,
                    "status": r.status.value,
                    "started_at": r.started_at.isoformat() if r.started_at else None,
                    "completed_at": r.completed_at.isoformat() if r.completed_at else None,
                    "complications_count": len(r.complications),
                    "interventions_count": len(r.interventions)
                }
                for r in self.stages
            ]
        }
```

### Warming Protocol Controller

```python
from dataclasses import dataclass
from typing import List, Optional, Callable
from enum import Enum
import math

class WarmingMethod(Enum):
    PERFUSION = "PERFUSION"
    EXTERNAL = "EXTERNAL"
    HYBRID = "HYBRID"

@dataclass
class TemperaturePoint:
    timestamp: datetime
    temperature: float
    location: str  # CORE, BRAIN, PERIPHERAL
    source: str

@dataclass
class WarmingParameters:
    initial_temperature: float  # Starting temp (typically -196C or -130C)
    target_temperature: float   # Target temp (typically 37C)
    max_warming_rate: float     # Max degrees per minute
    method: WarmingMethod
    phase_transition_temp: float  # Glass transition temp

class WarmingProtocolController:
    """Controls the warming phase of revival."""

    # Critical temperature thresholds
    GLASS_TRANSITION_TEMP = -123.0  # Typical vitrification glass transition
    ICE_FORMATION_RISK_RANGE = (-50.0, -10.0)
    REWARMING_INJURY_THRESHOLD = 2.0  # Max safe warming rate above 0C

    def __init__(self, parameters: WarmingParameters):
        self.parameters = parameters
        self.temperature_history: List[TemperaturePoint] = []
        self.alerts: List[str] = []
        self.current_temperature = parameters.initial_temperature

    def calculate_optimal_warming_rate(
        self,
        current_temp: float
    ) -> float:
        """Calculate optimal warming rate based on current temperature."""

        # Below glass transition: can warm faster
        if current_temp < self.GLASS_TRANSITION_TEMP:
            return min(self.parameters.max_warming_rate, 5.0)

        # In glass transition zone: slow and careful
        if self.GLASS_TRANSITION_TEMP <= current_temp < self.ICE_FORMATION_RISK_RANGE[0]:
            return min(self.parameters.max_warming_rate, 1.0)

        # In ice formation risk zone: very slow
        if self.ICE_FORMATION_RISK_RANGE[0] <= current_temp <= self.ICE_FORMATION_RISK_RANGE[1]:
            return min(self.parameters.max_warming_rate, 0.5)

        # Above freezing: moderate rate, avoid rewarming injury
        if current_temp > 0:
            return min(self.parameters.max_warming_rate, self.REWARMING_INJURY_THRESHOLD)

        return min(self.parameters.max_warming_rate, 1.0)

    def calculate_warming_profile(self) -> List[Dict[str, Any]]:
        """Generate the complete warming profile from current to target."""
        profile = []
        temp = self.parameters.initial_temperature
        elapsed_minutes = 0

        while temp < self.parameters.target_temperature:
            rate = self.calculate_optimal_warming_rate(temp)

            # Calculate time to next checkpoint
            if temp < self.GLASS_TRANSITION_TEMP:
                next_checkpoint = self.GLASS_TRANSITION_TEMP
            elif temp < self.ICE_FORMATION_RISK_RANGE[0]:
                next_checkpoint = self.ICE_FORMATION_RISK_RANGE[0]
            elif temp < self.ICE_FORMATION_RISK_RANGE[1]:
                next_checkpoint = self.ICE_FORMATION_RISK_RANGE[1]
            elif temp < 0:
                next_checkpoint = 0.0
            else:
                next_checkpoint = self.parameters.target_temperature

            next_checkpoint = min(next_checkpoint, self.parameters.target_temperature)
            temp_delta = next_checkpoint - temp
            time_needed = temp_delta / rate

            profile.append({
                "start_temp": temp,
                "end_temp": next_checkpoint,
                "warming_rate": rate,
                "duration_minutes": time_needed,
                "elapsed_minutes": elapsed_minutes,
                "phase": self._get_phase_name(temp)
            })

            temp = next_checkpoint
            elapsed_minutes += time_needed

        return profile

    def record_temperature(
        self,
        temperature: float,
        location: str,
        source: str
    ) -> Optional[str]:
        """Record a temperature reading and check for issues."""
        point = TemperaturePoint(
            timestamp=datetime.utcnow(),
            temperature=temperature,
            location=location,
            source=source
        )
        self.temperature_history.append(point)

        # Check for issues
        alert = self._check_temperature_alert(temperature, location)
        if alert:
            self.alerts.append(alert)
            return alert

        # Check warming rate
        rate_alert = self._check_warming_rate(location)
        if rate_alert:
            self.alerts.append(rate_alert)
            return rate_alert

        self.current_temperature = temperature
        return None

    def _check_temperature_alert(
        self,
        temperature: float,
        location: str
    ) -> Optional[str]:
        """Check for temperature-related alerts."""

        # Check for unexpected cooling
        if len(self.temperature_history) > 1:
            prev = self.temperature_history[-2]
            if prev.location == location and temperature < prev.temperature - 0.5:
                return f"ALERT: Temperature drop detected at {location}: {prev.temperature}C -> {temperature}C"

        # Check for overshoot
        if temperature > self.parameters.target_temperature + 0.5:
            return f"ALERT: Temperature overshoot at {location}: {temperature}C (target: {self.parameters.target_temperature}C)"

        return None

    def _check_warming_rate(self, location: str) -> Optional[str]:
        """Check if warming rate is within safe limits."""
        location_readings = [
            p for p in self.temperature_history[-10:]
            if p.location == location
        ]

        if len(location_readings) < 2:
            return None

        recent = location_readings[-1]
        prev = location_readings[-2]

        time_delta = (recent.timestamp - prev.timestamp).total_seconds() / 60
        if time_delta <= 0:
            return None

        rate = (recent.temperature - prev.temperature) / time_delta
        max_rate = self.calculate_optimal_warming_rate(prev.temperature)

        if rate > max_rate * 1.2:  # 20% tolerance
            return f"WARNING: Warming rate too fast at {location}: {rate:.2f}C/min (max: {max_rate:.2f}C/min)"

        return None

    def _get_phase_name(self, temp: float) -> str:
        """Get the warming phase name for a temperature."""
        if temp < self.GLASS_TRANSITION_TEMP:
            return "VITRIFIED"
        elif temp < self.ICE_FORMATION_RISK_RANGE[0]:
            return "GLASS_TRANSITION"
        elif temp <= self.ICE_FORMATION_RISK_RANGE[1]:
            return "ICE_RISK_ZONE"
        elif temp < 0:
            return "SUB_FREEZING"
        elif temp < 30:
            return "HYPOTHERMIC"
        else:
            return "NORMOTHERMIC"

    def is_warming_complete(self) -> bool:
        """Check if warming is complete."""
        return self.current_temperature >= self.parameters.target_temperature - 0.5

    def get_status(self) -> Dict[str, Any]:
        """Get current warming status."""
        return {
            "current_temperature": self.current_temperature,
            "target_temperature": self.parameters.target_temperature,
            "phase": self._get_phase_name(self.current_temperature),
            "optimal_rate": self.calculate_optimal_warming_rate(self.current_temperature),
            "is_complete": self.is_warming_complete(),
            "alerts_count": len(self.alerts),
            "readings_count": len(self.temperature_history)
        }
```

### Cardiovascular Restart Controller

```python
from dataclasses import dataclass
from typing import List, Optional, Dict
from enum import Enum

class RestartMethod(Enum):
    ELECTRICAL = "ELECTRICAL"
    MECHANICAL = "MECHANICAL"
    PHARMACOLOGICAL = "PHARMACOLOGICAL"
    COMBINED = "COMBINED"

class CardiacRhythm(Enum):
    ASYSTOLE = "ASYSTOLE"
    VF = "VENTRICULAR_FIBRILLATION"
    VT = "VENTRICULAR_TACHYCARDIA"
    PEA = "PULSELESS_ELECTRICAL_ACTIVITY"
    BRADYCARDIA = "BRADYCARDIA"
    SINUS = "SINUS_RHYTHM"
    AFIB = "ATRIAL_FIBRILLATION"
    PACED = "PACED"

@dataclass
class DefibrillationAttempt:
    timestamp: datetime
    energy_joules: int
    rhythm_before: CardiacRhythm
    rhythm_after: CardiacRhythm
    success: bool

@dataclass
class VasoactiveAgent:
    name: str
    dose: float
    unit: str
    route: str
    started_at: datetime
    current_rate: Optional[float]

@dataclass
class CardiovascularStatus:
    heart_rate: Optional[int]
    rhythm: CardiacRhythm
    systolic_bp: Optional[int]
    diastolic_bp: Optional[int]
    mean_arterial_pressure: Optional[int]
    cardiac_output: Optional[float]
    central_venous_pressure: Optional[int]

class CardiovascularRestartController:
    """Controls the cardiovascular restart phase."""

    # Target ranges
    TARGET_HR = (60, 100)
    TARGET_SBP = (90, 140)
    TARGET_DBP = (60, 90)
    TARGET_MAP = (65, 100)

    # Defibrillation parameters
    INITIAL_SHOCK_ENERGY = 200
    MAX_SHOCK_ENERGY = 360
    SHOCK_ESCALATION = 50

    def __init__(self, procedure_id: str):
        self.procedure_id = procedure_id
        self.defibrillation_attempts: List[DefibrillationAttempt] = []
        self.vasoactive_agents: List[VasoactiveAgent] = []
        self.status_history: List[CardiovascularStatus] = []
        self.current_status: Optional[CardiovascularStatus] = None

    def initialize_restart(
        self,
        initial_rhythm: CardiacRhythm
    ) -> Dict[str, Any]:
        """Initialize cardiovascular restart with initial rhythm."""
        self.current_status = CardiovascularStatus(
            heart_rate=None,
            rhythm=initial_rhythm,
            systolic_bp=None,
            diastolic_bp=None,
            mean_arterial_pressure=None,
            cardiac_output=None,
            central_venous_pressure=None
        )

        return self.recommend_action()

    def recommend_action(self) -> Dict[str, Any]:
        """Recommend next action based on current status."""
        if not self.current_status:
            return {"action": "INITIALIZE", "details": "Initialize restart first"}

        rhythm = self.current_status.rhythm

        # Shockable rhythms
        if rhythm in [CardiacRhythm.VF, CardiacRhythm.VT]:
            energy = self._calculate_shock_energy()
            return {
                "action": "DEFIBRILLATE",
                "energy_joules": energy,
                "details": f"Shockable rhythm detected: {rhythm.value}",
                "medications": ["Epinephrine 1mg IV", "Amiodarone 300mg IV"]
            }

        # Asystole or PEA
        if rhythm in [CardiacRhythm.ASYSTOLE, CardiacRhythm.PEA]:
            return {
                "action": "CPR_AND_MEDICATIONS",
                "details": f"Non-shockable rhythm: {rhythm.value}",
                "medications": ["Epinephrine 1mg IV q3-5min"],
                "consider": ["Transcutaneous pacing", "Reversible causes (H's and T's)"]
            }

        # Bradycardia with hypotension
        if rhythm == CardiacRhythm.BRADYCARDIA:
            return {
                "action": "PACING",
                "details": "Bradycardia - consider pacing",
                "medications": ["Atropine 0.5mg IV", "Dopamine infusion"],
                "pacing_rate": 60
            }

        # Stable rhythm - optimize hemodynamics
        if rhythm in [CardiacRhythm.SINUS, CardiacRhythm.PACED]:
            return self._optimize_hemodynamics()

        return {"action": "MONITOR", "details": "Continue monitoring"}

    def record_defibrillation(
        self,
        energy: int,
        rhythm_before: CardiacRhythm,
        rhythm_after: CardiacRhythm
    ) -> Dict[str, Any]:
        """Record a defibrillation attempt."""
        success = rhythm_after in [
            CardiacRhythm.SINUS,
            CardiacRhythm.BRADYCARDIA,
            CardiacRhythm.PACED
        ]

        attempt = DefibrillationAttempt(
            timestamp=datetime.utcnow(),
            energy_joules=energy,
            rhythm_before=rhythm_before,
            rhythm_after=rhythm_after,
            success=success
        )
        self.defibrillation_attempts.append(attempt)

        self.current_status.rhythm = rhythm_after

        return {
            "attempt_number": len(self.defibrillation_attempts),
            "success": success,
            "rhythm_achieved": rhythm_after.value,
            "next_action": self.recommend_action()
        }

    def add_vasoactive_agent(
        self,
        name: str,
        dose: float,
        unit: str,
        route: str
    ) -> None:
        """Add a vasoactive agent."""
        agent = VasoactiveAgent(
            name=name,
            dose=dose,
            unit=unit,
            route=route,
            started_at=datetime.utcnow(),
            current_rate=dose if "infusion" in route.lower() else None
        )
        self.vasoactive_agents.append(agent)

    def update_status(
        self,
        heart_rate: Optional[int] = None,
        rhythm: Optional[CardiacRhythm] = None,
        systolic_bp: Optional[int] = None,
        diastolic_bp: Optional[int] = None,
        cardiac_output: Optional[float] = None,
        cvp: Optional[int] = None
    ) -> Dict[str, Any]:
        """Update cardiovascular status."""
        if not self.current_status:
            self.current_status = CardiovascularStatus(
                heart_rate=None,
                rhythm=CardiacRhythm.ASYSTOLE,
                systolic_bp=None,
                diastolic_bp=None,
                mean_arterial_pressure=None,
                cardiac_output=None,
                central_venous_pressure=None
            )

        if heart_rate is not None:
            self.current_status.heart_rate = heart_rate
        if rhythm is not None:
            self.current_status.rhythm = rhythm
        if systolic_bp is not None:
            self.current_status.systolic_bp = systolic_bp
        if diastolic_bp is not None:
            self.current_status.diastolic_bp = diastolic_bp
        if systolic_bp is not None and diastolic_bp is not None:
            self.current_status.mean_arterial_pressure = int(
                diastolic_bp + (systolic_bp - diastolic_bp) / 3
            )
        if cardiac_output is not None:
            self.current_status.cardiac_output = cardiac_output
        if cvp is not None:
            self.current_status.central_venous_pressure = cvp

        self.status_history.append(self.current_status)

        return {
            "status": self._status_to_dict(),
            "in_target": self._check_targets(),
            "recommendations": self.recommend_action()
        }

    def _calculate_shock_energy(self) -> int:
        """Calculate next shock energy based on previous attempts."""
        if not self.defibrillation_attempts:
            return self.INITIAL_SHOCK_ENERGY

        last_energy = self.defibrillation_attempts[-1].energy_joules
        next_energy = last_energy + self.SHOCK_ESCALATION
        return min(next_energy, self.MAX_SHOCK_ENERGY)

    def _optimize_hemodynamics(self) -> Dict[str, Any]:
        """Recommend hemodynamic optimization."""
        recommendations = []

        if self.current_status.mean_arterial_pressure:
            map_val = self.current_status.mean_arterial_pressure
            if map_val < self.TARGET_MAP[0]:
                recommendations.append("Increase vasopressor support")
                recommendations.append("Consider fluid bolus if CVP low")
            elif map_val > self.TARGET_MAP[1]:
                recommendations.append("Consider vasodilator")
                recommendations.append("Reduce vasopressor if applicable")

        if self.current_status.heart_rate:
            hr = self.current_status.heart_rate
            if hr < self.TARGET_HR[0]:
                recommendations.append("Consider increasing pacing rate or chronotropes")
            elif hr > self.TARGET_HR[1]:
                recommendations.append("Consider rate control")

        if not recommendations:
            recommendations.append("Hemodynamics within target - continue monitoring")

        return {
            "action": "OPTIMIZE_HEMODYNAMICS",
            "details": recommendations,
            "current_map": self.current_status.mean_arterial_pressure,
            "target_map": self.TARGET_MAP
        }

    def _check_targets(self) -> Dict[str, bool]:
        """Check if vital signs are within target ranges."""
        return {
            "heart_rate": (
                self.current_status.heart_rate is not None and
                self.TARGET_HR[0] <= self.current_status.heart_rate <= self.TARGET_HR[1]
            ),
            "blood_pressure": (
                self.current_status.systolic_bp is not None and
                self.TARGET_SBP[0] <= self.current_status.systolic_bp <= self.TARGET_SBP[1]
            ),
            "map": (
                self.current_status.mean_arterial_pressure is not None and
                self.TARGET_MAP[0] <= self.current_status.mean_arterial_pressure <= self.TARGET_MAP[1]
            )
        }

    def _status_to_dict(self) -> Dict[str, Any]:
        """Convert current status to dictionary."""
        return {
            "heart_rate": self.current_status.heart_rate,
            "rhythm": self.current_status.rhythm.value,
            "systolic_bp": self.current_status.systolic_bp,
            "diastolic_bp": self.current_status.diastolic_bp,
            "map": self.current_status.mean_arterial_pressure,
            "cardiac_output": self.current_status.cardiac_output,
            "cvp": self.current_status.central_venous_pressure
        }

    def is_restart_successful(self) -> bool:
        """Check if cardiovascular restart is successful."""
        if not self.current_status:
            return False

        targets = self._check_targets()
        stable_rhythm = self.current_status.rhythm in [
            CardiacRhythm.SINUS,
            CardiacRhythm.PACED
        ]

        return stable_rhythm and all(targets.values())
```

### Neurological Restoration Monitor

```python
from dataclasses import dataclass
from typing import List, Optional, Dict
from enum import Enum

class ConsciousnessLevel(Enum):
    UNRESPONSIVE = 0
    PAIN_RESPONSE = 1
    PRESSURE_RESPONSE = 2
    VOICE_RESPONSE = 3
    CONFUSED = 4
    ORIENTED = 5

class PupilResponse(Enum):
    FIXED = "FIXED"
    SLUGGISH = "SLUGGISH"
    REACTIVE = "REACTIVE"
    BRISK = "BRISK"

@dataclass
class EEGReading:
    timestamp: datetime
    pattern: str  # FLAT, BURST_SUPPRESSION, SLOW_WAVE, NORMAL
    frequency_hz: Optional[float]
    amplitude_uv: Optional[float]
    seizure_activity: bool

@dataclass
class PupilAssessment:
    timestamp: datetime
    left_size_mm: float
    right_size_mm: float
    left_response: PupilResponse
    right_response: PupilResponse

@dataclass
class ReflexAssessment:
    timestamp: datetime
    corneal: bool
    gag: bool
    cough: bool
    oculocephalic: bool
    oculovestibular: bool

@dataclass
class NeurologicalStatus:
    consciousness: ConsciousnessLevel
    gcs_score: int  # Glasgow Coma Scale
    pupils: PupilAssessment
    reflexes: ReflexAssessment
    eeg: EEGReading
    intracranial_pressure: Optional[float]

class NeurologicalRestorationMonitor:
    """Monitors neurological restoration during revival."""

    # Target thresholds
    ICP_MAX = 20.0  # mmHg
    CPP_MIN = 60.0  # Cerebral perfusion pressure minimum

    def __init__(self, procedure_id: str):
        self.procedure_id = procedure_id
        self.eeg_history: List[EEGReading] = []
        self.pupil_history: List[PupilAssessment] = []
        self.reflex_history: List[ReflexAssessment] = []
        self.consciousness_history: List[tuple[datetime, ConsciousnessLevel]] = []
        self.current_status: Optional[NeurologicalStatus] = None
        self.alerts: List[str] = []

    def record_eeg(
        self,
        pattern: str,
        frequency_hz: Optional[float] = None,
        amplitude_uv: Optional[float] = None,
        seizure_activity: bool = False
    ) -> Dict[str, Any]:
        """Record EEG reading."""
        reading = EEGReading(
            timestamp=datetime.utcnow(),
            pattern=pattern,
            frequency_hz=frequency_hz,
            amplitude_uv=amplitude_uv,
            seizure_activity=seizure_activity
        )
        self.eeg_history.append(reading)

        # Check for seizure
        if seizure_activity:
            alert = "CRITICAL: Seizure activity detected"
            self.alerts.append(alert)
            return {
                "recorded": True,
                "alert": alert,
                "action_required": "Administer anticonvulsant"
            }

        # Track progression
        progression = self._assess_eeg_progression()

        return {
            "recorded": True,
            "pattern": pattern,
            "progression": progression,
            "alert": None
        }

    def record_pupil_exam(
        self,
        left_size: float,
        right_size: float,
        left_response: PupilResponse,
        right_response: PupilResponse
    ) -> Dict[str, Any]:
        """Record pupil examination."""
        assessment = PupilAssessment(
            timestamp=datetime.utcnow(),
            left_size_mm=left_size,
            right_size_mm=right_size,
            left_response=left_response,
            right_response=right_response
        )
        self.pupil_history.append(assessment)

        # Check for concerning findings
        concerns = []

        # Anisocoria (unequal pupils)
        if abs(left_size - right_size) > 1.0:
            concerns.append(f"Anisocoria: L={left_size}mm, R={right_size}mm")

        # Fixed pupils
        if left_response == PupilResponse.FIXED:
            concerns.append("Left pupil fixed")
        if right_response == PupilResponse.FIXED:
            concerns.append("Right pupil fixed")

        # Dilated pupils
        if left_size > 6.0 or right_size > 6.0:
            concerns.append("Dilated pupils - possible increased ICP")

        if concerns:
            alert = f"PUPIL CONCERNS: {'; '.join(concerns)}"
            self.alerts.append(alert)
            return {"recorded": True, "concerns": concerns, "alert": alert}

        return {"recorded": True, "concerns": [], "alert": None}

    def record_reflex_exam(
        self,
        corneal: bool,
        gag: bool,
        cough: bool,
        oculocephalic: bool,
        oculovestibular: bool
    ) -> Dict[str, Any]:
        """Record brainstem reflex examination."""
        assessment = ReflexAssessment(
            timestamp=datetime.utcnow(),
            corneal=corneal,
            gag=gag,
            cough=cough,
            oculocephalic=oculocephalic,
            oculovestibular=oculovestibular
        )
        self.reflex_history.append(assessment)

        # Count present reflexes
        reflexes_present = sum([
            corneal, gag, cough, oculocephalic, oculovestibular
        ])

        interpretation = self._interpret_reflexes(assessment)

        return {
            "recorded": True,
            "reflexes_present": reflexes_present,
            "interpretation": interpretation
        }

    def assess_consciousness(
        self,
        eye_response: int,  # 1-4
        verbal_response: int,  # 1-5
        motor_response: int  # 1-6
    ) -> Dict[str, Any]:
        """Assess consciousness using Glasgow Coma Scale."""
        gcs = eye_response + verbal_response + motor_response

        # Map GCS to consciousness level
        if gcs <= 3:
            level = ConsciousnessLevel.UNRESPONSIVE
        elif gcs <= 5:
            level = ConsciousnessLevel.PAIN_RESPONSE
        elif gcs <= 8:
            level = ConsciousnessLevel.PRESSURE_RESPONSE
        elif gcs <= 12:
            level = ConsciousnessLevel.VOICE_RESPONSE
        elif gcs <= 14:
            level = ConsciousnessLevel.CONFUSED
        else:
            level = ConsciousnessLevel.ORIENTED

        self.consciousness_history.append((datetime.utcnow(), level))

        # Check for improvement
        improvement = self._assess_consciousness_trend()

        return {
            "gcs_score": gcs,
            "consciousness_level": level.name,
            "eye": eye_response,
            "verbal": verbal_response,
            "motor": motor_response,
            "trend": improvement
        }

    def record_icp(
        self,
        icp_value: float,
        map_value: float
    ) -> Dict[str, Any]:
        """Record intracranial pressure and calculate CPP."""
        cpp = map_value - icp_value  # Cerebral perfusion pressure

        alerts = []

        if icp_value > self.ICP_MAX:
            alerts.append(f"ELEVATED ICP: {icp_value} mmHg (max: {self.ICP_MAX})")

        if cpp < self.CPP_MIN:
            alerts.append(f"LOW CPP: {cpp} mmHg (min: {self.CPP_MIN})")

        if alerts:
            for alert in alerts:
                self.alerts.append(alert)

        return {
            "icp": icp_value,
            "cpp": cpp,
            "icp_elevated": icp_value > self.ICP_MAX,
            "cpp_adequate": cpp >= self.CPP_MIN,
            "alerts": alerts,
            "interventions": self._recommend_icp_interventions(icp_value, cpp)
        }

    def _assess_eeg_progression(self) -> str:
        """Assess EEG pattern progression."""
        if len(self.eeg_history) < 2:
            return "INSUFFICIENT_DATA"

        patterns_order = ["FLAT", "BURST_SUPPRESSION", "SLOW_WAVE", "NORMAL"]

        recent = self.eeg_history[-1].pattern
        previous = self.eeg_history[-2].pattern

        if recent not in patterns_order or previous not in patterns_order:
            return "UNKNOWN"

        recent_idx = patterns_order.index(recent)
        prev_idx = patterns_order.index(previous)

        if recent_idx > prev_idx:
            return "IMPROVING"
        elif recent_idx < prev_idx:
            return "DETERIORATING"
        else:
            return "STABLE"

    def _interpret_reflexes(self, assessment: ReflexAssessment) -> str:
        """Interpret brainstem reflex examination."""
        if not any([
            assessment.corneal,
            assessment.gag,
            assessment.cough,
            assessment.oculocephalic,
            assessment.oculovestibular
        ]):
            return "ABSENT_BRAINSTEM_FUNCTION"

        if all([
            assessment.corneal,
            assessment.gag,
            assessment.cough
        ]):
            return "INTACT_BRAINSTEM_FUNCTION"

        return "PARTIAL_BRAINSTEM_FUNCTION"

    def _assess_consciousness_trend(self) -> str:
        """Assess trend in consciousness level."""
        if len(self.consciousness_history) < 3:
            return "INSUFFICIENT_DATA"

        recent_levels = [
            level.value for _, level in self.consciousness_history[-3:]
        ]

        if recent_levels[-1] > recent_levels[0]:
            return "IMPROVING"
        elif recent_levels[-1] < recent_levels[0]:
            return "DETERIORATING"
        else:
            return "STABLE"

    def _recommend_icp_interventions(
        self,
        icp: float,
        cpp: float
    ) -> List[str]:
        """Recommend interventions for elevated ICP."""
        interventions = []

        if icp > 20:
            interventions.append("Elevate head of bed to 30 degrees")
            interventions.append("Ensure adequate sedation")

        if icp > 25:
            interventions.append("Administer osmotic therapy (Mannitol or Hypertonic saline)")
            interventions.append("Consider hyperventilation to PaCO2 30-35")

        if icp > 30:
            interventions.append("URGENT: Consider surgical decompression")
            interventions.append("Neurosurgery consultation required")

        if cpp < 60:
            interventions.append("Increase MAP with vasopressors")

        return interventions

    def is_restoration_successful(self) -> bool:
        """Check if neurological restoration is successful."""
        if len(self.consciousness_history) < 1:
            return False

        latest_consciousness = self.consciousness_history[-1][1]

        return latest_consciousness.value >= ConsciousnessLevel.CONFUSED.value

    def get_summary(self) -> Dict[str, Any]:
        """Get neurological status summary."""
        latest_consciousness = (
            self.consciousness_history[-1][1].name
            if self.consciousness_history else "UNKNOWN"
        )
        latest_eeg = (
            self.eeg_history[-1].pattern
            if self.eeg_history else "UNKNOWN"
        )

        return {
            "consciousness": latest_consciousness,
            "eeg_pattern": latest_eeg,
            "alerts_count": len(self.alerts),
            "recent_alerts": self.alerts[-5:] if self.alerts else [],
            "restoration_successful": self.is_restoration_successful()
        }
```

### Re-Preservation Decision Engine

```python
from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

class RePreservationTrigger(Enum):
    IRREVERSIBLE_CARDIAC_FAILURE = "IRREVERSIBLE_CARDIAC_FAILURE"
    IRREVERSIBLE_BRAIN_DAMAGE = "IRREVERSIBLE_BRAIN_DAMAGE"
    MULTIPLE_ORGAN_FAILURE = "MULTIPLE_ORGAN_FAILURE"
    SUBJECT_REQUEST = "SUBJECT_REQUEST"
    GUARDIAN_REQUEST = "GUARDIAN_REQUEST"
    PROTOCOL_THRESHOLD = "PROTOCOL_THRESHOLD"

@dataclass
class RePreservationCriterion:
    criterion_id: str
    description: str
    weight: float
    met: bool
    evidence: str

@dataclass
class TeamVote:
    member_id: str
    role: str
    vote: str  # PROCEED, OPPOSE, ABSTAIN
    rationale: str
    timestamp: datetime

class RePreservationDecisionEngine:
    """Engine for making re-preservation decisions during failed revival."""

    # Criteria thresholds
    PROCEED_THRESHOLD = 0.7  # 70% weighted score to proceed
    MINIMUM_VOTERS = 3

    # Standard criteria
    STANDARD_CRITERIA = [
        {
            "id": "CARDIAC_IRREVERSIBILITY",
            "description": "Cardiac function cannot be restored after maximum intervention",
            "weight": 0.25,
            "assessment": "Multiple failed cardioversion/defibrillation attempts, refractory to all vasopressors"
        },
        {
            "id": "NEUROLOGICAL_IRREVERSIBILITY",
            "description": "Evidence of irreversible brain damage",
            "weight": 0.30,
            "assessment": "Absent brainstem reflexes, flat EEG >24 hours, fixed dilated pupils"
        },
        {
            "id": "MULTI_ORGAN_FAILURE",
            "description": "Three or more organ systems in failure",
            "weight": 0.20,
            "assessment": "Renal, hepatic, respiratory failure despite support"
        },
        {
            "id": "TIME_THRESHOLD",
            "description": "Maximum revival attempt duration exceeded",
            "weight": 0.10,
            "assessment": "Revival attempt >72 hours without improvement"
        },
        {
            "id": "TISSUE_VIABILITY",
            "description": "Tissue remains viable for re-preservation",
            "weight": 0.15,
            "assessment": "Tissue perfusion maintained, limited ischemic damage"
        }
    ]

    def __init__(self, procedure_id: str, consent_document_id: str):
        self.procedure_id = procedure_id
        self.consent_document_id = consent_document_id
        self.criteria: List[RePreservationCriterion] = []
        self.votes: List[TeamVote] = []
        self.trigger: Optional[RePreservationTrigger] = None
        self.decision: Optional[str] = None
        self.decision_timestamp: Optional[datetime] = None

    def initialize_criteria(self) -> None:
        """Initialize standard criteria for evaluation."""
        self.criteria = [
            RePreservationCriterion(
                criterion_id=c["id"],
                description=c["description"],
                weight=c["weight"],
                met=False,
                evidence=""
            )
            for c in self.STANDARD_CRITERIA
        ]

    def set_trigger(self, trigger: RePreservationTrigger) -> None:
        """Set the trigger for re-preservation consideration."""
        self.trigger = trigger

    def evaluate_criterion(
        self,
        criterion_id: str,
        met: bool,
        evidence: str
    ) -> Dict[str, Any]:
        """Evaluate a specific criterion."""
        for criterion in self.criteria:
            if criterion.criterion_id == criterion_id:
                criterion.met = met
                criterion.evidence = evidence
                return {
                    "criterion_id": criterion_id,
                    "met": met,
                    "weight": criterion.weight,
                    "current_score": self.calculate_score()
                }

        return {"error": f"Criterion {criterion_id} not found"}

    def calculate_score(self) -> float:
        """Calculate weighted score of met criteria."""
        total_weight = sum(c.weight for c in self.criteria)
        met_weight = sum(c.weight for c in self.criteria if c.met)

        return met_weight / total_weight if total_weight > 0 else 0.0

    def record_vote(
        self,
        member_id: str,
        role: str,
        vote: str,
        rationale: str
    ) -> Dict[str, Any]:
        """Record a team member's vote."""
        team_vote = TeamVote(
            member_id=member_id,
            role=role,
            vote=vote,
            rationale=rationale,
            timestamp=datetime.utcnow()
        )
        self.votes.append(team_vote)

        return {
            "recorded": True,
            "total_votes": len(self.votes),
            "vote_summary": self._get_vote_summary()
        }

    def _get_vote_summary(self) -> Dict[str, int]:
        """Get summary of votes."""
        summary = {"PROCEED": 0, "OPPOSE": 0, "ABSTAIN": 0}
        for vote in self.votes:
            if vote.vote in summary:
                summary[vote.vote] += 1
        return summary

    def can_make_decision(self) -> tuple[bool, List[str]]:
        """Check if enough information to make decision."""
        issues = []

        if not self.trigger:
            issues.append("No trigger set")

        if len(self.votes) < self.MINIMUM_VOTERS:
            issues.append(f"Need at least {self.MINIMUM_VOTERS} votes (have {len(self.votes)})")

        # Check if all criteria evaluated
        unevaluated = [c for c in self.criteria if not c.evidence]
        if unevaluated:
            issues.append(f"{len(unevaluated)} criteria not evaluated")

        return len(issues) == 0, issues

    def make_decision(self) -> Dict[str, Any]:
        """Make the re-preservation decision."""
        can_decide, issues = self.can_make_decision()

        if not can_decide:
            return {
                "decision": None,
                "error": "Cannot make decision",
                "issues": issues
            }

        # Calculate criteria score
        criteria_score = self.calculate_score()

        # Calculate vote result
        vote_summary = self._get_vote_summary()
        total_votes = vote_summary["PROCEED"] + vote_summary["OPPOSE"]
        proceed_percentage = (
            vote_summary["PROCEED"] / total_votes if total_votes > 0 else 0
        )

        # Decision logic
        if criteria_score >= self.PROCEED_THRESHOLD and proceed_percentage > 0.5:
            decision = "REPRESERVE"
            rationale = (
                f"Criteria score ({criteria_score:.1%}) meets threshold. "
                f"Team vote: {proceed_percentage:.1%} in favor."
            )
        elif criteria_score < 0.3 or proceed_percentage < 0.3:
            decision = "CONTINUE"
            rationale = (
                f"Revival should continue. Criteria score: {criteria_score:.1%}, "
                f"Vote to proceed: {proceed_percentage:.1%}"
            )
        else:
            decision = "REVIEW"
            rationale = (
                f"Decision requires further review. Criteria score: {criteria_score:.1%}, "
                f"Vote to proceed: {proceed_percentage:.1%}"
            )

        self.decision = decision
        self.decision_timestamp = datetime.utcnow()

        return {
            "decision": decision,
            "rationale": rationale,
            "criteria_score": criteria_score,
            "vote_summary": vote_summary,
            "proceed_percentage": proceed_percentage,
            "trigger": self.trigger.value if self.trigger else None,
            "timestamp": self.decision_timestamp.isoformat()
        }

    def generate_report(self) -> Dict[str, Any]:
        """Generate comprehensive decision report."""
        return {
            "procedure_id": self.procedure_id,
            "consent_document_id": self.consent_document_id,
            "trigger": self.trigger.value if self.trigger else None,
            "criteria": [
                {
                    "id": c.criterion_id,
                    "description": c.description,
                    "weight": c.weight,
                    "met": c.met,
                    "evidence": c.evidence
                }
                for c in self.criteria
            ],
            "criteria_score": self.calculate_score(),
            "votes": [
                {
                    "member_id": v.member_id,
                    "role": v.role,
                    "vote": v.vote,
                    "rationale": v.rationale,
                    "timestamp": v.timestamp.isoformat()
                }
                for v in self.votes
            ],
            "vote_summary": self._get_vote_summary(),
            "decision": self.decision,
            "decision_timestamp": (
                self.decision_timestamp.isoformat()
                if self.decision_timestamp else None
            )
        }
```

### Alert and Monitoring System

```python
from dataclasses import dataclass
from typing import List, Dict, Callable, Optional
from enum import Enum
from datetime import datetime
import json

class AlertSeverity(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    EMERGENCY = "EMERGENCY"

class AlertCategory(Enum):
    VITAL_SIGNS = "VITAL_SIGNS"
    LABORATORY = "LABORATORY"
    EQUIPMENT = "EQUIPMENT"
    PROTOCOL = "PROTOCOL"
    STAFFING = "STAFFING"

@dataclass
class Alert:
    alert_id: str
    timestamp: datetime
    severity: AlertSeverity
    category: AlertCategory
    source: str
    message: str
    data: Dict
    acknowledged: bool = False
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None
    resolved: bool = False
    resolved_by: Optional[str] = None
    resolution: Optional[str] = None

@dataclass
class AlertRule:
    rule_id: str
    name: str
    category: AlertCategory
    condition: Callable[[Dict], bool]
    severity: AlertSeverity
    message_template: str
    cooldown_seconds: int = 60

class AlertMonitoringSystem:
    """System for monitoring and alerting during revival procedures."""

    def __init__(self, procedure_id: str):
        self.procedure_id = procedure_id
        self.alerts: List[Alert] = []
        self.rules: List[AlertRule] = []
        self.alert_counter = 0
        self.last_alert_times: Dict[str, datetime] = {}
        self._initialize_default_rules()

    def _initialize_default_rules(self) -> None:
        """Initialize default monitoring rules."""
        self.rules = [
            # Vital sign rules
            AlertRule(
                rule_id="VS_HR_LOW",
                name="Heart Rate Low",
                category=AlertCategory.VITAL_SIGNS,
                condition=lambda d: d.get("heart_rate", 999) < 40,
                severity=AlertSeverity.CRITICAL,
                message_template="Heart rate critically low: {heart_rate} bpm"
            ),
            AlertRule(
                rule_id="VS_HR_HIGH",
                name="Heart Rate High",
                category=AlertCategory.VITAL_SIGNS,
                condition=lambda d: d.get("heart_rate", 0) > 150,
                severity=AlertSeverity.WARNING,
                message_template="Heart rate elevated: {heart_rate} bpm"
            ),
            AlertRule(
                rule_id="VS_BP_LOW",
                name="Blood Pressure Low",
                category=AlertCategory.VITAL_SIGNS,
                condition=lambda d: d.get("systolic_bp", 999) < 80,
                severity=AlertSeverity.CRITICAL,
                message_template="Systolic BP critically low: {systolic_bp} mmHg"
            ),
            AlertRule(
                rule_id="VS_O2_LOW",
                name="Oxygen Saturation Low",
                category=AlertCategory.VITAL_SIGNS,
                condition=lambda d: d.get("spo2", 100) < 90,
                severity=AlertSeverity.CRITICAL,
                message_template="SpO2 low: {spo2}%"
            ),
            AlertRule(
                rule_id="VS_TEMP_HIGH",
                name="Temperature High",
                category=AlertCategory.VITAL_SIGNS,
                condition=lambda d: d.get("temperature", 37) > 39,
                severity=AlertSeverity.WARNING,
                message_template="Temperature elevated: {temperature}C"
            ),
            # Laboratory rules
            AlertRule(
                rule_id="LAB_K_HIGH",
                name="Potassium High",
                category=AlertCategory.LABORATORY,
                condition=lambda d: d.get("potassium", 4) > 6.0,
                severity=AlertSeverity.CRITICAL,
                message_template="Potassium critically elevated: {potassium} mEq/L"
            ),
            AlertRule(
                rule_id="LAB_LACTATE_HIGH",
                name="Lactate High",
                category=AlertCategory.LABORATORY,
                condition=lambda d: d.get("lactate", 1) > 4.0,
                severity=AlertSeverity.WARNING,
                message_template="Lactate elevated: {lactate} mmol/L"
            ),
            AlertRule(
                rule_id="LAB_PH_LOW",
                name="pH Low",
                category=AlertCategory.LABORATORY,
                condition=lambda d: d.get("ph", 7.4) < 7.2,
                severity=AlertSeverity.CRITICAL,
                message_template="Severe acidosis: pH {ph}"
            ),
            # Equipment rules
            AlertRule(
                rule_id="EQUIP_PUMP_FAIL",
                name="Perfusion Pump Failure",
                category=AlertCategory.EQUIPMENT,
                condition=lambda d: d.get("pump_status") == "FAILED",
                severity=AlertSeverity.EMERGENCY,
                message_template="PERFUSION PUMP FAILURE - Immediate action required"
            ),
            AlertRule(
                rule_id="EQUIP_MONITOR_DISCONNECT",
                name="Monitor Disconnected",
                category=AlertCategory.EQUIPMENT,
                condition=lambda d: d.get("monitor_connected") is False,
                severity=AlertSeverity.WARNING,
                message_template="Monitoring device disconnected: {device_name}"
            )
        ]

    def add_custom_rule(self, rule: AlertRule) -> None:
        """Add a custom monitoring rule."""
        self.rules.append(rule)

    def process_data(self, data: Dict) -> List[Alert]:
        """Process incoming data and generate alerts."""
        new_alerts = []

        for rule in self.rules:
            # Check cooldown
            if self._is_in_cooldown(rule.rule_id):
                continue

            # Evaluate condition
            try:
                if rule.condition(data):
                    alert = self._create_alert(rule, data)
                    new_alerts.append(alert)
                    self.alerts.append(alert)
                    self.last_alert_times[rule.rule_id] = datetime.utcnow()
            except Exception as e:
                # Log but don't crash on rule evaluation errors
                pass

        return new_alerts

    def _is_in_cooldown(self, rule_id: str) -> bool:
        """Check if rule is in cooldown period."""
        if rule_id not in self.last_alert_times:
            return False

        rule = next((r for r in self.rules if r.rule_id == rule_id), None)
        if not rule:
            return False

        elapsed = (datetime.utcnow() - self.last_alert_times[rule_id]).total_seconds()
        return elapsed < rule.cooldown_seconds

    def _create_alert(self, rule: AlertRule, data: Dict) -> Alert:
        """Create an alert from a rule."""
        self.alert_counter += 1
        alert_id = f"ALERT-{self.procedure_id}-{self.alert_counter:04d}"

        # Format message with data
        try:
            message = rule.message_template.format(**data)
        except KeyError:
            message = rule.message_template

        return Alert(
            alert_id=alert_id,
            timestamp=datetime.utcnow(),
            severity=rule.severity,
            category=rule.category,
            source=rule.rule_id,
            message=message,
            data=data
        )

    def acknowledge_alert(
        self,
        alert_id: str,
        acknowledged_by: str
    ) -> bool:
        """Acknowledge an alert."""
        for alert in self.alerts:
            if alert.alert_id == alert_id:
                alert.acknowledged = True
                alert.acknowledged_by = acknowledged_by
                alert.acknowledged_at = datetime.utcnow()
                return True
        return False

    def resolve_alert(
        self,
        alert_id: str,
        resolved_by: str,
        resolution: str
    ) -> bool:
        """Resolve an alert."""
        for alert in self.alerts:
            if alert.alert_id == alert_id:
                alert.resolved = True
                alert.resolved_by = resolved_by
                alert.resolution = resolution
                return True
        return False

    def get_active_alerts(
        self,
        severity: Optional[AlertSeverity] = None
    ) -> List[Alert]:
        """Get all active (unresolved) alerts."""
        active = [a for a in self.alerts if not a.resolved]

        if severity:
            active = [a for a in active if a.severity == severity]

        return sorted(active, key=lambda a: a.timestamp, reverse=True)

    def get_alert_summary(self) -> Dict[str, Any]:
        """Get summary of all alerts."""
        active = self.get_active_alerts()

        by_severity = {}
        for severity in AlertSeverity:
            count = len([a for a in active if a.severity == severity])
            by_severity[severity.value] = count

        return {
            "total_alerts": len(self.alerts),
            "active_alerts": len(active),
            "by_severity": by_severity,
            "emergency_count": by_severity.get("EMERGENCY", 0),
            "critical_count": by_severity.get("CRITICAL", 0),
            "oldest_unacknowledged": next(
                (a.alert_id for a in active if not a.acknowledged),
                None
            )
        }
```

## JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cryo-revival/algorithms/v1",
  "definitions": {
    "StageTransition": {
      "type": "object",
      "required": ["from_stage", "to_stage", "criteria_met"],
      "properties": {
        "from_stage": { "$ref": "#/definitions/RevivalStage" },
        "to_stage": { "$ref": "#/definitions/RevivalStage" },
        "criteria_met": { "type": "boolean" },
        "issues": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "WarmingStatus": {
      "type": "object",
      "properties": {
        "current_temperature": { "type": "number" },
        "target_temperature": { "type": "number" },
        "phase": { "type": "string" },
        "optimal_rate": { "type": "number" },
        "is_complete": { "type": "boolean" }
      }
    },
    "CardiovascularAction": {
      "type": "object",
      "properties": {
        "action": {
          "type": "string",
          "enum": ["DEFIBRILLATE", "CPR_AND_MEDICATIONS", "PACING", "OPTIMIZE_HEMODYNAMICS", "MONITOR"]
        },
        "details": { "type": "string" },
        "medications": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "RePreservationDecision": {
      "type": "object",
      "properties": {
        "decision": {
          "type": "string",
          "enum": ["REPRESERVE", "CONTINUE", "REVIEW"]
        },
        "criteria_score": { "type": "number" },
        "vote_summary": {
          "type": "object",
          "properties": {
            "PROCEED": { "type": "integer" },
            "OPPOSE": { "type": "integer" },
            "ABSTAIN": { "type": "integer" }
          }
        }
      }
    }
  }
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
