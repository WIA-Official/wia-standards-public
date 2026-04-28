# WIA-AUTO-018 — Phase 3: Protocol

> Railway-dynamics and safety-protocol layer. Every safety exchange is wire-level with replay defence and audit-log discipline so an investigator can reconstruct any incident.

## 8. Railway Dynamics

### 8.1 Braking Distance

#### 8.1.1 Basic Formula

**Total Braking Distance:**
```
d_total = d_reaction + d_braking + d_safety
```

**Reaction Distance:**
```
d_reaction = v × t_reaction
```

Where:
- `v` = Initial velocity (m/s)
- `t_reaction` = System reaction time (typically 2-3 seconds)

**Braking Distance:**
```
d_braking = v² / (2 × a)
```

Where:
- `a` = Deceleration rate (m/s²)

**Safety Margin:**
```
d_safety = 50 meters (mainline), 20 meters (metro)
```

#### 8.1.2 Example Calculations

**High-Speed Train (300 km/h = 83.33 m/s):**
```
Given:
- v = 83.33 m/s (300 km/h)
- a = 0.7 m/s² (service braking)
- t_reaction = 3 seconds

Calculation:
- d_reaction = 83.33 × 3 = 250 m
- d_braking = 83.33² / (2 × 0.7) = 4,960 m
- d_safety = 50 m
- d_total = 250 + 4,960 + 50 = 5,260 meters

Result: 5.26 km braking distance
```

**Metro Train (80 km/h = 22.22 m/s):**
```
Given:
- v = 22.22 m/s (80 km/h)
- a = 1.0 m/s² (service braking)
- t_reaction = 2 seconds

Calculation:
- d_reaction = 22.22 × 2 = 44 m
- d_braking = 22.22² / (2 × 1.0) = 247 m
- d_safety = 20 m
- d_total = 44 + 247 + 20 = 311 meters
```

### 8.2 Tractive Effort and Resistance

#### 8.2.1 Resistance Formula

**Total Resistance:**
```
R_total = R_rolling + R_aero + R_grade + R_curve
```

**Rolling Resistance:**
```
R_rolling = k_r × W
```

Where:
- `k_r` = Rolling resistance coefficient (0.0015-0.003)
- `W` = Train weight (kN)

**Aerodynamic Resistance:**
```
R_aero = 0.5 × ρ × C_d × A × v²
```

Where:
- `ρ` = Air density (1.225 kg/m³)
- `C_d` = Drag coefficient (0.6-1.8 depending on train shape)
- `A` = Frontal area (m²)
- `v` = Velocity (m/s)

**Grade Resistance:**
```
R_grade = W × sin(θ) ≈ W × (g / 1000)
```

Where:
- `g` = Gradient (‰ per thousand)

**Curve Resistance:**
```
R_curve = k_c × W / r
```

Where:
- `k_c` = Curve constant (≈ 0.5)
- `r` = Curve radius (m)

#### 8.2.2 Tractive Effort

**Available Tractive Effort:**
```
F_max = μ × W_adhesion
```

Where:
- `μ` = Adhesion coefficient (0.2-0.4 depending on conditions)
- `W_adhesion` = Weight on driven axles (kN)

**Acceleration:**
```
a = (F_traction - R_total) / m
```

Where:
- `F_traction` = Applied tractive effort (kN)
- `R_total` = Total resistance (kN)
- `m` = Train mass (tonnes)

### 8.3 Energy Consumption

#### 8.3.1 Energy Formula

**Total Energy:**
```
E_total = E_kinetic + E_potential + E_resistance + E_losses
```

**Kinetic Energy:**
```
E_kinetic = 0.5 × m × v²
```

**Potential Energy (gradient):**
```
E_potential = m × g × h
```

Where:
- `h` = Vertical elevation change (m)
- `g` = 9.81 m/s²

**Energy Recovery (regenerative braking):**
```
E_recovered = η_regen × E_kinetic_braking
```

Where:
- `η_regen` = Regeneration efficiency (0.6-0.8)

#### 8.3.2 Energy Optimization

**Optimal Driving Profile:**
```
1. Accelerate at maximum until v_cruise
2. Maintain v_cruise at minimum power
3. Coast before braking (if schedule permits)
4. Regenerative braking to recover energy
```

**Energy Savings:**
- ATO vs manual driving: 15-30% reduction
- Regenerative braking: 20-40% reduction in net energy
- Optimal speed profiles: 5-15% reduction

---


## 11. Safety Protocols

### 11.1 Safety Integrity Levels (SIL)

Railway safety systems follow IEC 61508 and EN 50129 standards.

#### 11.1.1 SIL Definitions

**SIL 4 (Highest):**
- Probability of failure: 10⁻⁹ to 10⁻⁸ per hour
- Risk: Catastrophic (multiple fatalities)
- Examples: ATP, ATO emergency braking, interlocking

**SIL 3:**
- Probability of failure: 10⁻⁸ to 10⁻⁷ per hour
- Risk: Critical (single fatality)
- Examples: Level crossing protection, platform edge detection

**SIL 2:**
- Probability of failure: 10⁻⁷ to 10⁻⁶ per hour
- Risk: Marginal (serious injury)
- Examples: Passenger information, CCTV

**SIL 1:**
- Probability of failure: 10⁻⁶ to 10⁻⁵ per hour
- Risk: Minor injury
- Examples: Comfort functions, lighting

**SIL 0:**
- No safety function
- Examples: Entertainment, Wi-Fi

#### 11.1.2 Safety Requirements

**Redundancy:**
- SIL 4: N+2 redundancy (triple redundant with voting)
- SIL 3: N+1 redundancy (dual redundant with comparison)
- SIL 2: Single channel with monitoring

**Diversity:**
- Different hardware platforms
- Different software implementations
- Different physical principles

**Fail-Safe Defaults:**
```
if (system_failure) {
  state = SAFE_STATE;
  apply_emergency_brakes();
  set_signals_to_red();
  lock_points();
  alert_control_center();
}
```

### 11.2 RAMS (Reliability, Availability, Maintainability, Safety)

#### 11.2.1 Reliability

**Mean Time Between Failures (MTBF):**
```
MTBF_system = 1 / (Σ(1/MTBF_i))
```

Where `i` = individual component

**Target MTBF:**
- Safety-critical: > 10⁹ hours
- Operational: > 10⁶ hours

#### 11.2.2 Availability

**Definition:**
```
A = MTBF / (MTBF + MTTR)
```

Where:
- `MTBF` = Mean Time Between Failures
- `MTTR` = Mean Time To Repair

**Target Availability:**
- Mainline railway: > 99.5%
- Metro: > 99.9%
- Critical systems: > 99.99%

#### 11.2.3 Maintainability

**Mean Time To Repair (MTTR):**
- Critical systems: < 1 hour
- Important systems: < 4 hours
- Non-critical: < 24 hours

**Maintenance Strategy:**
- Predictive: Condition monitoring, trend analysis
- Preventive: Scheduled maintenance
- Corrective: Repair after failure (non-critical only)

### 11.3 Emergency Procedures

#### 11.3.1 Emergency Braking

**Trigger Sources:**
- Driver emergency button
- Passenger emergency button
- ATP automatic intervention
- Control center command
- Communication loss
- Obstacle detection

**Response:**
```
1. Immediate brake application (< 1 second)
2. Alert control center
3. Activate emergency lighting
4. Open emergency doors (after stop)
5. Broadcast emergency message
6. Deploy emergency services
```

#### 11.3.2 Evacuation Procedures

**Train Evacuation:**
```
1. Stop train safely (emergency brakes if needed)
2. Assess situation (fire, collision, medical emergency)
3. Announce evacuation instructions
4. Disable traction power (if safe)
5. Open all doors (manual if necessary)
6. Direct passengers to safe area
7. Account for all passengers
8. Coordinate with emergency services
```

**Tunnel Evacuation:**
- Proceed to nearest station if possible
- Emergency walkways available
- Emergency lighting (battery backup > 90 minutes)
- Emergency phones every 250 meters
- Ventilation control for smoke management

---



## A.1 Railway-dynamics protocol

The railway-dynamics protocol layer carries traction, braking, and
energy-recovery exchanges between rolling stock and trackside
infrastructure. Envelopes carry per-axle power, braking effort, and
regenerative-braking energy returned to the supply.

## A.2 Safety protocols

Safety protocols span: emergency-brake propagation (must reach all
axles within a documented latency budget), fire-detection
broadcast, and obstacle-detection alerts. Every safety protocol
exchange is signed and audited so an investigator can reconstruct
the sequence of any incident.

## A.3 Communication-based train control protocol

CBTC track-circuit-free signalling exchanges train position and
movement authority over a wireless backbone. The protocol envelopes
carry train position with documented uncertainty and the
trackside-issued authority with a fixed expiry.

## A.4 Replay defence and audit

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce
cache. Audit envelopes are written to an append-only log replicated
across at least two storage backends with retention sized to the
regulatory requirement (typically 5 years for major operators,
10 years for high-speed lines).

## A.5 Cross-operator federation

International high-speed services (Eurostar, ICE, TGV cross-border)
require cross-operator federation. The federation envelope reuses
WIA-SOCIAL Phase 3 §5 receipt shape; trust lists include both the
home and foreign operator, with the regulator subscribed as an
auditing peer.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/railway-system/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-railway-system-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/railway-system-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/railway-system.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.


## Z.1 Worked deployment trace

A typical first-90-day deployment of this standard at a host
operator follows the trace below:

```
Day 0    : Reference container stood up in dev environment.
Day 1-2  : Conformance suite passes against reference container.
Day 3-7  : Backend bridge implemented for the host's primary tool.
Day 8-10 : Conformance suite passes against bridged backend.
Day 11-15: Audit log replication wired up; first envelope chain audited.
Day 16-20: First federation peer onboarded; trust list cadence verified.
Day 21-30: Federation expanded to 3-5 peers; cross-peer audit verified.
Day 31-60: Production traffic shadow-routed through new stack.
Day 61-90: Cutover from legacy to new stack; legacy retained as
           fallback for the deprecation window.
```

The 90-day timeline accommodates conformance-suite passes,
operations-team training, and the regulator-notification cadence
typical for high-stakes deployments. Lighter deployments (small
operators, prototypes) compress this to 30 days.

## Z.2 Operations runbook excerpt

Day-to-day operations focus on three signals: (a) audit-log
replication lag — alarm if either replica falls more than 60s
behind the primary; (b) trust-list freshness — alarm 7 days
before any peer's signed list expires; (c) replay-cache footprint
— alarm if cache memory exceeds 80% of the documented budget.

The runbook also covers incident response: rotating signing keys
on suspected compromise, replaying the seen-nonce cache from
persistent storage on standby failover, and re-issuing federation
handshakes when the primary controller has been offline for
longer than the seen-nonce window.
