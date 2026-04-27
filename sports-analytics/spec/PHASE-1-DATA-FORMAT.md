# PHASE 1 — Data Format

> Sports-analytics canonical envelopes: player profile and
> performance schemas, the architecture map that hosts them,
> and the data-format / interchange shape used across every
> subsystem.

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for sports analytics, providing standardized methods for player performance tracking, game analysis, predictive modeling, team metrics, video analytics, and sports data interchange. The standard enables seamless integration across the sports industry ecosystem, from professional teams to amateur clubs.

### 1.2 Scope

The standard covers:
- Player performance metrics and skill ratings
- Real-time game analysis and tactical insights
- Predictive modeling for match outcomes and player development
- Team performance metrics and formation analysis
- Video analytics with automated event detection
- Injury prevention and biomechanical analysis
- Fan engagement and fantasy sports integration
- Scouting intelligence and talent evaluation
- Sports data interchange formats and APIs

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize sports analytics by making advanced data science accessible to teams at all levels, improve athlete performance and safety through evidence-based training, enhance fan engagement through deeper understanding, promote fair competition, and advance sports science for the benefit of all athletes, coaches, and fans worldwide.

### 1.4 Terminology

- **PPR**: Player Performance Rating (0-100 scale)
- **xG**: Expected Goals - probability-based goal metric
- **xA**: Expected Assists - probability-based assist metric
- **VAEP**: Valuing Actions by Estimating Probabilities
- **PPDA**: Passes Allowed Per Defensive Action
- **Heat Map**: Spatial visualization of player positioning
- **Pass Network**: Graph showing passing connections
- **Synergy Score**: Team chemistry and coordination metric
- **Injury Risk Index**: Predictive injury probability (0-10)
- **Win Probability**: Real-time match outcome likelihood
- **Biomechanical Analysis**: Movement pattern and efficiency study
- **Tactical Camera**: Bird's-eye view for formation analysis
- **Event Detection**: Automated recognition of match events
- **Progressive Action**: Movement toward opponent's goal
- **Press Resistance**: Ability to maintain possession under pressure

---


## 2. Sports Analytics Architecture

### 2.1 System Overview

The sports analytics ecosystem consists of:

```
Data Collection → Processing → Analysis → Insights → Visualization → Action
       ↓              ↓           ↓          ↓            ↓            ↓
    Sensors      Tracking    ML Models   Reports    Dashboards   Decisions
    Video        Events      Statistics  Alerts     Real-time    Training
    Manual       Stats       Predictions Patterns   Historical   Tactics
```

### 2.2 Component Hierarchy

#### 2.2.1 Data Collection Layer
- **Wearable Sensors**: GPS trackers, heart rate monitors, accelerometers
- **Optical Tracking**: Camera-based player and ball tracking
- **Manual Tagging**: Human-annotated events and actions
- **Broadcast Feed**: Live match video and metadata
- **External Data**: Weather, venue, historical records

#### 2.2.2 Processing Layer
- **Event Detection**: Automated recognition of goals, passes, tackles
- **Tracking System**: Player coordinates, speed, acceleration
- **Data Validation**: Quality checks and error correction
- **Normalization**: Standardization across different sources
- **Aggregation**: Summary statistics and rolling averages

#### 2.2.3 Analytics Layer
- **Descriptive Analytics**: What happened (stats, summaries)
- **Diagnostic Analytics**: Why it happened (patterns, correlations)
- **Predictive Analytics**: What will happen (forecasts, probabilities)
- **Prescriptive Analytics**: What should happen (recommendations)

#### 2.2.4 Presentation Layer
- **Dashboards**: Real-time and historical visualization
- **Reports**: Automated match reports and player summaries
- **Alerts**: Notifications for key events and thresholds
- **API**: Data access for third-party applications
- **Mobile Apps**: Coach and player-facing interfaces

### 2.3 Data Flow Architecture

```
Match Day:
  Live Video → Event Detection → Real-time Stats → Dashboard
  Wearables → Position Tracking → Physical Metrics → Alerts

Post-Match:
  Raw Data → Processing → Analysis → Report Generation
           → ML Training → Model Updates → Predictions

Long-term:
  Historical DB → Trend Analysis → Insights → Strategic Planning
                → Player Development → Recruitment → Team Building
```

### 2.4 Integration Points

- **Club Management Systems**: Player contracts, training schedules
- **Medical Systems**: Injury records, treatment plans
- **Scouting Platforms**: Player discovery, transfer market
- **Broadcasting**: Live graphics, commentary support
- **Fan Apps**: Engagement, fantasy sports, social media
- **League Systems**: Competition management, fair play

---


## 3. Player Performance Systems

### 3.1 Player Performance Rating (PPR)

#### 3.1.1 Formula

```
PPR (0-100) = (Skills × 0.4) + (Impact × 0.3) + (Consistency × 0.2) + (Fitness × 0.1)
```

**Components:**

**Skills (0-100):**
```
Skills = Σ(Sport_Specific_Metrics[i] × Weight[i]) / Total_Weight

Football: (Passing × 0.25) + (Shooting × 0.20) + (Dribbling × 0.20) +
          (Defending × 0.20) + (Physical × 0.15)

Basketball: (Scoring × 0.30) + (Playmaking × 0.25) + (Rebounding × 0.20) +
            (Defense × 0.15) + (Efficiency × 0.10)
```

**Impact (0-100):**
```
Impact = (Goals × 10) + (Assists × 7) + (Key_Actions × 5) + (Game_Winners × 15)

Where Key_Actions varies by sport:
  Football: Tackles won, interceptions, clearances, saves
  Basketball: Steals, blocks, clutch plays
  Baseball: RBIs, stolen bases, defensive plays
```

**Consistency (0-100):**
```
Consistency = 100 - (StdDev(Performance) / Mean(Performance) × 100)

Rolling average over last 10 games
Penalty for high variance in performance
```

**Fitness (0-100):**
```
Fitness = (Availability × 0.5) + (Physical_Condition × 0.5)

Availability = Games_Played / Games_Available × 100
Physical_Condition = (Distance × Speed × Accelerations) / Position_Average
```

#### 3.1.2 Position-Specific Adjustments

**Football:**
- **Goalkeeper**: Saves, clean sheets, distribution (different formula)
- **Defender**: Defensive actions weighted higher (0.35 vs 0.20)
- **Midfielder**: Passing and ball progression emphasized
- **Forward**: Goals and shooting weighted higher (0.40 vs 0.20)

**Basketball:**
- **Point Guard**: Assists and ball handling emphasized
- **Shooting Guard**: Scoring efficiency weighted higher
- **Center**: Rebounding and interior defense prioritized
- **Forward**: Versatility across multiple categories

#### 3.1.3 Context Normalization

```
Adjusted_PPR = Base_PPR × League_Factor × Competition_Factor × Era_Factor

League_Factor:
  Top 5 Leagues: 1.0
  Mid-tier: 0.85
  Lower tier: 0.70

Competition_Factor:
  Champions League: 1.15
  Domestic League: 1.0
  Cup matches: 0.95

Era_Factor: Accounts for rule changes and playing style evolution
```

### 3.2 Skill Breakdown System

#### 3.2.1 Technical Skills (Football)

**Passing:**
```
Pass_Score = (Completion_Rate × 0.4) + (Progressive_Passes × 0.3) +
             (Key_Passes × 0.2) + (Creativity × 0.1)

Progressive_Pass: Advances ball ≥10m toward goal
Key_Pass: Pass leading to shot
Creativity: Expected assist value of pass selections
```

**Shooting:**
```
Shoot_Score = (xG_per_Shot × 0.4) + (Conversion_Rate × 0.3) +
              (Shot_Placement × 0.2) + (Shot_Power × 0.1)

Shot_Placement: Distance from center of goal
Shot_Power: Ball velocity (km/h)
```

**Dribbling:**
```
Dribble_Score = (Success_Rate × 0.5) + (Progressive_Carries × 0.3) +
                (Tight_Space_Control × 0.2)

Progressive_Carry: Ball advancement ≥5m
Tight_Space: Ball retention under pressure
```

**Defending:**
```
Defense_Score = (Tackle_Success × 0.35) + (Interceptions × 0.25) +
                (Positioning × 0.25) + (Aerial_Duels × 0.15)

Positioning: Space control and coverage effectiveness
Aerial_Duels: Headers won/contested
```

#### 3.2.2 Physical Metrics

**Speed & Acceleration:**
```
Top_Speed: Maximum velocity (km/h)
Sprint_Distance: Total distance at >24 km/h
Accelerations: Count of rapid speed increases (>3 m/s²)
Decelerations: Count of rapid speed decreases (<-3 m/s²)

Speed_Score = (Top_Speed / Position_Max × 50) +
              (Sprint_Distance / Position_Avg × 30) +
              (Accelerations / Position_Avg × 20)
```

**Stamina & Endurance:**
```
Total_Distance: km covered per match
High_Intensity: Distance at >19.8 km/h
Work_Rate: Distance per minute played

Stamina_Score = (Total_Distance / Position_Avg × 40) +
                (High_Intensity / Position_Avg × 40) +
                (Performance_Maintenance × 20)

Performance_Maintenance: Last 15 min vs First 15 min metric comparison
```

**Power & Strength:**
```
Jump_Height: cm (from accelerometer)
Duels_Won: Physical contests won percentage
Power_Plays: High-intensity actions (tackles, jumps, sprints)

Power_Score = (Jump_Height / Position_Max × 35) +
              (Duels_Won × 40) +
              (Power_Plays / Position_Avg × 25)
```

#### 3.2.3 Mental Attributes

**Decision Making:**
```
Decision_Score = (Pass_Selection × 0.4) + (Shot_Selection × 0.3) +
                 (Risk_Management × 0.2) + (Adaptability × 0.1)

Pass_Selection: Expected value of passing choices
Shot_Selection: xG optimization (shot when xG > threshold)
Risk_Management: Appropriate risk in game situations
Adaptability: Performance across different tactics/formations
```

**Game Intelligence:**
```
Intelligence = (Positioning × 0.35) + (Anticipation × 0.30) +
               (Pattern_Recognition × 0.20) + (Communication × 0.15)

Positioning: Heat map correlation with optimal positions
Anticipation: Successful interceptions per attempt
Pattern_Recognition: Response to opponent patterns
Communication: On-field leadership indicators
```

### 3.3 Player Comparison System

#### 3.3.1 Multi-Dimensional Comparison

```typescript
interface PlayerComparison {
  players: Player[];
  metrics: Metric[];
  normalization: 'absolute' | 'percentile' | 'z-score';
  visualization: 'radar' | 'bar' | 'scatter';
  filters: {
    league?: string[];
    position?: string[];
    age?: { min: number; max: number };
    minutes?: number; // minimum minutes played
  };
}

// Percentile ranking
Percentile = (Players_Below / Total_Players) × 100

// Z-score normalization
Z_Score = (Value - Mean) / Standard_Deviation
```

#### 3.3.2 Similar Player Detection

```
Similarity_Score = 1 - (Euclidean_Distance / Max_Distance)

Where distance calculated across normalized metrics:
  Physical: Speed, stamina, power
  Technical: Passing, shooting, dribbling
  Tactical: Positioning, decision making
  Statistical: Goals, assists, defensive actions

Weight factors:
  Position-specific metrics: 2.0×
  General metrics: 1.0×

Minimum similarity threshold: 0.75 (75%)
```

### 3.4 Player Development Tracking

#### 3.4.1 Growth Curves

```
Expected_Rating(age) = Peak_Rating × Growth_Function(age, position)

Growth_Function:
  Age < Peak_Age:
    Growth_Rate = (Peak_Rating - Current) / (Peak_Age - Current_Age)

  Age ≥ Peak_Age:
    Decline_Rate = Decline_Factor × (Age - Peak_Age)

Peak_Age by position:
  Goalkeeper: 30-32
  Defender: 28-30
  Midfielder: 27-29
  Forward: 26-28
```

#### 3.4.2 Potential Rating

```
Potential (0-100) = Current_Rating + Growth_Margin

Growth_Margin = f(Age, Talent, Development_Rate, Training_Quality)

Talent: Natural ability indicators
Development_Rate: Historical improvement trajectory
Training_Quality: Coaching, facilities, playing time

Confidence Interval: ±5 rating points
```

#### 3.4.3 Training Impact Measurement

```
Training_Effect = (Post_Performance - Pre_Performance) / Time_Period

Effectiveness_Score = Actual_Improvement / Expected_Improvement

Training_ROI = (Performance_Gain × Market_Value_Increase) / Training_Cost
```

---


## 13. Data Formats & Standards

### 13.1 JSON Event Format

```json
{
  "event_id": "EVT-12345-67890",
  "match_id": "MTH-2025-12345",
  "timestamp": 1275000,
  "minute": 21,
  "second": 15,
  "period": 1,
  "type": "pass",
  "team_id": "TEAM-001",
  "player_id": "PLR-12345",
  "position": { "x": 45.2, "y": 32.1 },
  "end_position": { "x": 68.3, "y": 28.4 },
  "outcome": "complete",
  "metadata": {
    "pass_type": "through_ball",
    "body_part": "right_foot",
    "progressive": true,
    "distance_m": 24.7,
    "duration_s": 1.2
  },
  "related_events": ["EVT-12345-67891"],
  "context": {
    "score": { "home": 0, "away": 0 },
    "possession_team": "TEAM-001"
  }
}
```

### 13.2 CSV Match Summary

```csv
match_id,date,home_team,away_team,home_score,away_score,competition,venue
MTH-2025-12345,2025-12-27,Team A,Team B,2,1,Premier League,Stadium A
```

### 13.3 Tracking Data Format

```json
{
  "match_id": "MTH-2025-12345",
  "frame_rate": 25,
  "pitch_dimensions": { "length": 105, "width": 68 },
  "frames": [
    {
      "timestamp": 0,
      "ball": { "x": 52.5, "y": 34.0, "z": 0.0, "possession": null },
      "players": [
        {
          "player_id": "PLR-001",
          "team_id": "TEAM-A",
          "x": 45.0,
          "y": 30.0,
          "speed": 5.2,
          "acceleration": 1.1
        }
      ]
    }
  ]
}
```

---


