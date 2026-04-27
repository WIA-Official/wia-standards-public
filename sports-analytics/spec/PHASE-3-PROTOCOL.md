# PHASE 3 — Protocol

> Sports-analytics ingestion protocols: video analytics pipeline,
> data-collection and tracking infrastructure (wearables,
> on-pitch sensors, optical tracking), and the team-performance
> protocol that aggregates per-player envelopes into team-level
> indicators.

## 6. Team Performance Metrics

### 6.1 Team Rating Systems

#### 6.1.1 Overall Team Strength

```
Team_Strength (0-100) = (Squad_Quality × 0.4) + (Team_Performance × 0.35) +
                        (Tactical_Quality × 0.15) + (Squad_Depth × 0.10)

Squad_Quality: Average PPR of best starting XI
Team_Performance: Points per game × 3
Tactical_Quality: Organization, pressing, build-up scores
Squad_Depth: Quality drop-off from starters to substitutes
```

#### 6.1.2 Synergy Score

```
Synergy (0-100) = (Pass_Network_Strength × 0.35) +
                  (Position_Harmony × 0.30) +
                  (Communication_Index × 0.20) +
                  (Chemistry × 0.15)

Pass_Network_Strength:
  = Average eigenvector centrality of passing graph
  High value = Well-connected team

Position_Harmony:
  = 1 - Σ(Position_Overlap) / Optimal_Spacing
  High value = Good spacing and coverage

Communication_Index:
  = Successful_Tactical_Adjustments / Required_Adjustments

Chemistry:
  = (Shared_Game_Time × Understanding × Compatibility)
  Years_Together_Factor: +5% per season
```

#### 6.1.3 Tactical Flexibility

```
Flexibility = Formations_Used × Effectiveness_Variance^-1

High flexibility: Multiple successful formations
Low variance: Consistent performance across formations

Adaptability_Score = (Plan_A_Success × 0.4) + (Plan_B_Success × 0.6)

Plan_A: Primary formation/tactic
Plan_B: Alternative approaches when behind or ahead
```

### 6.2 Attack & Defense Ratings

#### 6.2.1 Attack Rating

```
Attack (0-100) = (Goals_Scored × 0.30) + (xG × 0.25) +
                 (Shot_Quality × 0.20) + (Creativity × 0.15) +
                 (Conversion × 0.10)

Goals_Scored: Actual goals per game
xG: Expected goals per game
Shot_Quality: Average xG per shot
Creativity: Key passes, progressive actions
Conversion: Goals / xG (finishing efficiency)

League_Adjusted = Attack_Rating × (League_Strength / 100)
```

#### 6.2.2 Defense Rating

```
Defense (0-100) = (100 - Goals_Allowed) × 0.30 + (xGA_Prevention × 0.25) +
                  (Defensive_Actions × 0.20) + (Organization × 0.15) +
                  (Pressing × 0.10)

Goals_Allowed: Per game (inverted)
xGA_Prevention: 100 - (xG allowed per game × 10)
Defensive_Actions: Tackles, interceptions, blocks
Organization: Compactness, shape maintenance
Pressing: PPDA, regains in final third

Elite_Defense: >80 rating
Average: 50-70 rating
Poor: <50 rating
```

#### 6.2.3 Transition Quality

```
Transition_Attack = Speed_to_Shot × Effectiveness

Speed_to_Shot: Seconds from regaining possession to shot
  Fast transition: <8 seconds
  Moderate: 8-15 seconds
  Slow: >15 seconds

Effectiveness: (Counter_Attacks_to_Goals / Total_Counters)

Transition_Defense = Recovery_Speed × Prevention_Rate

Recovery_Speed: Time to reestablish defensive shape
Prevention_Rate: Counters prevented / Possession losses
```

### 6.3 Possession Metrics

#### 6.3.1 Possession Value

```
Possession_Value = Effective_Possession × Territory_Control × Outcome

Effective_Possession:
  = (Final_Third_Touches / Total_Touches) × Possession_Percentage

Territory_Control:
  = (0.3 × Own_Third + 0.5 × Middle_Third + 0.7 × Final_Third)

Outcome:
  = Shot_Creation_Rate × Expected_Threat

Expected_Threat (xT):
  Value of moving ball from one zone to another
  Based on probability of scoring from each pitch zone
```

#### 6.3.2 Passing Networks

```
Network_Strength = Centrality_Distribution + Connection_Quality

Centrality_Metrics:
  - Degree: Number of connections
  - Betweenness: Bridge between other players
  - Eigenvector: Connected to important players
  - PageRank: Importance in passing flow

Connection_Quality:
  = (Successful_Passes / Attempted) × Pass_Progressiveness

Key_Playmaker: Highest eigenvector centrality
Team_Hub: Highest betweenness centrality
```

### 6.4 Set Piece Effectiveness

```
Set_Piece_Rating = (Attacking_SP × 0.5) + (Defensive_SP × 0.5)

Attacking_SP:
  Goals_From_Set_Pieces / Total_Set_Pieces × 100

  Types: Corners, free kicks, penalties, throw-ins

  Elite: >15% conversion
  Average: 8-12%
  Poor: <8%

Defensive_SP:
  Goals_Conceded_From_SP / Opponent_Set_Pieces × 100 (inverted)

  Elite: <5% conceded
  Average: 8-12%
  Poor: >12%
```

---


## 7. Video Analytics

### 7.1 Automated Event Detection

#### 7.1.1 Computer Vision Pipeline

```
Video Frame → Object Detection → Tracking → Event Classification → Output

Object Detection:
  - Players (team A, team B, referee)
  - Ball
  - Goal posts
  - Field lines and markings

Model: YOLO v8 or Faster R-CNN
Frame rate: 25-30 FPS
Resolution: 1080p minimum

Tracking:
  - Multi-object tracking (MOT)
  - Player identity maintenance
  - Ball trajectory prediction

Algorithm: DeepSORT or ByteTrack
Accuracy: >95% player tracking
Ball detection: >90% in-play
```

#### 7.1.2 Event Recognition

```
Event_Classification = CNN(Frame_Sequence) → Event_Type + Confidence

Events detected:
  Goals: >99% accuracy
  Shots: >95% accuracy
  Passes: >90% accuracy
  Tackles: >85% accuracy
  Fouls: >80% accuracy (requires context)
  Offsides: >88% accuracy

Model Architecture:
  - 3D CNN for temporal features
  - ResNet backbone for spatial features
  - LSTM for sequence modeling

Training Data: 500,000+ labeled events
Inference: Real-time (<100ms per frame)
```

#### 7.1.3 Player Identification

```
Player_ID = Jersey_Number_Recognition + Face_Recognition + Gait_Analysis

Jersey_Number_OCR:
  - Character recognition on jersey
  - Accuracy: >95% when visible
  - Fallback: Team position + movement patterns

Face_Recognition:
  - When player faces camera
  - Cross-reference with player database
  - Accuracy: >98% with clear view

Gait_Analysis:
  - Unique movement signatures
  - Body proportions and running style
  - Useful when jersey obscured
```

### 7.2 Tactical Video Analysis

#### 7.2.1 Formation Visualization

```
Formation_Overlay = Average_Positions + Convex_Hull

Average_Positions:
  For each player: (ΣX / N, ΣY / N) over time window

Convex_Hull:
  Smallest polygon containing all players
  Indicates team compactness and shape

Heat_Map:
  Grid: 20m × 15m cells
  Value: Time spent in each cell
  Visualization: Color gradient (cool → hot)

Output:
  - Formation shape (4-4-2, etc.)
  - Defensive/attacking transitions
  - Player movement tendencies
```

#### 7.2.2 Space Control Analysis

```
Voronoi_Diagram: Partition pitch based on player proximity

Each cell represents area controlled by a player
Cell_Area = Space controlled
Border_Proximity = Defensive pressure

Pitch_Control_Model:
  P(team_controls_space) = f(player_positions, velocities, distances)

Dominant_Regions:
  Team_A_Area / Total_Pitch_Area

Space_Creation:
  Detect large Voronoi cells (free space)
  Track exploitation of these spaces
```

#### 7.2.3 Pressing Behavior

```
Press_Detection:
  - Count defenders within 5m radius of ball
  - Measure time to close down ball carrier
  - Identify pressing triggers (backward pass, heavy touch)

Press_Intensity = Defenders_Nearby × Closing_Speed

Coordinated_Press:
  - Multiple players converging simultaneously
  - Covering passing lanes
  - Compressing space

Counter-Press:
  - Immediate press after losing possession
  - Time threshold: <3 seconds
  - Player involvement: ≥3 players
```

### 7.3 Performance Visualization

#### 7.3.1 Heat Maps

```
Heat_Map(player, match):
  Grid_Resolution: 5m × 5m
  Value(cell) = Time_in_Cell + Weighted_Actions

  Action_Weights:
    Goal: +10
    Assist: +7
    Shot: +3
    Key_Pass: +2
    Touch: +1

Normalization: Per 90 minutes
Overlay: On pitch diagram with position labels
```

#### 7.3.2 Pass Maps

```
Pass_Map:
  Nodes: Player average positions
  Edges: Passes between players

  Edge_Thickness: Number of passes
  Edge_Color: Completion rate
    Green: >85%
    Yellow: 70-85%
    Red: <70%

  Node_Size: Total passes
  Node_Color: Team color

Progressive_Passes:
  Highlighted with arrows showing direction and distance
```

#### 7.3.3 Shot Maps

```
Shot_Map:
  Location: (X, Y) coordinates on pitch
  Marker_Size: xG value (bigger = higher xG)
  Marker_Color:
    Green: Goal
    Yellow: On target
    Red: Off target
    Gray: Blocked

  Overlay: Goalkeeper position, defenders

Aggregate_Shot_Map:
  Hexbin heatmap of all shots from location
  Shows preferred shooting zones
```

### 7.4 Highlight Generation

#### 7.4.1 Key Moment Detection

```
Importance_Score(event) = Event_Value × Context_Multiplier × Rarity

Event_Value:
  Goal: 1.0
  xG > 0.3 shot: 0.7
  Save: 0.6
  Red card: 0.9
  Penalty: 0.8

Context_Multiplier:
  Winning goal: ×1.5
  Late in match: ×(1 + (90-Minute)/90)
  Derby/rivalry: ×1.3

Rarity:
  Long-range goal: ×1.4
  Solo run: ×1.3
  Bicycle kick: ×1.6

Top_N_Moments: Sort by importance and select highlights
```

#### 7.4.2 Automatic Clip Generation

```
Clip_Window:
  Start: 5 seconds before event
  Event: Main action
  End: 3 seconds after (capture celebration/reaction)

Replay_Inclusion:
  High importance events: Add 1-2 angles
  Goals: Multiple angles + celebration

Transitions:
  Fade: Between different moments
  Cut: For continuous action

Music_Sync:
  Beat detection in background track
  Align cuts with musical beats
```

---


## 9. Data Collection & Tracking

### 9.1 Data Sources

#### 9.1.1 Wearable Technology

```
GPS_Trackers:
  Frequency: 10 Hz (professional) or 18 Hz (elite)
  Metrics:
    - Position (X, Y coordinates)
    - Velocity (m/s)
    - Acceleration (m/s²)
    - Distance (total, zones)

  Accuracy: ±0.5m position, ±2% distance

Heart_Rate_Monitors:
  Method: Chest strap (ECG) or optical (PPG)
  Sampling: 1 Hz
  Metrics:
    - Instantaneous heart rate
    - Heart rate zones
    - Heart rate variability (HRV)

  Accuracy: ±1-2 bpm (chest), ±5 bpm (optical)

Accelerometers:
  Axes: 3-axis (X, Y, Z)
  Sampling: 100 Hz
  Metrics:
    - PlayerLoad™
    - Impacts and collisions
    - Jump height (flight time method)
```

#### 9.1.2 Optical Tracking Systems

```
Camera_Based_Tracking:

  Setup:
    Cameras: 6-12 around stadium
    Coverage: Full pitch from multiple angles
    Frame rate: 25-30 FPS
    Resolution: 1080p or 4K

  Processing:
    1. Camera calibration (homography)
    2. Background subtraction
    3. Player detection (YOLO, etc.)
    4. Multi-camera fusion
    5. Tracking (Kalman filter, DeepSORT)

  Output:
    - Player positions (X, Y, timestamp)
    - Ball position and possession
    - Team assignments
    - Player identities

  Accuracy: ±20cm position, >95% tracking continuity
```

#### 9.1.3 Event Tagging Systems

```
Manual_Tagging:

  Tools: Video analysis software (Hudl, Wyscout, InStat)

  Events_Tagged:
    - Passes (type, outcome)
    - Shots (location, xG)
    - Tackles and interceptions
    - Fouls and cards
    - Set pieces
    - Contextual notes

  Tagger_Training: >95% inter-rater reliability
  Real-time vs Post-match: Both supported

Semi-Automated:
  ML models suggest events, human verifies
  Speeds up process by 60-70%
```

### 9.2 Data Quality & Validation

#### 9.2.1 Quality Checks

```
Position_Data_Validation:

  Sanity_Checks:
    - Speed_Max: <37 km/h (Usain Bolt ≈ 44 km/h)
    - Acceleration_Max: <10 m/s²
    - On_Pitch: X in [0, 105m], Y in [0, 68m]
    - Ball_Possession: Only one team at a time

  Outlier_Detection:
    Z-Score method: |Z| > 3 flagged
    IQR method: Outside [Q1-1.5×IQR, Q3+1.5×IQR]

  Missing_Data:
    Gap_Filling: Linear interpolation for gaps <2s
    Larger_Gaps: Mark as missing, exclude from analysis

Event_Data_Validation:

  Consistency_Checks:
    - Goals ≤ Shots on target
    - Passes = Receptions (team-level)
    - Cards have associated fouls
    - Substitutions ≤ maximum allowed

  Cross-Reference:
    Match official stats with provider data
    Discrepancy threshold: ±5%
```

#### 9.2.2 Data Standardization

```
Coordinate_Systems:

  Normalization: All pitches to 105m × 68m
  Origin: Center of pitch (52.5m, 34m)
  Axes: X (length), Y (width)

  Transform:
    X_normalized = X_actual × (105 / Actual_Length)
    Y_normalized = Y_actual × (68 / Actual_Width)

Time_Synchronization:

  Reference: Match clock (00:00 to 90:00+)
  Sources: GPS timestamp, video timestamp, event timestamp

  Sync_Method:
    Align to kickoff event (t=0)
    Cross-check with known events (goals, subs)
    Drift correction if needed

Metric_Units:

  Distance: meters (m), kilometers (km)
  Speed: km/h (convert from m/s: ×3.6)
  Time: seconds (s), minutes (min)
  Acceleration: m/s²
  Force: Newtons (N), multiples of body weight
```

### 9.3 Data Storage & Management

#### 9.3.1 Database Schema

```sql
-- Players table
CREATE TABLE players (
  player_id VARCHAR(20) PRIMARY KEY,
  name VARCHAR(100),
  position VARCHAR(20),
  date_of_birth DATE,
  height_cm INT,
  weight_kg DECIMAL(4,1),
  nationality VARCHAR(50)
);

-- Matches table
CREATE TABLE matches (
  match_id VARCHAR(20) PRIMARY KEY,
  home_team_id VARCHAR(20),
  away_team_id VARCHAR(20),
  date DATE,
  competition VARCHAR(50),
  venue VARCHAR(100),
  attendance INT
);

-- Events table
CREATE TABLE events (
  event_id VARCHAR(30) PRIMARY KEY,
  match_id VARCHAR(20),
  timestamp INT, -- milliseconds from kickoff
  event_type VARCHAR(30),
  player_id VARCHAR(20),
  team_id VARCHAR(20),
  x_coordinate DECIMAL(5,2),
  y_coordinate DECIMAL(5,2),
  outcome VARCHAR(20),
  metadata JSON,
  FOREIGN KEY (match_id) REFERENCES matches(match_id),
  FOREIGN KEY (player_id) REFERENCES players(player_id)
);

-- Tracking data (high frequency)
CREATE TABLE tracking (
  match_id VARCHAR(20),
  timestamp INT,
  player_id VARCHAR(20),
  x DECIMAL(5,2),
  y DECIMAL(5,2),
  speed DECIMAL(4,2),
  acceleration DECIMAL(4,2),
  heart_rate INT,
  PRIMARY KEY (match_id, timestamp, player_id),
  FOREIGN KEY (match_id) REFERENCES matches(match_id),
  FOREIGN KEY (player_id) REFERENCES players(player_id)
);

-- Player statistics (aggregated)
CREATE TABLE player_stats (
  player_id VARCHAR(20),
  match_id VARCHAR(20),
  goals INT,
  assists INT,
  shots INT,
  passes INT,
  pass_accuracy DECIMAL(5,2),
  distance_km DECIMAL(4,2),
  sprints INT,
  tackles INT,
  ppr DECIMAL(5,2),
  PRIMARY KEY (player_id, match_id),
  FOREIGN KEY (player_id) REFERENCES players(player_id),
  FOREIGN KEY (match_id) REFERENCES matches(match_id)
);
```

#### 9.3.2 Data Retention & Archiving

```
Retention_Policy:

  Real-time_Data (tracking, live events):
    - Hot storage: 7 days (SSD)
    - Warm storage: 90 days (HDD)
    - Cold storage: 5+ years (archive)

  Aggregated_Stats:
    - Hot storage: 2 seasons
    - Warm storage: 10 years
    - Cold storage: Indefinite

  Video:
    - Full matches: 3 seasons (compressed)
    - Highlights: Indefinite
    - Raw footage: 30 days

Backup_Strategy:
  - Daily incremental backups
  - Weekly full backups
  - Geographic redundancy (3 locations)
  - Versioning: Keep last 30 daily, 12 weekly
```

---


