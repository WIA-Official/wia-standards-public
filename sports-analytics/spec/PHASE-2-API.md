# PHASE 2 — API

> Sports-analytics REST surface and predictive-model contract:
> game analysis endpoints, predictive modeling, the underlying
> statistical models, and the API verbs that consumers call to
> drive coaching dashboards and broadcast graphics.

## 4. Game Analysis Framework

### 4.1 Real-Time Match Analysis

#### 4.1.1 Live Event Tracking

```typescript
interface MatchEvent {
  timestamp: number; // milliseconds from kickoff
  type: EventType;
  player: PlayerId;
  team: TeamId;
  location: { x: number; y: number }; // pitch coordinates
  outcome: 'success' | 'failure';
  context: MatchContext;
  relatedEvents: EventId[]; // linked events (pass → shot)
}

enum EventType {
  Pass, Shot, Goal, Save, Tackle, Interception, Foul,
  OffsideCard, Substitution, SetPiece, Dribble, Cross,
  Clearance, Block, Aerial, Turnover, Recovery
}
```

#### 4.1.2 Possession Analysis

```
Possession (%) = (Team_Touches / Total_Touches) × 100

Effective_Possession = Possession_in_Final_Third × Shot_Creation_Rate

Territory_Control:
  Own Third: 0-33% of pitch
  Middle Third: 33-67% of pitch
  Attacking Third: 67-100% of pitch

Possession_Value = (Final_Third × 3) + (Middle_Third × 2) + (Own_Third × 1)
```

#### 4.1.3 Expected Goals (xG)

```
xG = P(Goal | Shot_Characteristics)

Features:
  - Distance from goal (m)
  - Angle to goal (degrees)
  - Body part (foot, head, other)
  - Assist type (through ball, cross, set piece, rebound)
  - Number of defenders between ball and goal
  - Goalkeeper position
  - Pressure on shooter (defenders within 2m)
  - Game state (score difference, time remaining)
  - Shot type (open play, counter, set piece)

Model: Gradient Boosted Trees or Neural Network
Training: 500,000+ historical shots
Accuracy: 0.78-0.82 AUC-ROC
```

**xG Calibration:**
```
Calibration_Error = |Actual_Goals - Expected_Goals| / Total_Shots

Over/Under Performance:
  xG Overperformance: Actual > xG (good finishing)
  xG Underperformance: Actual < xG (poor finishing)

Regression to mean expected over time
```

#### 4.1.4 Expected Assists (xA)

```
xA = P(Assist | Pass_Characteristics) × P(Goal | Shot_From_Pass)

Pass_Quality:
  - Pass type and trajectory
  - Receiver position
  - Defender pressure on receiver
  - Space created
  - Pass accuracy and pace

xA per pass: 0.0 to 1.0
Season xA: Sum of all passes
```

### 4.2 Tactical Analysis

#### 4.2.1 Formation Detection

```
Formation = Cluster_Analysis(Player_Positions)

Algorithm:
  1. Remove goalkeeper from analysis
  2. Calculate average positions (excluding anomalies)
  3. Cluster into defensive/midfield/forward lines
  4. Classify formation based on line structure

Common formations:
  4-4-2: [4 defenders] [4 midfielders] [2 forwards]
  4-3-3: [4] [3] [3]
  3-5-2: [3] [5] [2]
  4-2-3-1: [4] [2 DM] [3 AM] [1 FW]

Confidence_Score = 1 - (Position_Variance / Expected_Variance)
```

#### 4.2.2 Pressing Analysis

```
PPDA = Passes_Allowed / Defensive_Actions

Defensive_Actions = Tackles + Interceptions + Fouls

Lower PPDA = More aggressive pressing
Higher PPDA = Deeper defensive block

Press_Intensity_Zones:
  High Press: PPDA < 8 (actions in attacking third)
  Medium Press: PPDA 8-12
  Low Block: PPDA > 12 (actions in defensive third)

Press_Success_Rate = Turnovers_Forced / Press_Attempts
```

#### 4.2.3 Build-Up Analysis

```
Build_Up_Patterns:
  - Direct: <3 passes from defense to attack
  - Short: 3-5 passes, progressive through midfield
  - Long: >5 passes, patient possession

Pass_Progressiveness = Distance_Advanced / Pass_Distance

Progressive_Pass: Advances ball ≥10m toward goal
Progressive_Carry: Ball advancement ≥5m via dribble

Build_Up_Efficiency = (Shots / Build_Up_Sequences) × Success_Rate
```

#### 4.2.4 Defensive Shape

```
Compactness:
  Horizontal = Average distance between widest players
  Vertical = Average distance between highest and lowest

Compact_Team: H < 40m, V < 30m
Stretched_Team: H > 45m, V > 35m

Defensive_Line_Height = Average Y-coordinate of back line
  High Line: >50m from own goal
  Mid Block: 30-50m
  Deep Block: <30m

Press_Coverage = (Defenders_Within_5m_of_Ball / Total_Defenders)
```

### 4.3 Momentum & Win Probability

#### 4.3.1 Win Probability Model

```
Win_Prob(t) = f(Score_Diff, Time_Remaining, Team_Strength, Momentum)

Score_Diff: Current goal difference
Time_Remaining: Minutes left
Team_Strength: Pre-match ratings
Momentum: Recent xG, possession, dangerous attacks

Initial (0:00):
  Win_Prob = 50% + (Team_Rating_Diff × 2%) + (Home_Advantage × 5%)

Update every minute based on:
  - Score changes: Major shifts
  - xG accumulation: Gradual changes
  - Cards: Red card = ±15-20% swing
  - Substitutions: Quality-based adjustments
```

#### 4.3.2 Momentum Calculation

```
Momentum(t) = Σ(Event_Value[i] × Time_Decay[i])

Event_Values:
  Goal: ±0.8
  Shot on target: ±0.3
  Dangerous attack: ±0.2
  Turnover in attacking third: ±0.15
  Save: ±0.25

Time_Decay = e^(-λt)
  λ = 0.1 (events decay over ~10 minutes)

Momentum range: -1.0 (fully against) to +1.0 (fully for)
```

### 4.4 Set Piece Analysis

#### 4.4.1 Corner Kick Analysis

```
Corner_Success_Rate = Goals / Corner_Kicks

Corner_Types:
  - Inswinger: Curve toward goal
  - Outswinger: Curve away from goal
  - Short: <20m pass to teammate
  - Direct: Shot attempt from corner

Delivery_Quality:
  - Target zone accuracy
  - Ball trajectory and pace
  - Defender clearance difficulty

Effectiveness = (Shots / Corners) × Conversion_Rate
```

#### 4.4.2 Free Kick Analysis

```
Free_Kick_xG = f(Distance, Angle, Wall_Quality, Taker_Skill)

Distance_Factor:
  Optimal: 18-22m (highest xG ~0.05-0.08)
  Too_Close: <15m (wall blocks view)
  Too_Far: >30m (too difficult)

Wall_Configuration:
  Players_In_Wall = Round(Distance_in_yards / 10)
  Wall_Positioning: Cover near post, goalkeeper covers far

Success_Rate = Goals / Direct_Free_Kicks
Elite takers: 5-8%
Average: 2-3%
```

#### 4.4.3 Throw-In Tactics

```
Throw_In_Retention = Retained_Possession / Total_Throws

Dangerous_Throw: In attacking third + leads to shot/chance

Long_Throw_Effectiveness:
  Distance: >20m into box
  Target: Aerial threat players
  Success: Shot created or retained possession
```

---


## 5. Predictive Modeling

### 5.1 Match Outcome Prediction

#### 5.1.1 Pre-Match Prediction Model

```
P(Home_Win) = f(Features)

Features (60+ variables):
  Team Strength:
    - ELO ratings
    - Recent form (last 5-10 matches)
    - Season performance metrics
    - Squad value and quality

  Head-to-Head:
    - Historical results
    - Goal difference in past meetings
    - Venue-specific performance

  Situational:
    - Home advantage
    - Rest days between matches
    - Injuries and suspensions
    - Manager experience
    - Motivation (league position, stakes)

  External:
    - Weather conditions
    - Referee strictness
    - Crowd size
    - Derby/rivalry factor

Model: XGBoost, Random Forest, or Neural Network
Accuracy: 68-75% for top leagues
Calibration: Brier score < 0.25
```

**ELO Rating System:**
```
New_Rating = Old_Rating + K × (Actual - Expected)

Expected = 1 / (1 + 10^((Opponent_Rating - Team_Rating) / 400))

K-Factor:
  Regular match: K = 20
  Cup match: K = 15
  Friendly: K = 10

Home advantage: +100 ELO points
```

#### 5.1.2 In-Play Prediction Updates

```
Live_Win_Prob(t) = Update(Pre_Match_Prob, Match_Events(0:t))

Update frequency: Every minute or after significant events

Event impacts:
  Goal: Immediate recalculation based on new score
  Red card: ±12-18% based on time remaining
  Injury to key player: ±3-5%
  xG accumulation: Gradual drift toward attacking team

Model confidence: Decreases with time (more uncertainty early)
```

#### 5.1.3 Score Prediction

```
Expected_Score(Home) = Base_Goals × Attack_Strength × Defense_Weakness

Attack_Strength = Team_Goals_Per_Game / League_Average
Defense_Weakness = Opponent_Conceded_Per_Game / League_Average

Poisson distribution for goal probabilities:
P(X = k) = (λ^k × e^-λ) / k!

Where λ = expected goals

Most_Likely_Score = Mode of joint probability distribution
```

### 5.2 Player Performance Prediction

#### 5.2.1 Next Match Performance

```
Predicted_Performance = f(Recent_Form, Opponent, Venue, Fatigue)

Recent_Form: Weighted average of last 5 performances
  Weights: [0.35, 0.25, 0.20, 0.12, 0.08] (most recent weighted highest)

Opponent_Adjustment:
  vs Top Teams: -8% to -15%
  vs Mid-table: ±2%
  vs Weak Teams: +5% to +12%

Venue_Factor:
  Home: +5% to +8%
  Away: -3% to -5%
  Neutral: 0%

Fatigue_Impact:
  < 3 days rest: -10% to -15%
  3-5 days: -3% to -5%
  6+ days: 0%
  > 10 days: +2% (fully rested)
```

#### 5.2.2 Season Projection

```
Projected_Season_Stats = Games_Remaining × Avg_Per_Game × Form_Factor

Avg_Per_Game = (Weighted_Historical + Current_Season) / 2

Form_Factor:
  Improving: 1.05 - 1.15
  Stable: 0.95 - 1.05
  Declining: 0.85 - 0.95

Confidence intervals: ±15% for counting stats
```

#### 5.2.3 Career Trajectory

```
Career_Arc = Peak_Performance × Age_Curve(age, position)

Age_Curve:
  Growth phase (17-Peak): Exponential improvement
  Peak phase (Peak±2): Plateau at maximum
  Decline phase (Peak+3 onwards): Gradual decrease

Peak_Age:
  Physical positions (wingers, fullbacks): 26-27
  Technical positions (midfielders): 28-29
  Experience positions (centerbacks, GK): 30-32

Decline_Rate:
  Physical attributes: -2% per year after peak
  Technical attributes: -0.5% per year after peak
  Mental attributes: +0.3% per year (experience gain)
```

### 5.3 Team Performance Forecasting

#### 5.3.1 League Position Prediction

```
Projected_Points = Current_Points + (Games_Left × PPG_Forecast)

PPG_Forecast = (Recent_PPG × 0.5) + (Season_PPG × 0.3) + (Quality_PPG × 0.2)

Recent_PPG: Last 10 games
Season_PPG: Current season average
Quality_PPG: Squad strength-based expectation

Monte Carlo simulation (10,000 iterations):
  - Probabilistic outcome for each remaining match
  - Aggregate results to final standings
  - Calculate position probabilities

Output:
  Expected_Position: Mean of simulations
  Position_Range: 80% confidence interval
  Relegation_Risk: P(Position > Safe_Zone)
  Europe_Chance: P(Position ≤ European_Spots)
```

#### 5.3.2 Tournament Simulation

```
Tournament_Probability(team, stage) = Knockout_Simulation(brackets, ratings)

For each round:
  1. Calculate match win probabilities
  2. Sample outcomes from probability distributions
  3. Advance winners to next round
  4. Repeat until tournament complete

Run 100,000 simulations
Output probabilities for each team:
  - Reach quarterfinals: X%
  - Reach semifinals: Y%
  - Reach final: Z%
  - Win tournament: W%

Factors:
  - Team strength (ELO)
  - Form
  - Bracket difficulty
  - Home advantage (if applicable)
```

### 5.4 Injury Prediction

#### 5.4.1 Injury Risk Model

```
Injury_Risk(0-10) = f(Workload, Fatigue, History, Biomechanics, Age)

Workload (0-10):
  Acute: Last 7 days intensity
  Chronic: Last 28 days intensity
  Ratio = Acute / Chronic

  Optimal Ratio: 0.8-1.3 (low risk)
  High Risk: >1.5 or <0.5

Fatigue (0-10):
  = (Minutes_Played / Expected_Minutes) × Match_Density

  Match_Density = Matches_in_Last_30_Days / 30
  Red flag: >0.5 (match every 2 days)

History (0-10):
  Previous_Injuries:
    Same injury: High recurrence risk (×2.5)
    Similar area: Moderate risk (×1.5)
    Different area: Baseline risk (×1.0)

  Time_Since_Injury:
    < 3 months: High risk (×2.0)
    3-6 months: Moderate risk (×1.3)
    > 6 months: Normal risk (×1.0)

Biomechanics (0-10):
  - Movement asymmetry
  - Running gait abnormalities
  - Jump landing mechanics
  - Muscular imbalances

Age (0-10):
  < 23: Lower baseline risk
  23-29: Normal risk
  30-32: Slightly elevated (+1)
  > 32: Higher risk (+2 to +3)

Risk_Level:
  0-3: Low (routine monitoring)
  4-6: Moderate (increased attention)
  7-8: High (training modification)
  9-10: Very High (rest recommended)
```

#### 5.4.2 Return-to-Play Prediction

```
Days_Until_Return = Base_Recovery_Time × Player_Factors

Player_Factors:
  Age_Factor:
    < 25: ×0.9
    25-30: ×1.0
    > 30: ×1.15

  Fitness_Factor:
    Elite: ×0.85
    Good: ×1.0
    Average: ×1.15

  Injury_Severity:
    Grade 1: ×1.0
    Grade 2: ×1.5
    Grade 3: ×2.5

Confidence_Interval: ±20%

Rehabilitation_Milestones:
  Phase 1: Rest and initial healing (20%)
  Phase 2: Movement restoration (30%)
  Phase 3: Strength building (25%)
  Phase 4: Sport-specific training (15%)
  Phase 5: Return to full training (10%)
```

---


## 10. Statistical Models

### 10.1 Regression Models

#### 10.1.1 Linear Regression

```
Goals = β₀ + β₁×xG + β₂×Shots + β₃×BigChances + ε

Use cases:
  - Goal prediction from underlying metrics
  - Performance trend analysis
  - Simple interpretability

Assumptions:
  - Linear relationship
  - Independence of errors
  - Homoscedasticity
  - Normality of residuals

Validation:
  R²: Coefficient of determination
  RMSE: Root mean squared error
  p-values: Statistical significance
```

#### 10.1.2 Logistic Regression

```
P(Win) = 1 / (1 + e^-(β₀ + β₁×Rating_Diff + β₂×Home + ...))

Use cases:
  - Binary outcomes (win/loss, injury/healthy)
  - Probability estimation
  - Baseline for comparison

Metrics:
  - AUC-ROC: Area under curve (>0.7 good)
  - Log-loss: Calibration measure
  - Accuracy, Precision, Recall
```

#### 10.1.3 Poisson Regression

```
Goals ~ Poisson(λ)
λ = exp(β₀ + β₁×Attack_Strength + β₂×Defense_Weakness)

Use cases:
  - Scoreline prediction
  - Count data (goals, shots, cards)
  - Tournament simulations

Properties:
  - Mean = Variance = λ
  - Non-negative integer outcomes
  - Independent events assumption
```

### 10.2 Machine Learning Models

#### 10.2.1 Random Forest

```
Ensemble of decision trees
Each tree votes on prediction
Final prediction by majority (classification) or average (regression)

Hyperparameters:
  n_estimators: 100-500 trees
  max_depth: 10-30
  min_samples_split: 2-10
  max_features: sqrt(n) or log2(n)

Advantages:
  - Handles non-linear relationships
  - Feature importance ranking
  - Robust to overfitting
  - Little preprocessing needed

Use cases:
  - Match outcome prediction
  - Player performance forecasting
  - Injury risk classification
```

#### 10.2.2 Gradient Boosting (XGBoost, LightGBM)

```
Sequential ensemble: Each tree corrects previous errors
Optimizes custom loss function

Hyperparameters:
  learning_rate: 0.01-0.1
  n_estimators: 100-1000
  max_depth: 3-10
  subsample: 0.8-1.0
  colsample_bytree: 0.8-1.0

Advantages:
  - State-of-the-art performance
  - Handles missing data
  - Feature interaction detection
  - Regularization built-in

Use cases:
  - Win probability models
  - xG models
  - Player rating systems
  - Transfer value prediction
```

#### 10.2.3 Neural Networks

```
Architecture for match prediction:
  Input Layer: 60 features (team stats, form, etc.)
  Hidden Layer 1: 128 units, ReLU activation
  Hidden Layer 2: 64 units, ReLU activation
  Hidden Layer 3: 32 units, ReLU activation
  Output Layer: 3 units (Win/Draw/Loss), Softmax

Training:
  Optimizer: Adam
  Learning rate: 0.001
  Batch size: 32
  Epochs: 100
  Loss: Categorical cross-entropy

Advantages:
  - Captures complex patterns
  - Scales with data
  - Transfer learning possible

Challenges:
  - Requires large datasets
  - Hyperparameter tuning
  - Interpretability
```

### 10.3 Time Series Analysis

#### 10.3.1 Moving Averages

```
Simple_MA(n) = (X₁ + X₂ + ... + Xₙ) / n

Exponential_MA(α) = α×Xₜ + (1-α)×EMA_{t-1}

α (smoothing factor): 0 < α < 1
  High α: More weight on recent values
  Low α: Smoother, slower to react

Use cases:
  - Form estimation
  - Trend detection
  - Noise reduction
```

#### 10.3.2 ARIMA Models

```
ARIMA(p, d, q):
  AR(p): Autoregressive order
  I(d): Integration (differencing) order
  MA(q): Moving average order

Model selection:
  1. Check stationarity (ADF test)
  2. ACF/PACF plots for p, q
  3. Grid search for best parameters
  4. Validate with AIC/BIC

Use cases:
  - Season-long performance forecasting
  - Attendance prediction
  - Long-term trend analysis
```

---


## 14. API Interface

### 14.1 RESTful API Endpoints

```
GET /api/v1/players/{player_id}
GET /api/v1/players/{player_id}/stats?season=2025-26
GET /api/v1/matches/{match_id}
GET /api/v1/matches/{match_id}/events
GET /api/v1/teams/{team_id}/performance
POST /api/v1/predictions/match
POST /api/v1/analysis/player-comparison
GET /api/v1/scouting/search?position=midfielder&minPPR=75
```

### 14.2 WebSocket Live Data

```javascript
ws://api.sportsanalytics.com/live/{match_id}

Message types:
  - event: Match event occurred
  - stats_update: Aggregated stats refresh
  - tracking_frame: Position data
  - prediction_update: Win probability change
```

---


