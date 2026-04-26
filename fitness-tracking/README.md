# 🏃 WIA-IND-012: Fitness Tracking Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-012
> **Version:** 1.0.0
> **Status:** Active
> **Category:** Industry / Health & Fitness
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-IND-012 standard defines a comprehensive framework for fitness tracking, including activity monitoring, heart rate tracking, calorie calculation, workout logging, and health metrics integration. This standard enables seamless interoperability between fitness devices, applications, and health platforms.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to promote global health and wellness by providing unified, accessible fitness tracking capabilities that empower individuals to monitor and improve their physical well-being.

## 🎯 Key Features

- **Activity Monitoring**: Real-time tracking of steps, distance, and movement patterns
- **Heart Rate Tracking**: Continuous and on-demand heart rate monitoring with zone analysis
- **Calorie Calculation**: Accurate energy expenditure based on activity, metrics, and physiology
- **Workout Logging**: Structured exercise sessions with detailed performance metrics
- **Health Metrics Integration**: Unified interface for weight, body composition, sleep, and vitals
- **Goal Management**: Personalized targets with progress tracking and achievements
- **Multi-Device Sync**: Seamless data synchronization across devices and platforms

## 📊 Core Concepts

### 1. Activity Energy Expenditure

```
Calories = (MET × Weight_kg × Duration_hours) + RMR_component
```

Where:
- `MET` = Metabolic Equivalent of Task (activity-specific)
- `Weight_kg` = Body weight in kilograms
- `Duration_hours` = Activity duration
- `RMR_component` = Resting Metabolic Rate portion

### 2. Heart Rate Zones

```
Zone 1 (Recovery): 50-60% of Max HR
Zone 2 (Aerobic): 60-70% of Max HR
Zone 3 (Tempo): 70-80% of Max HR
Zone 4 (Threshold): 80-90% of Max HR
Zone 5 (Maximum): 90-100% of Max HR

Max HR ≈ 220 - Age (basic formula)
```

### 3. Training Load Calculation

```
Training Load = Duration × Intensity Factor × Frequency
```

Where intensity considers heart rate, power output, and perceived exertion.

## 🔧 Components

### TypeScript SDK

```typescript
import {
  ActivityTracker,
  HeartRateMonitor,
  WorkoutLogger,
  CalorieCalculator
} from '@wia/ind-012';

// Initialize activity tracker
const tracker = new ActivityTracker({
  userId: 'user-123',
  deviceId: 'watch-001'
});

// Log activity
const activity = await tracker.logActivity({
  type: 'running',
  duration: 3600, // seconds
  distance: 10000, // meters
  avgHeartRate: 155,
  calories: 680
});

// Calculate calories for workout
const calculator = new CalorieCalculator();
const calories = calculator.calculateWorkoutCalories({
  activityType: 'cycling',
  weight: 70, // kg
  duration: 2700, // 45 minutes
  intensity: 'moderate',
  avgHeartRate: 145
});

console.log(`Calories burned: ${calories.total}`);

// Monitor heart rate zones
const monitor = new HeartRateMonitor({ age: 35 });
const zones = monitor.analyzeWorkout({
  heartRateData: [120, 135, 150, 165, 155, 145, 130],
  timestamps: [...] // corresponding times
});

console.log(`Time in Zone 3: ${zones.zone3.duration} minutes`);
```

### CLI Tool

```bash
# Log activity
wia-ind-012 log-activity --type running --duration 3600 --distance 10000 --hr 155

# Calculate calories
wia-ind-012 calc-calories --activity cycling --weight 70 --duration 45 --intensity moderate

# Analyze heart rate
wia-ind-012 analyze-hr --age 35 --hr-data "120,135,150,165,155,145,130"

# Track daily steps
wia-ind-012 log-steps --steps 12500 --date 2025-12-27

# Create workout plan
wia-ind-012 create-workout --name "Morning Run" --type cardio --duration 45 --target-hr 150

# Get fitness summary
wia-ind-012 summary --period week

# Set fitness goals
wia-ind-012 set-goal --type steps --target 10000 --period daily
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-012-v1.0.md](./spec/WIA-IND-012-v1.0.md) | Complete technical specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-012.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/fitness-tracking

# Run installation script
./install.sh

# Verify installation
wia-ind-012 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-012

# Or yarn
yarn add @wia/ind-012
```

```typescript
import { FitnessSDK } from '@wia/ind-012';

const sdk = new FitnessSDK({
  userId: 'user-123',
  profile: {
    age: 35,
    weight: 70,
    height: 175,
    gender: 'male'
  }
});

// Log a run
const workout = await sdk.logWorkout({
  type: 'running',
  startTime: new Date('2025-12-27T07:00:00Z'),
  duration: 2700, // 45 minutes
  distance: 8000, // 8km
  avgHeartRate: 155,
  maxHeartRate: 178
});

console.log(`Workout logged: ${workout.id}`);
console.log(`Calories burned: ${workout.caloriesBurned}`);
console.log(`Average pace: ${workout.avgPace} min/km`);

// Get weekly summary
const summary = await sdk.getWeeklySummary();
console.log(`Total workouts: ${summary.totalWorkouts}`);
console.log(`Total distance: ${summary.totalDistance} km`);
console.log(`Active days: ${summary.activeDays}/7`);
```

## 💪 Activity Types & MET Values

| Activity | MET Value | Intensity |
|----------|-----------|-----------|
| Walking (3 mph) | 3.5 | Light |
| Walking (4 mph) | 5.0 | Moderate |
| Running (5 mph) | 8.3 | Moderate |
| Running (6 mph) | 9.8 | Vigorous |
| Running (7 mph) | 11.0 | Vigorous |
| Cycling (12-13.9 mph) | 8.0 | Moderate |
| Cycling (14-15.9 mph) | 10.0 | Vigorous |
| Swimming (moderate) | 5.8 | Moderate |
| Swimming (vigorous) | 9.8 | Vigorous |
| Strength Training | 3.5-6.0 | Varies |

## ❤️ Heart Rate Metrics

| Metric | Formula | Purpose |
|--------|---------|---------|
| Max HR | 220 - Age | Upper limit |
| Resting HR | Morning measurement | Recovery indicator |
| HR Reserve | Max HR - Resting HR | Training zones base |
| Target HR | Resting HR + (HR Reserve × %) | Zone calculation |
| Recovery HR | Drop after 1 min | Fitness indicator |

## ⚠️ Health & Safety

1. **Medical Clearance**: Consult healthcare provider before starting new fitness programs
2. **Heart Rate Limits**: Stay within safe zones based on age and fitness level
3. **Hydration**: Monitor fluid intake during extended activities
4. **Rest & Recovery**: Include adequate rest days in training schedules
5. **Gradual Progression**: Increase intensity and duration gradually (10% rule)
6. **Device Accuracy**: Regular calibration of tracking devices for accuracy

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language fitness queries and goal setting
- **WIA-OMNI-API**: Universal health data gateway
- **WIA-SOCIAL**: Social fitness challenges and community features
- **WIA-HEALTH**: Medical health records and clinical integration
- **WIA-NUTRITION**: Diet and nutrition tracking correlation

## 📖 Use Cases

1. **Personal Fitness**: Individual health and workout tracking
2. **Athletic Training**: Professional athlete performance monitoring
3. **Corporate Wellness**: Company fitness programs and challenges
4. **Medical Rehabilitation**: Recovery tracking and therapy monitoring
5. **Research Studies**: Population health and fitness research data
6. **Insurance Integration**: Activity-based insurance premium calculations
7. **Smart Gym Equipment**: Connected fitness equipment data sharing

## 🏆 Achievement System

- **Daily Goals**: Steps, calories, active minutes
- **Weekly Challenges**: Distance, workout frequency, consistency
- **Milestones**: Personal records, streak days, total achievements
- **Badges**: Activity diversity, endurance, strength, consistency
- **Leaderboards**: Social competition and motivation

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
