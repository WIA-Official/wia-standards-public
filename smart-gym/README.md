# 🏋️ WIA-IND-014: Smart Gym Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-014
> **Version:** 1.0.0
> **Status:** Active
> **Category:** Industry / Health & Fitness
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-IND-014 standard defines a comprehensive framework for smart gym facilities, encompassing connected fitness equipment, automated member management, personalized workout programs, facility IoT sensors, virtual training systems, and performance analytics. This standard enables seamless integration between gym equipment, member devices, coaching systems, and facility management platforms.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard democratizes access to professional fitness technology, making high-quality training available to everyone while optimizing gym operations and enhancing member experiences through intelligent automation and personalized coaching.

## 🎯 Key Features

- **Connected Equipment**: Real-time tracking of treadmills, bikes, weights, rowing machines with IoT sensors
- **Member Management**: Automated check-in, membership tracking, access control, attendance analytics
- **Automated Workout Programs**: AI-powered personalized training plans with progressive overload
- **Facility IoT**: Environmental monitoring (temperature, humidity, CO2), equipment availability, crowd density
- **Virtual Training**: Live streaming classes, on-demand workouts, AR/VR guided exercises, remote coaching
- **Performance Analytics**: Progress tracking, workout history, body composition trends, goal achievement
- **Safety Monitoring**: Form analysis, injury prevention, emergency response, equipment malfunction detection

## 📊 Core Concepts

### 1. Equipment Data Collection

```
Connected Equipment Metrics:
  • Resistance/Weight Level
  • Speed/Cadence
  • Distance/Repetitions
  • Heart Rate Integration
  • Power Output (Watts)
  • Calorie Burn Estimation
  • Form Analysis (Computer Vision)
  • Equipment Usage Time
```

### 2. Member Progression Formula

```
Progressive Overload = (Current_Weight × Reps × Sets) × Intensity_Factor
Training Volume = ∑(Weight × Reps × Sets) per workout
Weekly Load = Training Volume × Frequency
Recovery Score = (1 - Fatigue_Index) × Sleep_Quality × Nutrition_Score
```

### 3. Facility Optimization

```
Equipment Utilization = (Active_Time / Available_Time) × 100%
Peak Hours Detection = Automated crowd density analysis
Maintenance Scheduling = Usage_hours × Wear_factor
Energy Efficiency = Power_consumed / Member_hours
```

## 🔧 TypeScript SDK Usage

```typescript
import {
  SmartGymSDK,
  EquipmentManager,
  MemberManager,
  WorkoutPlanner,
  FacilityMonitor
} from '@wia/ind-014';

// Initialize Smart Gym System
const gym = new SmartGymSDK({
  facilityId: 'GYM-2025-001',
  location: 'Seoul Fitness Center',
  equipment: {
    treadmills: 20,
    bikes: 15,
    rowers: 10,
    weights: true,
    functionalArea: true
  },
  features: {
    virtualTraining: true,
    iotSensors: true,
    aiCoaching: true,
    memberApp: true
  }
});

// Connect equipment
const equipment = new EquipmentManager(gym);

await equipment.registerDevice({
  deviceId: 'TREAD-001',
  type: 'treadmill',
  brand: 'ProFit',
  model: 'X5000',
  capabilities: {
    heartRate: true,
    incline: true,
    speedRange: { min: 0, max: 20 }, // km/h
    programs: ['interval', 'hill', 'fat-burn', 'custom']
  }
});

// Track live workout
const session = await equipment.startWorkout({
  deviceId: 'TREAD-001',
  memberId: 'MEM-12345',
  program: 'interval',
  duration: 30, // minutes
  targetHeartRate: 150,
  difficulty: 'intermediate'
});

session.on('update', (data) => {
  console.log(`Speed: ${data.speed} km/h`);
  console.log(`Distance: ${data.distance} km`);
  console.log(`Heart Rate: ${data.heartRate} bpm`);
  console.log(`Calories: ${data.caloriesBurned}`);
  console.log(`Time Remaining: ${data.timeRemaining} min`);
});

// Member management
const members = new MemberManager(gym);

const member = await members.checkIn({
  memberId: 'MEM-12345',
  method: 'rfid-card', // or 'app', 'biometric', 'qr-code'
  timestamp: new Date()
});

console.log(`Welcome ${member.name}!`);
console.log(`Membership: ${member.plan.type} (${member.plan.daysRemaining} days left)`);
console.log(`Today's Workout: ${member.todaysPlan.name}`);

// AI-powered workout planning
const planner = new WorkoutPlanner({
  model: 'ai-coach-v3',
  personalization: true
});

const workoutPlan = await planner.generatePlan({
  memberId: 'MEM-12345',
  goals: ['strength', 'muscle-gain'],
  experience: 'intermediate',
  frequency: 4, // days per week
  duration: 60, // minutes per session
  availableEquipment: equipment.getAvailable(),
  restrictions: ['lower-back-injury'],
  preferences: {
    avoidCardio: false,
    preferFreeWeights: true,
    maxExercisesPerSession: 8
  }
});

console.log('8-Week Strength Program:');
workoutPlan.weeks.forEach((week, index) => {
  console.log(`\nWeek ${index + 1}:`);
  week.sessions.forEach(session => {
    console.log(`  ${session.day}: ${session.focus} (${session.exercises.length} exercises)`);
  });
});

// Facility monitoring
const facility = new FacilityMonitor(gym);

const status = await facility.getStatus();
console.log(`Temperature: ${status.environment.temperature}°C`);
console.log(`Humidity: ${status.environment.humidity}%`);
console.log(`CO2 Level: ${status.environment.co2} ppm`);
console.log(`Current Occupancy: ${status.occupancy.current}/${status.occupancy.max}`);
console.log(`Available Treadmills: ${status.equipment.available.treadmills}`);

// Virtual training
const virtualTraining = gym.getVirtualTraining();

const liveClass = await virtualTraining.joinClass({
  classId: 'CLASS-2025-0327',
  memberId: 'MEM-12345',
  device: 'mobile-app'
});

console.log(`Joined: ${liveClass.name} with ${liveClass.instructor.name}`);
console.log(`${liveClass.participants.length} participants live`);

// Analytics and reporting
const analytics = gym.getAnalytics();

const memberStats = await analytics.getMemberStats('MEM-12345', {
  period: '30days'
});

console.log(`Total Workouts: ${memberStats.totalWorkouts}`);
console.log(`Total Time: ${memberStats.totalMinutes} minutes`);
console.log(`Avg Heart Rate: ${memberStats.avgHeartRate} bpm`);
console.log(`Calories Burned: ${memberStats.totalCalories}`);
console.log(`Strength Progress: +${memberStats.strengthIncrease}%`);
console.log(`Consistency Score: ${memberStats.consistencyScore}/100`);
```

## 🖥️ CLI Tool Usage

```bash
# Install CLI tool
./install.sh

# Register new gym facility
wia-ind-014 facility register --name "Seoul Fitness" --capacity 200

# Add equipment
wia-ind-014 equipment add --type treadmill --id TREAD-001 --brand ProFit

# Member check-in
wia-ind-014 member checkin --id MEM-12345 --method rfid

# Start workout session
wia-ind-014 workout start --member MEM-12345 --equipment TREAD-001 --duration 30

# Monitor live session
wia-ind-014 workout monitor --session SESSION-001

# Generate workout plan
wia-ind-014 plan create --member MEM-12345 --goal strength --weeks 8

# Facility status
wia-ind-014 facility status --live

# Equipment maintenance check
wia-ind-014 equipment maintenance --check-all

# Analytics report
wia-ind-014 analytics member --id MEM-12345 --period 30days
wia-ind-014 analytics facility --period week --export pdf

# Virtual class schedule
wia-ind-014 virtual list-classes --date today
wia-ind-014 virtual join --class CLASS-001 --member MEM-12345

# Calculations
wia-ind-014 calc --metric one-rep-max --weight 100 --reps 8
wia-ind-014 calc --metric training-volume --weight 80 --reps 10 --sets 3
wia-ind-014 calc --metric calorie-burn --activity weights --duration 60 --weight 75
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-014-v1.0.md](./spec/WIA-IND-014-v1.0.md) | Complete technical specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-014.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/smart-gym

# Run installation script
./install.sh

# Verify installation
wia-ind-014 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-014

# Or yarn
yarn add @wia/ind-014
```

## 🏆 Equipment Types Supported

| Equipment | IoT Sensors | Metrics Tracked | AI Features |
|-----------|-------------|-----------------|-------------|
| Treadmills | Speed, Incline, HR | Distance, Pace, Calories | Form analysis, Auto-adjust |
| Exercise Bikes | Resistance, Cadence, HR | Power, Distance, RPM | Virtual courses, Adaptive resistance |
| Rowing Machines | Resistance, Stroke Rate | Distance, Split time, Power | Technique coaching, Posture alerts |
| Weight Machines | Load sensors, Position | Weight, Reps, ROM | Progressive overload, Safety alerts |
| Free Weights | RFID tracking | Weight, Sets, Reps | Form analysis (camera), Spotter alerts |
| Functional Trainers | Force sensors | Power, Speed, Resistance | Movement pattern analysis |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language gym queries and booking
- **WIA-OMNI-API**: Universal fitness data gateway
- **WIA-SOCIAL**: Social fitness challenges and communities
- **WIA-IND-012**: Fitness tracking for wearables integration
- **WIA-IND-011**: Sports tech for athlete training programs
- **WIA-MED**: Health records for medical fitness programs

## 🏅 Philosophy: 홍익인간 (弘益人間) - Benefit All Humanity

Smart gyms should empower everyone to achieve their fitness goals:

1. **Accessibility**: Making professional training technology available to all members
2. **Personalization**: AI-powered programs adapted to individual needs and limitations
3. **Safety**: Real-time form analysis and injury prevention systems
4. **Inclusivity**: Programs for all ages, fitness levels, and abilities
5. **Efficiency**: Optimizing gym operations to reduce costs and improve service
6. **Community**: Fostering supportive fitness communities through technology
7. **Privacy**: Protecting member health data and workout information

## 📦 Use Cases

1. **Commercial Gyms**: Large fitness centers with 100+ members
2. **Boutique Studios**: Specialized training facilities (CrossFit, Yoga, Pilates)
3. **Corporate Wellness**: Company fitness centers with employee health programs
4. **Hotel Gyms**: Smart amenities for traveling guests
5. **Physical Therapy**: Rehabilitation centers with tracked recovery programs
6. **Sports Teams**: Training facilities with performance analytics
7. **Home Gyms**: Connected equipment for personal use

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
