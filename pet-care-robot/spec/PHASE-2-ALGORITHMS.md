# PHASE 2: Algorithm Specification

## WIA-PET-CARE-ROBOT Processing Algorithms

### 2.1 Feeding Control Algorithm

```python
"""
Intelligent Feeding Control System
Manages automated feeding with portion control and pet recognition
"""

from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import List, Optional, Dict
from enum import Enum
import asyncio

class FeedingState(Enum):
    IDLE = "idle"
    DETECTING_PET = "detecting_pet"
    VERIFYING_ACCESS = "verifying_access"
    DISPENSING = "dispensing"
    MONITORING = "monitoring"
    COMPLETED = "completed"
    BLOCKED = "blocked"

@dataclass
class FeedingContext:
    pet_id: Optional[str]
    scheduled: bool
    portion_grams: float
    dispensed_grams: float
    start_time: datetime
    state: FeedingState

class FeedingController:
    """
    Manages intelligent feeding operations
    """

    def __init__(self, config: 'FeedingConfiguration'):
        self.config = config
        self.current_context: Optional[FeedingContext] = None
        self.daily_dispensed: Dict[str, float] = {}
        self.last_feeding: Dict[str, datetime] = {}

    async def process_feeding_request(
        self,
        trigger: str,  # 'SCHEDULE', 'MANUAL', 'PET_APPROACH', 'REMOTE'
        pet_id: Optional[str] = None,
        portion_override: Optional[float] = None
    ) -> 'FeedingResult':
        """
        Process a feeding request from various triggers
        """

        # Initialize context
        self.current_context = FeedingContext(
            pet_id=pet_id,
            scheduled=(trigger == 'SCHEDULE'),
            portion_grams=portion_override or 0,
            dispensed_grams=0,
            start_time=datetime.now(),
            state=FeedingState.DETECTING_PET
        )

        try:
            # Step 1: Pet detection if not specified
            if not pet_id:
                pet_id = await self._detect_approaching_pet()
                self.current_context.pet_id = pet_id

            if not pet_id:
                return FeedingResult(
                    success=False,
                    reason="No pet detected"
                )

            # Step 2: Verify access rights
            self.current_context.state = FeedingState.VERIFYING_ACCESS
            access_result = await self._verify_access(pet_id)

            if not access_result.allowed:
                self.current_context.state = FeedingState.BLOCKED
                return FeedingResult(
                    success=False,
                    reason=access_result.reason
                )

            # Step 3: Calculate portion
            portion = await self._calculate_portion(pet_id, portion_override)
            self.current_context.portion_grams = portion

            if portion <= 0:
                return FeedingResult(
                    success=False,
                    reason="Daily limit reached or portion too small"
                )

            # Step 4: Dispense food
            self.current_context.state = FeedingState.DISPENSING
            dispensed = await self._dispense_food(portion)
            self.current_context.dispensed_grams = dispensed

            # Step 5: Monitor consumption
            self.current_context.state = FeedingState.MONITORING
            consumption = await self._monitor_consumption(pet_id, dispensed)

            # Step 6: Record and complete
            self.current_context.state = FeedingState.COMPLETED
            await self._record_feeding(pet_id, dispensed, consumption)

            return FeedingResult(
                success=True,
                pet_id=pet_id,
                dispensed_grams=dispensed,
                consumed_grams=consumption.consumed,
                duration_seconds=consumption.duration
            )

        except Exception as e:
            return FeedingResult(
                success=False,
                reason=str(e)
            )

    async def _detect_approaching_pet(self) -> Optional[str]:
        """
        Detect and identify pet approaching feeder
        Uses RFID, microchip, or facial recognition
        """
        detection_methods = []

        if self.config.access_control.method == 'RFID_TAG':
            detection_methods.append(self._detect_rfid())
        elif self.config.access_control.method == 'MICROCHIP':
            detection_methods.append(self._detect_microchip())
        elif self.config.access_control.method == 'FACIAL':
            detection_methods.append(self._detect_facial())

        # Try all enabled methods
        results = await asyncio.gather(*detection_methods, return_exceptions=True)

        for result in results:
            if isinstance(result, str):  # Valid pet ID
                return result

        return None

    async def _verify_access(self, pet_id: str) -> 'AccessResult':
        """
        Verify pet has access rights to this feeder
        """
        # Check if pet is in allowed list
        if pet_id not in self.config.access_control.allowed_pets:
            return AccessResult(
                allowed=False,
                reason="Pet not authorized for this feeder"
            )

        # Check time between feedings
        if pet_id in self.last_feeding:
            elapsed = datetime.now() - self.last_feeding[pet_id]
            min_interval = timedelta(
                minutes=self.config.portion_control.min_time_between_feedings
            )
            if elapsed < min_interval:
                remaining = min_interval - elapsed
                return AccessResult(
                    allowed=False,
                    reason=f"Too soon. Wait {remaining.seconds // 60} minutes"
                )

        # Check multi-pet mode
        if self.config.access_control.multi_pet_mode == 'SEQUENTIAL':
            if await self._is_another_pet_eating():
                return AccessResult(
                    allowed=False,
                    reason="Another pet is currently eating"
                )

        return AccessResult(allowed=True)

    async def _calculate_portion(
        self,
        pet_id: str,
        override: Optional[float]
    ) -> float:
        """
        Calculate appropriate portion size
        """
        if override:
            return min(override, self.config.portion_control.max_single_portion_grams)

        # Get daily target
        daily_target = self.config.portion_control.daily_target_grams

        # Check veterinary override
        if self.config.portion_control.veterinary_override:
            vet = self.config.portion_control.veterinary_override
            if vet.expires_at > datetime.now():
                daily_target = vet.prescribed_grams

        # Calculate remaining allowance
        today_dispensed = self.daily_dispensed.get(pet_id, 0)
        remaining = daily_target - today_dispensed

        if remaining <= 0:
            return 0

        # Adjust for activity if enabled
        if self.config.portion_control.adjust_for_activity:
            activity_modifier = await self._get_activity_modifier(pet_id)
            remaining *= activity_modifier

        # Cap at max single portion
        portion = min(remaining, self.config.portion_control.max_single_portion_grams)

        return round(portion, 1)

    async def _dispense_food(self, grams: float) -> float:
        """
        Control dispenser to release specified amount
        Returns actual amount dispensed
        """
        dispensed = 0.0
        target = grams
        tolerance = 2.0  # grams

        while dispensed < target - tolerance:
            # Dispense in small increments for accuracy
            increment = min(10.0, target - dispensed)

            # Activate dispenser motor
            await self._activate_dispenser(increment)

            # Weigh dispensed amount
            actual = await self._weigh_dispensed()
            dispensed = actual

            # Safety check - stop if stuck
            if await self._is_dispenser_jammed():
                await self._alert_jam()
                break

        return dispensed

    async def _monitor_consumption(
        self,
        pet_id: str,
        dispensed: float
    ) -> 'ConsumptionResult':
        """
        Monitor pet eating and track consumption
        """
        start = datetime.now()
        last_weight = dispensed
        no_change_count = 0
        max_no_change = 30  # 30 second timeout

        while True:
            await asyncio.sleep(1)

            current_weight = await self._weigh_bowl()
            consumed = dispensed - current_weight

            # Check if still eating
            if abs(current_weight - last_weight) < 0.5:
                no_change_count += 1
            else:
                no_change_count = 0
                last_weight = current_weight

            # Pet finished or left
            if no_change_count >= max_no_change:
                break

            # Safety timeout (30 minutes max)
            if (datetime.now() - start).seconds > 1800:
                break

        duration = (datetime.now() - start).seconds
        final_weight = await self._weigh_bowl()
        consumed = dispensed - final_weight

        return ConsumptionResult(
            consumed=consumed,
            remaining=final_weight,
            duration=duration
        )
```

### 2.2 Navigation Algorithm

```python
"""
Robot Navigation System
Path planning and obstacle avoidance for mobile pet care robots
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional, Set
import heapq
import math

@dataclass
class Position:
    x: float
    y: float
    theta: float = 0.0  # Orientation in radians

@dataclass
class NavigationGoal:
    position: Position
    tolerance: float = 0.1  # meters
    orientation_required: bool = False

class NavigationController:
    """
    A* based navigation with dynamic obstacle avoidance
    """

    def __init__(self, environment_map: 'EnvironmentMap'):
        self.map = environment_map
        self.grid_resolution = 0.1  # 10cm grid
        self.robot_radius = 0.15    # 15cm robot
        self.current_position: Optional[Position] = None
        self.obstacles: Set[Tuple[int, int]] = set()
        self.pet_positions: Dict[str, Position] = {}

    def plan_path(
        self,
        start: Position,
        goal: NavigationGoal
    ) -> List[Position]:
        """
        Plan optimal path using A* algorithm
        """
        # Convert to grid coordinates
        start_grid = self._to_grid(start)
        goal_grid = self._to_grid(goal.position)

        # Update obstacle map with current pet positions
        dynamic_obstacles = self._get_dynamic_obstacles()
        all_obstacles = self.obstacles | dynamic_obstacles

        # A* search
        path_grid = self._astar(start_grid, goal_grid, all_obstacles)

        if not path_grid:
            return []

        # Convert back to world coordinates and smooth
        path_world = [self._to_world(p) for p in path_grid]
        smoothed_path = self._smooth_path(path_world)

        return smoothed_path

    def _astar(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        obstacles: Set[Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        """
        A* pathfinding algorithm
        """
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1),
                          (-1,-1), (-1,1), (1,-1), (1,1)]:
                new_pos = (pos[0] + dx, pos[1] + dy)
                if self._is_valid_position(new_pos, obstacles):
                    neighbors.append(new_pos)
            return neighbors

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))

            for neighbor in get_neighbors(current):
                # Diagonal movement costs more
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if dx + dy == 2 else 1.0

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def _is_valid_position(
        self,
        pos: Tuple[int, int],
        obstacles: Set[Tuple[int, int]]
    ) -> bool:
        """
        Check if position is valid (within bounds and not in obstacle)
        """
        # Check bounds
        if not self._in_bounds(pos):
            return False

        # Check obstacle collision with robot radius
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                check_pos = (pos[0] + dx, pos[1] + dy)
                if check_pos in obstacles:
                    return False

        # Check restricted areas
        world_pos = self._to_world(pos)
        for area in self.map.restricted_areas:
            if self._point_in_polygon(world_pos, area.boundaries):
                return False

        return True

    def _get_dynamic_obstacles(self) -> Set[Tuple[int, int]]:
        """
        Get current pet positions as dynamic obstacles
        """
        obstacles = set()
        safety_radius = 0.5  # 50cm safety distance from pets
        grid_radius = int(safety_radius / self.grid_resolution)

        for pet_id, position in self.pet_positions.items():
            center = self._to_grid(position)

            # Create circular obstacle around pet
            for dx in range(-grid_radius, grid_radius + 1):
                for dy in range(-grid_radius, grid_radius + 1):
                    if dx*dx + dy*dy <= grid_radius*grid_radius:
                        obstacles.add((center[0] + dx, center[1] + dy))

        return obstacles

    def _smooth_path(self, path: List[Position]) -> List[Position]:
        """
        Smooth path using gradient descent
        """
        if len(path) <= 2:
            return path

        smoothed = [Position(p.x, p.y) for p in path]
        weight_smooth = 0.5
        weight_data = 0.5
        tolerance = 0.001

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(smoothed) - 1):
                for dim in ['x', 'y']:
                    old_val = getattr(smoothed[i], dim)
                    orig_val = getattr(path[i], dim)
                    prev_val = getattr(smoothed[i-1], dim)
                    next_val = getattr(smoothed[i+1], dim)

                    new_val = old_val
                    new_val += weight_data * (orig_val - old_val)
                    new_val += weight_smooth * (prev_val + next_val - 2*old_val)

                    setattr(smoothed[i], dim, new_val)
                    change += abs(old_val - new_val)

        return smoothed

    async def execute_path(
        self,
        path: List[Position],
        speed: float = 0.3  # m/s
    ) -> bool:
        """
        Execute planned path with real-time adjustments
        """
        for i, target in enumerate(path):
            while True:
                current = await self._get_current_position()
                distance = self._distance(current, target)

                if distance < 0.05:  # 5cm tolerance
                    break

                # Check for new obstacles
                if await self._detect_obstacle_ahead():
                    # Replan from current position
                    new_path = self.plan_path(current, NavigationGoal(path[-1]))
                    if new_path:
                        return await self.execute_path(new_path, speed)
                    else:
                        await self._emergency_stop()
                        return False

                # Calculate velocity
                velocity = self._calculate_velocity(current, target, speed)

                # Apply pet-safe speed limits
                velocity = await self._apply_safety_limits(velocity)

                # Send to motor controller
                await self._set_velocity(velocity)

                await asyncio.sleep(0.05)  # 20Hz control loop

        await self._stop()
        return True

    async def _apply_safety_limits(self, velocity: Tuple[float, float]) -> Tuple[float, float]:
        """
        Apply pet-safe speed limits based on environment
        """
        vx, vy = velocity
        speed = math.sqrt(vx*vx + vy*vy)

        # Check for nearby pets
        for pet_id, pet_pos in self.pet_positions.items():
            current = await self._get_current_position()
            distance = self._distance(current, pet_pos)

            if distance < 1.0:  # Within 1 meter
                # Slow down proportionally
                factor = max(0.3, distance / 1.0)
                vx *= factor
                vy *= factor

            if distance < 0.3:  # Very close - stop
                return (0, 0)

        return (vx, vy)
```

### 2.3 Play Engagement Algorithm

```python
"""
Interactive Play Controller
AI-driven play sessions adapted to pet preferences
"""

from dataclasses import dataclass
from typing import List, Optional, Dict
from datetime import datetime, timedelta
import random
import numpy as np

@dataclass
class PlayState:
    active: bool
    play_type: str
    start_time: datetime
    engagement_score: float
    energy_estimate: float

class PlayEngagementController:
    """
    Controls interactive play with adaptive behavior
    """

    def __init__(self, config: 'PlayConfiguration'):
        self.config = config
        self.state: Optional[PlayState] = None
        self.behavior_model = BehaviorModel()
        self.session_history: List['PlaySession'] = []

    async def start_play_session(
        self,
        play_type: str,
        duration_minutes: int
    ) -> 'PlaySession':
        """
        Start an interactive play session
        """
        # Initialize state
        self.state = PlayState(
            active=True,
            play_type=play_type,
            start_time=datetime.now(),
            engagement_score=1.0,
            energy_estimate=1.0
        )

        session = PlaySession(
            session_id=generate_uuid(),
            play_type=play_type,
            started_at=datetime.now(),
            target_duration=duration_minutes
        )

        # Main play loop
        end_time = datetime.now() + timedelta(minutes=duration_minutes)

        while datetime.now() < end_time and self.state.active:
            # Generate next play action
            action = await self._generate_action()

            # Execute action
            await self._execute_action(action)

            # Monitor pet response
            response = await self._monitor_response()

            # Update engagement model
            self._update_engagement(response)

            # Adapt behavior
            if self.state.engagement_score < 0.3:
                # Pet losing interest - try different approach
                await self._change_pattern()

            if self.state.energy_estimate < 0.2:
                # Pet getting tired - slow down or end
                if not await self._offer_rest():
                    break

            # Safety checks
            if await self._safety_check_failed():
                break

            await asyncio.sleep(0.1)

        # End session
        self.state.active = False
        session.ended_at = datetime.now()
        session.engagement_average = self._calculate_average_engagement()

        # Learn from session
        await self._learn_from_session(session)

        self.session_history.append(session)
        return session

    async def _generate_action(self) -> 'PlayAction':
        """
        Generate next play action based on type and pet state
        """
        if self.state.play_type == 'LASER_POINTER':
            return await self._generate_laser_action()
        elif self.state.play_type == 'ROLLING_BALL':
            return await self._generate_ball_action()
        elif self.state.play_type == 'CHASE_ROBOT':
            return await self._generate_chase_action()
        elif self.state.play_type == 'TREAT_PUZZLE':
            return await self._generate_puzzle_action()
        else:
            return await self._generate_generic_action()

    async def _generate_laser_action(self) -> 'PlayAction':
        """
        Generate laser pointer movement pattern
        """
        # Get pet's current position and attention
        pet_pos = await self._get_pet_position()
        attention = await self._get_pet_attention()

        # Patterns based on engagement
        if self.state.engagement_score > 0.7:
            # High engagement - keep it interesting
            pattern = self._select_pattern([
                'ZIGZAG', 'CIRCLE', 'ESCAPE', 'PAUSE_DART'
            ])
        elif self.state.engagement_score > 0.4:
            # Medium engagement - simpler patterns
            pattern = self._select_pattern([
                'SLOW_MOVE', 'STOP_START', 'CIRCLE'
            ])
        else:
            # Low engagement - try to regain attention
            pattern = self._select_pattern([
                'NEAR_PET', 'ERRATIC', 'DISAPPEAR_REAPPEAR'
            ])

        # Generate waypoints for pattern
        waypoints = self._generate_laser_waypoints(pattern, pet_pos)

        return PlayAction(
            type='LASER',
            pattern=pattern,
            waypoints=waypoints,
            speed=self._calculate_speed()
        )

    def _generate_laser_waypoints(
        self,
        pattern: str,
        pet_pos: Position
    ) -> List[Position]:
        """
        Generate laser movement waypoints
        """
        waypoints = []

        if pattern == 'ZIGZAG':
            # Zigzag away from pet
            current = pet_pos
            for i in range(5):
                offset_x = random.uniform(0.3, 0.6) * (1 if i % 2 == 0 else -1)
                offset_y = random.uniform(0.2, 0.4)
                waypoints.append(Position(
                    x=current.x + offset_x,
                    y=current.y + offset_y
                ))
                current = waypoints[-1]

        elif pattern == 'CIRCLE':
            # Circle around pet
            radius = random.uniform(0.3, 0.5)
            for angle in np.linspace(0, 2*np.pi, 16):
                waypoints.append(Position(
                    x=pet_pos.x + radius * np.cos(angle),
                    y=pet_pos.y + radius * np.sin(angle)
                ))

        elif pattern == 'NEAR_PET':
            # Stay close to regain attention
            for _ in range(8):
                waypoints.append(Position(
                    x=pet_pos.x + random.uniform(-0.2, 0.2),
                    y=pet_pos.y + random.uniform(-0.2, 0.2)
                ))

        elif pattern == 'ESCAPE':
            # Quick dart away (prey behavior)
            direction = random.uniform(0, 2*np.pi)
            distance = random.uniform(0.5, 1.0)
            waypoints.append(Position(
                x=pet_pos.x + distance * np.cos(direction),
                y=pet_pos.y + distance * np.sin(direction)
            ))

        # Ensure waypoints are in safe zones
        waypoints = [self._clamp_to_safe_zone(w) for w in waypoints]

        return waypoints

    async def _monitor_response(self) -> 'PetResponse':
        """
        Monitor pet's response to play action
        """
        # Get current pet state
        pet_pos = await self._get_pet_position()
        pet_velocity = await self._get_pet_velocity()
        pet_posture = await self._detect_posture()

        # Calculate metrics
        is_chasing = pet_velocity > 0.5  # m/s
        is_pouncing = await self._detect_pounce()
        is_watching = await self._detect_attention()
        is_resting = pet_velocity < 0.1 and pet_posture == 'LYING'

        # Estimate engagement
        engagement = 0.0
        if is_chasing:
            engagement += 0.4
        if is_pouncing:
            engagement += 0.3
        if is_watching:
            engagement += 0.2
        if is_resting:
            engagement -= 0.3

        # Estimate energy level
        energy = await self._estimate_energy()

        return PetResponse(
            position=pet_pos,
            velocity=pet_velocity,
            posture=pet_posture,
            is_engaged=engagement > 0.3,
            engagement_score=min(1.0, max(0.0, engagement)),
            energy_level=energy
        )

    def _update_engagement(self, response: 'PetResponse'):
        """
        Update engagement model with exponential moving average
        """
        alpha = 0.3  # Smoothing factor

        self.state.engagement_score = (
            alpha * response.engagement_score +
            (1 - alpha) * self.state.engagement_score
        )

        self.state.energy_estimate = (
            alpha * response.energy_level +
            (1 - alpha) * self.state.energy_estimate
        )

    async def _learn_from_session(self, session: 'PlaySession'):
        """
        Update behavior model based on session results
        """
        # Record what worked
        effective_patterns = []
        ineffective_patterns = []

        for action in session.actions:
            if action.response_engagement > 0.6:
                effective_patterns.append(action.pattern)
            elif action.response_engagement < 0.3:
                ineffective_patterns.append(action.pattern)

        # Update pet preferences
        self.behavior_model.update_preferences(
            pet_id=self.config.pet_id,
            play_type=session.play_type,
            effective=effective_patterns,
            ineffective=ineffective_patterns,
            optimal_duration=session.actual_duration,
            time_of_day=session.started_at.hour
        )
```

### 2.4 Health Pattern Detection

```python
"""
Health Pattern Detection from Robot Sensors
Identifies health concerns from feeding and activity data
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from datetime import datetime, timedelta
import numpy as np
from scipy import stats

@dataclass
class HealthPattern:
    pattern_type: str
    severity: str
    confidence: float
    description: str
    recommendation: str
    detected_at: datetime

class HealthPatternDetector:
    """
    Detects health patterns from care robot data
    """

    def __init__(self, pet_id: str):
        self.pet_id = pet_id
        self.feeding_history: List['FeedingRecord'] = []
        self.activity_history: List['ActivityRecord'] = []
        self.litter_history: List['LitterRecord'] = []
        self.baselines: Dict[str, 'Baseline'] = {}

    def analyze_feeding_patterns(
        self,
        days: int = 7
    ) -> List[HealthPattern]:
        """
        Analyze feeding data for health indicators
        """
        patterns = []
        recent = self._get_recent_feeding(days)

        if len(recent) < 7:  # Need minimum data
            return patterns

        # Calculate daily totals
        daily_intake = self._calculate_daily_intake(recent)
        daily_duration = self._calculate_daily_eating_time(recent)

        # Check for reduced appetite
        baseline_intake = self.baselines.get('daily_intake')
        if baseline_intake:
            recent_avg = np.mean(list(daily_intake.values())[-3:])
            if recent_avg < baseline_intake.mean * 0.7:
                patterns.append(HealthPattern(
                    pattern_type='REDUCED_APPETITE',
                    severity='WARNING',
                    confidence=self._calculate_confidence(
                        recent_avg, baseline_intake
                    ),
                    description=f"Food intake reduced to {recent_avg:.0f}g "
                               f"(baseline: {baseline_intake.mean:.0f}g)",
                    recommendation="Monitor for 2-3 days. If persists, "
                                  "consult veterinarian.",
                    detected_at=datetime.now()
                ))

        # Check for increased eating speed
        baseline_duration = self.baselines.get('eating_duration')
        if baseline_duration:
            recent_avg = np.mean(list(daily_duration.values())[-3:])
            if recent_avg < baseline_duration.mean * 0.5:
                patterns.append(HealthPattern(
                    pattern_type='RAPID_EATING',
                    severity='INFO',
                    confidence=0.7,
                    description="Eating significantly faster than normal",
                    recommendation="Consider puzzle feeder or slow-feed bowl",
                    detected_at=datetime.now()
                ))

        # Check for irregular meal times
        meal_times = self._extract_meal_times(recent)
        variance = np.var([t.hour * 60 + t.minute for t in meal_times])
        if variance > 3600:  # High variance
            patterns.append(HealthPattern(
                pattern_type='IRREGULAR_FEEDING',
                severity='INFO',
                confidence=0.6,
                description="Meal times showing high variability",
                recommendation="Consider more consistent feeding schedule",
                detected_at=datetime.now()
            ))

        # Check for skipped meals
        expected_meals = days * 2  # Assume 2 meals per day
        actual_meals = len(recent)
        if actual_meals < expected_meals * 0.7:
            patterns.append(HealthPattern(
                pattern_type='SKIPPED_MEALS',
                severity='WARNING',
                confidence=0.8,
                description=f"Only {actual_meals} meals in {days} days "
                           f"(expected ~{expected_meals})",
                recommendation="Investigate cause. Check food freshness, "
                              "stress factors, or health issues.",
                detected_at=datetime.now()
            ))

        return patterns

    def analyze_water_intake(
        self,
        days: int = 7
    ) -> List[HealthPattern]:
        """
        Analyze water consumption patterns
        """
        patterns = []
        recent = self._get_recent_water(days)

        if len(recent) < 3:
            return patterns

        daily_intake = self._calculate_daily_water(recent)
        baseline = self.baselines.get('water_intake')

        if baseline:
            recent_avg = np.mean(list(daily_intake.values())[-3:])

            # Check for increased thirst (possible diabetes, kidney issues)
            if recent_avg > baseline.mean * 1.5:
                patterns.append(HealthPattern(
                    pattern_type='INCREASED_THIRST',
                    severity='WARNING',
                    confidence=self._calculate_confidence(
                        recent_avg, baseline
                    ),
                    description=f"Water intake increased to {recent_avg:.0f}ml "
                               f"(baseline: {baseline.mean:.0f}ml)",
                    recommendation="Increased thirst can indicate health issues. "
                                  "Consult veterinarian if persists.",
                    detected_at=datetime.now()
                ))

            # Check for reduced water intake
            elif recent_avg < baseline.mean * 0.5:
                patterns.append(HealthPattern(
                    pattern_type='REDUCED_WATER_INTAKE',
                    severity='WARNING',
                    confidence=self._calculate_confidence(
                        recent_avg, baseline
                    ),
                    description=f"Water intake reduced to {recent_avg:.0f}ml",
                    recommendation="Ensure fresh water available. "
                                  "Check for dehydration signs.",
                    detected_at=datetime.now()
                ))

        return patterns

    def analyze_litter_patterns(
        self,
        days: int = 7
    ) -> List[HealthPattern]:
        """
        Analyze litter box usage patterns (cats)
        """
        patterns = []
        recent = self._get_recent_litter(days)

        if len(recent) < 7:
            return patterns

        # Calculate daily visits
        daily_visits = self._calculate_daily_litter_visits(recent)
        visit_durations = [r.duration for r in recent]

        baseline_visits = self.baselines.get('litter_visits')
        baseline_duration = self.baselines.get('litter_duration')

        # Check for increased frequency (possible UTI)
        if baseline_visits:
            recent_avg = np.mean(list(daily_visits.values())[-3:])
            if recent_avg > baseline_visits.mean * 1.5:
                patterns.append(HealthPattern(
                    pattern_type='FREQUENT_URINATION',
                    severity='WARNING',
                    confidence=0.75,
                    description="Increased litter box visits detected",
                    recommendation="May indicate urinary tract issues. "
                                  "Monitor for straining or vocalization.",
                    detected_at=datetime.now()
                ))

        # Check for prolonged visits (possible constipation)
        if baseline_duration:
            recent_durations = [r.duration for r in recent[-10:]]
            recent_avg = np.mean(recent_durations)
            if recent_avg > baseline_duration.mean * 2:
                patterns.append(HealthPattern(
                    pattern_type='PROLONGED_LITTER_VISITS',
                    severity='INFO',
                    confidence=0.7,
                    description="Spending more time in litter box",
                    recommendation="May indicate constipation or discomfort. "
                                  "Monitor stool quality.",
                    detected_at=datetime.now()
                ))

        # Check for avoidance (possible stress or pain)
        if baseline_visits:
            recent_avg = np.mean(list(daily_visits.values())[-3:])
            if recent_avg < baseline_visits.mean * 0.5:
                patterns.append(HealthPattern(
                    pattern_type='LITTER_AVOIDANCE',
                    severity='WARNING',
                    confidence=0.7,
                    description="Reduced litter box usage",
                    recommendation="Check for accidents elsewhere. "
                                  "May indicate stress, pain, or litter preference.",
                    detected_at=datetime.now()
                ))

        return patterns

    def analyze_activity_patterns(
        self,
        days: int = 7
    ) -> List[HealthPattern]:
        """
        Analyze activity level patterns
        """
        patterns = []
        recent = self._get_recent_activity(days)

        if len(recent) < 7:
            return patterns

        daily_activity = self._calculate_daily_activity(recent)
        baseline = self.baselines.get('activity_level')

        if baseline:
            recent_avg = np.mean(list(daily_activity.values())[-3:])

            # Check for lethargy
            if recent_avg < baseline.mean * 0.5:
                patterns.append(HealthPattern(
                    pattern_type='REDUCED_ACTIVITY',
                    severity='WARNING',
                    confidence=self._calculate_confidence(
                        recent_avg, baseline
                    ),
                    description="Activity level significantly reduced",
                    recommendation="Reduced activity may indicate illness, "
                                  "pain, or depression. Monitor closely.",
                    detected_at=datetime.now()
                ))

            # Check for restlessness
            elif recent_avg > baseline.mean * 1.5:
                patterns.append(HealthPattern(
                    pattern_type='INCREASED_RESTLESSNESS',
                    severity='INFO',
                    confidence=0.6,
                    description="Activity level higher than normal",
                    recommendation="May indicate stress, discomfort, or "
                                  "environmental changes.",
                    detected_at=datetime.now()
                ))

        # Check for changed sleep patterns
        sleep_data = self._analyze_sleep_patterns(recent)
        if sleep_data.night_activity > baseline.get('night_activity', 0) * 2:
            patterns.append(HealthPattern(
                pattern_type='NIGHT_RESTLESSNESS',
                severity='INFO',
                confidence=0.65,
                description="Increased nighttime activity detected",
                recommendation="May indicate discomfort, anxiety, or "
                              "cognitive changes in senior pets.",
                detected_at=datetime.now()
            ))

        return patterns

    def _calculate_confidence(
        self,
        observed: float,
        baseline: 'Baseline'
    ) -> float:
        """
        Calculate statistical confidence of deviation
        """
        z_score = abs(observed - baseline.mean) / baseline.std
        # Convert z-score to confidence (0.5 to 0.99)
        confidence = min(0.99, 0.5 + (z_score * 0.15))
        return confidence
```

### 2.5 Safety Monitor Algorithm

```python
"""
Pet Safety Monitoring System
Real-time safety checks and emergency response
"""

from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from enum import Enum
import asyncio

class SafetyLevel(Enum):
    NORMAL = "normal"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"

@dataclass
class SafetyStatus:
    level: SafetyLevel
    checks: List['SafetyCheck']
    active_alerts: List['SafetyAlert']
    last_updated: datetime

class SafetyMonitor:
    """
    Continuous safety monitoring for pet care robots
    """

    def __init__(self, robot_id: str, config: 'SafetyConfiguration'):
        self.robot_id = robot_id
        self.config = config
        self.status = SafetyStatus(
            level=SafetyLevel.NORMAL,
            checks=[],
            active_alerts=[],
            last_updated=datetime.now()
        )
        self.monitoring = False

    async def start_monitoring(self):
        """
        Start continuous safety monitoring loop
        """
        self.monitoring = True

        while self.monitoring:
            try:
                # Run all safety checks
                await self._run_safety_checks()

                # Update overall status
                self._update_status()

                # Execute any required actions
                await self._execute_safety_actions()

                # Short interval for responsive monitoring
                await asyncio.sleep(0.5)

            except Exception as e:
                await self._handle_monitor_error(e)

    async def _run_safety_checks(self):
        """
        Run all configured safety checks
        """
        checks = []

        # Pet proximity check
        proximity_result = await self._check_pet_proximity()
        checks.append(proximity_result)

        # Collision detection
        collision_result = await self._check_collision_risk()
        checks.append(collision_result)

        # Environmental hazards
        env_result = await self._check_environment()
        checks.append(env_result)

        # Equipment status
        equipment_result = await self._check_equipment()
        checks.append(equipment_result)

        # Pet wellbeing
        wellbeing_result = await self._check_pet_wellbeing()
        checks.append(wellbeing_result)

        self.status.checks = checks

    async def _check_pet_proximity(self) -> 'SafetyCheck':
        """
        Check for safe distance from pets during movement
        """
        if not self._is_robot_moving():
            return SafetyCheck(
                name="pet_proximity",
                passed=True,
                level=SafetyLevel.NORMAL
            )

        pets = await self._detect_nearby_pets()
        min_distance = float('inf')
        closest_pet = None

        for pet in pets:
            distance = self._calculate_distance(pet.position)
            if distance < min_distance:
                min_distance = distance
                closest_pet = pet

        if min_distance < 0.2:  # 20cm - too close
            return SafetyCheck(
                name="pet_proximity",
                passed=False,
                level=SafetyLevel.CRITICAL,
                message=f"Pet too close: {min_distance:.2f}m",
                action="EMERGENCY_STOP"
            )
        elif min_distance < 0.5:  # 50cm - caution
            return SafetyCheck(
                name="pet_proximity",
                passed=True,
                level=SafetyLevel.CAUTION,
                message=f"Pet nearby: {min_distance:.2f}m",
                action="SLOW_DOWN"
            )
        else:
            return SafetyCheck(
                name="pet_proximity",
                passed=True,
                level=SafetyLevel.NORMAL
            )

    async def _check_collision_risk(self) -> 'SafetyCheck':
        """
        Check for potential collisions with obstacles
        """
        obstacles = await self._scan_obstacles()

        for obstacle in obstacles:
            time_to_collision = self._estimate_collision_time(obstacle)

            if time_to_collision < 0.5:  # Less than 0.5 seconds
                return SafetyCheck(
                    name="collision_risk",
                    passed=False,
                    level=SafetyLevel.CRITICAL,
                    message="Imminent collision detected",
                    action="EMERGENCY_STOP"
                )
            elif time_to_collision < 2.0:
                return SafetyCheck(
                    name="collision_risk",
                    passed=True,
                    level=SafetyLevel.WARNING,
                    message="Obstacle in path",
                    action="REROUTE"
                )

        return SafetyCheck(
            name="collision_risk",
            passed=True,
            level=SafetyLevel.NORMAL
        )

    async def _check_environment(self) -> 'SafetyCheck':
        """
        Check environmental conditions
        """
        readings = await self._get_environment_readings()
        issues = []
        max_level = SafetyLevel.NORMAL

        # Temperature check
        if readings.temperature > 30:
            issues.append("High temperature")
            max_level = SafetyLevel.WARNING
        elif readings.temperature < 10:
            issues.append("Low temperature")
            max_level = SafetyLevel.WARNING

        # Air quality
        if readings.air_quality_index > 150:
            issues.append("Poor air quality")
            max_level = SafetyLevel.WARNING

        # Ammonia (litter box area)
        if readings.ammonia > 25:
            issues.append("High ammonia levels")
            max_level = SafetyLevel.CAUTION

        if issues:
            return SafetyCheck(
                name="environment",
                passed=max_level != SafetyLevel.WARNING,
                level=max_level,
                message=", ".join(issues),
                action="ALERT_OWNER"
            )

        return SafetyCheck(
            name="environment",
            passed=True,
            level=SafetyLevel.NORMAL
        )

    async def _check_equipment(self) -> 'SafetyCheck':
        """
        Check robot equipment status
        """
        issues = []
        max_level = SafetyLevel.NORMAL

        # Battery level
        battery = await self._get_battery_level()
        if battery < 10:
            issues.append("Critical battery")
            max_level = SafetyLevel.WARNING
        elif battery < 20:
            issues.append("Low battery")
            max_level = SafetyLevel.CAUTION

        # Motor status
        motors = await self._check_motors()
        if not motors.all_functional:
            issues.append("Motor issue detected")
            max_level = SafetyLevel.WARNING

        # Sensor status
        sensors = await self._check_sensors()
        if not sensors.all_functional:
            issues.append("Sensor malfunction")
            max_level = SafetyLevel.WARNING

        # Dispenser status
        if self._has_dispenser():
            dispenser = await self._check_dispenser()
            if dispenser.jammed:
                issues.append("Dispenser jammed")
                max_level = SafetyLevel.WARNING

        if issues:
            return SafetyCheck(
                name="equipment",
                passed=max_level == SafetyLevel.CAUTION,
                level=max_level,
                message=", ".join(issues),
                action="MAINTENANCE_ALERT"
            )

        return SafetyCheck(
            name="equipment",
            passed=True,
            level=SafetyLevel.NORMAL
        )

    async def _check_pet_wellbeing(self) -> 'SafetyCheck':
        """
        Check for signs of pet distress
        """
        pets = await self._get_monitored_pets()
        issues = []
        max_level = SafetyLevel.NORMAL

        for pet in pets:
            # Check for fear/anxiety response to robot
            if await self._detect_fear_response(pet):
                issues.append(f"{pet.name} showing fear response")
                max_level = SafetyLevel.CAUTION

                # Robot should back off
                if self.config.pet_safety.fear_response == 'RETREAT':
                    await self._retreat_from_pet(pet)

            # Check for distress vocalizations
            if await self._detect_distress_vocalization(pet):
                issues.append(f"{pet.name} distress vocalization")
                max_level = SafetyLevel.WARNING

            # Check for injury signs
            if await self._detect_injury_signs(pet):
                issues.append(f"{pet.name} possible injury")
                max_level = SafetyLevel.WARNING

        if issues:
            return SafetyCheck(
                name="pet_wellbeing",
                passed=max_level != SafetyLevel.WARNING,
                level=max_level,
                message=", ".join(issues),
                action="ALERT_OWNER"
            )

        return SafetyCheck(
            name="pet_wellbeing",
            passed=True,
            level=SafetyLevel.NORMAL
        )

    async def _execute_safety_actions(self):
        """
        Execute required safety actions based on checks
        """
        for check in self.status.checks:
            if not check.passed or check.level in [SafetyLevel.WARNING, SafetyLevel.CRITICAL]:
                action = check.action

                if action == "EMERGENCY_STOP":
                    await self._emergency_stop()
                elif action == "SLOW_DOWN":
                    await self._reduce_speed()
                elif action == "REROUTE":
                    await self._initiate_reroute()
                elif action == "ALERT_OWNER":
                    await self._send_owner_alert(check.message)
                elif action == "MAINTENANCE_ALERT":
                    await self._send_maintenance_alert(check.message)
                elif action == "RETREAT":
                    await self._retreat_to_safe_position()

    async def _emergency_stop(self):
        """
        Immediate stop of all robot movement
        """
        # Stop all motors immediately
        await self._stop_all_motors()

        # Disable play features
        await self._disable_play()

        # Create emergency alert
        alert = SafetyAlert(
            alert_id=generate_uuid(),
            level=SafetyLevel.EMERGENCY,
            message="Emergency stop activated",
            timestamp=datetime.now(),
            requires_acknowledgment=True
        )

        self.status.active_alerts.append(alert)

        # Notify all emergency contacts
        for contact in self.config.emergency_contacts:
            await self._notify_contact(contact, alert)
```

---

## Document Information

| Field | Value |
|-------|-------|
| Standard | WIA-PET-CARE-ROBOT Version 1.0.0 |
| Phase | 2 - Algorithms |
| Status | Active |
| Philosophy | Hongik Ingan |
