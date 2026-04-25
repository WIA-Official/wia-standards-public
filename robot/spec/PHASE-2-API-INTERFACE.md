# WIA Robot Standard — Phase 2: API Interface Specification

## Version Information
- **Document Version**: 2.0.0
- **Last Updated**: 2026-04-25
- **Status**: Phase 2 — Complete
- **Standard**: WIA-ROBOT-API-002

---

## 1. Overview

이 문서는 WIA Robot Standard의 API 인터페이스를 정의합니다. 보조 로봇 시스템(Exoskeleton, Prosthetics, Rehabilitation, Care, Surgical, Mobility Aid)과 로봇 운영 미들웨어(ROS 2) 간의 상호운용 REST/WebSocket/DDS API를 규정합니다.

### 1.1 API Architecture Stack

```
┌─────────────────────────────────────────────────────┐
│               Client Applications                    │
│  (Clinical Dashboard · Teleop Console · Isaac Sim)   │
├─────────────────────────────────────────────────────┤
│           WIA Robot REST Gateway (OpenAPI 3.1)       │
│     GET /robots  POST /command  GET /telemetry       │
├─────────────────────────────────────────────────────┤
│         ROS 2 Bridge (rclpy / rclcpp)                │
│   Topics · Services · Actions · Parameters           │
├─────────────────────────────────────────────────────┤
│          DDS Layer (OMG RTPS 2.5 / FastDDS 3.x)     │
│      Domain 0 (default) · Reliability QoS           │
├─────────────────────────────────────────────────────┤
│          Hardware Abstraction (ros2_control)          │
│       JointTrajectoryController · ForceTorque        │
└─────────────────────────────────────────────────────┘
```

### 1.2 Conformance Levels

| Level | Scope |
|-------|-------|
| **L1 — Query** | GET endpoints only; read telemetry + status |
| **L2 — Command** | L1 + POST/PATCH for motion and parameter commands |
| **L3 — Full** | L2 + WebSocket streaming + E-Stop authority |

---

## 2. OpenAPI 3.1 Base Specification

```yaml
openapi: "3.1.0"
info:
  title: WIA Robot Gateway API
  version: "2.0.0"
  description: >
    RESTful and WebSocket API for WIA assistive and collaborative robots.
    Conforms to ISO 10218-1:2011 (industrial), ISO 15066:2016 (collaborative),
    IEC 61508 SIL-2 functional safety, and ROS 2 REP-3 / REP-2000 lifecycle.
  contact:
    name: WIA Standards Board
    url: https://standards.wia.global/robot
  license:
    name: Creative Commons Attribution 4.0
    url: https://creativecommons.org/licenses/by/4.0/

servers:
  - url: https://robot-api.wia.global/v2
    description: Production gateway
  - url: http://localhost:8080/v2
    description: Local ROS 2 bridge (ros2_web_bridge)

tags:
  - name: robots        # Device enumeration and status
  - name: command       # Motion + E-Stop + parameter tuning
  - name: telemetry     # Sensor streams + joint states
  - name: planning      # MoveIt 2 motion planning requests
  - name: slam          # SLAM map and pose queries
  - name: safety        # ISO 10218 safety zones + collision detection
  - name: sessions      # Clinical / operational session management
```

---

## 3. Core Endpoints

### 3.1 Device Registry (`/robots`)

```yaml
paths:
  /robots:
    get:
      operationId: listRobots
      tags: [robots]
      summary: Enumerate registered robot endpoints
      security:
        - BearerAuth: []
      parameters:
        - name: type
          in: query
          schema:
            type: string
            enum: [exoskeleton, prosthetic, rehabilitation,
                   care, surgical, mobility_aid, collaborative]
        - name: status
          in: query
          schema:
            type: string
            enum: [online, offline, fault, e_stop_active]
        - name: ros_domain
          in: query
          description: ROS_DOMAIN_ID filter (0-232)
          schema:
            type: integer
            minimum: 0
            maximum: 232
      responses:
        "200":
          description: Array of robot descriptors
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/RobotDescriptor'

  /robots/{robot_id}:
    get:
      operationId: getRobot
      tags: [robots]
      parameters:
        - name: robot_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        "200":
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RobotDescriptor'
        "404":
          $ref: '#/components/responses/NotFound'
```

### 3.2 Motion Command (`/robots/{id}/command`)

```yaml
  /robots/{robot_id}/command:
    post:
      operationId: sendCommand
      tags: [command]
      summary: Send motion or configuration command to robot
      description: >
        Commands are forwarded to the ROS 2 action server
        (/wia_robot/execute_trajectory) as FollowJointTrajectory actions
        (control_msgs/action/FollowJointTrajectory).
        E-Stop commands are delivered via dedicated /emergency_stop topic
        at RELIABLE QoS and bypass all command queues.
      security:
        - BearerAuth: []
        - CommandScope: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RobotCommand'
      responses:
        "202":
          description: Command accepted; action goal ID returned
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CommandAck'
        "400":
          $ref: '#/components/responses/BadRequest'
        "409":
          description: E-Stop active -- cannot accept motion commands
          content:
            application/problem+json:
              schema:
                $ref: '#/components/schemas/Problem'

  /robots/{robot_id}/command/estop:
    post:
      operationId: triggerEStop
      tags: [command, safety]
      summary: Trigger emergency stop (ISO 10218-1 S5.4.2)
      description: >
        Publishes std_msgs/Bool(data=true) to /emergency_stop with
        RELIABLE/TRANSIENT_LOCAL QoS. Response guaranteed less than 10 ms.
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [reason]
              properties:
                reason:
                  type: string
                  enum: [operator_request, fault_detected,
                         safety_zone_breach, watchdog_timeout]
                source_system:
                  type: string
      responses:
        "200":
          description: E-Stop acknowledged
        "503":
          description: Robot unreachable
```

### 3.3 Telemetry (`/robots/{id}/telemetry`)

```yaml
  /robots/{robot_id}/telemetry:
    get:
      operationId: getLatestTelemetry
      tags: [telemetry]
      summary: Latest joint states + sensor snapshot
      responses:
        "200":
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TelemetrySnapshot'

  /robots/{robot_id}/telemetry/stream:
    get:
      operationId: streamTelemetry
      tags: [telemetry]
      summary: WebSocket upgrade for real-time telemetry (50 Hz max)
      description: >
        After HTTP 101 Upgrade, server pushes TelemetryFrame JSON objects.
        Mapped from sensor_msgs/JointState ROS 2 topic at /joint_states.
        Cycle time configurable via ?hz= (1-50, default 10).
      responses:
        "101":
          description: Switching Protocols -- WebSocket established
```

### 3.4 MoveIt 2 Motion Planning (`/robots/{id}/plan`)

```yaml
  /robots/{robot_id}/plan:
    post:
      operationId: requestMotionPlan
      tags: [planning]
      summary: Request MoveIt 2 motion plan (OMPL / STOMP / PILZ)
      description: >
        Bridges to MoveIt 2 MoveGroupInterface via the
        /move_group/plan_kinematic_path ROS 2 service.
        Supports OMPL planners (RRTConnect, BiTRRT, LBKPIECE),
        STOMP stochastic optimization, and PILZ industrial motion
        profiles (LIN, PTP, CIRC).
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/PlanRequest'
      responses:
        "200":
          description: Plan computed; trajectory returned
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/PlanResult'
        "422":
          description: No IK solution found or goal in collision
```

### 3.5 SLAM (`/robots/{id}/slam`)

```yaml
  /robots/{robot_id}/slam/map:
    get:
      operationId: getSlamMap
      tags: [slam]
      summary: Current occupancy grid map (nav_msgs/OccupancyGrid)
      parameters:
        - name: format
          in: query
          schema:
            type: string
            enum: [json, pgm_base64]
            default: json
      responses:
        "200":
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/OccupancyGrid'

  /robots/{robot_id}/slam/pose:
    get:
      operationId: getSlamPose
      tags: [slam]
      summary: Current robot pose estimate from SLAM localizer
      description: >
        Returns geometry_msgs/PoseWithCovarianceStamped. Populated by
        Cartographer (2D LiDAR SLAM) or ORB-SLAM3 (visual-inertial SLAM)
        depending on sensor configuration.
      responses:
        "200":
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/PoseWithCovariance'
```

---

## 4. Data Schemas

### 4.1 `RobotDescriptor`

```json
{
  "robot_id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "ReWalk-6.0-Unit-042",
  "type": "exoskeleton",
  "manufacturer": "ReWalk Robotics",
  "model": "ReWalk 6.0",
  "serial_number": "RW-6-042",
  "firmware_version": "6.4.1",
  "ros2_fqdn": "rewalk042.local",
  "ros_domain_id": 12,
  "rmw_implementation": "rmw_fastrtps_cpp",
  "dds_config": "BEST_EFFORT_sensors__RELIABLE_commands",
  "status": "online",
  "e_stop_active": false,
  "safety_zones": ["ZONE_A_RESTRICTED", "ZONE_B_COLLABORATIVE"],
  "capabilities": ["joint_trajectory", "moveit2", "cartographer_slam"],
  "iso_compliance": ["ISO_10218_1", "ISO_15066_2016"],
  "last_seen": "2026-04-25T08:00:00Z"
}
```

### 4.2 `RobotCommand`

```json
{
  "command_type": "joint_trajectory",
  "target_robot": "550e8400-e29b-41d4-a716-446655440000",
  "priority": "normal",
  "timeout_ms": 5000,
  "payload": {
    "joint_names": ["hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"],
    "trajectory_points": [
      {
        "positions": [0.1, -0.2, 0.05, 0.1, -0.2, 0.05],
        "velocities": [0.05, -0.08, 0.02, 0.05, -0.08, 0.02],
        "time_from_start_ms": 500
      },
      {
        "positions": [0.2, -0.35, 0.1, 0.2, -0.35, 0.1],
        "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "time_from_start_ms": 1000
      }
    ],
    "path_tolerance": {"position_rad": 0.01, "velocity_rad_s": 0.05},
    "goal_tolerance":  {"position_rad": 0.005}
  }
}
```

### 4.3 `TelemetrySnapshot`

```json
{
  "robot_id": "550e8400-e29b-41d4-a716-446655440000",
  "seq": 100423,
  "stamp": "2026-04-25T08:00:00.123Z",
  "joint_state": {
    "name": ["hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"],
    "position": [0.198, -0.347, 0.098, 0.201, -0.352, 0.101],
    "velocity": [0.0012, -0.0008, 0.0005, 0.0011, -0.0009, 0.0004],
    "effort":   [12.4, 18.7, 6.2, 12.1, 18.5, 6.1]
  },
  "imu": {
    "linear_acceleration": {"x": 0.02, "y": -0.01, "z": 9.81},
    "angular_velocity":    {"x": 0.001, "y": -0.002, "z": 0.0}
  },
  "battery": {
    "voltage_v": 22.4,
    "current_a": 3.2,
    "state_of_charge_pct": 78,
    "time_remaining_min": 142
  },
  "safety": {
    "e_stop_active": false,
    "safety_zone": "ZONE_B_COLLABORATIVE",
    "collision_detected": false,
    "watchdog_ok": true
  }
}
```

### 4.4 `PlanRequest` (MoveIt 2)

```json
{
  "planner_id": "RRTConnect",
  "group_name": "lower_body",
  "goal_type": "joint_values",
  "goal_joint_values": {
    "hip_l": 0.3,
    "knee_l": -0.5,
    "ankle_l": 0.15,
    "hip_r": 0.3,
    "knee_r": -0.5,
    "ankle_r": 0.15
  },
  "workspace_bounds": {
    "min_x": -2.0, "max_x": 2.0,
    "min_y": -2.0, "max_y": 2.0,
    "min_z":  0.0, "max_z": 2.5
  },
  "planning_time_s": 5.0,
  "num_planning_attempts": 3,
  "allow_replanning": true,
  "velocity_scaling": 0.5,
  "acceleration_scaling": 0.5
}
```

---

## 5. DDS / ROS 2 Topic Mapping

### 5.1 ROS 2 Topic to REST Mapping

| ROS 2 Topic | Message Type | QoS | REST Endpoint |
|-------------|-------------|-----|---------------|
| `/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT 50 Hz | `GET /telemetry/stream` |
| `/imu/data_raw` | `sensor_msgs/Imu` | BEST_EFFORT | embedded in telemetry |
| `/emergency_stop` | `std_msgs/Bool` | RELIABLE + TRANSIENT_LOCAL | `POST /command/estop` |
| `/move_group/goal` | `moveit_msgs/MoveGroupActionGoal` | RELIABLE | `POST /plan` |
| `/map` | `nav_msgs/OccupancyGrid` | RELIABLE + TRANSIENT_LOCAL | `GET /slam/map` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | BEST_EFFORT | `GET /slam/pose` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | BEST_EFFORT | embedded in `/robots/{id}` |

### 5.2 QoS Profiles

```yaml
wia_robot_qos_profiles:
  commands:                     # Safety-critical; must not drop
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history: KEEP_LAST
    depth: 10
    deadline_ms: 100            # ISO 10218-1 S5.4 response budget

  telemetry_high_rate:          # Joint states at 50 Hz
    reliability: BEST_EFFORT
    durability: VOLATILE
    history: KEEP_LAST
    depth: 1
    lifespan_ms: 25             # Discard stale frames

  map_static:                   # SLAM maps; persist after publisher dies
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history: KEEP_LAST
    depth: 1
```

### 5.3 FastDDS 3.x XML Configuration

```xml
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>WIA_UDPv4</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="wia_robot_participant" is_default_participant="true">
    <rtps>
      <name>WIARobotParticipant</name>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>0.0.0.0</address>
            <port>7412</port>
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
      <builtin>
        <discovery_config>
          <leaseDuration><sec>DURATION_INFINITY</sec></leaseDuration>
          <initial_announcements>
            <count>5</count>
            <period><nanosec>100000000</nanosec></period>
          </initial_announcements>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

---

## 6. Authentication and Authorization

### 6.1 Security Scheme

```yaml
components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
      description: >
        JWT issued by WIA Identity Service. Claims must include
        robot_id (or wildcard) and command_scope.

    CommandScope:
      type: oauth2
      flows:
        clientCredentials:
          tokenUrl: https://auth.wia.global/oauth2/token
          scopes:
            robot:read:    "Read telemetry and status"
            robot:command: "Send motion commands"
            robot:estop:   "Trigger emergency stop"
            robot:admin:   "Modify safety zones and parameters"
```

### 6.2 JWT Payload Required Claims

```json
{
  "iss": "https://auth.wia.global",
  "sub": "operator-12345",
  "aud": "robot-gateway-v2",
  "exp": 1745539200,
  "iat": 1745535600,
  "robot_ids": ["550e8400-e29b-41d4-a716-446655440000"],
  "scopes": ["robot:read", "robot:command"],
  "safety_cert_level": "SIL-2",
  "operator_type": "clinical"
}
```

---

## 7. Safety Zone API (ISO 15066:2016)

### 7.1 Zone Configuration

```json
{
  "zone_id": "ZONE_A_RESTRICTED",
  "zone_type": "ISO_15066_RESTRICTED",
  "description": "Human body within 500 mm -- robot halts (Category 0 stop)",
  "geometry": {
    "type": "Cylinder",
    "radius_m": 0.5,
    "height_m": 2.0,
    "origin": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "action_on_breach": "CATEGORY_0_STOP",
  "power_and_force_limit": {
    "max_contact_force_n": 65,
    "max_static_force_n": 65,
    "reference_standard": "ISO_15066_Table_A1_Body_Region_3"
  }
}
```

### 7.2 Collision Checking Integration

The API delegates collision checking to MoveIt 2's `PlanningSceneMonitor`, which subscribes to `/collision_object` (moveit_msgs/CollisionObject) and `/attached_collision_object` topics. Real-time collision avoidance during execution uses the `ros2_control` hardware abstraction layer with force-torque sensor integration.

---

## 8. Isaac Sim Integration

### 8.1 Simulation API Bridge

NVIDIA Isaac Sim (USD Composer / Omniverse Kit) exposes the same WIA Robot API endpoints via the `isaacsim.ros2_bridge` extension, enabling simulation-to-real transfer without API code changes.

```python
# Isaac Sim Python scripting API -- connect WIA Robot Gateway
from isaacsim.ros2_bridge import ROS2Bridge

ros2_bridge = ROS2Bridge()
ros2_bridge.start(
    ros_domain_id=12,
    robot_namespace="/wia_robot",
    publish_joint_states=True,
    hz=50
)
```

### 8.2 ROS 2 to Isaac Sim Joint State Echo

```bash
# Verify joint state bridge from real robot to sim
ros2 topic echo /joint_states sensor_msgs/msg/JointState \
  --ros-args --remap /joint_states:=/wia_robot/joint_states
```

---

## 9. Error Responses (RFC 9457 Problem Details)

```yaml
components:
  responses:
    NotFound:
      description: Robot not found
      content:
        application/problem+json:
          schema:
            $ref: '#/components/schemas/Problem'
    BadRequest:
      description: Invalid request payload
      content:
        application/problem+json:
          schema:
            $ref: '#/components/schemas/Problem'

  schemas:
    Problem:
      type: object
      required: [type, title, status]
      properties:
        type:   {type: string, format: uri}
        title:  {type: string}
        status: {type: integer}
        detail: {type: string}
        instance: {type: string, format: uri}
      example:
        type: "https://standards.wia.global/robot/errors/estop-active"
        title: "Emergency Stop Active"
        status: 409
        detail: "Robot 550e8400 is in E-Stop state; clear fault before issuing motion commands."
        instance: "/robots/550e8400-e29b-41d4-a716-446655440000"
```

---

## 10. Conformance Checklist

| Requirement | Reference | Mandatory |
|-------------|-----------|-----------|
| E-Stop endpoint less than 10 ms response | ISO 10218-1 S5.4.2 | Yes |
| JWT authentication on all command endpoints | WIA-SEC-AUTH | Yes |
| QoS RELIABLE for command topics | OMG DDS S7.1.3 | Yes |
| Safety zone power/force limits enforced | ISO 15066:2016 Table A.1 | Yes |
| OpenAPI 3.1 specification present | WIA-ROBOT-API-002 | Yes |
| MoveIt 2 motion planning integration | ROS 2 REP-2000 | Yes |
| SLAM map + pose endpoints | ROS 2 Nav2 | Yes |
| Isaac Sim ROS 2 bridge compatible | NVIDIA Isaac SDK | Yes |
| IEC 61508 SIL-2 JWT claim validation | IEC 61508-3 | Yes |
| RFC 9457 Problem Details error responses | IETF RFC 9457 | Yes |
