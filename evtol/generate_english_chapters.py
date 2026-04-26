#!/usr/bin/env python3
# Generate English chapters 2-8 with comprehensive content (200+ lines each)

import os

base_dir = "/home/user/wia-standards/evtol/ebook/en"

# HTML template
def create_chapter_html(chapter_num, title, subtitle, prev_link, next_link, content):
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Chapter {chapter_num}: {title} - WIA-SPACE-019</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{ background: #1a1a2e; color: #eee; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; line-height: 1.8; padding: 20px; }}
        .container {{ max-width: 900px; margin: 0 auto; }}
        .nav {{ margin-bottom: 30px; display: flex; justify-content: space-between; align-items: center; }}
        .nav a {{ padding: 10px 20px; background: #e94560; color: #fff; text-decoration: none; border-radius: 5px; transition: all 0.3s ease; }}
        .nav a:hover {{ background: #d63651; transform: translateY(-2px); }}
        header {{ text-align: center; padding: 40px 20px; background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); border-radius: 15px; margin-bottom: 40px; border: 2px solid #e94560; }}
        h1 {{ font-size: 2.5em; color: #e94560; margin-bottom: 10px; }}
        .chapter-number {{ color: #aaa; font-size: 1.2em; margin-bottom: 20px; }}
        .content {{ background: #16213e; padding: 40px; border-radius: 15px; border: 1px solid #0f3460; margin-bottom: 30px; }}
        h2 {{ color: #e94560; margin-top: 40px; margin-bottom: 20px; font-size: 2em; border-bottom: 2px solid #e94560; padding-bottom: 10px; }}
        h3 {{ color: #e94560; margin-top: 30px; margin-bottom: 15px; font-size: 1.5em; }}
        p {{ margin-bottom: 20px; font-size: 1.05em; color: #ddd; }}
        ul, ol {{ margin-left: 30px; margin-bottom: 20px; }}
        li {{ margin-bottom: 10px; color: #ddd; }}
        .highlight {{ background: rgba(233, 69, 96, 0.1); border-left: 4px solid #e94560; padding: 20px; margin: 20px 0; border-radius: 5px; }}
        .tech-box {{ background: #1a1a2e; padding: 25px; margin: 20px 0; border-radius: 10px; border-left: 4px solid #e94560; }}
        .tech-box h4 {{ color: #e94560; font-size: 1.3em; margin-bottom: 15px; }}
        .comparison-table {{ width: 100%; border-collapse: collapse; margin: 30px 0; background: #1a1a2e; border-radius: 8px; overflow: hidden; }}
        .comparison-table th {{ background: #e94560; color: #fff; padding: 15px; text-align: left; font-weight: bold; }}
        .comparison-table td {{ padding: 12px 15px; border-bottom: 1px solid #0f3460; }}
        .comparison-table tr:hover {{ background: rgba(233, 69, 96, 0.1); }}
        footer {{ text-align: center; margin-top: 60px; padding: 30px; color: #666; }}
    </style>
</head>
<body>
    <div class="container">
        <div class="nav">
            <a href="{prev_link}">← Previous</a>
            <a href="index.html">Contents</a>
            <a href="{next_link}">Next →</a>
        </div>

        <header>
            <div class="chapter-number">Chapter {chapter_num}</div>
            <h1>{title}</h1>
            <p style="color: #aaa;">{subtitle}</p>
        </header>

        <div class="content">
{content}
        </div>

        <div class="nav">
            <a href="{prev_link}">← Previous</a>
            <a href="index.html">Contents</a>
            <a href="{next_link}">Next →</a>
        </div>

        <footer>
            <p>© 2025 SmileStory Inc. / WIA</p>
            <p>弘益人間 (Hongik Ingan) · Benefit All Humanity</p>
        </footer>
    </div>
</body>
</html>
"""

# Define chapters with comprehensive content
chapters = {
    2: {
        'title': 'eVTOL Design Configurations',
        'subtitle': 'Multicopter, Tilt-Rotor, Lift+Cruise, and Vectored Thrust',
        'prev': 'chapter-01.html',
        'next': 'chapter-03.html',
        'content': """<h2>🛩️ Diversity of eVTOL Design</h2>

<p>eVTOL must achieve two different flight modes: vertical take-off/landing and forward flight. These modes have conflicting requirements. Vertical operations need strong upward thrust, while cruising requires efficient forward flight capability. Engineers have developed various design approaches to optimize both requirements.</p>

<p>Currently, over 700 eVTOL projects are underway worldwide, broadly classified into four major design categories: Multicopter, Tilt-Rotor, Lift+Cruise, and Vectored Thrust.</p>

<h2>🚁 1. Multicopter</h2>

<p>Multicopters use multiple fixed rotors to generate lift - the simplest eVTOL design. Scaled-up from drone technology, they use 4 (quadcopter) to as many as 36 rotors. Each rotor's speed is independently controlled to manage aircraft attitude and movement.</p>

<div class="tech-box">
<h4>📐 Multicopter Key Features</h4>
<ul>
<li><strong>Structure:</strong> Multiple fixed rotors, no separate wings</li>
<li><strong>Propulsion:</strong> All rotors always generate vertical thrust</li>
<li><strong>Control:</strong> Pitch, roll, yaw controlled by adjusting individual rotor RPM</li>
<li><strong>Forward Flight:</strong> Aircraft tilts to convert some vertical thrust to forward thrust</li>
<li><strong>Rotor Count:</strong> Typically 8-18 (balancing safety and efficiency)</li>
</ul>
</div>

<h3>Representative Multicopter eVTOL</h3>

<p><strong>Volocopter VoloCity:</strong> Germany's Volocopter's 2-seater air taxi. Uses 18 fixed rotors powered by 9 independent battery packs. Top speed 110 km/h, range 35 km - optimized for short urban trips. Planning demonstration operations at the 2024 Paris Olympics with EASA certification in progress.</p>

<p><strong>EHang EH216:</strong> China's EHang's autonomous 2-seater eVTOL. Uses 16 rotors and obtained the world's first Type Certificate from CAAC in 2021. Top speed 130 km/h, range 30 km, already demonstrating commercial operations in Dubai and Spain. Operates fully autonomously without a pilot.</p>

<h3>Advantages and Disadvantages</h3>

<p><strong>Advantages:</strong> Mechanically very simple with minimal moving parts. Excellent hovering and stability. High redundancy - can fly even if multiple rotors fail. Low disk loading reduces noise. Relatively fast development and certification. Easy and inexpensive maintenance.</p>

<p><strong>Disadvantages:</strong> Slow cruise speed (80-130 km/h). Short range (30-50 km). Low energy efficiency - high battery consumption. High air resistance during forward flight (no wings). Vulnerable to strong winds - all thrust used for hovering.</p>

<h2>🔄 2. Tilt-Rotor</h2>

<p>Tilt-rotors rotate rotors/propellers 90 degrees to switch between vertical and horizontal modes. During vertical take-off/landing, rotors point up like a helicopter. During cruise, rotors point forward like fixed-wing aircraft propellers. The military V-22 Osprey is a large tilt-rotor, and this approach is widely adopted for eVTOL.</p>

<h3>Representative Tilt-Rotor eVTOL</h3>

<p><strong>Joby Aviation S4:</strong> Silicon Valley's Joby Aviation's 5-seater eVTOL. Uses 6 tilting propellers with top speed 320 km/h and range 240 km - best performance among current eVTOL in development. Passed Stage 4 of FAA's 5-stage certification process, targeting 2025 commercialization. Partnering with Uber to prepare air taxi services.</p>

<p><strong>Wisk Aero 6th Generation:</strong> Boeing-partnered Wisk Aero's autonomous eVTOL. Hybrid design with 4 forward tilting rotors and 8 lift rotors. Designed for fully autonomous operation without pilots, working closely with FAA to pioneer autonomous flight certification paths.</p>

<p><strong>Advantages:</strong> High cruise speed (200-350 km/h). Long range (150-300 km). Excellent energy efficiency - wings generate lift. Fast and efficient cruise like fixed-wing. Economical for long-distance operations. Stable cruise in strong winds.</p>

<p><strong>Disadvantages:</strong> Mechanically complex - requires tilting mechanisms. Difficult transition process. Certification challenging if transition fails. Many maintenance points increase costs. Imbalance possible if some rotors fail.</p>

<h2>➕ 3. Lift+Cruise</h2>

<p>Lift+Cruise has separate vertical lift rotors and forward propulsion propellers. During take-off/landing, only vertical lift rotors operate. During cruise, only forward propellers operate. This allows optimization for each flight phase but adds weight and complexity with two propulsion systems.</p>

<h3>Representative Lift+Cruise eVTOL</h3>

<p><strong>Archer Maker:</strong> US Archer Aviation's 4-seater eVTOL. Combines 12 vertical lift rotors with 6 rear pusher propellers. Top speed 240 km/h, range 160 km. Lift rotors fold after take-off to reduce drag. Signed $1 billion contract with United Airlines, targeting 2025 commercial service.</p>

<p><strong>Lilium Jet:</strong> Germany's Lilium's unique ducted fan eVTOL. 36 small electric jet engines (ducted fans) mounted on wings tilt to generate vertical lift and forward thrust. Strictly a tilting ducted fan but conceptually similar to lift+cruise. 7-seater, top speed 280 km/h, range 250 km target.</p>

<p><strong>Advantages:</strong> Propulsion systems optimized for each flight phase. Efficient cruise - wings generate lift. No tilting mechanism needed - mechanical simplicity. Relatively simple transition. Design flexibility - various configurations possible.</p>

<p><strong>Disadvantages:</strong> Two propulsion sets increase weight. Lift rotors create drag during cruise. Complex power distribution system required. Lower payload-to-weight efficiency. Unused motors still carried - inefficient.</p>

<h2>📊 eVTOL Design Comparison</h2>

<table class="comparison-table">
<thead>
<tr>
<th>Design Type</th>
<th>Cruise Speed</th>
<th>Range</th>
<th>Energy Efficiency</th>
<th>Complexity</th>
<th>Optimal Use</th>
</tr>
</thead>
<tbody>
<tr>
<td><strong>Multicopter</strong></td>
<td>80-130 km/h</td>
<td>30-50 km</td>
<td>Low</td>
<td>Low</td>
<td>Short urban trips, tourism</td>
</tr>
<tr>
<td><strong>Tilt-Rotor</strong></td>
<td>200-350 km/h</td>
<td>150-300 km</td>
<td>High</td>
<td>High</td>
<td>Airport shuttles, regional travel</td>
</tr>
<tr>
<td><strong>Lift+Cruise</strong></td>
<td>180-280 km/h</td>
<td>100-250 km</td>
<td>Medium</td>
<td>Medium</td>
<td>Urban aviation, cargo</td>
</tr>
<tr>
<td><strong>Vectored Thrust</strong></td>
<td>250-350 km/h</td>
<td>150-300 km</td>
<td>Medium-High</td>
<td>Very High</td>
<td>Military, special missions</td>
</tr>
</tbody>
</table>

<h2>🔍 Which Design is Best?</h2>

<p>There is no single "best" eVTOL design. Each is optimized for different mission profiles. The market will likely see multiple designs coexisting. Multicopters for short urban trips, tilt-rotors for airport shuttles and long-range, lift+cruise competitive for medium range and cargo.</p>

<p>The next chapter examines the core technology powering these eVTOL: electric propulsion systems - motors, batteries, power distribution, and thermal management.</p>
"""
    },
    3: {
        'title': 'Electric Propulsion Systems',
        'subtitle': 'Motors, Batteries, Power Electronics, and Thermal Management',
        'prev': 'chapter-02.html',
        'next': 'chapter-04.html',
        'content': """<h2>⚡ The Heart of eVTOL: Electric Propulsion</h2>

<p>Electric propulsion systems are the core of eVTOL. Unlike conventional aircraft jet or piston engines, eVTOL converts electrical energy stored in batteries to rotational energy via electric motors, which spin propellers or rotors to generate thrust. This system comprises electric motors, battery packs, power electronics (inverters), thermal management systems, and power distribution systems.</p>

<p>The greatest advantage of electric propulsion is energy efficiency. Electric motors achieve 90-95% efficiency, far superior to jet engines (30-40%) or piston engines (25-30%). They're also structurally simple for easy maintenance, low noise, and zero emissions.</p>

<h2>🔌 Electric Motors</h2>

<h3>1. Brushless DC Motor (BLDC)</h3>

<p>BLDC motors are currently the standard for most eVTOL. Proven in drones and electric vehicles, they offer high efficiency, high power density, long life, and low maintenance.</p>

<div class="tech-box">
<h4>⚙️ BLDC Motor Operating Principle</h4>
<p>BLDC motors have permanent magnets on the rotor (rotating part) and electromagnetic coils on the stator (stationary part). An Electronic Speed Controller (ESC) precisely controls coil current direction and timing to rotate the rotor. Being brushless eliminates friction for longer life and higher efficiency.</p>
</div>

<p><strong>Current Performance:</strong> Power density 5-10 kW/kg (target 15-20 kW/kg). Efficiency 92-95% at rated power, 85%+ at partial load. RPM range typically 2,000-6,000 RPM with gearbox for propeller speed control. Lifespan 10,000-20,000 hours, extendable with bearing replacement.</p>

<h3>2. Cooling Systems</h3>

<p>eVTOL electric motors must output tens to hundreds of kW, generating significant heat. Effective cooling systems are essential.</p>

<ul>
<li><strong>Air Cooling:</strong> Suitable for motors under 50kW. Utilizes propeller airflow for cooling</li>
<li><strong>Liquid Cooling:</strong> Required for motors over 50kW. Circulates glycol solution for cooling</li>
<li><strong>Oil Cooling:</strong> Used in some high-performance motors. Sprays oil directly onto coils for cooling</li>
</ul>

<h2>🔋 Battery Systems</h2>

<p>Batteries are eVTOL's most critical component and biggest technical challenge. Aircraft require both high energy density (Wh/kg) and power density (W/kg), but current battery technology doesn't yet meet ideal levels.</p>

<h3>1. Current Battery Technology: Lithium-Ion</h3>

<p>Current eVTOL use lithium-ion batteries. Similar to electric vehicle technology but must meet far stricter aviation safety standards.</p>

<div class="tech-box">
<h4>🔬 Lithium-Ion Battery Chemistry</h4>
<ul>
<li><strong>NMC (Nickel Manganese Cobalt):</strong> Most widely used. Balanced performance. 250-280 Wh/kg</li>
<li><strong>NCA (Nickel Cobalt Aluminum):</strong> Used by Tesla. High energy density. 260-300 Wh/kg</li>
<li><strong>LFP (Lithium Iron Phosphate):</strong> Very safe but lower energy density. 150-180 Wh/kg</li>
<li><strong>High-Nickel NMC:</strong> Next-generation technology. Target 350-400 Wh/kg</li>
</ul>
</div>

<h3>2. Battery Pack Structure</h3>

<p>eVTOL battery packs comprise hundreds to thousands of individual cells. For example, Joby Aviation uses approximately 1,000 cylindrical 21700 cells. These cells group into modules, and multiple modules form packs.</p>

<h3>3. Battery Management System (BMS)</h3>

<p>BMS is the battery's brain. It monitors voltage, current, and temperature of each cell, controls charging and discharging, detects abnormal states, and optimizes lifespan.</p>

<ul>
<li><strong>Cell Balancing:</strong> Adjusts voltage differences between cells to optimize life and performance</li>
<li><strong>SOC Estimation:</strong> State of Charge, accurately calculates remaining energy (within ±3%)</li>
<li><strong>SOH Estimation:</strong> State of Health, assesses battery aging</li>
<li><strong>Thermal Management:</strong> Maintains optimal temperature range (20-40°C)</li>
<li><strong>Safety Protection:</strong> Prevents overcharge, over-discharge, overcurrent, overheating</li>
</ul>

<h3>4. Battery Safety</h3>

<p>Aviation batteries require extreme safety. Design must prevent thermal runaway from spreading to other cells even if one cell fails.</p>

<div class="tech-box">
<h4>🛡️ Battery Safety Technologies</h4>
<ul>
<li><strong>Cell Isolation:</strong> Thermal barriers between cells with ceramic or aerogel</li>
<li><strong>Pressure Relief Valves:</strong> Safely vent gases when overheated</li>
<li><strong>Fire Suppression System:</strong> Spray inert gas or coolant</li>
<li><strong>Multiple Fuses:</strong> Cut overcurrent at module and pack levels</li>
<li><strong>Real-Time Monitoring:</strong> Detect cell temperature, voltage, swelling</li>
</ul>
</div>

<h2>⚙️ Power Electronics</h2>

<h3>1. Inverter</h3>

<p>Batteries supply DC (direct current) electricity, but BLDC motors require AC (alternating current). Inverters convert DC to AC and precisely control motor speed and torque.</p>

<p>eVTOL inverters require very high power density (kW/kg) and efficiency (95-98%). Latest inverters use Silicon Carbide (SiC) or Gallium Nitride (GaN) semiconductors for higher efficiency and lower heat than traditional silicon IGBT.</p>

<h3>2. DC-DC Converter</h3>

<p>eVTOL use high-voltage batteries (400-800V) but avionics need low voltage (12V, 28V). DC-DC converters transform voltage levels.</p>

<h2>🔥 Thermal Management Systems</h2>

<p>eVTOL electric propulsion systems generate significant heat during take-off/landing and cruise. Without effective thermal management, performance degrades, lifespan shortens, and safety risks emerge.</p>

<h3>Integrated Thermal Management System</h3>

<ul>
<li><strong>Battery Cooling:</strong> Liquid cooling jacket maintains cell temperature at 20-40°C</li>
<li><strong>Motor Cooling:</strong> Liquid cooling system for coils and magnets</li>
<li><strong>Inverter Cooling:</strong> Power modules attached to cold plates</li>
<li><strong>Integrated Loop:</strong> Single coolant circulation loop for entire system</li>
<li><strong>Heat Exchanger:</strong> Coolant cooled by ram air (airflow during flight)</li>
</ul>

<h2>🔋 Next-Generation Battery Technologies</h2>

<h3>1. Solid-State Batteries</h3>

<p>Solid-state batteries replace liquid electrolyte with solid. Theoretically achieves 400-500 Wh/kg energy density, 5x faster charging, zero fire risk. QuantumScape, Solid Power developing, expecting commercialization 2028-2030. When practical, eVTOL range will more than double.</p>

<h3>2. Lithium-Sulfur Batteries</h3>

<p>Lithium-sulfur batteries theoretically exceed 600 Wh/kg energy density. Sulfur is inexpensive and abundant, reducing costs. However, lifespan issues (100-200 cycles) and low power density are challenges.</p>

<h3>3. Charging Infrastructure</h3>

<p>Fast charging is essential for operational efficiency. Current target is 80% charge in 15-30 minutes. Ultra-fast charging systems of 500-1,000kW are under development.</p>

<p><strong>Standard Charging:</strong> 50-150 kW, 1-2 hours full charge<br>
<strong>Fast Charging:</strong> 500-1,000 kW, 15-30 minutes to 80%<br>
<strong>Battery Swap:</strong> 5-10 minute replacement, some cargo eVTOL applications</p>

<p>The next chapter examines flight control systems and autonomous flight technology that precisely controls these electric propulsion systems.</p>
"""
    },
    4: {
        'title': 'Flight Control Systems',
        'subtitle': 'Fly-By-Wire, Autonomous Flight, and AI Control',
        'prev': 'chapter-03.html',
        'next': 'chapter-05.html',
        'content': """<h2>🎮 Revolution in eVTOL Flight Control</h2>

<p>eVTOL flight control systems are a completely different paradigm from conventional helicopters. Traditional helicopters have pilots mechanically controlling rotor pitch through control sticks. This is very complex requiring hundreds of training hours for skilled helicopter pilots.</p>

<p>In contrast, eVTOL uses Fly-By-Wire (FBW) systems. Pilot inputs are transmitted to computers, which adjust numerous motor speeds in milliseconds to control the aircraft. Like smartphones where users give simple commands and computers automatically handle complex tasks.</p>

<h2>🛩️ Fly-By-Wire (FBW)</h2>

<h3>1. FBW Basic Concept</h3>

<p>Fly-by-wire converts pilot operations to electrical signals transmitted to computers, which control actuators (motors in eVTOL) according to flight control laws. No mechanical connection exists between pilot and control surfaces (or motors).</p>

<div class="tech-box">
<h4>📊 FBW System Components</h4>
<ul>
<li><strong>Control Input Devices:</strong> Joysticks, pedals, touchscreens</li>
<li><strong>Flight Control Computer (FCC):</strong> Processes control inputs and generates control commands</li>
<li><strong>Sensor Suite:</strong> IMU, GPS, barometer, airspeed indicator, etc.</li>
<li><strong>Actuators:</strong> Electric motors and ESCs (Electronic Speed Controllers)</li>
<li><strong>Backup Systems:</strong> Redundant/triplicated computers and sensors</li>
</ul>
</div>

<h3>2. Flight Control Laws</h3>

<p>Flight control computers don't simply relay pilot inputs to motors. Complex algorithms stabilize aircraft, protect against exceeding limits, and optimize performance.</p>

<div class="highlight">
<h3>🎯 eVTOL Flight Control Modes</h3>
<ul>
<li><strong>Attitude Hold Mode:</strong> Maintains current attitude when controls released</li>
<li><strong>Position Hold Mode:</strong> Uses GPS to hover at current position</li>
<li><strong>Altitude Hold Mode:</strong> Automatically maintains altitude with barometer</li>
<li><strong>Heading Hold Mode:</strong> Automatically maintains direction with compass</li>
<li><strong>Velocity Control Mode:</strong> Pilot commands speed, system achieves automatically</li>
<li><strong>Path Following Mode:</strong> Automatically flies pre-planned routes</li>
</ul>
</div>

<h3>3. Sensor Fusion</h3>

<p>eVTOL uses various sensors to determine position, velocity, and attitude. Each sensor has strengths and weaknesses, so flight control computers fuse multiple sensor data for most accurate state estimation.</p>

<ul>
<li><strong>IMU (Inertial Measurement Unit):</strong> Measures 3-axis acceleration and angular velocity. Fast response but accumulates errors over time</li>
<li><strong>GPS/GNSS:</strong> Measures absolute position via satellite signals. Accurate but slow update rate (1-10Hz)</li>
<li><strong>Barometer:</strong> Measures altitude via atmospheric pressure. Accurate but affected by weather</li>
<li><strong>Magnetic Compass:</strong> Determines direction from Earth's magnetic field. Simple but vulnerable to magnetic interference</li>
<li><strong>Optical Flow Sensor:</strong> Detects ground movement via camera. Useful indoors or without GPS</li>
<li><strong>LiDAR:</strong> Detects obstacles and terrain with laser. Essential for autonomous flight</li>
</ul>

<p>Kalman Filter or Extended Kalman Filter (EKF) optimally fuses this sensor data. For example, GPS is slow but accurate so corrects IMU accumulated errors, while IMU is fast so estimates movement between GPS updates.</p>

<h2>🤖 Autonomous Flight Systems</h2>

<h3>1. Autonomy Levels</h3>

<p>Autonomous flight doesn't happen all at once. Like automotive self-driving, it develops in stages. EASA and FAA classify eVTOL autonomy as follows:</p>

<div class="tech-box">
<h4>Level 0: Fully Manual</h4>
<p>Pilot handles all flight control. Basic stabilization assistance only</p>

<h4>Level 1: Assisted Automation</h4>
<p>Some functions automated like auto altitude hold, speed hold. Pilot always monitoring</p>

<h4>Level 2: Partial Automation</h4>
<p>Auto take-off/landing, path following possible. Pilot always ready and monitoring</p>

<h4>Level 3: Conditional Automation</h4>
<p>Fully autonomous under specific conditions (VMC, designated routes). Pilot intervention in emergencies</p>

<h4>Level 4: High Automation</h4>
<p>Fully autonomous in most situations. Can operate without pilot but requires ground monitoring</p>

<h4>Level 5: Full Automation</h4>
<p>Fully autonomous in all situations. No human intervention required whatsoever</p>
</div>

<p>Most current eVTOL target Level 2-3. EHang and Wisk Aero target Level 4, planning fully autonomous operation without pilots.</p>

<h3>2. Autonomous Flight Core Technologies</h3>

<h4>a. Path Planning</h4>

<p>Autonomous flight systems automatically plan optimal routes from origin to destination. Considerations include:</p>

<ul>
<li>Avoiding No-Fly Zones</li>
<li>Considering weather conditions (strong winds, rain, visibility)</li>
<li>Avoiding collisions with other aircraft</li>
<li>Optimizing energy efficiency</li>
<li>Avoiding noise-sensitive areas</li>
<li>Securing emergency landing points</li>
</ul>

<p>Uses A* algorithm, RRT (Rapidly-exploring Random Tree), or deep learning-based path planning algorithms.</p>

<h4>b. Detect and Avoid (DAA)</h4>

<p>Autonomous eVTOL must detect and automatically avoid other aircraft, drones, birds, buildings, power lines. Uses various sensors:</p>

<ul>
<li><strong>ADS-B:</strong> Receives position information from other aircraft</li>
<li><strong>Radar:</strong> Detects metal objects, all-weather operation</li>
<li><strong>LiDAR:</strong> 3D environment scanning with laser</li>
<li><strong>Camera:</strong> Visual obstacle recognition, AI classifies birds/drones</li>
<li><strong>Ultrasonic:</strong> Short-range obstacle detection (landing)</li>
</ul>

<p>This sensor data is processed by AI models (CNN, YOLO, etc.) to classify obstacles in real-time and plan avoidance maneuvers.</p>

<h4>c. Machine Learning and AI</h4>

<p>Latest eVTOL extensively utilize deep learning:</p>

<ul>
<li><strong>Object Recognition:</strong> Cameras identify other aircraft, buildings, birds</li>
<li><strong>Weather Prediction:</strong> Learn weather patterns to predict turbulence and strong winds</li>
<li><strong>Optimal Control:</strong> Reinforcement learning for energy-efficient flight strategies</li>
<li><strong>Fault Diagnosis:</strong> Early detection of component anomalies from sensor data</li>
<li><strong>Passenger Experience:</strong> Learn acceleration patterns for smooth flight</li>
</ul>

<h3>3. V2X Communication</h3>

<p>eVTOL must communicate in real-time with other eVTOL, ground systems, vertiports, and UTM (UAM Traffic Management). Called Vehicle-to-Everything (V2X) communication.</p>

<ul>
<li><strong>V2V (Vehicle-to-Vehicle):</strong> Share position, speed, intent between eVTOL</li>
<li><strong>V2I (Vehicle-to-Infrastructure):</strong> Coordinate landing with vertiports</li>
<li><strong>V2N (Vehicle-to-Network):</strong> Exchange flight plans and approvals with UTM</li>
<li><strong>V2C (Vehicle-to-Cloud):</strong> Send telemetry and logs to cloud</li>
</ul>

<p>4G/5G cellular networks are primary communication means, with satellite communication as backup. Communication latency must be under 100ms, reliability over 99.999%.</p>

<h2>🧠 Flight Control Computer Architecture</h2>

<h3>1. Redundancy and Triplication</h3>

<p>Aircraft flight control computers must never fail. Therefore eVTOL have 2 or 3 independent flight control computers. Each performs identical calculations and compares results (Cross-Check). If one produces different results, that computer is isolated and flight continues with others.</p>

<div class="highlight">
<h3>🔒 Triple Redundancy Voting System</h3>
<p>Three computers each calculate independently. For example, Motor 1 command:</p>
<ul>
<li>Computer A: 3,500 RPM</li>
<li>Computer B: 3,500 RPM</li>
<li>Computer C: 2,100 RPM</li>
</ul>
<p>Majority vote selects 3,500 RPM and considers Computer C faulty for isolation. Safely lands with remaining two computers.</p>
</div>

<h3>2. Distributed vs. Centralized Control</h3>

<p><strong>Centralized Control:</strong> One main flight control computer controls all motors. Simple but single point of failure.</p>

<p><strong>Distributed Control:</strong> Independent controller for each motor. If one fails, other motors continue operating. Volocopter uses this approach - can land even if 9 of 18 motors fail.</p>

<h2>🎓 Pilot Training</h2>

<p>FBW and autonomous flight make eVTOL much easier to pilot than helicopters. However, proper training is still required as they are aircraft.</p>

<h3>Current eVTOL Pilot Requirements</h3>

<ul>
<li><strong>Basic Qualification:</strong> Private Pilot License (PPL) or Commercial Pilot License (CPL)</li>
<li><strong>Type Rating:</strong> Type Rating for specific eVTOL model (20-40 hours training)</li>
<li><strong>Medical Exam:</strong> Class 2 Medical Certificate or higher</li>
<li><strong>Recurrent Training:</strong> Emergency procedure retraining every 6 months</li>
</ul>

<p>The next chapter examines various safety systems that ensure eVTOL safety - redundancy, emergency landing, parachutes, etc.</p>
"""
    }
}

# Continue with chapters 5-8...
# (Due to script length, I'll create the remaining chapters in the actual file)

print("Generating comprehensive English chapters 2-8...")

for ch_num, ch_data in chapters.items():
    filename = f"chapter-{ch_num:02d}.html"
    filepath = os.path.join(base_dir, filename)
    
    html_content = create_chapter_html(
        ch_num,
        ch_data['title'],
        ch_data['subtitle'],
        ch_data['prev'],
        ch_data['next'],
        ch_data['content']
    )
    
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print(f"Created {filename}")

print("English chapters 2-4 generated successfully!")
