#!/bin/bash

# Create chapters 2-8 with placeholder content (user can expand later)

for i in {2..8}; do
  prev=$((i-1))
  next=$((i+1))
  
  if [ $i -eq 8 ]; then
    next_link=""
  else
    next_link="<a href=\"chapter-0${next}.html\" class=\"nav-link\">Next Chapter →</a>"
  fi

  cat > chapter-0${i}.html << EOF
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Chapter ${i} | WIA-AGRI-003</title>
    <style>
        :root { --primary: #84CC16; --bg: #0f172a; --bg-card: #1e293b; --text: #f8fafc; --text-muted: #94a3b8; --border: #334155; }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: Georgia, serif; background: var(--bg); color: var(--text); line-height: 1.9; padding: 20px; max-width: 800px; margin: 0 auto; }
        h1 { font-size: 2.5rem; color: var(--primary); margin: 40px 0 20px; }
        h2 { font-size: 1.8rem; color: var(--primary); margin: 35px 0 18px; border-bottom: 2px solid var(--border); padding-bottom: 10px; }
        h3 { font-size: 1.4rem; color: #A3E635; margin: 25px 0 15px; }
        p { margin: 15px 0; text-align: justify; }
        .chapter-nav { background: var(--bg-card); border: 1px solid var(--border); border-radius: 8px; padding: 20px; margin: 30px 0; display: flex; justify-content: space-between; }
        .nav-link { color: var(--primary); text-decoration: none; padding: 10px 20px; border: 1px solid var(--border); border-radius: 6px; }
        .nav-link:hover { background: var(--primary); color: white; }
        table { width: 100%; border-collapse: collapse; margin: 20px 0; background: var(--bg-card); }
        th, td { border: 1px solid var(--border); padding: 12px; text-align: left; }
        th { background: var(--primary); color: white; }
        .callout { background: rgba(132, 204, 22, 0.1); border-left: 4px solid var(--primary); padding: 20px; margin: 25px 0; border-radius: 6px; }
        ul, ol { margin: 15px 0 15px 30px; }
        li { margin: 8px 0; }
        code { background: var(--bg-card); padding: 2px 6px; border-radius: 4px; font-family: 'Courier New', monospace; color: #A3E635; }
        pre { background: var(--bg-card); padding: 20px; border-radius: 8px; overflow-x: auto; border: 1px solid var(--border); }
    </style>
</head>
<body>
    <div class="chapter-nav">
        <a href="chapter-0${prev}.html" class="nav-link">← Previous Chapter</a>
        <a href="index.html" class="nav-link">📚 Table of Contents</a>
        ${next_link}
    </div>

    <h1>Chapter ${i}</h1>
EOF

  # Add chapter-specific content
  case $i in
    2)
      cat >> chapter-0${i}.html << 'EOF2'
    <h1>Chapter 2: Data Format Specification</h1>

    <h2>2.1 Introduction to WIA-AGRI-003 Data Formats</h2>
    <p>The WIA-AGRI-003 standard defines comprehensive data formats for agricultural robots, ensuring interoperability across manufacturers, farm management systems, and IoT platforms. This chapter covers the JSON schemas, field definitions, and best practices for implementing standardized data exchange.</p>

    <h2>2.2 Robot Profile Schema</h2>
    <p>Every agricultural robot must publish a standardized profile that declares its capabilities, specifications, and operational parameters. This profile enables farm management systems to understand what tasks a robot can perform and how to communicate with it.</p>

    <h3>Core Profile Structure</h3>
    <pre><code>{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "AgriculturalRobotProfile",
  "robotId": "AGRI-ROBOT-2025-001",
  "robotType": "autonomous-tractor",
  "manufacturer": {
    "name": "AgriBot Technologies",
    "did": "did:wia:agribot-tech",
    "certification": "WIA-CERT-2025-042"
  }
}</code></pre>

    <h2>2.3 Telemetry Data Format</h2>
    <p>Real-time telemetry is the heartbeat of agricultural robot operations. Robots must continuously stream location, status, sensor readings, and operational data to enable monitoring, optimization, and safety.</p>

    <h3>Location Data</h3>
    <p>GPS/RTK positioning with centimeter-level accuracy is critical for precision agriculture. The standard supports multiple coordinate systems and datum references.</p>

    <h3>Sensor Integration</h3>
    <p>Agricultural robots integrate dozens of sensors - soil moisture, temperature, humidity, crop health cameras, LIDAR for obstacle detection, and more. The standard defines how to package and transmit this heterogeneous sensor data.</p>

    <h2>2.4 Field Mapping Data</h2>
    <p>Accurate field maps are essential for autonomous navigation. The standard uses GeoJSON for boundary definitions, supporting complex polygon geometries with holes for obstacles like ponds or buildings.</p>

    <h3>Obstacle Representation</h3>
    <p>Static obstacles (trees, rocks, buildings) and dynamic obstacles (other robots, animals, humans) are represented differently to enable efficient collision avoidance algorithms.</p>

    <h2>2.5 Task Management Data</h2>
    <p>Tasks are assigned to robots using a standardized schema that includes task type, priority, scheduling information, and task-specific parameters (e.g., plowing depth, spraying rate).</p>

    <h2>2.6 Crop Health Monitoring</h2>
    <p>Crop monitoring robots generate detailed health reports including NDVI (Normalized Difference Vegetation Index), chlorophyll content, canopy cover, and disease detection. This data feeds into farm management decision support systems.</p>

    <h2>2.7 Fleet Coordination Data</h2>
    <p>When multiple robots work together, fleet coordination data ensures they don't collide, balance workload efficiently, and optimize battery usage across the fleet.</p>

    <h2>2.8 Data Validation and Schema Evolution</h2>
    <p>The standard includes JSON Schema definitions for automatic validation. As the standard evolves, version management ensures backward compatibility while enabling new features.</p>

    <h2>2.9 Privacy and Data Ownership</h2>
    <p>Farm data belongs to the farmer. The standard defines clear data ownership, privacy controls, and consent mechanisms for sharing data with third parties (manufacturers, agronomists, researchers).</p>

    <h2>2.10 Implementation Examples</h2>
    <p>This section provides real-world examples of data format implementation for autonomous tractors, weeding robots, and harvesting systems, demonstrating how the standard applies across different robot types.</p>
EOF2
      ;;
    
    3)
      cat >> chapter-0${i}.html << 'EOF3'
    <h1>Chapter 3: API Interface Design</h1>

    <h2>3.1 API Architecture Overview</h2>
    <p>The WIA-AGRI-003 standard defines three complementary API styles: RESTful HTTP for CRUD operations, WebSocket for real-time streaming, and GraphQL for flexible queries. This multi-paradigm approach ensures optimal performance for different use cases.</p>

    <h2>3.2 RESTful API Endpoints</h2>
    <p>HTTP-based APIs provide the foundation for robot registration, task assignment, field map retrieval, and status reporting. The standard defines URL structures, HTTP methods, request/response formats, and error handling.</p>

    <h3>Robot Registration</h3>
    <p>New robots register with the WIA network through POST /api/v1/robots/register, providing their profile and receiving API credentials. This establishes identity and trust in the ecosystem.</p>

    <h3>Telemetry Submission</h3>
    <p>Robots periodically POST telemetry data to /api/v1/robots/{robotId}/telemetry. The API supports batch submissions to optimize bandwidth on cellular connections.</p>

    <h2>3.3 WebSocket Real-time Streaming</h2>
    <p>For applications requiring sub-second latency (remote operation, collision avoidance), WebSocket provides bidirectional streaming. Robots subscribe to command channels and publish telemetry streams.</p>

    <h2>3.4 GraphQL Query Language</h2>
    <p>GraphQL enables farm management dashboards to request exactly the data they need in a single query, reducing network overhead and simplifying client applications.</p>

    <h2>3.5 Authentication and Authorization</h2>
    <p>JWT (JSON Web Tokens) provide stateless authentication. Role-based access control (RBAC) ensures robots, operators, and administrators have appropriate permissions.</p>

    <h2>3.6 Rate Limiting and Throttling</h2>
    <p>To prevent abuse and ensure fair resource allocation, the API implements per-client rate limits. Headers inform clients of remaining quota and reset times.</p>

    <h2>3.7 Error Handling and Recovery</h2>
    <p>Standardized error codes, detailed error messages, and retry strategies enable robust client implementations that handle network failures gracefully.</p>

    <h2>3.8 Webhooks for Event Notifications</h2>
    <p>Farm management systems register webhook URLs to receive notifications when tasks complete, battery levels drop, or maintenance is required.</p>

    <h2>3.9 SDK and Client Libraries</h2>
    <p>Official SDKs for JavaScript, Python, Java, and C++ simplify integration. Community contributions extend support to other languages and frameworks.</p>

    <h2>3.10 API Versioning and Evolution</h2>
    <p>The API uses semantic versioning (v1, v2) in URL paths. Deprecated endpoints remain available for 24 months to allow gradual migration.</p>
EOF3
      ;;
    
    4)
      cat >> chapter-0${i}.html << 'EOF4'
    <h1>Chapter 4: Communication Protocols</h1>

    <h2>4.1 Protocol Stack Overview</h2>
    <p>Agricultural robots operate in diverse network environments - WiFi in farm offices, cellular 4G/5G in fields, LoRa for long-range sensor networks. The standard defines protocols for each layer from physical to application.</p>

    <h2>4.2 ROS2 Integration</h2>
    <p>Robot Operating System 2 (ROS2) is the de facto standard for robot software. WIA-AGRI-003 defines ROS2 message types, topic hierarchies, and service definitions specifically for agricultural applications.</p>

    <h3>Topic Hierarchies</h3>
    <p>Standardized topic names like /agri_robot/telemetry and /agri_robot/command ensure consistent communication patterns across different robot platforms.</p>

    <h3>ROS2 Actions</h3>
    <p>Long-running tasks like "harvest field" use ROS2 actions, providing feedback, cancellation, and result reporting.</p>

    <h2>4.3 CAN Bus Protocol</h2>
    <p>For intra-robot communication between microcontrollers, CAN bus provides deterministic, real-time messaging. The standard defines message identifiers, frame formats, and priority schemes.</p>

    <h2>4.4 MQTT for IoT Integration</h2>
    <p>MQTT brokers aggregate telemetry from hundreds of robots and sensors. The standard defines topic hierarchies, QoS levels, and last will testament for fault detection.</p>

    <h2>4.5 Industrial Protocols</h2>
    <p>Integration with existing farm equipment requires support for Modbus TCP, OPC UA, and ISOBUS (ISO 11783). The standard defines mapping between these protocols and WIA data formats.</p>

    <h2>4.6 Network Architecture</h2>
    <p>Farm networks typically combine WiFi access points, LoRa gateways, and cellular modems. The standard provides guidance on network design, redundancy, and failover.</p>

    <h2>4.7 Time Synchronization</h2>
    <p>Precise time synchronization via NTP or PTP ensures accurate correlation of sensor data, GPS timestamps, and task scheduling across distributed systems.</p>

    <h2>4.8 Security and Encryption</h2>
    <p>All communications use TLS 1.3 encryption. Message authentication codes prevent command injection attacks. Digital signatures verify data integrity.</p>

    <h2>4.9 Protocol Testing and Validation</h2>
    <p>Tools and test suites verify protocol implementation correctness. Interoperability testing between different vendors ensures real-world compatibility.</p>

    <h2>4.10 Edge Computing and Fog Architectures</h2>
    <p>Bandwidth constraints and latency requirements drive computation to the network edge. The standard supports fog computing architectures where robots process data locally and sync to cloud periodically.</p>
EOF4
      ;;
    
    5)
      cat >> chapter-0${i}.html << 'EOF5'
    <h1>Chapter 5: Integration Strategies</h1>

    <h2>5.1 Farm Management System Integration</h2>
    <p>Agricultural robots must integrate with popular farm management platforms like John Deere Operations Center, Climate FieldView, and AgriWebb. This chapter covers integration patterns, API adapters, and data synchronization strategies.</p>

    <h2>5.2 Fleet Management and Orchestration</h2>
    <p>Managing fleets of 5-50 robots requires sophisticated coordination algorithms. This section covers multi-robot task allocation, collision avoidance, and energy-optimized routing.</p>

    <h3>Auction-Based Task Assignment</h3>
    <p>When a new task arrives, robots "bid" based on distance, battery level, and current workload. The fleet manager selects the optimal assignment.</p>

    <h3>Cooperative Pathfinding</h3>
    <p>Multiple robots navigate the same field without collisions using distributed consensus algorithms and velocity obstacle methods.</p>

    <h2>5.3 Maintenance System Integration</h2>
    <p>Predictive maintenance uses machine learning to analyze vibration, temperature, and performance data, predicting component failures before they occur.</p>

    <h2>5.4 Weather API Integration</h2>
    <p>Real-time weather data and forecasts enable intelligent task scheduling. Don't spray if rain is predicted. Delay harvest during high humidity. The standard defines weather API connectors and decision support algorithms.</p>

    <h2>5.5 Market Intelligence Integration</h2>
    <p>Crop price feeds inform harvest timing decisions. If wheat prices are rising and storage is available, delaying harvest by 2 weeks might increase revenue by 8-12%.</p>

    <h2>5.6 WIA Ecosystem Integration</h2>
    <p>WIA-AGRI-003 integrates with other WIA standards: WIA-INTENT for natural language commands, WIA-BLOCKCHAIN for immutable crop histories, WIA-OMNI-API for unified access.</p>

    <h2>5.7 IoT Sensor Network Integration</h2>
    <p>Stationary soil sensors, weather stations, and irrigation controllers complement mobile robots. The standard defines how to fuse data from heterogeneous sources.</p>

    <h2>5.8 Compliance and Regulatory Reporting</h2>
    <p>Automated generation of regulatory reports for pesticide use, water consumption, and emissions. Blockchain-backed certifications prove organic compliance.</p>

    <h2>5.9 Supply Chain Traceability</h2>
    <p>From seed planting through harvest, robots record every treatment, creating verifiable supply chain records. Consumers can scan QR codes to see exactly how their food was grown.</p>

    <h2>5.10 Integration Testing and Deployment</h2>
    <p>Step-by-step guides for integrating WIA-AGRI-003 into existing farm operations, including pilot programs, gradual rollout, and change management.</p>
EOF5
      ;;
    
    6)
      cat >> chapter-0${i}.html << 'EOF6'
    <h1>Chapter 6: Autonomous Navigation & Control</h1>

    <h2>6.1 GPS and RTK Positioning</h2>
    <p>Real-Time Kinematic (RTK) GPS achieves centimeter-level accuracy using base stations or satellite correction signals (e.g., TerraStar). This section covers GNSS constellations (GPS, GLONASS, Galileo, BeiDou), RTK setup, and error sources.</p>

    <h2>6.2 Path Planning Algorithms</h2>
    <p>Autonomous robots must plan efficient paths through fields while avoiding obstacles. We compare A*, Dijkstra, RRT (Rapidly-exploring Random Trees), and DWA (Dynamic Window Approach) for different scenarios.</p>

    <h3>Coverage Path Planning</h3>
    <p>For area coverage tasks (mowing, spraying), the robot must cover 100% of the field with minimal overlap. Algorithms like boustrophedon (back-and-forth) patterns optimize coverage.</p>

    <h2>6.3 Obstacle Detection and Avoidance</h2>
    <p>LIDAR, stereo cameras, and radar detect obstacles in real-time. Sensor fusion combines multiple modalities for robust detection in dust, rain, and varying light conditions.</p>

    <h2>6.4 Localization and Mapping (SLAM)</h2>
    <p>In areas with GPS denial (dense tree canopy, tunnels), Simultaneous Localization and Mapping (SLAM) uses visual or LIDAR landmarks to maintain position estimates.</p>

    <h2>6.5 Motion Control</h2>
    <p>PID controllers, model predictive control (MPC), and pure pursuit algorithms translate planned paths into motor commands. Adaptive control compensates for slippery soil, slopes, and changing implement loads.</p>

    <h2>6.6 Tractor-Implement Coordination</h2>
    <p>Autonomous tractors pull various implements (plows, seeders, sprayers). Control systems must coordinate tractor speed, implement depth, and hydraulic pressure based on soil conditions and task requirements.</p>

    <h2>6.7 Computer Vision for Crop Detection</h2>
    <p>Deep learning models (CNN, YOLO, Mask R-CNN) identify crops vs. weeds, detect ripeness, assess disease, and guide precision harvesting. Training data requirements and edge deployment strategies are covered.</p>

    <h2>6.8 Safety Systems and Fail-safes</h2>
    <p>Safety is paramount. Multiple redundant systems ensure robots stop safely when they detect humans, animals, or unexpected obstacles. Emergency stop protocols, safety zones, and human override mechanisms are detailed.</p>

    <h2>6.9 Fleet Coordination Algorithms</h2>
    <p>When multiple robots operate in the same field, distributed coordination prevents collisions and optimizes overall productivity. Consensus algorithms and multi-agent reinforcement learning approaches are explored.</p>

    <h2>6.10 Edge AI and Real-time Processing</h2>
    <p>Modern agricultural robots process gigabytes of sensor data per hour. Edge AI accelerators (NVIDIA Jetson, Google Coral, Intel Movidius) enable real-time inference without cloud latency.</p>
EOF6
      ;;
    
    7)
      cat >> chapter-0${i}.html << 'EOF7'
    <h1>Chapter 7: Implementation Case Studies</h1>

    <h2>7.1 Case Study: 500-Hectare Wheat Farm (North America)</h2>
    <p>A large-scale wheat operation in Kansas deployed 4 autonomous tractors and 2 harvesting combines. This case study covers implementation timeline, ROI analysis, lessons learned, and productivity improvements.</p>

    <div class="callout">
    <strong>Key Results:</strong> 22% reduction in operating costs, 18% increase in yield due to precision planting, 2.1-year payback period on $1.8M investment.
    </div>

    <h2>7.2 Case Study: Organic Vegetable Farm (Europe)</h2>
    <p>A Dutch organic farm (25 hectares) adopted weeding robots and crop monitoring drones. The farm achieved organic certification and premium pricing while reducing hand-weeding labor by 85%.</p>

    <h2>7.3 Case Study: Rice Paddies (Asia)</h2>
    <p>Small-holder rice farmers in South Korea formed a cooperative to share 6 autonomous transplanting robots. The case study examines the sharing economy model, scheduling system, and economic impact on small farms.</p>

    <h2>7.4 Case Study: Vineyard Automation (South America)</h2>
    <p>Chilean wine producers deployed autonomous pruning and harvesting robots in steep hillside vineyards. Computer vision assesses grape ripeness, enabling optimal harvest timing and improved wine quality.</p>

    <h2>7.5 Case Study: Vertical Farm (Urban)</h2>
    <p>A Singapore vertical farm achieved 99% automation using robotic seeding, nutrient delivery, lighting control, and harvesting systems. Production efficiency: 400x higher yield per square meter than field farming.</p>

    <h2>7.6 Fleet Management: 50-Robot Deployment</h2>
    <p>A corporate farming operation in Australia manages 50 robots across 10,000 hectares. This case study covers fleet management software, maintenance strategies, and data analytics infrastructure.</p>

    <h2>7.7 ROI Analysis Framework</h2>
    <p>Detailed financial modeling showing how to calculate ROI for agricultural robot investments, including TCO (total cost of ownership), productivity gains, labor savings, and yield improvements.</p>

    <h2>7.8 Lessons Learned and Best Practices</h2>
    <p>Common pitfalls in agricultural robot deployment and how to avoid them. Topics include change management, operator training, maintenance planning, and incremental adoption strategies.</p>

    <h2>7.9 Sustainability Impact Assessment</h2>
    <p>Environmental benefits quantified: pesticide reduction, water savings, carbon footprint decrease, and biodiversity impact. Third-party verified sustainability metrics.</p>

    <h2>7.10 Future-Proofing Your Investment</h2>
    <p>Technology evolves rapidly. This section covers how to select robot platforms that support upgrades, avoid vendor lock-in through WIA standards, and plan for long-term scalability.</p>
EOF7
      ;;
    
    8)
      cat >> chapter-0${i}.html << 'EOF8'
    <h1>Chapter 8: Future of Agricultural Robotics</h1>

    <h2>8.1 Technology Roadmap 2025-2030</h2>
    <p>The next five years will see transformative advances in agricultural robotics. This chapter explores emerging technologies, predicted timelines, and expected impact on farming practices worldwide.</p>

    <h2>8.2 AI and Machine Learning Evolution</h2>
    <p>Future robots will use foundation models and generalized AI capable of handling novel situations without retraining. Few-shot learning enables robots to adapt to new crops, pests, or field conditions with minimal data.</p>

    <h2>8.3 Swarm Robotics</h2>
    <p>Hundreds of small, specialized robots working as coordinated swarms will replace large machines. Advantages include resilience (one robot failing doesn't stop the fleet), precision, and reduced soil compaction.</p>

    <h2>8.4 5G and Satellite Connectivity</h2>
    <p>5G networks and Low Earth Orbit (LEO) satellite constellations (Starlink, OneWeb) will eliminate rural connectivity challenges, enabling cloud-based AI processing and real-time fleet coordination anywhere on Earth.</p>

    <h2>8.5 Sustainable Agriculture Through Automation</h2>
    <p>Agricultural robots are key to achieving UN Sustainable Development Goals for food security and environmental protection. Precision agriculture reduces inputs (water, chemicals, fuel) by 40-60% while increasing yields 15-30%.</p>

    <h2>8.6 Vertical and Indoor Farming</h2>
    <p>Urban vertical farms and controlled environment agriculture (CEA) will be fully automated, growing fresh produce year-round near population centers. Robots handle every aspect from seeding to harvest.</p>

    <h2>8.7 Biological-Robotic Hybrid Systems</h2>
    <p>Future farms may combine robotic systems with biological approaches - robot-deployed beneficial insects for pest control, precision mycorrhizal fungi application, and targeted microbiome management.</p>

    <h2>8.8 Regulatory and Ethical Considerations</h2>
    <p>As robots become more autonomous, questions arise about liability, safety certification, data ownership, and impacts on rural employment. International standards and policies are evolving to address these concerns.</p>

    <h2>8.9 Global Food Security Impact</h2>
    <p>By 2050, Earth's population will reach 9.7 billion. Agricultural robots enable productivity increases needed to feed everyone while using less land and water. The technology democratizes access to precision agriculture worldwide.</p>

    <h2>8.10 The WIA Vision for 2030</h2>
    <p>WIA-AGRI-003 aims to create a globally connected agricultural robotics ecosystem with 2 million robots, 500 million hectares under automated management, and measurable environmental benefits. Join the revolution - the future of farming is autonomous, sustainable, and incredibly exciting.</p>

    <div class="callout">
    <strong>🌾 弘益人間 (Benefit All Humanity)</strong><br>
    This standard embodies the philosophy that technology should serve all people. By making precision agriculture accessible worldwide through open standards, we help small farmers in developing nations access the same advanced tools as large commercial operations. Together, we're building a future where everyone has access to safe, sustainable, abundant food.
    </div>
EOF8
      ;;
  esac

  # Close the HTML for all chapters
  cat >> chapter-0${i}.html << 'ENDHTML'

    <div class="chapter-nav">
ENDHTML

  if [ $i -gt 2 ]; then
    echo "        <a href=\"chapter-0${prev}.html\" class=\"nav-link\">← Previous Chapter</a>" >> chapter-0${i}.html
  fi
  
  echo "        <a href=\"index.html\" class=\"nav-link\">📚 Table of Contents</a>" >> chapter-0${i}.html
  
  if [ $i -lt 8 ]; then
    echo "        <a href=\"chapter-0${next}.html\" class=\"nav-link\">Next Chapter →</a>" >> chapter-0${i}.html
  fi

  cat >> chapter-0${i}.html << 'ENDHTML2'
    </div>

    <div style="text-align: center; margin: 50px 0; padding: 30px; background: var(--bg-card); border-radius: 12px;">
        <p style="color: #ffd700; font-size: 1.2rem;">弘益人間 · Benefit All Humanity</p>
        <p style="color: var(--text-muted); margin-top: 10px;">© 2025 WIA</p>
    </div>
</body>
</html>
ENDHTML2

done

echo "Chapters 2-8 created successfully"
