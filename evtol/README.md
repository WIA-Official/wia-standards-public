# WIA-SPACE-019: Electric Vertical Take-Off and Landing (eVTOL) Standard 🚁

> **홍익인간 (弘益人間) (홍익인간) - Benefit All Humanity**

Complete documentation standard for Electric Vertical Take-Off and Landing (eVTOL) aircraft technology, design principles, propulsion systems, flight control, safety, certification, and future development.

## Overview

WIA-SPACE-019 provides comprehensive documentation of eVTOL technology, from fundamental principles through detailed technical systems to future development roadmaps. This standard serves as both technical reference and educational resource for the emerging electric vertical aviation industry.

## Repository Structure

```
evtol/
├── index.html                      # Main landing page
├── README.md                       # This file
├── spec/
│   └── WIA-SPACE-019-v1.0.md      # Technical specification
└── ebook/
    ├── ko/                         # Korean ebook
    │   ├── index.html             # Table of contents
    │   ├── chapter-01.html        # eVTOL 개요
    │   ├── chapter-02.html        # eVTOL 설계 유형
    │   ├── chapter-03.html        # 전기 추진 시스템
    │   ├── chapter-04.html        # 비행 제어 시스템
    │   ├── chapter-05.html        # 안전 시스템
    │   ├── chapter-06.html        # 감항 인증
    │   ├── chapter-07.html        # eVTOL 제조사
    │   └── chapter-08.html        # eVTOL 미래 전망
    └── en/                         # English ebook
        ├── index.html             # Table of contents
        ├── chapter-01.html        # eVTOL Overview
        ├── chapter-02.html        # eVTOL Design Types
        ├── chapter-03.html        # Electric Propulsion Systems
        ├── chapter-04.html        # Flight Control Systems
        ├── chapter-05.html        # Safety Systems
        ├── chapter-06.html        # Airworthiness Certification
        ├── chapter-07.html        # eVTOL Manufacturers
        └── chapter-08.html        # Future of eVTOL
```

## Quick Start

### View Documentation

1. **Web Interface:** Open `index.html` in your browser
2. **Korean Ebook:** Navigate to `ebook/ko/index.html`
3. **English Ebook:** Navigate to `ebook/en/index.html`
4. **Technical Spec:** Read `spec/WIA-SPACE-019-v1.0.md`

### Topics Covered

#### Chapter 1: eVTOL Overview
- What is eVTOL: Electric Vertical Take-Off and Landing
- Evolution from helicopters to electric VTOL aircraft
- Why eVTOL: sustainability, noise reduction, urban mobility
- VTOL principles vs. conventional fixed-wing aircraft
- Applications: air taxi, cargo delivery, emergency services, tourism
- Vision for electric aviation future

#### Chapter 2: eVTOL Design Types
- **Multicopter:** Multiple fixed rotors (Volocopter, EHang)
- **Tilt-Rotor:** Rotating propellers for vertical/horizontal flight (Joby, Bell)
- **Lift+Cruise:** Separate lift and cruise propulsion (Lilium, Archer)
- **Tilt-Wing:** Entire wing tilts (NASA research)
- **Vectored Thrust:** Jet-based thrust vectoring
- Design trade-offs: efficiency, speed, range, complexity
- Comparative performance analysis

#### Chapter 3: Electric Propulsion Systems
- **Electric Motors:** BLDC, PMSM technology and specifications
- **Battery Systems:** Lithium-ion, solid-state, lithium-sulfur, lithium-air
- **Battery Management Systems (BMS):** Cell balancing, thermal management
- **Power Electronics:** Inverters, converters, motor controllers
- **Energy Management:** Power distribution and optimization
- **Thermal Systems:** Active cooling for batteries and motors
- Future: high energy density batteries (500+ Wh/kg)

#### Chapter 4: Flight Control Systems
- **Fly-by-Wire (FBW):** Digital flight control architecture
- **Autonomous Flight Levels:** Level 0-5 automation hierarchy
- **Sensor Fusion:** IMU, GPS, barometer, magnetometer integration
- **Flight Control Computers:** Redundant processing architectures
- **Autopilot Systems:** Waypoint navigation and autonomous landing
- **Obstacle Detection:** Computer vision, LiDAR, radar
- **V2X Communication:** Vehicle-to-vehicle and vehicle-to-infrastructure

#### Chapter 5: Safety Systems
- **Redundancy:** N+1, N+2, N/2 motor configurations
- **Battery Redundancy:** Multiple independent modules
- **Flight Control Redundancy:** Triple-redundant computers
- **Fault Detection:** Real-time health monitoring
- **Emergency Landing:** Controlled descent and autorotation
- **Ballistic Recovery System (BRS):** Whole-aircraft parachute
- **Fire Prevention:** Thermal runaway protection
- **Ditching Safety:** Water landing capabilities

#### Chapter 6: Airworthiness Certification
- **FAA (USA):** Part 23 + Special Conditions for Powered Lift
- **EASA (Europe):** SC-VTOL Special Condition for VTOL Aircraft
- **CAAC (China):** Type certification process (EHang first certified)
- **Korean MOLIT:** K-UAM certification standards
- **5-Stage Certification Process:** Basis, compliance, verification, approval, TC
- **Testing Requirements:** Ground, flight, system safety analysis
- **Noise Certification:** 65 dB @ 100m takeoff/landing limit
- **Production Certification:** Quality management and traceability
- **Continuing Airworthiness:** Maintenance and AD compliance

#### Chapter 7: eVTOL Manufacturers
- **Joby Aviation (USA):** Tilt-rotor, 320 km/h, 240 km range, FAA Stage 4
- **Archer Aviation (USA):** Lift+cruise, fast turnaround, United Airlines partnership
- **Wisk Aero (USA):** Boeing-backed, fully autonomous, no pilot
- **Volocopter (Germany):** Multicopter pioneer, 18 rotors, EASA certification
- **Lilium (Germany):** Ducted fan, 36 engines, 280 km/h, long range
- **EHang (China):** First type certified (2021), autonomous operations
- **Hyundai Supernal (Korea):** Mass production capability, 2028 target
- **Vertical Aerospace (UK):** 1,400+ pre-orders, airline partnerships
- Global competition landscape and market positioning

#### Chapter 8: Future of eVTOL
- **Market Outlook:** $1.5T by 2040 (Morgan Stanley), 2.3B passengers/year
- **Technology Roadmap 2025-2050:**
  - 2025-2027: Initial commercialization ($5-8/mile)
  - 2028-2030: Growth and expansion (Level 3 autonomy)
  - 2031-2035: Popularization (Level 4-5 autonomy, $2-4/mile)
  - 2036-2040: Maturity (hydrogen fuel cells, $1-2/mile)
  - 2041-2050: Personal ownership, ultra-high-speed, 3D traffic networks
- **Next-Generation Technologies:**
  - Solid-state batteries (400-500 Wh/kg by 2028)
  - Lithium-sulfur (600-800 Wh/kg by 2032)
  - Lithium-air (1,000+ Wh/kg by 2040)
  - Hydrogen fuel cells (500+ km range)
  - Fully autonomous flight (pilotless by 2030s)
  - 3D altitude traffic networks
- **Social Impact:** Urban planning, economic development, environmental benefits
- **Challenges:** Battery density, infrastructure, regulations, public acceptance
- **Vision for 2050:** Personal eVTOL ownership, sky highways, carbon-neutral operations

## Key Features

### 🌐 Bilingual Documentation
- Complete Korean and English ebooks
- Technical specification in English
- Industry-standard terminology and definitions

### 📚 Comprehensive Coverage
- Historical evolution from helicopters to eVTOL
- Detailed technical systems (design, propulsion, control)
- Safety systems and certification processes
- Global manufacturer landscape and competition
- Future technology roadmap through 2050

### 🎨 Modern Design
- Dark theme optimized for reading (#1a1a2e background, #e94560 accent)
- Responsive layout for all devices
- Clear typography and visual hierarchy
- Interactive navigation between chapters

### 🔬 Educational Value
- Suitable for students, educators, engineers, manufacturers, regulators
- Real-world technical specifications and performance data
- Industry examples from leading manufacturers
- Comprehensive certification requirements
- Inspiring vision for electric aviation future

## Technical Specifications

### eVTOL Design Configurations

| Configuration | Hover Efficiency | Cruise Speed | Range | Complexity | Examples |
|---------------|------------------|--------------|-------|------------|----------|
| **Multicopter** | High | Low (100-130 km/h) | Short (30-50 km) | Low | Volocopter, EHang |
| **Tilt-Rotor** | Medium | High (250-320 km/h) | Long (200-250 km) | High | Joby, Bell Nexus |
| **Lift+Cruise** | Medium-High | Medium-High (220-280 km/h) | Medium-Long (140-250 km) | Medium | Archer, Lilium |
| **Tilt-Wing** | Medium | Very High (300+ km/h) | Very Long (300+ km) | Very High | NASA X-57 |

### Battery Technology Evolution

| Technology | Energy Density | Timeline | Advantages | Challenges |
|------------|----------------|----------|------------|------------|
| **Lithium-Ion (Current)** | 250-300 Wh/kg | 2020-2027 | Proven, available | Limited range, fire risk |
| **Solid-State** | 400-500 Wh/kg | 2028-2032 | Safer, faster charging | Manufacturing complexity |
| **Lithium-Sulfur** | 600-800 Wh/kg | 2032-2038 | High capacity, low cost | Cycle life issues |
| **Lithium-Air** | 1,000+ Wh/kg | 2040+ | Theoretical maximum | Research stage |

### Leading eVTOL Manufacturers

| Manufacturer | Country | Type | Passengers | Speed | Range | Status |
|--------------|---------|------|------------|-------|-------|--------|
| **Joby Aviation** | USA | Tilt-rotor | 4+1 | 320 km/h | 240 km | FAA Stage 4/5 |
| **Archer Aviation** | USA | Lift+cruise | 4+1 | 240 km/h | 160 km | FAA certification |
| **Wisk Aero** | USA | Lift+cruise | 4 | 220 km/h | 140 km | Autonomous |
| **Volocopter** | Germany | Multicopter | 1+1 | 110 km/h | 35 km | EASA certified |
| **Lilium** | Germany | Ducted fan | 6+1 | 280 km/h | 250 km | EASA early stage |
| **EHang** | China | Multicopter | 2 | 130 km/h | 30 km | Type certified (2021) |
| **Hyundai Supernal** | Korea | Tilt-rotor | 4-5 | 240+ km/h | 100+ km | FAA certification |
| **Vertical Aerospace** | UK | Lift+cruise | 4+1 | 320 km/h | 160 km | Pre-orders: 1,400+ |

### Certification Authorities

- **FAA (USA):** Part 23 + Special Conditions for Powered Lift category
- **EASA (Europe):** SC-VTOL (Special Condition for VTOL Aircraft)
- **CAAC (China):** EH216 first type certified (2021)
- **Korean MOLIT:** K-UAM certification standards development
- **International:** ICAO standards harmonization in progress

## Standards Compliance

This standard references and complies with:
- **FAA Part 23:** Small Aircraft Airworthiness Standards
- **FAA Special Conditions:** Powered Lift Category for eVTOL
- **EASA SC-VTOL:** Special Condition for VTOL Aircraft
- **ICAO Annex 16:** Aircraft Noise Certification Standards
- **MIL-STD-1553:** Digital time division command/response multiplex data bus
- **DO-178C:** Software Considerations in Airborne Systems
- **DO-254:** Design Assurance Guidance for Airborne Electronic Hardware
- **SAE AS9100:** Quality Management Systems for Aviation
- **ISO 9001:** Quality Management Systems

## Use Cases

### Education
- Aerospace engineering programs (eVTOL design courses)
- Electrical engineering (propulsion and power systems)
- Aviation safety and certification studies
- High school STEM education and career exploration
- Public outreach and technology literacy

### Professional
- eVTOL manufacturer engineering reference
- Battery and motor supplier specifications
- Flight control software developers
- Certification authority guidelines
- Investor technical due diligence
- Insurance and risk assessment

### Research
- Academic research on electric aviation
- Battery technology development
- Autonomous flight systems
- Noise reduction technologies
- Urban air mobility integration
- Environmental impact assessment

## Global eVTOL Developments

### United States
- **Companies:** Joby, Archer, Wisk, Beta Technologies, Kitty Hawk (retired)
- **Investment:** >$10B raised by eVTOL startups
- **Certification:** FAA developing Powered Lift category pathways
- **Timeline:** First commercial operations targeted 2025-2026
- **Applications:** Airport shuttles, urban air taxi, cargo delivery

### Europe
- **Companies:** Volocopter, Lilium, Vertical Aerospace
- **Certification:** EASA SC-VTOL framework established
- **Cities:** Paris, Munich, London early adopters
- **Events:** Paris 2024 Olympics demonstration flights
- **Goals:** Zero-emission urban aviation by 2030

### China
- **Companies:** EHang (first certified), AutoFlight, XPeng HT
- **Achievement:** EH216 first Type Certificate (2021)
- **Operations:** Commercial flights in Guangzhou, Dubai
- **Advantage:** Rapid certification and deployment
- **Government:** Strong policy support and investment

### South Korea
- **K-UAM Initiative:** Government-led UAM development program
- **Companies:** Hyundai Supernal, Hanwha Systems
- **Timeline:** 2025 K-UAM Grand Challenge, 2028+ commercialization
- **Routes:** Seoul-Incheon Airport initial corridor
- **Investment:** ₩2 trillion (~$1.7B) through 2030
- **Infrastructure:** Vertiports at Gimpo, Yeouido, Gangnam, Jamsil

### Middle East
- **UAE:** Dubai early adopter, EHang operations
- **Saudi Arabia:** NEOM smart city integration
- **Investment:** Billions in eVTOL infrastructure
- **Vision:** 25% autonomous transport by 2030

## Market Outlook

### Global eVTOL/UAM Market Projections

| Source | Market Size | Timeline | Notes |
|--------|-------------|----------|-------|
| **Morgan Stanley** | $1.5T | by 2040 | Air taxi $850B, Cargo $650B |
| **Roland Berger** | $90B | by 2050 | 2.3B annual passengers |
| **McKinsey** | $500B | by 2035 | Cumulative market size |
| **ARK Invest** | $800B | by 2030 | Including autonomous flight |

### Technology Development Timeline

| Period | Key Milestones | Price per Mile | Passengers/Year |
|--------|----------------|----------------|-----------------|
| **2025-2027** | Initial commercialization, pilot onboard | $5-8 | 1 million |
| **2028-2030** | Level 3 autonomy, solid-state batteries | $3-5 | 150 million |
| **2031-2035** | Level 4-5 autonomy, scheduled routes | $2-4 | 1.5 billion |
| **2036-2040** | Hydrogen fuel cells, carbon neutral | $1-2 | 2.3 billion |
| **2041-2050** | Personal ownership, 3D traffic networks | <$1 | 5+ billion |

## Contributing

This standard is maintained by the WIA Technical Committee. For updates, corrections, or enhancements:

1. Review current documentation
2. Identify areas for improvement
3. Submit detailed proposals with technical references
4. Follow 홍익인간 (弘益人間) philosophy

Contact: standards@wia-official.org

## Philosophy: 弘익人間

**홍익인간 (Hongik Ingan) - Benefit All Humanity**

Electric Vertical Take-Off and Landing aircraft represent a revolutionary step toward sustainable, accessible, and efficient urban aviation. This standard is published freely to:

- **Educate** future generations of aerospace engineers
- **Inspire** innovation in electric propulsion and autonomous flight
- **Document** eVTOL technology evolution and best practices
- **Accelerate** safe certification and deployment
- **Unite** global efforts toward clean, quiet, efficient aviation
- **Ensure** eVTOL benefits all of humanity, not just the privileged few

## Future Updates

Planned additions to this standard:
- **v1.1:** Advanced autonomous flight systems and AI integration
- **v1.2:** Hydrogen fuel cell hybrid propulsion systems
- **v1.3:** High-altitude operations and weather resistance
- **v1.4:** Inter-city and regional eVTOL operations (500+ km range)
- **v2.0:** Personal eVTOL ownership and urban 3D traffic networks

## Related WIA Standards

- **WIA-SPACE-018:** Urban Air Mobility (UAM) - broader UAM ecosystem
- **WIA-SPACE-001 through WIA-SPACE-017:** Other space and aviation standards
- **WIA-INTENT:** Intention expression framework
- **WIA-OMNI-API:** Universal API standard
- **WIA-AIR-POWER:** Distributed computing standard
- **WIA-AIR-SHIELD:** Security and protection standard

## License

This standard is published under the **弘익人間** (Benefit All Humanity) philosophy.

You are free to:
- **Use** this documentation for any purpose
- **Share** with anyone worldwide
- **Adapt** for your specific educational or research needs
- **Teach** aerospace engineering courses from these materials
- **Implement** eVTOL technologies described herein
- **Build** upon this knowledge to advance electric aviation

We only ask that you:
- Attribute the source (WIA-SPACE-019)
- Share improvements and updates back to the community
- Use knowledge for peaceful and beneficial purposes
- Help make eVTOL technology safe, sustainable, and accessible to all
- Honor the 홍익인간 (弘益人間) philosophy in your work

## Acknowledgments

This standard honors:
- **Aviation pioneers** who dreamed of electric vertical flight
- **Engineers and researchers** developing eVTOL technologies worldwide
- **Test pilots and certification authorities** ensuring flight safety
- **Battery scientists** pushing energy density boundaries
- **Software developers** enabling autonomous flight
- **Regulators** creating frameworks for safe eVTOL operations
- **Investors and entrepreneurs** bringing eVTOL to commercialization
- **Future passengers** who will benefit from clean, quiet, efficient air transportation

From early electric VTOL experiments to today's sophisticated autonomous aircraft, from concept to certification - this is humanity's shared journey toward sustainable aviation.

## Statistics

- **Languages:** 2 (Korean, English)
- **Chapters:** 8 comprehensive chapters per language
- **Total Content:** 250+ pages equivalent
- **Technical Depth:** Industry-grade specifications and engineering details
- **Coverage:** Complete eVTOL technology stack
- **Timeline:** 2010s (early prototypes) to 2050+ (future vision)
- **Manufacturers Covered:** 8 leading global eVTOL companies
- **Certification Standards:** FAA, EASA, CAAC, Korean MOLIT

## Contact

- **Organization:** World Certification Industry Association (WIA)
- **Developer:** SmileStory Inc.
- **Email:** standards@wia-official.org
- **Website:** https://github.com/WIA-Official/wia-standards
- **Philosophy:** 홍익인간 (弘益人間) (홍익인간) - Benefit All Humanity

---

## Quick Links

- 🏠 [Main Landing Page](index.html)
- 🇰🇷 [Korean Ebook](ebook/ko/index.html)
- 🇺🇸 [English Ebook](ebook/en/index.html)
- 📋 [Technical Specification](spec/WIA-SPACE-019-v1.0.md)

---

**Electric Vertical Take-Off and Landing: The Future of Aviation is Electric 🚁**

Together, we rise on silent wings. Together, we reach for cleaner skies. Together, we benefit all humanity.

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
