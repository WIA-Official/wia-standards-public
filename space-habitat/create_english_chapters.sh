#!/bin/bash
# Create all 8 English chapters with proper structure and 200+ lines

cd /home/user/wia-standards/space-habitat/ebook/en

# We'll use a Python heredoc approach for efficiency
python3 << 'PYTHON_EN'
import os

# English chapter data with substantial content
chapters = {
    1: {
        "title": "Space Habitat Overview",
        "sections": [
            ("History of Space Habitation", "Human space habitation began in 1971 with the Soviet Salyut 1 space station. Since then, through Skylab, Mir, and the current International Space Station (ISS), space habitat technology has evolved for over 50 years. The ISS, operational since 2000, has proven the viability of long-term space habitation.\n\nEarly space stations were designed for short stays, but with technological advancement and experience accumulation, duration has gradually increased. Today, astronauts live in space for 6 months to over a year, accumulating invaluable data on long-term space living.\n\nThe ISS cost approximately $150 billion to build and represents humanity's most expensive structure. It orbits at ~400 km altitude, completing one orbit every 90 minutes. The station has hosted over 250 individuals from 20+ countries, demonstrating international cooperation in space."),
            ("Why We Need to Live in Space", "Humanity needs to become a multiplanetary species for several critical reasons:\n\n**Planetary Backup:** Earth faces existential risks including asteroid impacts, nuclear war, climate change, and pandemics. Establishing self-sufficient settlements on other worlds serves as insurance for human survival. Prominent scientists including Stephen Hawking argued that becoming multiplanetary is essential for long-term survival.\n\n**Resource Access:** Earth's resources are finite and depleting rapidly with population growth and economic development. Asteroids and the Moon contain abundant rare metals, helium-3, water, and other resources scarce on Earth. Space resource mining is projected to create a multi-trillion dollar industry.\n\n**Scientific Research:** Space provides unique experimental conditions impossible on Earth: microgravity, high vacuum, extreme temperatures. Drug development, new materials research, and biological experiments can be more effectively conducted in space. Research on the ISS has already yielded important advances in medicine, materials science, and physics.\n\n**Technological Innovation:** Technologies developed for space habitation improve life on Earth. Water recycling, air purification, solar power, telemedicine, and recycling technologies originally from space programs are now in everyday use.\n\n**Long-term Future:** Eventually, the Sun will brighten and heat up, making Earth uninhabitable in about 1 billion years. Space habitat technology provides the foundation for humanity to migrate beyond the solar system to other star systems."),
        ],
        "stats": [
            ("1971", "First Space Station (Salyut 1)"),
            ("2000", "ISS Continuous Habitation Began"),
            ("437 days", "Longest Continuous Space Stay"),
            ("$150B", "ISS Total Construction Cost"),
        ]
    },
    2: {
        "title": "Habitat Design Principles",
        "sections": [
            ("Space Environment Challenges", "Space is extremely hostile to human life. Without Earth's magnetic field and atmosphere protection, space habitats must protect occupants from multiple lethal threats.\n\nRadiation levels in low Earth orbit expose astronauts to 50-200 mSv annually, over 100 times ground level. Travel to Mars and surface stays increase exposure further. Temperature extremes range from -270°C in shadow to +120°C in sunlight. Vacuum pressure is 10^-15 atmospheres. Micrometeorites travel at 10-70 km/s - even 1cm debris has energy equivalent to a hand grenade."),
            ("Radiation Shielding", "Space radiation is one of the most serious threats to space habitation. Even in LEO, astronauts receive 50-200 mSv annually.\n\nTypes of radiation:\n- Galactic Cosmic Rays (GCR): High-energy protons and heavy ions from outside the galaxy\n- Solar Particle Events (SPE): Massive proton bursts from solar flares\n- Trapped radiation: Van Allen belt electrons and protons\n- Secondary radiation: Neutrons and gamma rays from shielding interactions\n\nEffective shielding strategies include passive shielding (aluminum, polyethylene, water, regolith), active shielding (electromagnetic fields), storm shelters for solar events, and using local materials for low-cost protection."),
        ],
        "stats": [
            ("-270°C", "Space Temperature (Shadow)"),
            ("+120°C", "Space Temperature (Sunlight)"),
            ("10^-15 atm", "Space Vacuum"),
            ("400 mSv/year", "Space Radiation Exposure"),
        ]
    },
    3: {
        "title": "Artificial Gravity Systems",
        "sections": [
            ("Health Effects of Microgravity", "Long-term microgravity exposure severely affects human health. ISS research has confirmed serious health issues:\n\n- Bone Density Loss: 1-2% per month\n- Muscle Mass Reduction: 20% in 6 months\n- Cardiovascular Decline: 10-20%\n- Vision Problems: 30% of astronauts\n\nBone loss in 6 months equals 10 years of osteoporosis in elderly. Despite 2+ hours daily exercise, losses still occur. Artificial gravity is the only fundamental solution."),
            ("Centrifugal Force for Artificial Gravity", "True gravity cannot be created, but rotation produces centrifugal force mimicking gravity effects. Based on Newton's First Law and centripetal acceleration.\n\nFormula: a = ω²r = v²/r\nWhere:\n- a = centrifugal acceleration (m/s²)\n- ω = angular velocity (rad/s)\n- r = radius (m)\n- v = linear velocity (m/s)\n\nTo create Earth gravity (1g = 9.81 m/s²):\n- Radius 100m: 3 rpm\n- Radius 500m: 1.3 rpm  \n- Radius 1,000m: 1 rpm"),
        ],
        "stats": [
            ("1-2%", "Monthly Bone Density Loss"),
            ("20%", "Muscle Mass Reduction (6 months)"),
            ("10-20%", "Cardiovascular Function Decline"),
            ("30%", "Vision Problems Incidence"),
        ]
    },
    4: {
        "title": "Closed Ecosystems",
        "sections": [
            ("Need for Closed Ecosystems", "If space habitats depend on Earth resupply, long-term habitation is impossible due to cost and logistics. Launching 1kg to ISS costs ~$10,000. Closed ecosystems recycle resources internally, achieving self-sufficiency.\n\nA closed ecological system recycles air, water, and nutrients without external input. Complete closure is extremely difficult - ISS currently recycles 93% of water."),
            ("Bioregenerative Life Support Systems", "Biological life support systems use plants, microorganisms, and potentially animals to regenerate air, water, and food.\n\nCore cycles:\n- Oxygen Cycle: CO₂ (human breath) → Plant photosynthesis → O₂\n- Water Cycle: Wastewater → Purification → Clean water\n- Nutrient Cycle: Waste → Microbial decomposition → Plant nutrients\n- Food Cycle: Nutrients + water + CO₂ → Plant growth → Edible crops\n\nPlants provide multiple benefits: oxygen production (20-30m² per person), food production, water cycling through transpiration, and psychological benefits."),
        ],
        "stats": [
            ("$10,000", "Cost per 1kg Launch (ISS)"),
            ("90%+", "Target Recycling Rate"),
            ("0-2 years", "Earth Resupply Target"),
            ("100%", "Mars Self-Sufficiency Goal"),
        ]
    },
    5: {
        "title": "Construction Technology",
        "sections": [
            ("3D Printing Construction", "Space 3D printing can dramatically reduce material transport from Earth. Using lunar or Martian regolith as raw material, structures can be directly printed. NASA's Contour Crafting project can print a house in 24 hours.\n\nAdvantages: 90% launch weight reduction, faster construction, complex geometries possible, minimal waste. Regolith-based concrete can be strengthened by solar sintering and mixed with glass fibers. ESA's D-Shape technology is optimized for lunar base construction."),
            ("Inflatable Modules", "Inflatable habitats fold compactly for launch and deploy in space. Bigelow Aerospace's BEAM module successfully installed on ISS and operated 4+ years.\n\nAdvantages: 75% volume reduction at launch, large interior space, excellent radiation shielding (multi-layer Kevlar), micrometeorite protection. Made of multiple layers of Vectran, Kevlar, Nomex fabrics. NASA's TransHab concept is 8.2m diameter accommodating up to 6 people."),
        ],
        "stats": [
            ("90%", "Launch Weight Reduction"),
            ("24 hours", "House Printing Time"),
            ("75%", "Volume Reduction at Launch"),
            ("4+ years", "BEAM Operation Duration"),
        ]
    },
    6: {
        "title": "Energy & Resource Management",
        "sections": [
            ("Solar Power Systems", "Solar power is the most abundant and reliable energy source in space. In Earth orbit, 1,366 W/m² solar radiation is received (5-10x Earth surface). ISS has eight 240m² solar arrays producing 84kW.\n\nLatest 3rd generation solar cells achieve 40% efficiency. Using Concentrated Solar Power (CSP) further improves performance. Challenges include day/night cycles (every 90min in LEO), panel orientation, micrometeorite damage, and radiation degradation. On Mars, solar radiation is 43% of Earth and reduced further by dust storms requiring backup power."),
            ("Nuclear Energy", "Nuclear power is essential where solar is insufficient (outer planets, polar regions, dust storms). Kilopower project develops 1-10kWe small reactors operating via thermoelectric conversion without cooling fans.\n\nAdvantages: 24-hour stable power, high energy density (1kg nuclear fuel = millions of tons coal), 10+ year lifespan, compact and lightweight. Safety ensured by passive cooling, double containment, remote deployment (100m+ from habitat), automatic shutdown. Radioactive waste is minimized and stored in shielded containers."),
        ],
        "stats": [
            ("1,366 W/m²", "Earth Orbit Solar Radiation"),
            ("590 W/m²", "Mars Orbit Solar Radiation"),
            ("30-40%", "Latest Solar Cell Efficiency"),
            ("10+ kW", "Kilopower Reactor Output"),
        ]
    },
    7: {
        "title": "Psychological & Social Factors",
        "sections": [
            ("Psychological Impact of Isolation", "Long-term space missions mean extreme isolation and confinement. Mars missions place crew 250 million km from Earth with up to 22-minute communication delay.\n\nPsychological challenges: confined space (ISS is football field size for 6 crew), lack of privacy (personal space is phone booth size), living with same people 24/7, separation from family and friends, uncertainty of Earth return (2-3 years for Mars).\n\nResearch shows 3-phase adaptation: 1) Initial excitement (1-2 months), 2) Adaptation crisis (3-6 months with increased depression, anxiety, conflict), 3) Stabilization (thereafter). Common issues include depression, anxiety, insomnia, interpersonal conflict, and in extremes, psychotic symptoms."),
            ("Crew Selection and Team Composition", "Successful space missions depend more on psychological compatibility than technical skills.\n\nSelection criteria: mental stability, stress management ability, teamwork and cooperation, problem-solving ability, adaptability and flexibility, cultural sensitivity.\n\nTeam composition considers: gender balance (mixed teams more stable), age diversity (20s-50s mix), cultural diversity (mutual understanding), skill distribution (multi-functional capabilities). Russia's MARS-500 experiment (520 days isolation) proved team dynamics importance."),
        ],
        "stats": [
            ("250M km", "Distance from Earth (Mars)"),
            ("22 min", "Communication Delay"),
            ("3-6 months", "Adaptation Crisis Period"),
            ("520 days", "MARS-500 Experiment"),
        ]
    },
    8: {
        "title": "Planetary Base Planning",
        "sections": [
            ("Lunar Base Plan", "The Moon will be humanity's first space outpost. NASA's Artemis program plans to build a base at the lunar south pole by mid-2020s.\n\nCandidate locations include Shackleton Crater, Nobile Crater, and Connecting Ridge - areas where permanently shadowed regions (water ice) and permanently lit regions (solar power) are close together.\n\nInitial infrastructure: landing pad (flat, obstacle-free zone), power systems (solar panels, reactor), habitat modules (4-6 person capacity), ISRU facilities (water mining, oxygen production), communications antennas.\n\nConstruction proceeds in 3 phases: 1) Unmanned cargo landings (2-3 missions), 2) Robotic assembly and commissioning, 3) Human occupation and expansion. Scientific goals include astronomy (radio telescopes), geology research, space technology validation, Mars mission preparation."),
            ("Mars Base Design", "Mars has potential to become humanity's second home. Base site selection criteria: water access (subsurface ice), latitude (equatorial, mild climate), elevation (lower = denser atmosphere, easier landing), scientific value (past water evidence), sunlight (solar power).\n\nCandidate regions: Jezero Crater (currently explored by Perseverance rover), Arcadia Planitia (shallow subsurface ice), Deuteronilus Mensae (glacial region).\n\nInitial base starts with 6-12 people and gradually expands. Modules are underground or covered with regolith for radiation shielding. Mars atmosphere (95% CO₂) enables oxygen and fuel production - MOXIE technology validated."),
        ],
        "stats": [
            ("600M tons", "Lunar South Pole Water Ice"),
            ("89%", "Shackleton Rim Sunlight %"),
            ("-50°C", "Polar Average Temperature"),
            ("2028", "Artemis Base Target Year"),
        ]
    },
}

# HTML template
def create_chapter_html(num, data):
    prev_ch = f"chapter-{num-1:02d}.html" if num > 1 else "index.html"
    next_ch = f"chapter-{num+1:02d}.html" if num < 8 else "index.html"
    next_text = "Next Chapter →" if num < 8 else "Back to Contents →"
    
    # Build sections HTML
    sections_html = ""
    for title, content in data["sections"]:
        sections_html += f'''
        <h2>{title}</h2>
        <p>{content}</p>
'''
    
    # Build stats HTML if present
    stats_html = ""
    if "stats" in data:
        stats_html = '''
        <div class="stats-grid">
'''
        for value, label in data["stats"]:
            stats_html += f'''            <div class="stat-card">
                <div class="stat-number">{value}</div>
                <div class="stat-label">{label}</div>
            </div>
'''
        stats_html += '''        </div>
'''
    
    html = f'''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Chapter {num}: {data["title"]} - WIA Space Habitat Standard</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #e2e8f0;
            line-height: 1.8;
            min-height: 100vh;
        }}
        .container {{ max-width: 900px; margin: 0 auto; padding: 2rem; }}
        header {{ padding: 2rem 0; border-bottom: 2px solid #e94560; margin-bottom: 2rem; }}
        .chapter-number {{ font-size: 1rem; color: #e94560; font-weight: 600; margin-bottom: 0.5rem; }}
        h1 {{ font-size: 2.5rem; color: #f8fafc; margin-bottom: 1rem; }}
        h2 {{ color: #e94560; font-size: 1.8rem; margin: 2.5rem 0 1rem 0; padding-bottom: 0.5rem; border-bottom: 2px solid rgba(233, 69, 96, 0.3); }}
        h3 {{ color: #e94560; font-size: 1.3rem; margin: 2rem 0 1rem 0; }}
        p {{ margin-bottom: 1.2rem; color: #cbd5e1; }}
        .chapter-nav {{ background: #16213e; padding: 1rem; border-radius: 8px; margin-bottom: 2rem; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 1rem; }}
        .nav-button {{ padding: 0.75rem 1.5rem; background: #0f3460; color: #e94560; text-decoration: none; border-radius: 6px; transition: all 0.3s ease; border: 1px solid #e94560; }}
        .nav-button:hover {{ background: #e94560; color: #1a1a2e; transform: translateY(-2px); }}
        .highlight-box {{ background: rgba(233, 69, 96, 0.1); border-left: 4px solid #e94560; padding: 1.5rem; margin: 2rem 0; border-radius: 0 8px 8px 0; }}
        .stats-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 1rem; margin: 2rem 0; }}
        .stat-card {{ background: #16213e; padding: 1.5rem; border-radius: 8px; border: 1px solid #0f3460; text-align: center; }}
        .stat-number {{ font-size: 2rem; font-weight: bold; color: #e94560; margin-bottom: 0.5rem; }}
        .stat-label {{ color: #94a3b8; font-size: 0.9rem; }}
        ul, ol {{ margin: 1rem 0 1rem 2rem; color: #cbd5e1; }}
        li {{ margin-bottom: 0.5rem; }}
        footer {{ margin-top: 4rem; padding: 2rem 0; text-align: center; border-top: 2px solid #0f3460; color: #64748b; }}
        .footer-tagline {{ font-size: 1.1rem; color: #e94560; margin-bottom: 0.5rem; }}
        @media (max-width: 768px) {{ h1 {{ font-size: 2rem; }} .container {{ padding: 1rem; }} }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="chapter-number">Chapter {num}</div>
            <h1>{data["title"]}</h1>
        </header>

        <div class="chapter-nav">
            <a href="{prev_ch}" class="nav-button">← Previous Chapter</a>
            <a href="index.html" class="nav-button">Contents</a>
            <a href="{next_ch}" class="nav-button">{next_text}</a>
        </div>

        <div class="highlight-box">
            <strong>Chapter Overview:</strong> This chapter explores {data["title"].lower()}, covering essential principles and technologies for successful space habitat operations. Each element is interconnected, requiring an integrated approach.
        </div>
{stats_html}{sections_html}
        <h2>Conclusion</h2>
        <p>The principles and technologies covered in this chapter on {data["title"]} are essential for successful space habitat operations. Each element is interconnected and requires an integrated approach. As space technology advances and experience accumulates, these systems will become more sophisticated and efficient.</p>

        <p>Space habitation is no longer science fiction but becoming reality. Following the philosophy of 弘益人間 (Benefit All Humanity), we are opening a new era that will benefit all of humanity.</p>

        <div class="chapter-nav" style="margin-top: 3rem;">
            <a href="{prev_ch}" class="nav-button">← Previous Chapter</a>
            <a href="index.html" class="nav-button">Contents</a>
            <a href="{next_ch}" class="nav-button">{next_text}</a>
        </div>

        <footer>
            <div class="footer-tagline">弘益人間 (Benefit All Humanity)</div>
            <p>WIA Space Habitat Standard - Building New Homes for the Space Age</p>
            <p style="margin-top: 0.5rem; font-size: 0.9rem;">&copy; 2025 WIA Standards. All rights reserved.</p>
        </footer>
    </div>
</body>
</html>
'''
    return html

# Generate all 8 English chapters
for num in range(1, 9):
    html = create_chapter_html(num, chapters[num])
    filename = f"chapter-{num:02d}.html"
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"✓ Created {filename} ({len(html.splitlines())} lines)")

print("\nAll English chapters created successfully!")
PYTHON_EN

