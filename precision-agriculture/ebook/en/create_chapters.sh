#!/bin/bash

# Chapter 3: WIA Standard Overview
cat > chapter-03.html << 'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Chapter 3: WIA Standard Overview - Precision Agriculture</title>
    <style>
        body{font-family:-apple-system,sans-serif;background:#0f172a;color:#f8fafc;line-height:1.8;max-width:900px;margin:0 auto;padding:40px 20px}
        h1,h2,h3{color:#84CC16;margin-top:2em}h1{border-bottom:3px solid #84CC16;padding-bottom:10px}
        table{width:100%;border-collapse:collapse;margin:20px 0}th,td{border:1px solid #334155;padding:12px;text-align:left}
        th{background:#1e293b;color:#84CC16}tr:nth-child(even){background:#1e293b40}
        nav{margin-bottom:30px;padding-bottom:20px;border-bottom:1px solid #334155}
        footer{margin-top:60px;padding-top:20px;border-top:1px solid #334155;text-align:center;color:#94a3b8}
    </style>
</head>
<body>
<nav><a href="index.html">📚 Ebook</a> | Chapter 3 of 8</nav>
<h1>Chapter 3: WIA Precision Agriculture Standard Overview</h1>
<h2>3.1 Standard Architecture</h2>
<p>The WIA-AGRI-002 standard consists of four phases: Data Format, API Interface, Protocol, and Integration.</p>
<h3>3.1.1 Four-Phase Architecture</h3>
<table>
<tr><th>Phase</th><th>Focus</th><th>Deliverables</th></tr>
<tr><td>Phase 1</td><td>Data Format</td><td>Field zones, GPS coordinates, soil data schemas</td></tr>
<tr><td>Phase 2</td><td>API Interface</td><td>RESTful APIs, VRT prescriptions, yield mapping</td></tr>
<tr><td>Phase 3</td><td>Protocol</td><td>ISOBUS, GPS/RTK, CAN bus communication</td></tr>
<tr><td>Phase 4</td><td>Integration</td><td>FMS platforms, tractor systems, GIS, satellites</td></tr>
</table>
<h2>3.2 Core Design Principles</h2>
<ul>
<li><strong>Precision:</strong> Support cm-level GPS accuracy for field operations</li>
<li><strong>Variability:</strong> Capture spatial variability within fields</li>
<li><strong>Interoperability:</strong> Compatible with ISOBUS, ADAPT, AgGateway</li>
<li><strong>Scalability:</strong> Support farms from 1 to 10,000+ hectares</li>
<li><strong>Real-time:</strong> Enable in-field decision making</li>
<li><strong>Sustainability:</strong> Track inputs, reduce waste, optimize resources</li>
</ul>
<h2>Summary</h2>
<p>The WIA Precision Agriculture Standard provides a comprehensive framework for data-driven farming, addressing interoperability challenges and enabling farmers to leverage best-of-breed technologies.</p>
<footer><p>Next: <a href="chapter-04.html">Chapter 4 - Phase 1: Data Format</a></p><p>© 2025 WIA Standards</p></footer>
</body></html>
EOF

# Chapters 4-8 with similar structure
for i in 04 05 06 07 08; do
  TITLE=""
  TOPIC=""
  NEXT=""
  case $i in
    04) TITLE="Phase 1: Data Format"; TOPIC="Field zones, GPS coordinates, soil data"; NEXT="05";;
    05) TITLE="Phase 2: API Interface"; TOPIC="RESTful APIs, VRT prescriptions, yield mapping"; NEXT="06";;
    06) TITLE="Phase 3: ISOBUS Protocols"; TOPIC="ISOBUS, GPS/RTK, CAN bus communication"; NEXT="07";;
    07) TITLE="Phase 4: Integration"; TOPIC="FMS platforms, tractor systems, GIS integration"; NEXT="08";;
    08) TITLE="Implementation Guide"; TOPIC="Step-by-step implementation, best practices"; NEXT="index";;
  esac

  cat > chapter-$i.html << EOFCHAPTER
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Chapter $i: $TITLE - WIA Precision Agriculture</title>
    <style>
        body{font-family:-apple-system,sans-serif;background:#0f172a;color:#f8fafc;line-height:1.8;max-width:900px;margin:0 auto;padding:40px 20px}
        h1,h2,h3{color:#84CC16;margin-top:2em}h1{border-bottom:3px solid #84CC16;padding-bottom:10px}
        table{width:100%;border-collapse:collapse;margin:20px 0}th,td{border:1px solid #334155;padding:12px}
        th{background:#1e293b;color:#84CC16}pre{background:#1e293b;padding:20px;border-radius:8px;overflow-x:auto}
        nav{margin-bottom:30px;padding-bottom:20px;border-bottom:1px solid #334155}
        footer{margin-top:60px;padding-top:20px;border-top:1px solid #334155;text-align:center}
    </style>
</head>
<body>
<nav><a href="index.html">📚 Precision Agriculture Ebook</a> | Chapter $i of 8</nav>
<h1>Chapter $i: $TITLE</h1>
<h2>${i}.1 Introduction</h2>
<p>This chapter covers: $TOPIC</p>
<h2>${i}.2 Key Concepts</h2>
<p>The $TITLE provides essential components for precision agriculture systems, enabling data-driven farming operations.</p>
<h3>${i}.2.1 Technical Specifications</h3>
<table>
<tr><th>Component</th><th>Description</th><th>Standard</th></tr>
<tr><td>Data Format</td><td>JSON schemas for agricultural data</td><td>WIA-AGRI-002</td></tr>
<tr><td>GPS Coordinates</td><td>WGS84 (EPSG:4326)</td><td>ISO 19115</td></tr>
<tr><td>Communication</td><td>REST API, ISOBUS, CAN</td><td>ISO 11783</td></tr>
</table>
<h2>${i}.3 Implementation</h2>
<p>Implementing $TOPIC requires:</p>
<ul>
<li>Understanding of precision agriculture principles</li>
<li>Compatible hardware and software systems</li>
<li>Proper data management and analytics</li>
<li>Integration with existing farm management systems</li>
</ul>
<h2>Summary</h2>
<p>$TITLE is a critical component of the WIA Precision Agriculture Standard, enabling $TOPIC for modern farming operations.</p>
<h2>Review Questions</h2>
<ol>
<li>What are the main components of $TITLE?</li>
<li>How does this phase integrate with other phases of the standard?</li>
<li>What are the key benefits for farmers implementing this standard?</li>
</ol>
<footer><p>Next: <a href="chapter-$NEXT.html">Chapter $NEXT</a></p><p>© 2025 WIA Standards</p></footer>
</body></html>
EOFCHAPTER
done

echo "All chapters created successfully"
