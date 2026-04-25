#!/usr/bin/env python3
# Generate remaining chapters 02-08 for WIA-DATA_VISUALIZATION

css = ":root{--primary:#8B5CF6;--bg:#0f172a;--bg-card:#1e293b;--text:#f8fafc;--text-secondary:#cbd5e1;--border:#334155;--accent:#3b82f6;--code-bg:#1e1e1e;--success:#10b981;}*{margin:0;padding:0;box-sizing:border-box;}body{font-family:'Malgun Gothic',sans-serif;background:var(--bg);color:var(--text);line-height:1.8;}.container{max-width:1000px;margin:0 auto;padding:40px 20px;}header{background:linear-gradient(135deg,var(--bg-card) 0%,#2d1f4a 100%);padding:40px;border-radius:15px;margin-bottom:40px;border-left:5px solid var(--primary);}h1{color:var(--primary);font-size:2.5em;margin-bottom:15px;}h2{color:var(--primary);font-size:2em;margin:50px 0 25px 0;padding-bottom:10px;border-bottom:2px solid var(--primary);}h3{color:var(--accent);font-size:1.5em;margin:35px 0 20px 0;}p{margin:20px 0;font-size:1.1em;line-height:1.9;}table{width:100%;border-collapse:collapse;margin:30px 0;background:var(--bg-card);border-radius:10px;overflow:hidden;}th{background:var(--primary);color:white;padding:15px;text-align:left;}td{padding:15px;border-bottom:1px solid var(--border);}tr:hover{background:rgba(139,92,246,0.1);}ul,ol{margin:20px 0;padding-left:30px;}li{margin:10px 0;font-size:1.1em;line-height:1.8;}.code-block{background:var(--code-bg);border:1px solid var(--border);border-radius:8px;padding:20px;margin:25px 0;overflow-x:auto;font-family:Consolas,monospace;font-size:0.95em;}.info-box{background:rgba(59,130,246,0.1);border-left:4px solid var(--accent);padding:25px;margin:30px 0;border-radius:8px;}.key-takeaways{background:linear-gradient(135deg,rgba(139,92,246,0.2) 0%,rgba(59,130,246,0.2) 100%);padding:30px;border-radius:15px;margin:40px 0;}.review-questions{background:var(--bg-card);padding:30px;border-radius:15px;margin:40px 0;}.nav-buttons{display:flex;justify-content:space-between;margin-top:50px;gap:20px;}.btn{padding:15px 30px;border-radius:8px;text-decoration:none;font-weight:600;transition:all 0.3s;flex:1;text-align:center;}.btn-primary{background:var(--primary);color:white;}.btn-secondary{background:var(--bg-card);color:var(--primary);border:2px solid var(--primary);}footer{margin-top:60px;padding-top:30px;border-top:2px solid var(--border);text-align:center;color:var(--text-secondary);}"

chapters = [
    (2, "차트 유형 및 선택 가이드", "Bar Chart, Line Chart, Pie Chart, Scatter Plot, Heatmap, Network Graph의 특성과 활용"),
    (3, "시각화 라이브러리", "D3.js, Chart.js, Plotly, ECharts 비교 및 구현 가이드"),
    (4, "대시보드 설계", "레이아웃 원칙, 색상 이론, 타이포그래피, UX 패턴"),
    (5, "인터랙티브 시각화", "애니메이션, 필터링, 드릴다운, 툴팁 구현"),
    (6, "접근성 및 포용적 디자인", "WCAG 준수, 색맹 대응, 스크린 리더 지원"),
    (7, "실시간 데이터 시각화", "WebSocket, Server-Sent Events, 스트리밍 차트"),
    (8, "미래 트렌드와 혁신", "AI 생성 시각화, VR/AR, 자연어 쿼리, 자동화")
]

print(f"Generating {len(chapters)} chapters...")
for num, title, desc in chapters:
    print(f"  - Chapter {num:02d}: {title}")

print("\nReady to generate files...")
