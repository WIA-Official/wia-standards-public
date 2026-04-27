#!/bin/bash

# Chapter 2
cat > chapter-02.html << 'EOF'
<!DOCTYPE html>
<html lang="ko">
<head><meta charset="UTF-8"><title>2장: 시계열 데이터 모델링</title></head>
<body>
<p><a href="index.html">⏱️ WIA-DATA-014</a> | 2장 / 8장</p><hr>
<h1>2장: 시계열 데이터 모델링</h1>
<blockquote><p><strong>홍익인간</strong> - 효율적인 데이터 모델링으로 모두에게 이로운 시스템을 구축합니다.</p></blockquote><hr>

<h2>2.1 시계열 데이터 포인트 구조</h2>
<p>시계열 데이터베이스에서 각 데이터 포인트는 다음 요소로 구성됩니다:</p>
<h3>2.1.1 핵심 구성 요소</h3>
<table border="1" cellpadding="10"><thead><tr><th>구성 요소</th><th>설명</th><th>필수 여부</th><th>예시</th></tr></thead>
<tbody>
<tr><td><strong>타임스탬프</strong></td><td>측정 시점</td><td>필수</td><td>2025-12-26T10:30:00Z</td></tr>
<tr><td><strong>측정값</strong></td><td>무엇을 측정하는가</td><td>필수</td><td>temperature, cpu_usage</td></tr>
<tr><td><strong>필드</strong></td><td>실제 측정 데이터</td><td>필수</td><td>value=23.5</td></tr>
<tr><td><strong>태그</strong></td><td>인덱싱된 메타데이터</td><td>선택</td><td>host=server1, region=us-east</td></tr>
</tbody></table>

<h3>2.1.2 InfluxDB 라인 프로토콜</h3>
<p>InfluxDB는 다음과 같은 라인 프로토콜을 사용합니다:</p>
<pre><code># 기본 구조
measurement[,tag_key=tag_value,...] field_key=field_value[,field_key=field_value] [timestamp]

# 예시
temperature,location=office,sensor=DHT22 value=23.5,humidity=65 1735208400000000000
cpu,host=server1,region=us-east usage=75.2,cores=8 1735208400000000000
network,interface=eth0 bytes_in=1024000,bytes_out=512000 1735208400000000000
</code></pre>

<h3>2.1.3 TimescaleDB 스키마</h3>
<p>TimescaleDB는 PostgreSQL 확장이므로 표준 SQL DDL을 사용합니다:</p>
<pre><code>CREATE TABLE sensor_data (
    time        TIMESTAMPTZ NOT NULL,
    sensor_id   TEXT NOT NULL,
    location    TEXT,
    temperature DOUBLE PRECISION,
    humidity    DOUBLE PRECISION
);

-- 하이퍼테이블로 변환
SELECT create_hypertable('sensor_data', 'time');

-- 인덱스 생성
CREATE INDEX idx_sensor_location ON sensor_data (sensor_id, location, time DESC);
</code></pre>

<h2>2.2 태그 vs 필드 설계</h2>
<h3>2.2.1 태그 (Tags)</h3>
<p><strong>태그</strong>는 인덱싱되는 메타데이터로, WHERE 절과 GROUP BY에 사용됩니다:</p>
<ul>
<li><strong>특징:</strong> 문자열만 가능, 인덱싱됨, 카디널리티 제한</li>
<li><strong>용도:</strong> 필터링, 그룹화, 집계</li>
<li><strong>예시:</strong> host, region, datacenter, environment</li>
<li><strong>주의사항:</strong> 고유 값이 많으면 성능 저하 (예: user_id 사용 지양)</li>
</ul>

<h3>2.2.2 필드 (Fields)</h3>
<p><strong>필드</strong>는 실제 측정값으로, 인덱싱되지 않습니다:</p>
<ul>
<li><strong>특징:</strong> 숫자, 문자열, 불린 등 다양한 타입</li>
<li><strong>용도:</strong> 실제 측정 데이터 저장</li>
<li><strong>예시:</strong> temperature, cpu_usage, response_time</li>
<li><strong>주의사항:</strong> WHERE 절에 사용 시 성능 저하</li>
</ul>

<h3>2.2.3 설계 지침</h3>
<table border="1" cellpadding="10"><thead><tr><th>시나리오</th><th>태그로 사용</th><th>필드로 사용</th></tr></thead>
<tbody>
<tr><td>서버 이름</td><td>✅ host=server1</td><td>❌</td></tr>
<tr><td>지역</td><td>✅ region=us-east</td><td>❌</td></tr>
<tr><td>CPU 사용률</td><td>❌</td><td>✅ cpu_usage=75.2</td></tr>
<tr><td>온도</td><td>❌</td><td>✅ temperature=23.5</td></tr>
<tr><td>사용자 ID</td><td>⚠️ 카디널리티 주의</td><td>✅</td></tr>
</tbody></table>

<h2>2.3 측정값 네이밍 컨벤션</h2>
<h3>2.3.1 네이밍 규칙</h3>
<ul>
<li><strong>소문자 사용:</strong> temperature (O), Temperature (X)</li>
<li><strong>언더스코어 구분:</strong> cpu_usage, network_traffic</li>
<li><strong>명확한 이름:</strong> temp (X), temperature (O)</li>
<li><strong>단위 포함 고려:</strong> response_time_ms, memory_bytes</li>
<li><strong>네임스페이스:</strong> system.cpu.usage, app.http.requests</li>
</ul>

<h3>2.3.2 조직 패턴</h3>
<pre><code># 계층적 구조
system.cpu.usage
system.memory.used
system.disk.io_read
system.network.bytes_in

# 도메인별 구분
iot.sensor.temperature
iot.sensor.humidity
web.http.requests
web.http.response_time
</code></pre>

<h2>2.4 스키마 설계 패턴</h2>
<h3>2.4.1 단일 측정값 패턴</h3>
<p>각 메트릭을 별도의 측정값으로 저장:</p>
<pre><code>cpu,host=server1 usage=75.2 1735208400
memory,host=server1 used=4096 1735208400
disk,host=server1 used=512000 1735208400
</code></pre>
<p><strong>장점:</strong> 쿼리 단순, 명확한 의미, 독립적 보관 정책<br>
<strong>단점:</strong> 쓰기 작업 증가, 조인 필요 시 복잡</p>

<h3>2.4.2 다중 필드 패턴</h3>
<p>관련 메트릭을 하나의 측정값에 저장:</p>
<pre><code>system,host=server1 cpu=75.2,memory=4096,disk=512000 1735208400
</code></pre>
<p><strong>장점:</strong> 쓰기 효율, 원자적 업데이트, 쿼리 편의<br>
<strong>단점:</strong> 보관 정책 일괄 적용, 측정값 크기 증가</p>

<h3>2.4.3 시계열 ID 패턴</h3>
<p>태그 조합으로 고유한 시계열 식별:</p>
<pre><code># 각 조합이 별도의 시계열 생성
temperature,sensor=A,location=room1 value=23.5
temperature,sensor=A,location=room2 value=24.1
temperature,sensor=B,location=room1 value=23.8
</code></pre>

<h2>2.5 카디널리티 관리</h2>
<h3>2.5.1 카디널리티란?</h3>
<p><strong>카디널리티(Cardinality)</strong>는 시계열의 고유 조합 수입니다:</p>
<pre><code>카디널리티 = 측정값 수 × 태그1 고유값 × 태그2 고유값 × ...

예시:
- 측정값: temperature (1개)
- sensor: A, B, C (3개)
- location: room1, room2 (2개)
- 총 카디널리티 = 1 × 3 × 2 = 6개 시계열
</code></pre>

<h3>2.5.2 높은 카디널리티 문제</h3>
<ul>
<li><strong>메모리 소비:</strong> 각 시계열마다 인덱스 항목 생성</li>
<li><strong>쿼리 성능:</strong> 검색할 시계열 수 증가</li>
<li><strong>쓰기 성능:</strong> 인덱스 업데이트 부하</li>
</ul>

<h3>2.5.3 카디널리티 최적화</h3>
<table border="1" cellpadding="10"><thead><tr><th>전략</th><th>설명</th><th>예시</th></tr></thead>
<tbody>
<tr><td><strong>태그 수 제한</strong></td><td>필수 태그만 사용</td><td>5개 이하 권장</td></tr>
<tr><td><strong>태그 값 제한</strong></td><td>고유 값 수 제한</td><td>환경별로 10,000개 이하</td></tr>
<tr><td><strong>동적 태그 회피</strong></td><td>UUID, 타임스탬프 태그 사용 금지</td><td>user_id를 태그 대신 필드로</td></tr>
<tr><td><strong>데이터 집계</strong></td><td>상위 레벨로 집계</td><td>개별 사용자 대신 지역별 집계</td></tr>
</tbody></table>

<h2>2.6 데이터 타입 선택</h2>
<h3>2.6.1 수치형 데이터</h3>
<table border="1" cellpadding="10"><thead><tr><th>타입</th><th>범위</th><th>용도</th><th>예시</th></tr></thead>
<tbody>
<tr><td><strong>Integer</strong></td><td>-2^63 ~ 2^63-1</td><td>카운터, 인덱스</td><td>request_count=1000</td></tr>
<tr><td><strong>Float</strong></td><td>IEEE-754</td><td>측정값, 비율</td><td>temperature=23.5</td></tr>
<tr><td><strong>Unsigned</strong></td><td>0 ~ 2^64-1</td><td>바이트 수, 시간</td><td>bytes_sent=1024000</td></tr>
</tbody></table>

<h3>2.6.2 문자열 데이터</h3>
<ul>
<li><strong>태그:</strong> 짧고 반복적인 문자열 (예: "production", "us-east")</li>
<li><strong>필드:</strong> 긴 텍스트, 로그 메시지</li>
<li><strong>주의:</strong> 문자열 필드는 집계 불가능</li>
</ul>

<h3>2.6.3 불린 데이터</h3>
<pre><code>system,host=server1 healthy=true,disk_full=false 1735208400</code></pre>

<h2>2.7 시간 해상도 설계</h2>
<h3>2.7.1 타임스탬프 정밀도</h3>
<table border="1" cellpadding="10"><thead><tr><th>정밀도</th><th>사용 사례</th><th>저장 크기</th></tr></thead>
<tbody>
<tr><td><strong>초 (s)</strong></td><td>일반 모니터링</td><td>4-8 bytes</td></tr>
<tr><td><strong>밀리초 (ms)</strong></td><td>애플리케이션 성능</td><td>8 bytes</td></tr>
<tr><td><strong>마이크로초 (μs)</strong></td><td>고빈도 트레이딩</td><td>8 bytes</td></tr>
<tr><td><strong>나노초 (ns)</strong></td><td>하드웨어 레벨 측정</td><td>8 bytes</td></tr>
</tbody></table>

<h3>2.7.2 수집 간격 설계</h3>
<ul>
<li><strong>실시간 (< 1초):</strong> 고빈도 트레이딩, 산업 제어</li>
<li><strong>초 단위 (1-10초):</strong> 인프라 모니터링, IoT 센서</li>
<li><strong>분 단위 (1-15분):</strong> 비즈니스 메트릭, 에너지 소비</li>
<li><strong>시간 단위:</strong> 일일 집계, 장기 추세</li>
</ul>

<h2>2.8 보관 정책 설계</h2>
<h3>2.8.1 다단계 보관 전략</h3>
<table border="1" cellpadding="10"><thead><tr><th>단계</th><th>해상도</th><th>보관 기간</th><th>용도</th></tr></thead>
<tbody>
<tr><td><strong>원시 데이터</strong></td><td>1초</td><td>7일</td><td>실시간 모니터링</td></tr>
<tr><td><strong>5분 집계</strong></td><td>5분</td><td>90일</td><td>단기 분석</td></tr>
<tr><td><strong>1시간 집계</strong></td><td>1시간</td><td>1년</td><td>중기 추세</td></tr>
<tr><td><strong>1일 집계</strong></td><td>1일</td><td>5년</td><td>장기 보관</td></tr>
</tbody></table>

<h3>2.8.2 InfluxDB 보관 정책 예시</h3>
<pre><code>-- 원시 데이터: 7일 보관
CREATE RETENTION POLICY "raw_data" ON "metrics" 
  DURATION 7d REPLICATION 1 DEFAULT

-- 5분 집계: 90일 보관
CREATE RETENTION POLICY "downsampled_5m" ON "metrics"
  DURATION 90d REPLICATION 1

-- 연속 쿼리로 자동 다운샘플링
CREATE CONTINUOUS QUERY "cq_5m" ON "metrics"
BEGIN
  SELECT mean("value") AS "value"
  INTO "downsampled_5m"."cpu"
  FROM "raw_data"."cpu"
  GROUP BY time(5m), *
END
</code></pre>

<h2>2.9 인덱싱 전략</h2>
<h3>2.9.1 태그 인덱스</h3>
<p>모든 태그는 자동으로 인덱싱됩니다:</p>
<pre><code>-- 빠른 쿼리 (인덱스 사용)
SELECT * FROM cpu WHERE host='server1' AND region='us-east'

-- 느린 쿼리 (필드는 인덱싱 안 됨)
SELECT * FROM cpu WHERE value > 80
</code></pre>

<h3>2.9.2 TimescaleDB 추가 인덱스</h3>
<pre><code>-- 복합 인덱스
CREATE INDEX idx_sensor_time ON sensor_data (sensor_id, time DESC);

-- 부분 인덱스
CREATE INDEX idx_high_temp ON sensor_data (time)
  WHERE temperature > 30;

-- BRIN 인덱스 (시간 정렬 데이터에 효율적)
CREATE INDEX idx_time_brin ON sensor_data USING BRIN (time);
</code></pre>

<h2>요약</h2>
<p>이 장에서는 효율적인 시계열 데이터 모델링을 위한 핵심 개념을 다루었습니다. 태그와 필드의 올바른 사용, 카디널리티 관리, 적절한 데이터 타입 선택, 그리고 다단계 보관 정책이 성능과 비용 효율성의 핵심입니다.</p>
<p><strong>핵심 포인트:</strong></p>
<ul>
<li>태그는 필터링용, 필드는 측정값용</li>
<li>카디널리티를 낮게 유지하여 성능 최적화</li>
<li>다단계 보관 정책으로 스토리지 비용 절감</li>
<li>적절한 시간 해상도와 수집 간격 설계</li>
</ul>
<p><a href="chapter-03.html">다음: 3장 - 시계열 데이터베이스 →</a></p>
</body></html>
EOF

echo "Korean chapter 2 created (should be 15KB+)"
wc -c chapter-02.html

