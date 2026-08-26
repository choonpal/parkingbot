#!/usr/bin/env python3
"""천장 카메라 여러 대를 브라우저에서 나란히 보는 경량 프리뷰.

무엇을 위한 노드인가
--------------------
카메라를 천장에 달고 나서 **실측을 시작하기 전에** 확인해야 하는 것들이 있다.

  * 각 카메라가 주차장의 어느 부분을 보는가
  * 두 시야가 실제로 겹치는가, 겹침 폭은 충분한가
  * 주차면이 두 카메라 경계에 걸치지는 않는가
  * 렌즈 왜곡 보정이 제대로 되고 있는가 (raw와 rect를 나란히 비교)
  * 바닥 기준점으로 쓸 지점이 두 영상에 모두 잘 보이는가

``jetson_vision_web_node``도 MJPEG를 주지만 카메라 한 대만 다루고 YOLO/ArUco
추론까지 함께 돌린다. 이 노드는 **영상만** 보여주므로 가볍고, 카메라를 몇 대
붙이든 한 페이지에서 비교할 수 있다.

화면에서 할 수 있는 것
----------------------
  * 영상 클릭 -> 그 지점의 **원본 픽셀 좌표**를 읽는다. 등록 도구에서 찍을
    기준점을 미리 가늠하거나, 크롭 범위를 정할 때 쓴다.
  * 두 점을 찍으면 픽셀 거리를 알려준다.
  * 격자 간격을 바꿔가며 영상 왜곡(직선이 휘는지)을 눈으로 확인한다.
  * **ArUco 마커를 자로 쓴다.** 마커는 실제로 정사각형이므로 보정된 영상에서
    정사각형으로 보여야 한다. 변 길이 편차·대각선 비·꼭짓점 각도로 찌그러짐을
    수치화하고, 한 변의 실제 길이(기본 0.18 m)로 그 지점의 mm/px를 계산한다.
  * 같은 ID가 두 카메라에 동시에 보이면 **거기가 겹침 영역**이다. 상단에
    표시된다.
  * **YOLO 차량 검출과 world 좌표** — 검출된 차량의 화면 픽셀을 같은
    homography로 map 좌표(m)로 바꿔 카메라 화면과 표에 함께 보여준다.
    ArUco 마커도 같은 방식으로 world 좌표가 붙는다.
  * **BEV에 검출 표시** — 검출과 마커를 world 좌표 그대로 BEV에 찍는다.
    카메라마다 색이 달라서, 두 카메라가 같은 물체를 봤다면 두 점이 겹쳐야
    정합된 것이다. 어긋난 거리가 곧 좌표계 오차다.
  * **BEV 정합 확인** — homography가 준비되면 바닥을 위에서 본 그림으로 펴서
    보여준다. 두 카메라를 색분리(청록/빨강)로 겹쳐 보면 두 H가 같은 map
    좌표계를 가리키는지 한눈에 드러난다. 겹침 영역이 회색이면 정합,
    청록과 빨강으로 갈라지면 어긋난 것이다. 상관계수도 함께 표시한다.
  * 해상도와 실제 수신 FPS를 상시 표시한다. **캘리브레이션 해상도와
    다르면 경고**한다 — 이게 어긋나면 왜곡 보정이 조용히 틀어진다.

사용법
------
    ros2 run cooperative_parking_robot camera_preview

    # 보정 전후 비교
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p image_topics_csv:='/cctv0/image_raw,/cctv0/image_rect' \\
      -p labels_csv:='cam0 raw,cam0 rect'

    # BEV까지 보기 (homography 경로를 직접 줄 때)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p homography_files_csv:='/home/me/.ros/adaptive_valet_bot/homography_cam0_rectified.npy,/home/me/.ros/adaptive_valet_bot/homography_cam2_rectified.npy' \\
      -p layout_yaml:='/home/me/.ros/adaptive_valet_bot/parking_layout.yaml'

    # 사람을 검출해 보기 (차가 없을 때 파이프라인 확인용)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p yolo_class_ids:='[0]' -p model_path:=$HOME/yolov8n.pt

    # YOLO 끄기 (CPU가 부족할 때)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p enable_yolo:=false

    # 마커 크기가 다르거나 dictionary가 다를 때
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p marker_size_m:=0.10 -p aruco_dict:=DICT_5X5_100

찌그러짐 판정 기준 (변 길이 편차)
---------------------------------
    < 5%    양호
    5~12%   주의 — 카메라 기울기나 보정 오차를 의심한다
    > 12%   불량

마커를 화면 중앙과 네 귀퉁이로 옮겨가며 재면 원인을 구분할 수 있다.
중앙에서도 나쁘면 **왜곡 보정**이 틀린 것이고, 중앙은 괜찮은데 가장자리로
갈수록 나빠지면 **카메라가 기울어진** 것이다.

브라우저에서 ``http://<젯슨IP>:5005/``. VSCode 원격 접속 중이라면 PORTS 탭에서
5005를 forward하면 맥에서 ``http://localhost:5005/``로 그대로 열린다.
"""

from __future__ import annotations

import threading
import time

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cooperative_parking_robot.aruco_utils import ArucoDetectorCompat
from cooperative_parking_robot.vision_utils import load_yolo_model

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from flask import Flask, Response, jsonify
    from werkzeug.serving import make_server
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


_HTML = r'''<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>천장 카메라 프리뷰</title>
<style>
  :root { color-scheme: dark; --bg:#10151d; --panel:#18212d; --line:#344358;
          --blue:#48a7ff; --green:#49d17d; --red:#ff6b6b; --orange:#ffb454; }
  *{box-sizing:border-box} body{margin:0;font-family:system-ui,sans-serif;
    background:var(--bg);color:#edf3fa}
  header{padding:12px 18px;border-bottom:1px solid var(--line);
    display:flex;gap:16px;align-items:center;flex-wrap:wrap}
  h1{font-size:17px;margin:0}
  .controls{display:flex;gap:8px;align-items:center;flex-wrap:wrap;font-size:13px}
  input[type=number]{width:70px;background:#0d141e;color:#edf3fa;
    border:1px solid #43546c;border-radius:6px;padding:5px}
  button{background:#0d141e;color:#edf3fa;border:1px solid #43546c;
    border-radius:6px;padding:6px 10px;cursor:pointer;font-size:13px}
  button:hover{border-color:var(--blue)}
  button.active{background:#234d73;border-color:var(--blue)}
  main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));
    gap:14px;padding:14px}
  .cam{background:var(--panel);border:1px solid var(--line);border-radius:10px;
    padding:12px}
  .cam h2{font-size:14px;margin:0 0 8px;display:flex;justify-content:space-between;
    align-items:baseline;gap:8px}
  .meta{font-size:12px;color:#aebcd0;font-weight:400}
  .wrap{position:relative;line-height:0}
  img{width:100%;height:auto;border-radius:7px;border:1px solid var(--line);
    background:#05070a;cursor:crosshair;display:block}
  .dead{outline:2px solid var(--red)}
  .readout{margin-top:8px;font-size:12px;color:#b9c8da;min-height:34px;
    background:#0d141e;border-radius:6px;padding:7px;white-space:pre-wrap}
  table{width:100%;border-collapse:collapse;font-size:12px;margin-top:8px}
  th,td{border-bottom:1px solid #2c394c;padding:4px 5px;text-align:right}
  th:first-child,td:first-child{text-align:left}
  th{color:#8fa3ba;font-weight:600}
  .badge{display:inline-block;padding:3px 9px;border-radius:12px;
    background:#263448;font-size:12px}
  .badge.good{background:#1d4030;color:#8ef0b5}
  .badge.bad{background:#43222a;color:#ffb3b3}
  .warn{color:var(--orange)}
  .err{color:var(--red)}
  .ok{color:var(--green)}
</style>
</head>
<body>
<header>
  <h1>천장 카메라 프리뷰</h1>
  <div class="controls">
    <label>격자 <input id="grid" type="number" value="100" min="0" step="10">px</label>
    <button onclick="applyGrid()">적용</button>
    <button onclick="clearPoints()">찍은 점 지우기</button>
    <span id="overlap" class="badge">겹침 확인 중…</span>
    <span id="yolo" class="badge">YOLO 확인 중…</span>
    <span id="hint" class="meta">영상 클릭 = 픽셀 좌표. ArUco 한 변 <b id="msize">0.18</b> m 기준.</span>
  </div>
</header>
<main id="grid-root"></main>
<section id="bevbox" style="padding:0 14px 20px">
  <div class="cam">
    <h2>
      <span>BEV — 위에서 본 바닥 (두 카메라 정합 확인)</span>
      <span class="meta" id="bevmeta">준비 중…</span>
    </h2>
    <div class="controls" style="margin-bottom:10px">
      <button id="bmA" onclick="setBev('anaglyph')">색분리 (청록 vs 빨강)</button>
      <button id="bmB" onclick="setBev('average')">평균 (이중상)</button>
      <span class="meta">색분리에서 겹침 영역이 <b>회색</b>이면 정합,
        <b style="color:#7ff">청록</b>/<b style="color:#f77">빨강</b>으로 갈라지면 어긋난 것입니다.</span>
    </div>
    <div id="bevgrid" style="display:grid;
         grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px"></div>
  </div>
</section>
<script>
let CAMS = [];
let points = {};

function esc(s){ return String(s).replace(/[&<>"]/g, c =>
  ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }

async function boot(){
  const info = await (await fetch('/api/info')).json();
  CAMS = info.cameras;
  const root = document.getElementById('grid-root');
  root.innerHTML = CAMS.map((c,i) => `
    <section class="cam">
      <h2><span>${esc(c.label)}</span><span class="meta" id="meta${i}">연결 중…</span></h2>
      <div class="wrap">
        <img id="img${i}" src="/video/${i}" alt="${esc(c.label)}"
             onclick="pick(${i}, event)">
      </div>
      <div class="readout" id="out${i}">클릭해서 픽셀 좌표 확인</div>
      <div id="det${i}"></div>
      <div id="mk${i}"></div>
    </section>`).join('');
  CAMS.forEach((c,i)=>{ points[i]=[]; });
  if(info.bev && info.bev.ready){
    document.getElementById('bevgrid').innerHTML =
      CAMS.map((c,i)=>`<div><div class="meta" style="margin-bottom:5px">${esc(c.label)}</div>
        <img src="/bev/${i}" style="cursor:default"></div>`).join('')
      + `<div><div class="meta" style="margin-bottom:5px">합성 (정합 확인)</div>
         <img id="bevmerged" src="/bev/merged" style="cursor:default"></div>`;
  } else {
    document.getElementById('bevbox').style.display='none';
  }
  tick();
  setInterval(tick, 700);
}

async function tick(){
  let info;
  try { info = await (await fetch('/api/info')).json(); } catch(e){ return; }
  info.cameras.forEach((c,i)=>{
    const el = document.getElementById('meta'+i); if(!el) return;
    const img = document.getElementById('img'+i);
    if(!c.alive){
      el.innerHTML = '<span class="err">수신 없음</span> · ' + esc(c.topic);
      if(img) img.classList.add('dead');
      return;
    }
    if(img) img.classList.remove('dead');
    let size = c.width + '×' + c.height;
    let cls = 'ok';
    let note = '';
    if(c.calib_width && (c.width !== c.calib_width || c.height !== c.calib_height)){
      cls = 'warn';
      note = ' · <span class="warn">캘리브레이션 ' + c.calib_width + '×' + c.calib_height + '와 불일치</span>';
    }
    el.innerHTML = '<span class="'+cls+'">'+size+'</span> · '
      + c.fps.toFixed(1) + ' fps · ' + esc(c.topic) + note;
    renderMarkers(i, c.markers || []);
    renderDetections(i, c.detections || []);
  });

  document.getElementById('msize').textContent = info.marker_size_m;

  const y = info.yolo, yb = document.getElementById('yolo');
  if(y && yb){
    if(y.ready){
      const n = info.cameras.reduce((s,c)=>s+(c.detections||[]).length,0);
      yb.className = 'badge good';
      yb.textContent = 'YOLO 검출 ' + n + '개 (클래스 ' + y.classes.join(',') + ')';
    } else {
      yb.className = 'badge bad';
      yb.textContent = 'YOLO: ' + (y.error || '비활성');
    }
  }

  const b = info.bev;
  if(b){
    const bm = document.getElementById('bevmeta');
    if(!b.ready){
      if(bm) bm.innerHTML = '<span class="err">'+esc(b.error||'미준비')+'</span>';
    } else if(bm) {
      let txt = b.map_w.toFixed(2)+'×'+b.map_h.toFixed(2)+' m · '+b.ppm+' px/m · H '
        + b.cameras.length + '개 · 슬롯 ' + b.slots;
      const ov = b.overlap;
      if(ov){
        if(ov.correlation === null){
          txt += ' · 겹침 ' + ov.overlap_m2.toFixed(2) + ' m²';
        } else {
          const c = ov.correlation;
          const cls = c > 0.7 ? 'ok' : (c > 0.4 ? 'warn' : 'err');
          const word = c > 0.7 ? '정합 양호' : (c > 0.4 ? '주의' : '어긋남');
          txt += ' · 겹침 ' + ov.overlap_m2.toFixed(2) + ' m² · 상관 <span class="'
               + cls + '">' + c.toFixed(3) + ' (' + word + ')</span>';
        }
      }
      bm.innerHTML = txt;
    }
    for(const [id,mode] of [['bmA','anaglyph'],['bmB','average']]){
      const el=document.getElementById(id); if(!el) continue;
      el.classList.toggle('active', b.mode===mode);
    }
  }
  const ov = document.getElementById('overlap');
  const shared = info.shared_marker_ids || [];
  if(shared.length){
    ov.className = 'badge good';
    ov.textContent = '겹침 확인: ID ' + shared.join(', ') + ' 가 두 카메라에 동시에 보임';
  } else {
    ov.className = 'badge';
    ov.textContent = '겹침 미확인 — 마커를 두 화면에 동시에 보이는 곳으로';
  }
}

async function setBev(mode){
  await fetch('/api/bev_mode/'+mode, {method:'POST'});
  const img=document.getElementById('bevmerged');
  if(img) img.src='/bev/merged?t='+Date.now();
}

function verdict(spread){
  if(spread < 0.05) return ['양호','ok'];
  if(spread < 0.12) return ['주의','warn'];
  return ['불량','err'];
}

function renderDetections(i, dets){
  const box = document.getElementById('det'+i);
  if(!box) return;
  if(!dets.length){ box.innerHTML = ''; return; }
  box.innerHTML = '<table><tr><th>검출</th><th>신뢰도</th>'
    + '<th>화면 px</th><th>World X (m)</th><th>World Y (m)</th></tr>'
    + dets.map(d => {
        const w = d.world;
        return `<tr><td>${esc(d.name)}</td>`
          + `<td>${d.confidence.toFixed(2)}</td>`
          + `<td>${d.center_px[0].toFixed(0)}, ${d.center_px[1].toFixed(0)}</td>`
          + (w ? `<td class="ok">${w[0].toFixed(3)}</td><td class="ok">${w[1].toFixed(3)}</td>`
               : `<td colspan="2" class="warn">H 없음</td>`)
          + '</tr>';
      }).join('') + '</table>';
}

function renderMarkers(i, markers){
  const box = document.getElementById('mk'+i);
  if(!box) return;
  if(!markers.length){ box.innerHTML =
    '<div class="meta" style="margin-top:8px">ArUco 미검출</div>'; return; }
  box.innerHTML = '<table><tr><th>ID</th><th>중심 px</th><th>World (m)</th>'
    + '<th>한 변</th><th>변 편차</th><th>대각비</th><th>각도오차</th>'
    + '<th>mm/px</th><th>판정</th></tr>'
    + markers.map(m => {
        const [word, cls] = verdict(m.side_spread);
        const w = m.world;
        return `<tr><td>${m.id}</td>`
          + `<td>${m.center[0].toFixed(0)}, ${m.center[1].toFixed(0)}</td>`
          + (w ? `<td class="ok">${w[0].toFixed(2)}, ${w[1].toFixed(2)}</td>`
               : `<td class="warn">—</td>`)
          + `<td>${m.mean_side_px.toFixed(1)}</td>`
          + `<td class="${cls}">${(m.side_spread*100).toFixed(1)}%</td>`
          + `<td>${m.diagonal_ratio.toFixed(3)}</td>`
          + `<td>${m.max_angle_error_deg.toFixed(1)}°</td>`
          + `<td>${m.mm_per_px.toFixed(2)}</td>`
          + `<td class="${cls}">${word}</td></tr>`;
      }).join('') + '</table>';
}

function pick(i, ev){
  const img = ev.target;
  const r = img.getBoundingClientRect();
  const x = (ev.clientX - r.left) * img.naturalWidth  / r.width;
  const y = (ev.clientY - r.top ) * img.naturalHeight / r.height;
  const p = [Math.round(x), Math.round(y)];
  points[i].push(p);
  if(points[i].length > 2) points[i] = [p];
  const out = document.getElementById('out'+i);
  if(points[i].length === 1){
    out.textContent = `점 1: (${p[0]}, ${p[1]}) px`;
  } else {
    const [a,b] = points[i];
    const d = Math.hypot(b[0]-a[0], b[1]-a[1]);
    out.textContent = `점 1: (${a[0]}, ${a[1]}) px\n점 2: (${b[0]}, ${b[1]}) px\n`
      + `픽셀 거리: ${d.toFixed(1)} px   Δx ${b[0]-a[0]}  Δy ${b[1]-a[1]}`;
  }
}

function clearPoints(){
  CAMS.forEach((c,i)=>{ points[i]=[];
    const o=document.getElementById('out'+i); if(o) o.textContent='클릭해서 픽셀 좌표 확인'; });
}

async function applyGrid(){
  const step = parseInt(document.getElementById('grid').value, 10) || 0;
  await fetch('/api/grid/' + step, {method:'POST'});
  // MJPEG 스트림을 다시 열어야 새 격자가 반영된다
  CAMS.forEach((c,i)=>{
    const img=document.getElementById('img'+i);
    if(img) img.src = '/video/'+i+'?t='+Date.now();
  });
}

boot();
</script>
</body></html>'''


def _camera_key(text):
    """토픽이나 라벨에서 카메라 이름을 뽑는다.

    ``/cctv0/image_rect`` -> ``cctv0``, ``cam0 rect`` -> ``cam0``.
    """
    token = str(text).strip().split()[0] if str(text).strip() else ''
    parts = [p for p in token.split('/') if p]
    return parts[0] if parts else (token or 'cam')


def _homography_candidates(key, search_dirs):
    """그 카메라의 homography 파일이 있을 만한 경로를 순서대로 만든다.

    현장에서 파일 이름이 ``homography_cam0_rectified.npy``인데 토픽은
    ``/cctv0/...``인 식으로 엇갈리는 일이 잦다. cctvN <-> camN 을 서로
    바꿔가며 찾아 그 불일치를 흡수한다.
    """
    names = [key]
    if key.startswith('cctv'):
        names.append('cam' + key[4:])
    elif key.startswith('cam'):
        names.append('cctv' + key[3:])
    candidates = []
    for directory in search_dirs:
        for name in names:
            candidates.append(
                os.path.join(directory, f'homography_{name}_rectified.npy'))
    return candidates


def marker_metrics(corners, marker_size_m):
    """정사각형 마커가 화면에서 얼마나 찌그러졌는지 잰다.

    ArUco 마커는 실제로 정사각형이므로, **보정된 영상에서 정사각형으로
    보여야 정상**이다. 찌그러짐의 원인은 둘이다.

      * 렌즈 왜곡 보정이 틀림 — 화면 어디에 두든 찌그러진다
      * 카메라가 기울어짐 — 광축에서 멀어질수록 사다리꼴이 심해진다

    그래서 마커를 화면 여러 위치로 옮겨가며 이 값을 보면 두 원인을
    구분할 수 있다. 중심에서도 나쁘면 왜곡, 가장자리에서만 나빠지면 기울기다.

    반환값의 ``mm_per_px``는 그 지점의 실효 해상도다. 이 값이 크면
    (1픽셀이 넓은 거리를 뜻하면) 그 위치의 좌표 정밀도가 낮다.
    """
    points = [(float(c[0]), float(c[1])) for c in corners]
    if len(points) != 4:
        raise ValueError('marker corners must contain four points')
    size = float(marker_size_m)
    if size <= 0.0:
        raise ValueError('marker_size_m must be positive')

    def distance(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    sides = [distance(points[i], points[(i + 1) % 4]) for i in range(4)]
    mean_side = sum(sides) / 4.0
    if mean_side <= 1e-6:
        raise ValueError('degenerate marker')
    diagonals = [distance(points[0], points[2]), distance(points[1], points[3])]

    # 각 꼭짓점의 내각. 정사각형이면 전부 90도다.
    angles = []
    for i in range(4):
        prev_point = points[(i - 1) % 4]
        next_point = points[(i + 1) % 4]
        v1 = (prev_point[0] - points[i][0], prev_point[1] - points[i][1])
        v2 = (next_point[0] - points[i][0], next_point[1] - points[i][1])
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 <= 1e-9 or n2 <= 1e-9:
            angles.append(90.0)
            continue
        cosine = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
        angles.append(math.degrees(math.acos(max(-1.0, min(1.0, cosine)))))

    center = (sum(p[0] for p in points) / 4.0, sum(p[1] for p in points) / 4.0)
    return {
        'center': [round(center[0], 1), round(center[1], 1)],
        'sides_px': [round(s, 1) for s in sides],
        'mean_side_px': round(mean_side, 1),
        # 변 길이 편차 — 사다리꼴로 찌그러진 정도
        'side_spread': round((max(sides) - min(sides)) / mean_side, 4),
        # 두 대각선 길이 비 — 1.0이면 완전한 정사각형/직사각형
        'diagonal_ratio': round(max(diagonals) / max(1e-6, min(diagonals)), 4),
        # 90도에서 가장 많이 벗어난 꼭짓점
        'max_angle_error_deg': round(max(abs(a - 90.0) for a in angles), 2),
        'mm_per_px': round(size * 1000.0 / mean_side, 3),
    }


class CameraPreviewNode(Node):
    def __init__(self):
        super().__init__('camera_preview_node')

        self.declare_parameter(
            'image_topics_csv', '/cctv0/image_rect,/cctv2/image_rect')
        self.declare_parameter('labels_csv', '')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5005)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('grid_step_px', 100)
        # 실제 스트림 해상도가 캘리브레이션과 다르면 화면에서 경고한다.
        # 0이면 검사하지 않는다.
        self.declare_parameter('calibration_width_px', 640)
        self.declare_parameter('calibration_height_px', 480)
        self.declare_parameter('stale_after_s', 2.0)
        # --- ArUco ---
        # 마커는 실제로 정사각형이라 "보정이 맞는지" 재는 자로 쓸 수 있다.
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.18)
        # 검출은 매 프레임 할 필요가 없다. CPU를 아낀다.
        self.declare_parameter('aruco_every_n', 3)
        # --- BEV (bird's eye view) ---
        # 카메라별 homography로 바닥을 위에서 본 그림으로 편다. 두 카메라를
        # 겹쳐 보면 H 두 개가 같은 map 좌표계를 가리키는지 눈으로 확인된다.
        self.declare_parameter('enable_bev', True)
        # 비우면 ~/.ros/adaptive_valet_bot/homography_<label>_rectified.npy 를 찾는다.
        self.declare_parameter('homography_files_csv', '')
        self.declare_parameter('homography_scale_to_m', 1.0)
        self.declare_parameter(
            'layout_yaml', '~/.ros/adaptive_valet_bot/parking_layout.yaml')
        # 0이면 layout_yaml에서 읽는다.
        self.declare_parameter('map_width_m', 0.0)
        self.declare_parameter('map_height_m', 0.0)
        self.declare_parameter('bev_pixels_per_m', 100)
        # --- YOLO ---
        # 검출 결과를 카메라 화면과 BEV 양쪽에 world 좌표와 함께 표시한다.
        # CPU 추론이라 매 프레임 돌리면 프리뷰가 느려진다.
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('model_path', 'yolov8n.pt')
        # COCO 기본: 2=car 3=motorcycle 5=bus 7=truck. 사람은 [0].
        self.declare_parameter('yolo_class_ids', [2, 3, 5, 7])
        self.declare_parameter('yolo_confidence', 0.4)
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('yolo_every_n', 10)
        # .engine 을 쓸 때 task 를 정하는 데 필요하다.
        # 'coco' | 'vehicle_seg' | 'parking_seg'
        self.declare_parameter('model_mode', 'coco')

        if not DEPS_OK:
            raise RuntimeError(
                'camera_preview 의존성 없음 (cv2, numpy, cv_bridge, flask)')

        topics = [t.strip() for t in
                  str(self.get_parameter('image_topics_csv').value).split(',')
                  if t.strip()]
        if not topics:
            raise ValueError('image_topics_csv must not be empty')
        labels = [l.strip() for l in
                  str(self.get_parameter('labels_csv').value).split(',')
                  if l.strip()]
        if labels and len(labels) != len(topics):
            raise ValueError('labels_csv 길이가 image_topics_csv와 다릅니다')
        if not labels:
            # '/cctv0/image_rect' -> 'cctv0'. 토픽 전체를 라벨로 쓰면 화면도
            # 지저분하고, homography 파일 자동 탐색 이름도 깨진다.
            labels = [_camera_key(topic) for topic in topics]

        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1,100]')
        self.grid_step = max(0, int(self.get_parameter('grid_step_px').value))
        self.calib_w = int(self.get_parameter('calibration_width_px').value)
        self.calib_h = int(self.get_parameter('calibration_height_px').value)
        self.stale_after = float(self.get_parameter('stale_after_s').value)
        self.enable_aruco = bool(self.get_parameter('enable_aruco').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        if self.marker_size_m <= 0.0:
            raise ValueError('marker_size_m must be positive')
        self.aruco_every_n = max(1, int(self.get_parameter('aruco_every_n').value))
        self.detector = None
        if self.enable_aruco:
            self.detector = ArucoDetectorCompat(
                cv2, str(self.get_parameter('aruco_dict').value))
        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self._setup_yolo()

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        # 종료 시 MJPEG 제너레이터가 빠져나오게 한다. 이게 없으면 Ctrl+C 후
        # 스트림 스레드가 남아 프로세스가 안 죽는다.
        self._stop_event = threading.Event()
        self._mask_shape = None
        self.cameras = []
        for label, topic in zip(labels, topics):
            state = {
                'label': label, 'topic': topic, 'frame': None,
                'wall': 0.0, 'count': 0, 'fps': 0.0, 'fps_wall': time.monotonic(),
                'fps_count': 0, 'markers': [], 'marker_wall': 0.0,
                'detections': [], 'detection_wall': 0.0,
            }
            self.cameras.append(state)
            self.create_subscription(
                Image, topic,
                lambda msg, s=state: self.image_cb(s, msg),
                qos_profile_sensor_data)

        self._bev_mode = 'anaglyph'
        self._setup_bev(labels)

        self._app = self._make_app()
        self._server = make_server(
            self.web_host, self.web_port, self._app, threaded=True)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name='camera-preview-web', daemon=True)
        self._thread.start()

        self.get_logger().warn(
            f'카메라 프리뷰: http://{self.web_host}:{self.web_port}/  '
            '(신뢰 가능한 내부망에서만 사용)')
        self.get_logger().info(
            'VSCode 원격이면 PORTS 탭에서 '
            f'{self.web_port} 를 forward 하세요')
        for state in self.cameras:
            self.get_logger().info(
                f"  {state['label']} <- {state['topic']}")

    # ------------------------------------------------------------------
    def image_cb(self, state, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # 인코딩이 달라도 노드가 죽지 않게 한다
            self.get_logger().warn(
                f"[{state['label']}] 이미지 변환 실패: {exc}",
                throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        markers = None
        if self.detector is not None and state['count'] % self.aruco_every_n == 0:
            markers = self._detect_markers(frame, state)
        detections = None
        if self.yolo is not None and state['count'] % self.yolo_every_n == 0:
            detections = self._detect_vehicles(frame, state)
        with self._lock:
            state['frame'] = frame
            state['wall'] = now
            if markers is not None:
                state['markers'] = markers
                state['marker_wall'] = now
            if detections is not None:
                state['detections'] = detections
                state['detection_wall'] = now
            state['count'] += 1
            state['fps_count'] += 1
            elapsed = now - state['fps_wall']
            if elapsed >= 1.0:
                state['fps'] = state['fps_count'] / elapsed
                state['fps_count'] = 0
                state['fps_wall'] = now

    def _detect_markers(self, frame, state):
        """프레임에서 ArUco를 찾아 찌그러짐 지표까지 계산해 돌려준다."""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detect_markers(gray)
        except Exception as exc:
            self.get_logger().warn(
                f"[{state['label']}] ArUco 검출 실패: {exc}",
                throttle_duration_sec=10.0)
            return []
        if ids is None:
            return []
        try:
            flat_ids = [int(v) for v in ids.flatten()]
        except AttributeError:
            flat_ids = [int(v) for v in ids]

        found = []
        for index, marker_id in enumerate(flat_ids):
            candidate = corners[index]
            if len(candidate) == 1 and hasattr(candidate[0], '__len__'):
                points = candidate[0]
            else:
                points = candidate
            try:
                metrics = marker_metrics(points, self.marker_size_m)
            except (TypeError, ValueError):
                continue
            metrics['id'] = marker_id
            metrics['corners'] = [[float(pt[0]), float(pt[1])] for pt in points]
            metrics['world'] = self._pixel_to_world(
                state['label'], metrics['center'][0], metrics['center'][1])
            found.append(metrics)
        found.sort(key=lambda m: m['id'])
        return found

    def _draw_markers(self, canvas, markers):
        for marker in markers:
            pts = np.asarray(marker['corners'], dtype=np.int32)
            # 변 길이 편차가 크면 빨강, 보통이면 노랑, 좋으면 초록
            spread = marker['side_spread']
            colour = ((80, 230, 100) if spread < 0.05
                      else (40, 200, 255) if spread < 0.12
                      else (60, 60, 240))
            cv2.polylines(canvas, [pts], True, colour, 2)
            # 첫 코너(TL)를 굵게 찍어 코너 순서를 눈으로 확인한다
            cv2.circle(canvas, tuple(pts[0]), 5, colour, -1)
            cx, cy = (int(marker['center'][0]), int(marker['center'][1]))
            lines = [
                f"ID{marker['id']}  {marker['mean_side_px']:.0f}px",
                f"dev {spread * 100:.1f}%  {marker['mm_per_px']:.1f}mm/px",
            ]
            world = marker.get('world')
            if world is not None:
                lines.append(f"({world[0]:.2f}, {world[1]:.2f})m")
            for row, text in enumerate(lines):
                origin = (cx + 12, cy - 6 + row * 17)
                cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, colour, 1, cv2.LINE_AA)
        return canvas

    # ------------------------------------------------------------------
    # YOLO
    # ------------------------------------------------------------------
    def _setup_yolo(self):
        """YOLO를 준비한다. 실패해도 노드는 계속 돈다.

        이 노드의 본업은 카메라 확인이므로, 모델이 없거나 ultralytics가
        안 깔려 있다고 해서 프리뷰까지 죽으면 곤란하다. 경고만 남기고
        검출 기능만 끈다.
        """
        self.yolo = None
        self.yolo_error = ''
        self.yolo_names = {}
        if not bool(self.get_parameter('enable_yolo').value):
            self.yolo_error = 'enable_yolo=false'
            return
        self.yolo_class_ids = {int(v) for v in
                               self.get_parameter('yolo_class_ids').value}
        self.yolo_conf = float(self.get_parameter('yolo_confidence').value)
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.yolo_every_n = max(1, int(self.get_parameter('yolo_every_n').value))
        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        self.model_mode = str(self.get_parameter('model_mode').value)
        try:
            from ultralytics import YOLO
            self.yolo, task = load_yolo_model(YOLO, model_path, self.model_mode)
            names = getattr(self.yolo, 'names', {}) or {}
            self.yolo_names = ({int(k): str(v) for k, v in names.items()}
                               if isinstance(names, dict)
                               else {i: str(n) for i, n in enumerate(names)})
            labels = ', '.join(
                f'{i}={self.yolo_names.get(i, "?")}'
                for i in sorted(self.yolo_class_ids))
            self.get_logger().info(
                f'YOLO 로드: {model_path} | mode={self.model_mode} | '
                f'task={task or "auto"} | 검출 클래스 {labels} | '
                f'conf={self.yolo_conf} imgsz={self.yolo_imgsz} '
                f'every_n={self.yolo_every_n}')
        except Exception as exc:
            self.yolo = None
            self.yolo_error = f'YOLO 로드 실패: {exc}'
            self.get_logger().warn(self.yolo_error)

    def _detect_vehicles(self, frame, state):
        """YOLO 검출 + 각 검출의 world 좌표까지 계산해 돌려준다."""
        try:
            results = self.yolo(frame, conf=self.yolo_conf,
                                imgsz=self.yolo_imgsz, verbose=False)
        except Exception as exc:
            self.get_logger().warn(
                f"[{state['label']}] YOLO 추론 실패: {exc}",
                throttle_duration_sec=10.0)
            return []
        found = []
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue
            for box in boxes:
                try:
                    class_id = int(box.cls[0])
                    if class_id not in self.yolo_class_ids:
                        continue
                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                    confidence = float(box.conf[0])
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue
                center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                found.append({
                    'class_id': class_id,
                    'name': self.yolo_names.get(class_id, str(class_id)),
                    'confidence': round(confidence, 3),
                    'box': [round(x1, 1), round(y1, 1),
                            round(x2, 1), round(y2, 1)],
                    'center_px': [round(center[0], 1), round(center[1], 1)],
                    'world': self._pixel_to_world(state['label'], *center),
                })
        found.sort(key=lambda d: -d['confidence'])
        return found

    def _pixel_to_world(self, label, px, py):
        """영상 픽셀을 map 좌표(m)로. H가 없으면 None."""
        matrix = self.pixel_to_world_H.get(label)
        if matrix is None:
            return None
        vector = matrix @ np.array([float(px), float(py), 1.0])
        if abs(float(vector[2])) < 1e-12:
            return None
        x = float(vector[0] / vector[2])
        y = float(vector[1] / vector[2])
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return [round(x, 3), round(y, 3)]

    def _draw_detections(self, canvas, detections):
        for item in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in item['box']]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 170, 60), 2)
            world = item['world']
            text = f"{item['name']} {item['confidence']:.2f}"
            if world is not None:
                text += f"  ({world[0]:.2f}, {world[1]:.2f})m"
            origin = (x1, max(14, y1 - 6))
            cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 170, 60), 1, cv2.LINE_AA)
        return canvas

    # ------------------------------------------------------------------
    # BEV
    # ------------------------------------------------------------------
    def _setup_bev(self, labels):
        """카메라별 homography를 읽고 metre -> BEV 픽셀 변환을 준비한다.

        BEV 변환은 두 단계다.

            픽셀 --H--> map metre --(스케일/상하반전)--> BEV 픽셀

        두 번째 행렬은 ``bev_layout_calibrator_node._render_preview``와 같은
        관례를 쓴다. ROS map처럼 원점이 좌하단이고 +y가 위쪽이 되도록
        표시 좌표만 뒤집는다.
        """
        self.bev_ready = False
        self.bev_error = ''
        self.homographies = {}
        # BEV 그리기용 행렬과 별개로, "픽셀 -> map metre" 원본도 갖고 있어야
        # YOLO/ArUco 검출의 world 좌표를 계산할 수 있다.
        self.pixel_to_world_H = {}
        self.bev_masks = {}
        self.slots = []
        self.waiting = None

        if not bool(self.get_parameter('enable_bev').value):
            self.bev_error = 'enable_bev=false'
            return

        # --- layout에서 map 크기와 슬롯을 가져온다(있으면) ---
        map_w = float(self.get_parameter('map_width_m').value)
        map_h = float(self.get_parameter('map_height_m').value)
        layout_path = os.path.expanduser(
            str(self.get_parameter('layout_yaml').value))
        if os.path.isfile(layout_path):
            try:
                from cooperative_parking_robot.bev_layout_core import (
                    load_layout_yaml)
                layout = load_layout_yaml(layout_path)
                if layout:
                    if map_w <= 0.0:
                        map_w = layout['map_width_m']
                    if map_h <= 0.0:
                        map_h = layout['map_height_m']
                    self.slots = list(zip(
                        [s.slot_id for s in layout['slots']],
                        layout['slot_polygons']))
                    self.waiting = layout['waiting_polygon']
                    self.get_logger().info(
                        f'layout 로드: {layout_path} '
                        f'(슬롯 {len(self.slots)}개)')
            except Exception as exc:
                self.get_logger().warn(f'layout 로드 실패: {exc}')
        if map_w <= 0.0 or map_h <= 0.0:
            map_w, map_h = 6.0, 4.0
            self.get_logger().warn(
                'map 크기를 알 수 없어 6.0x4.0m 로 가정한다 — '
                'map_width_m/map_height_m 또는 layout_yaml 을 주면 정확해진다')
        self.map_w, self.map_h = map_w, map_h

        ppm = int(self.get_parameter('bev_pixels_per_m').value)
        if ppm <= 0:
            raise ValueError('bev_pixels_per_m must be positive')
        self.bev_ppm = ppm
        self.bev_w = max(1, int(round(map_w * ppm)))
        self.bev_h = max(1, int(round(map_h * ppm)))
        # metre -> BEV 픽셀 (원점 좌하단, +y 위쪽)
        self.metre_to_bev = np.array([
            [ppm, 0.0, 0.0],
            [0.0, -ppm, self.bev_h - 1.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        # --- homography 파일 ---
        files = [f.strip() for f in
                 str(self.get_parameter('homography_files_csv').value).split(',')
                 if f.strip()]
        if files and len(files) != len(labels):
            raise ValueError(
                'homography_files_csv 길이가 image_topics_csv와 다릅니다')
        search_dirs = [os.path.expanduser('~/.ros/adaptive_valet_bot')]
        try:
            from ament_index_python.packages import get_package_share_directory
            search_dirs.append(os.path.join(
                get_package_share_directory('cooperative_parking_robot'),
                'config'))
        except Exception:
            pass
        # 소스 트리의 config/ 도 본다 (--symlink-install 로 개발 중일 때)
        source_config = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config')
        if os.path.isdir(source_config):
            search_dirs.append(source_config)

        auto = not files
        if auto:
            files = [None] * len(labels)

        scale = float(self.get_parameter('homography_scale_to_m').value)
        if scale <= 0.0:
            raise ValueError('homography_scale_to_m must be positive')
        scale_matrix = np.diag([scale, scale, 1.0])

        missing = []
        for label, path in zip(labels, files):
            if path is None:
                tried = _homography_candidates(
                    _camera_key(label), search_dirs)
                expanded = next(
                    (c for c in tried if os.path.isfile(c)), None)
                if expanded is None:
                    missing.append(
                        f'{label}: 다음 경로에 없음 -> ' + ' | '.join(tried))
                    continue
            else:
                expanded = os.path.expanduser(path)
                if not os.path.isfile(expanded):
                    missing.append(expanded)
                    continue
            try:
                matrix = np.load(expanded)
                if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
                    raise ValueError('3x3 유한 행렬이 아님')
                if abs(float(np.linalg.det(matrix))) < 1e-12:
                    raise ValueError('특이행렬')
            except Exception as exc:
                missing.append(f'{expanded} ({exc})')
                continue
            self.pixel_to_world_H[label] = scale_matrix @ matrix
            self.homographies[label] = (
                self.metre_to_bev @ self.pixel_to_world_H[label])
            self.get_logger().info(f'[{label}] BEV homography: {expanded}')

        if missing:
            self.bev_error = 'homography 없음: ' + ', '.join(missing)
            self.get_logger().warn(self.bev_error)
        self.bev_ready = len(self.homographies) > 0
        if self.bev_ready:
            self.get_logger().info(
                f'BEV 준비 완료 {self.bev_w}x{self.bev_h}px '
                f'({map_w:.2f}x{map_h:.2f} m @ {ppm} px/m)')

    def _bev_mask(self, label, shape):
        """그 카메라가 BEV에서 실제로 덮는 영역. 한 번만 계산해 캐시한다."""
        key = (label, shape)
        cached = self.bev_masks.get(key)
        if cached is not None:
            return cached
        # 1채널로 만들어야 한다. 3채널로 워프하면 마스크가 HxWx3이 되어
        # 면적 계산이 3배로 부풀고, 회색조와 인덱싱할 때 차원이 안 맞는다.
        white = np.full(shape[:2], 255, dtype=np.uint8)
        mask = cv2.warpPerspective(
            white, self.homographies[label], (self.bev_w, self.bev_h))
        mask = (mask > 0).astype(np.uint8)
        self.bev_masks[key] = mask
        return mask

    def _warp_to_bev(self, frame, label):
        return cv2.warpPerspective(
            frame, self.homographies[label], (self.bev_w, self.bev_h))

    def _draw_bev_overlay(self, canvas, show_coverage=True):
        """미터 격자, 원점, 슬롯, 대기영역, 카메라 시야 경계를 그린다."""
        ppm = self.bev_ppm

        def to_px(x, y):
            return (int(round(x * ppm)),
                    int(round(self.bev_h - 1 - y * ppm)))

        half = max(1, ppm // 2)
        for gx in range(0, self.bev_w, half):
            major = (gx % ppm) == 0
            cv2.line(canvas, (gx, 0), (gx, self.bev_h - 1),
                     (95, 95, 95) if major else (45, 45, 45), 1)
        for gy in range(0, self.bev_h, half):
            major = ((self.bev_h - 1 - gy) % ppm) == 0
            cv2.line(canvas, (0, gy), (self.bev_w - 1, gy),
                     (95, 95, 95) if major else (45, 45, 45), 1)

        if show_coverage:
            palette = [(255, 200, 80), (120, 120, 255), (120, 255, 120)]
            for index, label in enumerate(self.homographies):
                mask = self.bev_masks.get((label, self._mask_shape))
                if mask is None:
                    continue
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(canvas, contours, -1,
                                 palette[index % len(palette)], 2)

        for slot_id, polygon in self.slots:
            pts = np.asarray([to_px(x, y) for x, y in polygon], dtype=np.int32)
            cv2.polylines(canvas, [pts], True, (80, 230, 100), 2)
            cx = int(sum(p[0] for p in pts) / len(pts))
            cy = int(sum(p[1] for p in pts) / len(pts))
            cv2.putText(canvas, slot_id, (cx - 12, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 230, 100), 1,
                        cv2.LINE_AA)
        if self.waiting:
            pts = np.asarray([to_px(x, y) for x, y in self.waiting],
                             dtype=np.int32)
            cv2.polylines(canvas, [pts], True, (40, 170, 255), 2)

        # --- 검출 결과를 world 좌표 그대로 BEV에 찍는다 ---
        # 카메라마다 색을 달리해 어느 카메라가 본 것인지 구분한다. 두
        # 카메라가 같은 물체를 봤다면 두 점이 겹쳐야 정합된 것이다.
        palette = [(255, 200, 80), (120, 120, 255), (120, 255, 120)]
        with self._lock:
            snapshot = [(s['label'], list(s.get('detections') or []),
                         list(s.get('markers') or []))
                        for s in self.cameras]
        for index, (label, detections, markers) in enumerate(snapshot):
            colour = palette[index % len(palette)]
            for item in detections:
                world = item.get('world')
                if world is None:
                    continue
                point = to_px(world[0], world[1])
                cv2.circle(canvas, point, 7, colour, -1)
                cv2.circle(canvas, point, 7, (0, 0, 0), 1)
                text = (f"{item['name']} "
                        f"({world[0]:.2f},{world[1]:.2f})")
                for thickness, shade in ((3, (0, 0, 0)), (1, colour)):
                    cv2.putText(canvas, text, (point[0] + 10, point[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, shade,
                                thickness, cv2.LINE_AA)
            for marker in markers:
                world = marker.get('world')
                if world is None:
                    continue
                point = to_px(world[0], world[1])
                size = 7
                cv2.rectangle(canvas,
                              (point[0] - size, point[1] - size),
                              (point[0] + size, point[1] + size), colour, 2)
                text = (f"ID{marker['id']} "
                        f"({world[0]:.2f},{world[1]:.2f})")
                for thickness, shade in ((3, (0, 0, 0)), (1, colour)):
                    cv2.putText(canvas, text, (point[0] + 11, point[1] + 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, shade,
                                thickness, cv2.LINE_AA)

        origin = to_px(0.0, 0.0)
        cv2.circle(canvas, origin, 6, (255, 255, 255), -1)
        cv2.putText(canvas, '(0,0)', (origin[0] + 8, origin[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(canvas, f'{self.map_w:.2f} x {self.map_h:.2f} m  '
                            f'grid 1.0m', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, f'{self.map_w:.2f} x {self.map_h:.2f} m  '
                            f'grid 1.0m', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                    cv2.LINE_AA)
        return canvas

    def _bev_single(self, index):
        with self._lock:
            state = self.cameras[index]
            frame = None if state['frame'] is None else state['frame'].copy()
            label = state['label']
        if frame is None or label not in self.homographies:
            return None
        self._mask_shape = frame.shape
        self._bev_mask(label, frame.shape)
        canvas = self._warp_to_bev(frame, label)
        return self._draw_bev_overlay(canvas, show_coverage=False)

    def _bev_merged(self, mode='anaglyph'):
        """두 카메라의 BEV를 겹쳐 정합을 확인한다.

        ``anaglyph``: 첫 카메라는 청록, 두 번째는 빨강으로 칠한다. 두 H가
        같은 좌표계를 가리키면 겹침 영역의 바닥 무늬가 **회색/흰색**으로
        합쳐진다. 어긋나면 청록과 빨강이 갈라져 보이므로 오차 방향과 크기가
        바로 읽힌다.

        ``average``: 단순 평균. 어긋나면 이중상(ghost)으로 나타난다.
        """
        warped = {}
        with self._lock:
            snapshot = [(s['label'],
                         None if s['frame'] is None else s['frame'].copy())
                        for s in self.cameras]
        for label, frame in snapshot:
            if frame is None or label not in self.homographies:
                continue
            self._mask_shape = frame.shape
            self._bev_mask(label, frame.shape)
            warped[label] = (self._warp_to_bev(frame, label),
                             self.bev_masks[(label, frame.shape)])
        if not warped:
            return None

        canvas = np.zeros((self.bev_h, self.bev_w, 3), dtype=np.uint8)
        labels = list(warped)
        if mode == 'average' or len(labels) == 1:
            total = np.zeros((self.bev_h, self.bev_w, 3), dtype=np.float32)
            count = np.zeros((self.bev_h, self.bev_w, 1), dtype=np.float32)
            for label in labels:
                image, mask = warped[label]
                total += image.astype(np.float32) * mask[:, :, None]
                count += mask[:, :, None].astype(np.float32)
            np.divide(total, np.maximum(count, 1.0), out=total)
            canvas = total.astype(np.uint8)
        else:
            # BGR 채널에 나눠 칠한다: 0번 -> B,G (청록) / 1번 -> R (빨강)
            first_gray = cv2.cvtColor(warped[labels[0]][0], cv2.COLOR_BGR2GRAY)
            canvas[:, :, 0] = first_gray
            canvas[:, :, 1] = first_gray
            if len(labels) > 1:
                second_gray = cv2.cvtColor(
                    warped[labels[1]][0], cv2.COLOR_BGR2GRAY)
                canvas[:, :, 2] = second_gray
        return self._draw_bev_overlay(canvas, show_coverage=True)

    def _bev_overlap_stats(self):
        """겹침 영역에서 두 BEV가 얼마나 닮았는지 수치로 낸다.

        완전히 정합되면 두 그림이 거의 같으므로 상관계수가 1에 가깝다.
        어긋나면 급격히 떨어진다. RMS 재투영 오차처럼 절대적인 기준은 아니고
        "지금 손댄 게 나아졌는가"를 비교하는 상대 지표로 쓴다.
        """
        with self._lock:
            snapshot = [(s['label'],
                         None if s['frame'] is None else s['frame'].copy())
                        for s in self.cameras]
        usable = [(l, f) for l, f in snapshot
                  if f is not None and l in self.homographies]
        if len(usable) < 2:
            return None
        (label_a, frame_a), (label_b, frame_b) = usable[0], usable[1]
        self._mask_shape = frame_a.shape
        mask_a = self._bev_mask(label_a, frame_a.shape)
        mask_b = self._bev_mask(label_b, frame_b.shape)
        overlap = (mask_a & mask_b).astype(bool)
        area_px = int(overlap.sum())
        if area_px < 200:
            return {'overlap_px': area_px, 'overlap_m2': 0.0,
                    'correlation': None}
        gray_a = cv2.cvtColor(self._warp_to_bev(frame_a, label_a),
                              cv2.COLOR_BGR2GRAY).astype(np.float32)[overlap]
        gray_b = cv2.cvtColor(self._warp_to_bev(frame_b, label_b),
                              cv2.COLOR_BGR2GRAY).astype(np.float32)[overlap]
        if gray_a.std() < 1e-3 or gray_b.std() < 1e-3:
            correlation = None
        else:
            correlation = float(np.corrcoef(gray_a, gray_b)[0, 1])
        return {
            'overlap_px': area_px,
            'overlap_m2': round(area_px / (self.bev_ppm ** 2), 3),
            'correlation': None if correlation is None else round(correlation, 4),
        }

    def _annotate(self, frame, state):
        """격자·중심선·라벨을 그린다. 원본은 건드리지 않는다."""
        canvas = frame.copy()
        height, width = canvas.shape[:2]
        self._draw_markers(canvas, state.get('markers') or [])
        self._draw_detections(canvas, state.get('detections') or [])
        if self.grid_step > 0:
            for x in range(self.grid_step, width, self.grid_step):
                major = (x % (self.grid_step * 5) == 0)
                cv2.line(canvas, (x, 0), (x, height - 1),
                         (90, 90, 90) if major else (55, 55, 55), 1)
            for y in range(self.grid_step, height, self.grid_step):
                major = (y % (self.grid_step * 5) == 0)
                cv2.line(canvas, (0, y), (width - 1, y),
                         (90, 90, 90) if major else (55, 55, 55), 1)
        # 영상 중심 십자 — 광축 위치를 눈으로 잡는 기준
        cx, cy = width // 2, height // 2
        cv2.line(canvas, (cx - 18, cy), (cx + 18, cy), (0, 200, 255), 1)
        cv2.line(canvas, (cx, cy - 18), (cx, cy + 18), (0, 200, 255), 1)
        label = f"{state['label']}  {width}x{height}  {state['fps']:.1f}fps"
        cv2.putText(canvas, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (80, 230, 100), 1, cv2.LINE_AA)
        if self.calib_w > 0 and (width != self.calib_w or height != self.calib_h):
            warn = f"! calib {self.calib_w}x{self.calib_h} mismatch"
            cv2.putText(canvas, warn, (8, 46), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, warn, (8, 46), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (40, 170, 255), 1, cv2.LINE_AA)
        return canvas

    def _placeholder(self, state):
        canvas = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(canvas, f"{state['label']}: no image",
                    (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (60, 60, 220), 2, cv2.LINE_AA)
        cv2.putText(canvas, state['topic'], (20, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1,
                    cv2.LINE_AA)
        return canvas

    def _encode(self, index):
        with self._lock:
            if index < 0 or index >= len(self.cameras):
                return None
            state = self.cameras[index]
            frame = None if state['frame'] is None else state['frame'].copy()
            snapshot = dict(state)
        canvas = self._placeholder(snapshot) if frame is None \
            else self._annotate(frame, snapshot)
        ok, buf = cv2.imencode(
            '.jpg', canvas, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return buf.tobytes() if ok else None

    def _make_app(self):
        app = Flask('camera_preview')

        @app.get('/')
        def index():
            return Response(_HTML, mimetype='text/html; charset=utf-8')

        @app.get('/health')
        def health():
            return jsonify({'cameras': len(self.cameras)})

        @app.get('/api/info')
        def info():
            now = time.monotonic()
            payload = []
            with self._lock:
                for state in self.cameras:
                    frame = state['frame']
                    alive = (frame is not None and
                             now - state['wall'] <= self.stale_after)
                    payload.append({
                        'label': state['label'],
                        'topic': state['topic'],
                        'alive': bool(alive),
                        'width': 0 if frame is None else int(frame.shape[1]),
                        'height': 0 if frame is None else int(frame.shape[0]),
                        'fps': float(state['fps']),
                        'frames': int(state['count']),
                        'calib_width': self.calib_w,
                        'calib_height': self.calib_h,
                        'markers': [
                            {k: v for k, v in m.items() if k != 'corners'}
                            for m in (state['markers'] or [])
                        ] if now - state['marker_wall'] <= self.stale_after else [],
                        'detections': (
                            state['detections']
                            if now - state['detection_wall'] <= self.stale_after
                            else []),
                    })
            # 같은 ID가 두 카메라에 동시에 보이면 그 지점이 겹침 영역이다.
            seen = {}
            for cam in payload:
                for marker in cam['markers']:
                    seen.setdefault(marker['id'], []).append(cam['label'])
            shared = sorted(mid for mid, cams in seen.items() if len(cams) > 1)
            return jsonify({
                'cameras': payload,
                'grid_step_px': self.grid_step,
                'marker_size_m': self.marker_size_m,
                'shared_marker_ids': shared,
                'yolo': {
                    'ready': self.yolo is not None,
                    'error': self.yolo_error,
                    'classes': (sorted(self.yolo_class_ids)
                                if self.yolo is not None else []),
                },
                'bev': {
                    'ready': self.bev_ready,
                    'error': self.bev_error,
                    'mode': self._bev_mode,
                    'map_w': getattr(self, 'map_w', 0.0),
                    'map_h': getattr(self, 'map_h', 0.0),
                    'ppm': getattr(self, 'bev_ppm', 0),
                    'cameras': sorted(self.homographies),
                    'slots': len(self.slots),
                    'overlap': (self._bev_overlap_stats()
                                if self.bev_ready else None),
                },
            })

        @app.post('/api/grid/<int:step>')
        def set_grid(step):
            self.grid_step = max(0, min(1000, int(step)))
            return jsonify({'grid_step_px': self.grid_step})

        def _jpeg(canvas):
            ok, buf = cv2.imencode(
                '.jpg', canvas,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            return buf.tobytes() if ok else None

        def _mjpeg(producer):
            def stream():
                while not self._stop_event.is_set():
                    canvas = producer()
                    payload = None if canvas is None else _jpeg(canvas)
                    if payload is not None:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                               b'Content-Length: ' +
                               str(len(payload)).encode() + b'\r\n\r\n' +
                               payload + b'\r\n')
                    self._stop_event.wait(0.12)
            return Response(
                stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.get('/bev/<int:index>')
        def bev_single(index):
            if not self.bev_ready:
                return jsonify({'error': self.bev_error or 'BEV 미준비'}), 404
            if index < 0 or index >= len(self.cameras):
                return jsonify({'error': 'unknown camera index'}), 404
            return _mjpeg(lambda: self._bev_single(index))

        @app.get('/bev/merged')
        def bev_merged():
            if not self.bev_ready:
                return jsonify({'error': self.bev_error or 'BEV 미준비'}), 404
            mode = self._bev_mode
            return _mjpeg(lambda: self._bev_merged(mode))

        @app.post('/api/bev_mode/<mode>')
        def set_bev_mode(mode):
            self._bev_mode = 'average' if mode == 'average' else 'anaglyph'
            return jsonify({'mode': self._bev_mode})

        @app.get('/video/<int:index>')
        def video(index):
            if index < 0 or index >= len(self.cameras):
                return jsonify({'error': 'unknown camera index'}), 404

            def stream():
                while not self._stop_event.is_set():
                    payload = self._encode(index)
                    if payload is not None:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                               b'Content-Length: ' +
                               str(len(payload)).encode() + b'\r\n\r\n' +
                               payload + b'\r\n')
                    # 약 12fps — 눈으로 확인하는 용도라 충분하고, 원본 30fps를
                    # 그대로 밀면 인코딩이 CPU를 잡아먹는다.
                    self._stop_event.wait(0.08)

            return Response(
                stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        return app

    def destroy_node(self):
        self._stop_event.set()
        try:
            self._server.shutdown()
        except Exception:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
