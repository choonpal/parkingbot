#!/usr/bin/env python3
r"""천장 카메라 여러 대를 브라우저에서 나란히 보는 경량 프리뷰.

무엇을 위한 노드인가
--------------------
카메라를 천장에 달고 나서 **실측을 시작하기 전에** 확인해야 하는 것들이 있다.

  * 각 카메라가 주차장의 어느 부분을 보는가
  * 두 시야가 실제로 겹치는가, 겹침 폭은 충분한가
  * 주차면이 두 카메라 경계에 걸치지는 않는가
  * 렌즈 왜곡 보정이 제대로 되고 있는가 (raw와 rect를 나란히 비교)
  * 바닥 기준점으로 쓸 지점이 두 영상에 모두 잘 보이는가

``jetson_vision_web_node``도 MJPEG를 주지만 카메라 한 대만 다룬다. 이 노드는
여러 영상을 한 페이지에서 비교하며, 필요하면 내장 YOLO를 쓰거나 Production
검출 토픽을 직접 받아 모델을 추가로 올리지 않고 상세 결과를 표시한다.

화면에서 할 수 있는 것
----------------------
  * 영상 클릭 -> 그 지점의 **원본 픽셀 좌표**를 읽는다. 등록 도구에서 찍을
    기준점을 미리 가늠하거나, 크롭 범위를 정할 때 쓴다.
  * 두 점을 찍으면 픽셀 거리를 알려준다.
  * 격자 간격을 바꿔가며 영상 왜곡(직선이 휘는지)을 눈으로 확인한다.
  * **ArUco 마커 상태를 확인한다.** 최소 변 길이·면적·화면 경계 여유·최근
    검출 연속성과 실제 주행 노드의 ``/{front,rear}/cctv_marker_visible`` 을
    함께 보여준다. 원근 투영으로 생기는 변 편차·대각선 비·각도 오차는
    참고값일 뿐, 그 값만으로 주행 가능/불가를 판정하지 않는다.
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
      -p homography_files_csv:='/path/cam0.npy,/path/cam2.npy' \\
      -p layout_yaml:="${HOME}/.ros/adaptive_valet_bot/parking_layout.yaml"

    # 사람을 검출해 보기 (차가 없을 때 파이프라인 확인용)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p yolo_class_ids:='[0]' -p model_path:=$HOME/yolov8n.pt

    # YOLO 끄기 (CPU가 부족할 때)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p enable_yolo:=false

    # Production YOLO 결과를 그대로 보기 (추론/모델 추가 로드 없음)
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p enable_yolo:=false \\
      -p detection_topics_csv:='/cctv0/detections,/cctv2/detections'

    # 마커 크기가 다르거나 dictionary가 다를 때
    ros2 run cooperative_parking_robot camera_preview --ros-args \\
      -p marker_size_m:=0.10 -p aruco_dict:=DICT_5X5_100

주행 판정 기준
--------------
최종 판정은 실제 절대 pose 공급 노드의 ``cctv_marker_visible`` 을 따른다.
프리뷰의 픽셀 크기·면적·경계 여유·검출 연속성은 불안정 가능성을 설명하는
보조 지표다. 카메라가 바닥에 대해 기울어져 있으면 정사각형 마커도 영상에서는
정상적으로 사다리꼴이 되므로, raw 변 편차가 크다는 이유만으로 불량 처리하지
않는다.

브라우저에서 ``http://<젯슨IP>:5005/``. VSCode 원격 접속 중이라면 PORTS 탭에서
5005를 forward하면 맥에서 ``http://localhost:5005/``로 그대로 열린다.
"""

from __future__ import annotations

import json
import threading
import time

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from cooperative_parking_robot.aruco_utils import ArucoDetectorCompat
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
# 런타임(cctv_merge)이 쓰는 것과 **같은** 점유 판정 로직을 그대로 쓴다.
# 프리뷰가 자체 규칙으로 판정하면 화면과 실제 발행값이 어긋난다.
# 바닥 homography 는 바닥 위의 점만 맞는다. 높이가 있는 점을 되돌리는
# 역변환은 런타임 yolo_bev_map 이 쓰는 것과 **같은 함수**를 쓴다.
from cooperative_parking_robot.vision_utils import correct_floor_projection
from cooperative_parking_robot.bev_fusion_core import (
    SlotOccupancyTracker,
    # 런타임 merge_detections 와 **같은** 중복 판정을 쓰기 위해 가져온다.
    # 프리뷰가 자체 규칙으로 합치면 화면과 실제 발행값이 달라진다.
    _mutual_overlap,
    decode_detection_envelope,
    image_corner_coverage,
    polygon_centroid,
    slot_observability,
)

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from flask import Flask, Response, jsonify, request
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
  .sourcebar{margin-top:8px;font-size:12px;color:#b9c8da;
    background:#111a25;border:1px solid #2c394c;border-radius:6px;padding:7px}
  table{width:100%;border-collapse:collapse;font-size:12px;margin-top:8px}
  th,td{border-bottom:1px solid #2c394c;padding:4px 5px;text-align:right;
    white-space:nowrap}
  #grid-root [id^="det"],#grid-root [id^="mk"]{overflow-x:auto}
  th:first-child,td:first-child{text-align:left}
  th{color:#8fa3ba;font-weight:600}
  .badge{display:inline-block;padding:3px 9px;border-radius:12px;
    background:#263448;font-size:12px}
  .badge.good{background:#1d4030;color:#8ef0b5}
  .badge.bad{background:#43222a;color:#ffb3b3}
  .warn{color:var(--orange)}
  .err{color:var(--red)}
  .ok{color:var(--green)}

  /* ---- 관제탑 ---- */
  #tower{padding:14px 14px 0}
  .tw-row{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));
    gap:12px;margin-bottom:12px}
  .tw-tile{background:var(--panel);border:1px solid var(--line);
    border-radius:10px;padding:12px 14px;border-left:5px solid #46586f}
  .tw-tile.good{border-left-color:var(--green)}
  .tw-tile.warn{border-left-color:var(--orange)}
  .tw-tile.bad{border-left-color:var(--red)}
  .tw-tile .k{font-size:12px;color:#8fa3ba;letter-spacing:.03em}
  .tw-tile .v{font-size:26px;font-weight:650;margin:3px 0 1px;
    line-height:1.15}
  .tw-tile .s{font-size:12px;color:#aebcd0;min-height:16px}
  .tw-panels{display:grid;
    grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:12px}
  .tw-card{background:var(--panel);border:1px solid var(--line);
    border-radius:10px;padding:12px 14px}
  .tw-card h3{margin:0 0 10px;font-size:13px;font-weight:600;
    display:flex;justify-content:space-between;align-items:baseline;gap:8px}
  .tw-sub{font-size:12px;color:#8fa3ba;font-weight:400}
  .tw-slots{display:grid;
    grid-template-columns:repeat(auto-fill,minmax(84px,1fr));gap:8px}
  .slot{border-radius:8px;padding:9px 6px;text-align:center;
    border:1px solid #2c394c;background:#0d141e}
  .slot .id{font-size:15px;font-weight:650}
  .slot .st{font-size:11px;margin-top:2px}
  .slot.free{background:#12301f;border-color:#2b6b45}
  .slot.free .st{color:#8ef0b5}
  .slot.busy{background:#33161c;border-color:#7a3540}
  .slot.busy .st{color:#ffb3b3}
  .slot.unk{background:#1b222c;border-color:#3b4a5e}
  .slot.unk .st{color:#9fb0c4}
  .tw-legend{display:flex;gap:14px;margin-top:10px;font-size:12px;
    color:#9fb0c4;flex-wrap:wrap}
  .tw-legend i{display:inline-block;width:10px;height:10px;border-radius:3px;
    margin-right:5px;vertical-align:-1px}
  .tw-legend i.free{background:#2b6b45}
  .tw-legend i.busy{background:#7a3540}
  .tw-legend i.unk{background:#3b4a5e}
  .tw-chain{display:flex;gap:6px;flex-wrap:wrap;align-items:center}
  .step{border-radius:7px;padding:6px 9px;font-size:12px;background:#0d141e;
    border:1px solid #2c394c;white-space:nowrap}
  .step.good{border-color:#2b6b45;color:#8ef0b5}
  .step.warn{border-color:#7a5a2a;color:#ffd79a}
  .step.bad{border-color:#7a3540;color:#ffb3b3}
  .step .n{display:block;font-size:10px;color:#8fa3ba}
  .arrow{color:#5a6d84;font-size:13px}
  .tw-note{margin-top:9px;font-size:12px;color:#aebcd0;min-height:16px}
  .tw-kv{display:grid;grid-template-columns:auto 1fr;gap:5px 12px;
    font-size:12.5px;align-items:baseline}
  .tw-kv .k{color:#8fa3ba}
  .tw-kv .v{color:#edf3fa}
</style>
</head>
<body>
<header>
  <h1>CCTV 관제탑</h1>
  <div class="controls">
    <label>격자 <input id="grid" type="number" value="100" min="0" step="10">px</label>
    <button onclick="applyGrid()">적용</button>
    <button onclick="clearPoints()">찍은 점 지우기</button>
    <button onclick="resetTrack()">이동 기준점 재설정</button>
    <span id="moved" class="badge">이동 추적 대기…</span>
    <span id="overlap" class="badge">겹침 확인 중…</span>
    <span id="slots" class="badge">슬롯 확인 중…</span>
    <span id="guide" class="badge">안내 대기…</span>
    <span id="drivegate" class="badge">CCTV 주행 입력 확인 중…</span>
    <span class="meta">안내 미션
      <button onclick="setGuide('auto')">자동</button>
      <button onclick="setGuide('park')">입차</button>
      <button onclick="setGuide('retrieve')">출차</button>
    </span>
    <span id="relpose" class="badge">상대 거리 확인 중…</span>
    <span id="yolo" class="badge">YOLO 확인 중…</span>
    <span id="hint" class="meta">영상 클릭 = 픽셀 좌표. ArUco 한 변 <b id="msize">0.24</b> m 기준.</span>
  </div>
</header>
<section id="tower">
  <div class="tw-row" id="twTiles"></div>
  <div class="tw-panels">
    <div class="tw-card">
      <h3><span>주차면 현황</span><span class="tw-sub" id="twSlotSub">확인 중…</span></h3>
      <div class="tw-slots" id="twSlots"></div>
      <div class="tw-legend">
        <span><i class="free"></i>빈자리</span>
        <span><i class="busy"></i>점유</span>
        <span><i class="unk"></i>미관측 — 카메라가 못 본 칸</span>
      </div>
    </div>
    <div class="tw-card">
      <h3><span>CCTV 파이프라인</span><span class="tw-sub" id="twChainSub">—</span></h3>
      <div class="tw-chain" id="twChain"></div>
      <div class="tw-note" id="twChainNote">—</div>
    </div>
    <div class="tw-card">
      <h3><span>로봇 · 미션</span><span class="tw-sub" id="twRobotSub">—</span></h3>
      <div class="tw-kv" id="twRobots"></div>
    </div>
  </div>
</section>
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
    <div id="yolo-region-controls" class="controls" style="margin-bottom:10px">
      <b>YOLO 구역</b>
      <select id="rgcam"></select>
      <span class="meta">를 고르고 <b>합성 BEV 위에서 드래그</b></span>
      <button onclick="clearRegion()">이 구역 지우기</button>
      <button onclick="saveRegions()">저장</button>
      <button id="rgmode" onclick="toggleSwitchMode()"
              title="끔 → 구역 → 미션(입차/출차) 순으로 바뀝니다">전환: 끔</button>
      <span class="meta" id="rgmsg">—</span>
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
      <div class="sourcebar" id="src${i}">검출 소스 확인 중…</div>
      <div id="det${i}"></div>
      <div id="mk${i}"></div>
    </section>`).join('');
  CAMS.forEach((c,i)=>{ points[i]=[]; });
  if(info.bev && info.bev.ready){
    document.getElementById('bevgrid').innerHTML =
      CAMS.map((c,i)=>`<div><div class="meta" style="margin-bottom:5px">
        ${esc(c.label)}</div><img id="bev${i}" src="/bev/${i}"
        style="cursor:default;width:100%;display:block"></div>`).join('')
      + `<div><div class="meta" style="margin-bottom:5px">합성 (정합 확인 · 드래그로 구역 지정)</div>
         <div id="bevwrap" style="position:relative;display:inline-block;width:100%">
           <img id="bevmerged" src="/bev/merged" style="cursor:crosshair;width:100%;display:block">
           <div id="rgbox" style="position:absolute;display:none;border:2px dashed #ffd166;
                background:rgba(255,209,102,.15);pointer-events:none"></div>
         </div></div>`;
    initRegionDrag(info);
    if(info.yolo && !info.yolo.controls_enabled){
      const controls = document.getElementById('yolo-region-controls');
      if(controls) controls.innerHTML =
        '<b>Production 검출 모드</b><span class="meta">카메라 선택과 추론은 '
        + 'Production 노드가 담당하며, 프리뷰는 결과만 표시합니다.</span>';
    }
  } else {
    document.getElementById('bevbox').style.display='none';
  }
  tick();
  setInterval(tick, 700);
}

function twTile(k, v, s, cls){
  return '<div class="tw-tile ' + cls + '"><div class="k">' + k + '</div>'
       + '<div class="v">' + v + '</div><div class="s">' + s + '</div></div>';
}

function twStep(name, label, cls){
  return '<div class="step ' + cls + '"><span class="n">' + name + '</span>'
       + label + '</div>';
}

// 관제탑은 판정을 새로 하지 않는다. /api/info 가 이미 실어 보낸 값을
// 사람이 읽는 순서로 다시 배치할 뿐이다.
function renderTower(info){
  const cams = info.cameras || [];
  const alive = cams.filter(c => c.alive);
  const sl = info.slots || {};
  const items = (sl.ready && sl.items) ? sl.items : [];
  const free = items.filter(x => x.observed && !x.occupied);
  const busy = items.filter(x => x.observed && x.occupied);
  const unk  = items.filter(x => !x.observed);
  const y = info.yolo || {};
  const g = info.guidance || {};
  const rp = info.relative_pose || {};
  const roles = Object.keys(g.robots || {});

  // ---- 상태 타일 ----
  let tiles = '';
  tiles += twTile('카메라 인식',
    alive.length + ' / ' + cams.length,
    alive.length ? alive.map(c => esc(c.label) + ' ' + c.fps.toFixed(0) + 'fps')
                       .join(' · ')
                 : '수신 없음',
    !cams.length ? 'bad' : (alive.length === cams.length ? 'good'
                            : (alive.length ? 'warn' : 'bad')));

  // YOLO 가 죽은 뒤에도 직전 검출이 payload 에 남을 수 있다. 그 숫자를
  // 그대로 띄우면 "잡히고 있다"는 오해를 준다.
  const detN = cams.reduce((s,c) => s + (c.detections || []).length, 0);
  tiles += twTile('차량 검출',
    y.ready ? detN + '대' : '—',
    y.ready ? ('담당 ' + (y.active || '전체')
               + (y.scanning ? ' (스캔 중)' : ''))
            : ('YOLO ' + (y.error || '비활성')),
    y.ready ? (detN ? 'good' : '') : 'bad');

  tiles += twTile('빈자리',
    sl.ready ? (free.length + ' / ' + items.length) : '—',
    !sl.ready ? 'layout 또는 homography 없음'
      : (free.length ? free.map(x => esc(x.id)).join(', ') : '없음')
        + (unk.length ? ' · 미관측 ' + unk.length : ''),
    !sl.ready ? 'bad' : (unk.length ? 'warn' : (free.length ? 'good' : '')));

  const missionKo = {park:'입차', retrieve:'출차'}[g.mission] || null;
  tiles += twTile('미션',
    missionKo || '대기',
    g.distance_m !== null && g.distance_m !== undefined
      ? (esc(g.goal) + '까지 ' + (g.distance_m * 100).toFixed(0) + ' cm')
      : (g.reason || '—'),
    missionKo ? (g.distance_m === null ? 'warn' : 'good') : '');
  document.getElementById('twTiles').innerHTML = tiles;

  // ---- 주차면 현황판 ----
  const box = document.getElementById('twSlots');
  if(!sl.ready){
    box.innerHTML = '<div class="tw-note">슬롯 정보 없음 — '
      + 'parking_layout.yaml 과 homography 를 확인하세요.</div>';
    document.getElementById('twSlotSub').textContent = '준비 안 됨';
  } else {
    box.innerHTML = items.map(x => {
      const cls = !x.observed ? 'unk' : (x.occupied ? 'busy' : 'free');
      const st  = !x.observed ? '미관측' : (x.occupied ? '점유' : '빈자리');
      return '<div class="slot ' + cls + '"><div class="id">' + esc(x.id)
           + '</div><div class="st">' + st + '</div></div>';
    }).join('');
    document.getElementById('twSlotSub').textContent =
      '빈 ' + free.length + ' · 점유 ' + busy.length
      + (unk.length ? ' · 미관측 ' + unk.length : '')
      + ' · 겹침판정 ' + (sl.overlap_threshold * 100).toFixed(0) + '%';
  }

  // ---- 파이프라인 체인 ----
  // 앞 단계가 막히면 뒤 단계 초록은 의미가 없다. 막힌 첫 지점을 알리려고
  // 단계를 순서대로 늘어놓는다.
  const bev = info.bev || {};
  const steps = [];
  steps.push(['카메라',
    alive.length + '/' + cams.length,
    alive.length === cams.length && cams.length ? 'good'
      : (alive.length ? 'warn' : 'bad')]);
  const mismatch = cams.filter(c => c.alive && c.calib_width &&
    (c.width !== c.calib_width || c.height !== c.calib_height));
  steps.push(['해상도',
    mismatch.length ? '불일치 ' + mismatch.length : '일치',
    mismatch.length ? 'warn' : 'good']);
  steps.push(['BEV 정합',
    bev.ready ? (bev.cameras || []).length + '대' : '없음',
    bev.ready ? 'good' : 'bad']);
  steps.push(['YOLO',
    y.ready ? detN + '검출' : '비활성',
    y.ready ? 'good' : 'bad']);
  steps.push(['슬롯 판정',
    sl.ready ? (items.length - unk.length) + '/' + items.length + ' 관측'
             : '없음',
    !sl.ready ? 'bad' : (unk.length ? 'warn' : 'good')]);
  steps.push(['안내',
    (g.distance_m !== null && g.distance_m !== undefined) ? '산출됨' : '대기',
    (g.distance_m !== null && g.distance_m !== undefined) ? 'good'
      : (g.mission ? 'warn' : '')]);
  document.getElementById('twChain').innerHTML = steps
    .map(s => twStep(s[0], s[1], s[2]))
    .join('<span class="arrow">›</span>');

  // 끊긴 것(bad)과 주의(warn)는 말을 다르게 한다. 미관측 한 칸 때문에
  // "막혀 있다"고 하면 실제 정지 상황과 구분이 안 된다.
  const stopped = steps.find(s => s[2] === 'bad');
  const caution = steps.find(s => s[2] === 'warn');
  document.getElementById('twChainNote').innerHTML = stopped
    ? '<span class="err">' + esc(stopped[0]) + ' 단계에서 끊겼습니다.</span>'
      + (g.reason ? ' · ' + esc(g.reason) : '')
    : (caution
        ? '<span class="warn">' + esc(caution[0]) + ' 단계 주의</span>'
          + (g.reason ? ' · ' + esc(g.reason) : '')
        : '<span class="ok">전 구간 정상</span>');
  document.getElementById('twChainSub').textContent =
    (y.switch_mode === 'off' ? '전환 끔'
     : y.switch_mode === 'region' ? '구역 전환'
     : y.switch_mode === 'mission' ? '미션 전환' : y.switch_mode || '');

  // ---- 로봇 · 미션 ----
  let kv = '';
  const ko = {front:'Front', rear:'Rear'};
  ['front','rear'].forEach(role => {
    const p = (g.robots || {})[role];
    kv += '<div class="k">' + (ko[role] || role) + ' 마커</div><div class="v">'
       + (p ? p[0].toFixed(2) + ', ' + p[1].toFixed(2) + ' m'
            : '<span class="err">미검출</span>') + '</div>';
  });
  if(rp.configured){
    kv += '<div class="k">Rear→Front</div><div class="v">'
       + (rp.fresh && rp.visible !== false
          ? (rp.forward_m * 100).toFixed(1) + ' cm · 좌우 '
            + (rp.lateral_m * 100).toFixed(1) + ' cm · '
            + rp.yaw_deg.toFixed(1) + '°'
          : (rp.visible === false ? '<span class="err">마커 미검출</span>'
                                  : '<span class="warn">수신 대기</span>'))
       + '</div>';
  }
  kv += '<div class="k">미션</div><div class="v">'
     + (missionKo || '대기')
     + (g.forced ? ' <span class="warn">(수동 지정)</span>' : '')
     + '</div>';
  if(g.goal){
    kv += '<div class="k">목적지</div><div class="v">' + esc(g.goal) + '</div>';
  }
  if(g.distance_m !== null && g.distance_m !== undefined){
    kv += '<div class="k">남은 거리</div><div class="v">'
       + (g.distance_m * 100).toFixed(0) + ' cm · 방위 '
       + g.heading_deg.toFixed(0) + '°</div>';
  }
  if(g.reason){
    kv += '<div class="k">사유</div><div class="v warn">'
       + esc(g.reason) + '</div>';
  }
  document.getElementById('twRobots').innerHTML = kv;
  document.getElementById('twRobotSub').textContent =
    roles.length + ' / 2 마커';
}

async function tick(){
  let info;
  try { info = await (await fetch('/api/info')).json(); } catch(e){ return; }
  info.cameras.forEach((c,i)=>{
    const el = document.getElementById('meta'+i); if(!el) return;
    const img = document.getElementById('img'+i);
    renderDetectionSource(i, c);
    renderMarkers(i, c.markers || []);
    renderDetections(i, c.detections || []);
    if(!c.alive){
      el.innerHTML = '<span class="err">수신 없음</span> · ' + esc(c.topic);
      if(img) img.classList.add('dead');
      return;
    }
    if(img) img.classList.remove('dead');
    let size = c.width + '×' + c.height;
    if(c.detection_source === 'production' && c.detection_live){
      size += ' · 검출 ' + c.detection_rate_hz.toFixed(1) + 'Hz';
    } else if(c.infer_ms) size += ' · yolo ' + c.infer_ms.toFixed(0) + 'ms';
    if(c.held) size += ' · <span class="warn">직전 검출 유지</span>';
    let cls = 'ok';
    let note = '';
    if(c.calib_width &&
       (c.width !== c.calib_width || c.height !== c.calib_height)){
      cls = 'warn';
      note = ' · <span class="warn">캘리브레이션 '
        + c.calib_width + '×' + c.calib_height + '와 불일치</span>';
    }
    el.innerHTML = '<span class="'+cls+'">'+size+'</span> · '
      + c.fps.toFixed(1) + ' fps · ' + esc(c.topic) + note;
  });

  renderDriveGate(info.drive_markers || []);

  document.getElementById('msize').textContent = info.marker_size_m;

  const mb = document.getElementById('moved');
  if(mb){
    const rows = Object.entries(info.tracks || {})
      .filter(([,t]) => t.last)
      .map(([lab,t]) => `${esc(lab)} 이동 ${(t.moved_m*100).toFixed(1)}cm`
                        + ` / 경로 ${(t.path_m*100).toFixed(1)}cm`);
    if(rows.length){ mb.className='badge good'; mb.innerHTML = rows.join(' · '); }
    else { mb.className='badge'; mb.textContent='이동 추적 대기 — 차량 미검출'; }
  }

  const rp = info.relative_pose, rpb = document.getElementById('relpose');
  if(rpb && rp){
    if(rp.fresh && rp.visible === true){
      rpb.className = 'badge good';
      rpb.textContent = 'ID0 raw 카메라→마커 ' + (rp.forward_m*100).toFixed(1) + ' cm'
        + ' · 좌우 ' + (rp.lateral_m*100).toFixed(1) + ' cm'
        + ' · 틀어짐 ' + rp.yaw_deg.toFixed(1) + '°';
    } else if(rp.visible === false){
      rpb.className = 'badge bad';
      rpb.textContent = '상대 pose: 마커 미검출';
    } else if(!rp.configured){
      rpb.className = 'badge';
      rpb.textContent = '상대 pose: 토픽 미설정';
    } else {
      rpb.className = 'badge bad';
      rpb.textContent = '상대 pose 대기 중';
    }
  }

  const y = info.yolo, yb = document.getElementById('yolo');
  if(y && yb){
    if(y.ready){
      const n = info.cameras.reduce((s,c)=>s+(c.detections||[]).length,0);
      yb.className = 'badge good';
      let t = y.source === 'production'
        ? 'Production 차량 검출 ' + n + '개 · 센서 '
          + y.live_cameras.length + '/' + info.cameras.length
        : 'YOLO 검출 ' + n + '개 (클래스 ' + y.classes.join(',') + ')';
      if(y.switch_mode === 'region' || y.switch_mode === 'mission'){
        t += y.active
           ? ' · 담당 ' + y.active + (y.scanning ? ' (스캔 중)' : '')
           : ' · 담당 없음';
      }
      if(y.switch_mode === 'mission'){
        const ko = {park:'입차', retrieve:'출차'}[y.mission_type];
        t += ' · 미션 ' + (!y.mission_fresh ? 'fleet 수신 없음'
                          : (ko || '대기'));
      }
      yb.textContent = t;
    } else {
      yb.className = 'badge bad';
      yb.textContent = 'YOLO: ' + (y.error || '비활성');
    }
  }

  try { renderTower(info); } catch(e){ /* 관제탑 실패가 영상을 막지 않게 */ }

  if((++BEVTICK % 3) === 0) refreshBev();

  const sl = info.slots, sb = document.getElementById('slots');
  if(sb){
    if(!sl || !sl.ready){
      sb.className = 'badge';
      sb.textContent = '슬롯: layout 또는 homography 없음';
    } else {
      const free = sl.items.filter(x => x.observed && !x.occupied);
      const busy = sl.items.filter(x => x.observed && x.occupied);
      const unk  = sl.items.filter(x => !x.observed);
      // 미관측이 있으면 '빈자리 N개'만 보여주는 것은 위험하다. 못 본 칸이
      // 몇 개인지 같이 띄운다.
      sb.className = unk.length ? 'badge warn' : 'badge good';
      sb.innerHTML = '빈자리 <b>' + free.length + '</b>'
        + (free.length ? ' (' + free.map(x => x.id).join(', ') + ')' : '')
        + ' · 점유 ' + busy.length
        + (unk.length ? ' · <span class="warn">미관측 ' + unk.length
                        + ' (' + unk.map(x => x.id).join(', ') + ')</span>' : '');
    }
  }

  const g = info.guidance, gb = document.getElementById('guide');
  if(gb){
    if(!g || !g.mission){
      gb.className = 'badge';
      gb.textContent = '안내: ' + ((g && g.reason) || '미션 없음');
    } else {
      const ko = {park:'입차', retrieve:'출차'}[g.mission] || g.mission;
      const roles = Object.keys(g.robots || {});
      if(g.distance_m === null){
        gb.className = 'badge warn';
        gb.textContent = ko + ': ' + (g.reason || '목적지 미정');
      } else {
        gb.className = 'badge good';
        gb.innerHTML = ko + ' → <b>' + esc(g.goal) + '</b> '
          + g.distance_m.toFixed(2) + ' m · ' + g.heading_deg.toFixed(0) + '°'
          + ' · 로봇 ' + roles.length + '대'
          + (g.forced ? ' <span class="warn">(수동 고정)</span>' : '')
          + (g.reason ? ' <span class="warn">· ' + esc(g.reason) + '</span>' : '');
      }
    }
  }

  const modeBtn = document.getElementById('rgmode');
  if(modeBtn && info.yolo){
    const m = info.yolo.switch_mode;
    modeBtn.textContent = '전환: ' + ({off:'끔', region:'구역', mission:'미션'}[m] || m);
    modeBtn.classList.toggle('active', m !== 'off');
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

let BEVPPM = 0;
let BEVW = 0, BEVH = 0;   // BEV 원본 픽셀 크기 (map_w*ppm, map_h*ppm)
let BEVTICK = 0;
let DRAGGING = false;

// BEV 는 MJPEG 스트림이 아니라 한 장짜리 JPEG 다. 구역을 바꿔도 다시
// 받아오지 않으면 화면에 안 보여서, 바꾼 직후와 몇 초마다 새로 받는다.
function refreshBev(){
  // 드래그 중에 src 를 갈면 그 순간 이미지 크기가 0 이 된다.
  if(DRAGGING) return;
  const stamp = Date.now();
  document.querySelectorAll('#bevgrid img').forEach(img => {
    const base = img.src.split('?')[0];
    img.src = base + '?t=' + stamp;
  });
}

function initRegionDrag(info){
  BEVPPM = (info.bev && info.bev.ppm) || 0;
  // img.naturalWidth 는 src 가 바뀌는 순간 0 으로 떨어진다. 그걸 쓰면
  // 새로고침과 겹친 드래그가 0x0 으로 계산된다. 크기는 상수이므로
  // /api/info 에서 받아 고정한다.
  BEVW = Math.round((info.bev.map_w || 0) * BEVPPM);
  BEVH = Math.round((info.bev.map_h || 0) * BEVPPM);
  const sel = document.getElementById('rgcam');
  const labels = (info.yolo && info.yolo.labels) || [];
  sel.innerHTML = labels.map(l => `<option>${esc(l)}</option>`).join('');
  const img = document.getElementById('bevmerged');
  const box = document.getElementById('rgbox');
  if(!img || !box || !BEVPPM) return;

  let start = null;
  const local = ev => {
    const r = img.getBoundingClientRect();
    return {cx: ev.clientX - r.left, cy: ev.clientY - r.top,
            sx: BEVW / r.width, sy: BEVH / r.height};
  };
  const toMetre = p => ({
    x: p.cx * p.sx / BEVPPM,
    y: (BEVH - 1 - p.cy * p.sy) / BEVPPM,
  });
  const fmt = (a, b) =>
    'x ' + Math.min(a.x, b.x).toFixed(2) + '~' + Math.max(a.x, b.x).toFixed(2)
    + '  y ' + Math.min(a.y, b.y).toFixed(2) + '~' + Math.max(a.y, b.y).toFixed(2)
    + ' m';

  img.addEventListener('mousedown', ev => {
    ev.preventDefault();
    if(!BEVW || !BEVH){ setMsg('BEV 크기를 못 읽었습니다 (새로고침해 보세요)', false); return; }
    DRAGGING = true;
    start = local(ev);
    box.style.display = 'block';
    box.style.left = start.cx + 'px'; box.style.top = start.cy + 'px';
    box.style.width = '0px'; box.style.height = '0px';
  });
  window.addEventListener('mousemove', ev => {
    if(!start) return;
    const now = local(ev);
    box.style.left = Math.min(start.cx, now.cx) + 'px';
    box.style.top = Math.min(start.cy, now.cy) + 'px';
    box.style.width = Math.abs(now.cx - start.cx) + 'px';
    box.style.height = Math.abs(now.cy - start.cy) + 'px';
    // 끄는 동안 실제 미터값을 보여준다. 숫자가 0 근처면 좌표 변환이
    // 깨진 것이고, 정상이면 원하는 크기까지 보면서 끌 수 있다.
    setMsg(fmt(toMetre(start), toMetre(now)), true);
  });
  window.addEventListener('mouseup', async ev => {
    if(!start) return;
    const a = toMetre(start), b = toMetre(local(ev));
    start = null; DRAGGING = false; box.style.display = 'none';
    const label = document.getElementById('rgcam').value;
    if(!label) return;
    const res = await fetch('/api/yolo_region/' + encodeURIComponent(label), {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({x1:a.x, y1:a.y, x2:b.x, y2:b.y})});
    const data = await res.json();
    setMsg(res.ok ? '지정: ' + data.csv : data.error, res.ok);
    if(res.ok) refreshBev();
  });
}

function setMsg(text, good){
  const el = document.getElementById('rgmsg');
  if(!el) return;
  el.className = good ? 'meta ok' : 'meta err';
  el.textContent = text;
}

async function clearRegion(){
  const label = document.getElementById('rgcam').value;
  if(!label) return;
  const res = await fetch('/api/yolo_region_clear/' + encodeURIComponent(label),
                          {method:'POST'});
  const data = await res.json();
  setMsg('남은 구역: ' + (data.csv || '없음'), true);
  refreshBev();
}

async function saveRegions(){
  const res = await fetch('/api/yolo_region_save', {method:'POST'});
  const data = await res.json();
  setMsg(res.ok ? '저장됨 → ' + data.path : data.error, res.ok);
}

async function toggleSwitchMode(){
  const info = await (await fetch('/api/info')).json();
  const order = ['off', 'region', 'mission'];
  const at = order.indexOf(info.yolo ? info.yolo.switch_mode : 'off');
  const next = order[(at + 1) % order.length];
  const res = await fetch('/api/yolo_switch_mode/' + next, {method:'POST'});
  const data = await res.json();
  setMsg(res.ok ? '구역 전환: ' + data.switch_mode : data.error, res.ok);
  if(res.ok) refreshBev();
}

async function setGuide(mission){
  const res = await fetch('/api/guidance_mission/' + mission, {method:'POST'});
  const data = await res.json();
  setMsg(res.ok ? ('안내 미션: ' + (data.forced || '자동')) : data.error, res.ok);
  refreshBev();
}

async function resetTrack(){
  await fetch('/api/track_reset', {method:'POST'});
}

async function setBev(mode){
  await fetch('/api/bev_mode/'+mode, {method:'POST'});
  const img=document.getElementById('bevmerged');
  if(img) img.src='/bev/merged?t='+Date.now();
}

function renderDriveGate(markers){
  const box = document.getElementById('drivegate');
  if(!box) return;
  if(!markers || !markers.length){
    box.className = 'badge';
    box.textContent = '주행 마커: 설정 없음';
    return;
  }
  const allReady = markers.every(m => m.drive_ready === true);
  const anyBlocked = markers.some(m => m.drive_ready === false);
  box.className = 'badge ' + (allReady ? 'good' : anyBlocked ? 'bad' : '');
  box.title = markers.map(m => `${m.role} ID${m.id}: ${m.reason}`).join('\n');
  box.textContent = 'CCTV 주행 입력 · ' + markers.map(m =>
    `${m.role} ID${m.id} ${m.status}`).join(' · ');
}

function renderDetectionSource(i, c){
  const box = document.getElementById('src'+i);
  if(!box) return;
  if(c.detection_source === 'production'){
    const live = c.detection_live;
    const age = c.source_age_s == null ? '—' : c.source_age_s.toFixed(2)+'s';
    const transport = c.transport_age_s == null
      ? '—' : c.transport_age_s.toFixed(3)+'s';
    const hid = c.homography_ok === true
      ? '<span class="ok">H 정상</span>'
      : c.homography_ok === false
        ? '<span class="err">H 오류</span>' : 'H 대기';
    let text = '<b class="'+(live?'ok':'err')+'">Production '
      + (live?'수신 중':'수신 대기') + '</b>'
      + ' · ' + esc(c.detection_topic)
      + ' · sensor ' + esc(c.detection_camera_id || '—')
      + ' · ' + c.detection_rate_hz.toFixed(1) + ' Hz'
      + ' · 데이터 age ' + age + ' (전송 ' + transport + ')'
      + ' · seq ' + c.detection_sequence
      + ' · msg ' + c.detection_messages
      + ' · ' + hid;
    if(c.detection_invalid || c.detection_dropped){
      text += ' · <span class="warn">오류 ' + c.detection_invalid
        + ' / 순서폐기 ' + c.detection_dropped + '</span>';
    }
    if(c.detection_error){
      text += ' · <span class="err">' + esc(c.detection_error) + '</span>';
    }
    box.innerHTML = text;
  } else if(c.detection_source === 'internal'){
    const age = c.infer_age_s == null ? '—' : c.infer_age_s.toFixed(2)+'s';
    box.innerHTML = '<b>프리뷰 내장 YOLO</b> · 마지막 추론 '+age;
  } else {
    box.innerHTML = '<span class="warn">차량 검출 비활성</span>';
  }
}

function renderDetections(i, dets){
  const box = document.getElementById('det'+i);
  if(!box) return;
  if(!dets.length){ box.innerHTML = ''; return; }
  box.innerHTML = '<table><tr><th>검출</th><th>신뢰도</th><th>중심</th>'
    + '<th>World X</th><th>World Y</th>'
    + '<th>길이×폭</th><th>Yaw</th><th>광축거리</th>'
    + '<th>영역</th><th>분류/휠베이스</th><th>이동 (cm)</th></tr>'
    + dets.map(d => {
        const w = d.world, g = d.geometry;
        const size = (d.length_m != null && d.width_m != null)
          ? `${d.length_m.toFixed(2)}×${d.width_m.toFixed(2)}` : '—';
        const angle = d.yaw_deg != null ? d.yaw_deg : (g ? g.angle_deg : null);
        const ang = angle != null ? `${angle.toFixed(1)}°` : '—';
        const axis = d.axis_dist_m != null ? `${d.axis_dist_m.toFixed(2)}m` : '—';
        const zone = d.in_waiting === true
          ? '<span class="warn">WAIT</span>' : '일반';
        const kind = esc(d.vehicle_class || '—')
          + (d.classified_wheelbase_m != null
             ? ` / ${d.classified_wheelbase_m.toFixed(2)}m` : '');
        const mv = (d.moved_m != null)
          ? `<b>${(d.moved_m*100).toFixed(1)}</b> / ${(d.path_m*100).toFixed(1)}`
          : '—';
        const dup = (d.merged_count > 1)
          ? ` <span class="warn">(중복 ${d.merged_count}개 합침)</span>` : '';
        return `<tr><td>${esc(d.name)}${dup}</td>`
          + `<td>${d.confidence.toFixed(2)}</td>`
          + `<td class="${d.center_source==='mask'?'ok':'warn'}">${esc(d.center_source)}</td>`
          + (w ? `<td class="ok">${w[0].toFixed(3)}</td><td class="ok">${w[1].toFixed(3)}</td>`
               : `<td colspan="2" class="warn">H 없음</td>`)
          + `<td>${size}</td><td>${ang}</td><td>${axis}</td>`
          + `<td>${zone}</td><td>${kind}</td><td>${mv}</td>`
          + '</tr>';
      }).join('') + '</table>'
    + '<div class="meta" style="margin-top:4px">이동 = 기준점 대비 직선거리 / '
    + '경로 = 누적 이동거리. Production 중심/윤곽은 실제 검출 노드의 '
    + 'map 좌표를 영상에 역투영한 값입니다.</div>';
}

function renderMarkers(i, markers){
  const box = document.getElementById('mk'+i);
  if(!box) return;
  if(!markers.length){ box.innerHTML =
    '<div class="meta" style="margin-top:8px">ArUco 미검출</div>'; return; }
  box.innerHTML = '<table><tr><th>ID/역할</th><th>중심 px</th><th>World (m)</th>'
    + '<th>최소 변</th><th>최근 검출</th><th>경계 여유</th>'
    + '<th title="카메라 원근 투영으로 커질 수 있으며 주행 불량 기준이 아닙니다">원근 편차*</th>'
    + '<th title="원근 투영 참고값">대각비/각도*</th><th>mm/px</th>'
    + '<th>주행 입력 상태</th><th>이유</th></tr>'
    + markers.map(m => {
        const cls = m.drive_class || 'warn';
        const w = m.world;
        const history = m.history_samples
          ? `${m.history_hits}/${m.history_samples} (${(m.detection_ratio*100).toFixed(0)}%)`
          : '측정 중';
        const role = m.role ? `<span class="meta">${esc(m.role)}</span>` : '';
        return `<tr><td>${m.id} ${role}</td>`
          + `<td>${m.center[0].toFixed(0)}, ${m.center[1].toFixed(0)}</td>`
          + (w ? `<td class="ok">${w[0].toFixed(2)}, ${w[1].toFixed(2)}</td>`
               : `<td class="warn">—</td>`)
          + `<td>${m.min_side_px.toFixed(1)} px</td>`
          + `<td>${history}</td>`
          + `<td>${m.edge_margin_px.toFixed(0)} px</td>`
          + `<td>${(m.side_spread*100).toFixed(1)}%</td>`
          + `<td>${m.diagonal_ratio.toFixed(3)} / ${m.max_angle_error_deg.toFixed(1)}°</td>`
          + `<td>${m.mm_per_px.toFixed(2)}</td>`
          + `<td class="${cls}"><b>${esc(m.drive_status || '확인 중')}</b></td>`
          + `<td class="${cls}" style="text-align:left;white-space:normal;min-width:260px">`
          + `${esc(m.drive_reason || '')}</td></tr>`;
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


def camera_ids_match(label, camera_id):
    """프리뷰 라벨과 production envelope의 카메라 ID를 비교한다.

    영상 토픽은 보통 ``/cctv0/image_rect``라서 라벨이 ``cctv0``이고,
    검출 노드는 같은 장비를 ``cam0``으로 싣는다. 숫자 suffix가 같은
    ``camN``/``cctvN``은 같은 카메라로 인정하되 다른 이름은 엄격히 막는다.
    """
    left = _camera_key(label).strip().lower()
    right = _camera_key(camera_id).strip().lower()
    if left == right:
        return True

    def canonical(value):
        for prefix in ('cctv', 'cam'):
            if value.startswith(prefix) and value[len(prefix):].isdigit():
                return 'camera-' + value[len(prefix):]
        return value

    return canonical(left) == canonical(right)


def parse_detection_topics(text, labels):
    """외부 검출 토픽 CSV를 카메라 라벨에 1:1로 연결한다.

    빈 문자열은 내장 YOLO/검출 끔 모드다. 값을 주면 개수가 영상 라벨과
    정확히 같아야 한다. 암묵적으로 첫 토픽을 모든 카메라에 재사용하면
    cam0 검출이 cam2 영상에 그려지는 위험한 오진 화면이 되기 때문이다.
    """
    raw = str(text).strip()
    if not raw:
        return {}
    topics = [topic.strip() for topic in raw.split(',')]
    if any(not topic for topic in topics):
        raise ValueError('detection_topics_csv 에 빈 토픽이 있습니다')
    if len(topics) != len(labels):
        raise ValueError(
            'detection_topics_csv 길이가 image_topics_csv와 다릅니다: '
            f'{len(topics)} != {len(labels)}')
    if len(set(topics)) != len(topics):
        raise ValueError('detection_topics_csv 토픽은 서로 달라야 합니다')
    return dict(zip(labels, topics))


def production_detection_item(detection):
    """``CameraDetection``을 프리뷰/웹에서 쓰는 JSON-safe dict로 바꾼다."""
    yaw = detection.yaw
    yaw_deg = None if yaw is None else math.degrees(float(yaw))
    polygon = detection.polygon
    return {
        'name': 'vehicle',
        'source': 'production',
        'camera_id': str(detection.camera_id),
        'confidence': round(float(detection.confidence), 3),
        'center_source': 'production',
        'world': [round(float(detection.center[0]), 3),
                  round(float(detection.center[1]), 3)],
        'world_polygon': (
            None if polygon is None
            else [[round(float(point[0]), 3), round(float(point[1]), 3)]
                  for point in polygon]),
        'yaw_deg': None if yaw_deg is None else round(yaw_deg, 2),
        'length_m': detection.length_m,
        'width_m': detection.width_m,
        'in_waiting': bool(detection.in_waiting),
        'axis_dist_m': round(float(detection.axis_dist_m), 3),
        'vehicle_class': detection.vehicle_class,
        'classified_wheelbase_m': detection.classified_wheelbase_m,
        'merged_count': 1,
    }


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


def mask_center_geometry(mask_polygon):
    """차량 segmentation mask에서 중심선과 중심점을 뽑는다.

    구하는 순서
    -----------
    1. mask 외곽을 감싸는 **최소 회전 사각형**(minAreaRect)을 구한다.
       축 정렬 bbox 를 쓰면 차량이 비스듬할 때 실제보다 크게 잡혀서
       중심이 밀린다. 회전 사각형은 차량 자세를 따라간다.
    2. 그 사각형의 **네 변 중점**을 구한다.
    3. 마주 보는 중점끼리 이으면 **중심선 두 개**가 나온다.
    4. 두 중심선의 **교점이 중심점**이다.

    직사각형에서 두 중심선의 교점은 수학적으로 사각형 중심과 같지만,
    화면에 중점과 중심선을 같이 그리면 차량 자세가 제대로 잡혔는지
    눈으로 바로 확인할 수 있다.

    반환: ``None`` 또는
    ``{'corners':[4x2], 'edge_midpoints':[4x2], 'center':[x,y],
       'axes':[[p,q],[p,q]], 'length_px':, 'width_px':, 'angle_deg':}``
    ``edge_midpoints``는 [장축쪽, 단축쪽, 장축쪽, 단축쪽] 순서로,
    ``axes[0]``이 차량 **길이 방향** 중심선이다.
    """
    if mask_polygon is None or len(mask_polygon) < 3:
        return None
    points = np.asarray(mask_polygon, dtype=np.float32).reshape(-1, 2)
    if not np.all(np.isfinite(points)):
        return None
    rect = cv2.minAreaRect(points)
    (cx, cy), (w, h), angle = rect
    if not all(math.isfinite(v) for v in (cx, cy, w, h, angle)):
        return None
    if w < 1e-3 or h < 1e-3:
        return None
    corners = cv2.boxPoints(rect).astype(np.float64)

    # 네 변의 중점. boxPoints 는 사각형 둘레를 순서대로 돌므로
    # i번째 변은 corners[i] -> corners[i+1] 이다.
    midpoints = [((corners[i] + corners[(i + 1) % 4]) / 2.0).tolist()
                 for i in range(4)]

    # 마주 보는 중점끼리 이은 두 중심선
    axis_a = [midpoints[0], midpoints[2]]
    axis_b = [midpoints[1], midpoints[3]]

    def seg_len(pair):
        return math.dist(pair[0], pair[1])

    # 긴 쪽이 차량 길이 방향이다. 그 축을 먼저 둔다.
    if seg_len(axis_a) < seg_len(axis_b):
        axis_a, axis_b = axis_b, axis_a
        midpoints = [midpoints[1], midpoints[2], midpoints[3], midpoints[0]]

    length_px = seg_len(axis_a)
    width_px = seg_len(axis_b)
    heading = math.degrees(math.atan2(axis_a[1][1] - axis_a[0][1],
                                      axis_a[1][0] - axis_a[0][0]))
    # 장축은 방향이 180도 뒤집혀도 같은 축이다. -90~90 으로 정규화한다.
    while heading > 90.0:
        heading -= 180.0
    while heading < -90.0:
        heading += 180.0

    return {
        'corners': [[round(float(x), 1), round(float(y), 1)]
                    for x, y in corners],
        'edge_midpoints': [[round(float(x), 1), round(float(y), 1)]
                           for x, y in midpoints],
        'center': [round(float(cx), 1), round(float(cy), 1)],
        'axes': [[[round(float(v), 1) for v in pt] for pt in axis_a],
                 [[round(float(v), 1) for v in pt] for pt in axis_b]],
        'length_px': round(length_px, 1),
        'width_px': round(width_px, 1),
        'angle_deg': round(heading, 1),
    }


def marker_metrics(corners, marker_size_m, frame_width=None,
                   frame_height=None):
    """마커의 픽셀 크기와 원근 투영 참고값을 계산한다.

    기울어진 카메라에서 바닥과 평행한 정사각형은 정상이어도 사다리꼴로
    투영된다. 따라서 ``side_spread``·``diagonal_ratio``·각도 오차는 화면의
    원근 정도를 설명할 뿐 주행 불량 gate가 아니다. 실제 검출 안정성은
    최소 변, 픽셀 면적, 화면 경계 여유와 검출 연속성으로 따로 판단한다.
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
    area_px = abs(sum(
        points[i][0] * points[(i + 1) % 4][1]
        - points[(i + 1) % 4][0] * points[i][1]
        for i in range(4))) / 2.0
    edge_margin_px = None
    if frame_width is not None and frame_height is not None:
        width = float(frame_width)
        height = float(frame_height)
        if width <= 0.0 or height <= 0.0:
            raise ValueError('frame dimensions must be positive')
        edge_margin_px = min(
            min(x, width - 1.0 - x, y, height - 1.0 - y)
            for x, y in points)

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
        'min_side_px': round(min(sides), 1),
        'area_px': round(area_px, 1),
        'edge_margin_px': (None if edge_margin_px is None
                           else round(edge_margin_px, 1)),
        # 변 길이 편차 — 사다리꼴로 찌그러진 정도
        'side_spread': round((max(sides) - min(sides)) / mean_side, 4),
        # 두 대각선 길이 비 — 1.0이면 완전한 정사각형/직사각형
        'diagonal_ratio': round(max(diagonals) / max(1e-6, min(diagonals)), 4),
        # 90도에서 가장 많이 벗어난 꼭짓점
        'max_angle_error_deg': round(max(abs(a - 90.0) for a in angles), 2),
        'mm_per_px': round(size * 1000.0 / mean_side, 3),
    }


def marker_readiness(marker, role='', production_visible=None,
                     production_fresh=False):
    """프리뷰 지표와 실제 Production gate를 사람이 읽을 문장으로 합친다.

    최종 주행 가능 여부는 ``cctv_robot_marker_node``가 발행하는
    ``marker_visible``이 결정한다. 픽셀 지표는 TRUE 상태가 간헐적이거나
    곧 끊길 가능성을 설명하는 보조 진단이다.
    """
    if not role:
        return {
            'drive_status': '참고용 ID',
            'drive_class': '',
            'drive_ready': None,
            'drive_reason': '주행 마커 ID 목록에 없는 진단용 마커',
        }

    severe = []
    warnings = []
    observations = []
    if marker is None:
        observations.append('프리뷰에서는 현재 미검출')
    else:
        minimum = float(marker.get('min_side_px') or 0.0)
        area = float(marker.get('area_px') or 0.0)
        margin = marker.get('edge_margin_px')
        samples = int(marker.get('history_samples') or 0)
        hits = int(marker.get('history_hits') or 0)
        ratio = float(marker.get('detection_ratio') or 0.0)

        observations.append(f'최소 변 {minimum:.0f}px')
        observations.append(f'면적 {area:.0f}px²')
        if margin is not None:
            observations.append(f'경계 여유 {float(margin):.0f}px')
        if samples:
            observations.append(
                f'최근 검출 {hits}/{samples}회({ratio * 100.0:.0f}%)')

        # Production의 기본 면적 gate는 100px²다. 최소 변/경계/연속성은
        # 프리뷰가 추가로 알려 주는 보수적인 안정성 지표다.
        if area < 100.0:
            severe.append('Production 최소 면적 100px² 미달')
        elif area < 200.0:
            warnings.append('마커 면적이 작음')
        if minimum < 12.0:
            severe.append('최소 변 12px 미만')
        elif minimum < 20.0:
            warnings.append('최소 변이 작음')
        if margin is not None:
            if float(margin) < 4.0:
                severe.append('화면 경계에 붙음')
            elif float(margin) < 10.0:
                warnings.append('화면 경계 여유가 작음')
        if samples >= 6:
            if ratio < 0.60:
                severe.append('최근 검출이 자주 끊김')
            elif ratio < 0.85:
                warnings.append('최근 검출이 간헐적임')

    detail = ' · '.join(observations)
    if not production_fresh:
        reason = (f'/{role}/cctv_marker_visible 수신 없음 — 실제 주행 gate를 '
                  '확인할 수 없음')
        if detail:
            reason += f' · {detail}'
        return {
            'drive_status': 'Production 미수신',
            'drive_class': 'warn',
            'drive_ready': None,
            'drive_reason': reason,
        }
    if production_visible is not True:
        reason = ('Production marker_visible=false — 현재 절대 pose를 '
                  '주행에 공급하지 않음')
        if severe or warnings:
            reason += ' · ' + ' · '.join(severe + warnings)
        elif detail:
            reason += f' · {detail}'
        return {
            'drive_status': 'CCTV pose 입력 중단',
            'drive_class': 'err',
            'drive_ready': False,
            'drive_reason': reason,
        }

    issues = severe + warnings
    if issues:
        reason = 'Production marker_visible=true · ' + ' · '.join(issues)
        if detail:
            reason += f' · {detail}'
        return {
            'drive_status': 'CCTV pose 입력 정상(주의)',
            'drive_class': 'warn',
            'drive_ready': True,
            'drive_reason': reason,
        }
    return {
        'drive_status': 'CCTV pose 입력 정상',
        'drive_class': 'ok',
        'drive_ready': True,
        'drive_reason': ('Production marker_visible=true'
                         + (f' · {detail}' if detail else '')),
    }


# 구역 사각형 색 (BGR). 카메라 순서대로 돌려 쓴다.
REGION_COLOURS = [(255, 220, 90), (90, 120, 255), (120, 255, 120)]

# 로봇 안내 화살표 색 (BGR)
GUIDANCE_COLOUR = (0, 215, 255)      # 주황 — 이동해야 할 방향
ROBOT_AXIS_COLOUR = (255, 160, 60)   # 하늘 — 두 로봇을 잇는 축

ROBOT_ROLES = ('front', 'rear')


def box_iou(a, b):
    """픽셀 bbox 두 개의 IoU. world 좌표가 없을 때의 대비책."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_w = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    inter_h = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = inter_w * inter_h
    if intersection <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return 0.0 if union <= 0.0 else intersection / union


def dedupe_detections(detections, center_gate_m=0.35, overlap_ratio=0.30):
    """같은 차량이 여러 번 잡힌 것을 하나로 합친다.

    NMS 를 통과하고도 남는 중복이 있다. segmentation 모델이 한 차량을 앞뒤로
    나눠 잡거나, 신뢰도 문턱을 낮추면 특히 자주 생긴다.

    판정 규칙은 런타임 ``merge_detections`` 와 같다.

      * 월드 중심거리가 ``center_gate_m`` 이내이거나
      * 폴리곤 상호 겹침률이 ``overlap_ratio`` 이상이면

    같은 차량으로 본다. 실물 크기상 차량 두 대의 중심이 35 cm 안에 들어올 수
    없다는 전제다. 월드 좌표가 없으면(homography 미등록) 픽셀 bbox IoU 로
    대신 판단한다 — 없는 것보다는 낫다.

    남기는 쪽은 **신뢰도가 높은 것**이고, 몇 개가 합쳐졌는지 ``merged_count``
    에 남긴다. 합친 사실을 숨기면 모델이 잘 맞추는 것처럼 보인다.
    """
    gate = float(center_gate_m)
    threshold = float(overlap_ratio)
    if gate < 0.0:
        raise ValueError('center_gate_m must be non-negative')
    if not 0.0 <= threshold <= 1.0:
        raise ValueError('overlap_ratio must be in [0,1]')

    kept = []
    for item in sorted(detections, key=lambda d: -d['confidence']):
        duplicate_of = None
        for chosen in kept:
            here, there = item.get('world'), chosen.get('world')
            if (here is not None and there is not None
                    and math.dist(here, there) <= gate):
                duplicate_of = chosen
                break
            mine, other = item.get('world_polygon'), chosen.get('world_polygon')
            if mine and other and _mutual_overlap(other, mine) >= threshold:
                duplicate_of = chosen
                break
            if (here is None or there is None) and threshold > 0.0:
                if box_iou(item['box'], chosen['box']) >= threshold:
                    duplicate_of = chosen
                    break
        if duplicate_of is None:
            item['merged_count'] = 1
            kept.append(item)
        else:
            duplicate_of['merged_count'] = duplicate_of.get(
                'merged_count', 1) + 1
    return kept


def parse_camera_optics(text):
    """``'cctv0:2.463,1.982,2.610; cctv2:...'`` 를 광학 정보 표로 바꾼다.

    값은 순서대로 광축 지상점 X(m), Y(m), 카메라 높이(m) 다. 광축 지상점은
    렌즈에서 추를 내렸을 때 바닥에 닿는 점이며, homography 와 **같은 map
    좌표계**여야 한다.
    """
    optics = {}
    for chunk in str(text or '').replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ':' not in chunk:
            raise ValueError(
                f"camera_optics_csv 항목은 'label:x,y,height' 형식이어야 "
                f"합니다: {chunk!r}")
        label, values = chunk.split(':', 1)
        label = label.strip()
        if not label:
            raise ValueError(f'camera_optics_csv 라벨이 비어 있습니다: {chunk!r}')
        parts = [p.strip() for p in values.split(',') if p.strip()]
        if len(parts) != 3:
            raise ValueError(
                f'{label}: x,y,height 세 값이 필요합니다 (받은 값 {len(parts)}개)')
        try:
            x, y, height = (float(p) for p in parts)
        except ValueError as exc:
            raise ValueError(f'{label}: 숫자로 읽을 수 없습니다: {values!r}') from exc
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(height)):
            raise ValueError(f'{label}: NaN/Inf 는 쓸 수 없습니다')
        if height <= 0.0:
            raise ValueError(f'{label}: 카메라 높이는 0보다 커야 합니다')
        optics[label] = (x, y, height)
    return optics


def parse_robot_markers(text):
    """``'front:2, rear:1'`` 을 ``{역할: 마커ID}`` 로 바꾼다."""
    mapping = {}
    for chunk in str(text).replace('\n', ',').replace(';', ',').split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ':' not in chunk:
            raise ValueError(
                "robot_marker_ids_csv 는 'front:2, rear:1' 형식입니다: "
                f'{chunk!r}')
        role, _, number = chunk.partition(':')
        role = role.strip().lower()
        if role not in ROBOT_ROLES:
            raise ValueError(f"역할은 'front' 또는 'rear' 여야 합니다: {role!r}")
        try:
            marker_id = int(number.strip())
        except ValueError as exc:
            raise ValueError(
                f'마커 ID 가 정수가 아닙니다: {chunk!r}') from exc
        if marker_id < 0:
            raise ValueError(f'마커 ID 는 0 이상이어야 합니다: {marker_id}')
        mapping[role] = marker_id
    if len(set(mapping.values())) != len(mapping):
        raise ValueError(
            f'front 와 rear 에 같은 마커 ID 를 줄 수 없습니다: {mapping}')
    return mapping


# 슬롯 상태별 색 (BGR)
SLOT_FREE_COLOUR = (80, 230, 100)       # 초록 — 빈자리 확정
SLOT_BUSY_COLOUR = (60, 60, 235)        # 빨강 — 점유
SLOT_UNKNOWN_COLOUR = (140, 140, 140)   # 회색 — 보는 카메라 없음


class SlotDetection:
    """``SlotOccupancyTracker`` 가 기대하는 최소 형태의 관측.

    프리뷰의 검출 dict 를 그대로 넘기면 tracker 가 ``.center`` / ``.polygon``
    을 못 읽는다. 런타임의 ``MergedDetection`` 과 같은 두 속성만 맞춰준다.
    """

    __slots__ = ('center', 'polygon')

    def __init__(self, center, polygon=None):
        self.center = (float(center[0]), float(center[1]))
        self.polygon = polygon


def format_yolo_regions(regions):
    """구역 표를 ``parse_yolo_regions`` 가 다시 읽을 수 있는 글로 되돌린다."""
    return '; '.join(
        '{}:{:.3f},{:.3f},{:.3f},{:.3f}'.format(label, *regions[label])
        for label in sorted(regions))


# fleet_manager 가 /fleet/state 에 싣는 mission_type 값.
MISSION_PARK = 'park'
MISSION_RETRIEVE = 'retrieve'
MISSION_LABELS_KO = {MISSION_PARK: '입차', MISSION_RETRIEVE: '출차'}


def parse_mission_cameras(text):
    """``'park:cctv0, retrieve:cctv2'`` 를 ``{미션: 라벨}`` 로 바꾼다."""
    mapping = {}
    for chunk in str(text).replace('\n', ',').replace(';', ',').split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ':' not in chunk:
            raise ValueError(
                "yolo_mission_cameras_csv 는 'park:cctv0, retrieve:cctv2' "
                f'형식입니다: {chunk!r}')
        mission, _, label = chunk.partition(':')
        mission, label = mission.strip().lower(), label.strip()
        if mission not in (MISSION_PARK, MISSION_RETRIEVE):
            raise ValueError(
                f"미션은 '{MISSION_PARK}' 또는 '{MISSION_RETRIEVE}' 여야 "
                f'합니다: {mission!r}')
        if not label:
            raise ValueError(f'카메라 라벨이 비었습니다: {chunk!r}')
        mapping[mission] = label
    return mapping


def parse_yolo_regions(text):
    """``'cctv0:0,0,2.2,3.83; cctv2:2.2,0,4.4,3.83'`` 를 구역 표로 바꾼다.

    반환은 ``{label: (xmin, ymin, xmax, ymax)}`` 이고 단위는 **map(월드) 미터**다.
    BEV 화면 왼쪽 아래가 (0,0) 이므로 거기 보이는 숫자를 그대로 적으면 된다.

    항목 구분은 ``;`` 또는 줄바꿈, 라벨과 좌표는 ``:``, 좌표끼리는 ``,`` 다.
    min/max 를 거꾸로 적어도 정규화해서 받아준다.
    """
    regions = {}
    for chunk in str(text).replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ':' not in chunk:
            raise ValueError(
                "yolo_regions_csv 항목은 'label:xmin,ymin,xmax,ymax' 형식입니다: "
                f'{chunk!r}')
        label, _, numbers = chunk.partition(':')
        label = label.strip()
        if not label:
            raise ValueError(f'yolo_regions_csv 라벨이 비었습니다: {chunk!r}')
        try:
            values = [float(v) for v in numbers.split(',') if v.strip()]
        except ValueError as exc:
            raise ValueError(
                f'yolo_regions_csv 좌표를 숫자로 못 읽었습니다: {chunk!r}') from exc
        if len(values) != 4:
            raise ValueError(
                'yolo_regions_csv 는 좌표 4개(xmin,ymin,xmax,ymax)가 필요합니다. '
                f'{len(values)}개 받음: {chunk!r}')
        xmin, ymin, xmax, ymax = values
        regions[label] = (min(xmin, xmax), min(ymin, ymax),
                          max(xmin, xmax), max(ymin, ymax))
    return regions


def relative_pose_metrics(msg):
    """Return the rear-to-front distance, lateral offset, and yaw."""
    position = msg.pose.position
    orientation = msg.pose.orientation
    values = (
        float(position.x), float(position.y), float(orientation.x),
        float(orientation.y), float(orientation.z), float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('relative pose contains non-finite values')
    qx, qy, qz, qw = values[2:]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError('relative pose quaternion norm must be positive')
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz))
    return {
        'forward_m': values[0],
        'lateral_m': values[1],
        'yaw_deg': math.degrees(yaw),
        'frame_id': str(msg.header.frame_id),
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
        self.declare_parameter('marker_size_m', 0.24)
        # Keep the generic/CCTV default unless a close mounting-board edge
        # requires the lower Rear ID0 setting supplied by its launch file.
        self.declare_parameter('aruco_min_marker_distance_rate', 0.05)
        # 검출은 매 프레임 할 필요가 없다. CPU를 아낀다.
        self.declare_parameter('aruco_every_n', 3)
        # aruco_tracker_node가 보정된 카메라 행렬로 계산한 실제 상대 pose.
        # 픽셀 기반 마커 품질 수치와 구분해 웹 상단에 거리/좌우/yaw를 표시한다.
        self.declare_parameter('relative_pose_topic', '/sync/relative_pose')
        self.declare_parameter('marker_visible_topic', '/sync/marker_visible')
        self.declare_parameter('relative_pose_frame', 'rear_base')
        self.declare_parameter('relative_pose_future_tolerance_s', 0.10)
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
        # Production yolo_bev_map이 이미 추론 중이면 모델을 또 올리지 않고
        # 그 결과 envelope(std_msgs/String)를 직접 받는다. 영상 토픽/라벨과
        # 같은 순서로 적으며, 값이 있으면 내장 YOLO보다 항상 우선한다.
        self.declare_parameter('detection_topics_csv', '')
        self.declare_parameter('model_path', 'yolov8n.pt')
        # COCO 기본: 2=car 3=motorcycle 5=bus 7=truck. 사람은 [0].
        self.declare_parameter('yolo_class_ids', [2, 3, 5, 7])
        self.declare_parameter('yolo_confidence', 0.4)
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('yolo_every_n', 10)
        # 비우면 모든 카메라에 YOLO 를 돌린다. 'cctv0' 처럼 주면 그 카메라에만
        # 돌린다 (라벨 기준, 쉼표 구분). 검출이 한 대만 필요할 때 부담을 줄인다.
        self.declare_parameter('yolo_cameras_csv', '')
        # --- 구역별 카메라 전환 ---
        # 'off'  : 대상 카메라 전부에 매번 YOLO (기존 동작)
        # 'region': 차량이 들어간 구역을 맡은 카메라 **한 대만** 추론한다.
        #           GPU 부담이 절반이고, 차량과 가까운 카메라를 쓰게 되어
        #           시차(높이 때문에 바깥으로 밀리는 오차)도 줄어든다.
        self.declare_parameter('yolo_switch_mode', 'off')
        # 'cctv0:xmin,ymin,xmax,ymax; cctv2:...' (map 좌표 m)
        self.declare_parameter('yolo_regions_csv', '')
        # 경계에서 두 카메라가 매 프레임 번갈아 잡히는 것(채터링)을 막는 여유.
        # 지금 카메라 구역을 이만큼 넘어서야 상대 카메라로 넘긴다.
        self.declare_parameter('yolo_switch_margin_m', 0.30)
        # 마지막 검출이 이보다 오래되면 '차량 위치 모름'으로 보고 스캔한다.
        self.declare_parameter('yolo_target_timeout_s', 2.0)
        # 스캔(차량 위치 모를 때) 중 카메라를 바꾸는 주기.
        self.declare_parameter('yolo_scan_period_s', 1.0)
        # 웹 화면에서 드래그로 정한 구역을 여기에 저장/복원한다. 비우면
        # ~/.ros/adaptive_valet_bot/yolo_regions.csv 를 쓴다.
        self.declare_parameter('yolo_regions_file', '')
        # 'mission' 모드에서 쓰는 미션->카메라 대응.
        self.declare_parameter('yolo_mission_cameras_csv',
                               'park:cctv0, retrieve:cctv2')
        # fleet_manager 가 미션 상태를 싣는 토픽.
        self.declare_parameter('fleet_state_topic', '/fleet/state')
        # 이보다 오래 소식이 없으면 미션을 모르는 것으로 보고 스캔한다.
        self.declare_parameter('fleet_state_timeout_s', 5.0)
        # --- 슬롯 점유/빈자리 ---
        # 런타임(cctv_merge)의 기본값과 같게 둔다. 다르게 두면 화면에서 빈칸인데
        # 실제로는 안 비었다고 나오는 상황이 생긴다.
        self.declare_parameter('slot_overlap_threshold', 0.10)
        self.declare_parameter('slot_empty_confirm_frames', 5)
        self.declare_parameter('slot_occupied_hold_s', 0.75)
        # 이보다 오래된 검출은 '그 카메라는 지금 못 보고 있다'로 친다.
        self.declare_parameter('slot_detection_stale_s', 1.5)
        # 슬롯 사각형을 카메라 원본 화면에도 되짚어 그릴지.
        self.declare_parameter('draw_slots_on_camera', True)
        # --- 높이(시차) 보정 ---
        # 바닥 homography 는 바닥 점만 맞다. 높이가 있는 점은 카메라마다
        # 자기 광축 지상점 바깥으로 밀린다. 두 카메라가 서로 다른 방향으로
        # 밀어내므로 BEV 에서 같은 차가 두 곳에 보인다.
        #   오차 = (물체높이 / 카메라높이) x 광축지상점에서의 거리
        # 값을 안 주면(높이 0) 보정하지 않으므로 기존 동작 그대로다.
        self.declare_parameter('camera_optics_csv', '')
        self.declare_parameter('vehicle_detection_height_m', 0.0)
        self.declare_parameter('marker_height_m', 0.0)
        # --- 로봇 안내 화살표 ---
        # 천장에서 보이는 ArUco 두 개가 Front/Rear 주차로봇이다.
        # 기본값은 cctv_robot_marker_node 와 같게 둔다.
        self.declare_parameter('robot_marker_ids_csv', 'front:2, rear:1')
        # fleet_manager 없이 확인할 때 미션을 손으로 고정한다. '' 면 자동.
        self.declare_parameter('guidance_default_mission', '')
        # 마커가 이보다 오래됐으면 로봇을 못 보고 있는 것으로 친다.
        self.declare_parameter('robot_marker_stale_s', 2.0)
        # 실제 주행용 cctv_robot_marker_node의 Bool gate가 이보다 오래되면
        # FALSE로 단정하지 않고 '주행 노드 미수신'으로 표시한다.
        self.declare_parameter('production_marker_visible_stale_s', 1.0)
        # --- 검출 깜빡임 완화 ---
        # 추론 한 번이 아무것도 못 찾았다고 바로 박스를 지우면, 다음 추론까지
        # (yolo_every_n / fps) 초 동안 화면이 빈다. 신뢰도가 문턱 근처에서
        # 흔들릴 때 이게 깜빡임으로 보인다. 이 시간 안에는 직전 결과를
        # 유지하되, 유지 중이라는 것을 화면에 표시한다.
        self.declare_parameter('detection_hold_s', 0.6)
        # --- 중복 검출 제거 ---
        # NMS IoU. 낮출수록 겹친 박스를 더 공격적으로 지운다.
        self.declare_parameter('yolo_iou', 0.45)
        # 한 프레임에서 남길 최대 검출 수. 0 이면 제한 없음.
        self.declare_parameter('yolo_max_det', 0)
        # NMS 를 통과하고도 남는 중복을 world 좌표에서 한 번 더 합친다.
        # 런타임 merge_detections 와 같은 기본값.
        self.declare_parameter('duplicate_center_gate_m', 0.35)
        self.declare_parameter('duplicate_overlap_ratio', 0.30)
        # --- 차량 중심점 이동 추적 ---
        # 직전 위치에서 이 거리를 넘으면 다른 차량으로 본다.
        self.declare_parameter('track_gate_m', 1.0)
        # 검출 흔들림이 누적거리에 쌓이지 않게 하는 죽은 구간.
        self.declare_parameter('track_min_step_m', 0.005)
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
        labels = [label.strip() for label in
                  str(self.get_parameter('labels_csv').value).split(',')
                  if label.strip()]
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
        if self.stale_after <= 0.0:
            raise ValueError('stale_after_s must be positive')
        self.relative_pose_frame = str(
            self.get_parameter('relative_pose_frame').value)
        self.relative_pose_gate = StampGate(
            self.stale_after, float(self.get_parameter(
                'relative_pose_future_tolerance_s').value))
        self.enable_aruco = bool(self.get_parameter('enable_aruco').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        if self.marker_size_m <= 0.0:
            raise ValueError('marker_size_m must be positive')
        self.aruco_every_n = max(1, int(self.get_parameter('aruco_every_n').value))
        self.detector = None
        if self.enable_aruco:
            self.detector = ArucoDetectorCompat(
                cv2, str(self.get_parameter('aruco_dict').value),
                min_marker_distance_rate=float(self.get_parameter(
                    'aruco_min_marker_distance_rate').value))
        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.external_detection_topics = parse_detection_topics(
            self.get_parameter('detection_topics_csv').value, labels)
        self.detection_source = (
            'production' if self.external_detection_topics
            else 'internal' if self.enable_yolo else 'off')
        self.yolo_cameras = {c.strip() for c in
                             str(self.get_parameter('yolo_cameras_csv').value).split(',')
                             if c.strip()}
        # YOLO 대상 카메라를 '등장 순서대로' 고정해 둔다. 스캔이 매번 같은
        # 순서로 돌아야 로그를 보고 동작을 따라갈 수 있다.
        self._yolo_labels = [label for label in labels
                             if not self.yolo_cameras or
                             label in self.yolo_cameras]
        self.yolo_switch_mode = str(
            self.get_parameter('yolo_switch_mode').value).strip().lower()
        if self.yolo_switch_mode not in ('off', 'region', 'mission'):
            raise ValueError(
                "yolo_switch_mode 는 'off' | 'region' | 'mission' 중 하나여야 "
                f'합니다: {self.yolo_switch_mode!r}')
        self.mission_cameras = parse_mission_cameras(
            self.get_parameter('yolo_mission_cameras_csv').value)
        stray = sorted(set(self.mission_cameras.values()) - set(self._yolo_labels))
        if (self.detection_source == 'internal'
                and self.yolo_switch_mode == 'mission' and stray):
            raise ValueError(
                f'yolo_mission_cameras_csv 에 없는 카메라 라벨: {stray}. '
                f'가능한 라벨: {self._yolo_labels}')
        self.fleet_state_timeout_s = float(
            self.get_parameter('fleet_state_timeout_s').value)
        self._mission_type = ''
        self._mission_wall = 0.0
        self._mission_state = ''
        regions_file = str(self.get_parameter('yolo_regions_file').value).strip()
        self.yolo_regions_file = os.path.expanduser(
            regions_file or '~/.ros/adaptive_valet_bot/yolo_regions.csv')
        given = str(self.get_parameter('yolo_regions_csv').value).strip()
        if given:
            # 인자로 준 값이 항상 이긴다. 실행할 때 적은 게 화면에서 만졌던
            # 옛 파일에 덮이면 원인을 찾기 어렵다.
            self.yolo_regions = parse_yolo_regions(given)
            self._regions_source = 'yolo_regions_csv'
        elif os.path.isfile(self.yolo_regions_file):
            with open(self.yolo_regions_file, encoding='utf-8') as handle:
                body = ''.join(line for line in handle
                               if not line.lstrip().startswith('#'))
            self.yolo_regions = parse_yolo_regions(body)
            self._regions_source = self.yolo_regions_file
        else:
            self.yolo_regions = {}
            self._regions_source = None
        unknown = sorted(set(self.yolo_regions) - set(self._yolo_labels))
        if (self.detection_source == 'internal'
                and self.yolo_switch_mode == 'region' and unknown):
            raise ValueError(
                f'yolo_regions_csv 에 없는 카메라 라벨이 있습니다: {unknown}. '
                f'가능한 라벨: {self._yolo_labels}')
        if (self.detection_source == 'internal'
                and self.yolo_switch_mode == 'region'
                and not self.yolo_regions):
            # 구역 없이 region 모드로 두면 담당이 안 정해져 아무 데서도
            # 추론이 안 도는 것처럼 보인다. 여기서 막는 편이 친절하다.
            raise ValueError(
                'yolo_switch_mode:=region 인데 구역이 없습니다. '
                "yolo_regions_csv:='cctv0:0,0,2.2,3.83; cctv2:2.2,0,4.4,3.83' "
                '처럼 주거나, 먼저 off 로 띄운 뒤 웹 화면에서 드래그로 '
                f'구역을 정하고 저장하세요 ({self.yolo_regions_file})')
        self.yolo_switch_margin_m = float(
            self.get_parameter('yolo_switch_margin_m').value)
        if self.yolo_switch_margin_m < 0.0:
            raise ValueError('yolo_switch_margin_m must be >= 0')
        self.yolo_target_timeout_s = float(
            self.get_parameter('yolo_target_timeout_s').value)
        self.yolo_scan_period_s = max(
            0.05, float(self.get_parameter('yolo_scan_period_s').value))
        # 마지막으로 확인한 차량 위치. 이걸 기준으로 담당 카메라를 고른다.
        self._yolo_target = None
        self._yolo_target_wall = 0.0
        self._yolo_active = None
        self._yolo_scanning = True
        # world -> 픽셀 역행렬 캐시. 프레임마다 역행렬을 다시 구할 이유가 없다.
        self._world_to_pixel_H = {}
        self.fleet_state_topic = str(
            self.get_parameter('fleet_state_topic').value).strip()
        self.slot_overlap_threshold = float(
            self.get_parameter('slot_overlap_threshold').value)
        self.slot_empty_confirm_frames = int(
            self.get_parameter('slot_empty_confirm_frames').value)
        self.slot_occupied_hold_s = float(
            self.get_parameter('slot_occupied_hold_s').value)
        self.slot_detection_stale_s = float(
            self.get_parameter('slot_detection_stale_s').value)
        self.draw_slots_on_camera = bool(
            self.get_parameter('draw_slots_on_camera').value)
        # _setup_bev 에서 layout 을 읽은 뒤에 채운다.
        self.slot_tracker = None
        self.slot_state = {}
        self.camera_coverage = {}
        self.camera_optics = parse_camera_optics(
            self.get_parameter('camera_optics_csv').value)
        self.vehicle_detection_height = float(
            self.get_parameter('vehicle_detection_height_m').value)
        self.marker_height = float(self.get_parameter('marker_height_m').value)
        for name, value in (('vehicle_detection_height_m',
                             self.vehicle_detection_height),
                            ('marker_height_m', self.marker_height)):
            if value < 0.0:
                raise ValueError(f'{name} 는 0 이상이어야 합니다')
        # 카메라보다 높은 물체는 이 모형으로 되돌릴 수 없다. 조용히 이상한
        # 좌표를 내놓느니 기동에서 막는다.
        for label, (_x, _y, height) in self.camera_optics.items():
            worst = max(self.vehicle_detection_height, self.marker_height)
            if worst >= height:
                raise ValueError(
                    f'{label}: 카메라 높이 {height:.3f} m 가 물체 높이 '
                    f'{worst:.3f} m 이하입니다')
        self.robot_marker_ids = parse_robot_markers(
            self.get_parameter('robot_marker_ids_csv').value)
        self.robot_marker_stale_s = float(
            self.get_parameter('robot_marker_stale_s').value)
        self.production_marker_visible_stale_s = float(
            self.get_parameter('production_marker_visible_stale_s').value)
        if self.production_marker_visible_stale_s <= 0.0:
            raise ValueError('production_marker_visible_stale_s must be positive')
        self.detection_hold_s = float(
            self.get_parameter('detection_hold_s').value)
        if self.detection_hold_s < 0.0:
            raise ValueError('detection_hold_s must be >= 0')
        self.yolo_iou = float(self.get_parameter('yolo_iou').value)
        if not 0.0 < self.yolo_iou <= 1.0:
            raise ValueError('yolo_iou must be in (0,1]')
        self.yolo_max_det = int(self.get_parameter('yolo_max_det').value)
        self.duplicate_center_gate_m = float(
            self.get_parameter('duplicate_center_gate_m').value)
        self.duplicate_overlap_ratio = float(
            self.get_parameter('duplicate_overlap_ratio').value)
        forced = str(
            self.get_parameter('guidance_default_mission').value).strip().lower()
        if forced and forced not in (MISSION_PARK, MISSION_RETRIEVE):
            raise ValueError(
                "guidance_default_mission 은 '' | 'park' | 'retrieve' 여야 "
                f'합니다: {forced!r}')
        self.guidance_forced_mission = forced
        self._destination_slot_id = ''
        self._source_slot_id = ''
        self.track_gate_m = float(self.get_parameter('track_gate_m').value)
        self.track_min_step_m = float(
            self.get_parameter('track_min_step_m').value)
        self.tracks = {}
        self._setup_yolo()

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        # 종료 시 MJPEG 제너레이터가 빠져나오게 한다. 이게 없으면 Ctrl+C 후
        # 스트림 스레드가 남아 프로세스가 안 죽는다.
        self._stop_event = threading.Event()
        self.production_marker_visibility = {
            role: {
                'topic': f'/{role}/cctv_marker_visible',
                'visible': None,
                'wall': 0.0,
            }
            for role in self.robot_marker_ids
        }
        self._production_marker_subscriptions = []
        for role, runtime in self.production_marker_visibility.items():
            subscription = self.create_subscription(
                Bool, runtime['topic'],
                lambda msg, r=role: self.production_marker_visible_cb(r, msg),
                SENSOR_LATEST_QOS)
            self._production_marker_subscriptions.append(subscription)
        self._mask_shape = None
        self.relative_pose_topic = str(
            self.get_parameter('relative_pose_topic').value).strip()
        self.marker_visible_topic = str(
            self.get_parameter('marker_visible_topic').value).strip()
        self.relative_pose = None
        self.relative_pose_wall = 0.0
        self.marker_visible = None
        self.marker_visible_wall = 0.0
        self._pose_subscription = None
        self._visible_subscription = None
        if self.relative_pose_topic:
            self._pose_subscription = self.create_subscription(
                PoseStamped, self.relative_pose_topic,
                self.relative_pose_cb, SENSOR_LATEST_QOS)
        if self.marker_visible_topic:
            self._visible_subscription = self.create_subscription(
                Bool, self.marker_visible_topic,
                self.marker_visible_cb, SENSOR_LATEST_QOS)
        self.cameras = []
        for label, topic in zip(labels, topics):
            state = {
                'label': label, 'topic': topic, 'frame': None,
                'wall': 0.0, 'count': 0, 'fps': 0.0, 'fps_wall': time.monotonic(),
                'fps_count': 0, 'markers': [], 'marker_wall': 0.0,
                # 최근 ArUco 검사 15회의 ID 집합. 프레임 FPS가 아니라 실제
                # detector 실행 횟수를 분모로 써서 aruco_every_n과 무관하다.
                'marker_history': [],
                'detections': [], 'detection_wall': 0.0,
                # 외부/내장 검출 공통 상태. ``detections``는 화면 깜빡임을
                # 막기 위해 hold될 수 있지만 slot_detections는 최신 원본이다.
                'slot_detections': [],
                # inference_wall: 결과가 비었더라도 '추론이 돌았다'는 사실.
                #   카메라가 살아 있는지 판단할 때 이걸 쓴다.
                # detection_wall: 마지막으로 **뭔가 찾은** 시각. 유지 시간 계산용.
                'inference_wall': 0.0, 'held': False, 'infer_ms': 0.0,
                'detection_source': self.detection_source,
                'detection_topic': self.external_detection_topics.get(label, ''),
                'detection_camera_id': '', 'detection_sequence': 0,
                'detection_stamp_ns': 0, 'detection_messages': 0,
                'detection_invalid': 0, 'detection_dropped': 0,
                'detection_rate_hz': 0.0, 'detection_rate_count': 0,
                'detection_rate_wall': time.monotonic(),
                'transport_age_s': None, 'homography_ok': None,
                'source_coverage': None, 'detection_error': '',
            }
            self.cameras.append(state)
            self.create_subscription(
                Image, topic,
                lambda msg, s=state: self.image_cb(s, msg),
                SENSOR_LATEST_QOS)

        # fleet_manager 가 지금 입차 중인지 출차 중인지 알려준다.
        if self.fleet_state_topic:
            self.create_subscription(
                String, self.fleet_state_topic, self.fleet_state_cb,
                STATE_LATEST_QOS)

        self._bev_mode = 'anaglyph'
        self._setup_bev(labels)

        # 외부 envelope는 world->pixel 역투영에 homography가 필요하다.
        # _setup_bev가 모든 행렬/슬롯 상태를 만든 뒤에 구독을 연결해야 첫
        # 메시지가 생성자 중간에 들어와도 초기화 race가 없다.
        self._external_detection_subscriptions = []
        for state in self.cameras:
            detection_topic = state['detection_topic']
            if detection_topic:
                subscription = self.create_subscription(
                    String, detection_topic,
                    lambda msg, s=state: self.external_detection_cb(s, msg),
                    SENSOR_LATEST_QOS)
                self._external_detection_subscriptions.append(subscription)

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
            tag = ''
            if state['detection_topic']:
                tag = f"  [Production 검출 {state['detection_topic']}]"
            elif self.yolo is not None:
                label = state['label']
                if label not in self._yolo_labels:
                    tag = '  [YOLO 제외]'
                elif self.yolo_switch_mode == 'region':
                    rect = self.yolo_regions.get(label)
                    tag = ('  [YOLO 구역 '
                           f'x {rect[0]:.2f}~{rect[2]:.2f} '
                           f'y {rect[1]:.2f}~{rect[3]:.2f} m]'
                           if rect else '  [YOLO 구역 미지정]')
                else:
                    tag = '  [YOLO]'
            self.get_logger().info(
                f"  {state['label']} <- {state['topic']}{tag}")
        if self.yolo is not None and self.yolo_switch_mode == 'region':
            self.get_logger().info(
                '구역 전환 켜짐: 차량이 들어간 구역의 카메라 한 대만 추론합니다 '
                f'(여유 {self.yolo_switch_margin_m:.2f} m, '
                f'미검출 {self.yolo_target_timeout_s:.1f} s 뒤 스캔)')
        if self.yolo is not None and self.yolo_switch_mode == 'mission':
            pairs = ', '.join(
                f'{MISSION_LABELS_KO.get(m, m)} -> {c}'
                for m, c in sorted(self.mission_cameras.items()))
            self.get_logger().info(
                f'미션 전환 켜짐: {pairs} '
                f'({self.fleet_state_topic} 구독, '
                f'{self.fleet_state_timeout_s:.1f} s 무소식이면 스캔)')

    # ------------------------------------------------------------------
    def relative_pose_cb(self, msg):
        if msg.header.frame_id != self.relative_pose_frame:
            self.get_logger().warn(
                f'상대 pose frame 무시: {msg.header.frame_id}',
                throttle_duration_sec=5.0)
            return
        accepted, reason = self.relative_pose_gate.accept(
            stamp_to_ns(msg.header.stamp), self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'상대 pose stamp 무시: {reason}',
                throttle_duration_sec=5.0)
            return
        try:
            metrics = relative_pose_metrics(msg)
        except ValueError as exc:
            self.get_logger().warn(
                f'상대 pose 무시: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self.relative_pose = metrics
            self.relative_pose_wall = time.monotonic()

    def marker_visible_cb(self, msg):
        with self._lock:
            self.marker_visible = bool(msg.data)
            self.marker_visible_wall = time.monotonic()

    def production_marker_visible_cb(self, role, msg):
        """실제 CCTV 절대 pose 노드의 역할별 유효성 gate를 저장한다."""
        with self._lock:
            runtime = self.production_marker_visibility.get(role)
            if runtime is None:
                return
            runtime['visible'] = bool(msg.data)
            runtime['wall'] = time.monotonic()

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
        if (self._yolo_should_run(state['label'], now) and
                state['count'] % self.yolo_every_n == 0):
            detections = self._detect_vehicles(frame, state)
            self._update_tracks(state['label'], detections)
            self._note_yolo_target(detections, now)
        with self._lock:
            state['frame'] = frame
            state['wall'] = now
            if markers is not None:
                state['markers'] = markers
                state['marker_wall'] = now
            if detections is not None:
                state['slot_detections'] = detections
                self._apply_detection_result(state, detections, now)
            state['count'] += 1
            state['fps_count'] += 1
            elapsed = now - state['fps_wall']
            if elapsed >= 1.0:
                state['fps'] = state['fps_count'] / elapsed
                state['fps_count'] = 0
                state['fps_wall'] = now
        if detections is not None:
            # 검출이 새로 나온 주기에만 부른다. 매 프레임 부르면
            # empty_confirm_frames 가 실제로는 훨씬 짧은 시간이 되어
            # 런타임(cctv_merge)과 판정 타이밍이 어긋난다.
            self._update_slots(now)

    def _project_production_detection(self, label, item):
        """Production world 검출을 현재 카메라 영상에 되투영한다.

        envelope에는 일부러 픽셀 bbox를 중복 저장하지 않는다. segmentation
        polygon과 중심은 map 좌표가 기준이므로, 프리뷰가 가진 같은
        homography의 역행렬로 화면에 되그린다. polygon이 없는 COCO fallback은
        중심 십자만 표시한다.
        """
        center = item.get('world')
        if center is not None:
            pixel = self._world_to_pixel(label, center[0], center[1])
            if pixel is not None:
                item['center_px'] = [round(pixel[0], 1), round(pixel[1], 1)]
        polygon = item.get('world_polygon')
        if polygon:
            projected = [self._world_to_pixel(label, point[0], point[1])
                         for point in polygon]
            if all(point is not None for point in projected):
                item['pixel_polygon'] = [
                    [round(point[0], 1), round(point[1], 1)]
                    for point in projected]
                xs = [point[0] for point in projected]
                ys = [point[1] for point in projected]
                item['box'] = [round(min(xs), 1), round(min(ys), 1),
                               round(max(xs), 1), round(max(ys), 1)]
        return item

    def external_detection_cb(self, state, msg):
        """Production ``/<camera>/detections`` envelope를 프리뷰에 반영한다."""
        now = time.monotonic()
        try:
            envelope = decode_detection_envelope(msg.data)
        except (TypeError, ValueError) as exc:
            with self._lock:
                state['detection_invalid'] += 1
                state['detection_error'] = f'JSON/envelope 오류: {exc}'
            self.get_logger().warn(
                f"[{state['label']}] {state['detection_topic']} 해석 실패: {exc}",
                throttle_duration_sec=5.0)
            return

        camera_id = envelope['camera_id']
        if not camera_ids_match(state['label'], camera_id):
            reason = (f"camera_id 불일치: 화면={state['label']} "
                      f'envelope={camera_id}')
            with self._lock:
                state['detection_invalid'] += 1
                state['detection_error'] = reason
            self.get_logger().warn(reason, throttle_duration_sec=5.0)
            return

        stamp_ns = int(envelope.get('stamp_ns') or 0)
        sequence = int(envelope.get('sequence') or 0)
        with self._lock:
            previous_stamp = int(state.get('detection_stamp_ns') or 0)
            previous_sequence = int(state.get('detection_sequence') or 0)
            out_of_order = (
                stamp_ns > 0 and previous_stamp > 0
                and (stamp_ns < previous_stamp
                     or (stamp_ns == previous_stamp
                         and sequence <= previous_sequence)))
            if out_of_order:
                state['detection_dropped'] += 1
                state['detection_error'] = (
                    f'오래된 envelope 폐기: seq={sequence}, '
                    f'이전={previous_sequence}')
        if out_of_order:
            return

        items = [self._project_production_detection(
                    state['label'], production_detection_item(detection))
                 for detection in envelope['detections']]
        self._update_tracks(state['label'], items)
        self._note_yolo_target(items, now)

        ros_now_ns = self.get_clock().now().nanoseconds
        transport_age = (
            None if stamp_ns <= 0
            else round((ros_now_ns - stamp_ns) / 1_000_000_000.0, 3))
        coverage = envelope.get('coverage_polygon')
        coverage_json = (
            None if coverage is None
            else [[float(point[0]), float(point[1])] for point in coverage])
        with self._lock:
            state['detection_camera_id'] = camera_id
            state['detection_sequence'] = sequence
            state['detection_stamp_ns'] = stamp_ns
            state['detection_messages'] += 1
            state['detection_rate_count'] += 1
            rate_elapsed = now - state['detection_rate_wall']
            if rate_elapsed >= 1.0:
                state['detection_rate_hz'] = (
                    state['detection_rate_count'] / rate_elapsed)
                state['detection_rate_count'] = 0
                state['detection_rate_wall'] = now
            state['transport_age_s'] = transport_age
            state['homography_ok'] = bool(envelope.get('homography_ok', False))
            state['source_coverage'] = coverage_json
            state['detection_error'] = ''
            state['slot_detections'] = items
            state['infer_ms'] = 0.0
            self._apply_detection_result(state, items, now)

        # 슬롯 갱신은 _lock을 다시 잡으므로 위 임계영역 밖에서 호출한다.
        self._update_slots(now)

    def _apply_detection_result(self, state, detections, now):
        """추론 결과를 카메라 상태에 반영한다.

        핵심은 **빈 결과를 어떻게 다룰 것인가**다. 곧바로 지우면 다음 추론까지
        (yolo_every_n / fps) 초 동안 화면이 비어서 깜빡임으로 보인다. 신뢰도가
        문턱 근처에서 흔들릴 때 특히 심하다. 그래서 ``detection_hold_s`` 안에는
        직전 결과를 유지하되 ``held`` 로 표시해 화면에 알린다.

        ``inference_wall`` 은 결과가 비었더라도 갱신한다. "차를 못 찾았다"도
        "이 카메라가 지금 보고 있다"는 유효한 관측이라, 슬롯 관측 가능 판단은
        이 값을 써야 한다.
        """
        state['inference_wall'] = now
        if detections:
            state['detections'] = detections
            state['detection_wall'] = now
            state['held'] = False
            return state
        holding = (bool(state.get('detections'))
                   and self.detection_hold_s > 0.0
                   and (now - state.get('detection_wall', 0.0))
                   <= self.detection_hold_s)
        if holding:
            state['held'] = True
        else:
            state['detections'] = []
            state['held'] = False
        return state

    def _detect_markers(self, frame, state):
        """프레임에서 ArUco를 찾아 주행 안정성 보조 지표까지 계산한다."""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detect_markers(gray)
        except Exception as exc:
            self.get_logger().warn(
                f"[{state['label']}] ArUco 검출 실패: {exc}",
                throttle_duration_sec=10.0)
            self._record_marker_sample(state, [])
            return []
        if ids is None:
            self._record_marker_sample(state, [])
            return []
        try:
            flat_ids = [int(v) for v in ids.flatten()]
        except AttributeError:
            flat_ids = [int(v) for v in ids]

        found = []
        frame_height, frame_width = frame.shape[:2]
        role_by_id = {
            marker_id: role for role, marker_id in self.robot_marker_ids.items()
        }
        for index, marker_id in enumerate(flat_ids):
            candidate = corners[index]
            if len(candidate) == 1 and hasattr(candidate[0], '__len__'):
                points = candidate[0]
            else:
                points = candidate
            try:
                metrics = marker_metrics(
                    points, self.marker_size_m, frame_width, frame_height)
            except (TypeError, ValueError):
                continue
            metrics['id'] = marker_id
            metrics['role'] = role_by_id.get(marker_id, '')
            metrics['corners'] = [[float(pt[0]), float(pt[1])] for pt in points]
            # ArUco 는 로봇 상판 위에 있다. 바닥 평면이 아니다.
            metrics['world'] = self._pixel_to_world(
                state['label'], metrics['center'][0], metrics['center'][1],
                self.marker_height)
            found.append(metrics)
        found.sort(key=lambda m: m['id'])
        self._record_marker_sample(state, found)
        return found

    @staticmethod
    def _record_marker_sample(state, found):
        """최근 detector 실행 15회의 ID별 검출률을 현재 행에 붙인다."""
        history = state.setdefault('marker_history', [])
        history.append({int(marker['id']) for marker in found})
        del history[:-15]
        samples = len(history)
        for marker in found:
            hits = sum(int(marker['id']) in sample for sample in history)
            marker['history_hits'] = hits
            marker['history_samples'] = samples
            marker['detection_ratio'] = round(hits / max(1, samples), 4)

    def _decorate_marker(self, marker, now):
        """마커 행에 현재 Production gate 판정과 이유를 붙인다."""
        item = dict(marker)
        role = str(item.get('role') or '')
        runtime = self.production_marker_visibility.get(role)
        age = None
        fresh = False
        visible = None
        if runtime is not None and runtime['wall'] > 0.0:
            age = max(0.0, now - runtime['wall'])
            fresh = age <= self.production_marker_visible_stale_s
            visible = runtime['visible'] if fresh else None
        item['production_topic'] = '' if runtime is None else runtime['topic']
        item['production_age_s'] = None if age is None else round(age, 3)
        item.update(marker_readiness(item, role, visible, fresh))
        return item

    def _drive_marker_summary(self, cameras, now):
        """카메라별 표와 별개인 front/rear 실제 주행 gate 요약."""
        rows = []
        for role, marker_id in self.robot_marker_ids.items():
            sightings = []
            for camera in cameras:
                for marker in camera.get('markers') or []:
                    if int(marker.get('id', -1)) == int(marker_id):
                        sightings.append((camera['label'], marker))
            representative = None
            if sightings:
                representative = max(
                    (marker for _, marker in sightings),
                    key=lambda marker: float(marker.get('area_px') or 0.0))
            runtime = self.production_marker_visibility[role]
            age = (None if runtime['wall'] <= 0.0
                   else max(0.0, now - runtime['wall']))
            fresh = (age is not None and
                     age <= self.production_marker_visible_stale_s)
            visible = runtime['visible'] if fresh else None
            assessment = marker_readiness(
                representative, role, visible, fresh)
            rows.append({
                'role': role,
                'id': int(marker_id),
                'topic': runtime['topic'],
                'production_fresh': bool(fresh),
                'production_visible': visible,
                'production_age_s': None if age is None else round(age, 3),
                'seen_cameras': [label for label, _ in sightings],
                'status': assessment['drive_status'],
                'class': assessment['drive_class'],
                'drive_ready': assessment['drive_ready'],
                'reason': assessment['drive_reason'],
            })
        return rows

    def _draw_markers(self, canvas, markers):
        for marker in markers:
            pts = np.asarray(marker['corners'], dtype=np.int32)
            # 색은 raw 사다리꼴 정도가 아니라 실제 주행 gate를 따른다.
            spread = marker['side_spread']
            drive_class = marker.get('drive_class')
            colour = ({
                'ok': (80, 230, 100),
                'warn': (40, 200, 255),
                'err': (60, 60, 240),
            }.get(drive_class, (170, 170, 170)))
            cv2.polylines(canvas, [pts], True, colour, 2)
            # 첫 코너(TL)를 굵게 찍어 코너 순서를 눈으로 확인한다
            cv2.circle(canvas, tuple(pts[0]), 5, colour, -1)
            cx, cy = (int(marker['center'][0]), int(marker['center'][1]))
            drive_tag = {
                'ok': 'RUN OK', 'warn': 'RUN CHECK', 'err': 'RUN BLOCK',
            }.get(drive_class, 'REFERENCE')
            lines = [
                f"ID{marker['id']} {marker.get('role', '')}  {drive_tag}",
                (f"min {marker['min_side_px']:.0f}px  "
                 f"seen {marker.get('history_hits', 0)}/"
                 f"{marker.get('history_samples', 0)}"),
                f"perspective {spread * 100:.1f}% (info)",
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
        if self.detection_source == 'production':
            self.yolo_error = ''
            self.get_logger().info(
                '프리뷰 내장 YOLO 생략 — Production 검출 토픽을 직접 구독')
            return
        if not self.enable_yolo:
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
            # 구형 현장 RPi 패키지는 vision_utils에 이 helper가 없다. 이번
            # Rear 프리뷰처럼 YOLO를 끈 실행까지 import 단계에서 죽지 않도록
            # 실제로 YOLO가 필요할 때만 불러온다.
            from cooperative_parking_robot.vision_utils import load_yolo_model
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
        started = time.monotonic()
        try:
            predict_kwargs = {'conf': self.yolo_conf,
                              'imgsz': self.yolo_imgsz,
                              'iou': self.yolo_iou,
                              'verbose': False}
            if self.yolo_max_det > 0:
                predict_kwargs['max_det'] = self.yolo_max_det
            results = self.yolo(frame, **predict_kwargs)
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
            # segmentation 모델이면 mask 외곽선이 함께 온다.
            masks_xy = None
            if getattr(result, 'masks', None) is not None:
                masks_xy = getattr(result.masks, 'xy', None)
            for box_index, box in enumerate(boxes):
                try:
                    class_id = int(box.cls[0])
                    if class_id not in self.yolo_class_ids:
                        continue
                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                    confidence = float(box.conf[0])
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue
                mask_xy = (masks_xy[box_index]
                           if masks_xy is not None and box_index < len(masks_xy)
                           else None)
                geometry = mask_center_geometry(mask_xy)

                # mask 가 있으면 회전사각형 중심을, 없으면 bbox 중심을 쓴다.
                # bbox 중심은 차량이 비스듬할 때 실제 중심에서 밀린다.
                if geometry is not None:
                    center = tuple(geometry['center'])
                    center_source = 'mask'
                else:
                    center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                    center_source = 'bbox'

                world_center = self._pixel_to_world(
                    state['label'], *center, self.vehicle_detection_height)
                item = {
                    'class_id': class_id,
                    'name': self.yolo_names.get(class_id, str(class_id)),
                    'confidence': round(confidence, 3),
                    'box': [round(x1, 1), round(y1, 1),
                            round(x2, 1), round(y2, 1)],
                    'center_px': [round(center[0], 1), round(center[1], 1)],
                    'center_source': center_source,
                    'world': world_center,
                }
                if geometry is not None:
                    item['geometry'] = geometry
                    # 중복 판정과 슬롯 겹침 계산에 쓰려고 여기서 한 번만 낸다.
                    corners = [
                        self._pixel_to_world(
                            state['label'], x, y,
                            self.vehicle_detection_height)
                        for x, y in geometry['corners']]
                    item['world_polygon'] = (
                        None if any(c is None for c in corners)
                        else [tuple(c) for c in corners])
                    # 중심선 양 끝의 world 좌표로 실제 길이/폭(m)을 낸다.
                    item['length_m'] = self._world_length(
                        state['label'], geometry['axes'][0])
                    item['width_m'] = self._world_length(
                        state['label'], geometry['axes'][1])
                found.append(item)
        found = dedupe_detections(
            found, self.duplicate_center_gate_m, self.duplicate_overlap_ratio)
        # 추론에 얼마나 걸리는지 알아야 yolo_every_n 을 근거 있게 정할 수 있다.
        state['infer_ms'] = round((time.monotonic() - started) * 1000.0, 1)
        return found

    def _world_length(self, label, segment):
        """중심선 양 끝점을 world 로 옮겨 실제 길이(m)를 잰다."""
        start = self._pixel_to_world(label, segment[0][0], segment[0][1],
                                     self.vehicle_detection_height)
        end = self._pixel_to_world(label, segment[1][0], segment[1][1],
                                   self.vehicle_detection_height)
        if start is None or end is None:
            return None
        return round(math.dist(start, end), 3)

    def _update_tracks(self, label, detections):
        """차량 중심점이 얼마나 움직였는지 누적한다.

        여러 대가 잡힐 수 있으므로 **직전 위치에 가장 가까운 검출**을 같은
        차량으로 본다. 게이트(``track_gate_m``)를 넘으면 다른 차량으로 보고
        추적을 새로 시작한다. 차 한 대짜리 실험에서는 사실상 항상 같은
        차량이 이어진다.

        기준점(reference)은 추적이 시작된 자리다. UI 의 "기준점 재설정"을
        누르면 현재 위치로 옮긴다. 그래야 "여기서 얼마나 움직였나"를
        원하는 시점부터 잴 수 있다.
        """
        track = self.tracks.setdefault(
            label, {'ref': None, 'last': None, 'path_m': 0.0,
                    'moved_m': 0.0, 'samples': 0, 'reset_pending': False})
        candidates = [d for d in detections if d.get('world') is not None]
        if not candidates:
            return track
        if track['last'] is None:
            chosen = max(candidates, key=lambda d: d['confidence'])
        else:
            chosen = min(candidates,
                         key=lambda d: math.dist(d['world'], track['last']))
            if math.dist(chosen['world'], track['last']) > self.track_gate_m:
                # 다른 차량으로 판단 -> 추적 재시작
                track.update({'ref': None, 'last': None, 'path_m': 0.0,
                              'moved_m': 0.0, 'samples': 0})
                chosen = max(candidates, key=lambda d: d['confidence'])
        world = chosen['world']
        if track['reset_pending'] or track['ref'] is None:
            track['ref'] = list(world)
            track['path_m'] = 0.0
            track['reset_pending'] = False
        elif track['last'] is not None:
            step = math.dist(world, track['last'])
            # 검출 흔들림을 누적거리에 더하지 않도록 죽은 구간을 둔다.
            if step >= self.track_min_step_m:
                track['path_m'] += step
        track['last'] = list(world)
        track['moved_m'] = math.dist(world, track['ref'])
        track['samples'] += 1
        chosen['tracked'] = True
        chosen['moved_m'] = round(track['moved_m'], 3)
        chosen['path_m'] = round(track['path_m'], 3)
        return track

    # ------------------------------------------------------------------
    # 구역 기준 카메라 전환
    #
    # 호모그래피는 바닥 평면 전용이라, 높이가 있는 차체는 카메라 바로 아래
    # 지점에서 **바깥쪽으로** 밀려 보인다. 밀리는 양은 그 지점에서 멀수록
    # 커지므로, 차량에 더 가까운 카메라를 쓰는 것이 그 자체로 오차를 줄인다.
    # 구역을 나눠 담당 카메라를 정하는 이유가 이것이고, GPU 절약은 덤이다.
    # ------------------------------------------------------------------
    def _yolo_should_run(self, label, now):
        """이 프레임에서 이 카메라에 YOLO 를 돌릴지 결정한다."""
        if self.yolo is None:
            return False
        if label not in self._yolo_labels:
            return False
        if self.yolo_switch_mode not in ('region', 'mission'):
            return True
        active = self._yolo_pick_active(now)
        return active is None or label == active

    def fleet_state_cb(self, msg):
        """/fleet/state 에서 지금 미션이 입차인지 출차인지만 뽑아 둔다."""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'{self.fleet_state_topic} 를 JSON 으로 못 읽었습니다',
                throttle_duration_sec=10.0)
            return
        if not isinstance(payload, dict):
            return
        mission = str(payload.get('mission_type') or '').strip().lower()
        state = str(payload.get('state') or '').strip()
        if mission != self._mission_type:
            self.get_logger().info(
                '미션 변경: '
                + (MISSION_LABELS_KO.get(mission, mission) if mission
                   else '없음(대기)'))
        self._mission_type = mission
        self._mission_state = state
        # 어느 슬롯으로 가는지/어느 슬롯에서 빼는지도 여기 실려 온다.
        self._destination_slot_id = str(
            payload.get('active_destination_slot_id') or '')
        self._source_slot_id = str(payload.get('active_source_slot_id') or '')
        # 미션이 비어 있어도 '지금 대기 중'이라는 정보라서 시각은 갱신한다.
        self._mission_wall = time.monotonic()

    def _mission_camera(self, now):
        """미션 기준 담당 카메라. 알 수 없으면 None."""
        if (now - self._mission_wall) > self.fleet_state_timeout_s:
            # fleet_manager 가 안 떠 있거나 죽었다. 미션을 안다고 믿으면
            # 반대편 카메라를 영영 안 보게 된다.
            return None
        return self.mission_cameras.get(self._mission_type)

    def _yolo_pick_active(self, now):
        """지금 추론을 맡을 카메라 하나를 고른다. 없으면 None."""
        if not self._yolo_labels:
            return None
        if self.yolo_switch_mode == 'mission':
            chosen = self._mission_camera(now)
            if chosen is not None:
                self._set_yolo_active(chosen, scanning=False)
                return chosen
            return self._scan_active(now)
        target = self._yolo_target
        fresh = (target is not None and
                 (now - self._yolo_target_wall) <= self.yolo_target_timeout_s)
        if fresh:
            chosen = self._region_owner(target[0], target[1])
            self._set_yolo_active(chosen, scanning=False)
            return chosen
        # 차량 위치를 모른다 -> 두 카메라를 번갈아 훑는다. 한 대에만 계속
        # 붙어 있으면 반대편에 들어온 차를 영영 못 찾는다.
        return self._scan_active(now)

    def _scan_active(self, now):
        """담당을 정할 근거가 없을 때 카메라를 번갈아 본다."""
        index = int(now / self.yolo_scan_period_s) % len(self._yolo_labels)
        chosen = self._yolo_labels[index]
        self._set_yolo_active(chosen, scanning=True)
        return chosen

    def _set_yolo_active(self, label, scanning):
        if label != self._yolo_active or scanning != self._yolo_scanning:
            # 스캔 중에는 주기마다 바뀌므로 로그를 남기지 않는다. 실제
            # 인계(차량이 구역을 넘어감)만 남겨야 로그가 읽힌다.
            if label is not None and not scanning:
                self.get_logger().info(f'YOLO 담당 카메라 -> {label}')
            self._yolo_active = label
            self._yolo_scanning = scanning

    def _region_owner(self, x, y):
        """(x, y) 를 맡은 카메라. 채터링을 막으려 현재 카메라를 우대한다."""
        current = self._yolo_active
        # 현재 담당이 여유 구간 안에 아직 붙잡고 있으면 그대로 둔다.
        if (current in self.yolo_regions and
                self._in_region(current, x, y, self.yolo_switch_margin_m)):
            return current
        for label in self._yolo_labels:
            if self._in_region(label, x, y, 0.0):
                return label
        # 어느 구역에도 안 들어감(맵 밖 등) -> 담당을 바꾸지 않는다.
        return current if current in self._yolo_labels else self._yolo_labels[0]

    def _in_region(self, label, x, y, margin):
        rect = self.yolo_regions.get(label)
        if rect is None:
            return False
        xmin, ymin, xmax, ymax = rect
        return (xmin - margin <= x <= xmax + margin and
                ymin - margin <= y <= ymax + margin)

    def _note_yolo_target(self, detections, now):
        """검출에서 차량 위치를 뽑아 다음 프레임의 담당 카메라 판단에 쓴다.

        놓친 프레임에서 위치를 지우지는 않는다. 한 프레임 놓쳤다고 스캔으로
        떨어지면 담당이 계속 튄다. ``yolo_target_timeout_s`` 로만 만료시킨다.
        """
        if not detections:
            return
        located = [d for d in detections if d.get('world') is not None]
        if not located:
            return
        # _update_tracks 가 고른 차량이 있으면 그것을 따른다.
        chosen = next((d for d in located if d.get('tracked')), None)
        if chosen is None:
            chosen = max(located, key=lambda d: d['confidence'])
        self._yolo_target = list(chosen['world'])
        self._yolo_target_wall = now

    def set_yolo_region(self, label, x1, y1, x2, y2):
        """구역 하나를 바꾼다. 웹 화면(다른 스레드)에서 불린다."""
        if label not in self._yolo_labels:
            raise ValueError(
                f'YOLO 대상 카메라가 아닙니다: {label!r} '
                f'(가능: {self._yolo_labels})')
        values = [float(v) for v in (x1, y1, x2, y2)]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('구역 좌표에 숫자가 아닌 값이 있습니다')
        xmin, ymin = min(values[0], values[2]), min(values[1], values[3])
        xmax, ymax = max(values[0], values[2]), max(values[1], values[3])
        if xmax - xmin < 0.05 or ymax - ymin < 0.05:
            # 크기를 같이 알려준다. 0.00 이면 드래그가 아니라 화면 좌표
            # 변환이 깨진 것이라서, 원인을 바로 가를 수 있다.
            raise ValueError(
                f'구역이 너무 작습니다 ({xmax - xmin:.2f} x {ymax - ymin:.2f} m). '
                '한 변 5 cm 이상 끌어주세요')
        # 제자리에서 고치지 않고 통째로 갈아끼운다. 추론 스레드가 읽는
        # 도중에 반쯤 바뀐 표를 보는 일이 없다.
        updated = dict(self.yolo_regions)
        updated[label] = (xmin, ymin, xmax, ymax)
        self.yolo_regions = updated
        self.get_logger().info(
            f'구역 지정 {label}: x {xmin:.2f}~{xmax:.2f} '
            f'y {ymin:.2f}~{ymax:.2f} m')
        return self.yolo_regions[label]

    def clear_yolo_region(self, label):
        updated = dict(self.yolo_regions)
        updated.pop(label, None)
        self.yolo_regions = updated
        if self.yolo_switch_mode == 'region' and not updated:
            # 구역이 하나도 안 남으면 담당을 못 정한다. 조용히 멈추는 대신
            # 전부 추론하는 쪽으로 되돌린다.
            self.yolo_switch_mode = 'off'
            self.get_logger().warn(
                '구역이 모두 지워져 yolo_switch_mode 를 off 로 되돌립니다')
        self.get_logger().info(f'구역 지움: {label}')

    def save_yolo_regions(self):
        """지금 구역을 파일로 남긴다. 다음에 띄우면 자동으로 불러온다."""
        directory = os.path.dirname(self.yolo_regions_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        text = format_yolo_regions(self.yolo_regions)
        with open(self.yolo_regions_file, 'w', encoding='utf-8') as handle:
            handle.write('# camera_preview YOLO 구역 (map 좌표 m)\n')
            handle.write('# label:xmin,ymin,xmax,ymax\n')
            for label in sorted(self.yolo_regions):
                handle.write(
                    '{}:{:.3f},{:.3f},{:.3f},{:.3f}\n'.format(
                        label, *self.yolo_regions[label]))
        self._regions_source = self.yolo_regions_file
        self.get_logger().info(f'구역 저장: {self.yolo_regions_file}')
        return {'path': self.yolo_regions_file, 'csv': text}

    def set_yolo_switch_mode(self, mode):
        if getattr(self, 'detection_source', 'internal') != 'internal':
            raise ValueError(
                '카메라 전환은 프리뷰 내장 YOLO 모드에서만 바꿀 수 있습니다')
        mode = str(mode).strip().lower()
        if mode not in ('off', 'region', 'mission'):
            raise ValueError("yolo_switch_mode 는 'off' | 'region' | 'mission'")
        if mode == 'region' and not self.yolo_regions:
            raise ValueError('구역을 먼저 하나 이상 지정하세요')
        if mode == 'mission' and not self.mission_cameras:
            raise ValueError('yolo_mission_cameras_csv 가 비었습니다')
        self.yolo_switch_mode = mode
        self.get_logger().info(f'yolo_switch_mode -> {mode}')
        return mode

    def _world_to_pixel(self, label, x, y):
        """Map 좌표(m)를 그 카메라의 영상 픽셀로. 바닥 평면 기준이다."""
        inverse = self._world_to_pixel_H.get(label)
        if inverse is None:
            matrix = self.pixel_to_world_H.get(label)
            if matrix is None:
                return None
            try:
                inverse = np.linalg.inv(matrix)
            except np.linalg.LinAlgError:
                return None
            self._world_to_pixel_H[label] = inverse
        vector = inverse @ np.array([float(x), float(y), 1.0])
        w = float(vector[2])
        # w <= 0 이면 카메라 뒤쪽이다. 그대로 나누면 엉뚱한 곳에 찍힌다.
        if w <= 1e-9:
            return None
        return (float(vector[0]) / w, float(vector[1]) / w)

    # ------------------------------------------------------------------
    # 슬롯 점유 / 빈자리
    # ------------------------------------------------------------------
    def _detection_world_polygon(self, label, detection):
        """검출의 회전사각형 네 꼭짓점을 map 좌표로 옮긴다."""
        # _detect_vehicles 에서 이미 계산해 둔다. 프레임마다 다시 낼 이유가 없다.
        return detection.get('world_polygon')

    def _update_slots(self, now):
        """모든 카메라의 최근 검출로 슬롯 점유를 갱신한다.

        관측 가능 여부를 **"지금 실제로 추론이 도는 카메라"** 기준으로 낸다.
        homography 가 있다고 관측 가능으로 치면, 구역/미션 전환으로 YOLO 가
        꺼져 있는 카메라 쪽 슬롯이 검출 없음 -> 빈자리로 뒤집힌다. 차 있는
        칸을 비었다고 말하는 것이 이 시스템에서 제일 위험한 오류다.
        """
        if self.slot_tracker is None:
            return
        detections = []
        live_coverage = {}
        with self._lock:
            snapshot = [(state['label'],
                         ((state.get('slot_detections') or [])
                          if 'slot_detections' in state
                          else (state.get('detections') or [])),
                         # '검출이 있었나'가 아니라 '추론이 돌았나'로 본다.
                         # 빈 결과도 "저 칸에 차가 없다"는 유효한 관측이다.
                         state.get('inference_wall', 0.0),
                         state.get('source_coverage'))
                        for state in self.cameras]
        for label, found, wall, source_coverage in snapshot:
            if wall <= 0.0 or (now - wall) > self.slot_detection_stale_s:
                continue
            # Production envelope가 보낸 실제 coverage를 우선한다. 프리뷰가
            # 따로 계산한 값과 달라져 화면 판정이 런타임과 어긋나는 일을 막는다.
            coverage = source_coverage or self.camera_coverage.get(label)
            if coverage is None:
                continue
            live_coverage[label] = coverage
            for detection in found:
                center = detection.get('world')
                if center is None:
                    continue
                detections.append(SlotDetection(
                    center, self._detection_world_polygon(label, detection)))

        slot_polygons = {slot_id: polygon for slot_id, polygon in self.slots}
        observable = slot_observability(slot_polygons, live_coverage)
        state = self.slot_tracker.update(
            slot_polygons, detections, observable, now)
        # 그리기/웹은 다른 스레드가 읽는다. 통째로 갈아끼운다.
        self.slot_state = {
            slot_id: {'occupied': bool(item['occupied']),
                      'observed': bool(item['observed'])}
            for slot_id, item in state.items()
        }

    # ------------------------------------------------------------------
    # 로봇 안내: ArUco 두 개 -> 목적지 방향
    #
    # 천장에서 보이는 두 마커가 Front/Rear 주차로봇이다. 두 로봇은 차량을
    # 앞뒤에서 들어 올리므로, **두 마커의 중점**이 곧 실을 차량의 중심에
    # 가깝다. 그 점에서 목적지까지가 지금 가야 할 방향이다.
    #
    #   입차(park)     : 배정된 빈 슬롯으로
    #   출차(retrieve) : 대기영역(차를 내려놓는 곳)으로
    # ------------------------------------------------------------------
    def _robot_marker_world(self, now):
        """역할별 로봇 마커의 map 좌표. 최근에 본 것만 쓴다."""
        wanted = {marker_id: role
                  for role, marker_id in self.robot_marker_ids.items()}
        with self._lock:
            snapshot = [(state.get('markers') or [],
                         state.get('marker_wall', 0.0))
                        for state in self.cameras]
        found = {}
        for markers, wall in snapshot:
            if wall <= 0.0 or (now - wall) > self.robot_marker_stale_s:
                continue
            for marker in markers:
                role = wanted.get(marker.get('id'))
                world = marker.get('world')
                if role is None or world is None:
                    continue
                # 같은 마커가 두 카메라에 보이면 먼저 잡힌 쪽을 쓴다.
                # 평균을 내면 두 호모그래피 오차가 섞여 오히려 나빠진다.
                found.setdefault(role, (float(world[0]), float(world[1])))
        return found

    def _slot_centroid(self, slot_id):
        for candidate, polygon in self.slots:
            if candidate == slot_id:
                return polygon_centroid(polygon)
        return None

    def _guidance_goal(self, mission):
        """미션별 목적지 (좌표, 이름). 못 정하면 (None, 사유)."""
        if mission == MISSION_PARK:
            if self._destination_slot_id:
                centroid = self._slot_centroid(self._destination_slot_id)
                if centroid is not None:
                    return centroid, self._destination_slot_id
            # fleet 이 아직 슬롯을 안 정했다 -> 확정된 빈자리 중 첫 칸을 쓴다.
            for slot_id in self.empty_slot_ids():
                centroid = self._slot_centroid(slot_id)
                if centroid is not None:
                    return centroid, slot_id
            return None, '빈자리 없음'
        if mission == MISSION_RETRIEVE:
            if self.waiting:
                return polygon_centroid(self.waiting), 'WAIT'
            return None, '대기영역 미등록'
        return None, '미션 없음'

    def _marker_disagreement(self):
        """같은 ArUco 를 두 카메라가 본 위치 차이(m). 높이 조정의 기준.

        두 카메라의 밀림 방향이 다르므로, 높이 값이 실제와 맞을수록 이
        값이 줄어든다. 화면에서 이 숫자를 보며 높이를 맞추면 된다.
        """
        now = time.monotonic()
        seen = {}
        with self._lock:
            for state in self.cameras:
                if now - state['marker_wall'] > self.stale_after:
                    continue
                for marker in (state['markers'] or []):
                    world = marker.get('world')
                    if world is None:
                        continue
                    seen.setdefault(marker['id'], []).append(tuple(world))
        gaps = {}
        for marker_id, points in seen.items():
            if len(points) < 2:
                continue
            gaps[str(marker_id)] = round(
                max(math.dist(a, b)
                    for i, a in enumerate(points)
                    for b in points[i + 1:]), 3)
        return gaps or None

    def _guidance(self, now):
        """지금 화면에 그릴 안내. 조건이 안 되면 사유를 담아 돌려준다."""
        mission = self.guidance_forced_mission or self._mission_type
        robots = self._robot_marker_world(now)
        info = {
            'mission': mission,
            'forced': bool(self.guidance_forced_mission),
            'robots': {role: list(point) for role, point in robots.items()},
            'from': None, 'to': None, 'goal': '',
            'distance_m': None, 'heading_deg': None, 'reason': '',
        }
        if not mission:
            info['reason'] = '미션 없음 (fleet 대기 중)'
            return info
        if not robots:
            info['reason'] = f'로봇 마커 미검출 (ID {sorted(self.robot_marker_ids.values())})'
            return info
        if len(robots) == 2:
            (ax, ay), (bx, by) = robots['front'], robots['rear']
            origin = ((ax + bx) / 2.0, (ay + by) / 2.0)
        else:
            # 한 대만 보일 때도 방향은 보여준다. 다만 중점이 아니라는 것을
            # 사유에 남겨 화면만 보고 오해하지 않게 한다.
            role, origin = next(iter(robots.items()))
            info['reason'] = f'{role} 마커만 보임 (중점 아님)'
        goal, name = self._guidance_goal(mission)
        if goal is None:
            info['reason'] = name
            return info
        dx, dy = goal[0] - origin[0], goal[1] - origin[1]
        info.update({
            'from': [round(origin[0], 3), round(origin[1], 3)],
            'to': [round(goal[0], 3), round(goal[1], 3)],
            'goal': name,
            'distance_m': round(math.hypot(dx, dy), 3),
            'heading_deg': round(math.degrees(math.atan2(dy, dx)), 1),
        })
        return info

    def _draw_guidance(self, canvas, to_px, scale=1.0):
        """안내 화살표를 그린다. ``to_px`` 는 map(m) -> 픽셀 변환."""
        guidance = self._guidance(time.monotonic())
        robots = guidance['robots']
        points = {}
        for role, world in robots.items():
            pixel = to_px(world[0], world[1])
            if pixel is None:
                continue
            points[role] = pixel
            cv2.circle(canvas, pixel, max(3, int(6 * scale)),
                       ROBOT_AXIS_COLOUR, -1)
            cv2.putText(canvas, role.upper(),
                        (pixel[0] + 8, pixel[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale,
                        ROBOT_AXIS_COLOUR, 1, cv2.LINE_AA)
        if len(points) == 2:
            # 두 로봇을 잇는 축 = 차량을 드는 방향
            cv2.line(canvas, points['front'], points['rear'],
                     ROBOT_AXIS_COLOUR, max(1, int(2 * scale)), cv2.LINE_AA)
        if guidance['from'] is None or guidance['to'] is None:
            return guidance
        start = to_px(guidance['from'][0], guidance['from'][1])
        end = to_px(guidance['to'][0], guidance['to'][1])
        if start is None or end is None:
            return guidance
        cv2.arrowedLine(canvas, start, end, GUIDANCE_COLOUR,
                        max(2, int(3 * scale)), cv2.LINE_AA, tipLength=0.12)
        tag = 'PARK' if guidance['mission'] == MISSION_PARK else 'EXIT'
        text = (f"{tag} -> {guidance['goal']}  "
                f"{guidance['distance_m']:.2f}m")
        origin = (start[0] + 10, start[1] - 12)
        cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 * scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 * scale, GUIDANCE_COLOUR, 1, cv2.LINE_AA)
        return guidance

    def _slot_appearance(self, slot_id):
        """슬롯 상태 -> (색, 꼬리표, 두께). 표시 규칙을 한 곳에 모은다."""
        item = self.slot_state.get(slot_id)
        if item is None or not item['observed']:
            # 안 보이는 칸은 초록도 빨강도 아니다. 모른다고 표시해야 한다.
            return SLOT_UNKNOWN_COLOUR, '?', 1
        if item['occupied']:
            return SLOT_BUSY_COLOUR, 'BUSY', 2
        return SLOT_FREE_COLOUR, 'FREE', 3

    def empty_slot_ids(self):
        return sorted(
            slot_id for slot_id, item in self.slot_state.items()
            if item['observed'] and not item['occupied'])

    def _pixel_to_world(self, label, px, py, height=0.0):
        """영상 픽셀을 map 좌표(m)로. H가 없으면 None.

        ``height`` 가 0 보다 크고 그 카메라의 광학 정보가 있으면, 바닥
        homography 가 낸 점을 실제 물체 높이의 평면으로 되돌린다. 광학
        정보가 없으면 보정 없이 그대로 둔다 — 틀린 값으로 보정하는 것보다
        보정을 안 하는 편이 낫고, 화면에도 그 사실이 표시된다.
        """
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
        # Pure geometry callers and lightweight test doubles predate the
        # optional parallax configuration. Missing optics must preserve the
        # original floor-homography behavior instead of becoming an API break.
        optics = getattr(self, 'camera_optics', {}).get(label)
        if optics is not None and float(height) > 0.0:
            ground_x, ground_y, cam_height = optics
            try:
                x, y = correct_floor_projection(
                    x, y, ground_x, ground_y, cam_height, float(height))
            except ValueError:
                # 기동에서 이미 막았지만, 파라미터가 런타임에 바뀌어도
                # 화면이 죽지는 않게 한다.
                pass
        return [round(x, 3), round(y, 3)]

    def parallax_shift_m(self, label, height):
        """이 카메라·높이에서 화면 가장자리가 얼마나 밀리는지(m). 화면 표시용."""
        optics = getattr(self, 'camera_optics', {}).get(label)
        if optics is None or float(height) <= 0.0:
            return None
        _gx, _gy, cam_height = optics
        coverage = self.camera_coverage.get(label)
        if not coverage:
            return None
        ground = (optics[0], optics[1])
        far = max(math.dist(ground, point) for point in coverage)
        return round(far * float(height) / float(cam_height), 3)

    def _draw_detections(self, canvas, detections):
        for item in detections:
            box = item.get('box')
            if box is not None:
                x1, y1, x2, y2 = [int(round(v)) for v in box]
                cv2.rectangle(canvas, (x1, y1), (x2, y2),
                              (255, 170, 60), 1)
            else:
                x1 = y1 = 0

            pixel_polygon = item.get('pixel_polygon')
            if pixel_polygon:
                polygon = np.asarray(pixel_polygon, dtype=np.int32)
                cv2.polylines(canvas, [polygon], True, (80, 230, 100), 2,
                              cv2.LINE_AA)

            geometry = item.get('geometry')
            if geometry is not None:
                # 최소 회전 사각형
                pts = np.asarray(geometry['corners'], dtype=np.int32)
                cv2.polylines(canvas, [pts], True, (80, 230, 100), 2)
                # 네 변의 중점
                for mid in geometry['edge_midpoints']:
                    cv2.circle(canvas, (int(mid[0]), int(mid[1])), 4,
                               (255, 255, 255), -1)
                    cv2.circle(canvas, (int(mid[0]), int(mid[1])), 4,
                               (0, 0, 0), 1)
                # 마주 보는 중점을 이은 중심선 두 개
                for axis, colour in zip(geometry['axes'],
                                        ((60, 220, 255), (255, 200, 90))):
                    cv2.line(canvas,
                             (int(axis[0][0]), int(axis[0][1])),
                             (int(axis[1][0]), int(axis[1][1])),
                             colour, 2, cv2.LINE_AA)
                # 두 중심선의 교점 = 중심점
                cx, cy = (int(geometry['center'][0]),
                          int(geometry['center'][1]))
                cv2.drawMarker(canvas, (cx, cy), (0, 0, 255),
                               cv2.MARKER_CROSS, 18, 2)
                cv2.circle(canvas, (cx, cy), 6, (0, 0, 255), 2)

            center_px = item.get('center_px')
            if center_px is not None and geometry is None:
                cx, cy = int(round(center_px[0])), int(round(center_px[1]))
                cv2.drawMarker(canvas, (cx, cy), (0, 0, 255),
                               cv2.MARKER_CROSS, 18, 2)
                cv2.circle(canvas, (cx, cy), 6, (0, 0, 255), 2)
                if box is None:
                    x1, y1 = cx, cy

            world = item.get('world')
            text = f"{item['name']} {item['confidence']:.2f}"
            if world is not None:
                text += f"  ({world[0]:.2f}, {world[1]:.2f})m"
            origin = (x1, max(14, y1 - 6))
            cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 170, 60), 1, cv2.LINE_AA)

            if item.get('source') == 'production':
                details = []
                if (item.get('length_m') is not None
                        and item.get('width_m') is not None):
                    details.append(
                        f"{item['length_m']:.2f}x{item['width_m']:.2f}m")
                if item.get('yaw_deg') is not None:
                    details.append(f"yaw {item['yaw_deg']:.1f}deg")
                if item.get('in_waiting'):
                    details.append('WAIT')
                if details:
                    detail_origin = (x1, max(30, y1 - 23))
                    detail_text = '  '.join(details)
                    cv2.putText(canvas, detail_text, detail_origin,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(canvas, detail_text, detail_origin,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (80, 230, 100), 1, cv2.LINE_AA)

            if item.get('tracked'):
                # cv2.putText 는 한글 글리프가 없어 ASCII 로 적는다.
                moved = (f"moved {item['moved_m'] * 100:.1f}cm  "
                         f"path {item['path_m'] * 100:.1f}cm")
                spot = (x1, max(30, y1 - 24))
                for thickness, shade in ((3, (0, 0, 0)), (1, (0, 0, 255))):
                    cv2.putText(canvas, moved, spot,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, shade,
                                thickness, cv2.LINE_AA)
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
        self._setup_slots()

    def _setup_slots(self):
        """슬롯 점유 판정 준비. layout 과 homography 가 둘 다 있어야 한다."""
        if not self.slots or not self.pixel_to_world_H:
            if self.slots and not self.pixel_to_world_H:
                self.get_logger().warn(
                    'homography 가 없어 빈자리 판정을 못 합니다')
            return
        # 각 카메라가 바닥에서 덮는 사각형. 영상 네 귀퉁이를 H 로 투영한다.
        for label, matrix in self.pixel_to_world_H.items():
            try:
                self.camera_coverage[label] = image_corner_coverage(
                    matrix, self.calib_w, self.calib_h)
            except (ValueError, TypeError) as exc:
                self.get_logger().warn(f'[{label}] 커버리지 계산 실패: {exc}')
        self.slot_tracker = SlotOccupancyTracker(
            [slot_id for slot_id, _ in self.slots],
            overlap_threshold=self.slot_overlap_threshold,
            empty_confirm_frames=self.slot_empty_confirm_frames,
            occupied_hold_s=self.slot_occupied_hold_s,
            now=time.monotonic())
        self.get_logger().info(
            f'빈자리 판정 준비: 슬롯 {len(self.slots)}개 · '
            f'겹침 {self.slot_overlap_threshold:.0%} 이상이면 점유 · '
            f'{self.slot_empty_confirm_frames}프레임 연속이면 빈자리 확정')

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

        if self.detection_source == 'internal':
            for index, label in enumerate(self._yolo_labels):
                rect = self.yolo_regions.get(label)
                if rect is None:
                    continue
                colour = REGION_COLOURS[index % len(REGION_COLOURS)]
                corner_a = to_px(rect[0], rect[3])
                corner_b = to_px(rect[2], rect[1])
                live = (label == self._yolo_active and not self._yolo_scanning)
                cv2.rectangle(canvas, corner_a, corner_b, colour,
                              3 if live else 1)
                cv2.putText(canvas, label + (' <-YOLO' if live else ''),
                            (corner_a[0] + 6, corner_a[1] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                            cv2.LINE_AA)

        for slot_id, polygon in self.slots:
            pts = np.asarray([to_px(x, y) for x, y in polygon], dtype=np.int32)
            colour, tag, thickness = self._slot_appearance(slot_id)
            cv2.polylines(canvas, [pts], True, colour, thickness)
            cx = int(sum(p[0] for p in pts) / len(pts))
            cy = int(sum(p[1] for p in pts) / len(pts))
            cv2.putText(canvas, f'{slot_id} {tag}', (cx - 26, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                        cv2.LINE_AA)
        self._draw_guidance(canvas, lambda x, y: to_px(x, y), scale=1.0)

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
                world_polygon = item.get('world_polygon')
                if world_polygon:
                    polygon = np.asarray(
                        [to_px(point[0], point[1])
                         for point in world_polygon], dtype=np.int32)
                    cv2.polylines(canvas, [polygon], True, colour, 2,
                                  cv2.LINE_AA)
                point = to_px(world[0], world[1])
                cv2.circle(canvas, point, 7, colour, -1)
                cv2.circle(canvas, point, 7, (0, 0, 0), 1)
                text = (f"{item['name']} "
                        f"{item['confidence']:.2f} "
                        f"({world[0]:.2f},{world[1]:.2f})")
                for thickness, shade in ((3, (0, 0, 0)), (1, colour)):
                    cv2.putText(canvas, text, (point[0] + 10, point[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, shade,
                                thickness, cv2.LINE_AA)
            track = self.tracks.get(label)
            if track and track.get('ref') and track.get('last'):
                ref_px = to_px(track['ref'][0], track['ref'][1])
                now_px = to_px(track['last'][0], track['last'][1])
                # 기준점(빈 원) -> 현재 중심점(채운 원) 변위 화살표
                cv2.circle(canvas, ref_px, 7, (200, 200, 200), 2)
                cv2.arrowedLine(canvas, ref_px, now_px, (0, 0, 255), 2,
                                cv2.LINE_AA, tipLength=0.25)
                cv2.drawMarker(canvas, now_px, (0, 0, 255),
                               cv2.MARKER_CROSS, 16, 2)
                moved = (f"moved {track['moved_m'] * 100:.1f}cm / "
                         f"path {track['path_m'] * 100:.1f}cm")
                for thickness, shade in ((3, (0, 0, 0)), (1, (0, 0, 255))):
                    cv2.putText(canvas, moved,
                                (now_px[0] + 12, now_px[1] + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, shade,
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
        usable = [(label, frame) for label, frame in snapshot
                  if frame is not None and label in self.homographies]
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

    def _draw_guidance_on_camera(self, canvas, label):
        """안내 화살표를 카메라 원본 화면에도 되짚어 그린다."""
        if self.pixel_to_world_H.get(label) is None:
            return
        height, width = canvas.shape[:2]

        def to_px(x, y):
            point = self._world_to_pixel(label, x, y)
            if point is None:
                return None
            px, py = int(round(point[0])), int(round(point[1]))
            # 화면에서 멀리 벗어난 점은 그리지 않는다. 화살표가 엉뚱한
            # 방향으로 화면을 가로질러 오히려 오해를 준다.
            if not (-width <= px <= 2 * width and -height <= py <= 2 * height):
                return None
            return (px, py)

        self._draw_guidance(canvas, to_px, scale=1.0)

    def _draw_slots_on_camera(self, canvas, label):
        """슬롯 사각형을 카메라 원본 화면에 되짚어 그린다.

        BEV 만 보면 "저 초록칸이 실제로 어디냐"가 안 잡힌다. 영상에서는
        원근 때문에 사다리꼴로 보이는 것이 정상이다.
        """
        if not self.slots or self.pixel_to_world_H.get(label) is None:
            return
        height, width = canvas.shape[:2]
        for slot_id, polygon in self.slots:
            points = [self._world_to_pixel(label, x, y) for x, y in polygon]
            if any(point is None for point in points):
                continue
            # 화면 밖으로 크게 벗어난 슬롯은 그리지 않는다. 투영값이 수천
            # 픽셀이면 선이 화면을 가로질러 오히려 방해가 된다.
            if all(not (-width <= x <= 2 * width and -height <= y <= 2 * height)
                   for x, y in points):
                continue
            colour, tag, thickness = self._slot_appearance(slot_id)
            pts = np.asarray(
                [[int(round(x)), int(round(y))] for x, y in points],
                dtype=np.int32)
            cv2.polylines(canvas, [pts], True, colour, thickness, cv2.LINE_AA)
            cx = int(sum(p[0] for p in pts) / len(pts))
            cy = int(sum(p[1] for p in pts) / len(pts))
            cv2.putText(canvas, f'{slot_id} {tag}', (cx - 26, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    def _draw_region_on_camera(self, canvas, label):
        """BEV 에서 정한 구역이 이 카메라 화면 어디인지 되짚어 그린다.

        바닥 평면 위의 사각형이라 호모그래피로 정확히 옮겨진다. 다만
        영상에서는 원근 때문에 **직사각형이 아니라 사다리꼴**로 보인다.
        그게 정상이고, 오히려 바닥에 제대로 붙었다는 표시다.
        """
        if self.detection_source != 'internal':
            return
        rect = self.yolo_regions.get(label)
        # numpy 배열은 truthiness 로 검사하면 ValueError 가 난다. is None 으로만 본다.
        if rect is None or self.pixel_to_world_H.get(label) is None:
            return
        xmin, ymin, xmax, ymax = rect
        corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
        points = [self._world_to_pixel(label, x, y) for x, y in corners]
        if any(p is None for p in points):
            return
        index = (self._yolo_labels.index(label)
                 if label in self._yolo_labels else 0)
        colour = REGION_COLOURS[index % len(REGION_COLOURS)]
        live = (label == self._yolo_active and not self._yolo_scanning)
        pts = np.asarray([[int(round(x)), int(round(y))] for x, y in points],
                         dtype=np.int32)
        cv2.polylines(canvas, [pts], True, colour, 3 if live else 2,
                      cv2.LINE_AA)
        top = min(pts, key=lambda p: p[1])
        cv2.putText(canvas, 'region' + (' <-YOLO' if live else ''),
                    (int(top[0]) + 6, max(16, int(top[1]) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    def _annotate(self, frame, state):
        """격자·중심선·라벨을 그린다. 원본은 건드리지 않는다."""
        canvas = frame.copy()
        height, width = canvas.shape[:2]
        self._draw_markers(canvas, state.get('markers') or [])
        self._draw_detections(canvas, state.get('detections') or [])
        self._draw_region_on_camera(canvas, state['label'])
        if self.draw_slots_on_camera:
            self._draw_slots_on_camera(canvas, state['label'])
        self._draw_guidance_on_camera(canvas, state['label'])
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
        if state.get('detection_source') == 'production':
            age = (time.monotonic() - state['inference_wall']
                   if state.get('inference_wall') else None)
            label += (f"  PROD {state.get('detection_rate_hz', 0.0):.1f}Hz"
                      f"  det {len(state.get('detections') or [])}")
            if age is not None:
                label += f"  age {age:.2f}s"
        elif state.get('infer_ms'):
            label += f"  yolo {state['infer_ms']:.0f}ms"
        if state.get('held'):
            label += '  [HOLD]'
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
            now = time.monotonic()
            snapshot['markers'] = [
                self._decorate_marker(marker, now)
                for marker in (state.get('markers') or [])
            ]
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
                    inference_age = (
                        None if not state.get('inference_wall')
                        else max(0.0, now - state['inference_wall']))
                    transport_age = state.get('transport_age_s')
                    source_age = (
                        None if transport_age is None or inference_age is None
                        else round(float(transport_age) + inference_age, 3))
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
                            {k: v for k, v in
                             self._decorate_marker(m, now).items()
                             if k != 'corners'}
                            for m in (state['markers'] or [])
                        ] if now - state['marker_wall'] <= self.stale_after else [],
                        'detections': (
                            state['detections']
                            if now - state['detection_wall'] <= self.stale_after
                            else []),
                        'held': bool(state.get('held')),
                        'infer_ms': float(state.get('infer_ms') or 0.0),
                        'infer_age_s': (None if inference_age is None
                                        else round(inference_age, 2)),
                        'detection_live': bool(
                            inference_age is not None
                            and inference_age <= self.stale_after),
                        'detection_source': state.get('detection_source', 'off'),
                        'detection_topic': state.get('detection_topic', ''),
                        'detection_camera_id': state.get(
                            'detection_camera_id', ''),
                        'detection_sequence': int(state.get(
                            'detection_sequence', 0)),
                        'detection_messages': int(state.get(
                            'detection_messages', 0)),
                        'detection_invalid': int(state.get(
                            'detection_invalid', 0)),
                        'detection_dropped': int(state.get(
                            'detection_dropped', 0)),
                        'detection_rate_hz': round(float(state.get(
                            'detection_rate_hz', 0.0)), 2),
                        'source_age_s': source_age,
                        'transport_age_s': transport_age,
                        'homography_ok': state.get('homography_ok'),
                        'detection_error': state.get('detection_error', ''),
                    })
                pose = None if self.relative_pose is None \
                    else dict(self.relative_pose)
                pose_age = None if self.relative_pose_wall <= 0.0 \
                    else max(0.0, now - self.relative_pose_wall)
                visible_age = None if self.marker_visible_wall <= 0.0 \
                    else max(0.0, now - self.marker_visible_wall)
                visible = self.marker_visible \
                    if (visible_age is not None and
                        visible_age <= self.stale_after) else None
                drive_markers = self._drive_marker_summary(payload, now)
            relative_pose = {
                'configured': bool(self.relative_pose_topic),
                'topic': self.relative_pose_topic,
                'visible_topic': self.marker_visible_topic,
                'fresh': bool(pose is not None and pose_age is not None and
                              pose_age <= self.stale_after),
                'visible': visible,
                'age_s': pose_age,
            }
            if pose is not None:
                relative_pose.update(pose)
            # 같은 ID가 두 카메라에 동시에 보이면 그 지점이 겹침 영역이다.
            seen = {}
            for cam in payload:
                for marker in cam['markers']:
                    seen.setdefault(marker['id'], []).append(cam['label'])
            shared = sorted(mid for mid, cams in seen.items() if len(cams) > 1)
            live_detection_labels = [
                camera['label'] for camera in payload
                if camera['detection_live']]
            if self.detection_source == 'production':
                detection_ready = bool(live_detection_labels)
                detection_error = (
                    '' if detection_ready else 'Production 검출 토픽 수신 대기')
            else:
                detection_ready = self.yolo is not None
                detection_error = self.yolo_error
            return jsonify({
                'cameras': payload,
                'grid_step_px': self.grid_step,
                'marker_size_m': self.marker_size_m,
                'shared_marker_ids': shared,
                'tracks': {
                    label: {
                        'ref': track.get('ref'),
                        'last': track.get('last'),
                        'moved_m': round(track.get('moved_m', 0.0), 4),
                        'path_m': round(track.get('path_m', 0.0), 4),
                        'samples': track.get('samples', 0),
                    }
                    for label, track in self.tracks.items()
                },
                'relative_pose': relative_pose,
                'drive_markers': drive_markers,
                'yolo': {
                    'ready': detection_ready,
                    'error': detection_error,
                    'source': self.detection_source,
                    'live_cameras': live_detection_labels,
                    'configured_topics': dict(self.external_detection_topics),
                    'classes': (sorted(self.yolo_class_ids)
                                if self.yolo is not None else []),
                    'cameras': sorted(self.yolo_cameras) or 'all',
                    'switch_mode': self.yolo_switch_mode,
                    'active': self._yolo_active,
                    'scanning': self._yolo_scanning,
                    'regions': {k: list(v)
                                for k, v in self.yolo_regions.items()},
                    'regions_file': self.yolo_regions_file,
                    'mission_cameras': dict(self.mission_cameras),
                    'mission_type': self._mission_type,
                    'mission_state': self._mission_state,
                    'mission_fresh': (
                        (time.monotonic() - self._mission_wall)
                        <= self.fleet_state_timeout_s),
                    'fleet_state_topic': self.fleet_state_topic,
                    'regions_source': self._regions_source,
                    'labels': list(self._yolo_labels),
                    'target': self._yolo_target,
                    'controls_enabled': self.detection_source == 'internal',
                },
                'slots': {
                    'ready': self.slot_tracker is not None,
                    'items': [
                        {'id': slot_id,
                         'observed': self.slot_state.get(
                             slot_id, {}).get('observed', False),
                         'occupied': self.slot_state.get(
                             slot_id, {}).get('occupied', True)}
                        for slot_id, _ in self.slots],
                    'empty': self.empty_slot_ids(),
                    'coverage_cameras': sorted(self.camera_coverage),
                    'overlap_threshold': self.slot_overlap_threshold,
                    'confirm_frames': self.slot_empty_confirm_frames,
                },
                'parallax': {
                    'configured': bool(self.camera_optics),
                    'vehicle_height_m': self.vehicle_detection_height,
                    'marker_height_m': self.marker_height,
                    'cameras': {
                        label: {'ground': [optics[0], optics[1]],
                                'height_m': optics[2],
                                'vehicle_edge_shift_m': self.parallax_shift_m(
                                    label, self.vehicle_detection_height),
                                'marker_edge_shift_m': self.parallax_shift_m(
                                    label, self.marker_height)}
                        for label, optics in self.camera_optics.items()},
                    # 같은 마커를 두 카메라가 볼 때 두 추정 사이의 거리.
                    # 높이 값이 맞을수록 0 에 가까워지므로 조정 기준이 된다.
                    'marker_disagreement_m': self._marker_disagreement(),
                },
                'guidance': self._guidance(time.monotonic()),
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

        @app.post('/api/track_reset')
        def track_reset():
            """모든 카메라의 이동 기준점을 현재 위치로 옮긴다."""
            for track in self.tracks.values():
                track['reset_pending'] = True
                track['path_m'] = 0.0
                track['moved_m'] = 0.0
            return jsonify({'reset': sorted(self.tracks)})

        @app.post('/api/yolo_region/<label>')
        def set_region(label):
            body = request.get_json(silent=True) or {}
            try:
                rect = self.set_yolo_region(
                    label, body.get('x1'), body.get('y1'),
                    body.get('x2'), body.get('y2'))
            except (TypeError, ValueError) as exc:
                return jsonify({'error': str(exc)}), 400
            return jsonify({'label': label, 'region': list(rect),
                            'csv': format_yolo_regions(self.yolo_regions)})

        @app.post('/api/yolo_region_clear/<label>')
        def clear_region(label):
            self.clear_yolo_region(label)
            return jsonify({'csv': format_yolo_regions(self.yolo_regions),
                            'switch_mode': self.yolo_switch_mode})

        @app.post('/api/yolo_region_save')
        def save_regions():
            try:
                return jsonify(self.save_yolo_regions())
            except OSError as exc:
                return jsonify({'error': f'저장 실패: {exc}'}), 500

        @app.post('/api/yolo_switch_mode/<mode>')
        def set_switch_mode(mode):
            try:
                return jsonify({'switch_mode': self.set_yolo_switch_mode(mode)})
            except ValueError as exc:
                return jsonify({'error': str(exc)}), 400

        @app.post('/api/guidance_mission/<mission>')
        def set_guidance_mission(mission):
            value = str(mission).strip().lower()
            if value in ('auto', ''):
                self.guidance_forced_mission = ''
            elif value in (MISSION_PARK, MISSION_RETRIEVE):
                self.guidance_forced_mission = value
            else:
                return jsonify({
                    'error': "mission 은 'park' | 'retrieve' | 'auto'"}), 400
            self.get_logger().info(
                '안내 미션 고정: '
                + (self.guidance_forced_mission or '자동(fleet 따름)'))
            return jsonify({'forced': self.guidance_forced_mission,
                            'mission': (self.guidance_forced_mission
                                        or self._mission_type)})

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
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
