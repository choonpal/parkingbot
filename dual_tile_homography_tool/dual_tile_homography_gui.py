#!/usr/bin/env python3
"""Jetson Orin Nano dual-CCTV tile-grid Homography GUI.

- CAM0/CAM2를 한 프로그램에서 관리
- 각 camera_calibration.npz를 먼저 적용(undistort)
- 40 cm 바닥 타일 번호(i,j) -> world X,Y 자동 계산
- 카메라별 Homography(H0/H2) 계산
- 겹침 영역 같은 타일 꼭짓점의 cam0/cam2 좌표 차이 검증
- parkingbot 런타임용 homography_cam{0,2}_rectified.npy 저장
"""
from __future__ import annotations

import argparse, hashlib, json, math, shutil, threading, time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server

HTML = r'''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dual CCTV Tile Homography</title><style>
:root{color-scheme:dark;--bg:#0e141b;--panel:#18212d;--line:#344358;--blue:#48a7ff;--green:#4fd184;--orange:#ffb454;--red:#ff7373}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#eef4fb;font-family:system-ui,sans-serif}header{padding:13px 18px;border-bottom:1px solid var(--line)}h1{font-size:20px;margin:0}main{display:grid;grid-template-columns:410px 1fr;gap:14px;padding:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;min-width:0}.row{display:flex;gap:7px;flex-wrap:wrap;align-items:center;margin:7px 0}h2{font-size:15px;margin:17px 0 8px}button,input{background:#0d141e;color:#eef4fb;border:1px solid #43546c;border-radius:6px;padding:8px}button{cursor:pointer}button.primary{border-color:var(--blue)}button.good{border-color:var(--green)}button.warn{border-color:var(--orange)}button.active{background:#244e73;border-color:var(--blue)}input[type=number]{width:90px}.small{font-size:12px;color:#afbed0;line-height:1.45}#status{white-space:pre-wrap;background:#0d141e;border-radius:7px;padding:10px;min-height:86px}canvas{width:100%;height:auto;border:1px solid var(--line);border-radius:8px;cursor:crosshair;background:#05070a}#preview{max-width:100%;border:1px solid var(--line);border-radius:8px;display:none}.tag{display:inline-block;padding:3px 7px;margin:2px;background:#263448;border-radius:11px;font-size:12px}.bad{color:var(--red)}table{width:100%;border-collapse:collapse}th,td{font-size:12px;border-bottom:1px solid #2e3b4d;padding:5px;text-align:right}th:first-child,td:first-child{text-align:left}@media(max-width:980px){main{grid-template-columns:1fr}}
</style></head><body><header><h1>Dual CCTV · 40cm Tile Grid Homography <span id="modeBadge" class="tag"></span></h1></header><main>
<section class="panel">
<div class="row"><button id="cam0Btn" onclick="selectCam('cam0')">CAM0 보기</button><button id="cam2Btn" class="active" onclick="selectCam('cam2')">CAM2 재등록</button><button id="snapshotCurrentBtn" class="primary" onclick="snapshotCurrent()">선택 CAM만 영상 정지</button><button onclick="snapshotAll()">두 영상 정지(H 보존)</button></div>
<p class="small">CAM2 전용 모드에서는 기존 CAM0 H를 잠급니다. 새 CAM2 H를 저장해도 CAM0 파일은 byte 단위로 동일한지 검증합니다.</p><div id="hState" class="small"></div>
<h2>1. 공통 타일 좌표계</h2><div class="row"><label>Pitch(m) <input id="pitch" type="number" value="0.400" step="0.001" onchange="updateWorld()"></label><label>Origin X <input id="ox" type="number" value="0.000" step="0.01" onchange="updateWorld()"></label><label>Origin Y <input id="oy" type="number" value="0.000" step="0.01" onchange="updateWorld()"></label></div>
<p class="small">권장: 바닥의 공통 타일 꼭짓점 하나를 Tile(0,0) = Map(0,0)m로 정합니다.</p>
<h2>2. 기준점</h2><div class="row"><button id="refBtn" class="active" onclick="setMode('ref')">기준점</button><button id="measureBtn" onclick="setMode('measure')">X,Y 측정</button><button id="overlapBtn" onclick="setMode('overlap')">겹침 검증</button></div>
<div class="row"><label>Tile i <input id="ti" type="number" value="0" step="1" onchange="updateWorld()"></label><label>Tile j <input id="tj" type="number" value="0" step="1" onchange="updateWorld()"></label></div><div id="world">World=(0.000,0.000)m</div>
<div class="row"><button class="good" onclick="addRef()">클릭점 + Tile 등록</button><button onclick="undoRef()">마지막 취소</button><button onclick="clearRefs()">현재 CAM 삭제</button></div><div id="refs" class="small">기준점 0개</div>
<h2>3. Homography</h2><div class="row"><button id="calcBtn" class="good" onclick="calcH()">현재 CAM H 계산</button><button onclick="showPreview()">선택 CAM BEV</button><button class="primary" onclick="showComposite()">CAM0 + CAM2 전체 합성</button><button onclick="showOverlapDiagnostic()">겹침 진단</button><button id="saveCurrentBtn" class="warn" onclick="saveCurrent()">CAM2만 저장</button><button id="saveAllBtn" onclick="saveAll()">H0 + H2 전체 저장</button></div>
<h2>4. 겹침 검증</h2><p class="small">H0/H2 생성 후 같은 overlap Tile(i,j)를 CAM0에서 클릭하고 CAM2에서도 같은 꼭짓점을 클릭하세요.</p><div id="overlap" class="small">검증점 없음</div>
<h2>5. 측정 결과</h2><div style="max-height:210px;overflow:auto"><table><thead><tr><th>CAM</th><th>u</th><th>v</th><th>X</th><th>Y</th></tr></thead><tbody id="rows"></tbody></table></div>
<h2>상태</h2><div id="status">기존 H와 스냅샷을 불러오는 중입니다.</div></section>
<section class="panel"><h2 id="imageTitle" style="margin-top:0">CAM2 · calibration 적용 정지 영상</h2><canvas id="canvas"></canvas><h2>BEV / 합성 미리보기</h2><p class="small">합성 화면: CAM0=청록, CAM2=빨강, 같은 구조가 맞으면 회색으로 겹칩니다.</p><img id="preview"></section>
</main><script>
const c=document.getElementById('canvas'),ctx=c.getContext('2d');let img=new Image(),cam='cam2',mode='ref',pending=null,refs={cam0:[],cam2:[]},ready={cam0:false,cam2:false},snapshots={cam0:false,cam2:false},hSource={cam0:'없음',cam2:'없음'},measures=[],overlap={},seq=0,cam2Only=false,previewUrl=null;
function stat(s,b=false){let e=document.getElementById('status');e.textContent=s;e.className=b?'bad':''}async function api(u,o={}){let r=await fetch(u,o),b=await r.json().catch(()=>({error:'응답 오류'}));if(!r.ok)throw new Error(b.error||r.statusText);return b}function post(u,p={}){return api(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)})}function locked(){return cam2Only&&cam==='cam0'}
function pitch(){return Number(document.getElementById('pitch').value)}function ox(){return Number(document.getElementById('ox').value)}function oy(){return Number(document.getElementById('oy').value)}function ti(){return Number(document.getElementById('ti').value)}function tj(){return Number(document.getElementById('tj').value)}function world(i,j){return[ox()+i*pitch(),oy()+j*pitch()]}function updateWorld(){let w=world(ti(),tj());document.getElementById('world').textContent=`World=(${w[0].toFixed(3)},${w[1].toFixed(3)})m`}
function renderHState(){document.getElementById('modeBadge').textContent=cam2Only?'CAM2 ONLY · CAM0 LOCKED':'FULL MODE';document.getElementById('hState').innerHTML=['cam0','cam2'].map(x=>`<span class="tag">${x.toUpperCase()} H: ${ready[x]?'준비됨':'재계산 필요'} · ${hSource[x]||'없음'}${cam2Only&&x==='cam0'?' · 잠금':''}</span>`).join('');document.getElementById('calcBtn').disabled=locked();document.getElementById('snapshotCurrentBtn').disabled=locked();document.getElementById('saveCurrentBtn').disabled=locked();document.getElementById('saveCurrentBtn').textContent=cam2Only?'CAM2만 저장':`${cam.toUpperCase()}만 저장`;document.getElementById('saveAllBtn').style.display=cam2Only?'none':'inline-block'}
function selectCam(x){cam=x;pending=null;document.getElementById('cam0Btn').classList.toggle('active',x==='cam0');document.getElementById('cam2Btn').classList.toggle('active',x==='cam2');document.getElementById('imageTitle').textContent=`${x.toUpperCase()} · calibration 적용 정지 영상${locked()?' · H 잠금':''}`;renderHState();renderRefs();loadImg()}
function setMode(m){mode=m;pending=null;document.getElementById('refBtn').classList.toggle('active',m==='ref');document.getElementById('measureBtn').classList.toggle('active',m==='measure');document.getElementById('overlapBtn').classList.toggle('active',m==='overlap');draw();stat(m==='ref'?'타일 꼭짓점을 클릭하고 Tile i,j를 등록하세요.':m==='measure'?'원하는 바닥점을 클릭하세요.':'같은 Tile i,j를 CAM0/CAM2에서 각각 클릭하세요.')}
async function snapshotAll(){try{let r=await post('/api/snapshot_all');seq=r.sequence;snapshots={cam0:true,cam2:true};if(cam2Only)refs.cam2=[];else refs={cam0:[],cam2:[]};measures=[];overlap={};pending=null;renderRefs();renderRows();renderOverlap();renderHState();loadImg();document.getElementById('preview').style.display='none';stat(`두 영상 정지 완료 · 기존 H 보존\nCAM0=${r.cam0.width}x${r.cam0.height}, CAM2=${r.cam2.width}x${r.cam2.height}\nCAM2 기준점 8~12개를 새로 등록하세요.`)}catch(e){stat(e.message,true)}}
async function snapshotCurrent(){if(locked()){stat('CAM0는 잠겨 있습니다.',true);return}try{let r=await post(`/api/snapshot/${cam}`);seq=r.sequence;snapshots[cam]=true;refs[cam]=[];measures=measures.filter(x=>x.cam!==cam);pending=null;renderRefs();renderRows();loadImg();stat(`${cam.toUpperCase()} 영상만 정지 완료 (${r.width}x${r.height})\n기존 H는 비교용으로 보존됐습니다. 새 기준점을 등록하세요.`)}catch(e){stat(e.message,true)}}
function loadImg(){ctx.clearRect(0,0,c.width,c.height);if(!snapshots[cam]){stat(`${cam.toUpperCase()} 정지 영상이 없습니다. 먼저 영상을 정지하세요.`,true);return}img=new Image();img.onload=()=>{c.width=img.naturalWidth;c.height=img.naturalHeight;draw()};img.onerror=()=>stat(`${cam.toUpperCase()} 정지 영상 로드 실패`,true);img.src=`/api/snapshot/${cam}.jpg?s=${seq}&t=${Date.now()}`}
c.addEventListener('click',async e=>{if(!img.naturalWidth)return;let r=c.getBoundingClientRect(),p=[(e.clientX-r.left)*c.width/r.width,(e.clientY-r.top)*c.height/r.height];if(mode==='ref'){if(locked()){stat('CAM0 기준점/H는 잠겨 있습니다. CAM2를 선택하세요.',true);return}pending=p;draw();stat(`${cam} pixel=(${p[0].toFixed(1)},${p[1].toFixed(1)})`);return}if(mode==='measure'){if(!ready[cam]){stat('현재 CAM H가 없습니다.',true);return}try{let a=await post(`/api/transform/${cam}`,{pixel:p});measures.push({cam,pixel:p,world:a.world_m});renderRows();draw();stat(`X=${a.world_m[0].toFixed(3)}m Y=${a.world_m[1].toFixed(3)}m`)}catch(x){stat(x.message,true)}return}if(mode==='overlap'){if(!(ready.cam0&&ready.cam2)){stat('H0/H2를 모두 먼저 계산하세요.',true);return}let i=ti(),j=tj(),k=`${i},${j}`;if(!Number.isInteger(i)||!Number.isInteger(j)){stat('Tile i,j는 정수여야 합니다.',true);return}overlap[k]??={tile:[i,j]};overlap[k][cam]=p;draw();if(overlap[k].cam0&&overlap[k].cam2){try{let a=await post('/api/overlap_check',{tile_i:i,tile_j:j,pitch_m:pitch(),origin_x_m:ox(),origin_y_m:oy(),cam0_pixel:overlap[k].cam0,cam2_pixel:overlap[k].cam2});overlap[k].result=a;renderOverlap();stat(`T(${i},${j}) CAM0↔CAM2 차이 ${(a.inter_camera_error_m*100).toFixed(2)}cm`)}catch(x){stat(x.message,true)}}else{renderOverlap();stat(`T(${i},${j}) ${cam} 클릭 저장. 다른 CAM에서 같은 점 클릭.`)}}})
function addRef(){if(locked()){stat('CAM0는 잠겨 있습니다. CAM2만 수정할 수 있습니다.',true);return}if(!pending){stat('먼저 타일 꼭짓점을 클릭하세요.',true);return}let i=ti(),j=tj();if(!Number.isInteger(i)||!Number.isInteger(j)){stat('Tile i,j는 정수여야 합니다.',true);return}if(refs[cam].some(r=>r.tile[0]===i&&r.tile[1]===j)){stat('현재 CAM에 같은 Tile이 이미 등록됨',true);return}let w=world(i,j);refs[cam].push({pixel:[...pending],tile:[i,j],world:w});pending=null;ready[cam]=false;hSource[cam]='새 기준점 · 재계산 필요';renderRefs();renderHState();draw();stat(`${cam} T(${i},${j}) → (${w[0].toFixed(3)},${w[1].toFixed(3)})m 등록`)}function undoRef(){if(locked())return;refs[cam].pop();ready[cam]=false;hSource[cam]='새 기준점 · 재계산 필요';renderRefs();renderHState();draw()}function clearRefs(){if(locked())return;refs[cam]=[];ready[cam]=false;hSource[cam]='기준점 없음';pending=null;renderRefs();renderHState();draw()}
function renderRefs(){document.getElementById('refs').innerHTML=refs[cam].length?refs[cam].map((r,i)=>`<span class="tag">R${i+1} T(${r.tile[0]},${r.tile[1]})→(${r.world[0].toFixed(2)},${r.world[1].toFixed(2)})</span>`).join(''):'기준점 0개'}
function renderRows(){document.getElementById('rows').innerHTML=measures.map(m=>`<tr><td>${m.cam}</td><td>${m.pixel[0].toFixed(1)}</td><td>${m.pixel[1].toFixed(1)}</td><td>${m.world[0].toFixed(3)}</td><td>${m.world[1].toFixed(3)}</td></tr>`).join('')}
function renderOverlap(){let ks=Object.keys(overlap);document.getElementById('overlap').innerHTML=ks.length?ks.map(k=>{let o=overlap[k],s=o.result?`Δcam=${(o.result.inter_camera_error_m*100).toFixed(2)}cm / Δ0=${(o.result.cam0_nominal_error_m*100).toFixed(2)} / Δ2=${(o.result.cam2_nominal_error_m*100).toFixed(2)}`:`${o.cam0?'cam0✓':'cam0-'} ${o.cam2?'cam2✓':'cam2-'}`;return`<span class="tag">T(${o.tile[0]},${o.tile[1]}) ${s}</span>`}).join(''):'검증점 없음'}
async function calcH(){if(locked()){stat('CAM0 H는 잠겨 있습니다.',true);return}try{let r=await post(`/api/homography/${cam}`,{references:refs[cam]});ready[cam]=true;hSource[cam]='현재 세션 계산값';renderHState();stat(`${cam.toUpperCase()} H 완료\n기준점=${r.reference_count} / RANSAC 유효=${r.inlier_count}\nRMS=${(r.rms_m*100).toFixed(2)}cm / MAX=${(r.max_error_m*100).toFixed(2)}cm\n저장 전에 CAM0+CAM2 합성으로 확인하세요.`);await showComposite()}catch(e){stat(e.message,true)}}
async function setPreview(url){try{let r=await fetch(`${url}${url.includes('?')?'&':'?'}t=${Date.now()}`);if(!r.ok){let b=await r.json().catch(()=>({error:r.statusText}));throw new Error(b.error||r.statusText)}let blob=await r.blob();if(previewUrl)URL.revokeObjectURL(previewUrl);previewUrl=URL.createObjectURL(blob);let p=document.getElementById('preview');p.src=previewUrl;p.style.display='block'}catch(e){stat(e.message,true)}}
async function showPreview(){if(!ready[cam]){stat('현재 CAM H가 없습니다.',true);return}await setPreview(`/api/preview/${cam}.jpg`)}async function showComposite(){if(!(ready.cam0&&ready.cam2)){stat('CAM0/CAM2 H가 모두 준비되어야 합성할 수 있습니다.',true);return}if(!(snapshots.cam0&&snapshots.cam2)){stat('두 카메라 정지 영상이 필요합니다.',true);return}await setPreview('/api/composite.jpg')}async function showOverlapDiagnostic(){if(!(ready.cam0&&ready.cam2)){stat('CAM0/CAM2 H가 모두 준비되어야 겹침을 진단할 수 있습니다.',true);return}await setPreview('/api/composite_diagnostic.jpg')}
async function saveCurrent(){if(locked()){stat('CAM0 파일은 잠겨 있습니다.',true);return}if(!ready[cam]){stat('새 H를 먼저 계산하세요.',true);return}try{let r=await post(`/api/save_camera/${cam}`,{pitch_m:pitch(),origin_x_m:ox(),origin_y_m:oy(),references:refs[cam]});hSource[cam]='저장된 새 파일';renderHState();stat(`${cam.toUpperCase()}만 저장 완료\nCAM0 보존 검증: ${r.cam0_preserved?'통과':'실패'}\n${r.npy}\n이전 ${cam.toUpperCase()} 백업: ${r.backup_dir||'없음'}\n런타임 적용 전 합성 화면을 마지막으로 확인하세요.`)}catch(e){stat(e.message,true)}}
async function saveAll(){try{let r=await post('/api/save_all',{pitch_m:pitch(),origin_x_m:ox(),origin_y_m:oy(),references:refs});stat(`전체 저장 완료\n${r.cam0_npy}\n${r.cam2_npy}\n${r.summary_json}`)}catch(e){stat(e.message,true)}}
function draw(){ctx.clearRect(0,0,c.width,c.height);if(img.naturalWidth)ctx.drawImage(img,0,0,c.width,c.height);ctx.font='16px system-ui';refs[cam].forEach((r,i)=>mark(r.pixel,`R${i+1} T(${r.tile[0]},${r.tile[1]})`,'#48a7ff'));if(pending)mark(pending,'R?','#ffb454');measures.filter(m=>m.cam===cam).forEach(m=>mark(m.pixel,`M(${m.world[0].toFixed(2)},${m.world[1].toFixed(2)})`,'#4fd184'));Object.values(overlap).forEach(o=>{if(o[cam])mark(o[cam],`O T(${o.tile[0]},${o.tile[1]})`,'#ff75d8')})}function mark(p,t,col){ctx.beginPath();ctx.arc(p[0],p[1],7,0,Math.PI*2);ctx.fillStyle=col;ctx.fill();ctx.fillText(t,p[0]+10,p[1]-8)}
async function boot(){try{let s=await api('/api/state');cam2Only=Boolean(s.cam2_only);seq=s.sequence||0;ready={cam0:Boolean(s.ready.cam0),cam2:Boolean(s.ready.cam2)};snapshots={cam0:Boolean(s.snapshots.cam0),cam2:Boolean(s.snapshots.cam2)};hSource=s.h_source||hSource;refs={cam0:Array.isArray(s.references.cam0)?s.references.cam0:[],cam2:Array.isArray(s.references.cam2)?s.references.cam2:[]};document.getElementById('pitch').value=Number(s.pitch_m||0.4).toFixed(3);document.getElementById('ox').value=Number(s.origin_world_m?.[0]||0).toFixed(3);document.getElementById('oy').value=Number(s.origin_world_m?.[1]||0).toFixed(3);updateWorld();selectCam(cam2Only?'cam2':'cam0');renderRows();renderOverlap();stat(`${cam2Only?'CAM2 전용 재등록':'전체 등록'} 모드 준비\nCAM0 H: ${hSource.cam0} / CAM2 H: ${hSource.cam2}\n${cam2Only?'CAM0는 읽기 전용입니다. CAM2 영상 정지 → 기준점 → H 계산 → 합성 → CAM2만 저장 순서로 진행하세요.':'두 카메라를 등록하세요.'}`);if(ready.cam0&&ready.cam2&&snapshots.cam0&&snapshots.cam2)await showComposite()}catch(e){stat(e.message,true)}}boot();
</script></body></html>'''


def load_calib(path: Path):
    with np.load(path, allow_pickle=False) as d:
        K=np.asarray(d['mtx'] if 'mtx' in d else d['camera_matrix'],np.float64)
        D=np.asarray(d['dist'] if 'dist' in d else d['dist_coeffs'],np.float64)
        size = None
        if 'image_width' in d and 'image_height' in d:
            size = (int(d['image_width']), int(d['image_height']))
    if K.shape!=(3,3) or D.size<4: raise RuntimeError(f'잘못된 calibration: {path}')
    return K,D,size


def parse_device(v):
    s=str(v).strip(); return int(s) if s.lstrip('-').isdigit() else s


class Camera:
    def __init__(self,label,device,calib,width=640,height=480,fps=30.0):
        self.label=label;self.device=parse_device(device);self.K,self.D,self.calibration_size=load_calib(Path(calib));self.width=width;self.height=height;self.fps=fps;self.latest=None;self.error='';self.lock=threading.RLock();self.running=True;self.cap=None
        if self.calibration_size is not None and self.calibration_size != (width,height):
            raise RuntimeError(f'{label} calibration 해상도 {self.calibration_size[0]}x{self.calibration_size[1]} != 실행 해상도 {width}x{height}')
        threading.Thread(target=self.loop,daemon=True).start()
    def open(self):
        b=cv2.CAP_V4L2 if hasattr(cv2,'CAP_V4L2') else 0;c=cv2.VideoCapture(self.device,b)
        if not c.isOpened(): c.release();c=cv2.VideoCapture(self.device)
        if not c.isOpened(): c.release();return None
        try:c.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:pass
        c.set(cv2.CAP_PROP_FRAME_WIDTH,self.width);c.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height);c.set(cv2.CAP_PROP_FPS,self.fps);c.set(cv2.CAP_PROP_BUFFERSIZE,1);return c
    def loop(self):
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self.cap=self.open()
                if self.cap is None:self.error=f'카메라 열기 실패: {self.device}';time.sleep(1);continue
            ok,f=self.cap.read()
            if not ok:self.error='frame read 실패';self.cap.release();self.cap=None;time.sleep(.2);continue
            if f.shape[1]!=self.width or f.shape[0]!=self.height:self.error=f'실제 해상도 {f.shape[1]}x{f.shape[0]} != 요청 {self.width}x{self.height}';time.sleep(.1);continue
            r=cv2.undistort(f,self.K,self.D)
            with self.lock:self.latest=r;self.error=''
    def get(self):
        with self.lock:return None if self.latest is None else self.latest.copy()
    def close(self):
        self.running=False
        if self.cap is not None:self.cap.release()


def validate_refs(raw):
    if not isinstance(raw,list) or len(raw)<4:raise ValueError('기준점 최소 4개 필요')
    src=[];dst=[];wp=set()
    for i,r in enumerate(raw):
        p=r.get('pixel');w=r.get('world')
        if not isinstance(p,list) or len(p)!=2 or not isinstance(w,list) or len(w)!=2:raise ValueError(f'reference {i} 형식 오류')
        src.append([float(p[0]),float(p[1])]);dst.append([float(w[0]),float(w[1])]);k=(round(float(w[0]),6),round(float(w[1]),6))
        if k in wp:raise ValueError('같은 Tile/world 좌표 중복')
        wp.add(k)
    src=np.asarray(src,np.float64);dst=np.asarray(dst,np.float64)
    if cv2.contourArea(cv2.convexHull(src.astype(np.float32)))<50:raise ValueError('픽셀 기준점이 너무 좁거나 거의 한 직선')
    if cv2.contourArea(cv2.convexHull(dst.astype(np.float32)))<0.02:raise ValueError('world 기준점이 너무 좁거나 거의 한 직선')
    return src,dst


def tx(H,pts):
    a=np.asarray(pts,np.float64).reshape(-1,1,2);o=cv2.perspectiveTransform(a,H).reshape(-1,2);return [[float(x),float(y)] for x,y in o]


def load_homography(path: Path):
    matrix = np.asarray(np.load(path, allow_pickle=False), dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError(f'잘못된 Homography: {path}')
    if abs(float(np.linalg.det(matrix))) < 1e-12:
        raise ValueError(f'특이 Homography: {path}')
    if abs(float(matrix[2, 2])) > 1e-12:
        matrix = matrix / float(matrix[2, 2])
    return matrix


def file_sha256(path: Path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


class Tool:
    def __init__(self,a):
        self.base = Path(__file__).resolve().parent
        self.out = Path(a.output_dir).expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.w, self.h = a.width, a.height
        self.map_w, self.map_h = a.map_width, a.map_height
        self.ppm = a.preview_ppm
        self.th = a.ransac_threshold_m
        self.port = a.port
        self.host = a.host
        self.cam2_only = bool(a.cam2_only)
        if self.w <= 0 or self.h <= 0:
            raise RuntimeError('카메라 해상도는 양수여야 합니다')
        self.cams = {
            'cam0': Camera(
                'cam0', a.cam0, a.cam0_calibration,
                self.w, self.h, a.fps),
            'cam2': Camera(
                'cam2', a.cam2, a.cam2_calibration,
                self.w, self.h, a.fps),
        }
        self.lock = threading.RLock()
        self.snap = {'cam0': None, 'cam2': None}
        self.H = {'cam0': None, 'cam2': None}
        self.meta = {'cam0': {}, 'cam2': {}}
        self.references = {'cam0': [], 'cam2': []}
        self.h_source = {'cam0': '없음', 'cam2': '없음'}
        self.pitch_m = 0.4
        self.origin_world_m = [0.0, 0.0]
        self.saved_camera_ns = {'cam0': 0, 'cam2': 0}
        self.seq = 0
        self._load_existing_outputs()
        if self.cam2_only and self.H['cam0'] is None:
            raise RuntimeError(
                '--cam2-only 모드는 기존 cam0 Homography가 필요합니다: '
                f'{self.out / "homography_cam0_rectified.npy"}')
        self.app = self.web()
        self.server = make_server(
            self.host, self.port, self.app, threaded=True)

    def _load_existing_outputs(self):
        summary_path = self.out / 'dual_homography_summary.json'
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding='utf-8'))
                self.pitch_m = float(summary.get('tile_pitch_m', 0.4))
                origin = summary.get('origin_world_m', [0.0, 0.0])
                if isinstance(origin, list) and len(origin) == 2:
                    self.origin_world_m = [float(origin[0]), float(origin[1])]
                raw_refs = summary.get('references', {})
                raw_meta = summary.get('fit_meta', {})
                raw_saved = summary.get('saved_camera_unix_ns', {})
                for cam in ('cam0', 'cam2'):
                    if isinstance(raw_refs.get(cam), list):
                        self.references[cam] = raw_refs[cam]
                    if isinstance(raw_meta.get(cam), dict):
                        self.meta[cam] = raw_meta[cam]
                    if isinstance(raw_saved, dict):
                        self.saved_camera_ns[cam] = int(raw_saved.get(cam, 0))
            except Exception as exc:
                print(f'기존 summary 로드 경고: {exc}')

        for cam in ('cam0', 'cam2'):
            matrix_path = self.out / f'homography_{cam}_rectified.npy'
            if matrix_path.is_file():
                try:
                    self.H[cam] = load_homography(matrix_path)
                    self.h_source[cam] = '기존 파일'
                except Exception as exc:
                    print(f'{cam} 기존 H 로드 경고: {exc}')
            snapshot_path = self.out / f'{cam}_rectified_snapshot.jpg'
            if snapshot_path.is_file():
                frame = cv2.imread(str(snapshot_path), cv2.IMREAD_COLOR)
                if frame is not None and frame.shape[:2] == (self.h, self.w):
                    self.snap[cam] = frame

    def require(self,cam,kind):
        if cam not in self.cams:
            raise ValueError('cam must be cam0/cam2')
        with self.lock:
            x=self.snap[cam] if kind=='snap' else self.H[cam]
            if x is None:raise ValueError(f'{cam} {kind} 없음')
            return x.copy()
    def jpg(self,f):
        ok,e=cv2.imencode('.jpg',f,[cv2.IMWRITE_JPEG_QUALITY,90]);
        if not ok:raise ValueError('JPEG 실패')
        return Response(e.tobytes(),mimetype='image/jpeg')
    def fit(self,raw):
        s,d=validate_refs(raw)
        if len(s)==4:H,m=cv2.findHomography(s,d,0)
        else:H,m=cv2.findHomography(s,d,cv2.RANSAC,ransacReprojThreshold=self.th,maxIters=5000,confidence=.995)
        if H is None or not np.all(np.isfinite(H)):raise ValueError('Homography 계산 실패')
        pred=np.asarray(tx(H,s.tolist()));err=np.linalg.norm(pred-d,axis=1);mask=np.ones(len(s),bool) if m is None else m.reshape(-1).astype(bool)
        if mask.sum()<4:raise ValueError('RANSAC 유효점 4개 미만')
        return H,{'reference_count':len(s),'inlier_count':int(mask.sum()),'rms_m':float(np.sqrt(np.mean(err[mask]**2))),'max_error_m':float(err[mask].max()),'rms_all_m':float(np.sqrt(np.mean(err**2))),'max_all_m':float(err.max()),'errors_m':err.tolist(),'inliers':mask.tolist()}

    def _bev_geometry(self):
        width = max(1, int(round(self.map_w * self.ppm)))
        height = max(1, int(round(self.map_h * self.ppm)))
        metre_to_bev = np.array([
            [self.ppm, 0.0, 0.0],
            [0.0, -self.ppm, height - 1.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        return width, height, metre_to_bev

    def _warp(self, cam):
        frame = self.require(cam, 'snap')
        matrix = self.require(cam, 'H')
        width, height, metre_to_bev = self._bev_geometry()
        transform = metre_to_bev @ matrix
        warped = cv2.warpPerspective(
            frame, transform, (width, height),
            flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        source_mask = np.full(frame.shape[:2], 255, dtype=np.uint8)
        mask = cv2.warpPerspective(
            source_mask, transform, (width, height),
            flags=cv2.INTER_NEAREST, borderValue=0)
        return warped, mask

    def _composite(self):
        warped0, mask0 = self._warp('cam0')
        warped2, mask2 = self._warp('cam2')
        canvas = np.zeros_like(warped0)
        valid0 = mask0 > 0
        valid2 = mask2 > 0
        only0 = valid0 & ~valid2
        only2 = valid2 & ~valid0
        overlap = valid0 & valid2
        canvas[only0] = warped0[only0]
        canvas[only2] = warped2[only2]
        blended = cv2.addWeighted(warped0, 0.5, warped2, 0.5, 0.0)
        canvas[overlap] = blended[overlap]
        self._draw_grid(canvas)
        union_px = int(np.count_nonzero(valid0 | valid2))
        overlap_px = int(np.count_nonzero(overlap))
        cv2.putText(
            canvas, 'FULL UNION: CAM0 + CAM2  (OVERLAP 50/50)', (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (20, 20, 20), 2,
            cv2.LINE_AA)
        cv2.putText(
            canvas, 'FULL UNION: CAM0 + CAM2  (OVERLAP 50/50)', (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (245, 245, 245), 1,
            cv2.LINE_AA)
        cv2.putText(
            canvas,
            f'union={union_px / (self.ppm ** 2):.2f}m2  '
            f'overlap={overlap_px / (self.ppm ** 2):.2f}m2',
            (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
            (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f'union={union_px / (self.ppm ** 2):.2f}m2  '
            f'overlap={overlap_px / (self.ppm ** 2):.2f}m2',
            (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
            (235, 235, 235), 1, cv2.LINE_AA)
        return canvas

    def _draw_grid(self, canvas):
        for x_m in np.arange(0.0, self.map_w + 1e-9, 0.4):
            x = int(round(x_m * self.ppm))
            cv2.line(canvas, (x, 0), (x, canvas.shape[0] - 1),
                     (80, 80, 80), 1)
        for y_m in np.arange(0.0, self.map_h + 1e-9, 0.4):
            y = canvas.shape[0] - 1 - int(round(y_m * self.ppm))
            cv2.line(canvas, (0, y), (canvas.shape[1] - 1, y),
                     (80, 80, 80), 1)

    def _composite_diagnostic(self):
        warped0, mask0 = self._warp('cam0')
        warped2, mask2 = self._warp('cam2')
        gray0 = cv2.cvtColor(warped0, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(warped2, cv2.COLOR_BGR2GRAY)
        canvas = np.zeros_like(warped0)
        valid0 = mask0 > 0
        valid2 = mask2 > 0
        canvas[..., 0][valid0] = gray0[valid0]
        canvas[..., 1][valid0] = gray0[valid0]
        canvas[..., 2][valid2] = gray2[valid2]
        self._draw_grid(canvas)
        overlap_px = int(np.count_nonzero(valid0 & valid2))
        cv2.putText(
            canvas, 'CAM0=CYAN  CAM2=RED  ALIGNED=GRAY', (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (245, 245, 245), 1,
            cv2.LINE_AA)
        cv2.putText(
            canvas, f'overlap={overlap_px / (self.ppm ** 2):.2f}m2',
            (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
            (210, 210, 210), 1, cv2.LINE_AA)
        return canvas

    def _backup(self, cam):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup_dir = self.out / 'backups' / f'{stamp}_{cam}'
        candidates = [
            self.out / f'homography_{cam}_rectified.npy',
            self.out / f'homography_{cam}_rectified.npz',
            self.out / f'{cam}_rectified_snapshot.jpg',
            self.out / 'dual_homography_summary.json',
        ]
        existing = [path for path in candidates if path.is_file()]
        if existing:
            backup_dir.mkdir(parents=True, exist_ok=True)
            for path in existing:
                shutil.copy2(path, backup_dir / path.name)
            return str(backup_dir)
        return ''

    def _save_matrix(self, cam):
        matrix = self.require(cam, 'H')
        npy_path = self.out / f'homography_{cam}_rectified.npy'
        npz_path = self.out / f'homography_{cam}_rectified.npz'
        np.save(npy_path, matrix, allow_pickle=False)
        np.savez(npz_path, H=matrix)
        self.saved_camera_ns[cam] = time.time_ns()
        return npy_path, npz_path

    def _write_summary(self):
        h0 = self.require('cam0', 'H')
        h2 = self.require('cam2', 'H')
        summary = {
            'format': 'dual_tile_homography_v2',
            'tile_pitch_m': self.pitch_m,
            'origin_world_m': self.origin_world_m,
            'image_size': [self.w, self.h],
            'map_size_m': [self.map_w, self.map_h],
            'cam0_H': h0.tolist(),
            'cam2_H': h2.tolist(),
            'references': self.references,
            'fit_meta': self.meta,
            'saved_camera_unix_ns': self.saved_camera_ns,
            'saved_unix_ns': time.time_ns(),
        }
        path = self.out / 'dual_homography_summary.json'
        path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        return path

    def web(self):
        app=Flask('dual_tile_homography')
        @app.get('/')
        def home():return Response(HTML,mimetype='text/html; charset=utf-8')
        @app.get('/api/state')
        def state():
            return jsonify(
                cam2_only=self.cam2_only,
                sequence=self.seq,
                ready={cam: self.H[cam] is not None for cam in self.cams},
                snapshots={cam: self.snap[cam] is not None for cam in self.cams},
                references=self.references,
                h_source=self.h_source,
                fit_meta=self.meta,
                pitch_m=self.pitch_m,
                origin_world_m=self.origin_world_m,
                map_size_m=[self.map_w, self.map_h])
        @app.post('/api/snapshot_all')
        def snapall():
            c0,c2=self.cams['cam0'].get(),self.cams['cam2'].get()
            if c0 is None or c2 is None:return jsonify(error=f"두 카메라 프레임 필요: cam0={self.cams['cam0'].error or '대기'}, cam2={self.cams['cam2'].error or '대기'}"),503
            with self.lock:
                self.snap={'cam0':c0,'cam2':c2}
                self.seq+=1
            return jsonify(sequence=self.seq,cam0={'width':c0.shape[1],'height':c0.shape[0]},cam2={'width':c2.shape[1],'height':c2.shape[0]},existing_h_preserved=True)
        @app.post('/api/snapshot/<cam>')
        def snapshot_one(cam):
            try:
                if self.cam2_only and cam != 'cam2':
                    raise ValueError('cam2-only 모드에서는 CAM0를 변경하지 않습니다')
                if cam not in self.cams:
                    raise ValueError('cam must be cam0/cam2')
                frame = self.cams[cam].get()
                if frame is None:
                    raise ValueError(self.cams[cam].error or f'{cam} 프레임 대기')
                with self.lock:
                    self.snap[cam] = frame
                    self.seq += 1
                return jsonify(
                    sequence=self.seq, cam=cam,
                    width=frame.shape[1], height=frame.shape[0],
                    existing_h_preserved=True)
            except Exception as exc:
                return jsonify(error=str(exc)), 400
        @app.get('/api/snapshot/<cam>.jpg')
        def snapjpg(cam):
            try:return self.jpg(self.require(cam,'snap'))
            except Exception as e:return jsonify(error=str(e)),404
        @app.post('/api/homography/<cam>')
        def homography(cam):
            try:
                if self.cam2_only and cam != 'cam2':
                    raise ValueError('CAM0 H는 잠겨 있습니다. CAM2만 다시 계산하세요')
                raw_refs=(request.get_json(force=True) or {}).get('references',[])
                H,m=self.fit(raw_refs)
                with self.lock:
                    self.H[cam]=H
                    self.meta[cam]=m
                    self.references[cam]=raw_refs
                    self.h_source[cam]='현재 세션 계산값'
                return jsonify(matrix=H.tolist(),**m)
            except Exception as e:return jsonify(error=str(e)),400
        @app.post('/api/transform/<cam>')
        def transform(cam):
            try:return jsonify(world_m=tx(self.require(cam,'H'),[(request.get_json(force=True) or {})['pixel']])[0])
            except Exception as e:return jsonify(error=str(e)),400
        @app.post('/api/overlap_check')
        def overlap_check():
            try:
                q=request.get_json(force=True) or {};i,j=int(q['tile_i']),int(q['tile_j']);p=float(q['pitch_m']);nom=np.array([float(q.get('origin_x_m',0))+i*p,float(q.get('origin_y_m',0))+j*p]);w0=np.array(tx(self.require('cam0','H'),[q['cam0_pixel']])[0]);w2=np.array(tx(self.require('cam2','H'),[q['cam2_pixel']])[0]);return jsonify(tile=[i,j],nominal_world_m=nom.tolist(),cam0_world_m=w0.tolist(),cam2_world_m=w2.tolist(),cam0_nominal_error_m=float(np.linalg.norm(w0-nom)),cam2_nominal_error_m=float(np.linalg.norm(w2-nom)),inter_camera_error_m=float(np.linalg.norm(w0-w2)))
            except Exception as e:return jsonify(error=str(e)),400
        @app.get('/api/preview/<cam>.jpg')
        def preview(cam):
            try:
                warped,_=self._warp(cam)
                return self.jpg(warped)
            except Exception as e:return jsonify(error=str(e)),404
        @app.get('/api/composite.jpg')
        def composite():
            try:return self.jpg(self._composite())
            except Exception as e:return jsonify(error=str(e)),404
        @app.get('/api/composite_diagnostic.jpg')
        def composite_diagnostic():
            try:return self.jpg(self._composite_diagnostic())
            except Exception as e:return jsonify(error=str(e)),404
        @app.post('/api/save_camera/<cam>')
        def save_camera(cam):
            try:
                if self.cam2_only and cam != 'cam2':
                    raise ValueError('CAM0 파일은 잠겨 있습니다')
                q=request.get_json(force=True) or {}
                self.require('cam0','H');self.require('cam2','H')
                refs=q.get('references')
                if isinstance(refs,list) and refs:
                    self.references[cam]=refs
                self.pitch_m=float(q.get('pitch_m',self.pitch_m))
                self.origin_world_m=[float(q.get('origin_x_m',self.origin_world_m[0])),float(q.get('origin_y_m',self.origin_world_m[1]))]
                cam0_path=self.out/'homography_cam0_rectified.npy'
                cam0_sha_before=file_sha256(cam0_path) if cam0_path.is_file() else ''
                backup=self._backup(cam)
                npy_path,npz_path=self._save_matrix(cam)
                frame=self.snap.get(cam)
                if frame is not None:cv2.imwrite(str(self.out/f'{cam}_rectified_snapshot.jpg'),frame)
                summary_path=self._write_summary()
                cam0_sha_after=file_sha256(cam0_path) if cam0_path.is_file() else ''
                if cam!='cam0' and cam0_sha_before!=cam0_sha_after:
                    raise RuntimeError('CAM0 파일 보존 검증 실패')
                return jsonify(cam=cam,npy=str(npy_path),npz=str(npz_path),summary_json=str(summary_path),backup_dir=backup,cam0_preserved=(cam0_sha_before==cam0_sha_after))
            except Exception as e:return jsonify(error=str(e)),400
        @app.post('/api/save_all')
        def saveall():
            try:
                if self.cam2_only:raise ValueError('cam2-only 모드에서는 CAM0+CAM2 전체 저장이 차단됩니다')
                q=request.get_json(force=True) or {};self.require('cam0','H');self.require('cam2','H');self.pitch_m=float(q.get('pitch_m',.4));self.origin_world_m=[float(q.get('origin_x_m',0)),float(q.get('origin_y_m',0))];raw_refs=q.get('references',{});self.references.update(raw_refs if isinstance(raw_refs,dict) else {});self._backup('cam0');self._backup('cam2');p0,_=self._save_matrix('cam0');p2,_=self._save_matrix('cam2');sp=self._write_summary()
                with self.lock:
                    for cam in ('cam0','cam2'):
                        if self.snap[cam] is not None:cv2.imwrite(str(self.out/f'{cam}_rectified_snapshot.jpg'),self.snap[cam])
                return jsonify(cam0_npy=str(p0),cam2_npy=str(p2),summary_json=str(sp))
            except Exception as e:return jsonify(error=str(e)),400
        return app
    def run(self):
        print('='*70)
        print('Dual CCTV Tile Homography GUI')
        print(f'Mode: {"CAM2 ONLY (CAM0 LOCKED)" if self.cam2_only else "CAM0 + CAM2"}')
        print(f'Bind: {self.host}:{self.port}')
        print(f'Local: http://127.0.0.1:{self.port}')
        print(f'Output: {self.out}');print('='*70)
        try:self.server.serve_forever()
        finally:
            for c in self.cams.values():c.close()


def self_test(base):
    for f in ('cctv0_camera_calibration.npz','cctv2_camera_calibration.npz'):K,D,_=load_calib(base/f);assert K.shape==(3,3) and D.size>=4
    world=np.array([[0,0],[1.6,0],[3.2,0],[0,1.6],[1.6,1.6],[3.2,1.6],[0,3.2],[1.6,3.2],[3.2,3.2]],np.float64);Hw=np.array([[120,7,80],[4,90,50],[.01,.02,1]],np.float64);pix=cv2.perspectiveTransform(world.reshape(-1,1,2),Hw).reshape(-1,2);H,_=cv2.findHomography(pix,world,0);rest=cv2.perspectiveTransform(pix.reshape(-1,1,2),H).reshape(-1,2);assert np.linalg.norm(rest-world,axis=1).max()<1e-5
    test_path=base/'output'/'homography_cam0_rectified.npy'
    if test_path.is_file():assert load_homography(test_path).shape==(3,3)
    print('SELF TEST OK')


def main():
    b=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--cam0',default='0');p.add_argument('--cam2',default='2');p.add_argument('--cam0-calibration',default=str(b/'cctv0_camera_calibration.npz'));p.add_argument('--cam2-calibration',default=str(b/'cctv2_camera_calibration.npz'));p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480);p.add_argument('--fps',type=float,default=30);p.add_argument('--host',default='127.0.0.1',help='GUI bind address (기본값은 로컬 전용)');p.add_argument('--port',type=int,default=5001);p.add_argument('--output-dir',default=str(b/'output'));p.add_argument('--map-width',type=float,default=4.40);p.add_argument('--map-height',type=float,default=3.83);p.add_argument('--preview-ppm',type=int,default=100);p.add_argument('--ransac-threshold-m',type=float,default=.03);p.add_argument('--cam2-only',action='store_true',help='기존 CAM0 H를 잠그고 CAM2만 재등록/저장');p.add_argument('--self-test',action='store_true');a=p.parse_args();
    if a.self_test:self_test(b);return
    Tool(a).run()
if __name__=='__main__':main()
