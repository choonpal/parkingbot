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

import argparse, json, math, socket, threading, time
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
</style></head><body><header><h1>Dual CCTV · 40cm Tile Grid Homography</h1></header><main>
<section class="panel">
<div class="row"><button id="cam0Btn" class="active" onclick="selectCam('cam0')">CAM0</button><button id="cam2Btn" onclick="selectCam('cam2')">CAM2</button><button class="primary" onclick="snapshotAll()">두 카메라 영상 정지</button></div>
<p class="small">두 카메라 모두 calibration.npz 적용 후 영상입니다. 같은 물리 타일 꼭짓점에는 CAM0/CAM2에서 같은 Tile i,j를 넣으세요.</p>
<h2>1. 공통 타일 좌표계</h2><div class="row"><label>Pitch(m) <input id="pitch" type="number" value="0.400" step="0.001" onchange="updateWorld()"></label><label>Origin X <input id="ox" type="number" value="0.000" step="0.01" onchange="updateWorld()"></label><label>Origin Y <input id="oy" type="number" value="0.000" step="0.01" onchange="updateWorld()"></label></div>
<p class="small">권장: 바닥의 공통 타일 꼭짓점 하나를 Tile(0,0) = Map(0,0)m로 정합니다.</p>
<h2>2. 기준점</h2><div class="row"><button id="refBtn" class="active" onclick="setMode('ref')">기준점</button><button id="measureBtn" onclick="setMode('measure')">X,Y 측정</button><button id="overlapBtn" onclick="setMode('overlap')">겹침 검증</button></div>
<div class="row"><label>Tile i <input id="ti" type="number" value="0" step="1" onchange="updateWorld()"></label><label>Tile j <input id="tj" type="number" value="0" step="1" onchange="updateWorld()"></label></div><div id="world">World=(0.000,0.000)m</div>
<div class="row"><button class="good" onclick="addRef()">클릭점 + Tile 등록</button><button onclick="undoRef()">마지막 취소</button><button onclick="clearRefs()">현재 CAM 삭제</button></div><div id="refs" class="small">기준점 0개</div>
<h2>3. Homography</h2><div class="row"><button class="good" onclick="calcH()">현재 CAM H 계산</button><button onclick="showPreview()">BEV 미리보기</button><button class="warn" onclick="saveAll()">H0 + H2 저장</button></div>
<h2>4. 겹침 검증</h2><p class="small">H0/H2 생성 후 같은 overlap Tile(i,j)를 CAM0에서 클릭하고 CAM2에서도 같은 꼭짓점을 클릭하세요.</p><div id="overlap" class="small">검증점 없음</div>
<h2>5. 측정 결과</h2><div style="max-height:210px;overflow:auto"><table><thead><tr><th>CAM</th><th>u</th><th>v</th><th>X</th><th>Y</th></tr></thead><tbody id="rows"></tbody></table></div>
<h2>상태</h2><div id="status">두 카메라 영상을 정지하세요.</div></section>
<section class="panel"><h2 id="imageTitle" style="margin-top:0">CAM0 · calibration 적용 정지 영상</h2><canvas id="canvas"></canvas><h2>BEV 미리보기</h2><img id="preview"></section>
</main><script>
const c=document.getElementById('canvas'),ctx=c.getContext('2d'),img=new Image();let cam='cam0',mode='ref',pending=null,refs={cam0:[],cam2:[]},ready={cam0:false,cam2:false},measures=[],overlap={},seq=0;
function stat(s,b=false){let e=document.getElementById('status');e.textContent=s;e.className=b?'bad':''}async function api(u,o={}){let r=await fetch(u,o),b=await r.json().catch(()=>({error:'응답 오류'}));if(!r.ok)throw new Error(b.error||r.statusText);return b}function post(u,p={}){return api(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)})}
function pitch(){return Number(document.getElementById('pitch').value)}function ox(){return Number(document.getElementById('ox').value)}function oy(){return Number(document.getElementById('oy').value)}function ti(){return Number(document.getElementById('ti').value)}function tj(){return Number(document.getElementById('tj').value)}function world(i,j){return[ox()+i*pitch(),oy()+j*pitch()]}function updateWorld(){let w=world(ti(),tj());document.getElementById('world').textContent=`World=(${w[0].toFixed(3)},${w[1].toFixed(3)})m`}
function selectCam(x){cam=x;pending=null;document.getElementById('cam0Btn').classList.toggle('active',x==='cam0');document.getElementById('cam2Btn').classList.toggle('active',x==='cam2');document.getElementById('imageTitle').textContent=`${x.toUpperCase()} · calibration 적용 정지 영상`;loadImg();renderRefs();draw()}
function setMode(m){mode=m;pending=null;document.getElementById('refBtn').classList.toggle('active',m==='ref');document.getElementById('measureBtn').classList.toggle('active',m==='measure');document.getElementById('overlapBtn').classList.toggle('active',m==='overlap');draw();stat(m==='ref'?'타일 꼭짓점을 클릭하고 Tile i,j를 등록하세요.':m==='measure'?'원하는 바닥점을 클릭하세요.':'같은 Tile i,j를 CAM0/CAM2에서 각각 클릭하세요.')}
async function snapshotAll(){try{let r=await post('/api/snapshot_all');seq=r.sequence;refs={cam0:[],cam2:[]};ready={cam0:false,cam2:false};measures=[];overlap={};pending=null;renderRefs();renderRows();renderOverlap();loadImg();document.getElementById('preview').style.display='none';stat(`정지 완료 CAM0=${r.cam0.width}x${r.cam0.height}, CAM2=${r.cam2.width}x${r.cam2.height}\n각 카메라 8~12점 권장`)}catch(e){stat(e.message,true)}}
function loadImg(){img.onload=()=>{c.width=img.naturalWidth;c.height=img.naturalHeight;draw()};img.src=`/api/snapshot/${cam}.jpg?s=${seq}&t=${Date.now()}`}
c.addEventListener('click',async e=>{if(!img.naturalWidth)return;let r=c.getBoundingClientRect(),p=[(e.clientX-r.left)*c.width/r.width,(e.clientY-r.top)*c.height/r.height];if(mode==='ref'){pending=p;draw();stat(`${cam} pixel=(${p[0].toFixed(1)},${p[1].toFixed(1)})`);return}if(mode==='measure'){if(!ready[cam]){stat('현재 CAM H가 없습니다.',true);return}try{let a=await post(`/api/transform/${cam}`,{pixel:p});measures.push({cam,pixel:p,world:a.world_m});renderRows();draw();stat(`X=${a.world_m[0].toFixed(3)}m Y=${a.world_m[1].toFixed(3)}m`)}catch(x){stat(x.message,true)}return}if(mode==='overlap'){if(!(ready.cam0&&ready.cam2)){stat('H0/H2를 모두 먼저 계산하세요.',true);return}let i=ti(),j=tj(),k=`${i},${j}`;if(!Number.isInteger(i)||!Number.isInteger(j)){stat('Tile i,j는 정수여야 합니다.',true);return}overlap[k]??={tile:[i,j]};overlap[k][cam]=p;draw();if(overlap[k].cam0&&overlap[k].cam2){try{let a=await post('/api/overlap_check',{tile_i:i,tile_j:j,pitch_m:pitch(),origin_x_m:ox(),origin_y_m:oy(),cam0_pixel:overlap[k].cam0,cam2_pixel:overlap[k].cam2});overlap[k].result=a;renderOverlap();stat(`T(${i},${j}) CAM0↔CAM2 차이 ${(a.inter_camera_error_m*100).toFixed(2)}cm`)}catch(x){stat(x.message,true)}}else{renderOverlap();stat(`T(${i},${j}) ${cam} 클릭 저장. 다른 CAM에서 같은 점 클릭.`)}}})
function addRef(){if(!pending){stat('먼저 타일 꼭짓점을 클릭하세요.',true);return}let i=ti(),j=tj();if(!Number.isInteger(i)||!Number.isInteger(j)){stat('Tile i,j는 정수여야 합니다.',true);return}if(refs[cam].some(r=>r.tile[0]===i&&r.tile[1]===j)){stat('현재 CAM에 같은 Tile이 이미 등록됨',true);return}let w=world(i,j);refs[cam].push({pixel:[...pending],tile:[i,j],world:w});pending=null;ready[cam]=false;renderRefs();draw();stat(`${cam} T(${i},${j}) → (${w[0].toFixed(3)},${w[1].toFixed(3)})m 등록`)}function undoRef(){refs[cam].pop();ready[cam]=false;renderRefs();draw()}function clearRefs(){refs[cam]=[];ready[cam]=false;pending=null;renderRefs();draw()}
function renderRefs(){document.getElementById('refs').innerHTML=refs[cam].length?refs[cam].map((r,i)=>`<span class="tag">R${i+1} T(${r.tile[0]},${r.tile[1]})→(${r.world[0].toFixed(2)},${r.world[1].toFixed(2)})</span>`).join(''):'기준점 0개'}
function renderRows(){document.getElementById('rows').innerHTML=measures.map(m=>`<tr><td>${m.cam}</td><td>${m.pixel[0].toFixed(1)}</td><td>${m.pixel[1].toFixed(1)}</td><td>${m.world[0].toFixed(3)}</td><td>${m.world[1].toFixed(3)}</td></tr>`).join('')}
function renderOverlap(){let ks=Object.keys(overlap);document.getElementById('overlap').innerHTML=ks.length?ks.map(k=>{let o=overlap[k],s=o.result?`Δcam=${(o.result.inter_camera_error_m*100).toFixed(2)}cm / Δ0=${(o.result.cam0_nominal_error_m*100).toFixed(2)} / Δ2=${(o.result.cam2_nominal_error_m*100).toFixed(2)}`:`${o.cam0?'cam0✓':'cam0-'} ${o.cam2?'cam2✓':'cam2-'}`;return`<span class="tag">T(${o.tile[0]},${o.tile[1]}) ${s}</span>`}).join(''):'검증점 없음'}
async function calcH(){try{let r=await post(`/api/homography/${cam}`,{references:refs[cam]});ready[cam]=true;stat(`${cam.toUpperCase()} H 완료\n기준점=${r.reference_count} / RANSAC 유효=${r.inlier_count}\nRMS=${(r.rms_m*100).toFixed(2)}cm / MAX=${(r.max_error_m*100).toFixed(2)}cm`);showPreview()}catch(e){stat(e.message,true)}}function showPreview(){if(!ready[cam]){stat('현재 CAM H가 없습니다.',true);return}let p=document.getElementById('preview');p.src=`/api/preview/${cam}.jpg?t=${Date.now()}`;p.style.display='block'}
async function saveAll(){try{let r=await post('/api/save_all',{pitch_m:pitch(),origin_x_m:ox(),origin_y_m:oy(),references:refs});stat(`저장 완료\n${r.cam0_npy}\n${r.cam2_npy}\n${r.summary_json}`)}catch(e){stat(e.message,true)}}
function draw(){ctx.clearRect(0,0,c.width,c.height);if(img.naturalWidth)ctx.drawImage(img,0,0,c.width,c.height);ctx.font='16px system-ui';refs[cam].forEach((r,i)=>mark(r.pixel,`R${i+1} T(${r.tile[0]},${r.tile[1]})`,'#48a7ff'));if(pending)mark(pending,'R?','#ffb454');measures.filter(m=>m.cam===cam).forEach(m=>mark(m.pixel,`M(${m.world[0].toFixed(2)},${m.world[1].toFixed(2)})`,'#4fd184'));Object.values(overlap).forEach(o=>{if(o[cam])mark(o[cam],`O T(${o.tile[0]},${o.tile[1]})`,'#ff75d8')})}function mark(p,t,col){ctx.beginPath();ctx.arc(p[0],p[1],7,0,Math.PI*2);ctx.fillStyle=col;ctx.fill();ctx.fillText(t,p[0]+10,p[1]-8)}updateWorld();
</script></body></html>'''


def load_calib(path: Path):
    with np.load(path, allow_pickle=False) as d:
        K=np.asarray(d['mtx'] if 'mtx' in d else d['camera_matrix'],np.float64)
        D=np.asarray(d['dist'] if 'dist' in d else d['dist_coeffs'],np.float64)
    if K.shape!=(3,3) or D.size<4: raise RuntimeError(f'잘못된 calibration: {path}')
    return K,D


def parse_device(v):
    s=str(v).strip(); return int(s) if s.lstrip('-').isdigit() else s


class Camera:
    def __init__(self,label,device,calib,width=640,height=480,fps=30.0):
        self.label=label;self.device=parse_device(device);self.K,self.D=load_calib(Path(calib));self.width=width;self.height=height;self.fps=fps;self.latest=None;self.error='';self.lock=threading.RLock();self.running=True;self.cap=None;threading.Thread(target=self.loop,daemon=True).start()
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
            if f.shape[1]!=self.width or f.shape[0]!=self.height:self.error=f'실제 해상도 {f.shape[1]}x{f.shape[0]} != 640x480';time.sleep(.1);continue
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


class Tool:
    def __init__(self,a):
        self.base=Path(__file__).resolve().parent;self.out=Path(a.output_dir).expanduser().resolve();self.out.mkdir(parents=True,exist_ok=True);self.w=a.width;self.h=a.height;self.map_w=a.map_width;self.map_h=a.map_height;self.ppm=a.preview_ppm;self.th=a.ransac_threshold_m;self.port=a.port
        if (self.w,self.h)!=(640,480):raise RuntimeError('포함 calibration 기준으로 640x480만 허용')
        self.cams={'cam0':Camera('cam0',a.cam0,a.cam0_calibration,self.w,self.h,a.fps),'cam2':Camera('cam2',a.cam2,a.cam2_calibration,self.w,self.h,a.fps)};self.lock=threading.RLock();self.snap={'cam0':None,'cam2':None};self.H={'cam0':None,'cam2':None};self.meta={'cam0':{},'cam2':{}};self.seq=0;self.app=self.web();self.server=make_server('0.0.0.0',self.port,self.app,threaded=True)
    def require(self,cam,kind):
        if cam not in self.cams:raise ValueError('cam must be cam0/cam2')
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
    def web(self):
        app=Flask('dual_tile_homography')
        @app.get('/')
        def home():return Response(HTML,mimetype='text/html; charset=utf-8')
        @app.post('/api/snapshot_all')
        def snapall():
            c0,c2=self.cams['cam0'].get(),self.cams['cam2'].get()
            if c0 is None or c2 is None:return jsonify(error=f"두 카메라 프레임 필요: cam0={self.cams['cam0'].error or '대기'}, cam2={self.cams['cam2'].error or '대기'}"),503
            with self.lock:self.snap={'cam0':c0,'cam2':c2};self.H={'cam0':None,'cam2':None};self.meta={'cam0':{},'cam2':{}};self.seq+=1
            return jsonify(sequence=self.seq,cam0={'width':c0.shape[1],'height':c0.shape[0]},cam2={'width':c2.shape[1],'height':c2.shape[0]})
        @app.get('/api/snapshot/<cam>.jpg')
        def snapjpg(cam):
            try:return self.jpg(self.require(cam,'snap'))
            except Exception as e:return jsonify(error=str(e)),404
        @app.post('/api/homography/<cam>')
        def homography(cam):
            try:
                H,m=self.fit((request.get_json(force=True) or {}).get('references',[]))
                with self.lock:self.H[cam]=H;self.meta[cam]=m
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
                f=self.require(cam,'snap');H=self.require(cam,'H');W=max(1,int(self.map_w*self.ppm));HH=max(1,int(self.map_h*self.ppm));M=np.array([[self.ppm,0,0],[0,-self.ppm,self.map_h*self.ppm],[0,0,1]],np.float64);return self.jpg(cv2.warpPerspective(f,M@H,(W,HH),borderValue=(25,25,25)))
            except Exception as e:return jsonify(error=str(e)),404
        @app.post('/api/save_all')
        def saveall():
            try:
                q=request.get_json(force=True) or {};H0=self.require('cam0','H');H2=self.require('cam2','H');p0=self.out/'homography_cam0_rectified.npy';p2=self.out/'homography_cam2_rectified.npy';np.save(p0,H0,allow_pickle=False);np.save(p2,H2,allow_pickle=False);np.savez(self.out/'homography_cam0_rectified.npz',H=H0);np.savez(self.out/'homography_cam2_rectified.npz',H=H2);summary={'format':'dual_tile_homography_v1','tile_pitch_m':float(q.get('pitch_m',.4)),'origin_world_m':[float(q.get('origin_x_m',0)),float(q.get('origin_y_m',0))],'image_size':[self.w,self.h],'cam0_H':H0.tolist(),'cam2_H':H2.tolist(),'references':q.get('references',{}),'fit_meta':self.meta,'saved_unix_ns':time.time_ns()};sp=self.out/'dual_homography_summary.json';sp.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');
                with self.lock:
                    for cam in ('cam0','cam2'):
                        if self.snap[cam] is not None:cv2.imwrite(str(self.out/f'{cam}_rectified_snapshot.jpg'),self.snap[cam])
                return jsonify(cam0_npy=str(p0),cam2_npy=str(p2),summary_json=str(sp))
            except Exception as e:return jsonify(error=str(e)),400
        return app
    def run(self):
        ip='JETSON_IP'
        try:s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.connect(('8.8.8.8',80));ip=s.getsockname()[0];s.close()
        except Exception:pass
        print('='*70);print('Dual CCTV Tile Homography GUI');print(f'Jetson: http://127.0.0.1:{self.port}');print(f'Other PC: http://{ip}:{self.port}');print(f'Output: {self.out}');print('='*70)
        try:self.server.serve_forever()
        finally:
            for c in self.cams.values():c.close()


def self_test(base):
    for f in ('cctv0_camera_calibration.npz','cctv2_camera_calibration.npz'):K,D=load_calib(base/f);assert K.shape==(3,3) and D.size>=4
    world=np.array([[0,0],[1.6,0],[3.2,0],[0,1.6],[1.6,1.6],[3.2,1.6],[0,3.2],[1.6,3.2],[3.2,3.2]],np.float64);Hw=np.array([[120,7,80],[4,90,50],[.01,.02,1]],np.float64);pix=cv2.perspectiveTransform(world.reshape(-1,1,2),Hw).reshape(-1,2);H,_=cv2.findHomography(pix,world,0);rest=cv2.perspectiveTransform(pix.reshape(-1,1,2),H).reshape(-1,2);assert np.linalg.norm(rest-world,axis=1).max()<1e-5;print('SELF TEST OK')


def main():
    b=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--cam0',default='0');p.add_argument('--cam2',default='2');p.add_argument('--cam0-calibration',default=str(b/'cctv0_camera_calibration.npz'));p.add_argument('--cam2-calibration',default=str(b/'cctv2_camera_calibration.npz'));p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480);p.add_argument('--fps',type=float,default=30);p.add_argument('--port',type=int,default=5001);p.add_argument('--output-dir',default=str(b/'output'));p.add_argument('--map-width',type=float,default=6.0);p.add_argument('--map-height',type=float,default=10.0);p.add_argument('--preview-ppm',type=int,default=100);p.add_argument('--ransac-threshold-m',type=float,default=.03);p.add_argument('--self-test',action='store_true');a=p.parse_args();
    if a.self_test:self_test(b);return
    Tool(a).run()
if __name__=='__main__':main()
