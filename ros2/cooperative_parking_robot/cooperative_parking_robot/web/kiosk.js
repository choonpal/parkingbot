(() => {
  const $ = (id) => document.getElementById(id);
  const parkTab = $('parkTab');
  const retrieveTab = $('retrieveTab');
  const parkPanel = $('parkPanel');
  const retrievePanel = $('retrievePanel');
  const park = $('park');
  const retrieve = $('retrieve');
  const parkVehicle = $('parkVehicle');
  const parkPassword = $('parkPassword');
  const retrieveVehicle = $('retrieveVehicle');
  const retrievePassword = $('retrievePassword');
  const banner = $('banner');
  const connection = $('connection');
  const sitePlan = $('sitePlan');
  const spaceCount = $('spaceCount');
  const toastEl = $('toast');
  const offline = $('offline');
  const cameraWaiting = $('cameraWaiting');
  const developerControls = $('developerControls');
  const developerSlot = $('developerSlot');
  const developerMode = new URLSearchParams(window.location.search).get('dev') === '1';
  let latestStatus = null;
  let pendingRequest = '';
  let lastRequest = null;
  let lastCompletion = null;

  const reasons = {
    INVALID_REQUEST: '입력 내용을 다시 확인해 주세요',
    MISSION_ALREADY_ACTIVE: '다른 차량을 처리하고 있습니다',
    INVALID_VEHICLE_NUMBER: '차량번호 형식을 확인해 주세요',
    INVALID_PASSWORD: '비밀번호는 4자 이상 입력해 주세요',
    VEHICLE_ALREADY_PARKED: '이미 입차된 차량번호입니다',
    VEHICLE_OR_PASSWORD_INVALID: '차량번호 또는 비밀번호가 일치하지 않습니다',
    DESTINATION_SLOT_NOT_FOUND: '주차공간을 찾을 수 없습니다',
    DESTINATION_SLOT_NOT_EMPTY: '선택한 주차공간을 사용할 수 없습니다',
    DESTINATION_SLOT_UNAVAILABLE: '현재 사용할 수 있는 주차공간이 없습니다',
    ROBOT_NOT_IDLE: '로봇이 복귀하는 중입니다',
    STALE_REQUEST: '요청 시간이 지나 다시 입력해야 합니다',
    DUPLICATE_REQUEST_ID: '이미 처리한 요청입니다',
    DUPLICATE_SEQUENCE: '이미 처리한 요청입니다'
  };

  function toast(message) {
    toastEl.textContent = message;
    toastEl.style.display = 'block';
    window.setTimeout(() => { toastEl.style.display = 'none'; }, 2600);
  }

  function setMode(mode) {
    const isPark = mode === 'park';
    parkTab.classList.toggle('active', isPark);
    retrieveTab.classList.toggle('active', !isPark);
    parkTab.setAttribute('aria-selected', String(isPark));
    retrieveTab.setAttribute('aria-selected', String(!isPark));
    parkPanel.classList.toggle('hidden', !isPark);
    retrievePanel.classList.toggle('hidden', isPark);
    updateButtons();
  }

  function validVehicle(value) { return value.replace(/\s/g, '').length > 0; }

  function updateButtons() {
    const status = latestStatus || {};
    const selectedAvailable = !developerMode || !developerSlot.value ||
      (status.parking_spaces || []).some((space) => space.slot_id === developerSlot.value && space.available);
    park.disabled = !(status.park_enabled && selectedAvailable && validVehicle(parkVehicle.value) && parkPassword.value.length >= 4);
    retrieve.disabled = !(status.retrieve_enabled && validVehicle(retrieveVehicle.value) && retrievePassword.value.length >= 4);
  }

  function svgElement(name, attributes = {}) {
    const element = document.createElementNS('http://www.w3.org/2000/svg', name);
    Object.entries(attributes).forEach(([key, value]) => element.setAttribute(key, value));
    return element;
  }

  function drawPolygon(points, className) {
    if (!Array.isArray(points) || points.length < 3) return;
    sitePlan.appendChild(svgElement('polygon', {
      points: points.map((point) => `${point[0]},${point[1]}`).join(' '),
      class: className
    }));
  }

  function centroid(points) {
    const total = points.reduce((acc, point) => [acc[0] + point[0], acc[1] + point[1]], [0, 0]);
    return [total[0] / points.length, total[1] / points.length];
  }

  function addSiteText(text, x, y, className) {
    const label = svgElement('text', {x, y, class: className});
    label.textContent = text;
    sitePlan.appendChild(label);
  }

  function addSitePatterns() {
    const defs = svgElement('defs');
    const pattern = svgElement('pattern', {
      id: 'entry-zone-hatch', width: 4, height: 4,
      patternUnits: 'userSpaceOnUse', patternTransform: 'rotate(45)'
    });
    pattern.appendChild(svgElement('rect', {width: 4, height: 4, fill: '#fff8e8'}));
    pattern.appendChild(svgElement('path', {d: 'M 0 0 V 4', stroke: '#e7c477', 'stroke-width': 1}));
    defs.appendChild(pattern);
    sitePlan.appendChild(defs);
  }

  function renderDeveloperSlots(spaces) {
    if (!developerMode) return;
    const previous = developerSlot.value;
    developerSlot.textContent = '';
    const automatic = document.createElement('option');
    automatic.value = '';
    automatic.textContent = '자동 배정';
    developerSlot.appendChild(automatic);
    spaces.forEach((space) => {
      const option = document.createElement('option');
      option.value = space.slot_id;
      option.disabled = !space.available;
      option.textContent = `${space.display_number}번 (${space.slot_id}) · ${space.available ? '가능' : '불가'}`;
      developerSlot.appendChild(option);
    });
    if (previous && spaces.some((space) => space.slot_id === previous && space.available)) {
      developerSlot.value = previous;
    }
  }

  function renderSitePlan(status) {
    sitePlan.textContent = '';
    addSitePatterns();
    const layout = status.site_layout || {};
    const vehicleState = layout.vehicle_state || (layout.vehicle_present ? 'READY' : 'ABSENT');
    const vehicleClass = vehicleState.toLowerCase();
    drawPolygon(layout.waiting_polygon, `site-zone waiting ${vehicleClass}`);
    drawPolygon(layout.robot_start_polygon, 'site-zone');

    if (Array.isArray(layout.waiting_polygon) && layout.waiting_polygon.length) {
      const center = centroid(layout.waiting_polygon);
      if (vehicleState === 'DETECTING' || vehicleState === 'READY') {
        sitePlan.appendChild(svgElement('rect', {
          x: center[0] - 5, y: center[1] - 7, width: 10, height: 5,
          rx: 1.2, class: `site-vehicle ${vehicleClass}`
        }));
      }
      const vehicleCopy = {
        ABSENT: '차량 없음',
        DETECTING: '차량 감지 중',
        READY: '정차 확인',
        PERCEPTION_UNAVAILABLE: '인식 확인 중'
      }[vehicleState] || '상태 확인 중';
      addSiteText('입차 구역', center[0], center[1] + 0.5, 'site-label site-zone-label');
      addSiteText(vehicleCopy, center[0], center[1] + 5, `site-sub site-zone-sub ${vehicleClass}`);
    }

    if (Array.isArray(layout.robot_start_polygon) && layout.robot_start_polygon.length) {
      const center = centroid(layout.robot_start_polygon);
      addSiteText('로봇', center[0], center[1] - 1.5, 'site-label site-zone-label');
      addSiteText('출발 위치', center[0], center[1] + 3.5, 'site-sub site-zone-sub');
    }

    const spaces = status.parking_spaces || [];
    renderDeveloperSlots(spaces);
    const selectedSlotId = developerMode ? developerSlot.value : '';
    spaces.forEach((space) => {
      const className = `site-space ${space.assigned ? 'assigned' : (space.available ? 'empty' : '')}${space.slot_id === selectedSlotId ? ' developer-selected' : ''}`;
      drawPolygon(space.polygon, className);
      const center = centroid(space.polygon);
      const label = svgElement('text', {x: center[0], y: center[1] - 2, class: 'site-label'});
      label.textContent = `${space.display_number}번`;
      sitePlan.appendChild(label);
      const sub = svgElement('text', {x: center[0], y: center[1] + 4, class: 'site-sub'});
      sub.textContent = space.assigned ? '배정 위치' : (space.available ? '주차 가능' : '사용 중');
      sitePlan.appendChild(sub);
    });
    const available = spaces.filter((space) => space.available).length;
    spaceCount.textContent = available ? `빈 공간 ${available}곳` : '빈 공간 없음';
    spaceCount.style.color = available ? '' : '#c7382d';
    spaceCount.style.background = available ? '' : '#fdebea';
  }

  function updateProgress(status) {
    const state = status.fleet ? status.fleet.state : 'UNKNOWN';
    const targetState = status.target_state || (status.target_ready ? 'READY' : 'ABSENT');
    const stateIndex = {WAIT_TARGET: 0, WAIT_LIFT: 1, PLAN_PATH: 2, NAVIGATING: 2}[state] ?? 0;
    const labels = {WAIT_LIFT: '로봇 접근 준비', PLAN_PATH: '주차 위치 배정', NAVIGATING: '차량 이동 중'};
    $('progressTitle').textContent = state === 'WAIT_TARGET'
      ? ({READY: '정차 확인 완료', DETECTING: '차량 확인 중', ABSENT: '차량 입차 대기', PERCEPTION_UNAVAILABLE: '카메라 인식 일시 중단'}[targetState] || '차량 상태 확인 중')
      : (labels[state] || '시스템 준비 중');
    Array.from($('progressSteps').children).forEach((item, index) => {
      item.classList.toggle('active', index === stateIndex);
      item.classList.toggle('done', index < stateIndex);
    });
  }

  function render(status) {
    latestStatus = status;
    banner.textContent = status.banner;
    banner.className = `banner ${(status.fault || status.planning_blocker) ? 'alert' : ((status.localization_warning || status.planning_warning) ? 'warn' : '')}`;
    connection.className = 'connection online';
    connection.querySelector('span').textContent = '시스템 연결됨';
    renderSitePlan(status);
    updateProgress(status);
    updateButtons();

    const completion = status.last_completed;
    const completionSequence = completion ? Number(completion.completion_sequence) : -1;
    if (lastCompletion === null) lastCompletion = completionSequence;
    else if (completionSequence > lastCompletion) {
      lastCompletion = completionSequence;
      toast(completion.mission_type === 'retrieve' ? '출차가 완료되었습니다' : '입차가 완료되었습니다');
    }

    const requestStatus = status.request_status;
    const requestKey = requestStatus ? `${requestStatus.request_id}:${requestStatus.status}` : '';
    if (lastRequest === null) lastRequest = requestKey;
    else if (requestKey && requestKey !== lastRequest && requestStatus.request_id === pendingRequest) {
      lastRequest = requestKey;
      if (requestStatus.status === 'ACCEPTED') toast('요청이 승인되었습니다');
      else if (requestStatus.status === 'REJECTED') toast(reasons[requestStatus.reason] || `요청 거부: ${requestStatus.reason}`);
    }
  }

  function poll() {
    Promise.all([
      fetch('/api/status').then((response) => response.json()),
      fetch('/health').then((response) => response.json())
    ]).then(([status, health]) => {
      offline.style.display = 'none';
      cameraWaiting.classList.toggle('hidden', Boolean(health.ready));
      render(status);
    }).catch(() => {
      connection.className = 'connection';
      connection.querySelector('span').textContent = '연결 끊김';
      offline.style.display = 'flex';
    });
  }

  park.onclick = () => {
    park.disabled = true;
    const payload = {vehicle_number: parkVehicle.value, password: parkPassword.value};
    if (developerMode && developerSlot.value) payload.destination_slot_id = developerSlot.value;
    fetch('/api/park', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload)
    }).then((response) => response.json()).then((result) => {
      if (result.submitted) pendingRequest = result.request_id;
      parkPassword.value = '';
      toast(result.message);
      render(result.status);
    }).catch(() => { parkPassword.value = ''; toast('요청 전송에 실패했습니다'); });
  };

  retrieve.onclick = () => {
    retrieve.disabled = true;
    fetch('/api/retrieve', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({vehicle_number: retrieveVehicle.value, password: retrievePassword.value})
    }).then((response) => response.json()).then((result) => {
      if (result.submitted) pendingRequest = result.request_id;
      retrievePassword.value = '';
      toast(result.message);
      render(result.status);
    }).catch(() => { retrievePassword.value = ''; toast('요청 전송에 실패했습니다'); });
  };

  parkTab.onclick = () => setMode('park');
  retrieveTab.onclick = () => setMode('retrieve');
  ['input', 'change'].forEach((eventName) => {
    [parkVehicle, parkPassword, retrieveVehicle, retrievePassword].forEach((element) => element.addEventListener(eventName, updateButtons));
  });
  if (developerMode) developerControls.hidden = false;
  developerSlot.addEventListener('change', () => {
    if (latestStatus) renderSitePlan(latestStatus);
    updateButtons();
  });
  setMode('park');
  poll();
  window.setInterval(poll, 500);
})();
