/* -------------------------------------------
    Globals & state
    -------------------------------------------*/
let map, droneMarker, gridLayer, pathLayer, drawnItems, drawControl;
let isDrawing = false, selectedDronePort = null, cameraOn = false;
let missionActive = false, missionInterval = null, missionStartTs = null;
let initialCenterSet = false, pathPoints = [], gridCells = [];
let currentMissionStatus = "Beklemede";
let dronePathLayer = null;  // kalıcı iz
let dronePath = null;
let plannedPathLayer = null; // geçici rota
let selectedMissionType = "fire"; // Varsayılan olarak yangın tespiti

/* -------------------------------------------
    Toast helper
    -------------------------------------------*/
function showToast(text, type = "info", timeout = 3000) {
  const cont = document.getElementById('toastContainer');
  const el = document.createElement('div');
  el.className = 'toast ' + (type === 'success' ? 'toast-success' : type === 'error' ? 'toast-error' : 'toast-info');
  el.textContent = text;
  cont.appendChild(el);
  setTimeout(() => { el.style.opacity = 0; setTimeout(() => el.remove(), 300); }, timeout);
}

/* -------------------------------------------
    Init map + draw control + event handlers
    -------------------------------------------*/
function initMap(lat = 39.925533, lon = 32.864353, zoom = 15) {
  map = L.map('map', { zoomControl: true, preferCanvas: true }).setView([lat, lon], zoom);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
  }).addTo(map);

  // drone marker
  const droneIcon = L.divIcon({ className: 'drone-marker', iconSize: [14, 14], iconAnchor: [7, 7] });
  droneMarker = L.marker([lat, lon], { icon: droneIcon }).addTo(map);

  // layers
  gridLayer = L.layerGroup().addTo(map);
  pathLayer = L.layerGroup().addTo(map);
  plannedPathLayer = L.layerGroup().addTo(map);
  dronePathLayer = L.layerGroup().addTo(map);
  dronePath = L.polyline([], { color: '#007AFF', weight: 2 }).addTo(dronePathLayer);

  // drawn items
  drawnItems = new L.FeatureGroup().addTo(map);
  drawControl = new L.Control.Draw({
    edit: { featureGroup: drawnItems, edit: true, remove: true },
    draw: {
      polygon: false, polyline: false, circle: false, marker: false, circlemarker: false,
      rectangle: {
        showArea: true,
        shapeOptions: { color: '#007AFF', fillOpacity: 0.12, weight: 2 }
      }
    }
  });
  map.addControl(drawControl);

  // events
  map.on(L.Draw.Event.DRAWSTART, () => { isDrawing = true; });
  map.on(L.Draw.Event.DRAWSTOP, () => { isDrawing = false; });
  map.on(L.Draw.Event.CREATED, (e) => {
    isDrawing = false;
    drawnItems.clearLayers();
    drawnItems.addLayer(e.layer);
    if (e.layerType === 'rectangle') {
      const bounds = e.layer.getBounds();
      createGrid(bounds);
    }
    updateButtonStates();
  });

  // Add zoom event listener for grid stability
  map.on('zoomend', function() {
    if (gridCells.length > 0) {
      gridLayer.clearLayers();
      gridCells.forEach(c => c.rect.addTo(gridLayer));
    }
  });

  // click outside dropdown to hide
  document.addEventListener('click', (ev) => {
    const dropdown = document.getElementById('droneDropdown');
    const btn = document.getElementById('droneSelector');
    if (!btn.contains(ev.target) && !dropdown.contains(ev.target)) dropdown.classList.add('hidden');
  });

  // initial small UI
  updateButtonStates();
}

/* -------------------------------------------
    Grid creation: verilen bounds'u sabit boyutta hücrelere böler
    -------------------------------------------*/
function createGrid(bounds, minCellSizeMeters = 30) {
    gridLayer.clearLayers();
    gridCells = [];
    pathPoints = [];

    const north = bounds.getNorth();
    const south = bounds.getSouth();
    const east = bounds.getEast();
    const west = bounds.getWest();

    const latDistance = bounds.getNorthEast().distanceTo(bounds.getSouthEast());
    const lonDistance = bounds.getNorthEast().distanceTo(bounds.getNorthWest());

    const cellsY = Math.max(1, Math.floor(latDistance / minCellSizeMeters));
    const cellsX = Math.max(1, Math.floor(lonDistance / minCellSizeMeters));

    const latStep = (north - south) / cellsY;
    const lonStep = (east - west) / cellsX;

    // Hücre merkezlerini saklamak için bir dizi
    const cellCenters = [];

    // create rectangles
    for (let i = 0; i < cellsY; i++) {
        for (let j = 0; j < cellsX; j++) {
            const cellNorth = north - (i * latStep);
            const cellSouth = north - ((i + 1) * latStep);
            const cellWest = west + (j * lonStep);
            const cellEast = west + ((j + 1) * lonStep);
            const cellBounds = [[cellNorth, cellWest], [cellSouth, cellEast]];

            // Hücre merkezi
            const centerLat = (cellNorth + cellSouth) / 2;
            const centerLon = (cellWest + cellEast) / 2;
            cellCenters.push([centerLat, centerLon]);

            const rect = L.rectangle(cellBounds, {
                className: 'grid-cell grid-cell-unvisited',
                weight: 0.5,
                fillOpacity: 0.35,
                interactive: false
            });
            rect.addTo(gridLayer);
            gridCells.push({
                rect,
                bounds: cellBounds,
                center: [centerLat, centerLon],
                i,
                j,
                status: 'unvisited'
            });
        }
    }

    // Hücre merkezlerini Python'a gönder
    setMissionArea(cellCenters);

    // Rota görselleştirme için hücre merkezlerini haritada çiz
    pathLayer.clearLayers();
    if (cellCenters.length > 1) {
        const tempPath = L.polyline(cellCenters, {
            color: '#FF0000',
            weight: 2,
            dashArray: '5, 10'
        }).addTo(pathLayer);

        // 1 saniye sonra kırmızı çizgiyi kaldır
        setTimeout(() => {
            pathLayer.removeLayer(tempPath);
        }, 1000);
    }

    showToast(`Görev alanı seçildi ve ${cellsX}x${cellsY} ızgara oluşturuldu.`, "success");
}

/* -------------------------------------------
    Drawing controls (button handlers)
    -------------------------------------------*/
function enableRectangleDrawing() {
  new L.Draw.Rectangle(map, drawControl.options.draw.rectangle).enable();
  showToast("Harita üzerinde bir alan çizin.", "info");
}
function clearDrawing() {
  drawnItems.clearLayers();
  gridLayer.clearLayers();
  pathLayer.clearLayers();
  gridCells = [];
  pathPoints = [];
  missionActive = false;
  updateButtonStates();
  showToast("Çizimler temizlendi.", "info");
}

// Yeni: Tespit sonuçlarını getiren ve görüntüleyen fonksiyon
async function updateDetectionDisplay() {
  try {
    const response = await fetch('/get_detections');
    const detections = await response.json();
    const detectionsList = document.getElementById('detectionsList');
    detectionsList.innerHTML = ''; // Önceki sonuçları temizle

    if (detections && detections.length > 0) {
      detections.forEach(detection => {
        const detectionEl = document.createElement('div');
        detectionEl.className = 'flex items-center justify-between text-xs text-white/60';
        detectionEl.innerHTML = `
          <span>${detection.class_name.toUpperCase()}</span>
          <span class="font-bold">${(detection.score * 100).toFixed(2)}%</span>
        `;
        detectionsList.appendChild(detectionEl);
      });
    } else {
      detectionsList.innerHTML = '<div class="text-xs text-white/60">Henüz bir şey tespit edilmedi.</div>';
    }
  } catch (error) {
    console.error('Tespit sonuçları alınırken hata oluştu:', error);
    const detectionsList = document.getElementById('detectionsList');
    detectionsList.innerHTML = '<div class="text-xs text-red-400">Veri alınamadı.</div>';
  }
}

/* -------------------------------------------
    Backend / API calls (app.py ile uyumlu)
    -------------------------------------------*/
async function connectToDrone() {
  const conn = document.getElementById('connectionString').value.trim();
  if (!conn) { showToast("Bağlantı adresi giriniz.", "error"); return; }
  try {
    const res = await fetch('/connect_drone', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ connection_string: conn }) });
    const data = await res.json();
    showToast(data.message || "Bağlantı isteği gönderildi.", data.status === 'ok' ? 'success' : 'error');
    if (data.status === 'ok') updateDroneList();
  } catch (err) { showToast("Bağlantı hatası: " + err, "error"); }
}

async function updateDroneList() {
  try {
    const res = await fetch('/status');
    const data = await res.json();
    const dropdown = document.getElementById('droneDropdown');
    dropdown.innerHTML = '';
    if (data.connected_drones && data.connected_drones.length) {
      data.connected_drones.forEach(d => {
        const btn = document.createElement('button');
        btn.className = 'w-full text-left px-3 py-2 hover:bg-[#1a1c1d]';
        btn.innerHTML = `<span style="display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:8px;background:${d.is_active ? '#30D158' : '#FFCC00'}"></span> Port: ${d.port}`;
        btn.addEventListener('click', () => selectDrone(d.port));
        dropdown.appendChild(btn);
      });
    } else {
      const div = document.createElement('div');
      div.className = 'px-3 py-2 text-sm text-white/60';
      div.textContent = 'Bağlı dron yok';
      dropdown.appendChild(div);
    }
  } catch (err) {
    showToast("Drone listesi alınamadı: " + err, "error");
  }
}

async function selectDrone(port) {
  try {
    const res = await fetch('/select_drone', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ port }) });
    const data = await res.json();
    showToast(data.message || "Dron seçildi.", data.status === 'ok' ? 'success' : 'error');
    if (data.status === 'ok') {
      selectedDronePort = port;
      document.getElementById('selectedDroneText').textContent = `Port: ${port}`;
      document.getElementById('droneDropdown').classList.add('hidden');
      document.getElementById('globalStatusText').textContent = 'Bağlı';
      document.getElementById('globalStatus').className = 'status-indicator status-connected';
      updateButtonStates();
    }
  } catch (err) { showToast("Dron seçilemedi: " + err, "error"); }
}

async function setMissionArea(coords) {
  try {
    const res = await fetch('/set_area', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ coordinates: coords }) });
    const data = await res.json();
    showToast(data.message || "Alan kaydedildi.", data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) { showToast("Alan kaydedilemedi: " + err, "error"); }
}

async function startMission() {
  if (!selectedDronePort) { showToast("Önce dron seçin.", "error"); return; }
  if (drawnItems.getLayers().length === 0) { showToast("Önce görev alanı çizin.", "error"); return; }
  if (!selectedMissionType) { showToast("Lütfen bir görev seçin.", "error"); return; }

  if (!missionActive) {
    try {
      showToast("Görev başlatılıyor...", "info");
      const res = await fetch('/start_mission', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mission_type: selectedMissionType })
      });
      const data = await res.json();

      if (data.status === 'ok') {
        missionActive = true;
        updateMissionStatus(data.message);
        missionStartTs = Date.now();
        // Değişiklik: Burada zamanlayıcıyı başlatan fonksiyonu çağırıyoruz
        startMissionTimer();
      }
      showToast(data.message, data.status);
      updateButtonStates();
    } catch (err) {
      showToast("Görev başlatılamadı: " + err, "error");
    }
  }
}

function updateMissionStatus(message) {
    document.getElementById('missionStatusText').textContent = message;
}


function updateMissionProgress() {
    if (!missionActive || !missionStartTs) {
        clearInterval(missionInterval);
        return;
    }
    const elapsed = Math.floor((Date.now() - missionStartTs) / 1000);
    const minutes = Math.floor(elapsed / 60).toString().padStart(2, '0');
    const seconds = (elapsed % 60).toString().padStart(2, '0');
    document.getElementById('missionTimer').textContent = `${minutes}:${seconds}`;
}

async function stopMission() {
  try {
    const res = await fetch('/stop_mission', { method: 'POST', headers: { 'Content-Type': 'application/json' } });
    const data = await res.json();
    showToast(data.message || "Görev durduruldu.", data.status === 'ok' ? 'success' : 'error');
    if (data.status === 'ok') {
      missionActive = false;
      stopMissionTimer();
      updateButtonStates();
    }
  } catch (err) { showToast("Görev durdurulamadı: " + err, "error"); }
}

/* -------------------------------------------
    Periodic status polling -> updates map, telemetry, grid, path
    -------------------------------------------*/
async function updateStatus() {
  if (isDrawing) return;

  try {
    const res = await fetch('/status');
    if (!res.ok) throw new Error('Network response not ok');
    const data = await res.json();

    // Bağlantı durumu
    document.getElementById('globalStatusText').textContent =
      data.status === 'connected' ? 'Bağlı' : 'Bağlı Değil';
    document.getElementById('globalStatus').className =
      'status-indicator ' + (data.status === 'connected' ? 'status-connected' : 'status-disconnected');

    if (data.current_location) {
      const lat = data.current_location.lat;
      const lon = data.current_location.lon;
      const alt = data.current_location.alt ?? 0;
      const battery = data.battery_level ?? 0;

      document.getElementById('currentAltitude').textContent = `${alt.toFixed(2)}m`;
      document.getElementById('batteryVoltage').textContent = `${(battery * 0.05 + 10).toFixed(1)}V`;

      // Marker güncelleme
      if (!droneMarker) {
        const droneIcon = L.divIcon({ className: 'drone-marker', iconSize: [14, 14], iconAnchor: [7, 7] });
        droneMarker = L.marker([lat, lon], { icon: droneIcon }).addTo(map);
      } else {
        droneMarker.setLatLng([lat, lon]);
      }

      // Drone yolu çizimi (canlı iz)
      if (!dronePath) {
        dronePath = L.polyline([], { color: '#007AFF', weight: 2 }).addTo(dronePathLayer);
      }
      const last = dronePath.getLatLngs();
      const isNewPoint =
        !last.length ||
        last[last.length - 1].lat.toFixed(6) !== lat.toFixed(6) ||
        last[last.length - 1].lng.toFixed(6) !== lon.toFixed(6);
      if (isNewPoint) {
        dronePath.addLatLng([lat, lon]);
      }

      // Harita ilk odak
      if (!initialCenterSet) {
        map.setView([lat, lon], 18);
        initialCenterSet = true;
      }

      // Grid status güncelle
      if (data.grid_status && typeof data.grid_status === 'object') {
        Object.entries(data.grid_status).forEach(([key, status]) => {
          const [row, col] = key.split(',').map(Number);
          const cell = gridCells.find(c => c.i === row && c.j === col);
          if (cell) {
            let cls = 'grid-cell-unvisited';
            if (status === 'visited') cls = 'grid-cell-visited';
            else if (status === 'old') cls = 'grid-cell-old';
            cell.rect.setStyle({ className: `grid-cell ${cls}`, fillOpacity: 0.35 });
            cell.status = status;
          }
        });
      }

      // Görev durumu ve tespit bilgisi yazısı
      const missionStatusBox = document.getElementById('missionStatusBox');
      const missionStatusText = document.getElementById('missionStatusText');
      const detectionInfoEl = document.getElementById('detectionInfo');

      if (data.is_mission_active) {
        // Görev aktifse ve henüz zamanlayıcı başlatılmadıysa
        if (!missionActive) {
          missionActive = true;
          missionStartTs = Date.now();
          startMissionTimer();
        }
        updateMissionStatus(data.status_message || 'Görev çalışıyor');
      } else {
        // Görev aktif değilse ve zamanlayıcı çalışıyorsa durdur
        if (missionActive) {
          missionActive = false;
          stopMissionTimer();
        }
        updateMissionStatus(data.status_message || 'Beklemede');
      }

      // --- Yeni Ekleme: Tespit Bilgisi ---
      if (data.detection_results && data.detection_results.length > 0) {
          // Tespit edilen sınıfları alıp benzersiz hale getiriyoruz
          const detectedClasses = [...new Set(data.detection_results.map(d => d.class_name))];
          
          // Tespit edilenleri metin olarak gösteriyoruz
          detectionInfoEl.textContent = `Tespit Edildi: ${detectedClasses.join(', ')}`;
          detectionInfoEl.classList.remove('hidden');

          // Kutu rengini kırmızı yapıyoruz
          detectionInfoEl.classList.remove('bg-gray-700', 'bg-green-700');
          detectionInfoEl.classList.add('bg-red-700');

          // Tespit devam ettiği sürece zamanlayıcıyı sıfırlıyoruz
          clearTimeout(detectionTimer);
          detectionTimer = setTimeout(() => {
              detectionInfoEl.classList.add('hidden');
              detectionInfoEl.classList.remove('bg-red-700', 'bg-green-700');
              detectionInfoEl.classList.add('bg-gray-700');
          }, 5000); // 5 saniye sonra gizle
      } else {
          // Tespit yok → kutuyu gizle
          detectionInfoEl.classList.add('hidden');
          detectionInfoEl.classList.remove('bg-red-700', 'bg-green-700');
          detectionInfoEl.classList.add('bg-gray-700');
      }
    }

    // Görev durumu yazısı (yedek kontrol)
    if (data.is_mission_active) {
        missionActive = true;
        document.getElementById('missionStatusText').textContent = data.status_message || 'Görev çalışıyor';
    } else if (!missionActive) {
        document.getElementById('missionStatusText').textContent = data.status_message || 'Beklemede';
    }

  } catch (err) {
    console.debug('Status update error:', err);
  }

  updateButtonStates();
}

/* -------------------------------------------
    Timer helpers for mission elapsed
    -------------------------------------------*/
function startMissionTimer() {
  if (missionInterval) clearInterval(missionInterval);
  missionInterval = setInterval(() => {
    const elapsed = missionStartTs ? Math.floor((Date.now() - missionStartTs) / 1000) : 0;
    document.getElementById('missionDuration').textContent = formatTime(elapsed);
  }, 1000);
}
function stopMissionTimer() { 
  if (missionInterval) clearInterval(missionInterval); 
  missionInterval = null; 
  missionStartTs = null; 
  document.getElementById('missionDuration').textContent = '00:00'; 
}

function formatTime(sec) {
  if (!sec) return '00:00';
  const h = Math.floor(sec / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
  if (h > 0) return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

/* -------------------------------------------
    Camera toggle and feed update
    -------------------------------------------*/
function toggleCamera() {
  cameraOn = !cameraOn;
  const overlay = document.getElementById('cameraOverlay');
  const circ = document.getElementById('cameraToggleCircle');
  const btn = document.getElementById('cameraToggle');
  const feedImg = document.getElementById('cameraFeed');

  if (cameraOn) {
    circ.style.transform = 'translateX(26px)';
    btn.classList.add('bg-green-600');
    updateCameraFeed(); 
  } else {
    circ.style.transform = 'translateX(4px)';
    btn.classList.remove('bg-green-600');
    feedImg.src = '/static/images/logo.png'; // varsayılan resmin yolu
  }
}


function updateCameraFeed() {
  if (!cameraOn) return;
  const img = document.getElementById('cameraFeed');
  img.src = '/camera_feed?ts=' + Date.now();
  setTimeout(updateCameraFeed, 1000);
}

/* -------------------------------------------
    Button states / UI helpers
    -------------------------------------------*/
function updateButtonStates() {
  document.getElementById('startMission').disabled = !(selectedDronePort && drawnItems.getLayers().length > 0) || missionActive;
  document.getElementById('stopMission').disabled = !missionActive;
  document.getElementById('pauseMission').disabled = !missionActive;
}

/* -------------------------------------------
    Camera overlay drag (handle)
    -------------------------------------------*/
function initCameraDrag() {
  const overlay = document.getElementById('cameraOverlay');
  const handle = overlay.querySelector('.draggable-handle');
  let dragging = false, offsetX = 0, offsetY = 0;

  handle.addEventListener('mousedown', (e) => {
    dragging = true;
    const rect = overlay.getBoundingClientRect();
    offsetX = e.clientX - rect.left;
    offsetY = e.clientY - rect.top;
    overlay.style.transition = 'none';
    handle.style.cursor = 'grabbing';
    e.preventDefault();
  });
  document.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const mapRect = document.getElementById('map').getBoundingClientRect();
    let left = e.clientX - offsetX;
    let top = e.clientY - offsetY;
    left = Math.max(mapRect.left + 8, Math.min(left, mapRect.right - overlay.offsetWidth - 8));
    top = Math.max(mapRect.top + 8, Math.min(top, mapRect.bottom - overlay.offsetHeight - 8));
    overlay.style.left = left + 'px';
    overlay.style.top = top + 'px';
    overlay.style.right = 'unset';
    overlay.style.bottom = 'unset';
  });
  document.addEventListener('mouseup', () => {
    if (dragging) { dragging = false; overlay.style.transition = 'all .12s ease'; handle.style.cursor = 'grab'; }
  });

  handle.addEventListener('touchstart', (e) => {
    dragging = true;
    const t = e.touches[0];
    const rect = overlay.getBoundingClientRect();
    offsetX = t.clientX - rect.left; offsetY = t.clientY - rect.top;
    overlay.style.transition = 'none';
  });
  document.addEventListener('touchmove', (e) => {
    if (!dragging) return;
    const t = e.touches[0];
    const mapRect = document.getElementById('map').getBoundingClientRect();
    let left = t.clientX - offsetX;
    let top = t.clientY - offsetY;
    left = Math.max(mapRect.left + 8, Math.min(left, mapRect.right - overlay.offsetWidth - 8));
    top = Math.max(mapRect.top + 8, Math.min(top, mapRect.bottom - overlay.offsetHeight - 8));
    overlay.style.left = left + 'px';
    overlay.style.top = top + 'px';
    overlay.style.right = 'unset';
    overlay.style.bottom = 'unset';
  });
  document.addEventListener('touchend', () => { if (dragging) { dragging = false; overlay.style.transition = 'all .12s ease'; } });
}

/* -------------------------------------------
    UI event bindings
    -------------------------------------------*/
document.addEventListener('DOMContentLoaded', function () {
  initMap();
  initCameraDrag();

  // Buttons
  document.getElementById('connectButton').addEventListener('click', connectToDrone);
  document.getElementById('drawRectangle').addEventListener('click', enableRectangleDrawing);
  document.getElementById('clearDrawing').addEventListener('click', clearDrawing);
  document.getElementById('startMission').addEventListener('click', startMission);
  document.getElementById('stopMission').addEventListener('click', stopMission);
  document.getElementById('pauseMission').addEventListener('click', () => showToast('Duraklatma sunucuda uygulanmamış olabilir.', 'info'));
  document.getElementById('droneSelector').addEventListener('click', () => {
    const dd = document.getElementById('droneDropdown');
    dd.classList.toggle('hidden');
    if (!dd.classList.contains('hidden') && typeof updateDroneList === "function") {
      updateDroneList();
    }
  });

  document.getElementById('cameraToggle').addEventListener('click', toggleCamera);

  // Yeni: Görev seçimi açılır menüsü için olay dinleyicisi
  document.getElementById('missionSelector').addEventListener('click', () => {
    const md = document.getElementById('missionDropdown');
    md.classList.toggle('hidden');
  });

  document.querySelectorAll('#missionDropdown div').forEach(item => {
    item.addEventListener('click', function() {
      selectedMissionType = this.getAttribute('data-value');
      document.getElementById('selectedMissionText').textContent = this.textContent;
      document.getElementById('missionDropdown').classList.add('hidden');
    });
  });

  // periodic status
  setInterval(updateStatus, 2000);
  updateStatus();
});
