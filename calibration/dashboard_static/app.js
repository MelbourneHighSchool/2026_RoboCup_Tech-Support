'use strict';
const $ = id => document.getElementById(id);
const lineTab = document.createElement('button');
lineTab.dataset.tab = 'lineSensors'; lineTab.textContent = 'Line sensors';
document.querySelector('nav').append(lineTab);
let token = null, state = null, tab = 'camera', frozen = null, thresholds = null;
let editUntil = 0, messageUntil = 0, lastEvent = '', samplesSignature = '', fitSignature = '';
let modelsSignature = '';
let lineDirty = false, lineLoaded = false, lineEditVersion = 0;
const fmt = (v, digits = 1) => v == null ? 'unavailable' : Number(v).toFixed(digits);
const escapeHtml = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function message(text) { $('message').textContent = text; messageUntil = Date.now() + 10000; }
async function api(action, data = {}) {
  const response = await fetch('/api/' + action, {method:'POST', headers:{'Content-Type':'application/json', ...(token ? {'X-Control-Token':token} : {})}, body:JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) {
    if (response.status === 403 && action === 'heartbeat') { token = null; message('Control lost. Motors disarmed. Take control and arm again.'); }
    throw new Error(result.error || 'Request failed');
  }
  return result;
}
function handle(id, action, data = () => ({})) { $(id).addEventListener('click', async () => { try { await api(action, data()); } catch (e) { message(e.message); } }); }
function motorAddresses() {
  const values = $('addresses').value.split(',').map(v => Number(v.trim()));
  if (!values.length || values.some(v => !Number.isInteger(v) || v < 8 || v > 119)) throw new Error('Enter valid ordered I2C addresses.');
  return values;
}
$('claim').onclick = async () => { try { token = (await api('claim')).token; message('You have control. Motors remain disarmed until you arm them.'); } catch (e) { message(e.message); } };
$('release').onclick = async () => { try { await api('release'); token = null; } catch (e) { message(e.message); } };
handle('stop', 'stop');
handle('startGpio', 'gpio');
handle('pcbStart', 'pcb_start');
handle('pcbStop', 'pcb_stop');
function lineValues() {
  const black = Number($('lineBlack').value), white = Number($('lineWhite').value);
  if (['lineBlack', 'lineWhite'].some(id => $(id).value === '') ||
      ![black, white].every(v => Number.isInteger(v) && v >= 0 && v <= 255) || black === white)
    throw new Error('Enter different whole-number thresholds between 0 and 255.');
  return {black, white};
}
function lineColour(value, black, white) {
  if (black < white) return value <= black ? 'black' : value >= white ? 'white' : 'green';
  return value >= black ? 'black' : value <= white ? 'white' : 'green';
}
let lineNodes = [], lineReceivedAt = 0;
function renderLineSensors(s) {
  if (!lineNodes.length) {
    const svg = $('sensorRing'), ns = 'http://www.w3.org/2000/svg';
    const element = (name, attrs, text) => {
      const node = document.createElementNS(ns, name);
      for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, value);
      if (text != null) node.textContent = text;
      svg.append(node); return node;
    };
    element('circle', {cx:300, cy:300, r:210, fill:'none', stroke:'#486070', 'stroke-dasharray':'4 6'});
    element('text', {x:300, y:22, 'text-anchor':'middle', fill:'#e9f0f4'}, 'FRONT ↑');
    element('text', {x:300, y:295, 'text-anchor':'middle', fill:'#a2b5c3'}, '75 mm radius');
    element('text', {x:300, y:322, 'text-anchor':'middle', fill:'#a2b5c3'}, '32 sensors · clockwise →');
    for (let i = 0; i < 32; i++) {
      const angle = i * Math.PI / 16;
      const circle = element('circle', {cx:300+210*Math.sin(angle), cy:300-210*Math.cos(angle), r:16, stroke:'#93a7b3', 'stroke-width':2});
      const title = document.createElementNS(ns, 'title'); circle.append(title);
      element('text', {x:300+245*Math.sin(angle), y:305-245*Math.cos(angle), 'text-anchor':'middle', fill:'#e9f0f4'}, String(i));
      const card = document.createElement('div'); card.className = 'sensor-reading'; $('sensorReadings').append(card);
      lineNodes.push({circle, title, card});
    }
  }
  if (s?.line_thresholds && (!lineLoaded || !lineDirty)) {
    setLineValues(s.line_thresholds); lineLoaded = true;
  }
  let values;
  try { values = lineValues(); } catch (e) { $('lineRule').textContent = e.message; }
  if (values) {
    const {black, white} = values;
    $('lineRule').textContent = `Black ${black < white ? '≤' : '≥'} ${black} · Green between · White ${black < white ? '≥' : '≤'} ${white}`;
  }
  $('lineEditStatus').textContent = lineDirty ? 'Unsaved preview' : 'Showing saved thresholds (64 / 192 are provisional defaults).';
  const pcb = s?.hardware.pcb;
  const age = pcb?.age_s == null ? null : pcb.age_s + (performance.now() - lineReceivedAt) / 1000;
  const fresh = pcb?.valid && age != null && age <= 0.5 && pcb.readings?.length === 32;
  const colours = {black:'#080c10', green:'#279b58', white:'#ffffff', unavailable:'#50616e'};
  lineNodes.forEach(({circle, title, card}, i) => {
    const raw = fresh ? pcb.readings[i] : null;
    const colour = raw != null && values ? lineColour(raw, values.black, values.white) : 'unavailable';
    circle.setAttribute('fill', colours[colour]);
    title.textContent = `Sensor ${i} · ${i*11.25}° · ${raw ?? '—'} · ${colour}`;
    card.textContent = `${i}: ${raw ?? '—'} · ${colour}`;
    card.style.borderLeftColor = colours[colour];
  });
  $('pcbStatus').textContent = fresh ? `Live · sample age ${fmt(age, 2)} s · black / green / white`
    : pcb?.valid ? 'Readings are stale — sensors shown grey.'
    : s?.hardware.error ? `Unavailable: ${s.hardware.error}` : 'No live readings. Start sensors to connect.';
}
function setLineValues(values) {
  for (const colour of ['Black', 'White'])
    $( 'line' + colour).value = $('line' + colour + 'Slider').value = values[colour.toLowerCase()];
}
for (const colour of ['Black', 'White']) {
  for (const suffix of ['', 'Slider']) $('line' + colour + suffix).oninput = () => {
    $('line' + colour + (suffix ? '' : 'Slider')).value = $('line' + colour + suffix).value;
    lineDirty = true; lineEditVersion++; renderLineSensors(state);
  };
}
$('saveLine').onclick = async () => {
  try {
    const values = lineValues(), version = lineEditVersion;
    await api('save_line_thresholds', values);
    if (version === lineEditVersion) {
      lineDirty = false;
      if (state) state.line_thresholds = values;
    }
    message('Line sensor thresholds saved.');
  } catch (e) { message(e.message); }
};
$('revertLine').onclick = () => {
  if (!state?.line_thresholds) return;
  lineDirty = false; lineEditVersion++; setLineValues(state.line_thresholds); renderLineSensors(state);
};
handle('kick', 'kick');
handle('applyGain', 'analogue_gain', () => ({gain:$('analogueGain').value}));
for (const button of document.querySelectorAll('.arm')) button.onclick = async () => { try { await api('arm'); } catch (e) { message(e.message); } };
const distanceTarget = () => ({target:$('distanceTarget').value});
$('distanceTarget').onchange = () => {
  $('captured').disabled = $('distanceTarget').value === 'bot';
  samplesSignature = ''; fitSignature = '';
  if (state) render(state);
};
handle('sample', 'sample', () => ({...distanceTarget(), distance:$('distance').value, captured:$('captured').checked}));
handle('fit', 'fit', distanceTarget); handle('saveBall', 'save_ball', distanceTarget); handle('clearSamples', 'clear_samples', distanceTarget);
$('saveGoals').onclick = async () => {
  try { clearTimeout(goalTimer); await api('thresholds', {thresholds}); await api('save_goals'); }
  catch(e) { message(e.message); }
};
for (const [id, action] of [['revertGoals','revert_goals'],['defaultGoals','default_goals']]) $(id).onclick = async () => { try { clearTimeout(goalTimer); editUntil = 0; await api(action); } catch(e) { message(e.message); } };
$('localise').onclick = async () => {
  try { await api($('localise').dataset.running === 'true' ? 'stop_localise' : 'localise'); }
  catch (e) { message(e.message); }
};
handle('calibrate', 'calibrate', () => ({addresses:motorAddresses(), wheels_clear:$('wheelsClear').checked}));
handle('drive', 'drive', () => ({addresses:motorAddresses(), speed:$('speed').value, target_yaw:$('targetYaw').value, target:[$('targetX').value,$('targetY').value]}));
$('modelSize').onchange = async () => {
  try { await api('select_model', {model:$('modelSize').value}); }
  catch (e) { message(e.message); }
};
$('speedSlider').oninput = () => { $('speed').value = $('speedSlider').value; };
$('speed').oninput = () => { $('speedSlider').value = Math.min(1000, Math.max(0, Number($('speed').value))); };
function streams() {
  for (const img of document.querySelectorAll('img[data-view]')) {
    if (img.closest('.panel').id !== tab || (img.id === 'cameraPreview' && frozen)) { if (img.id !== 'cameraPreview' || !frozen) img.removeAttribute('src'); continue; }
    let view = img.dataset.view;
    if (img.id === 'cameraPreview' && $('rawToggle').checked) view = 'raw';
    if (img.id === 'goalPreview' && !$('contourToggle').checked) view = 'camera';
    const path = '/stream.mjpg?view=' + view;
    if (img.getAttribute('src') !== path) img.src = path;
  }
}
for (const button of document.querySelectorAll('[data-tab]')) button.onclick = () => {
  tab = button.dataset.tab;
  for (const b of document.querySelectorAll('[data-tab]')) b.classList.toggle('active', b === button);
  for (const panel of document.querySelectorAll('.panel')) panel.classList.toggle('active', panel.id === tab);
  streams(); if (state) drawPitch(state.hardware);
};
$('rawToggle').onchange = streams; $('contourToggle').onchange = streams;
$('freeze').onclick = async () => {
  try {
    frozen = await api('freeze'); $('cameraPreview').src = '/frozen.png?id=' + frozen.id;
    $('cameraPreview').classList.add('picking'); $('unfreeze').hidden = false;
    $('pixelHelp').textContent = 'Frozen frame ' + frozen.frame_id + ' · Tap a pixel to read its original values.';
  } catch (e) { message(e.message); }
};
$('unfreeze').onclick = () => {
  frozen = null; $('cameraPreview').classList.remove('picking'); $('unfreeze').hidden = true;
  $('pixelHelp').textContent = 'Freeze a frame to inspect its original pixel colours.'; streams();
};
$('cameraPreview').onclick = async event => {
  if (!frozen) return;
  const bounds = event.currentTarget.getBoundingClientRect();
  const x = Math.min(frozen.width - 1, Math.floor((event.clientX - bounds.left) / bounds.width * frozen.width));
  const y = Math.min(frozen.height - 1, Math.floor((event.clientY - bounds.top) / bounds.height * frozen.height));
  try {
    const response = await fetch(`/api/pixel?id=${frozen.id}&x=${x}&y=${y}`); const p = await response.json();
    if (!response.ok) throw new Error(p.error);
    const swatch = document.createElement('canvas'); swatch.width = 30; swatch.height = 30; swatch.className = 'swatch';
    const context = swatch.getContext('2d'); context.fillStyle = `rgb(${p.rgb.join(',')})`; context.fillRect(0,0,30,30);
    $('pixelReadout').replaceChildren(swatch, document.createTextNode(`(${x}, ${y})\nRGB ${p.rgb.join(', ')}\nBGR ${p.bgr.join(', ')}\nHSV ${p.hsv.join(', ')}`));
  } catch (e) { message(e.message); }
};
let goalTimer;
for (const colour of ['blue', 'yellow']) {
  for (const side of ['lower', 'upper']) {
    const title = document.createElement('div'); title.className = 'bound-title'; title.textContent = side.toUpperCase(); $(colour + 'Bounds').append(title);
    ['H','S','V'].forEach((channel, index) => {
      const row = document.createElement('div'); row.className = 'bounds-row';
      const label = document.createElement('label'); label.textContent = channel; label.htmlFor = `${colour}-${side}-${index}-n`;
      const range = document.createElement('input'); range.type = 'range'; range.min = 0; range.max = index ? 255 : 179; range.className = 'control'; range.setAttribute('aria-label', `${colour} ${side} ${channel}`);
      const number = document.createElement('input'); number.type = 'number'; number.min = range.min; number.max = range.max; number.id = label.htmlFor; number.className = 'control';
      range.id = `${colour}-${side}-${index}-r`; row.append(label,range,number); $(colour+'Bounds').append(row);
      const update = input => {
        const value = Number(input.value); if (!Number.isInteger(value) || value < 0 || value > Number(range.max) || !thresholds) return;
        range.value = number.value = value; thresholds[colour][side][index] = value; editUntil = Date.now() + 1500;
        clearTimeout(goalTimer); goalTimer = setTimeout(async () => {
          try { await api('thresholds', {thresholds}); } catch(e) { message(e.message); }
        }, 120);
      };
      range.oninput = () => update(range); number.oninput = () => update(number);
    });
  }
}
function syncBounds(bounds) {
  if (Date.now() < editUntil) return;
  thresholds = structuredClone(bounds);
  for (const c of ['blue','yellow']) for (const side of ['lower','upper']) for (let i=0;i<3;i++) {
    $(`${c}-${side}-${i}-r`).value = bounds[c][side][i]; $(`${c}-${side}-${i}-n`).value = bounds[c][side][i];
  }
}
function readings(element, rows) {
  element.innerHTML = rows.map(([label,value]) => `<div class="reading"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)}</strong></div>`).join('');
}
function render(s) {
  lineReceivedAt = performance.now();
  renderLineSensors(s);
  state = s;
  $('leaseStatus').textContent = token ? (s.control.armed ? 'Controller · ARMED' : 'Controller · disarmed') : (s.control.occupied ? 'Viewer · controller connected' : 'Viewer');
  $('claim').disabled = !!token || s.control.occupied;
  document.querySelectorAll('.control').forEach(e => { e.disabled = !token; });
  $('pcbStart').disabled = !token || !['idle', 'stopped', 'monitoring'].includes(s.hardware.mode) || s.hardware.localisation != null;
  $('pcbStop').disabled = !token || !['pcb', 'queued pcb_start', 'starting pcb_start'].includes(s.hardware.mode);
  document.querySelectorAll('.arm').forEach(e => { e.disabled = !token || s.control.armed; });
  $('calibrate').disabled = $('drive').disabled = !token || !s.control.armed;
  $('kick').disabled = !token || !s.control.armed;
  const beam = s.hardware.break_beam;
  $('breakBeamStatus').textContent = beam?.error ? `Unavailable: ${beam.error}` : beam?.blocked == null ? 'Monitoring has not started.' : beam.blocked ? 'BLOCKED — ball detected' : 'CLEAR — no ball';
  const gainRange = s.camera.analogue_gain_range;
  const gainAvailable = s.camera.status === 'running' && gainRange != null;
  $('analogueGain').disabled = $('applyGain').disabled = !token || !gainAvailable;
  if (gainRange) {
    $('analogueGain').min = gainRange[0]; $('analogueGain').max = gainRange[1];
  }
  if (document.activeElement !== $('analogueGain') && document.activeElement !== $('applyGain')) $('analogueGain').value = s.camera.analogue_gain;
  $('gainHelp').textContent = gainAvailable ? `Requested gain: ${fmt(s.camera.analogue_gain, 2)}× · Range ${fmt(gainRange[0], 2)}–${fmt(gainRange[1], 2)}×. Applies live for this dashboard session.` : 'Camera analogue gain is unavailable.';
  const modelSignature = JSON.stringify(s.camera.models);
  if (modelsSignature !== modelSignature) {
    modelsSignature = modelSignature;
    $('modelSize').replaceChildren(...s.camera.models.map(model => {
      const option = document.createElement('option');
      option.value = model.id;
      const size = model.input_size ? ` · ${model.input_size.join('×')}` : '';
      option.textContent = `${model.label} (${model.id})${size}`;
      return option;
    }));
  }
  $('modelSize').value = s.camera.requested_model ?? '';
  $('modelSize').disabled = !token || s.camera.status === 'switching model' || s.camera.models.length < 2;
  const localisationRunning = s.hardware.localisation != null ||
    ['queued localise','starting localise','monitoring','driving','stopping localisation'].includes(s.hardware.mode);
  $('localise').dataset.running = String(localisationRunning);
  $('localise').textContent = localisationRunning ? 'Stop localisation' : 'Start localisation';
  $('localise').classList.toggle('danger', localisationRunning);
  $('localise').classList.toggle('primary', !localisationRunning);
  $('health').textContent = `CAMERA ${s.camera.status} | capture ${fmt(s.camera.fps.capture)} · inference ${fmt(s.camera.fps.inference)} · preview ${fmt(s.camera.fps.preview)} FPS | ${s.camera.age_s == null ? 'no frames' : 'frame age ' + fmt(s.camera.age_s) + 's' + (s.camera.age_s > .5 ? ' · STALE' : '')} | MOTORS ${s.hardware.mode}`;
  if (!$('addresses').value && s.addresses.length) $('addresses').value = s.addresses.join(',');
  syncBounds(s.thresholds);
  readings($('cameraDetails'), [['Resolution',s.camera.resolution ? s.camera.resolution.join(' × ') : 'unavailable'],['Inference frame',s.camera.frame_id ?? 'unavailable'],['Ball distance fit',s.fit ? 'available' : 'not loaded'],['Bot distance fit',s.bot_fit ? 'available' : 'not loaded']]);
  readings($('detections'), s.camera.detections.map(d => [d.label + ' ' + fmt(d.confidence*100,0) + '%', `${fmt(d.bearing % 360)}° · ${d.distance == null ? 'distance unavailable' : fmt(d.distance,0)+' mm'}`]));
  if (!s.camera.detections.length) $('detections').textContent = 'No current detections';
  const distanceState = $('distanceTarget').value === 'bot' ? {...s, samples:s.bot_samples, fit:s.bot_fit} : s;
  const signature = JSON.stringify(distanceState.samples);
  if (samplesSignature !== signature) {
    samplesSignature = signature; $('samples').replaceChildren();
    distanceState.samples.forEach((sample,index) => {
      const tr = document.createElement('tr');
      for (const value of [fmt(sample.distance_mm),fmt(sample.radial_pixels),`${fmt(sample.centre_x)}, ${fmt(sample.centre_y)}`, sample.captured ? 'yes':'no']) { const td=document.createElement('td'); td.textContent=value; tr.append(td); }
      const td=document.createElement('td'), button=document.createElement('button'); button.textContent='Remove'; button.className='control'; button.disabled=!token;
      button.onclick=async()=>{try{await api('remove_sample',{index, ...distanceTarget()});}catch(e){message(e.message);}}; td.append(button); tr.append(td); $('samples').append(tr);
    });
  }
  const fs = JSON.stringify([distanceState.samples,distanceState.fit]); if (fitSignature !== fs) { fitSignature = fs; drawFit(distanceState); }
  const calibration = s.hardware.calibration;
  readings($('motorProgress'), [['State',s.hardware.mode],['Active driver', calibration?.address ?? '—'],['Elapsed',fmt(calibration?.elapsed_s)+' s'],['Completed',calibration?.complete ? 'Saved':'—'],['Error',s.hardware.error ?? 'none']]);
  for (const motor of calibration?.results ?? []) { const p=document.createElement('p'); p.textContent=`Address ${motor.address}: ELECANGLEOFFSET ${motor.elecangleoffset} · SINCOSCENTRE ${motor.sincoscentre}`; $('motorProgress').append(p); }
  const l=s.hardware.localisation;
  if(l) {
    const p=l.pose, o=l.odometry, r=l.recovery, c=l.correction;
    readings($('localisationReadings'), [
      ['Fix',p[4] ? (l.fresh?'tracking':'STALE'):'NO FIX'],['Pose X / Y / yaw',`${fmt(p[0])} / ${fmt(p[1])} mm / ${fmt(p[2])}°`],['Confidence',fmt(p[3],3)],
      ['Scan age / IMU age',`${fmt(l.scan_age_s,2)} / ${fmt(l.imu_age_s,2)} s`],['Scans / corrections',`${l.scan_generation} / ${l.mcl_updates}`],['Points',l.scan_count],['Scan updates',l.scan_updates_enabled?'enabled':'PAUSED · predict only'],
      ['Correction error',c[9]?`${fmt(c[7])} mm / ${fmt(c[8])}°`:'unavailable'],['Predicted pose',c[9]?c.slice(1,4).map(v=>fmt(v)).join(' / '):'unavailable'],['Corrected pose',c[9]?c.slice(4,7).map(v=>fmt(v)).join(' / '):'unavailable'],
      ['Quality / baseline',r[4]?`${fmt(r[0],2)} / ${fmt(r[1],2)}`:'not ready'],['Bad scans / global recovery',`${r[2]} / ${fmt(r[3]*100,0)}%`],['Loop / gyro',`${fmt(o.dt_s*1000)} ms / ${fmt(o.omega_deg_s)}°/s`],
      ['Wheel vx / vy',o.has_wheel_odometry?`${fmt(o.wheel_vx)} / ${fmt(o.wheel_vy)} mm/s`:'unavailable'],['LIDAR vx / vy',o.has_wheel_odometry?`${fmt(o.lidar_vx)} / ${fmt(o.lidar_vy)} mm/s · ${o.lidar_fresh?'fresh':'stale'}`:'unavailable'],['Wheel trust',o.has_wheel_odometry?fmt(o.trust*100,0)+'%':'unavailable'],['Fed vx / vy',`${fmt(o.fed_vx)} / ${fmt(o.fed_vy)} mm/s`]
    ]);
  }
  const d=s.hardware.drive; $('driveInfo').textContent = d ? `Requested ${fmt(d.requested_speed,0)} mm/s · current target ${fmt(d.ramped_request,0)} · RPM-limited ${fmt(d.rpm_limited_speed,0)}${d.rpm_limited?' (motor limit active)':''}` : '';
  drawPitch(s.hardware);
  $('events').replaceChildren(...s.events.slice().reverse().map(e=>{const div=document.createElement('div'); div.textContent=new Date(e.time*1000).toLocaleTimeString()+' · '+e.message; if(e.error)div.className='error';return div;}));
  const latest=s.events.at(-1); if(latest && latest.time+latest.message !== lastEvent) { lastEvent=latest.time+latest.message; if(latest.error)message(latest.message); }
  if(Date.now()>messageUntil) $('message').textContent='';
}
function drawFit(s) {
  const canvas=$('fitChart'), c=canvas.getContext('2d'), W=canvas.width,H=canvas.height;
  c.clearRect(0,0,W,H); c.font='12px monospace'; c.fillStyle='#a2b5c3';
  const samples=s.samples, maxX=Math.max(1,...samples.map(v=>v.radial_pixels))*1.1, maxY=Math.max(1,...samples.map(v=>v.distance_mm))*1.1;
  const px=x=>55+x/maxX*(W-80), py=y=>H-40-y/maxY*(H-65);
  c.strokeStyle='#30424f'; c.beginPath(); c.moveTo(55,15); c.lineTo(55,H-40);c.lineTo(W-15,H-40); c.stroke();
  c.fillText('distance mm',10,12);c.fillText('radius px',W-110,H-8);c.fillText(fmt(maxY,0),5,32);c.fillText(fmt(maxX,0),W-70,H-24);
  c.fillStyle='#6ce3bd'; for(const sample of samples){c.beginPath();c.arc(px(sample.radial_pixels),py(sample.distance_mm),4,0,Math.PI*2);c.fill();}
  if(s.fit){
    const f=s.fit; c.save();c.beginPath();c.rect(55,15,W-70,H-55);c.clip();c.strokeStyle='#ffc878';c.beginPath();
    for(let i=0;i<=150;i++){const x=f.radial_pixel_range[0]+i/150*(f.radial_pixel_range[1]-f.radial_pixel_range[0]);const y=f.coefficients.reduce((sum,coef)=>sum*x+coef,0);i?c.lineTo(px(x),py(y)):c.moveTo(px(x),py(y));}c.stroke();c.restore();
    const metric=f.fit_metrics?.find(m=>m.degree===f.selected_degree);
    readings($('fitInfo'),[['Polynomial degree',f.selected_degree],['Selection',f.selection_reason],['Training RMSE',fmt(metric?.training_rmse_mm)+' mm'],['Leave-one-out RMSE',fmt(metric?.leave_one_out_rmse_mm)+' mm']]);
  }else $('fitInfo').textContent='Capture at least two distinct radial positions, then preview the fit.';
}
const mapScale= Math.min((800-60)/2430,(640-70)/1820), mapX=30, mapY=35;
function drawPitch(hardware) {
  const c=$('pitch').getContext('2d'); c.clearRect(0,0,800,640); c.save(); c.translate(mapX,mapY);c.scale(mapScale,mapScale);
  c.strokeStyle='#557080';c.lineWidth=4;c.strokeRect(0,0,2430,1820);
  const line=(x1,y1,x2,y2)=>{c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();};
  c.strokeStyle='#d9b64f';line(0,685,300,685);line(226,685,226,1140);line(0,1135,300,1135);
  c.strokeStyle='#4cb9e0';line(2130,685,2430,685);line(2204,685,2204,1140);line(2130,1135,2430,1135);
  const l=hardware.localisation,p=l?.pose;
  if(p && p.slice(0,3).every(v=>v!=null)) {
    c.fillStyle='#7fa1b5';for(const [angle,distance] of l.scan??[]){const rad=(p[2]+angle)*Math.PI/180,x=p[0]+distance*Math.cos(rad),y=p[1]+distance*Math.sin(rad);if(x>=0&&x<=2430&&y>=0&&y<=1820)c.fillRect(x-3,y-3,6,6);}
    c.strokeStyle='#2d8079';c.beginPath();(hardware.trajectory??[]).forEach((v,i)=>i?c.lineTo(v[0],v[1]):c.moveTo(v[0],v[1]));c.stroke();
    c.save();c.translate(p[0],p[1]);c.rotate(p[2]*Math.PI/180);c.fillStyle=p[4]&&l.fresh?'#6ce3bd':'#e49765';c.beginPath();c.moveTo(80,0);c.lineTo(-40,-45);c.lineTo(-40,45);c.closePath();c.fill();c.restore();
  }
  const x=Number($('targetX').value),y=Number($('targetY').value);c.strokeStyle='#ffce82';line(x-30,y,x+30,y);line(x,y-30,x,y+30);c.beginPath();c.arc(x,y,45,0,Math.PI*2);c.stroke();c.restore();
  c.fillStyle='#a2b5c3';c.font='13px monospace';c.fillText('0,0 · X →   Y ↓ · clockwise yaw',30,22);
}
$('pitch').onclick=event=>{const rect=$('pitch').getBoundingClientRect();$('targetX').value=Math.round(Math.max(0,Math.min(2430,((event.clientX-rect.left)/rect.width*800-mapX)/mapScale)));$('targetY').value=Math.round(Math.max(0,Math.min(1820,((event.clientY-rect.top)/rect.height*640-mapY)/mapScale)));if(state)drawPitch(state.hardware);};
for(const id of ['targetX','targetY'])$(id).oninput=()=>{if(state)drawPitch(state.hardware);};
async function heartbeat(){if(token){try{await api('heartbeat');}catch(e){token=null;message(e.message);}}setTimeout(heartbeat,500);}
async function poll(){try{const response=await fetch('/api/state');if(!response.ok)throw new Error('Status unavailable');render(await response.json());}catch(e){$('health').textContent='DISCONNECTED · motor control will expire automatically';renderLineSensors(null);}setTimeout(poll,300);}
window.addEventListener('pagehide',()=>{if(token)fetch('/api/release',{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:'{}',keepalive:true}).catch(()=>{});});
streams(); heartbeat(); poll();

const manualKeys = new Set();
let joystickVector = [0, 0], joystickPointer = null, manualSending = false, manualRequested = false;
function resetManualInput() {
  manualKeys.clear(); joystickVector = [0, 0]; joystickPointer = null;
  $('joystickKnob').style.transform = '';
}
function stopManual() {
  resetManualInput();
  if (token && (manualRequested || state?.hardware.mode === 'manual' || state?.hardware.mode?.includes('manual_start'))) {
    manualRequested = false;
    api('stop').catch(e => message(e.message));
  }
}
$('manualStart').onclick = async () => {
  resetManualInput(); manualRequested = true;
  try { await api('manual_start'); } catch (e) { manualRequested = false; message(e.message); }
};
handle('manualStop', 'stop');
$('manualStop').addEventListener('click', resetManualInput);
function manualReady() { return token && state?.control.armed && state?.hardware.mode === 'manual' && tab === 'manual' && !document.hidden; }
window.addEventListener('keydown', event => {
  if (!event.key.startsWith('Arrow') || !manualReady() || /INPUT|SELECT|TEXTAREA/.test(event.target.tagName)) return;
  event.preventDefault(); manualKeys.add(event.key);
});
window.addEventListener('keyup', event => {
  if (manualKeys.delete(event.key)) { event.preventDefault(); sendManual(); }
});
$('joystick').addEventListener('pointerdown', event => {
  if (!manualReady()) return;
  joystickPointer = event.pointerId; $('joystick').setPointerCapture(event.pointerId); updateJoystick(event);
});
function updateJoystick(event) {
  if (event.pointerId !== joystickPointer) return;
  const r = $('joystick').getBoundingClientRect();
  let right = (event.clientX - r.left - r.width / 2) / (r.width / 2 - 25);
  let forward = -(event.clientY - r.top - r.height / 2) / (r.height / 2 - 25);
  const scale = Math.max(1, Math.hypot(right, forward)); right /= scale; forward /= scale;
  joystickVector = [forward, right];
  $('joystickKnob').style.transform = `translate(${right * 83}px, ${-forward * 83}px)`;
}
$('joystick').addEventListener('pointermove', updateJoystick);
for (const event of ['pointerup', 'pointercancel', 'lostpointercapture']) $('joystick').addEventListener(event, () => { resetManualInput(); sendManual(); });
window.addEventListener('blur', stopManual);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopManual(); });
for (const button of document.querySelectorAll('[data-tab]')) button.addEventListener('click', () => { if (button.dataset.tab !== 'manual') stopManual(); });
async function sendManual() {
  if (!manualReady() || manualSending) return;
  const keys = manualKeys.size > 0;
  let forward = keys ? Number(manualKeys.has('ArrowUp')) - Number(manualKeys.has('ArrowDown')) : joystickVector[0];
  let right = keys ? Number(manualKeys.has('ArrowRight')) - Number(manualKeys.has('ArrowLeft')) : joystickVector[1];
  const speed = Math.min(1000, Math.max(0, Number($('manualSpeed').value) || 0));
  const scale = speed / Math.max(1, Math.hypot(forward, right));
  forward *= scale; right *= scale;
  manualSending = true;
  try { await api('manual_input', {forward, right}); }
  catch (e) { resetManualInput(); message(e.message); }
  finally { manualSending = false; }
}
setInterval(() => {
  $('manualStart').disabled = !token || !state?.control.armed || !['idle','stopped'].includes(state?.hardware.mode);
  $('manualStatus').textContent = state?.hardware.mode === 'manual' ? 'Manual control active · release to stop' : 'Manual control stopped.';
  if (!manualReady()) resetManualInput();
  sendManual();
}, 100);
