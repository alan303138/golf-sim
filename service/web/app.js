/* Golf simulator phone app. Plain JS, no dependencies. UI language: Traditional Chinese. */
(function () {
  'use strict';

  // ------------------------------------------------------------------ helpers
  var $ = function (id) { return document.getElementById(id); };
  var store = {
    get: function (k, d) { try { var v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } },
    set: function (k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } }
  };
  function esc(s) { return String(s).replace(/[&<>"']/g, function (c) { return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]; }); }
  function fmt(n, d) { return (n === null || n === undefined || isNaN(n)) ? '--' : Number(n).toFixed(d === undefined ? 1 : d); }
  function toast(msg) {
    var t = $('toast'); t.textContent = msg; t.classList.add('show');
    clearTimeout(toast._t); toast._t = setTimeout(function () { t.classList.remove('show'); }, 1800);
  }
  function api(method, path, body) {
    var opt = { method: method, headers: {} };
    if (body !== undefined) { opt.headers['Content-Type'] = 'application/json'; opt.body = JSON.stringify(body); }
    return fetch(path, opt).then(function (r) {
      return r.json().catch(function () { return {}; }).then(function (j) {
        if (!r.ok) { var e = new Error('HTTP ' + r.status); e.body = j; throw e; }
        return j;
      });
    });
  }

  // ------------------------------------------------------------------ state
  var DEFAULT_CFG = { distance_unit: 'yd', speed_unit: 'mph', handedness: 'right', altitude_m: 0, placement_distance_m: 2.0,
    gspro_enabled: false, gspro_host: '127.0.0.1', gspro_port: 921 };
  var S = {
    cfg: Object.assign({}, DEFAULT_CFG),
    shots: [],                  // newest first
    last: null,
    status: { state: 'idle', ball_detected: false, message: '' },
    hello: null, detail: null, upd: null,
    conn: false,
    scope: store.get('gs.scope', 'session'),
    sessionStart: store.get('gs.sessionStart', ''),
    screen: 'live', expanded: null, confirmDel: null
  };
  if (!S.sessionStart) { S.sessionStart = new Date().toISOString(); store.set('gs.sessionStart', S.sessionStart); }
  try { var cc = JSON.parse(store.get('gs.cfg', 'null')); if (cc) Object.assign(S.cfg, cc); } catch (e) { /* ignore */ }

  // ------------------------------------------------------------------ units
  var SPEED = { mph: { f: 2.2369363, l: 'mph' }, kmh: { f: 3.6, l: 'km/h' }, ms: { f: 1, l: 'm/s' } };
  var DIST = { yd: { f: 1.0936133, l: '碼' }, m: { f: 1, l: '公尺' } };
  function spd(mps) { return mps * SPEED[S.cfg.speed_unit].f; }
  function dist(m) { return m * DIST[S.cfg.distance_unit].f; }
  function spdL() { return SPEED[S.cfg.speed_unit].l; }
  function distL() { return DIST[S.cfg.distance_unit].l; }

  var flightCache = {};
  function flight(shot) {
    var k = shot.shot_id + '|' + S.cfg.altitude_m;
    if (!flightCache[k]) flightCache[k] = GSFlight.simulate(shot.ball, { altitude_m: S.cfg.altitude_m });
    return flightCache[k];
  }

  // ------------------------------------------------------------------ navigation
  function show(screen) {
    S.screen = screen;
    ['live', 'history', 'settings', 'device'].forEach(function (n) {
      $('screen-' + n).classList.toggle('active', n === screen);
    });
    document.querySelectorAll('.tabs button').forEach(function (b) { b.classList.toggle('on', b.dataset.screen === screen); });
    window.scrollTo(0, 0);
    if (screen === 'device') refreshDevice();
    if (screen === 'history') renderHistory();
  }
  document.querySelectorAll('.tabs button').forEach(function (b) {
    b.addEventListener('click', function () { show(b.dataset.screen); });
  });

  // ------------------------------------------------------------------ WebSocket
  var ws = null, backoff = 500, lastMsg = 0, pingT = null, wdT = null;
  function connect() {
    var url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/v1';
    try { ws = new WebSocket(url); } catch (e) { scheduleReconnect(); return; }
    ws.onopen = function () { backoff = 500; lastMsg = Date.now(); };
    ws.onmessage = function (ev) {
      lastMsg = Date.now();
      var m; try { m = JSON.parse(ev.data); } catch (e) { return; }
      handle(m);
    };
    ws.onclose = function () { setConn(false); clearInterval(pingT); clearInterval(wdT); scheduleReconnect(); };
    ws.onerror = function () { try { ws.close(); } catch (e) { /* ignore */ } };
    clearInterval(pingT); clearInterval(wdT);
    pingT = setInterval(function () { wsSend({ type: 'ping' }); }, 10000);
    // Status arrives every 2 s; silence means a dead link (e.g. phone left Wi-Fi).
    wdT = setInterval(function () { if (Date.now() - lastMsg > 7000) { try { ws.close(); } catch (e) { /* ignore */ } } }, 2000);
  }
  function scheduleReconnect() { setTimeout(connect, backoff); backoff = Math.min(backoff * 1.7, 5000); }
  function wsSend(o) { if (ws && ws.readyState === 1) { ws.send(JSON.stringify(o)); return true; } return false; }
  function setConn(on) {
    if (S.conn === on) return;
    S.conn = on; renderConn();
  }
  function handle(m) {
    switch (m.type) {
      case 'hello': S.hello = m; setConn(true); loadHistory(); loadConfig(); if (S.screen === 'device') refreshDevice(); break;
      case 'status': S.status = m; renderStatus(); break;
      case 'shot': addShot(m, true); break;
      case 'update': S.upd = Object.assign(S.upd || {}, { phase: m.phase, state: m.phase, progress: m.progress, message: m.message });
        if (m.phase === 'up_to_date' || m.phase === 'failed') refreshDevice(); else renderDevice(); break;
      default: break;
    }
  }

  // ------------------------------------------------------------------ data
  function addShot(shot, live) {
    if (S.shots.some(function (s) { return s.shot_id === shot.shot_id; })) return;
    S.shots.unshift(shot);
    S.shots.sort(function (a, b) { return a.ts < b.ts ? 1 : a.ts > b.ts ? -1 : 0; });
    if (live || !S.last) S.last = shot;
    if (live) { renderLive(true); renderHistory(); }
  }
  function loadHistory() {
    return api('GET', '/api/v1/shots?limit=200').then(function (r) {
      var have = {}; S.shots.forEach(function (s) { have[s.shot_id] = 1; });
      r.shots.forEach(function (s) { if (!have[s.shot_id]) S.shots.push(s); });
      S.shots.sort(function (a, b) { return a.ts < b.ts ? 1 : a.ts > b.ts ? -1 : 0; });
      if (!S.last && S.shots.length) S.last = S.shots[0];
      renderLive(false); renderHistory();
    }).catch(function () { /* offline */ });
  }
  function loadConfig() {
    return api('GET', '/api/v1/config').then(function (c) { S.cfg = Object.assign({}, DEFAULT_CFG, c); store.set('gs.cfg', JSON.stringify(S.cfg)); renderAll(); })
      .catch(function () { /* keep cached */ });
  }
  function saveConfig(patch) {
    return api('PUT', '/api/v1/config', patch).then(function (c) {
      S.cfg = Object.assign({}, DEFAULT_CFG, c); store.set('gs.cfg', JSON.stringify(S.cfg)); renderAll(); toast('已儲存');
    }).catch(function (e) {
      toast('儲存失敗'); loadConfig();
    });
  }
  function refreshDevice() {
    api('GET', '/api/v1/status').then(function (d) { S.detail = d; renderDevice(); }).catch(function () { });
    api('GET', '/api/v1/update').then(function (u) { S.upd = u; renderDevice(); }).catch(function () { });
  }
  setInterval(function () { if (S.screen === 'device' && S.conn) refreshDevice(); }, 5000);

  // ------------------------------------------------------------------ render: live
  var STATE_L = { idle: '待機', ready: '就緒', processing: '處理中', error: '錯誤', updating: '更新中' };
  function renderConn() {
    var p = $('connPill');
    p.className = 'pill ' + (S.conn ? 'on' : 'off');
    $('connText').textContent = S.conn ? '已連線' : '重新連線中…';
  }
  function renderStatus() {
    var st = S.status;
    var b = $('stateBadge'); b.className = 'badge ' + st.state; b.textContent = STATE_L[st.state] || st.state;
    var bp = $('ballPill'); bp.className = 'pill ' + (st.ball_detected ? 'on' : 'off');
    $('ballText').textContent = st.ball_detected ? '球已就位' : '未偵測到球';
    $('stateMsg').textContent = st.message || '';
    var armed = st.state === 'ready' || st.state === 'processing';
    var btn = $('armBtn');
    btn.textContent = armed ? '停用' : '啟動（就緒）';
    btn.classList.toggle('primary', !armed);
    btn.disabled = st.state === 'updating';
  }
  function dirText(v, pos, neg, zero) { return v > 0.05 ? pos : v < -0.05 ? neg : zero; }
  function renderLive(flash) {
    document.querySelectorAll('.u-dist').forEach(function (e) { e.textContent = distL(); });
    document.querySelectorAll('.u-speed').forEach(function (e) { e.textContent = spdL(); });
    $('u-ball').textContent = spdL();
    var s = S.last, card = $('shotCard');
    card.classList.toggle('empty', !s);
    if (!s) return;
    var b = s.ball, f = flight(s);
    $('v-ball').textContent = fmt(spd(b.speed_mps), 1);
    $('v-carry').textContent = fmt(dist(f.carry_m), 0);
    $('v-vla').textContent = fmt(b.vla_deg, 1);
    $('v-hla').textContent = fmt(Math.abs(b.hla_deg), 1);
    $('s-hla').textContent = dirText(b.hla_deg, '向右', '向左', '正對目標');
    $('v-back').textContent = fmt(b.back_spin_rpm, 0);
    $('v-side').textContent = fmt(Math.abs(b.side_spin_rpm), 0);
    $('s-side').textContent = dirText(b.side_spin_rpm, '右旋（球往右彎）', '左旋（球往左彎）', '無側旋');
    $('v-club').textContent = s.club ? fmt(spd(s.club.speed_mps), 1) : '--';
    $('s-smash').textContent = s.club && s.club.speed_mps > 0 ? '擊球效率 ' + fmt(b.speed_mps / s.club.speed_mps, 2) : '';
    var d = new Date(s.ts);
    $('shotTime').textContent = '最近一桿 ' + d.toLocaleTimeString('zh-TW', { hour12: false });
    $('shotConf').textContent = '可信度 ' + Math.round((s.quality && s.quality.confidence || 0) * 100) + '%' +
      (s.quality && !s.quality.spin_measured ? '・旋轉為估計值' : '');
    var lat = f.lateral_m;
    $('shotExtra').textContent = '預估值不含落地後滾動。最高點 ' + fmt(dist(f.apex_m), 0) + ' ' + distL() + '，滯空 ' + fmt(f.flight_s, 1) +
      ' 秒，落點偏' + dirText(lat, '右', '左', '中') + ' ' + fmt(Math.abs(dist(lat)), 0) + ' ' + distL() + '。';
    if (flash) { card.classList.remove('flash'); void card.offsetWidth; card.classList.add('flash'); }
  }

  // ------------------------------------------------------------------ render: history
  function scopedShots() {
    if (S.scope === 'all') return S.shots;
    return S.shots.filter(function (s) { return s.ts >= S.sessionStart; });
  }
  function mean(a) { return a.length ? a.reduce(function (x, y) { return x + y; }, 0) / a.length : NaN; }
  function sd(a) { if (a.length < 2) return NaN; var m = mean(a); return Math.sqrt(mean(a.map(function (x) { return (x - m) * (x - m); }))); }
  function niceStep(range, target) {
    var raw = range / target, p = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / p;
    return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * p;
  }
  function renderHistory() {
    if (S.screen !== 'history') return;
    $('scopeSession').classList.toggle('on', S.scope === 'session');
    $('scopeAll').classList.toggle('on', S.scope === 'all');
    var list = scopedShots();
    var fl = list.map(flight);
    var items = [
      ['擊球數', String(list.length), ''],
      ['平均球速', fmt(mean(list.map(function (s) { return spd(s.ball.speed_mps); })), 1), spdL()],
      ['平均飛行距離', fmt(dist(mean(fl.map(function (f) { return f.carry_m; }))), 0), distL()],
      ['平均發射角', fmt(mean(list.map(function (s) { return s.ball.vla_deg; })), 1), '°'],
      ['平均後旋', fmt(mean(list.map(function (s) { return s.ball.back_spin_rpm; })), 0), 'rpm'],
      ['平均方向', (function () { var m = mean(list.map(function (s) { return s.ball.hla_deg; })); return isNaN(m) ? '--' : dirText(m, '右 ', '左 ', '') + fmt(Math.abs(m), 1); })(), '°'],
      ['距離離散度 (σ)', fmt(dist(sd(fl.map(function (f) { return f.carry_m; }))), 1), distL()],
      ['橫向離散度 (σ)', fmt(dist(sd(fl.map(function (f) { return f.lateral_m; }))), 1), distL()],
      ['平均桿頭速度', fmt(mean(list.filter(function (s) { return s.club; }).map(function (s) { return spd(s.club.speed_mps); })), 1), spdL()]
    ];
    $('stats').innerHTML = items.map(function (i) { return '<div class="stat"><span>' + esc(i[0]) + '</span><b>' + esc(i[1]) + '<small class="muted"> ' + esc(i[2]) + '</small></b></div>'; }).join('');
    $('plotWrap').innerHTML = plotSvg(list, fl);
    $('plotNote').textContent = list.length ? '藍色大點為最近一桿；虛線橢圓為 1σ 範圍，十字為平均落點。橫向比例已放大。' : '';
    $('histCount').textContent = '（' + list.length + '）';
    $('histEmpty').hidden = list.length > 0;
    var ul = $('histList');
    ul.innerHTML = list.slice(0, 100).map(function (s, idx) {
      var f = flight(s), t = new Date(s.ts), open = S.expanded === s.shot_id;
      var inSess = s.ts >= S.sessionStart;
      return '<li><button class="hrow" data-id="' + esc(s.shot_id) + '" aria-expanded="' + open + '">' +
        '<span class="t">' + (inSess ? '<i class="dot-new"></i>' : '') + t.toLocaleString('zh-TW', { hour12: false, month: 'numeric', day: 'numeric', hour: '2-digit', minute: '2-digit', second: '2-digit' }) + '</span>' +
        '<span class="v">' + fmt(spd(s.ball.speed_mps), 1) + '<small>' + spdL() + '</small></span>' +
        '<span class="v">' + fmt(dist(f.carry_m), 0) + '<small>' + distL() + '</small></span>' +
        '<span class="v">' + dirText(s.ball.hla_deg, '右', '左', '') + fmt(Math.abs(s.ball.hla_deg), 1) + '<small>°</small></span></button>' +
        '<div class="detail' + (open ? ' open' : '') + '"><div class="stats">' +
        stat('發射角', fmt(s.ball.vla_deg, 1) + '°') + stat('後旋', fmt(s.ball.back_spin_rpm, 0)) + stat('側旋', fmt(s.ball.side_spin_rpm, 0)) +
        stat('總旋轉', fmt(s.ball.total_spin_rpm, 0)) + stat('旋轉軸', fmt(s.ball.spin_axis_deg, 1) + '°') +
        stat('桿速', s.club ? fmt(spd(s.club.speed_mps), 1) : '--') + '</div>' +
        '<button class="btn danger del" data-id="' + esc(s.shot_id) + '" type="button">' + (S.confirmDel === s.shot_id ? '再按一次確認刪除' : '刪除這一桿') + '</button></div></li>';
    }).join('');
  }
  function stat(l, v) { return '<div class="stat"><span>' + esc(l) + '</span><b>' + esc(v) + '</b></div>'; }
  $('histList').addEventListener('click', function (e) {
    var del = e.target.closest('.del');
    if (del) {
      var id = del.dataset.id;
      if (S.confirmDel !== id) { S.confirmDel = id; renderHistory(); return; }
      api('DELETE', '/api/v1/shots/' + encodeURIComponent(id)).then(function () {
        S.shots = S.shots.filter(function (s) { return s.shot_id !== id; });
        if (S.last && S.last.shot_id === id) S.last = S.shots[0] || null;
        S.confirmDel = null; renderHistory(); renderLive(false); toast('已刪除');
      }).catch(function () { toast('刪除失敗'); });
      return;
    }
    var row = e.target.closest('.hrow');
    if (row) { S.expanded = S.expanded === row.dataset.id ? null : row.dataset.id; S.confirmDel = null; renderHistory(); }
  });
  $('scopeSession').addEventListener('click', function () { S.scope = 'session'; store.set('gs.scope', S.scope); renderHistory(); });
  $('scopeAll').addEventListener('click', function () { S.scope = 'all'; store.set('gs.scope', S.scope); renderHistory(); });
  $('newSessionBtn').addEventListener('click', function () {
    S.sessionStart = new Date().toISOString(); store.set('gs.sessionStart', S.sessionStart); S.scope = 'session'; renderHistory(); toast('已開始新的練習');
  });

  // Top-down dispersion plot: x = lateral (right positive), y = carry. Axes use different scales.
  function plotSvg(list, fl) {
    var W = 320, H = 340, L = 38, Rr = 12, T = 12, B = 26, pw = W - L - Rr, ph = H - T - B;
    var maxC = 0, maxL = 0;
    fl.forEach(function (f) { maxC = Math.max(maxC, f.carry_m); maxL = Math.max(maxL, Math.abs(f.lateral_m)); });
    if (!list.length) { maxC = 200 / 1.0936; maxL = 15; }
    var yMaxD = Math.ceil(dist(maxC * 1.1) / niceStep(dist(maxC * 1.1), 4)) * niceStep(dist(maxC * 1.1), 4);
    var xHalfD = Math.max(dist(maxL * 1.3), S.cfg.distance_unit === 'yd' ? 15 : 12);
    var xs = niceStep(xHalfD * 2, 4); xHalfD = Math.ceil(xHalfD / xs) * xs;
    var ys = niceStep(yMaxD, 4);
    function X(d) { return L + pw / 2 + d / xHalfD * (pw / 2); }
    function Y(d) { return T + ph - d / yMaxD * ph; }
    var o = '<svg viewBox="0 0 ' + W + ' ' + H + '" role="img" aria-label="落點分佈圖"><rect class="plot-bg" x="' + L + '" y="' + T + '" width="' + pw + '" height="' + ph + '" rx="8"/>';
    for (var y = ys; y <= yMaxD + 1e-6; y += ys) o += '<line class="plot-grid" x1="' + L + '" x2="' + (L + pw) + '" y1="' + Y(y) + '" y2="' + Y(y) + '"/><text class="plot-text" x="' + (L - 4) + '" y="' + (Y(y) + 4) + '" text-anchor="end">' + Math.round(y) + '</text>';
    for (var x = -Math.floor(xHalfD / xs) * xs; x <= xHalfD + 1e-6; x += xs) {
      if (Math.abs(x) < 1e-6) continue;
      o += '<line class="plot-grid" y1="' + T + '" y2="' + (T + ph) + '" x1="' + X(x) + '" x2="' + X(x) + '"/><text class="plot-text" x="' + X(x) + '" y="' + (H - 8) + '" text-anchor="middle">' + (x > 0 ? '右' : '左') + Math.abs(Math.round(x)) + '</text>';
    }
    o += '<line class="plot-axis" x1="' + X(0) + '" x2="' + X(0) + '" y1="' + T + '" y2="' + (T + ph) + '"/>';
    o += '<text class="plot-text" x="' + (L + 4) + '" y="' + (T + 12) + '">' + distL() + '</text>';
    if (fl.length >= 3) {
      var mc = mean(fl.map(function (f) { return dist(f.carry_m); })), ml = mean(fl.map(function (f) { return dist(f.lateral_m); }));
      var sc = sd(fl.map(function (f) { return dist(f.carry_m); })), sl = sd(fl.map(function (f) { return dist(f.lateral_m); }));
      o += '<ellipse class="plot-ell" cx="' + X(ml) + '" cy="' + Y(mc) + '" rx="' + Math.max(2, sl / xHalfD * pw / 2) + '" ry="' + Math.max(2, sc / yMaxD * ph) + '"/>';
      o += '<path class="plot-mean" d="M' + (X(ml) - 6) + ' ' + Y(mc) + 'h12M' + X(ml) + ' ' + (Y(mc) - 6) + 'v12"/>';
    }
    var lastId = S.last && S.last.shot_id;
    for (var i = list.length - 1; i >= 0; i--) {
      var isLast = list[i].shot_id === lastId;
      if (isLast) continue;
      o += '<circle class="plot-dot" cx="' + X(dist(fl[i].lateral_m)) + '" cy="' + Y(dist(fl[i].carry_m)) + '" r="5"/>';
    }
    list.forEach(function (s, j) { if (s.shot_id === lastId) o += '<circle class="plot-dot last" cx="' + X(dist(fl[j].lateral_m)) + '" cy="' + Y(dist(fl[j].carry_m)) + '" r="8"/>'; });
    return o + '</svg>';
  }

  // ------------------------------------------------------------------ render: settings
  function setSeg(id, val) {
    var seg = $(id); if (!seg) return;
    seg.querySelectorAll('button').forEach(function (b) { b.classList.toggle('on', b.dataset.v === String(val)); });
  }
  function renderSettings() {
    var c = S.cfg;
    setSeg('seg-distance_unit', c.distance_unit); setSeg('seg-speed_unit', c.speed_unit);
    setSeg('seg-preset', c.distance_unit === 'm' && c.speed_unit === 'kmh' ? 'metric' : c.distance_unit === 'yd' && c.speed_unit === 'mph' ? 'imperial' : '');
    setSeg('seg-handedness', c.handedness); setSeg('seg-gspro_enabled', c.gspro_enabled);
    setSeg('seg-theme', store.get('gs.theme', 'auto'));
    [['altitude_m', c.altitude_m], ['placement_distance_m', c.placement_distance_m], ['gspro_host', c.gspro_host], ['gspro_port', c.gspro_port]].forEach(function (p) {
      var el = $('in-' + p[0]); if (el !== document.activeElement) el.value = p[1];
    });
    var g = S.detail && S.detail.gspro;
    $('gsproState').textContent = !c.gspro_enabled ? '' : (g && g.connected ? '已連上 GSPro。' : '尚未連上 GSPro，會自動重試。');
  }
  document.querySelectorAll('#screen-settings .seg').forEach(function (seg) {
    seg.addEventListener('click', function (e) {
      var b = e.target.closest('button'); if (!b) return;
      var key = seg.id.replace('seg-', ''), v = b.dataset.v;
      if (key === 'theme') { store.set('gs.theme', v); applyTheme(); renderSettings(); return; }
      if (key === 'preset') { saveConfig(v === 'metric' ? { distance_unit: 'm', speed_unit: 'kmh' } : { distance_unit: 'yd', speed_unit: 'mph' }); return; }
      var patch = {}; patch[key] = key === 'gspro_enabled' ? v === 'true' : v; saveConfig(patch);
    });
  });
  var debounce = {};
  ['altitude_m', 'placement_distance_m', 'gspro_host', 'gspro_port'].forEach(function (k) {
    $('in-' + k).addEventListener('input', function (e) {
      clearTimeout(debounce[k]);
      debounce[k] = setTimeout(function () {
        var v = e.target.value; if (v === '') return;
        var patch = {}; patch[k] = (k === 'gspro_host') ? v : Number(v); saveConfig(patch);
      }, 700);
    });
  });
  function applyTheme() {
    var t = store.get('gs.theme', 'auto'), r = document.documentElement;
    if (t === 'auto') r.removeAttribute('data-theme'); else r.setAttribute('data-theme', t);
  }

  // ------------------------------------------------------------------ render: device
  var PHASE_L = { idle: '閒置', checking: '檢查中', available: '有可用更新', downloading: '下載中', verifying: '驗證中', installing: '安裝中',
    rebooting: '重新啟動中', failed: '更新失敗', up_to_date: '已是最新版本' };
  var HEALTH_L = { camera_left: '左相機', camera_right: '右相機', radar: '雷達', mcu: '計時 MCU' };
  var BUSY = { downloading: 1, verifying: 1, installing: 1, rebooting: 1, checking: 1 };
  function kv(k, v) { return '<dt>' + esc(k) + '</dt><dd>' + esc(v) + '</dd>'; }
  function renderDevice() {
    var d = S.detail || {}, h = S.hello || {};
    $('devInfo').innerHTML = kv('裝置 ID', d.device_id || h.device_id || '--') + kv('軟體版本', d.sw_version || h.sw_version || '--') +
      kv('韌體版本', (d.fw_version || h.fw_version) || '未知') + kv('API 版本', d.api || h.api || '--') +
      kv('運作時間', d.uptime_s !== undefined ? fmtUptime(d.uptime_s) : '--') + kv('已連線裝置數', d.clients !== undefined ? d.clients : '--') +
      kv('已儲存擊球數', d.shots_stored !== undefined ? d.shots_stored : '--') + kv('資料來源', d.source || '--');
    var hl = d.health || {}, keys = Object.keys(hl);
    $('health').innerHTML = keys.length ? keys.map(function (k) {
      var v = hl[k], ok = v && typeof v === 'object' ? v.ok !== false : !!v;
      var extra = v && typeof v === 'object' ? Object.keys(v).filter(function (x) { return x !== 'ok'; }).map(function (x) { return x + ': ' + v[x]; }).join(' · ') : (typeof v === 'string' ? v : '');
      return '<li><span class="nm">' + esc(HEALTH_L[k] || k) + '</span><span class="dt">' + esc(extra) + ' <span class="pill ' + (ok ? 'on' : 'off') + '"><i></i>' + (ok ? '正常' : '異常') + '</span></span></li>';
    }).join('') : '<li class="muted">沒有感測器資訊</li>';
    var u = S.upd || {}, cur = u.current || {}, av = u.available || {};
    var ph = u.phase || u.state || 'idle';
    $('updInfo').innerHTML = Object.keys(cur).map(function (k) { return kv('目前版本・' + k, cur[k]); }).join('') +
      (Object.keys(av).length ? Object.keys(av).map(function (k) { return kv('可更新版本・' + k, av[k]); }).join('') : kv('可更新版本', '無')) + kv('狀態', PHASE_L[ph] || ph);
    var busy = !!BUSY[ph];
    $('updProgressWrap').hidden = !(busy && ph !== 'checking') && ph !== 'up_to_date';
    $('updBar').style.width = Math.round((ph === 'up_to_date' ? 1 : (u.progress || 0)) * 100) + '%';
    $('updMsg').textContent = (u.message || '') + (u.notes ? '　' + u.notes : '');
    $('updCheck').disabled = busy;
    $('updApply').disabled = !(ph === 'available' && Object.keys(av).length);
    renderSettings();
  }
  function fmtUptime(s) {
    var d = Math.floor(s / 86400), h = Math.floor(s % 86400 / 3600), m = Math.floor(s % 3600 / 60);
    return (d ? d + ' 天 ' : '') + (h ? h + ' 小時 ' : '') + m + ' 分';
  }
  $('updCheck').addEventListener('click', function () {
    $('updCheck').disabled = true;
    api('POST', '/api/v1/update/check').then(function (u) { S.upd = u; renderDevice(); }).catch(function () { toast('檢查失敗'); renderDevice(); });
  });
  $('updApply').addEventListener('click', function () {
    api('POST', '/api/v1/update/apply').then(function (u) { S.upd = u; renderDevice(); toast('開始更新'); }).catch(function () { toast('無法開始更新'); });
  });

  // ------------------------------------------------------------------ actions
  $('armBtn').addEventListener('click', function () {
    var armed = S.status.state === 'ready' || S.status.state === 'processing';
    var t = armed ? 'disarm' : 'arm';
    if (!wsSend({ type: t })) api('POST', '/api/v1/' + t).then(function (m) { S.status = m; renderStatus(); }).catch(function () { toast('無法連線'); });
  });
  $('testBtn').addEventListener('click', function () {
    api('POST', '/api/v1/sim/shot').then(function (shot) { addShot(shot, true); }).catch(function () { toast('無法送出測試擊球'); });
  });

  // ------------------------------------------------------------------ boot
  function renderAll() { renderConn(); renderStatus(); renderLive(false); renderHistory(); renderSettings(); renderDevice(); }
  applyTheme(); renderAll(); connect();
  if ('serviceWorker' in navigator) {
    window.addEventListener('load', function () { navigator.serviceWorker.register('/sw.js').catch(function () { /* needs https or localhost */ }); });
  }
  window.__gs = S; // handy for debugging / tests
})();
