/* Small golf ball-flight integrator: gravity + quadratic drag + Magnus lift.
 * Frame: +X toward target, +Y left, +Z up (same as the device API).
 * Inputs use API units (m/s, degrees, rpm). Output lateral is positive to the RIGHT. */
(function (root) {
  'use strict';
  var M = 0.04593, R = 0.02135, A = Math.PI * R * R, G = 9.80665;

  function airDensity(altM) {
    // Isothermal-ish barometric approximation, 15 C at sea level.
    return 1.225 * Math.exp(-(altM || 0) / 8500);
  }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function norm(a) { return Math.hypot(a[0], a[1], a[2]); }

  /* ball: {speed_mps, vla_deg, hla_deg, back_spin_rpm, side_spin_rpm}
   * opts: {altitude_m} */
  function simulate(ball, opts) {
    opts = opts || {};
    var rho = airDensity(opts.altitude_m);
    var v0 = ball.speed_mps, vla = ball.vla_deg * Math.PI / 180, hla = ball.hla_deg * Math.PI / 180;
    // Launch velocity; positive HLA = right = toward -Y.
    var vx = v0 * Math.cos(vla) * Math.cos(hla);
    var vy = -v0 * Math.cos(vla) * Math.sin(hla);
    var vz = v0 * Math.sin(vla);
    var vh = [Math.cos(hla), -Math.sin(hla), 0];       // horizontal direction of travel
    var left = [-vh[1], vh[0], 0];                      // Z x vh: horizontal unit vector to the left
    var vhat = [vx / v0, vy / v0, vz / v0];
    var up = cross(vhat, left);                          // perpendicular to v, roughly +Z
    var back = (ball.back_spin_rpm || 0) * 2 * Math.PI / 60;
    var side = (ball.side_spin_rpm || 0) * 2 * Math.PI / 60;
    // Spin vector: backspin about -left, positive sidespin (curve right) about -up.
    var w = [-back * left[0] - side * up[0], -back * left[1] - side * up[1], -back * left[2] - side * up[2]];
    var p = [0, 0, 0.0], v = [vx, vy, vz];
    var dt = 0.01, t = 0, apex = 0, lastP = p;
    var DECAY = 0.04; // spin decay per second

    function acc(v, w) {
      var sp = norm(v);
      var wm = norm(w);
      var S = wm * R / Math.max(sp, 1);                  // spin ratio
      var Cd = 0.21 + 0.35 * S;                          // drag grows with spin
      var Cl = Math.min(0.30, 0.54 * Math.pow(S, 0.4));  // lift saturates (Smits & Smith style fit)
      var q = 0.5 * rho * A * sp;
      var a = [0, 0, -G];
      // Drag opposes velocity: F = -q*Cd*v
      for (var i = 0; i < 3; i++) a[i] += -q * Cd * v[i] / M;
      // Magnus lift along w x v with magnitude q*Cl*|v|
      var m = cross(w, v), mm = norm(m);
      if (mm > 1e-9) for (i = 0; i < 3; i++) a[i] += q * Cl * sp * m[i] / mm / M;
      return a;
    }

    while (t < 30) {
      // Heun / RK2 step
      var a1 = acc(v, w);
      var vm = [v[0] + a1[0] * dt, v[1] + a1[1] * dt, v[2] + a1[2] * dt];
      var a2 = acc(vm, w);
      var nv = [v[0] + 0.5 * (a1[0] + a2[0]) * dt, v[1] + 0.5 * (a1[1] + a2[1]) * dt, v[2] + 0.5 * (a1[2] + a2[2]) * dt];
      lastP = p;
      p = [p[0] + 0.5 * (v[0] + nv[0]) * dt, p[1] + 0.5 * (v[1] + nv[1]) * dt, p[2] + 0.5 * (v[2] + nv[2]) * dt];
      v = nv; t += dt;
      var d = Math.exp(-DECAY * dt);
      w = [w[0] * d, w[1] * d, w[2] * d];
      if (p[2] > apex) apex = p[2];
      if (p[2] < 0 && t > 0.1) {
        var f = lastP[2] / (lastP[2] - p[2]);            // linear interpolation to z = 0
        var x = lastP[0] + (p[0] - lastP[0]) * f, y = lastP[1] + (p[1] - lastP[1]) * f;
        return { carry_m: x, lateral_m: -y, apex_m: apex, flight_s: t - dt * (1 - f), total_m: Math.hypot(x, y) };
      }
    }
    return { carry_m: p[0], lateral_m: -p[1], apex_m: apex, flight_s: t, total_m: Math.hypot(p[0], p[1]) };
  }

  var api = { simulate: simulate };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.GSFlight = api;
})(typeof window !== 'undefined' ? window : globalThis);
