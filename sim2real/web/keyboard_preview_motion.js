/* Pure local ROS2 echo preview interpolation: frame-rate display, not odometry.
 * Model is updated ONLY by a verified ROS2 subscriber HTTP snapshot. A fresh
 * snapshot is extrapolated for at most 120 ms to eliminate 4-Hz visual jumps.
 * No motor control, no scene collision checks, no FSM and no map bounds.
 */
(function (root) {
  'use strict';

  const MAX_PREDICT_MS = 120;

  function valid(snapshot) {
    const v = snapshot && snapshot.velocity;
    return !!(snapshot && snapshot.source_type === 'MOCK_KINEMATIC_ONLY' &&
      snapshot.preview_mode === 'direct' && snapshot.hardware_authorized === false &&
      snapshot.real_odom === false &&
      snapshot.frame === 'field_preview_m_not_odom' &&
      [snapshot.x_m, snapshot.y_m, snapshot.yaw_rad,
       v && v.vx, v && v.vy, v && v.wz].every(Number.isFinite));
  }

  function predict(snapshot, elapsedMs) {
    if (!valid(snapshot)) throw new Error('Invalid local ROS2 MOCK preview snapshot');
    const result = {x_m: snapshot.x_m, y_m: snapshot.y_m, yaw_rad: snapshot.yaw_rad};
    // Never extrapolate stale, STOPPED, wrong-route or unobserved speed.
    if (snapshot.state !== 'RUNNING' ||
        snapshot.input_provenance !== 'OBSERVED_PC_ROS2_LOCAL_TEST_TWIST') return result;

    const dt = Math.min(MAX_PREDICT_MS, Math.max(0,
      Number.isFinite(elapsedMs) ? elapsedMs : 0)) / 1000;
    const speed = snapshot.velocity;
    const midpoint = snapshot.yaw_rad + speed.wz * dt / 2;
    result.x_m += (Math.cos(midpoint) * speed.vx - Math.sin(midpoint) * speed.vy) * dt;
    result.y_m += (Math.sin(midpoint) * speed.vx + Math.cos(midpoint) * speed.vy) * dt;
    const yaw = snapshot.yaw_rad + speed.wz * dt;
    result.yaw_rad = Math.atan2(Math.sin(yaw), Math.cos(yaw));
    return result;
  }

  const api = Object.freeze({predict, valid, MAX_PREDICT_MS});
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (root) root.CarKeyboardPreviewMotion = api;
})(typeof window === 'undefined' ? null : window);
