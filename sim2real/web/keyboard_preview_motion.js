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

  // Display-only smoothing. Unlike a snapshot tracker this does not own any
  // ROS2 state: the existing known-working HTTP poll and RAF loop remain
  // unchanged. Even if RAF stalls, HTTP painting still works independently.
  function smoothPose(displayed, target, elapsedMs) {
    if(!target || ![target.x_m,target.y_m,target.yaw_rad].every(Number.isFinite))
      throw new Error('A finite MOCK target pose is required');
    if(!displayed) return {
      x_m:target.x_m,y_m:target.y_m,yaw_rad:target.yaw_rad
    };
    if(![displayed.x_m,displayed.y_m,displayed.yaw_rad].every(Number.isFinite))
      throw new Error('A finite MOCK displayed pose is required');
    const dt=Math.max(0,Math.min(50,Number.isFinite(elapsedMs)?elapsedMs:0));
    const xyAlpha=-Math.expm1(-dt/55);
    const yawAlpha=-Math.expm1(-dt/65);
    const diff=Math.atan2(
      Math.sin(target.yaw_rad-displayed.yaw_rad),
      Math.cos(target.yaw_rad-displayed.yaw_rad)
    );
    return {
      x_m:displayed.x_m+(target.x_m-displayed.x_m)*xyAlpha,
      y_m:displayed.y_m+(target.y_m-displayed.y_m)*xyAlpha,
      yaw_rad:Math.atan2(
        Math.sin(displayed.yaw_rad+diff*yawAlpha),
        Math.cos(displayed.yaw_rad+diff*yawAlpha))
    };
  }

  const api = Object.freeze({predict, smoothPose, valid, MAX_PREDICT_MS});
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (root) root.CarKeyboardPreviewMotion = api;
})(typeof window === 'undefined' ? null : window);
