/* Local ROS2 MOCK echo preview: smooth 60Hz rendering without pose snapping.
 * Only verified /api/keyboard/dryrun samples may define the target pose.
 * This module never emits motor, ROS2 or HTTP messages. Visualization != odom.
 */
(function(root) {
  'use strict';
  const MAX_PREDICT_MS = 120; // cap extrapolation after the last ROS2-backed pose
  const POSITION_TAU_MS = 58; // display-only smoothing, not vehicle acceleration
  const YAW_TAU_MS = 65;
  const MAX_FRAME_MS = 50; // tab resume must not take a giant smoothing step

  function valid(snapshot) {
    const v = snapshot && snapshot.velocity;
    return !!(snapshot && snapshot.source_type === 'MOCK_KINEMATIC_ONLY' &&
      snapshot.preview_mode === 'direct' && snapshot.hardware_authorized === false &&
      snapshot.real_odom === false &&
      snapshot.frame === 'field_preview_m_not_odom' &&
      [snapshot.x_m, snapshot.y_m, snapshot.yaw_rad,
       v && v.vx, v && v.vy, v && v.wz].every(Number.isFinite));
  }
  function angle(rad) {
    return Math.atan2(Math.sin(rad),Math.cos(rad));
  }
  function copyPose(s) {
    return {x_m:s.x_m,y_m:s.y_m,yaw_rad:angle(s.yaw_rad)};
  }
  function predict(snapshot, elapsedMs) {
    if (!valid(snapshot)) throw new Error('Invalid local ROS2 MOCK preview snapshot');
    const result = copyPose(snapshot);
    // An HTTP command alone cannot generate motion; the server has to have
    // accepted a fresh subscribed local ROS2 message with active deadman.
    if (snapshot.state !== 'RUNNING' ||
        snapshot.input_provenance !== 'OBSERVED_PC_ROS2_LOCAL_TEST_TWIST') return result;
    const dt = Math.min(MAX_PREDICT_MS, Math.max(0,
      Number.isFinite(elapsedMs) ? elapsedMs : 0)) / 1000;
    const speed = snapshot.velocity;
    const mid = snapshot.yaw_rad + speed.wz * dt / 2;
    result.x_m += (Math.cos(mid)*speed.vx-Math.sin(mid)*speed.vy)*dt;
    result.y_m += (Math.sin(mid)*speed.vx+Math.cos(mid)*speed.vy)*dt;
    result.yaw_rad = angle(snapshot.yaw_rad+speed.wz*dt);
    return result;
  }

  function createTracker() {
    // The render pose survives new network packets: a new sample updates only
    // the target. This eliminates the 75ms snap-back that previously occurred
    // after EVERY /dryrun response.
    let source=null, receivedAt=0, lastFrameAt=null, pose=null;
    let generation=null, sampleCount=-1, simulationTime=-Infinity;
    function reset(snapshot, now) {
      if (!valid(snapshot) || !Number.isFinite(now))
        throw new Error('Reset requires finite time and a valid MOCK sample');
      source=snapshot;receivedAt=now;lastFrameAt=now;
      pose=copyPose(snapshot);
      generation=snapshot.generation??null;
      sampleCount=Number.isFinite(snapshot.samples)?snapshot.samples:-1;
      simulationTime=Number.isFinite(snapshot.sim_time_s)?snapshot.sim_time_s:-Infinity;
      return copyPose(pose);
    }
    function ingest(snapshot, now) {
      if (!valid(snapshot) || !Number.isFinite(now))
        throw new Error('Invalid ROS2-backed motion sample');
      if (!pose || generation!==(snapshot.generation??null)) {
        reset(snapshot,now);return true;
      }
      // Single GET in flight in the caller; these checks also protect
      // against delayed responses / older simulation states.
      const nextCount=Number.isFinite(snapshot.samples)?snapshot.samples:-1;
      const nextTime=Number.isFinite(snapshot.sim_time_s)?snapshot.sim_time_s:-Infinity;
      if(nextCount<sampleCount || nextTime<simulationTime-0.001)
        return false;
      source=snapshot;receivedAt=now;
      sampleCount=nextCount;simulationTime=nextTime;
      return true;
    }
    function frame(now) {
      if(!pose)return null;
      if(!Number.isFinite(now))return copyPose(pose);
      if(!source) {
        lastFrameAt=now;
        return copyPose(pose); // immediate local STOP freeze
      }
      const elapsed=lastFrameAt===null?0:
        Math.max(0,Math.min(MAX_FRAME_MS,now-lastFrameAt));
      lastFrameAt=now;
      if(elapsed===0)return copyPose(pose);
      // Each frame corrects a fraction of the remaining geometric
      // discrepancy. The smoothing curve is frame-rate independent.
      const objective=predict(source,now-receivedAt);
      const xyGain=-Math.expm1(-elapsed/POSITION_TAU_MS);
      const yawGain=-Math.expm1(-elapsed/YAW_TAU_MS);
      pose.x_m+=(objective.x_m-pose.x_m)*xyGain;
      pose.y_m+=(objective.y_m-pose.y_m)*xyGain;
      // Blend the shortest circular difference: +179 -> -179 is 2 degrees,
      // not a disruptive full revolution.
      pose.yaw_rad=angle(pose.yaw_rad+
        angle(objective.yaw_rad-pose.yaw_rad)*yawGain);
      return copyPose(pose);
    }
    function freeze(now) {
      // The caller's deadman/keyup/blur event freezes the visual immediately,
      // without pretending that a fresh ROS2 receipt has arrived.
      const stopped=frame(now);
      source=null;lastFrameAt=Number.isFinite(now)?now:null;
      return stopped;
    }
    function clear() {
      source=null;pose=null;generation=null;
      receivedAt=0;lastFrameAt=null;sampleCount=-1;
      simulationTime=-Infinity;
    }
    function hasSource(){return source!==null;}
    return Object.freeze({ingest,frame,freeze,reset,clear,hasSource});
  }
  const api=Object.freeze({valid,predict,createTracker,MAX_PREDICT_MS,
    POSITION_TAU_MS,YAW_TAU_MS});
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  if(root)root.CarKeyboardPreviewMotion=api;
})(typeof window==='undefined'?null:window);
