// 特征提取函数 - 复制到前端使用
function extractFeatures(trajectory) {
  if (!trajectory || trajectory.length < 3) {
    return new Array(16).fill(0)
  }
  
  const points = trajectory
  const n = points.length
  
  // 1. 速度特征
  const speeds = []
  for (let i = 1; i < n; i++) {
    const dx = points[i].x - points[i-1].x
    const dy = points[i].y - points[i-1].y
    const dt = Math.max(points[i].t - points[i-1].t, 1)
    speeds.push(Math.sqrt(dx*dx + dy*dy) / dt)
  }
  const avgSpeed = speeds.reduce((a,b) => a+b, 0) / speeds.length
  const maxSpeed = Math.max(...speeds)
  const speedVariance = speeds.reduce((s, v) => s + Math.pow(v - avgSpeed, 2), 0) / speeds.length
  
  // 2. 加速度特征
  const accels = []
  for (let i = 1; i < speeds.length; i++) {
    accels.push(Math.abs(speeds[i] - speeds[i-1]))
  }
  const avgAccel = accels.length > 0 ? accels.reduce((a,b) => a+b, 0) / accels.length : 0
  const maxAccel = accels.length > 0 ? Math.max(...accels) : 0
  
  // 3. 角度变化（方向变化）
  const angles = []
  for (let i = 2; i < n; i++) {
    const v1x = points[i-1].x - points[i-2].x
    const v1y = points[i-1].y - points[i-2].y
    const v2x = points[i].x - points[i-1].x
    const v2y = points[i].y - points[i-1].y
    const dot = v1x * v2x + v1y * v2y
    const cross = v1x * v2y - v1y * v2x
    angles.push(Math.atan2(cross, dot))
  }
  const avgAngle = angles.length > 0 ? angles.reduce((a,b) => a + Math.abs(b), 0) / angles.length : 0
  const angleVariance = angles.length > 0 
    ? angles.reduce((s, v) => s + Math.pow(v, 2), 0) / angles.length 
    : 0
  
  // 4. 停顿检测
  let pauseCount = 0
  for (let i = 1; i < n; i++) {
    const dt = points[i].t - points[i-1].t
    if (dt > 100) pauseCount++ // 100ms 以上算停顿
  }
  const pauseRatio = pauseCount / n
  
  // 5. 时间间隔特征
  const intervals = []
  for (let i = 1; i < n; i++) {
    intervals.push(points[i].t - points[i-1].t)
  }
  const avgInterval = intervals.reduce((a,b) => a+b, 0) / intervals.length
  const intervalVariance = intervals.reduce((s, v) => s + Math.pow(v - avgInterval, 2), 0) / intervals.length
  
  // 6. 路径特征
  let totalDist = 0
  for (let i = 1; i < n; i++) {
    const dx = points[i].x - points[i-1].x
    const dy = points[i].y - points[i-1].y
    totalDist += Math.sqrt(dx*dx + dy*dy)
  }
  const directDist = Math.sqrt(
    Math.pow(points[n-1].x - points[0].x, 2) + 
    Math.pow(points[n-1].y - points[0].y, 2)
  )
  const efficiency = directDist > 0 ? directDist / Math.max(totalDist, 1) : 0
  
  // 7. 抖动（连续点的微小变化）
  let jitter = 0
  for (let i = 2; i < n; i++) {
    const dx1 = points[i-1].x - points[i-2].x
    const dy1 = points[i-1].y - points[i-2].y
    const dx2 = points[i].x - points[i-1].x
    const dy2 = points[i].y - points[i-1].y
    jitter += Math.abs(dx2 - dx1) + Math.abs(dy2 - dy1)
  }
  jitter = jitter / n
  
  // 8. 时长和点数
  const duration = points[n-1].t - points[0].t
  const pointDensity = n / Math.max(duration, 1) * 1000 // 每秒点数
  
  // 归一化特征向量 (16维)
  return [
    Math.min(avgSpeed / 2, 1),           // 0: 平均速度
    Math.min(maxSpeed / 5, 1),           // 1: 最大速度
    Math.min(speedVariance / 2, 1),      // 2: 速度方差
    Math.min(avgAccel / 0.5, 1),         // 3: 平均加速度
    Math.min(maxAccel / 1, 1),           // 4: 最大加速度
    Math.min(avgAngle / Math.PI, 1),     // 5: 平均角度变化
    Math.min(angleVariance / 2, 1),      // 6: 角度方差
    Math.min(pauseRatio, 1),             // 7: 停顿比例
    Math.min(avgInterval / 100, 1),      // 8: 平均时间间隔
    Math.min(intervalVariance / 5000, 1),// 9: 间隔方差
    Math.min(efficiency, 1),             // 10: 路径效率
    Math.min(jitter / 10, 1),            // 11: 抖动
    Math.min(duration / 5000, 1),        // 12: 总时长
    Math.min(pointDensity / 100, 1),     // 13: 点密度
    Math.min(n / 100, 1),                // 14: 点数
    Math.min(totalDist / 2000, 1),       // 15: 总距离
  ]
}

export { extractFeatures }
