// Chinese rendering of the simulator's event messages (the backend logs in English).
// Phrase-level rules so numbers are kept and unknown text simply passes through.
const KIND = { cold: '冷启动', warm: '热启动', final: '五边' };

const RULES = [
  // spawn / scenario
  [/^spawn: /, '随机出生点：'],
  [/([\d.]+) km from runway \(bearing (-?\d+) deg\)/, '距跑道 $1 km（方位 $2°）'],
  [/\balt ([\d.]+) km/, '高度 $1 km'],
  [/\bMach ([\d.]+)/, '马赫 $1'],
  [/\bqbar ([\d.]+) kPa/, '动压 $1 kPa'],
  [/\bheading (-?\d+) deg/, '航向 $1°'],
  [/\bgamma (-?[\d.]+) deg/, '航迹角 $1°'],
  [/\balpha (-?[\d.]+) deg/, '迎角 $1°'],
  [/^approach: ([\d.]+) m AGL, ([\d.]+) m\/s/, '短五边：离地 $1 m，$2 m/s'],
  // sim
  [/^mode -> /, '控制模式 → '],
  [/^guidance -> /, '制导律 → '],
  [/^guidance error: /, '制导异常：'],
  [/^planner error: /, '规划器异常：'],
  [/state became non-finite - resetting/, '状态数值发散，已重置'],
  [/^wind ([\d.]+) m\/s from (\d+) deg/, '风 $1 m/s，来向 $2°'],
  [/turbulence ([\d.]+)/, '湍流强度 $1'],
  [/ \(auto\)/, '（自动变化）'],
  // touchdown / rollout
  [/drag chute deploy at (\d+) KGS/, '减速伞展开（地速 $1 kt）'],
  [/drag chute jettison at (\d+) KGS/, '减速伞抛离（地速 $1 kt）'],
  [/main gear touchdown: sink ([\d.]+) m\/s \(([\d.]+) ft\/s\)/, '主起落架接地：下沉率 $1 m/s（$2 ft/s）'],
  [/nose gear touchdown at (\d+) KGS/, '前起落架接地（地速 $1 kt）'],
  [/pitch (-?[\d.]+) deg/, '俯仰 $1°'],
  [/(\d+) m past RWY(\d+) threshold/, '越过 RWY$2 入口 $1 m'],
  [/wheels stop: (\d+) m from RWY(\d+) threshold/, '停稳：距 RWY$2 入口 $1 m'],
  [/([+-]?[\d.]+) m off centreline/, '偏离中线 $1 m'],
  [/off runway/i, '偏出跑道'],
  [/^tail scrape: /, '擦尾：'],
  [/^CRASH - /, '坠毁 — '],
  [/gear collapse: sink rate ([\d.]+) m\/s/, '起落架压垮：下沉率 $1 m/s'],
  [/ground impact: /, '机体撞地：'],
  [/\(sink ([\d.]+) m\/s\)/, '（下沉率 $1 m/s）'],
  [/(\d+) KGS/g, '地速 $1 kt'],
  // GNC
  [/^plan #(\d+) \((\w+)\) RWY(\d+): SCP (\d+) it, (\d+) ms, /, (m, a, k, r, it, ms) => `规划 #${a}（${KIND[k] || k}）RWY${r}：SCP ${it} 次迭代，${ms} ms，`],
  [/not converged/, '未收敛'],
  [/\bconverged\b/, '已收敛'],
  [/tf (\d+) s to ALI/, '到 A/L 接口 $1 s'],
  [/ \[geometric fallback\]/, '［几何备用基准］'],
  [/low energy: shortened approach \(level (\d+)\)/, '能量偏低：缩短进近（第 $1 级）'],
  [/LOW ENERGY: runway not reachable - flying maximum range toward the field/, '能量不足：到不了跑道，按最大航程飞向机场'],
  [/ALI not reachable \(miss: dh ([+-]?\d+) m, dV ([+-]?[\d.]+) m\/s\) - best-effort trajectory/, 'A/L 接口无法精确到达（高度差 $1 m，速度差 $2 m/s），采用尽力轨迹'],
  [/SCP failed - geometric reference/, 'SCP 未收敛，改用几何基准'],
  [/established on final approach/, '已建立五边进近'],
  [/^phase (\S+) \/ (\S*)\s+\(h (\d+) m, (\d+) KEAS\)/, '阶段 $1 / $2（高度 $3 m，$4 KEAS）'],
  [/gear down at (\d+) m/, '放下起落架（离地 $1 m）'],
  [/^replan: tracking error (\d+) m \/ dh (-?\d+) m \/ dE (-?\d+) m/, '重新规划：跟踪误差 $1 m / 高度差 $2 m / 能量高度差 $3 m'],
];

export function zhEvent(text) {
  if (typeof text !== 'string') return text;
  let s = text;
  for (const [re, rep] of RULES) s = s.replace(re, rep);
  // punctuation once something was translated
  if (s !== text) s = s.replace(/, /g, '，').replace(/; /g, '；').replace(/ - /g, ' — ');
  return s;
}
