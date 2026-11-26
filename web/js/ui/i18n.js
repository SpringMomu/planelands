// Chinese rendering of the simulator's event messages (the backend logs in English).
// Phrase-level rules so numbers are kept and unknown text simply passes through.
const KIND = { cold: '冷启动', warm: '热启动', final: '五边', switch: '改选进场' };

const RULES = [
  // envelope warnings (sim/envelope.py)
  [/^OVERSPEED: (\d+) KEAS above VMO (\d+) KEAS, qbar ([\d.]+) kPa/, '超速：当量空速 $1 kt，超过 VMO $2 kt（动压 $3 kPa）'],
  [/^OVERSPEED: (\d+) KEAS above VLE (\d+) KEAS \(gear not up\), qbar ([\d.]+) kPa/, '超速：当量空速 $1 kt，起落架未收起，超过 VLE $2 kt（动压 $3 kPa）'],
  [/^overspeed cleared: (\d+) KEAS/, '超速解除：当量空速 $1 kt'],
  [/^STALL: (both wings|left wing|right wing) past maximum lift - alpha (-?[\d.]+) deg, (\d+) KEAS, nz (-?[\d.]+) g, lift lost L (\d+)% R (\d+)%/,
    (m, w, a, k, n, l, r) => `失速：${{ 'both wings': '两侧机翼', 'left wing': '左翼', 'right wing': '右翼' }[w]}越过最大升力点（迎角 ${a}°，当量空速 ${k} kt，过载 ${n} g，分离损失升力 左 ${l}% 右 ${r}%）`],
  [/^stall recovered: alpha (-?[\d.]+) deg, (\d+) KEAS/, '失速改出：迎角 $1°，当量空速 $2 kt'],
  // in-flight adjustment
  [/^adjust: only possible in free flight/, '飞行中调整：只能在空中使用'],
  [/^adjust: altitude raised to (\d+) m \(150 m above the terrain\)/, '飞行中调整：高度太低，已抬到 $1 m（离地 150 m）'],
  [/^adjust: altitude limited to 40 km/, '飞行中调整：高度限制在 40 km'],
  [/^adjust: speed clamped to (\d+) m\/s/, '飞行中调整：速度超限，已限制为 $1 m/s'],
  [/^adjust: alt ([\d.]+) km, Mach ([\d.]+), (\d+) KEAS, heading (\d+) deg, gamma (-?[\d.]+) deg - replanning$/,
    '飞行中调整：高度 $1 km，马赫 $2，当量空速 $3 kt，航向 $4°，航迹角 $5°，重新规划'],
  // spawn / scenario
  [/^spawn: /, '随机出生点：'],
  [/^custom spawn: position clamped to the map \((\d+) km\)/, '自定义起点：位置超出地图，已限制在 ±$1 km'],
  [/^custom spawn: altitude raised to (\d+) m \(150 m above the terrain\)/, '自定义起点：高度太低，已抬到 $1 m（离地 150 m）'],
  [/^custom spawn: speed clamped to (\d+) m\/s/, '自定义起点：速度超限，已限制为 $1 m/s'],
  [/^custom spawn: too slow for 1 g at this altitude: alpha limited to ([\d.]+) deg \(will sink \/ accelerate\)/, '自定义起点：这个高度速度太低，迎角限制在 $1°（会掉高度加速）'],
  [/^custom spawn: /, '自定义起点：'],
  [/^custom spawn reachable: RWY(\d+) (right|left) HAC, energy band ([\d.]+)/, (m, r, sd, b) => `自定义起点可达：RWY${r} ${sd === 'right' ? '右' : '左'}侧 HAC，能量带位置 ${b}`],
  [/^custom spawn HIGH ENERGY: runway (\d+) only reachable on a stretched approach into a strong headwind \((\d+) m\/s average\) - no margin/, '自定义起点能量过高：只有顶着强逆风（平均 $2 m/s）拉长航迹才到得了 RWY$1，没有余量'],
  [/^HIGH ENERGY: runway (\d+) only reachable on a stretched approach into a strong headwind \((\d+) m\/s average\) - no margin/, '能量过高：只有顶着强逆风（平均 $2 m/s）拉长航迹才到得了 RWY$1，没有余量'],
  [/high energy: stretched approach \(HAC radius ([\d.]+) km, (\d+) extra loops?\)/, '能量偏高：拉长航迹（HAC 半径 $1 km，多绕 $2 圈）'],
  [/^custom spawn LOW ENERGY: runway (\d+) needs ([\d.]+) km energy height, ([-\d.]+) km available/, '自定义起点能量不足：到 RWY$1 需 $2 km 能量高度，只有 $3 km'],
  [/^custom spawn HIGH ENERGY: runway (\d+) can dissipate at most ([\d.]+) km energy height, ([-\d.]+) km available - too close for this altitude \/ speed/, '自定义起点能量过高（离跑道太近）：到 RWY$1 最多耗散 $2 km 能量高度，现有 $3 km'],
  [/^HIGH ENERGY: runway (\d+) can dissipate at most ([\d.]+) km energy height, ([-\d.]+) km available - too close for this altitude \/ speed/, '能量过高（离跑道太近）：到 RWY$1 最多耗散 $2 km 能量高度，现有 $3 km'],
  [/^LOW ENERGY: runway (\d+) needs ([\d.]+) km energy height, ([-\d.]+) km available/, '能量不足：到 RWY$1 需 $2 km 能量高度，只有 $3 km'],
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
  [/^(left|right) wing tip scrape at (\d+) KGS \(sink ([\d.]+) m\/s\)/, (m, sd, gs, v) => `${sd === 'left' ? '左' : '右'}翼尖擦地，火花四溅（地速 ${gs} kt，下沉率 ${v} m/s）`],
  [/^tail scrape: (tail|nozzle|belly) at (\d+) KGS \(sink ([\d.]+) m\/s\)/, (m, r, gs, v) => `擦尾：${{ tail: '体襟翼后缘', nozzle: '发动机喷管', belly: '机腹' }[r]}触地摩擦出火花（地速 ${gs} kt，下沉率 ${v} m/s）`],
  [/^tail scrape: /, '擦尾：'],
  [/^hydroplaning: main gear at (\d+) KGS on ([\d.]+) mm of water - tyre friction ([\d.]+) \(dry ([\d.]+)\), braking and steering lost/,
    '滑水：主起落架压在 $2 mm 积水上（地速 $1 kt），轮胎摩擦系数 $3（干跑道 $4），刹车和转向失效'],
  [/^hydroplaning over at (\d+) KGS - tyre friction ([\d.]+)/, '滑水结束（地速 $1 kt），轮胎摩擦系数恢复到 $2'],
  [/^structural damage: (left wing tip|right wing tip|body flap|engine nozzle|belly) crushed \/ worn (\d+) cm/, (m, w, d) => `结构损伤：${{ 'left wing tip': '左翼尖', 'right wing tip': '右翼尖', 'body flap': '体襟翼', 'engine nozzle': '发动机喷管', belly: '机腹' }[w]}压溃 / 磨损 ${d} cm`],
  [/^CRASH - /, '坠毁 — '],
  [/(left|right) wing destroyed: tip crushed \/ ground away ([\d.]+) m/, (m, sd, d) => `${sd === 'left' ? '左' : '右'}翼损毁：翼尖压溃 / 磨掉 ${d} m`],
  [/rolled over onto the wing \(bank (-?\d+) deg\)/, '侧翻压上机翼（滚转 $1°）'],
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
  [/HIGH ENERGY: too close for this altitude \/ speed - best-effort energy dissipation/, '能量过高：这个高度 / 速度离跑道太近，尽力耗散能量'],
  [/ALI not reachable \(miss: dh ([+-]?\d+) m, dV ([+-]?[\d.]+) m\/s\) - best-effort trajectory/, 'A/L 接口无法精确到达（高度差 $1 m，速度差 $2 m/s），采用尽力轨迹'],
  [/SCP failed - geometric reference/, 'SCP 未收敛，改用几何基准'],
  [/established on final approach/, '已建立五边进近'],
  [/^phase (\S+) \/ (\S*)\s+\(h (\d+) m, (\d+) KEAS\)/, '阶段 $1 / $2（高度 $3 m，$4 KEAS）'],
  [/gear down at (\d+) m/, '放下起落架（离地 $1 m）'],
  [/^HAC switch: RWY(\d+) (right|left) HAC -> RWY(\d+) (right|left) HAC \(/, (m, r0, s0, r1, s1) => `改选进场：RWY${r0} ${s0 === 'right' ? '右' : '左'}侧 HAC → RWY${r1} ${s1 === 'right' ? '右' : '左'}侧 HAC（`],
  [/current plan out of the energy band/, '原方案已超出能量带'],
  [/better energy margin \(score ([\d.]+) -> ([\d.]+)\)/, '能量余量更好（评分 $1 → $2）'],
  [/energy band ([\d.]+) -> ([\d.]+)/, '能量带位置 $1 → $2'],
  [/wind (\d+) m\/s from (\d+)\)/, '风 $1 m/s，来向 $2°）'],
  [/^replan: wind update (\d+) m\/s from (\d+) \(changed ([\d.]+) m\/s since the last plan\)/, '重新规划：风况更新 $1 m/s，来向 $2°（较上次规划变化 $3 m/s）'],
  [/^approach re-selection: (\d+) (right|left) HAC rejected \(SCP did not reach the ALI on the new approach\)/, (m, r, sd) => `改选进场被否决：RWY${r} ${sd === 'right' ? '右' : '左'}侧 HAC 的 SCP 到不了 A/L 接口`],
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
