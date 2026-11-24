// Panel contents of the orbiter forward flight deck (post-MEDS configuration, as flown 2000–2011).
// Positions are panel-local metres (origin top-left, u right, v down).  The arrangement follows the
// real panels: F2/F3/F4 glareshield (flight-control mode pushbuttons, master alarm, drag chute, HUD),
// F6/F8 commander / pilot instrument panels (two MDUs each, ADI/HSI select switches, landing gear
// arm/down and talkbacks, abort), F7 centre (caution & warning matrix, CRT1/CRT3/CRT2, MFD1/MFD2,
// event timer), C2/C3 centre console (keyboards, major-function switches, DAP pushbuttons, OMS),
// L1/L2 and R1/R2 side consoles and the O1–O9 overhead panels.
//
// Item types: id txt box tg kn cb pb lens mdu win meter stripe placard velcro cue kb sbtc
//   pb / lens:  { leg: legend key, ch: light channel }   tg: { pos: [...], st: lever position -1/0/+1 }

const tgRow = (x0, y, dx, list, extra = {}) => list.map((L, i) => ({ t: 'tg', x: x0 + i * dx, y, lab: L[0], pos: L[1] || ['ON', 'OFF'], st: L[2] ?? 1, ...extra }));
const cbGrid = (x0, y0, dx, dy, cols, labels, amps = [5, 3, 7.5, 10, 5, 3]) => labels.map((l, i) => ({ t: 'cb', x: x0 + (i % cols) * dx, y: y0 + Math.floor(i / cols) * dy, lab: l, amp: amps[i % amps.length], out: (i * 7919) % 23 === 3 }));

// ------------------------------------------------------------------ legends (lighted pushbuttons, C&W)
export const LEGENDS = [];
const leg = (key, o) => { LEGENDS.push({ key, ...o }); return key; };
leg('ma', { lines: ['MASTER', 'ALARM'], color: 'red' });
leg('pitchAC', { split: ['AUTO', 'CSS'], color: 'white' });
leg('rollAC', { split: ['AUTO', 'CSS'], color: 'white' });
leg('sbAM', { split: ['AUTO', 'MAN'], color: 'white' });
leg('bfAM', { split: ['AUTO', 'MAN'], color: 'white' });
leg('arm', { lines: ['ARM'], color: 'amber' });
leg('dn', { lines: ['DN'], color: 'green' });
leg('dcArm', { lines: ['ARM'], color: 'amber' });
leg('dcDpy', { lines: ['DPY'], color: 'green' });
leg('dcJett', { lines: ['JETT'], color: 'amber' });
leg('abort', { lines: ['ABORT'], color: 'red' });
leg('smoke', { lines: ['SMOKE', 'DETECT'], color: 'red' });
leg('fsArm', { lines: ['ARM'], color: 'amber' });
leg('fsDis', { lines: ['DISCH'], color: 'red' });
leg('meL', { lines: ['L'], color: 'amber', aspect: 1 });
leg('meC', { lines: ['C'], color: 'amber', aspect: 1 });
leg('meR', { lines: ['R'], color: 'amber', aspect: 1 });
leg('hudOn', { lines: ['HUD'], color: 'green' });
['A', 'B', 'AUTO', 'MAN', 'INRTL', 'LVLH', 'FREE', 'PCT', 'LOW Z', 'HIGH', 'NORM', 'PULSE', 'DISC RATE', 'ACCEL', 'VERN', 'PRI', 'ALT', 'TRANS X', 'TRANS Y', 'TRANS Z', 'ROT ROLL', 'ROT PITCH', 'ROT YAW', 'BLOCK']
  .forEach((s, i) => leg(`dap${i}`, { lines: s.split(' '), color: 'white' }));
// caution & warning matrix (F7, above CRT3): 40 lights, 5 columns x 8 rows, red warnings / yellow cautions
export const CW = [
  ['O2 PRESS', 'H2 PRESS', 'FUEL CELL REAC', 'FUEL CELL STACK TEMP', 'FUEL CELL PUMP'],
  ['CABIN ATM', 'O2 HEATER TEMP', 'MAIN BUS UNDERVOLT', 'AC VOLTAGE', 'AC OVERLOAD'],
  ['FREON LOOP', 'AV BAY/ CABIN AIR', 'IMU', 'FWD RCS', 'RCS JET'],
  ['H2O LOOP', 'RGA/ ACCEL', 'AIR DATA', 'LEFT RHC', 'RIGHT/ AFT RHC'],
  ['H2 HEATER TEMP', 'LEFT OMS', 'RIGHT OMS', 'PAYLOAD WARNING', 'GPC'],
  ['FCS SATURATION', 'OMS KIT', 'OMS TVC', 'PAYLOAD CAUTION', 'PRIMARY C/W'],
  ['FCS CHANNEL', 'MPS', 'BACKUP C/W ALARM', 'APU TEMP', 'APU OVERSPEED'],
  ['APU UNDERSPEED', 'HYD PRESS', 'LEFT RCS', 'RIGHT RCS', 'LEFT AFT RHC'],
];
CW.forEach((row, r) => row.forEach((s, c) => leg(`cw${r}_${c}`, { lines: s.split(' '), color: r < 2 || (r === 6 && c < 2) ? 'red' : 'amber', aspect: 1.95, fsMax: 0.30 })));

// MDU bezel (MEDS): ~9 x 8.5 in, 6.7 in square LCD, six edge keys under the screen
export const MDU_W = 0.229, MDU_H = 0.215;

// ------------------------------------------------------------------ F6 / F8 (commander / pilot)
function fwdSide(side) {
  const L = side === 'L';
  const id = L ? 'F6' : 'F8', w = 0.515, h = 0.50;
  const mdu = L ? ['CDR1', 'CDR2'] : ['PLT1', 'PLT2'];
  const m0 = L ? 0.0265 : 0.0175, m1 = m0 + MDU_W + 0.013;
  const items = [
    { t: 'id', s: id },
    { t: 'box', x: 0.012, y: 0.032, w: 0.205, h: 0.070, title: 'ADI' },
    ...tgRow(0.045, 0.074, 0.068, [['ATTITUDE', ['INRTL', 'LVLH', 'REF'], 0], ['ERROR', ['HIGH', 'MED', 'LOW'], 0], ['RATE', ['HIGH', 'MED', 'LOW'], 0]]),
    { t: 'box', x: 0.228, y: 0.032, w: 0.205, h: 0.070, title: 'HSI SELECT' },
    ...tgRow(0.262, 0.074, 0.068, [['MODE', ['ENTRY', 'TAEM', 'APPROACH'], 0], ['SOURCE', ['TACAN', 'NAV', 'MLS'], 0], ['', ['1', '2', '3'], 0]]),
    { t: 'kn', x: 0.474, y: 0.060, r: 0.0085, lab: 'AIR DATA', pos: ['LEFT', 'NAV', 'RIGHT'] },
    { t: 'mdu', x: m0, y: 0.112, w: MDU_W, h: MDU_H, id: mdu[0] },
    { t: 'mdu', x: m1, y: 0.112, w: MDU_W, h: MDU_H, id: mdu[1] },
    // landing gear (guarded ARM yellow / DN green, NOSE / LEFT / RIGHT talkbacks)
    { t: 'box', x: 0.012, y: 0.352, w: 0.235, h: 0.135, title: 'LANDING GEAR' },
    { t: 'txt', s: 'ARM', x: 0.048, y: 0.378, size: 3.8 }, { t: 'txt', s: 'DN', x: 0.094, y: 0.378, size: 3.8 },
    { t: 'pb', x: 0.048, y: 0.400, leg: 'arm', ch: 'gearArm', w: 0.026, h: 0.026, guard: true },
    { t: 'pb', x: 0.094, y: 0.400, leg: 'dn', ch: 'gearDn', w: 0.026, h: 0.026, guard: true },
    { t: 'win', x: 0.128, y: 0.390, w: 0.030, h: 0.018, dyn: 'tbN', lab: 'NOSE' },
    { t: 'win', x: 0.166, y: 0.390, w: 0.030, h: 0.018, dyn: 'tbL', lab: 'LEFT' },
    { t: 'win', x: 0.204, y: 0.390, w: 0.030, h: 0.018, dyn: 'tbR', lab: 'RIGHT' },
    { t: 'tg', x: 0.060, y: 0.460, lab: 'HUD DATA BUS', pos: ['1', '2'], st: -1, ls: 3.4 },
    { t: 'tg', x: 0.180, y: 0.460, lab: 'RDR ALTM', pos: ['1', '2'], st: -1, ls: 3.4 },
  ];
  if (L) {
    items.push(
      { t: 'stripe', x: 0.258, y: 0.352, w: 0.130, h: 0.135 },
      { t: 'txt', s: 'ABORT MODE', x: 0.300, y: 0.375, size: 3.8 },
      { t: 'kn', x: 0.300, y: 0.420, r: 0.0105, pos: ['OFF', 'RTLS', 'TAL', 'ATO'], a0: -70, a1: 70 },
      { t: 'pb', x: 0.358, y: 0.414, leg: 'abort', ch: 'abort', w: 0.026, h: 0.026 },
      { t: 'box', x: 0.398, y: 0.352, w: 0.105, h: 0.135, title: 'MAIN ENGINE' },
      ...tgRow(0.425, 0.425, 0.050, [['LIMIT', ['AUTO', 'INHIBIT', 'ENABLE'], 0], ['SHUT DN', ['', ''], 1]]),
    );
  } else {
    items.push(
      { t: 'box', x: 0.258, y: 0.352, w: 0.130, h: 0.135, title: 'MLS / TACAN' },
      ...tgRow(0.290, 0.425, 0.064, [['MLS', ['ON', 'OFF'], -1], ['TACAN', ['GPC', 'TEST'], -1]]),
      { t: 'box', x: 0.398, y: 0.352, w: 0.105, h: 0.135, title: 'CLOCK' },
      { t: 'tg', x: 0.450, y: 0.425, lab: 'ANNUN', pos: ['TEST', 'NORM'], st: 1 },
    );
  }
  return { id, w, h, items };
}

// ------------------------------------------------------------------ F7 (centre)
function f7() {
  const w = 0.730, h = 0.50, items = [{ t: 'id', s: 'F7', x: 0.258, y: 0.016 }];
  const c1 = 0.0155, c2 = c1 + MDU_W + 0.006, c3 = c2 + MDU_W + 0.006;
  items.push({ t: 'mdu', x: c1, y: 0.006, w: MDU_W, h: MDU_H, id: 'CRT1' });
  items.push({ t: 'mdu', x: c1, y: 0.227, w: MDU_W, h: MDU_H, id: 'MFD1' });
  items.push({ t: 'mdu', x: c2, y: 0.115, w: MDU_W, h: MDU_H, id: 'CRT3' });
  items.push({ t: 'mdu', x: c3, y: 0.006, w: MDU_W, h: MDU_H, id: 'CRT2' });
  items.push({ t: 'mdu', x: c3, y: 0.227, w: MDU_W, h: MDU_H, id: 'MFD2' });
  // caution & warning matrix above CRT3
  const cw = 0.0232, ch = 0.0118, cx0 = c2 + (MDU_W - cw * 5) / 2, cy0 = 0.012;
  items.push({ t: 'box', x: cx0 - 0.004, y: cy0 - 0.004, w: cw * 5 + 0.008, h: ch * 8 + 0.008 });
  CW.forEach((row, r) => row.forEach((s, c) => items.push({ t: 'lens', x: cx0 + c * cw + cw / 2, y: cy0 + r * ch + ch / 2, w: cw - 0.0022, h: ch - 0.0022, leg: `cw${r}_${c}`, ch: `cw${r}_${c}` })));
  items.push({ t: 'txt', s: 'C/W', x: c2 + 0.022, y: 0.060, size: 4.2 }, { t: 'txt', s: 'MASTER', x: c2 + MDU_W - 0.024, y: 0.056, size: 3.0 }, { t: 'txt', s: 'ALARM', x: c2 + MDU_W - 0.024, y: 0.062, size: 3.0 });
  // under CRT3: main engine status, event timer, flight controller power
  items.push({ t: 'box', x: c2 + 0.006, y: 0.346, w: 0.090, h: 0.060, title: 'MAIN ENGINE STATUS' });
  ['meL', 'meC', 'meR'].forEach((k, i) => items.push({ t: 'pb', x: c2 + 0.022 + i * 0.029, y: 0.383, w: 0.020, h: 0.020, leg: k, ch: 'meStatus', lens: true }));
  items.push({ t: 'win', x: c2 + 0.108, y: 0.358, w: 0.112, h: 0.030, dyn: 'evt', lab: 'EVENT TIMER' });
  items.push(...tgRow(c2 + 0.125, 0.425, 0.040, [['', ['START', 'STOP'], -1], ['', ['UP', 'DOWN'], 1], ['', ['SET', 'RESET'], 0]], { ls: 3.2 }));
  items.push({ t: 'box', x: c2 + 0.006, y: 0.416, w: 0.090, h: 0.075, title: 'FLT CNTLR PWR' });
  items.push(...tgRow(c2 + 0.032, 0.462, 0.040, [['CDR', ['ON', 'OFF'], -1], ['PLT', ['ON', 'OFF'], -1]]));
  items.push({ t: 'placard', x: c1 + 0.020, y: 0.455, w: 0.190, h: 0.028, s: 'CAUTION: GEAR DN NOT ABOVE 300 KEAS', size: 3.0 });
  items.push({ t: 'placard', x: c3 + 0.020, y: 0.455, w: 0.190, h: 0.028, s: 'DO NOT JETT CHUTE ABOVE 230 KEAS', size: 3.0 });
  return { id: 'F7', w, h, items };
}

// ------------------------------------------------------------------ F2 / F3 / F4 (glareshield aft face)
// The HUD sits on the glareshield in each pilot's line of sight; the drag chute ARM / DPY / JETT
// pushbuttons are on either side of it and the HUD MODE switch directly below it.
function glareSide(side) {
  const L = side === 'L', w = 0.600, h = 0.084;
  const X = (u) => (L ? u : w - u);                 // mirror positions for the pilot
  const hud = L ? 0.400 : 0.200;
  const items = [
    { t: 'id', s: L ? 'F2' : 'F4', x: L ? 0.012 : w - 0.030 },
    { t: 'pb', x: X(0.040), y: 0.050, w: 0.030, h: 0.030, leg: 'ma', ch: 'ma' },
    { t: 'box', x: L ? 0.072 : w - 0.072 - 0.205, y: 0.016, w: 0.205, h: 0.064, title: 'FLIGHT CONTROL' },
    { t: 'pb', x: X(0.097), y: 0.054, w: 0.024, h: 0.024, leg: 'pitchAC', ch: 'pitchAC', lab: 'PITCH' },
    { t: 'pb', x: X(0.147), y: 0.054, w: 0.024, h: 0.024, leg: 'rollAC', ch: 'rollAC', lab: 'ROLL/YAW' },
    { t: 'pb', x: X(0.200), y: 0.054, w: 0.024, h: 0.024, leg: 'sbAM', ch: 'sbAM', lab: 'SPD BK/THROT' },
    { t: 'pb', x: X(0.252), y: 0.054, w: 0.024, h: 0.024, leg: 'bfAM', ch: 'bfAM', lab: 'BODY FLAP' },
    { t: 'box', x: hud - 0.110, y: 0.016, w: 0.220, h: 0.064, title: 'DRAG CHUTE        HUD        DRAG CHUTE', ts: 3.6 },
    { t: 'pb', x: hud - 0.085, y: 0.054, w: 0.024, h: 0.024, leg: 'dcArm', ch: 'dcArm', lab: 'ARM', guard: true },
    { t: 'pb', x: hud - 0.045, y: 0.054, w: 0.024, h: 0.024, leg: 'dcDpy', ch: 'dcDpy', lab: 'DPY', guard: true },
    { t: 'tg', x: hud, y: 0.058, lab: 'MODE', pos: ['DCLT', 'NORM', 'TEST'], st: 0, ls: 3.4 },
    { t: 'pb', x: hud + 0.060, y: 0.054, w: 0.024, h: 0.024, leg: 'dcJett', ch: 'dcJett', lab: 'JETT', guard: true },
    { t: 'kn', x: hud + 0.095, y: 0.050, r: 0.0065, lab: 'BRT' },
  ];
  const fx = L ? 0.525 : 0.010;
  items.push({ t: 'box', x: fx, y: 0.016, w: 0.065, h: 0.064, title: 'BFC' }, { t: 'pb', x: fx + 0.032, y: 0.052, w: 0.020, h: 0.020, leg: 'bfc', ch: 'off', lens: true });
  return { id: L ? 'F2' : 'F4', w, h, items };
}
function f3() {
  const w = 0.690, h = 0.084;
  return {
    id: 'F3', w, h, items: [
      { t: 'id', s: 'F3', x: 0.330, y: 0.016 },
      { t: 'box', x: 0.020, y: 0.016, w: 0.120, h: 0.062, title: 'HUD POWER' },
      { t: 'tg', x: 0.080, y: 0.056, lab: 'CDR', pos: ['ON', 'OFF'], st: -1, ls: 3.4 },
      { t: 'box', x: 0.170, y: 0.016, w: 0.150, h: 0.062, title: 'TRIM' },
      ...tgRow(0.205, 0.056, 0.080, [['PANEL', ['ENABLE', 'INHIBIT'], -1], ['RHC/PANEL', ['ENABLE', 'INHIBIT'], -1]], { ls: 3.2 }),
      { t: 'box', x: 0.370, y: 0.016, w: 0.150, h: 0.062, title: 'ANNUNCIATOR' },
      ...tgRow(0.405, 0.056, 0.080, [['LAMP TEST', ['LEFT', 'RIGHT'], 0], ['BUS SEL', ['MN A', 'MN B'], -1]], { ls: 3.2 }),
      { t: 'box', x: 0.550, y: 0.016, w: 0.120, h: 0.062, title: 'HUD POWER' },
      { t: 'tg', x: 0.610, y: 0.056, lab: 'PLT', pos: ['ON', 'OFF'], st: -1, ls: 3.4 },
    ],
  };
}
leg('bfc', { lines: ['BFC'], color: 'amber' });

// ------------------------------------------------------------------ C2 / C3 (centre console)
function c2() {
  const w = 0.44, h = 0.232, items = [{ t: 'id', s: 'C2' }];
  items.push({ t: 'kb', x: 0.022, y: 0.040, side: 'L' }, { t: 'kb', x: 0.262, y: 0.040, side: 'R' });
  items.push({ t: 'txt', s: 'CDR KEYBOARD', x: 0.100, y: 0.030, size: 3.6 }, { t: 'txt', s: 'PLT KEYBOARD', x: 0.340, y: 0.030, size: 3.6 });
  items.push({ t: 'box', x: 0.186, y: 0.040, w: 0.068, h: 0.180, title: 'IDP/CRT', ts: 3.6 });
  ['1', '2', '3', '4'].forEach((s, i) => items.push({ t: 'tg', x: 0.205 + (i % 2) * 0.030, y: 0.080 + Math.floor(i / 2) * 0.060, lab: `MAJ FUNC ${s}`, ls: 2.6, lw: 0.028, pos: ['GNC', 'PL'], st: -1 }));
  items.push({ t: 'kn', x: 0.220, y: 0.200, r: 0.007, pos: ['1', '2', '3'] });
  return { id: 'C2', w, h, items };
}
function c3() {
  const w = 0.44, h = 0.52, items = [{ t: 'id', s: 'C3' }];
  items.push({ t: 'box', x: 0.012, y: 0.030, w: 0.300, h: 0.165, title: 'ORBITAL DAP' });
  for (let i = 0; i < 24; i++) items.push({ t: 'pb', x: 0.040 + (i % 6) * 0.049, y: 0.064 + Math.floor(i / 6) * 0.034, w: 0.024, h: 0.024, leg: `dap${i}`, ch: 'dap', lab: '' });
  items.push({ t: 'txt', s: 'SPD BK / THROT', x: 0.376, y: 0.030, size: 3.6 });
  items.push({ t: 'sbtc', x: 0.376, y: 0.118, len: 0.130 });
  items.push({ t: 'box', x: 0.012, y: 0.200, w: 0.200, h: 0.110, title: 'OMS ENG' });
  items.push(...tgRow(0.060, 0.262, 0.100, [['LEFT', ['ARM/PRESS', 'ARM', 'OFF'], 1], ['RIGHT', ['ARM/PRESS', 'ARM', 'OFF'], 1]]));
  items.push({ t: 'box', x: 0.228, y: 0.200, w: 0.200, h: 0.110, title: 'AIR DATA PROBE' });
  items.push(...tgRow(0.262, 0.262, 0.042, [['LEFT', ['ENABLE', 'INHIBIT', 'DEPLOY'], 1], ['', ['STOW', 'DEPLOY'], 1], ['RIGHT', ['ENABLE', 'INHIBIT', 'DEPLOY'], 1], ['', ['STOW', 'DEPLOY'], 1]], { ls: 3.2 }));
  items.push({ t: 'box', x: 0.012, y: 0.330, w: 0.416, h: 0.090, title: 'BODY FLAP / SPEED BRAKE / ENTRY' });
  items.push(...tgRow(0.050, 0.380, 0.060, [['BODY FLAP', ['UP', '', 'DOWN'], 0], ['ENTRY MODE', ['AUTO', 'LO GAIN', 'NO Y JET'], -1], ['ADI', ['NORM', 'BKUP'], -1], ['BFS', ['ENGAGE', ''], 0], ['ANTI SKID', ['ON', 'OFF'], -1], ['NW STRG', ['1', 'OFF', '2'], -1], ['BRAKES', ['ON', 'OFF'], -1]], { ls: 3.2 }));
  items.push({ t: 'cue', x: 0.300, y: 0.440, w: 0.120, h: 0.064, title: 'ENTRY NO-GO' });
  items.push({ t: 'velcro', x: 0.030, y: 0.448, w: 0.090, h: 0.040 });
  return { id: 'C3', w, h, items };
}

// ------------------------------------------------------------------ side panels
function l1() {
  const w = 0.50, h = 0.30, items = [{ t: 'id', s: 'L1' }];
  items.push({ t: 'box', x: 0.012, y: 0.028, w: 0.230, h: 0.120, title: 'FIRE SUPPRESSION' });
  ['AV BAY 1', 'AV BAY 2', 'AV BAY 3'].forEach((s, i) => {
    items.push({ t: 'txt', s, x: 0.050 + i * 0.075, y: 0.050, size: 3.4 });
    items.push({ t: 'pb', x: 0.035 + i * 0.075, y: 0.080, w: 0.020, h: 0.020, leg: 'fsArm', ch: 'off', guard: true });
    items.push({ t: 'pb', x: 0.065 + i * 0.075, y: 0.080, w: 0.020, h: 0.020, leg: 'fsDis', ch: 'off', guard: true });
  });
  items.push({ t: 'pb', x: 0.050, y: 0.126, w: 0.026, h: 0.018, leg: 'smoke', ch: 'off', lens: true });
  items.push({ t: 'box', x: 0.255, y: 0.028, w: 0.232, h: 0.120, title: 'CABIN' });
  items.push(...tgRow(0.280, 0.098, 0.046, [['CABIN FAN', ['A', 'OFF', 'B'], -1], ['HUMIDITY SEP', ['A', 'OFF', 'B'], -1], ['PRESS CNTL', ['1', 'OFF', '2'], -1], ['AV BAY FAN', ['A', 'OFF', 'B'], 1], ['H2O PUMP', ['GPC', 'OFF', 'ON'], -1]], { ls: 3.2 }));
  items.push({ t: 'box', x: 0.012, y: 0.165, w: 0.475, h: 0.120, title: 'ATMOSPHERE / H2O LOOP / FREON' });
  items.push(...tgRow(0.040, 0.235, 0.045, [['O2/N2 CNTL', ['OPEN', 'CLOSE'], -1], ['N2 SUPPLY', ['OPEN', 'CLOSE'], -1], ['H2O LOOP 1', ['ON', 'OFF'], -1], ['H2O LOOP 2', ['ON', 'OFF'], 1], ['FREON 1', ['ON', 'OFF'], -1], ['FREON 2', ['ON', 'OFF'], -1], ['FLOW', ['NORM', 'BYP'], -1], ['TEMP', ['A', 'B'], -1], ['BYPASS', ['AUTO', 'MAN'], -1], ['ECU', ['HI', 'LO'], 1]], { ls: 3.0 }));
  return { id: 'L1', w, h, items };
}
function l2() {
  const w = 0.36, h = 0.46, items = [{ t: 'id', s: 'L2' }];
  items.push({ t: 'box', x: 0.016, y: 0.030, w: 0.328, h: 0.110, title: 'NOSE WHEEL STEERING / ANTI SKID' });
  items.push(...tgRow(0.050, 0.098, 0.062, [['NWS', ['1', 'OFF', '2'], -1], ['ANTI SKID', ['ON', 'OFF'], -1], ['BRAKES MN A', ['ON', 'OFF'], -1], ['MN B', ['ON', 'OFF'], -1], ['MN C', ['ON', 'OFF'], -1]], { ls: 3.2 }));
  items.push({ t: 'sbtc', x: 0.210, y: 0.270 });
  items.push({ t: 'txt', s: 'SPD BK / THROT', x: 0.210, y: 0.170, size: 4.4 });
  items.push({ t: 'txt', s: 'OPEN', x: 0.210, y: 0.190, size: 3.4 }, { t: 'txt', s: 'CLOSE', x: 0.210, y: 0.375, size: 3.4 });
  for (let i = 0; i <= 10; i++) items.push({ t: 'txt', s: i % 2 ? '' : '—', x: 0.165, y: 0.200 + i * 0.016, size: 3 });
  items.push({ t: 'box', x: 0.016, y: 0.170, w: 0.110, h: 0.200, title: 'BUS' });
  items.push(...cbGrid(0.040, 0.215, 0.032, 0.040, 3, ['RHC', 'THC', 'SBTC', 'RPTA', 'BRK', 'NWS', 'HUD', 'ADI', 'HSI', 'AMI', 'AVVI', 'ACCEL']));
  items.push({ t: 'cue', x: 0.250, y: 0.390, w: 0.095, h: 0.060, title: 'LANDING' });
  return { id: 'L2', w, h, items };
}
function r1() {
  const w = 0.50, h = 0.30, items = [{ t: 'id', s: 'R1' }];
  items.push({ t: 'box', x: 0.012, y: 0.028, w: 0.475, h: 0.120, title: 'POWER DISTRIBUTION' });
  items.push(...tgRow(0.040, 0.098, 0.045, [['FC 1', ['CNCT', 'OFF'], -1], ['FC 2', ['CNCT', 'OFF'], -1], ['FC 3', ['CNCT', 'OFF'], -1], ['MN BUS A', ['ON', 'OFF'], -1], ['MN BUS B', ['ON', 'OFF'], -1], ['MN BUS C', ['ON', 'OFF'], -1], ['ESS 1BC', ['ON', 'OFF'], -1], ['ESS 2CA', ['ON', 'OFF'], -1], ['ESS 3AB', ['ON', 'OFF'], -1], ['AC 1', ['ON', 'OFF'], -1]], { ls: 3.0 }));
  ['tb1', 'tb2', 'tb3'].forEach((k, i) => items.push({ t: 'win', x: 0.030 + i * 0.045, y: 0.050, w: 0.020, h: 0.012 }));
  items.push({ t: 'box', x: 0.012, y: 0.165, w: 0.475, h: 0.120, title: 'FUEL CELL / CRYO' });
  items.push(...tgRow(0.040, 0.235, 0.045, [['REAC 1', ['OPEN', 'CLOSE'], -1], ['REAC 2', ['OPEN', 'CLOSE'], -1], ['REAC 3', ['OPEN', 'CLOSE'], -1], ['PURGE', ['GPC', 'OFF'], 1], ['O2 HTR 1', ['AUTO', 'OFF'], -1], ['O2 HTR 2', ['AUTO', 'OFF'], -1], ['H2 HTR 1', ['AUTO', 'OFF'], -1], ['H2 HTR 2', ['AUTO', 'OFF'], -1], ['MANF 1', ['OPEN', 'CLOSE'], -1], ['MANF 2', ['OPEN', 'CLOSE'], -1]], { ls: 3.0 }));
  return { id: 'R1', w, h, items };
}
function r2() {
  const w = 0.36, h = 0.46, items = [{ t: 'id', s: 'R2' }];
  items.push({ t: 'box', x: 0.016, y: 0.030, w: 0.328, h: 0.170, title: 'APU / HYD' });
  ['1', '2', '3'].forEach((s, i) => {
    items.push({ t: 'txt', s: `APU ${s}`, x: 0.070 + i * 0.110, y: 0.052, size: 3.8 });
    items.push({ t: 'tg', x: 0.050 + i * 0.110, y: 0.092, lab: 'CONTROL', pos: ['START/RUN', 'OFF'], st: -1, ls: 3.0, guard: true });
    items.push({ t: 'tg', x: 0.090 + i * 0.110, y: 0.092, lab: 'OPER', pos: ['ENABLE', 'OFF'], st: -1, ls: 3.0 });
    items.push({ t: 'tg', x: 0.050 + i * 0.110, y: 0.160, lab: 'HYD PUMP', pos: ['NORM', 'LOW'], st: -1, ls: 3.0 });
    items.push({ t: 'tg', x: 0.090 + i * 0.110, y: 0.160, lab: 'BOILER', pos: ['ON', 'OFF'], st: -1, ls: 3.0 });
  });
  items.push({ t: 'box', x: 0.016, y: 0.220, w: 0.328, h: 0.120, title: 'MPS / HE ISOL' });
  items.push(...tgRow(0.050, 0.290, 0.058, [['ENG PWR L', ['AC2', 'OFF'], -1], ['C', ['AC1', 'OFF'], -1], ['R', ['AC3', 'OFF'], -1], ['HE ISOL A', ['OPEN', 'CLOSE'], 1], ['B', ['OPEN', 'CLOSE'], 1]], { ls: 3.0 }));
  items.push({ t: 'velcro', x: 0.030, y: 0.370, w: 0.120, h: 0.050 });
  items.push({ t: 'cue', x: 0.200, y: 0.360, w: 0.120, h: 0.080, title: 'APU START' });
  return { id: 'R2', w, h, items };
}

// ------------------------------------------------------------------ overhead
const OH_NAMES = [
  'GPC 1', 'GPC 2', 'GPC 3', 'GPC 4', 'GPC 5', 'MDM FF1', 'MDM FF2', 'MDM FF3', 'MDM FF4', 'MDM FA1', 'MDM FA2', 'MDM FA3', 'MDM FA4',
  'IMU 1', 'IMU 2', 'IMU 3', 'TACAN 1', 'TACAN 2', 'TACAN 3', 'RDR ALT 1', 'RDR ALT 2', 'MLS 1', 'MLS 2', 'MLS 3', 'ADTA 1', 'ADTA 2', 'ADTA 3', 'ADTA 4',
  'RGA 1', 'RGA 2', 'RGA 3', 'RGA 4', 'AA 1', 'AA 2', 'AA 3', 'AA 4', 'FWD RCS', 'MANF 1', 'MANF 2', 'MANF 3', 'MANF 4', 'MANF 5', 'HE PRESS A', 'HE PRESS B',
  'TANK ISOL', 'XFEED', 'OMS HE', 'OMS VAP', 'CRSFD', 'CNTLR 1', 'CNTLR 2', 'CNTLR 3', 'S BD PM', 'S BD FM', 'UHF', 'AUDIO', 'CCTV', 'DDU 1', 'DDU 2', 'DDU 3',
];
function overhead(id, w, h, seed, opts = {}) {
  const items = [{ t: 'id', s: id }];
  let k = seed * 7;
  const name = () => OH_NAMES[(k++ * 13 + seed) % OH_NAMES.length];
  const rows = opts.meters ? 1 : 2;
  if (opts.meters) {
    items.push({ t: 'box', x: 0.012, y: 0.026, w: w - 0.024, h: 0.150, title: opts.meters });
    const n = Math.floor((w - 0.06) / 0.034);
    for (let i = 0; i < n; i++) items.push({ t: 'meter', x: 0.030 + i * 0.034, y: 0.045, w: 0.020, h: 0.090, lab: name().split(' ')[0], k: [10, 20, 100, 50][i % 4] });
    items.push({ t: 'kn', x: w - 0.050, y: 0.205, r: 0.009, lab: 'SELECT', pos: ['1', '2', '3', '4'] });
  }
  if (opts.met) items.push({ t: 'win', x: 0.030, y: 0.192, w: 0.150, h: 0.034, dyn: 'met', lab: 'MISSION TIME' });
  const y0 = opts.meters ? 0.250 : 0.030;
  for (let r = 0; r < rows + (opts.meters ? 0 : 1); r++) {
    const y = y0 + r * 0.140;
    if (y + 0.12 > h) break;
    const title = opts.titles?.[r] || `${name().split(' ')[0]} ${['PWR', 'CNTL', 'SEL', 'MODE'][r % 4]}`;
    items.push({ t: 'box', x: 0.012, y, w: w - 0.024, h: 0.120, title });
    const n = Math.floor((w - 0.04) / 0.040);
    for (let i = 0; i < n; i++) items.push({ t: 'tg', x: 0.034 + i * 0.040, y: y + 0.072, lab: name(), ls: 3.0, lw: 0.036, pos: (k % 3 ? ['ON', 'OFF'] : ['ON', 'OFF', 'STBY']), st: (k * 31) % 5 === 0 ? 1 : -1 });
  }
  if (opts.cbs) {
    const yy = h - 0.120;
    items.push({ t: 'box', x: 0.012, y: yy, w: w - 0.024, h: 0.105, title: opts.cbs });
    const cols = Math.floor((w - 0.04) / 0.024);
    const labs = [];
    for (let i = 0; i < cols * 2; i++) labs.push(name().replace(' ', ''));
    items.push(...cbGrid(0.030, yy + 0.030, 0.024, 0.042, cols, labs));
  }
  return { id, w, h, items };
}

// corner panels between the instrument panel and the side windows (F1 commander / F5 pilot)
function corner(id, L) {
  const w = 0.42, h = 0.36, items = [{ t: 'id', s: id }];
  items.push({ t: 'box', x: 0.014, y: 0.030, w: 0.392, h: 0.110, title: L ? 'ENTRY / AIR DATA' : 'COMM / AUDIO' });
  items.push(...tgRow(0.050, 0.100, 0.056, L
    ? [['ENTRY ROLL', ['AUTO', 'LO GAIN'], -1], ['ADTA', ['1', '2'], -1], ['PROBE HTR', ['ON', 'OFF'], -1], ['RCS JETS', ['NORM', 'LO Z'], -1], ['CNTLR', ['ON', 'OFF'], -1], ['CSS PITCH', ['ON', 'OFF'], 1], ['STBY', ['A', 'B'], 1]]
    : [['UHF', ['T/R', 'OFF'], -1], ['ICOM', ['ON', 'OFF'], -1], ['A/G 1', ['T/R', 'RCV'], -1], ['A/G 2', ['T/R', 'RCV'], 1], ['A/A', ['T/R', 'RCV'], 1], ['RCD', ['ON', 'OFF'], 1], ['XMIT', ['PTT', 'VOX'], -1]], { ls: 3.2 }));
  items.push({ t: 'box', x: 0.014, y: 0.160, w: 0.190, h: 0.120, title: L ? 'RDR ALTM' : 'AUDIO' });
  items.push({ t: 'kn', x: 0.060, y: 0.215, r: 0.010, pos: ['OFF', '1', '2'], lab: 'SEL' }, { t: 'kn', x: 0.150, y: 0.215, r: 0.010, lab: 'VOL' });
  items.push({ t: 'cue', x: 0.230, y: 0.160, w: 0.170, h: 0.115, title: L ? 'TAEM / A-L CUE CARD' : 'COMM CUE CARD' });
  items.push({ t: 'velcro', x: 0.030, y: 0.300, w: 0.110, h: 0.040 }, { t: 'velcro', x: 0.260, y: 0.300, w: 0.110, h: 0.040 });
  return { id, w, h, items };
}

export function buildPanelSpecs() {
  return {
    F6: fwdSide('L'), F8: fwdSide('R'), F7: f7(), F2: glareSide('L'), F4: glareSide('R'), F3: f3(),
    C2: c2(), C3: c3(), F1: corner('F1', true), F5: corner('F5', false), L1: l1(), L2: l2(), R1: r1(), R2: r2(),
    O1: overhead('O1', 0.30, 0.455, 1, { meters: 'ATMOSPHERE / CRYO', cbs: 'MAIN A' }),
    O2: overhead('O2', 0.30, 0.455, 2, { meters: 'CRYO O2 / H2', cbs: 'MAIN B' }),
    O3: overhead('O3', 0.30, 0.455, 3, { meters: 'RCS / OMS / HYD', met: true }),
    O5: overhead('O5', 0.30, 0.455, 5, { titles: ['GPC POWER', 'IMU', 'MDM FLT CRIT'] }),
    O6: overhead('O6', 0.30, 0.455, 6, { titles: ['GPC MODE', 'GPC OUTPUT', 'TACAN / MLS'] }),
    O7: overhead('O7', 0.30, 0.455, 7, { titles: ['RCS MANIFOLD', 'FWD RCS', 'OMS HE PRESS'] }),
    O8: overhead('O8', 0.30, 0.455, 8, { meters: 'APU / HYD', cbs: 'MAIN C' }),
    O9: overhead('O9', 0.30, 0.455, 9, { titles: ['RDR ALT', 'AIR DATA', 'ANNUNCIATOR'], cbs: 'CNTL AB' }),
  };
}
