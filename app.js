// Scissor Lift Visualizer (3D render)
// Solver is 2D (X/Y). We render solids in 3D by giving the mechanism a Z depth.

import * as THREE from "https://unpkg.com/three@0.161.0/build/three.module.js";
import { OrbitControls } from "https://unpkg.com/three@0.161.0/examples/jsm/controls/OrbitControls.js";

const $ = (id) => document.getElementById(id);

const ui = {
  L: $("L"),
  N: $("N"),
  thetaMin: $("thetaMin"),
  thetaMax: $("thetaMax"),
  platformWidth: $("platformWidth"),
  baseWidth: $("baseWidth"),
  baseXOffset: $("baseXOffset"),
  topXOffset: $("topXOffset"),
  theta: $("theta"),
  thetaReadout: $("thetaReadout"),

  // 3D controls
  liftDepth: $("liftDepth"),
  armThk: $("armThk"),
  platThk: $("platThk"),
  jointR: $("jointR"),
  btnFit: $("btnFit"),

  geomUnits: $("geomUnits"),
  label_L: $("label_L"),
  label_platformWidth: $("label_platformWidth"),
  label_baseWidth: $("label_baseWidth"),
  label_baseXOffset: $("label_baseXOffset"),
  label_topXOffset: $("label_topXOffset"),

  loadUnits: $("loadUnits"),
  label_Wpayload: $("label_Wpayload"),
  label_Wplatform: $("label_Wplatform"),
  label_WarmsStage: $("label_WarmsStage"),

  actEnable: $("actEnable"),
  actBase: $("actBase"),
  actMove: $("actMove"),
  actLen: $("actLen"),
  actLenReadout: $("actLenReadout"),
  label_actLen: $("label_actLen"),
  showPointLabels: $("showPointLabels"),

  // actuator endpoint offsets (WORLD X/Y, in geom units)
  actBaseOffX: $("actBaseOffX"),
  actBaseOffY: $("actBaseOffY"),
  actMoveOffX: $("actMoveOffX"),
  actMoveOffY: $("actMoveOffY"),
  label_actBaseOffX: $("label_actBaseOffX"),
  label_actBaseOffY: $("label_actBaseOffY"),
  label_actMoveOffX: $("label_actMoveOffX"),
  label_actMoveOffY: $("label_actMoveOffY"),

  Wpayload: $("Wpayload"),
  Wplatform: $("Wplatform"),
  WarmsStage: $("WarmsStage"),
  frictionPct: $("frictionPct"),
  SF: $("SF"),
  nAct: $("nAct"),

  out_h: $("out_h"),
  out_H: $("out_H"),
  out_w: $("out_w"),

  out_actEnabled: $("out_actEnabled"),
  out_actLen: $("out_actLen"),
  out_strokeUsed: $("out_strokeUsed"),
  out_strokeTotal: $("out_strokeTotal"),

  out_Wtotal: $("out_Wtotal"),
  out_Fact: $("out_Fact"),
  out_Fper: $("out_Fper"),

  warnings: $("warnings"),

  // 3D canvas + status
  glcanvas: $("glcanvas"),
  vizStatus: $("vizStatus"),

  // overlay for labels
  labelOverlay: $("labelOverlay"),

  btnLift: $("btnLift"),
  btnLower: $("btnLower"),
  btnStop: $("btnStop")
};

const DEG2RAD = Math.PI / 180;
function clamp(x, a, b){ return Math.max(a, Math.min(b, x)); }

function setStatus(msg, kind=""){
  if(!ui.vizStatus) return;
  ui.vizStatus.textContent = msg;
  ui.vizStatus.className = `vizStatus ${kind}`.trim();
}

// -------------------- Units (Geometry-only) --------------------
const M_PER_IN = 0.0254;
let lastGeomUnits = ui.geomUnits?.value || "metric";

function toMeters_fromGeomUnits(val, units){ return units === "us" ? (val * M_PER_IN) : val; }
function fromMeters_toGeomUnits(m, units){ return units === "us" ? (m / M_PER_IN) : m; }
function unitLabelGeom(units){ return units === "us" ? "in" : "m"; }
function fmtLenFromMeters(m, units, digits=3){
  const v = fromMeters_toGeomUnits(m, units);
  return `${v.toFixed(digits)} ${unitLabelGeom(units)}`;
}

function setLabelText(labelEl, text){
  if(!labelEl) return;
  const nodes = Array.from(labelEl.childNodes);
  const firstText = nodes.find(n => n.nodeType === Node.TEXT_NODE);
  if(firstText) firstText.textContent = text + "\n";
}

function updateGeomLabels(units){
  const u = unitLabelGeom(units);
  setLabelText(ui.label_L, `Arm length L (${u})`);
  setLabelText(ui.label_platformWidth, `Platform width (${u})`);
  setLabelText(ui.label_baseWidth, `Base width (${u})`);
  setLabelText(ui.label_baseXOffset, `Base X offset (${u}) — slide base platform left/right`);
  setLabelText(ui.label_topXOffset, `Top X offset (${u}) — slide top platform left/right`);

  setLabelText(ui.label_actLen, `Actuator length (${u}) — driving input (when actuator ON)`);
  setLabelText(ui.label_actBaseOffX, `Base endpoint X offset (${u})`);
  setLabelText(ui.label_actBaseOffY, `Base endpoint Y offset (${u})`);
  setLabelText(ui.label_actMoveOffX, `Moving endpoint X offset (${u})`);
  setLabelText(ui.label_actMoveOffY, `Moving endpoint Y offset (${u})`);
}

// -------------------- Units (Loads/Forces) --------------------
const N_PER_LBF = 4.4482216152605;
let lastLoadUnits = ui.loadUnits?.value || "N";

function toNewtons_fromLoadUnits(val, units){ return units === "lbf" ? (val * N_PER_LBF) : val; }
function fromNewtons_toLoadUnits(N, units){ return units === "lbf" ? (N / N_PER_LBF) : N; }
function unitLabelLoad(units){ return units === "lbf" ? "lbf" : "N"; }
function fmtForceFromNewtons(N, units, digits=1){
  const v = fromNewtons_toLoadUnits(N, units);
  return `${v.toFixed(digits)} ${unitLabelLoad(units)}`;
}

function updateLoadLabels(units){
  const u = unitLabelLoad(units);
  setLabelText(ui.label_Wpayload, `Payload weight (${u})`);
  setLabelText(ui.label_Wplatform, `Platform weight (${u})`);
  setLabelText(ui.label_WarmsStage, `Arm weight per stage (${u})`);
}

// -------------------- Geometry Solver --------------------
function solveScissor({ L, N, thetaDeg, baseWidth, platformWidth, baseXOffset, topXOffset }){
  const theta = thetaDeg * DEG2RAD;
  const h = L * Math.sin(theta);
  const w = L * Math.cos(theta);
  const H = N * h;

  const stages = [];
  for(let i=0; i<N; i++){
    const y = i * h;
    const A = {x:0, y:y};
    const B = {x:w, y:y};
    const D = {x:0, y:y+h};
    const C = {x:w, y:y+h};
    const P = {x:w/2, y:y+h/2};
    stages.push({ A,B,C,D,P });
  }

  const basePivL = stages[0].A;
  const basePivR = stages[0].B;
  const topPivL = stages[N-1].D;
  const topPivR = stages[N-1].C;

  const baseCenterX = (basePivL.x + basePivR.x) / 2 + (baseXOffset || 0);
  const topCenterX  = (topPivL.x + topPivR.x) / 2 + (topXOffset || 0);

  const bw = Math.max(1e-9, baseWidth || w);
  const pw = Math.max(1e-9, platformWidth || w);

  const bottomPlatform = {
    left:  { x: baseCenterX - bw/2, y: basePivL.y },
    right: { x: baseCenterX + bw/2, y: basePivR.y },
    pivL: basePivL,
    pivR: basePivR
  };

  const topPlatform = {
    left:  { x: topCenterX - pw/2, y: topPivL.y },
    right: { x: topCenterX + pw/2, y: topPivR.y },
    pivL: topPivL,
    pivR: topPivR
  };

  const xs = [0, w, bottomPlatform.left.x, bottomPlatform.right.x, topPlatform.left.x, topPlatform.right.x];
  const ys = [0, H, bottomPlatform.left.y, topPlatform.left.y];

  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);

  return { theta, h, w, H, stages, bottomPlatform, topPlatform, minX, maxX, minY, maxY };
}

// -------------------- Selectable points --------------------
function lerpPoint(p1, p2, t){ return { x: p1.x + (p2.x-p1.x)*t, y: p1.y + (p2.y-p1.y)*t }; }

function getSelectablePointsForStage(sol, i){
  const st = sol.stages[i];
  const {A,B,C,D,P} = st;

  return {
    [`A${i}`]: A,
    [`B${i}`]: B,
    [`C${i}`]: C,
    [`D${i}`]: D,
    [`P${i}`]: P,
    [`AP_${i}`]: lerpPoint(A, P, 0.5),
    [`PC_${i}`]: lerpPoint(P, C, 0.5),
    [`BP_${i}`]: lerpPoint(B, P, 0.5),
    [`PD_${i}`]: lerpPoint(P, D, 0.5)
  };
}

function getPointByKey(sol, key){
  if(!key) return null;

  let idx = null;
  if(key.includes("_")) idx = Number(key.split("_").at(-1));
  else idx = Number(key.slice(1));

  if(!Number.isFinite(idx) || idx < 0 || idx >= sol.stages.length) return null;
  const map = getSelectablePointsForStage(sol, idx);
  return map[key] || null;
}

function pointLabel(key){
  if(/^A\d+$/.test(key)) return `${key} — Bottom-Left joint`;
  if(/^B\d+$/.test(key)) return `${key} — Bottom-Right joint`;
  if(/^C\d+$/.test(key)) return `${key} — Top-Right joint`;
  if(/^D\d+$/.test(key)) return `${key} — Top-Left joint`;
  if(/^P\d+$/.test(key)) return `${key} — Center joint`;
  if(/^AP_\d+$/.test(key)) return `${key} — Mid(A→P)`;
  if(/^PC_\d+$/.test(key)) return `${key} — Mid(P→C)`;
  if(/^BP_\d+$/.test(key)) return `${key} — Mid(B→P)`;
  if(/^PD_\d+$/.test(key)) return `${key} — Mid(P→D)`;
  return key;
}

function buildSelectableKeyList(N){
  const keys = [];
  for(let i=0; i<N; i++){
    keys.push(`A${i}`, `B${i}`, `P${i}`, `D${i}`, `C${i}`);
    keys.push(`AP_${i}`, `PC_${i}`, `BP_${i}`, `PD_${i}`);
  }
  return keys;
}

// -------------------- Actuator preset (your screenshot) --------------------
let actuatorTouchedByUser = false;

function applyActuatorPresetScreenshot(geomUnits){
  // Your screenshot: Base=BP_1, Move=PD_0, offsets: +0.5/+1.25 and -0.5/-1.25 (in)
  // Convert to current geom units
  const bx_m = 0.5 * M_PER_IN;
  const by_m = 1.25 * M_PER_IN;
  const mx_m = -0.5 * M_PER_IN;
  const my_m = -1.25 * M_PER_IN;

  ui.actBaseOffX.value = fromMeters_toGeomUnits(bx_m, geomUnits).toFixed(3);
  ui.actBaseOffY.value = fromMeters_toGeomUnits(by_m, geomUnits).toFixed(3);
  ui.actMoveOffX.value = fromMeters_toGeomUnits(mx_m, geomUnits).toFixed(3);
  ui.actMoveOffY.value = fromMeters_toGeomUnits(my_m, geomUnits).toFixed(3);

  // only set selects if options exist
  if(ui.actBase && Array.from(ui.actBase.options).some(o => o.value === "BP_1")) ui.actBase.value = "BP_1";
  if(ui.actMove && Array.from(ui.actMove.options).some(o => o.value === "PD_0")) ui.actMove.value = "PD_0";
}

function refreshActuatorSelects(N){
  const keys = buildSelectableKeyList(N);

  const prevBase = ui.actBase?.value || `A0`;
  const prevMove = ui.actMove?.value || (N >= 2 ? `P1` : `P0`);

  ui.actBase.innerHTML = "";
  ui.actMove.innerHTML = "";

  for(const k of keys){
    const o1 = document.createElement("option");
    o1.value = k;
    o1.textContent = pointLabel(k);
    ui.actBase.appendChild(o1);

    const o2 = document.createElement("option");
    o2.value = k;
    o2.textContent = pointLabel(k);
    ui.actMove.appendChild(o2);
  }

  ui.actBase.value = keys.includes(prevBase) ? prevBase : `A0`;
  ui.actMove.value = keys.includes(prevMove) ? prevMove : (N >= 2 ? `P1` : `P0`);

  // If user hasn’t touched actuator setup yet, auto-apply your screenshot defaults (for N>=2)
  if(!actuatorTouchedByUser && N >= 2){
    applyActuatorPresetScreenshot(ui.geomUnits?.value || "metric");
  }
}

// -------------------- Actuator helpers --------------------
function pointWithXYOffset(sol, key, offX, offY){
  const p = getPointByKey(sol, key);
  if(!p) return null;
  return { x: p.x + offX, y: p.y + offY };
}

function actuatorLength(sol, baseKey, moveKey, baseOffX, baseOffY, moveOffX, moveOffY){
  const p1 = pointWithXYOffset(sol, baseKey, baseOffX, baseOffY);
  const p2 = pointWithXYOffset(sol, moveKey, moveOffX, moveOffY);
  if(!p1 || !p2) return NaN;
  return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

// -------------------- Placement-aware force model --------------------
function placementAwareForceVW({ p, baseKey, moveKey, thetaDeg, Wtotal }){
  const dThetaDeg = 0.05;
  const t1 = clamp(thetaDeg - dThetaDeg, p.thetaMin, p.thetaMax);
  const t2 = clamp(thetaDeg + dThetaDeg, p.thetaMin, p.thetaMax);

  const sol1 = solveScissor({ L:p.L, N:p.N, thetaDeg:t1, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  const sol2 = solveScissor({ L:p.L, N:p.N, thetaDeg:t2, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });

  const l1 = actuatorLength(sol1, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  const l2 = actuatorLength(sol2, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);

  if(!Number.isFinite(l1) || !Number.isFinite(l2)) return { F: NaN };

  const dH = (sol2.H - sol1.H);
  const dl = (l2 - l1);
  if(Math.abs(dl) < 1e-9) return { F: Infinity };

  const F = Wtotal * (dH / dl);
  return { F: Math.abs(F) };
}

// -------------------- Inverse solve: θ from actuator length --------------------
function solveThetaFromActuatorLength(p, baseKey, moveKey, targetLenM){
  const thetaMin = p.thetaMin;
  const thetaMax = p.thetaMax;

  const lenAt = (thetaDeg) => {
    const sol = solveScissor({ L:p.L, N:p.N, thetaDeg, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
    return actuatorLength(sol, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  };

  const lenMin = lenAt(thetaMin);
  const lenMax = lenAt(thetaMax);
  if(!Number.isFinite(lenMin) || !Number.isFinite(lenMax)){
    return { thetaDeg: thetaMin, ok:false, clamped:false };
  }

  const increasing = lenMax > lenMin;
  const lo = Math.min(lenMin, lenMax);
  const hi = Math.max(lenMin, lenMax);

  let tLen = targetLenM;
  let clamped = false;
  if(tLen < lo){ tLen = lo; clamped = true; }
  if(tLen > hi){ tLen = hi; clamped = true; }

  let a = thetaMin, b = thetaMax;
  for(let iter=0; iter<50; iter++){
    const mid = 0.5*(a+b);
    const lm = lenAt(mid);
    if(!Number.isFinite(lm)) break;

    if(increasing){
      if(lm < tLen) a = mid; else b = mid;
    } else {
      if(lm > tLen) a = mid; else b = mid;
    }
  }

  return { thetaDeg: 0.5*(a+b), ok:true, clamped, increasing };
}

// -------------------- Warnings / outputs --------------------
function renderWarnings(items){
  ui.warnings.innerHTML = "";
  for(const w of items){
    const li = document.createElement("li");
    li.textContent = w.msg;
    li.className = w.kind === "bad" ? "bad" : (w.kind === "ok" ? "ok" : "");
    ui.warnings.appendChild(li);
  }
}

function computeWarnings(p, sol, out){
  const warnings = [];
  const gu = p.geomUnits;
  const lu = p.loadUnits;

  if(p.thetaDeg < p.thetaMin + 2){
    warnings.push({ kind:"bad", msg:`θ is very close to θ_min. Near-collapse region: force sensitivity is high.` });
  } else if(p.thetaDeg < 10){
    warnings.push({ kind:"warn", msg:`Low θ (< 10°): expect major force spike near collapse.` });
  }

  if(p.baseWidth > sol.w){
    warnings.push({
      kind:"warn",
      msg:`Base width (${fromMeters_toGeomUnits(p.baseWidth, gu).toFixed(2)} ${unitLabelGeom(gu)}) exceeds current scissor span w (${fromMeters_toGeomUnits(sol.w, gu).toFixed(2)} ${unitLabelGeom(gu)}). If your base pivots are fixed, this is a geometry mismatch.`
    });
  }

  if(out.actEnabled && out.actClamped){
    warnings.push({ kind:"warn", msg:`Requested actuator length was outside achievable range for this placement; it was clamped to match θ limits.` });
  }

  if(out.actEnabled && !Number.isFinite(out.actLen)){
    warnings.push({ kind:"bad", msg:`Actuator endpoints invalid for current stage count.` });
  }

  if(out.actEnabled && (!Number.isFinite(out.FactVW) || out.FactVW === Infinity)){
    warnings.push({ kind:"bad", msg:`Actuator placement is near-singular (dℓ/dθ ≈ 0). Required actuator force spikes extremely high.` });
  }

  const hiN = 50000;
  const medN = 20000;

  if(Number.isFinite(out.FperAct) && out.FperAct > hiN){
    warnings.push({ kind:"bad", msg:`Per-actuator force is extremely high (> ${fmtForceFromNewtons(hiN, lu, 0)}).` });
  } else if(Number.isFinite(out.FperAct) && out.FperAct > medN){
    warnings.push({ kind:"warn", msg:`Per-actuator force is high (> ${fmtForceFromNewtons(medN, lu, 0)}).` });
  }

  if(warnings.length === 0) warnings.push({ kind:"ok", msg:`No warnings triggered.` });
  return warnings;
}

// -------------------- Read params --------------------
function readParams(){
  const geomUnits = ui.geomUnits?.value || "metric";
  updateGeomLabels(geomUnits);

  const loadUnits = ui.loadUnits?.value || "N";
  updateLoadLabels(loadUnits);

  const L = toMeters_fromGeomUnits(Number(ui.L.value), geomUnits);
  const N = Math.max(1, Math.round(Number(ui.N.value)));
  const thetaMin = Number(ui.thetaMin.value);
  const thetaMax = Number(ui.thetaMax.value);

  ui.theta.min = String(thetaMin);
  ui.theta.max = String(thetaMax);

  let thetaDeg = Number(ui.theta.value);
  thetaDeg = clamp(thetaDeg, thetaMin, thetaMax);
  ui.theta.value = String(thetaDeg);
  ui.thetaReadout.textContent = thetaDeg.toFixed(1);

  const platformWidth = toMeters_fromGeomUnits(Number(ui.platformWidth.value), geomUnits);
  const baseWidth = toMeters_fromGeomUnits(Number(ui.baseWidth.value), geomUnits);

  const baseXOffset = toMeters_fromGeomUnits(Number(ui.baseXOffset?.value || 0), geomUnits);
  const topXOffset  = toMeters_fromGeomUnits(Number(ui.topXOffset?.value || 0), geomUnits);

  const actBaseOffX = toMeters_fromGeomUnits(Number(ui.actBaseOffX?.value || 0), geomUnits);
  const actBaseOffY = toMeters_fromGeomUnits(Number(ui.actBaseOffY?.value || 0), geomUnits);
  const actMoveOffX = toMeters_fromGeomUnits(Number(ui.actMoveOffX?.value || 0), geomUnits);
  const actMoveOffY = toMeters_fromGeomUnits(Number(ui.actMoveOffY?.value || 0), geomUnits);

  const liftDepth = toMeters_fromGeomUnits(Number(ui.liftDepth?.value || 0.5), geomUnits);
  const armThk = toMeters_fromGeomUnits(Number(ui.armThk?.value || 0.04), geomUnits);
  const platThk = toMeters_fromGeomUnits(Number(ui.platThk?.value || 0.06), geomUnits);
  const jointR = toMeters_fromGeomUnits(Number(ui.jointR?.value || 0.03), geomUnits);

  const Wpayload = toNewtons_fromLoadUnits(Number(ui.Wpayload.value), loadUnits);
  const Wplatform = toNewtons_fromLoadUnits(Number(ui.Wplatform.value), loadUnits);
  const WarmsStage = toNewtons_fromLoadUnits(Number(ui.WarmsStage.value), loadUnits);

  const frictionPct = Number(ui.frictionPct.value);
  const SF = Number(ui.SF.value);
  const nAct = Math.max(1, Math.round(Number(ui.nAct.value)));

  const actEnabled = !!ui.actEnable?.checked;
  const showLabels = !!ui.showPointLabels?.checked;

  const actLenDisplay = Number(ui.actLen?.value || 0);
  const actLenM = toMeters_fromGeomUnits(actLenDisplay, geomUnits);

  return {
    geomUnits, loadUnits,
    L, N, thetaMin, thetaMax, thetaDeg,
    platformWidth, baseWidth, baseXOffset, topXOffset,
    actBaseOffX, actBaseOffY, actMoveOffX, actMoveOffY,
    liftDepth, armThk, platThk, jointR,
    Wpayload, Wplatform, WarmsStage, frictionPct, SF, nAct,
    actEnabled, actLenM, showLabels
  };
}

function setActuatorLenSliderRangeFromThetaLimits(p){
  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || "P0";

  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });

  const lenMin = actuatorLength(solMin, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  const lenMax = actuatorLength(solMax, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  if(!Number.isFinite(lenMin) || !Number.isFinite(lenMax)) return;

  const lo = Math.min(lenMin, lenMax);
  const hi = Math.max(lenMin, lenMax);

  ui.actLen.min = String(fromMeters_toGeomUnits(lo, p.geomUnits));
  ui.actLen.max = String(fromMeters_toGeomUnits(hi, p.geomUnits));

  let v = Number(ui.actLen.value);
  v = clamp(v, Number(ui.actLen.min), Number(ui.actLen.max));
  ui.actLen.value = String(v);
}

function updateDriveUI(p){
  ui.theta.disabled = p.actEnabled;
  ui.actLen.disabled = !p.actEnabled;

  const disp = fromMeters_toGeomUnits(p.actLenM, p.geomUnits);
  ui.actLenReadout.textContent = `${disp.toFixed(3)} ${unitLabelGeom(p.geomUnits)}`;
}

// ==================== 3D Renderer ====================
let renderer, scene, camera, controls;
let world;

let mats;
let platformBase, platformTop;
let arms = [];   // [ [armAC, armBD], ... ]
let joints = []; // per stage: {A,B,C,D,P}
let actuator = { rod:null, a:null, b:null };

let lastN = -1;
let lastVisualKey = "";

// ---- Overlay label helpers ----
const overlayEls = {
  base: null,
  move: null
};

function ensureOverlayLabels(){
  if(!ui.labelOverlay) return;

  if(!overlayEls.base){
    overlayEls.base = document.createElement("div");
    overlayEls.base.className = "ptLabel base";
    overlayEls.base.innerHTML = `<span class="tag">BASE</span><span class="txt"></span>`;
    ui.labelOverlay.appendChild(overlayEls.base);
  }
  if(!overlayEls.move){
    overlayEls.move = document.createElement("div");
    overlayEls.move.className = "ptLabel move";
    overlayEls.move.innerHTML = `<span class="tag">MOVE</span><span class="txt"></span>`;
    ui.labelOverlay.appendChild(overlayEls.move);
  }
}

function setOverlayVisible(vis){
  if(overlayEls.base) overlayEls.base.style.display = vis ? "block" : "none";
  if(overlayEls.move) overlayEls.move.style.display = vis ? "block" : "none";
}

function projectToScreen(x, y, z=0){
  const v = new THREE.Vector3(x, y, z);
  v.project(camera);

  const rect = ui.glcanvas.getBoundingClientRect();
  const sx = (v.x * 0.5 + 0.5) * rect.width;
  const sy = (-v.y * 0.5 + 0.5) * rect.height;
  return { x:sx, y:sy, behind: v.z > 1 || v.z < -1 };
}

function updateEndpointLabels(baseKey, moveKey, p1, p2){
  if(!ui.labelOverlay) return;
  ensureOverlayLabels();

  const show = !!ui.showPointLabels?.checked && !!ui.actEnable?.checked && p1 && p2;
  setOverlayVisible(show);
  if(!show) return;

  const b = projectToScreen(p1.x, p1.y, 0);
  const m = projectToScreen(p2.x, p2.y, 0);

  overlayEls.base.querySelector(".txt").textContent = baseKey;
  overlayEls.move.querySelector(".txt").textContent = moveKey;

  overlayEls.base.style.left = `${b.x}px`;
  overlayEls.base.style.top  = `${b.y}px`;

  overlayEls.move.style.left = `${m.x}px`;
  overlayEls.move.style.top  = `${m.y}px`;
}

function init3D(){
  const gl = ui.glcanvas.getContext("webgl2") || ui.glcanvas.getContext("webgl");
  if(!gl){
    setStatus("WebGL context unavailable (blocked or unsupported).", "bad");
    throw new Error("WebGL not available");
  }

  renderer = new THREE.WebGLRenderer({ canvas: ui.glcanvas, antialias:true, alpha:false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setClearColor(0x0a0d13, 1);

  scene = new THREE.Scene();

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 2000);
  camera.position.set(1.6, 1.2, 2.2);

  controls = new OrbitControls(camera, ui.glcanvas);
  controls.enableDamping = true;
  controls.target.set(0.6, 0.5, 0);

  scene.add(new THREE.AmbientLight(0xffffff, 0.60));
  const key = new THREE.DirectionalLight(0xffffff, 0.90);
  key.position.set(2.5, 3.5, 2.0);
  scene.add(key);

  world = new THREE.Group();
  scene.add(world);

  const grid = new THREE.GridHelper(6, 24);
  grid.position.y = 0;
  world.add(grid);

  mats = {
    arms: new THREE.MeshStandardMaterial({ color: 0xc3e88d, roughness:0.6, metalness:0.05 }),
    joints: new THREE.MeshStandardMaterial({ color: 0x7aa2f7, roughness:0.5, metalness:0.05 }),
    platforms: new THREE.MeshStandardMaterial({ color: 0xa9b1c3, roughness:0.75, metalness:0.02 }),
    actuator: new THREE.MeshStandardMaterial({ color: 0xff9e64, roughness:0.55, metalness:0.05 })
  };

  window.addEventListener("resize", resize3D);
  resize3D();

  setStatus("3D ready.", "ok");
  requestAnimationFrame(render3DLoop);
}

function resize3D(){
  const w = ui.glcanvas.clientWidth;
  const h = ui.glcanvas.clientHeight;
  if(w <= 0 || h <= 0) return;

  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function render3DLoop(){
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(render3DLoop);
}

function clearGroupKeepGrid(){
  const keep = world.children[0];
  for(let i=world.children.length-1; i>=0; i--){
    const c = world.children[i];
    if(c === keep) continue;
    world.remove(c);
    c.traverse?.(obj => {
      if(obj.geometry) obj.geometry.dispose();
      if(obj.material){
        if(Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
        else obj.material.dispose();
      }
    });
  }
}

function ensureStageMeshes(N, visual){
  const vKey = `${N}|${visual.armThk}|${visual.jointR}|${visual.platThk}`;
  if(N === lastN && vKey === lastVisualKey) return;

  lastN = N;
  lastVisualKey = vKey;

  arms = [];
  joints = [];
  platformBase = null;
  platformTop = null;

  clearGroupKeepGrid();

  const platGeo = new THREE.BoxGeometry(1, 1, 1);
  platformBase = new THREE.Mesh(platGeo, mats.platforms);
  platformTop  = new THREE.Mesh(platGeo, mats.platforms);
  world.add(platformBase, platformTop);

  const armGeo = new THREE.BoxGeometry(1, 1, 1);
  const jointGeo = new THREE.SphereGeometry(Math.max(visual.jointR, 1e-6), 18, 18);

  for(let i=0; i<N; i++){
    const armAC = new THREE.Mesh(armGeo, mats.arms);
    const armBD = new THREE.Mesh(armGeo, mats.arms);
    world.add(armAC, armBD);
    arms.push([armAC, armBD]);

    const j = {
      A: new THREE.Mesh(jointGeo, mats.joints),
      B: new THREE.Mesh(jointGeo, mats.joints),
      C: new THREE.Mesh(jointGeo, mats.joints),
      D: new THREE.Mesh(jointGeo, mats.joints),
      P: new THREE.Mesh(jointGeo, mats.joints)
    };
    Object.values(j).forEach(m => world.add(m));
    joints.push(j);
  }

  actuator.rod = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), mats.actuator);
  actuator.a   = new THREE.Mesh(new THREE.SphereGeometry(Math.max(visual.jointR*1.15, 1e-6), 18, 18), mats.actuator);
  actuator.b   = new THREE.Mesh(new THREE.SphereGeometry(Math.max(visual.jointR*1.15, 1e-6), 18, 18), mats.actuator);
  world.add(actuator.rod, actuator.a, actuator.b);
}

function placeRodBox(mesh, p1, p2, thk, z){
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const len = Math.hypot(dx, dy);

  const mx = (p1.x + p2.x) / 2;
  const my = (p1.y + p2.y) / 2;

  mesh.position.set(mx, my, z);
  mesh.rotation.set(0, 0, Math.atan2(dy, dx));
  mesh.scale.set(Math.max(len, 1e-6), Math.max(thk, 1e-6), Math.max(thk, 1e-6));
}

function render3D(sol, p){
  const zDepth = Math.max(p.liftDepth, 1e-6);
  const zPlane = 0;

  ensureStageMeshes(sol.stages.length, { armThk:p.armThk, jointR:p.jointR, platThk:p.platThk });

  const baseCenterX = (sol.bottomPlatform.left.x + sol.bottomPlatform.right.x) / 2;
  const baseY = sol.bottomPlatform.left.y;

  const topCenterX = (sol.topPlatform.left.x + sol.topPlatform.right.x) / 2;
  const topY = sol.topPlatform.left.y;

  platformBase.position.set(baseCenterX, baseY - p.platThk/2, zPlane);
  platformBase.scale.set(Math.max(p.baseWidth, 1e-6), Math.max(p.platThk, 1e-6), zDepth);

  platformTop.position.set(topCenterX, topY + p.platThk/2, zPlane);
  platformTop.scale.set(Math.max(p.platformWidth, 1e-6), Math.max(p.platThk, 1e-6), zDepth);

  for(let i=0; i<sol.stages.length; i++){
    const st = sol.stages[i];

    placeRodBox(arms[i][0], st.A, st.C, p.armThk, zPlane);
    placeRodBox(arms[i][1], st.B, st.D, p.armThk, zPlane);

    joints[i].A.position.set(st.A.x, st.A.y, zPlane);
    joints[i].B.position.set(st.B.x, st.B.y, zPlane);
    joints[i].C.position.set(st.C.x, st.C.y, zPlane);
    joints[i].D.position.set(st.D.x, st.D.y, zPlane);
    joints[i].P.position.set(st.P.x, st.P.y, zPlane);
  }

  const showAct = !!p.actEnabled;
  actuator.rod.visible = showAct;
  actuator.a.visible = showAct;
  actuator.b.visible = showAct;

  if(showAct && p.actP1w && p.actP2w){
    placeRodBox(actuator.rod, p.actP1w, p.actP2w, Math.max(p.armThk*0.85, 1e-6), zPlane);
    actuator.a.position.set(p.actP1w.x, p.actP1w.y, zPlane);
    actuator.b.position.set(p.actP2w.x, p.actP2w.y, zPlane);
  }
}

// IMPORTANT: renamed to avoid collisions with old code
function fitCameraToSolution3D(sol, p){
  const pad = 0.20;
  const spanX = Math.max(1e-6, sol.maxX - sol.minX);
  const spanY = Math.max(1e-6, sol.maxY - sol.minY);

  const cx = (sol.minX + sol.maxX) / 2;
  const cy = (sol.minY + sol.maxY) / 2;

  controls.target.set(cx, cy, 0);

  const fov = camera.fov * DEG2RAD;
  const maxSpan = Math.max(spanX, spanY) * (1 + pad);
  const dist = (maxSpan / 2) / Math.tan(fov / 2);

  camera.position.set(cx + dist*0.9, cy + dist*0.6, dist*1.1);
  camera.updateProjectionMatrix();
  controls.update();
}

// -------------------- Update loop --------------------
function update(){
  const p0 = readParams();

  refreshActuatorSelects(p0.N);
  setActuatorLenSliderRangeFromThetaLimits(p0);

  const p = readParams();
  updateDriveUI(p);

  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || (p.N >= 2 ? "P1" : "P0");

  let thetaDeg = p.thetaDeg;
  let actSolve = { ok:true, clamped:false, increasing:true };

  if(p.actEnabled){
    const res = solveThetaFromActuatorLength(p, baseKey, moveKey, p.actLenM);
    thetaDeg = clamp(res.thetaDeg, p.thetaMin, p.thetaMax);
    actSolve = res;

    ui.theta.value = String(thetaDeg);
    ui.thetaReadout.textContent = thetaDeg.toFixed(1);
  }

  const sol = solveScissor({ L:p.L, N:p.N, thetaDeg, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });

  const actLen = actuatorLength(sol, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  const actMin = actuatorLength(solMin, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  const actMax = actuatorLength(solMax, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);

  const strokeTotal = (Number.isFinite(actMin) && Number.isFinite(actMax)) ? (actMax - actMin) : NaN;
  const strokeUsed  = (Number.isFinite(actLen) && Number.isFinite(actMin)) ? (actLen - actMin) : NaN;

  if(p.actEnabled && Number.isFinite(actLen)){
    const actDisp = fromMeters_toGeomUnits(actLen, p.geomUnits);
    ui.actLen.value = String(actDisp);
    ui.actLenReadout.textContent = `${actDisp.toFixed(3)} ${unitLabelGeom(p.geomUnits)}`;
  }

  const Wtotal = p.Wpayload + p.Wplatform + p.N * p.WarmsStage;

  let FactVW = NaN;
  if(p.actEnabled){
    FactVW = placementAwareForceVW({ p, baseKey, moveKey, thetaDeg, Wtotal }).F;
  }

  const fricMult = 1 + (p.frictionPct / 100);
  const Frated = FactVW * fricMult * p.SF;
  const FperAct = (Number.isFinite(Frated) ? (Frated / p.nAct) : NaN);

  ui.out_h.textContent = fmtLenFromMeters(sol.h, p.geomUnits, 3);
  ui.out_H.textContent = fmtLenFromMeters(sol.H, p.geomUnits, 3);
  ui.out_w.textContent = fmtLenFromMeters(sol.w, p.geomUnits, 3);

  ui.out_actEnabled.textContent = p.actEnabled ? "Yes (actuator-driven)" : "No (θ-driven)";
  ui.out_actLen.textContent = p.actEnabled ? fmtLenFromMeters(actLen, p.geomUnits, 3) : "—";
  ui.out_strokeUsed.textContent = p.actEnabled ? fmtLenFromMeters(strokeUsed, p.geomUnits, 3) : "—";
  ui.out_strokeTotal.textContent = p.actEnabled ? fmtLenFromMeters(strokeTotal, p.geomUnits, 3) : "—";

  ui.out_Wtotal.textContent = fmtForceFromNewtons(Wtotal, p.loadUnits, 1);
  ui.out_Fact.textContent = p.actEnabled ? fmtForceFromNewtons(FactVW, p.loadUnits, 1) : "—";
  ui.out_Fper.textContent = p.actEnabled ? fmtForceFromNewtons(FperAct, p.loadUnits, 1) : "—";

  renderWarnings(computeWarnings(
    { ...p, thetaDeg },
    sol,
    { actEnabled:p.actEnabled, actClamped:!!actSolve.clamped, actLen, FactVW, FperAct }
  ));

  let actP1w = null, actP2w = null;
  if(p.actEnabled){
    actP1w = pointWithXYOffset(sol, baseKey, p.actBaseOffX, p.actBaseOffY);
    actP2w = pointWithXYOffset(sol, moveKey, p.actMoveOffX, p.actMoveOffY);
  }

  // Update overlay labels (endpoint keys) if enabled
  updateEndpointLabels(baseKey, moveKey, actP1w, actP2w);

  render3D(sol, { ...p, thetaDeg, actP1w, actP2w });
}

// -------------------- Animation --------------------
let anim = { running:false, dir:+1, raf:0 };

function actuatorLengthIncreasesWithTheta(p, baseKey, moveKey){
  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });

  const lenMin = actuatorLength(solMin, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);
  const lenMax = actuatorLength(solMax, baseKey, moveKey, p.actBaseOffX, p.actBaseOffY, p.actMoveOffX, p.actMoveOffY);

  if(!Number.isFinite(lenMin) || !Number.isFinite(lenMax)) return true;
  return lenMax > lenMin;
}

function startAnim(dir){
  anim.running = true;
  anim.dir = dir;

  // Determine whether “Lift” should increase or decrease actuator length for this placement
  const pStart = readParams();
  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || (pStart.N >= 2 ? "P1" : "P0");
  const lenIncreases = actuatorLengthIncreasesWithTheta(pStart, baseKey, moveKey);

  const step = () => {
    if(!anim.running) return;
    const p = readParams();
    const dt = 1/60;

    if(p.actEnabled){
      const speedFracPerSec = 0.25;
      const min = Number(ui.actLen.min);
      const max = Number(ui.actLen.max);
      const span = Math.max(1e-9, max - min);

      // Lift means theta increases.
      // If actuator length *decreases* with theta, flip the direction.
      const effDir = anim.dir * (lenIncreases ? +1 : -1);

      let v = Number(ui.actLen.value);
      v += effDir * speedFracPerSec * span * dt;

      if(v >= max){ v = max; anim.running = false; }
      if(v <= min){ v = min; anim.running = false; }

      ui.actLen.value = String(v);
      ui.actLenReadout.textContent = `${v.toFixed(3)} ${unitLabelGeom(p.geomUnits)}`;
    } else {
      const speedDegPerSec = 18;
      let t = Number(ui.theta.value);
      t += anim.dir * speedDegPerSec * dt;

      if(t >= p.thetaMax){ t = p.thetaMax; anim.running = false; }
      if(t <= p.thetaMin){ t = p.thetaMin; anim.running = false; }

      ui.theta.value = String(t);
    }

    update();
    anim.raf = requestAnimationFrame(step);
  };

  anim.raf = requestAnimationFrame(step);
}

function stopAnim(){
  anim.running = false;
  if(anim.raf) cancelAnimationFrame(anim.raf);
  anim.raf = 0;
}

// -------------------- Events --------------------
[
  ui.L, ui.N, ui.thetaMin, ui.thetaMax,
  ui.platformWidth, ui.baseWidth,
  ui.baseXOffset, ui.topXOffset,

  ui.liftDepth, ui.armThk, ui.platThk, ui.jointR,

  ui.actBaseOffX, ui.actBaseOffY, ui.actMoveOffX, ui.actMoveOffY,

  ui.theta,
  ui.Wpayload, ui.Wplatform, ui.WarmsStage,
  ui.frictionPct, ui.SF, ui.nAct
].forEach(inp => inp && inp.addEventListener("input", () => update()));

function markActuatorTouched(){
  actuatorTouchedByUser = true;
}

ui.actEnable?.addEventListener("change", () => {
  stopAnim();

  // If turning ON and user hasn’t touched actuator settings yet, apply your screenshot preset
  if(ui.actEnable.checked && !actuatorTouchedByUser){
    applyActuatorPresetScreenshot(ui.geomUnits?.value || "metric");
  }

  update();
});

ui.actBase?.addEventListener("change", () => { stopAnim(); markActuatorTouched(); update(); });
ui.actMove?.addEventListener("change", () => { stopAnim(); markActuatorTouched(); update(); });

ui.actLen?.addEventListener("input", () => { if(ui.actEnable.checked) update(); });

ui.showPointLabels?.addEventListener("change", () => update());

ui.actBaseOffX?.addEventListener("input", () => { markActuatorTouched(); update(); });
ui.actBaseOffY?.addEventListener("input", () => { markActuatorTouched(); update(); });
ui.actMoveOffX?.addEventListener("input", () => { markActuatorTouched(); update(); });
ui.actMoveOffY?.addEventListener("input", () => { markActuatorTouched(); update(); });

ui.btnFit?.addEventListener("click", () => {
  const p = readParams();
  const sol = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaDeg, baseWidth:p.baseWidth, platformWidth:p.platformWidth, baseXOffset:p.baseXOffset, topXOffset:p.topXOffset });
  fitCameraToSolution3D(sol, p);
});

ui.geomUnits?.addEventListener("change", () => {
  stopAnim();
  const newUnits = ui.geomUnits.value;
  const oldUnits = lastGeomUnits;

  const convIds = [
    "L","platformWidth","baseWidth","baseXOffset","topXOffset",
    "actBaseOffX","actBaseOffY","actMoveOffX","actMoveOffY",
    "liftDepth","armThk","platThk","jointR"
  ];

  for(const id of convIds){
    const el = $(id);
    if(!el) continue;
    const vDisp = Number(el.value || 0);
    const m = toMeters_fromGeomUnits(vDisp, oldUnits);
    el.value = fromMeters_toGeomUnits(m, newUnits).toFixed(3);
  }

  lastGeomUnits = newUnits;
  update();
});

ui.loadUnits?.addEventListener("change", () => {
  stopAnim();
  const newUnits = ui.loadUnits.value;
  const oldUnits = lastLoadUnits;

  const wpN = toNewtons_fromLoadUnits(Number(ui.Wpayload.value), oldUnits);
  const wplN = toNewtons_fromLoadUnits(Number(ui.Wplatform.value), oldUnits);
  const waN = toNewtons_fromLoadUnits(Number(ui.WarmsStage.value), oldUnits);

  ui.Wpayload.value = fromNewtons_toLoadUnits(wpN, newUnits).toFixed(2);
  ui.Wplatform.value = fromNewtons_toLoadUnits(wplN, newUnits).toFixed(2);
  ui.WarmsStage.value = fromNewtons_toLoadUnits(waN, newUnits).toFixed(2);

  lastLoadUnits = newUnits;
  update();
});

ui.btnLift.addEventListener("click", () => { stopAnim(); startAnim(+1); });
ui.btnLower.addEventListener("click", () => { stopAnim(); startAnim(-1); });
ui.btnStop.addEventListener("click", () => stopAnim());

// -------------------- Init --------------------
try{
  setStatus("Initializing…");
  init3D();
  updateGeomLabels(lastGeomUnits);
  updateLoadLabels(lastLoadUnits);

  refreshActuatorSelects(Math.round(Number(ui.N.value)) || 2);

  // Ensure your preset is applied on fresh load (unless user already touched settings)
  if(!actuatorTouchedByUser){
    applyActuatorPresetScreenshot(ui.geomUnits?.value || "metric");
  }

  update();
  setStatus("Running.", "ok");
}catch(err){
  console.error(err);
  setStatus(`Init failed: ${err?.message || err}`, "bad");
}