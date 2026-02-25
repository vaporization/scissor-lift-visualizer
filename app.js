// Scissor Lift Visualizer (2D)
// L is FULL ARM LENGTH (end pivot → end pivot), not half-link.
// Therefore: per stage h = L*sinθ, w = L*cosθ. Bars drawn A→C and B→D have length exactly L.

const $ = (id) => document.getElementById(id);

const ui = {
  L: $("L"),
  N: $("N"),
  thetaMin: $("thetaMin"),
  thetaMax: $("thetaMax"),
  platformWidth: $("platformWidth"),
  baseWidth: $("baseWidth"),
  theta: $("theta"),
  thetaReadout: $("thetaReadout"),

  geomUnits: $("geomUnits"),
  label_L: $("label_L"),
  label_platformWidth: $("label_platformWidth"),
  label_baseWidth: $("label_baseWidth"),

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

  svg: $("svg"),
  scene: $("scene"),

  btnLift: $("btnLift"),
  btnLower: $("btnLower"),
  btnStop: $("btnStop")
};

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

function clamp(x, a, b){ return Math.max(a, Math.min(b, x)); }
function fmt(x, digits=3){ return Number.isFinite(x) ? x.toFixed(digits) : "—"; }

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

function updateGeomLabels(units){
  const u = unitLabelGeom(units);
  if(ui.label_L) ui.label_L.firstChild.textContent = `\n          Arm length L (${u})\n          `;
  if(ui.label_platformWidth) ui.label_platformWidth.firstChild.textContent = `\n          Platform width (${u})\n          `;
  if(ui.label_baseWidth) ui.label_baseWidth.firstChild.textContent = `\n          Base width (${u})\n          `;
  if(ui.label_actLen) ui.label_actLen.firstChild.textContent =
    `\n          Actuator length (${u}) — driving input (when actuator ON)\n          `;
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
  if(ui.label_Wpayload) ui.label_Wpayload.firstChild.textContent = `\n          Payload weight (${u})\n          `;
  if(ui.label_Wplatform) ui.label_Wplatform.firstChild.textContent = `\n          Platform weight (${u})\n          `;
  if(ui.label_WarmsStage) ui.label_WarmsStage.firstChild.textContent = `\n          Arm weight per stage (${u})\n          `;
}

// -------------------- Geometry Solver --------------------
// L = full bar length end-to-end.
// For one stage: w = L*cosθ, h = L*sinθ
function solveScissor({ L, N, thetaDeg }){
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

  const bottomPlatform = { left: stages[0].A, right: stages[0].B };
  const topPlatform = { left: stages[N-1].D, right: stages[N-1].C };

  return { theta, h, w, H, stages, bottomPlatform, topPlatform };
}

// -------------------- Selectable points for actuator endpoints --------------------
function lerpPoint(p1, p2, t){ return { x: p1.x + (p2.x-p1.x)*t, y: p1.y + (p2.y-p1.y)*t }; }

function getSelectablePointsForStage(sol, i){
  const st = sol.stages[i];
  const {A,B,C,D,P} = st;

  const AP_mid = lerpPoint(A, P, 0.5);
  const PC_mid = lerpPoint(P, C, 0.5);
  const BP_mid = lerpPoint(B, P, 0.5);
  const PD_mid = lerpPoint(P, D, 0.5);

  return {
    [`A${i}`]: A,
    [`B${i}`]: B,
    [`C${i}`]: C,
    [`D${i}`]: D,
    [`P${i}`]: P,

    [`AP_${i}`]: AP_mid,
    [`PC_${i}`]: PC_mid,
    [`BP_${i}`]: BP_mid,
    [`PD_${i}`]: PD_mid
  };
}

function getPointByKey(sol, key){
  if(!key) return null;
  let idx = null;
  if(key.includes("_")){
    idx = Number(key.split("_").at(-1));
  } else {
    idx = Number(key.slice(1));
  }
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
}

function actuatorLength(sol, baseKey, moveKey){
  const p1 = getPointByKey(sol, baseKey);
  const p2 = getPointByKey(sol, moveKey);
  if(!p1 || !p2) return NaN;
  return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

// -------------------- Placement-aware force model (virtual work) --------------------
// F ≈ W * (dH/dθ) / (dℓ/dθ)
function placementAwareForceVW({ p, baseKey, moveKey, thetaDeg, Wtotal }){
  const dThetaDeg = 0.05;
  const t1 = clamp(thetaDeg - dThetaDeg, p.thetaMin, p.thetaMax);
  const t2 = clamp(thetaDeg + dThetaDeg, p.thetaMin, p.thetaMax);

  const sol1 = solveScissor({ L:p.L, N:p.N, thetaDeg:t1 });
  const sol2 = solveScissor({ L:p.L, N:p.N, thetaDeg:t2 });

  const H1 = sol1.H;
  const H2 = sol2.H;

  const l1 = actuatorLength(sol1, baseKey, moveKey);
  const l2 = actuatorLength(sol2, baseKey, moveKey);

  if(!Number.isFinite(l1) || !Number.isFinite(l2)) return { F: NaN, dldtheta: NaN };

  const dH = (H2 - H1);
  const dl = (l2 - l1);

  if(Math.abs(dl) < 1e-9) return { F: Infinity, dldtheta: 0 };

  const F = Wtotal * (dH / dl);
  return { F: Math.abs(F), dldtheta: dl / ((t2 - t1) * DEG2RAD) };
}

// -------------------- Inverse solve: θ from actuator length --------------------
function solveThetaFromActuatorLength(p, baseKey, moveKey, targetLenM){
  const thetaMin = p.thetaMin;
  const thetaMax = p.thetaMax;

  const lenAt = (thetaDeg) => {
    const sol = solveScissor({ L:p.L, N:p.N, thetaDeg });
    return actuatorLength(sol, baseKey, moveKey);
  };

  const lenMin = lenAt(thetaMin);
  const lenMax = lenAt(thetaMax);

  if(!Number.isFinite(lenMin) || !Number.isFinite(lenMax)){
    return { thetaDeg: thetaMin, ok:false, reason:"invalid endpoints", clamped:false };
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

  return { thetaDeg: 0.5*(a+b), ok:true, clamped };
}

// -------------------- Rendering transform --------------------
function makeViewportTransform(sol){
  const margin = 0.18;
  const minX = -margin * (sol.w || 1);
  const maxX = (sol.w || 1) * (1 + margin);
  const minY = -margin * (sol.H || 1);
  const maxY = (sol.H || 1) * (1 + margin) + (sol.h || 0);

  const viewW = 1000, viewH = 700;
  const worldW = Math.max(1e-6, maxX - minX);
  const worldH = Math.max(1e-6, maxY - minY);

  const s = Math.min(viewW/worldW, viewH/worldH);
  const tx = (viewW - worldW*s)/2 - minX*s;
  const ty = (viewH - worldH*s)/2 + maxY*s;

  return { toScreen: (p) => ({ x: p.x*s + tx, y: -p.y*s + ty }) };
}

function clearScene(){ while(ui.scene.firstChild) ui.scene.removeChild(ui.scene.firstChild); }
function el(name, attrs={}){
  const n = document.createElementNS("http://www.w3.org/2000/svg", name);
  for(const [k,v] of Object.entries(attrs)) n.setAttribute(k, String(v));
  return n;
}

function draw(sol, geomUnits, showLabels){
  clearScene();
  const { toScreen } = makeViewportTransform(sol);

  const bpL = toScreen(sol.bottomPlatform.left);
  const bpR = toScreen(sol.bottomPlatform.right);
  const tpL = toScreen(sol.topPlatform.left);
  const tpR = toScreen(sol.topPlatform.right);

  ui.scene.appendChild(el("line", { x1:bpL.x, y1:bpL.y, x2:bpR.x, y2:bpR.y, stroke:"#a9b1c3", "stroke-width":6, "stroke-linecap":"round", opacity:0.95 }));
  ui.scene.appendChild(el("line", { x1:tpL.x, y1:tpL.y, x2:tpR.x, y2:tpR.y, stroke:"#a9b1c3", "stroke-width":6, "stroke-linecap":"round", opacity:0.95 }));

  for(const st of sol.stages){
    const A = toScreen(st.A), B = toScreen(st.B), C = toScreen(st.C), D = toScreen(st.D);
    ui.scene.appendChild(el("line", { x1:A.x, y1:A.y, x2:C.x, y2:C.y, stroke:"#c3e88d", "stroke-width":5, "stroke-linecap":"round" }));
    ui.scene.appendChild(el("line", { x1:B.x, y1:B.y, x2:D.x, y2:D.y, stroke:"#c3e88d", "stroke-width":5, "stroke-linecap":"round" }));
  }

  if(ui.actEnable?.checked){
    const baseKey = ui.actBase?.value || "A0";
    const moveKey = ui.actMove?.value || "P0";
    const p1w = getPointByKey(sol, baseKey);
    const p2w = getPointByKey(sol, moveKey);
    if(p1w && p2w){
      const p1 = toScreen(p1w);
      const p2 = toScreen(p2w);
      ui.scene.appendChild(el("line", { x1:p1.x, y1:p1.y, x2:p2.x, y2:p2.y, stroke:"#ff9e64", "stroke-width":6, "stroke-linecap":"round", opacity:0.95 }));
      ui.scene.appendChild(el("circle", { cx:p1.x, cy:p1.y, r:7, fill:"#ff9e64" }));
      ui.scene.appendChild(el("circle", { cx:p2.x, cy:p2.y, r:7, fill:"#ff9e64" }));
    }
  }

  for(const p of [sol.stages[0].A, sol.stages[0].B]){
    const s = toScreen(p);
    ui.scene.appendChild(el("circle", { cx:s.x, cy:s.y, r:7, fill:"#7aa2f7" }));
  }

  for(const st of sol.stages){
    for(const p of [st.A, st.B, st.C, st.D, st.P]){
      const s = toScreen(p);
      ui.scene.appendChild(el("circle", { cx:s.x, cy:s.y, r:5, fill:"#c3e88d" }));
    }
  }

  for(let i=0; i<sol.stages.length; i++){
    const pts = getSelectablePointsForStage(sol, i);
    const keys = [`AP_${i}`, `PC_${i}`, `BP_${i}`, `PD_${i}`];
    for(const k of keys){
      const p = pts[k];
      const s = toScreen(p);
      ui.scene.appendChild(el("circle", { cx:s.x, cy:s.y, r:4.5, fill:"#89ddff", opacity:0.95 }));
      if(showLabels){
        ui.scene.appendChild(el("text", { x:s.x+6, y:s.y-6, fill:"#89ddff", "font-size":"12" })).textContent = k;
      }
    }
  }

  ui.scene.appendChild(el("text", { x:tpL.x, y:tpL.y - 12, fill:"#a9b1c3", "font-size":"14" }))
    .textContent = `Top platform y = ${fromMeters_toGeomUnits(sol.H, geomUnits).toFixed(3)} ${unitLabelGeom(geomUnits)}`;

  ui.scene.appendChild(el("text", { x:bpL.x, y:bpL.y + 18, fill:"#a9b1c3", "font-size":"14" }))
    .textContent = `Base`;
}

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

  // Force reasonableness thresholds (kept in N internally, displayed in chosen load units)
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

function readParams(){
  const geomUnits = ui.geomUnits?.value || "metric";
  updateGeomLabels(geomUnits);

  const loadUnits = ui.loadUnits?.value || "N";
  updateLoadLabels(loadUnits);

  const L = toMeters_fromGeomUnits(Number(ui.L.value), geomUnits);
  const N = Math.round(Number(ui.N.value));
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

  // Loads read in current load units and converted to Newtons internally
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
    platformWidth, baseWidth,
    Wpayload, Wplatform, WarmsStage, frictionPct, SF, nAct,
    actEnabled, actLenM, showLabels
  };
}

function setActuatorLenSliderRangeFromThetaLimits(p){
  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || "P0";

  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax });

  const lenMin = actuatorLength(solMin, baseKey, moveKey);
  const lenMax = actuatorLength(solMax, baseKey, moveKey);
  if(!Number.isFinite(lenMin) || !Number.isFinite(lenMax)) return;

  const lo = Math.min(lenMin, lenMax);
  const hi = Math.max(lenMin, lenMax);

  const loDisp = fromMeters_toGeomUnits(lo, p.geomUnits);
  const hiDisp = fromMeters_toGeomUnits(hi, p.geomUnits);

  ui.actLen.min = String(loDisp);
  ui.actLen.max = String(hiDisp);

  let v = Number(ui.actLen.value);
  v = clamp(v, loDisp, hiDisp);
  ui.actLen.value = String(v);
}

function updateDriveUI(p){
  ui.theta.disabled = p.actEnabled;
  ui.actLen.disabled = !p.actEnabled;

  const disp = fromMeters_toGeomUnits(p.actLenM, p.geomUnits);
  ui.actLenReadout.textContent = `${disp.toFixed(3)} ${unitLabelGeom(p.geomUnits)}`;
}

function update(){
  const p0 = readParams();

  refreshActuatorSelects(p0.N);
  setActuatorLenSliderRangeFromThetaLimits(p0);

  const p = readParams();
  updateDriveUI(p);

  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || (p.N >= 2 ? "P1" : "P0");

  let thetaDeg = p.thetaDeg;
  let actSolve = { ok:true, clamped:false };

  if(p.actEnabled){
    const res = solveThetaFromActuatorLength(p, baseKey, moveKey, p.actLenM);
    thetaDeg = clamp(res.thetaDeg, p.thetaMin, p.thetaMax);
    actSolve = res;

    ui.theta.value = String(thetaDeg);
    ui.thetaReadout.textContent = thetaDeg.toFixed(1);
  }

  const sol = solveScissor({ L:p.L, N:p.N, thetaDeg });

  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax });

  const actLen = actuatorLength(sol, baseKey, moveKey);
  const actMin = actuatorLength(solMin, baseKey, moveKey);
  const actMax = actuatorLength(solMax, baseKey, moveKey);

  const strokeTotal = (Number.isFinite(actMin) && Number.isFinite(actMax)) ? (actMax - actMin) : NaN;
  const strokeUsed  = (Number.isFinite(actLen) && Number.isFinite(actMin)) ? (actLen - actMin) : NaN;

  if(p.actEnabled && Number.isFinite(actLen)){
    const actDisp = fromMeters_toGeomUnits(actLen, p.geomUnits);
    ui.actLen.value = String(actDisp);
    ui.actLenReadout.textContent = `${actDisp.toFixed(3)} ${unitLabelGeom(p.geomUnits)}`;
  }

  // Loads are already in Newtons internally
  const Wtotal = p.Wpayload + p.Wplatform + p.N * p.WarmsStage;

  let FactVW = NaN;
  if(p.actEnabled){
    const vw = placementAwareForceVW({ p, baseKey, moveKey, thetaDeg, Wtotal });
    FactVW = vw.F;
  }

  const fricMult = 1 + (p.frictionPct / 100);
  const Frated = FactVW * fricMult * p.SF;
  const FperAct = (Number.isFinite(Frated) ? (Frated / p.nAct) : NaN);

  // Outputs
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

  draw(sol, p.geomUnits, p.showLabels);
}

// Animation
let anim = { running:false, dir:+1, raf:0 };

function startAnim(dir){
  anim.running = true;
  anim.dir = dir;

  const step = () => {
    if(!anim.running) return;
    const p = readParams();
    const dt = 1/60;

    if(p.actEnabled){
      const speedFracPerSec = 0.25;
      const min = Number(ui.actLen.min);
      const max = Number(ui.actLen.max);
      const span = Math.max(1e-9, max - min);

      let v = Number(ui.actLen.value);
      v += anim.dir * speedFracPerSec * span * dt;

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

// Events
[
  ui.L, ui.N, ui.thetaMin, ui.thetaMax,
  ui.platformWidth, ui.baseWidth,
  ui.theta,
  ui.Wpayload, ui.Wplatform, ui.WarmsStage,
  ui.frictionPct, ui.SF, ui.nAct
].forEach(inp => inp.addEventListener("input", () => update()));

ui.actEnable?.addEventListener("change", () => { stopAnim(); update(); });
ui.actBase?.addEventListener("change", () => { stopAnim(); update(); });
ui.actMove?.addEventListener("change", () => { stopAnim(); update(); });
ui.actLen?.addEventListener("input", () => { if(ui.actEnable.checked) update(); });
ui.showPointLabels?.addEventListener("change", update);

// Geometry units toggle: convert displayed geometry values
ui.geomUnits?.addEventListener("change", () => {
  stopAnim();
  const newUnits = ui.geomUnits.value;
  const oldUnits = lastGeomUnits;

  const L_display = Number(ui.L.value);
  const platformW_display = Number(ui.platformWidth.value);
  const baseW_display = Number(ui.baseWidth.value);

  const L_m = toMeters_fromGeomUnits(L_display, oldUnits);
  const plat_m = toMeters_fromGeomUnits(platformW_display, oldUnits);
  const base_m = toMeters_fromGeomUnits(baseW_display, oldUnits);

  ui.L.value = fromMeters_toGeomUnits(L_m, newUnits).toFixed(3);
  ui.platformWidth.value = fromMeters_toGeomUnits(plat_m, newUnits).toFixed(3);
  ui.baseWidth.value = fromMeters_toGeomUnits(base_m, newUnits).toFixed(3);

  lastGeomUnits = newUnits;
  update();
});

// Load units toggle: convert displayed load values
ui.loadUnits?.addEventListener("change", () => {
  stopAnim();
  const newUnits = ui.loadUnits.value;
  const oldUnits = lastLoadUnits;

  const wp = Number(ui.Wpayload.value);
  const wpl = Number(ui.Wplatform.value);
  const wa = Number(ui.WarmsStage.value);

  const wpN = toNewtons_fromLoadUnits(wp, oldUnits);
  const wplN = toNewtons_fromLoadUnits(wpl, oldUnits);
  const waN = toNewtons_fromLoadUnits(wa, oldUnits);

  ui.Wpayload.value = fromNewtons_toLoadUnits(wpN, newUnits).toFixed(2);
  ui.Wplatform.value = fromNewtons_toLoadUnits(wplN, newUnits).toFixed(2);
  ui.WarmsStage.value = fromNewtons_toLoadUnits(waN, newUnits).toFixed(2);

  lastLoadUnits = newUnits;
  update();
});

ui.btnLift.addEventListener("click", () => { stopAnim(); startAnim(+1); });
ui.btnLower.addEventListener("click", () => { stopAnim(); startAnim(-1); });
ui.btnStop.addEventListener("click", () => stopAnim());

// Init
updateGeomLabels(lastGeomUnits);
updateLoadLabels(lastLoadUnits);
refreshActuatorSelects(Math.round(Number(ui.N.value)) || 2);
update();