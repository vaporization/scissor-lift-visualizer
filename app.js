// Scissor Lift Visualizer (2D) — θ-driven geometry
// Mental model: solve θ → geometry forces everything else into place.

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

  // Geometry units toggle
  geomUnits: $("geomUnits"),
  label_L: $("label_L"),
  label_platformWidth: $("label_platformWidth"),
  label_baseWidth: $("label_baseWidth"),

  // Actuator attachment selectors
  actBase: $("actBase"),
  actMove: $("actMove"),

  Wpayload: $("Wpayload"),
  Wplatform: $("Wplatform"),
  WarmsStage: $("WarmsStage"),
  frictionPct: $("frictionPct"),
  SF: $("SF"),
  nAct: $("nAct"),

  out_h: $("out_h"),
  out_H: $("out_H"),
  out_w: $("out_w"),

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

function clamp(x, a, b){ return Math.max(a, Math.min(b, x)); }
function fmt(x, digits=3){ return Number.isFinite(x) ? x.toFixed(digits) : "—"; }

// --- Units (geometry-only) ---
const M_PER_IN = 0.0254;
let lastGeomUnits = ui.geomUnits?.value || "metric";

function toMeters_fromGeomUnits(val, units){
  return units === "us" ? (val * M_PER_IN) : val;
}
function fromMeters_toGeomUnits(m, units){
  return units === "us" ? (m / M_PER_IN) : m;
}
function unitLabel(units){
  return units === "us" ? "in" : "m";
}
function fmtLenFromMeters(m, units, digits=3){
  const v = fromMeters_toGeomUnits(m, units);
  return `${v.toFixed(digits)} ${unitLabel(units)}`;
}
function updateGeomLabels(units){
  // Keep labels aligned with current geometry units
  const u = unitLabel(units);
  if(ui.label_L) ui.label_L.firstChild.textContent = `\n          Arm length L (${u})\n          `;
  if(ui.label_platformWidth) ui.label_platformWidth.firstChild.textContent = `\n          Platform width (${u})\n          `;
  if(ui.label_baseWidth) ui.label_baseWidth.firstChild.textContent = `\n          Base width (${u})\n          `;
}

// --- Geometry Solver ---
function solveScissor({ L, N, thetaDeg }){
  const theta = thetaDeg * DEG2RAD;
  const h = 2 * L * Math.sin(theta);
  const w = 2 * L * Math.cos(theta);
  const H = N * h;

  const stages = [];
  for(let i=0; i<N; i++){
    const y = i * h;
    const A = {x:0, y:y};       // bottom-left
    const B = {x:w, y:y};       // bottom-right
    const D = {x:0, y:y+h};     // top-left
    const C = {x:w, y:y+h};     // top-right
    const P = {x:w/2, y:y+h/2}; // center pivot
    stages.push({ A,B,C,D,P });
  }

  const bottomPlatform = { left: stages[0].A, right: stages[0].B };
  const topPlatform = { left: stages[N-1].D, right: stages[N-1].C };

  return { theta, h, w, H, stages, bottomPlatform, topPlatform };
}

// --- Joint key system for actuator endpoints ---
function getJointByKey(sol, key){
  if(!key || key.length < 2) return null;
  const joint = key[0];
  const idx = Number(key.slice(1));
  if(!Number.isFinite(idx) || idx < 0 || idx >= sol.stages.length) return null;

  const st = sol.stages[idx];
  switch(joint){
    case "A": return st.A;
    case "B": return st.B;
    case "C": return st.C;
    case "D": return st.D;
    case "P": return st.P;
    default: return null;
  }
}

function labelForJointKey(key){
  const j = key[0];
  const i = key.slice(1);
  const name =
    j === "A" ? "Bottom-Left" :
    j === "B" ? "Bottom-Right" :
    j === "C" ? "Top-Right" :
    j === "D" ? "Top-Left" :
    j === "P" ? "Center" : j;
  return `${name} (stage ${i})`;
}

function buildJointOptions(N){
  const joints = ["A","B","P","D","C"]; // display order
  const keys = [];
  for(let i=0; i<N; i++){
    for(const j of joints) keys.push(`${j}${i}`);
  }
  return keys;
}

function refreshActuatorSelects(N){
  if(!ui.actBase || !ui.actMove) return;

  const keys = buildJointOptions(N);
  const prevBase = ui.actBase.value || "A0";
  const prevMove = ui.actMove.value || "P0";

  ui.actBase.innerHTML = "";
  ui.actMove.innerHTML = "";

  for(const k of keys){
    const o1 = document.createElement("option");
    o1.value = k;
    o1.textContent = `${k} — ${labelForJointKey(k)}`;
    ui.actBase.appendChild(o1);

    const o2 = document.createElement("option");
    o2.value = k;
    o2.textContent = `${k} — ${labelForJointKey(k)}`;
    ui.actMove.appendChild(o2);
  }

  ui.actBase.value = keys.includes(prevBase) ? prevBase : "A0";
  ui.actMove.value = keys.includes(prevMove) ? prevMove : "P0";

  // Nice default for 2+ stages: push on an inner stage center
  if(N >= 2 && ui.actMove.value === "P0"){
    ui.actMove.value = "P1";
  }
}

// Actuator length derived from selected joints
function actuatorLength(sol, baseKey, moveKey){
  const p1 = getJointByKey(sol, baseKey);
  const p2 = getJointByKey(sol, moveKey);
  if(!p1 || !p2) return NaN;
  return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

// Force trend model (given): Fact ≈ N * Wtotal * tan(theta)
function forceTrend({ N, thetaRad, Wtotal }){
  return N * Wtotal * Math.tan(thetaRad);
}

// Rendering: map world coords to svg coords
function makeViewportTransform(sol){
  const margin = 0.15;

  const minX = -margin * sol.w;
  const maxX = sol.w * (1 + margin);
  const minY = -margin * (sol.H || 1);
  const maxY = sol.H * (1 + margin) + (sol.h || 0);

  const viewW = 1000;
  const viewH = 700;

  const worldW = Math.max(1e-6, maxX - minX);
  const worldH = Math.max(1e-6, maxY - minY);

  const sx = viewW / worldW;
  const sy = viewH / worldH;
  const s = Math.min(sx, sy);

  const tx = (viewW - worldW * s) / 2 - minX * s;
  const ty = (viewH - worldH * s) / 2 + maxY * s;

  function toScreen(p){
    return { x: p.x * s + tx, y: -p.y * s + ty };
  }
  return { toScreen };
}

function clearScene(){
  while(ui.scene.firstChild) ui.scene.removeChild(ui.scene.firstChild);
}

function el(name, attrs={}){
  const n = document.createElementNS("http://www.w3.org/2000/svg", name);
  for(const [k,v] of Object.entries(attrs)) n.setAttribute(k, String(v));
  return n;
}

function draw(sol, geomUnits){
  clearScene();
  const { toScreen } = makeViewportTransform(sol);

  // Platforms
  const bpL = toScreen(sol.bottomPlatform.left);
  const bpR = toScreen(sol.bottomPlatform.right);
  const tpL = toScreen(sol.topPlatform.left);
  const tpR = toScreen(sol.topPlatform.right);

  ui.scene.appendChild(el("line", {
    x1: bpL.x, y1: bpL.y, x2: bpR.x, y2: bpR.y,
    stroke: "#a9b1c3", "stroke-width": 6, "stroke-linecap":"round", opacity: 0.95
  }));

  ui.scene.appendChild(el("line", {
    x1: tpL.x, y1: tpL.y, x2: tpR.x, y2: tpR.y,
    stroke: "#a9b1c3", "stroke-width": 6, "stroke-linecap":"round", opacity: 0.95
  }));

  // Arms
  for(const st of sol.stages){
    const A = toScreen(st.A), B = toScreen(st.B), C = toScreen(st.C), D = toScreen(st.D);

    ui.scene.appendChild(el("line", {
      x1:A.x, y1:A.y, x2:C.x, y2:C.y,
      stroke:"#c3e88d", "stroke-width": 5, "stroke-linecap":"round"
    }));
    ui.scene.appendChild(el("line", {
      x1:B.x, y1:B.y, x2:D.x, y2:D.y,
      stroke:"#c3e88d", "stroke-width": 5, "stroke-linecap":"round"
    }));
  }

  // Actuator (derived): selected joints
  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || "P0";
  const p1w = getJointByKey(sol, baseKey);
  const p2w = getJointByKey(sol, moveKey);

  if(p1w && p2w){
    const p1 = toScreen(p1w);
    const p2 = toScreen(p2w);
    ui.scene.appendChild(el("line", {
      x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y,
      stroke:"#ff9e64", "stroke-width": 6, "stroke-linecap":"round", opacity:0.95
    }));
  }

  // Joints (pivots)
  const rFixed = 7, rJoint = 5;

  // Fixed pivots (bottom A0 and B0)
  for(const p of [sol.stages[0].A, sol.stages[0].B]){
    const s = toScreen(p);
    ui.scene.appendChild(el("circle", { cx:s.x, cy:s.y, r:rFixed, fill:"#7aa2f7" }));
  }

  // All stage joints
  for(const st of sol.stages){
    for(const p of [st.A, st.B, st.C, st.D, st.P]){
      const s = toScreen(p);
      ui.scene.appendChild(el("circle", { cx:s.x, cy:s.y, r:rJoint, fill:"#c3e88d" }));
    }
  }

  // Labels (simple)
  ui.scene.appendChild(el("text", {
    x: tpL.x, y: tpL.y - 12, fill:"#a9b1c3", "font-size":"14"
  })).textContent = `Top platform y = ${fromMeters_toGeomUnits(sol.H, geomUnits).toFixed(3)} ${unitLabel(geomUnits)}`;

  ui.scene.appendChild(el("text", {
    x: bpL.x, y: bpL.y + 18, fill:"#a9b1c3", "font-size":"14"
  })).textContent = `Base`;
}

function computeWarnings(params, sol, outputs){
  const warnings = [];

  const thetaDeg = params.thetaDeg;
  const thetaMin = params.thetaMin;
  const thetaMax = params.thetaMax;
  const units = params.geomUnits;

  // Singularity proximity
  if(thetaDeg < thetaMin + 2){
    warnings.push({ kind:"bad", msg:`θ is very close to θ_min. Near-collapse region: force sensitivity is high.` });
  } else if(thetaDeg < 10){
    warnings.push({ kind:"warn", msg:`Low θ (< 10°): expect major force spike near collapse.` });
  }

  // Platform/base fit vs span (all internal meters; display in chosen units)
  if(params.platformWidth > sol.w){
    warnings.push({
      kind:"bad",
      msg:`Platform width (${fromMeters_toGeomUnits(params.platformWidth, units).toFixed(2)} ${unitLabel(units)}) exceeds scissor span w (${fromMeters_toGeomUnits(sol.w, units).toFixed(2)} ${unitLabel(units)}). Geometry/mounting mismatch.`
    });
  }
  if(params.baseWidth > sol.w){
    warnings.push({
      kind:"warn",
      msg:`Base width (${fromMeters_toGeomUnits(params.baseWidth, units).toFixed(2)} ${unitLabel(units)}) exceeds scissor span w (${fromMeters_toGeomUnits(sol.w, units).toFixed(2)} ${unitLabel(units)}). Check bottom pivot mounting feasibility.`
    });
  }

  // Angle sanity
  if(thetaMin >= thetaMax){
    warnings.push({ kind:"bad", msg:`θ_min must be < θ_max.` });
  }

  // Force reasonableness (basic heuristic)
  if(outputs.FperAct > 50000){
    warnings.push({ kind:"bad", msg:`Per-actuator force is extremely high (>50 kN). This configuration is likely impractical.` });
  } else if(outputs.FperAct > 20000){
    warnings.push({ kind:"warn", msg:`Per-actuator force is high (>20 kN). Consider raising θ_min, reducing load, or changing actuator geometry.` });
  }

  // Actuator joint selection sanity
  if(outputs.actNaN){
    warnings.push({ kind:"bad", msg:`Actuator length is invalid. Check actuator endpoint selections.` });
  }

  if(warnings.length === 0){
    warnings.push({ kind:"ok", msg:`No warnings triggered.` });
  }
  return warnings;
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

function readParams(){
  const geomUnits = ui.geomUnits?.value || "metric";
  updateGeomLabels(geomUnits);

  // Geometry inputs are displayed in selected units; convert to meters internally
  const L = toMeters_fromGeomUnits(Number(ui.L.value), geomUnits);
  const N = Math.round(Number(ui.N.value));
  const thetaMin = Number(ui.thetaMin.value);
  const thetaMax = Number(ui.thetaMax.value);

  // Sync slider bounds with min/max
  ui.theta.min = String(thetaMin);
  ui.theta.max = String(thetaMax);

  let thetaDeg = Number(ui.theta.value);
  thetaDeg = clamp(thetaDeg, thetaMin, thetaMax);
  ui.theta.value = String(thetaDeg);
  ui.thetaReadout.textContent = thetaDeg.toFixed(1);

  const platformWidth = toMeters_fromGeomUnits(Number(ui.platformWidth.value), geomUnits);
  const baseWidth = toMeters_fromGeomUnits(Number(ui.baseWidth.value), geomUnits);

  const Wpayload = Number(ui.Wpayload.value);
  const Wplatform = Number(ui.Wplatform.value);
  const WarmsStage = Number(ui.WarmsStage.value);
  const frictionPct = Number(ui.frictionPct.value);
  const SF = Number(ui.SF.value);
  const nAct = Math.max(1, Math.round(Number(ui.nAct.value)));

  return {
    geomUnits,
    L, N, thetaMin, thetaMax, thetaDeg,
    platformWidth, baseWidth,
    Wpayload, Wplatform, WarmsStage, frictionPct, SF, nAct
  };
}

function update(){
  const p = readParams();

  // Ensure actuator joint dropdowns reflect current N (and keep selection when possible)
  refreshActuatorSelects(p.N);

  const sol = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaDeg });

  // Derived actuator lengths across travel for stroke
  const solMin = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMin });
  const solMax = solveScissor({ L:p.L, N:p.N, thetaDeg:p.thetaMax });

  const baseKey = ui.actBase?.value || "A0";
  const moveKey = ui.actMove?.value || "P0";

  const actLen = actuatorLength(sol, baseKey, moveKey);
  const actMin = actuatorLength(solMin, baseKey, moveKey);
  const actMax = actuatorLength(solMax, baseKey, moveKey);

  const strokeTotal = actMax - actMin;
  const strokeUsed = actLen - actMin;

  // Total weight and force trend
  const Wtotal = p.Wpayload + p.Wplatform + p.N * p.WarmsStage;
  const Fact = forceTrend({ N:p.N, thetaRad: sol.theta, Wtotal });
  const fricMult = 1 + (p.frictionPct / 100);
  const Frated = Fact * fricMult * p.SF;
  const FperAct = Frated / p.nAct;

  // Outputs (geometry in chosen units)
  ui.out_h.textContent = fmtLenFromMeters(sol.h, p.geomUnits, 3);
  ui.out_H.textContent = fmtLenFromMeters(sol.H, p.geomUnits, 3);
  ui.out_w.textContent = fmtLenFromMeters(sol.w, p.geomUnits, 3);

  ui.out_actLen.textContent = fmtLenFromMeters(actLen, p.geomUnits, 3);
  ui.out_strokeUsed.textContent = fmtLenFromMeters(strokeUsed, p.geomUnits, 3);
  ui.out_strokeTotal.textContent = fmtLenFromMeters(strokeTotal, p.geomUnits, 3);

  ui.out_Wtotal.textContent = `${fmt(Wtotal,1)} N`;
  ui.out_Fact.textContent = `${fmt(Fact,1)} N`;
  ui.out_Fper.textContent = `${fmt(FperAct,1)} N`;

  const warnings = computeWarnings(p, sol, { FperAct, actNaN: !Number.isFinite(actLen) });
  renderWarnings(warnings);

  // Draw
  draw(sol, p.geomUnits);
}

// Animation
let anim = { running:false, dir: +1, raf:0 };

function startAnim(dir){
  anim.running = true;
  anim.dir = dir;
  const step = () => {
    if(!anim.running) return;

    const p = readParams();
    const speedDegPerSec = 18; // adjust freely
    const dt = 1/60;
    let t = Number(ui.theta.value);
    t += anim.dir * speedDegPerSec * dt;

    // Clamp and stop at limits
    if(t >= p.thetaMax){ t = p.thetaMax; anim.running = false; }
    if(t <= p.thetaMin){ t = p.thetaMin; anim.running = false; }

    ui.theta.value = String(t);
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

// --- Hook events ---
[
  ui.L, ui.N, ui.thetaMin, ui.thetaMax,
  ui.platformWidth, ui.baseWidth,
  ui.theta,
  ui.Wpayload, ui.Wplatform, ui.WarmsStage,
  ui.frictionPct, ui.SF, ui.nAct
].forEach(inp => inp.addEventListener("input", () => update()));

ui.actBase?.addEventListener("change", update);
ui.actMove?.addEventListener("change", update);

// Convert displayed geometry values when units toggle changes (so design stays same physically)
ui.geomUnits?.addEventListener("change", () => {
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

ui.btnLift.addEventListener("click", () => { stopAnim(); startAnim(+1); });
ui.btnLower.addEventListener("click", () => { stopAnim(); startAnim(-1); });
ui.btnStop.addEventListener("click", () => stopAnim());

// Initial setup
updateGeomLabels(lastGeomUnits);
refreshActuatorSelects(Math.round(Number(ui.N.value)) || 2);
update();