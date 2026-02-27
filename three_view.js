// three_view.js (module)
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

let renderer, scene, camera, controls;
let root;

let armMeshes = [];      // per stage: 2 arms per plane (we’ll do 1 plane first)
let jointMeshes = [];    // per stage: A,B,C,D,P spheres
let platformMeshes = { base:null, top:null };
let actuatorMeshes = { rod:null, a:null, b:null };

let lastN = -1;

export function threeInit(canvas){
  renderer = new THREE.WebGLRenderer({ canvas, antialias:true, alpha:true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d13);

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 500);
  camera.position.set(1.6, 1.2, 2.2);

  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0.6, 0.5, 0);

  // lights
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const key = new THREE.DirectionalLight(0xffffff, 0.8);
  key.position.set(2, 3, 2);
  scene.add(key);

  root = new THREE.Group();
  scene.add(root);

  window.addEventListener("resize", () => resizeToCanvas(canvas));
  resizeToCanvas(canvas);

  // optional ground grid
  const grid = new THREE.GridHelper(5, 20);
  grid.position.y = 0;
  root.add(grid);
}

function resizeToCanvas(canvas){
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if(w <= 0 || h <= 0) return;

  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function clearGroup(g){
  while(g.children.length){
    const c = g.children.pop();
    c.geometry?.dispose?.();
    c.material?.dispose?.();
  }
}

export function threeSetStageCount(N){
  if(N === lastN) return;
  lastN = N;

  // wipe previous
  armMeshes = [];
  jointMeshes = [];
  clearGroup(root);

  // keep grid
  const grid = new THREE.GridHelper(5, 20);
  grid.position.y = 0;
  root.add(grid);

  // materials
  const matArms = new THREE.MeshStandardMaterial({ color: 0xc3e88d, roughness:0.6 });
  const matJoint = new THREE.MeshStandardMaterial({ color: 0x7aa2f7, roughness:0.5 });
  const matPlat = new THREE.MeshStandardMaterial({ color: 0xa9b1c3, roughness:0.7 });
  const matAct  = new THREE.MeshStandardMaterial({ color: 0xff9e64, roughness:0.5 });

  // geometry templates (scaled later)
  const armGeo = new THREE.BoxGeometry(1, 0.04, 0.04);     // length=1, thickness set by scale
  const jointGeo = new THREE.SphereGeometry(0.03, 18, 18);
  const platGeo = new THREE.BoxGeometry(1, 0.06, 0.5);    // X scale = width, Z scale = depth

  // platforms
  platformMeshes.base = new THREE.Mesh(platGeo, matPlat);
  platformMeshes.top  = new THREE.Mesh(platGeo, matPlat);
  root.add(platformMeshes.base, platformMeshes.top);

  // arms + joints
  for(let i=0; i<N; i++){
    const arm1 = new THREE.Mesh(armGeo, matArms);
    const arm2 = new THREE.Mesh(armGeo, matArms);
    root.add(arm1, arm2);
    armMeshes.push([arm1, arm2]);

    // A,B,C,D,P joints
    const joints = {
      A: new THREE.Mesh(jointGeo, matJoint),
      B: new THREE.Mesh(jointGeo, matJoint),
      C: new THREE.Mesh(jointGeo, matJoint),
      D: new THREE.Mesh(jointGeo, matJoint),
      P: new THREE.Mesh(jointGeo, matJoint),
    };
    Object.values(joints).forEach(m => root.add(m));
    jointMeshes.push(joints);
  }

  // actuator (rod + end spheres)
  actuatorMeshes.rod = new THREE.Mesh(new THREE.BoxGeometry(1, 0.035, 0.035), matAct);
  actuatorMeshes.a = new THREE.Mesh(new THREE.SphereGeometry(0.035, 18, 18), matAct);
  actuatorMeshes.b = new THREE.Mesh(new THREE.SphereGeometry(0.035, 18, 18), matAct);
  root.add(actuatorMeshes.rod, actuatorMeshes.a, actuatorMeshes.b);
}

function placeRod(mesh, p1, p2, z=0){
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const len = Math.hypot(dx, dy);

  const mx = (p1.x + p2.x) / 2;
  const my = (p1.y + p2.y) / 2;

  mesh.position.set(mx, my, z);

  // box is length 1 along X, so scale X to len
  mesh.scale.set(len, 1, 1);

  // rotate around Z so X axis aligns with segment
  mesh.rotation.set(0, 0, Math.atan2(dy, dx));
}

export function threeRender(sol, p){
  // if stage count changed, rebuild
  threeSetStageCount(sol.stages.length);

  // platform dimensions
  const depth = p.liftDepth ?? 0.5; // meters
  const baseW = p.baseWidth;
  const topW  = p.platformWidth;

  // base platform at y=0 (or sol.bottomPlatform.left.y)
  platformMeshes.base.position.set(
    (sol.bottomPlatform.left.x + sol.bottomPlatform.right.x)/2,
    sol.bottomPlatform.left.y,
    0
  );
  platformMeshes.base.scale.set(baseW, 1, depth);

  platformMeshes.top.position.set(
    (sol.topPlatform.left.x + sol.topPlatform.right.x)/2,
    sol.topPlatform.left.y,
    0
  );
  platformMeshes.top.scale.set(topW, 1, depth);

  // arms + joints
  for(let i=0; i<sol.stages.length; i++){
    const st = sol.stages[i];

    // arms
    placeRod(armMeshes[i][0], st.A, st.C, 0);
    placeRod(armMeshes[i][1], st.B, st.D, 0);

    // joints
    jointMeshes[i].A.position.set(st.A.x, st.A.y, 0);
    jointMeshes[i].B.position.set(st.B.x, st.B.y, 0);
    jointMeshes[i].C.position.set(st.C.x, st.C.y, 0);
    jointMeshes[i].D.position.set(st.D.x, st.D.y, 0);
    jointMeshes[i].P.position.set(st.P.x, st.P.y, 0);
  }

  // actuator (optional)
  const showAct = !!p.actEnabled;
  actuatorMeshes.rod.visible = showAct;
  actuatorMeshes.a.visible = showAct;
  actuatorMeshes.b.visible = showAct;

  if(showAct){
    // You already compute these in app.js: pointWithXYOffset(...)
    // Pass in actuator world points from app.js so this module doesn’t need your solver helpers.
    const a = p.actP1w; // {x,y}
    const b = p.actP2w; // {x,y}

    placeRod(actuatorMeshes.rod, a, b, 0);
    actuatorMeshes.a.position.set(a.x, a.y, 0);
    actuatorMeshes.b.position.set(b.x, b.y, 0);
  }

  controls.update();
  renderer.render(scene, camera);
}