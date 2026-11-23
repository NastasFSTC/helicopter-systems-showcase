// Robinson R44 II — 2-blade teetering rotor helicopter (POH Section 7, see RESEARCH.md B).
//   Lycoming IO-540 -> lower sheave on the engine output shaft -> 4 double vee-belts -> upper
//   sheave with an overrunning (sprag) clutch in its hub, raised by the clutch actuator between the
//   sheaves -> clutch shaft forward (flex coupling) to the single-stage spiral-bevel main gearbox,
//   and aft (flex couplings) along the tail drive shaft (no support bearings, one damper bearing)
//   to the 90° spiral-bevel tail gearbox and the 2-blade teetering tail rotor.
//   Main rotor: 2 blades on coning hinges, hub on a teeter hinge; 3 hydraulic servos on the
//   push-pull tubes support the swashplate. Throttle is correlated to the collective.
// Speeds (100 %): engine ≈ 2,665 rpm, sheave ratio 0.778 -> ≈ 2,073 rpm, main gearbox 11:57 ->
// 400 rpm. Tail gearbox 31:27 (ASSUMED) -> ≈ 2,380 rpm (published ≈ 2,378 at 100 %).
// Coordinates: +X forward, +Y up, +Z right (airframe axes).
import * as THREE from 'three';
import { Rotor, bevelPair } from './uh60-transmission.js';
import { MAT, shadow, cylZ, tubeZ, boltRing, flexCoupling } from './parts.js';
import { Joint, Link, Servo, hexNut, latheX } from './detail.js';
import { Lever } from './cockpit.js';
import { airfoilBlade, BLADE_MAT } from './tail.js';

const V = (x, y, z) => new THREE.Vector3(x, y, z);
const Yax = V(0, 1, 0);
const deg = THREE.MathUtils.degToRad;
const azDir = (phi) => V(Math.cos(phi), 0, -Math.sin(phi));
const tang = (a) => V(-Math.sin(a), 0, -Math.cos(a));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export const R44_DATA = {
  MAST_RPM: 400, SHEAVE_RATIO: 0.778, MAIN: { pinion: 11, gear: 57 }, TAIL: { input: 31, output: 27 },
};
const SMALL = { gap: 0.09, l: 0.11, h: 0.11, t: 0.022 };

function box(sx, sy, sz, p, mat = MAT.paint) {
  const m = shadow(new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat));
  m.position.copy(p);
  return m;
}
function bar(parent, a, b, r, mat = MAT.paint) {
  const d = b.clone().sub(a);
  const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, d.length(), 14), mat));
  m.position.copy(a).addScaledVector(d, 0.5);
  m.quaternion.setFromUnitVectors(Yax, d.normalize());
  parent.add(m);
  return m;
}
function cylY(r, y0, y1, mat, seg = 40) {
  const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, y1 - y0, seg), mat));
  m.position.y = (y0 + y1) / 2;
  return m;
}
function ringY(rOut, rIn, y0, y1, mat, seg = 72) {
  const m = tubeZ(rOut, rIn, y0, y1, mat, seg);
  m.rotation.x = -Math.PI / 2;
  return m;
}
function ghost(op = 0.07) { const g = MAT.glass.clone(); g.opacity = op; return g; }
function support(m) { m.userData.support = true; return m; }
function knee(a, b, l, out) {
  const mid = a.clone().add(b).multiplyScalar(0.5);
  const d = b.clone().sub(a);
  const h = Math.sqrt(Math.max(l * l - d.lengthSq() / 4, 0));
  const perp = out.clone().addScaledVector(d, -out.dot(d) / d.lengthSq()).normalize();
  return mid.addScaledVector(perp, h);
}
/** V-groove sheave on the local Z axis, centred on z = 0. */
function sheave(r, width, grooves, mat = MAT.steel) {
  const prof = [[0.1, -width / 2], [r, -width / 2]];
  const gw = width / grooves;
  for (let i = 0; i < grooves; i++) {
    const z0 = -width / 2 + i * gw;
    prof.push([r, z0 + gw * 0.1], [r - 0.06, z0 + gw * 0.5], [r, z0 + gw * 0.9]);
  }
  prof.push([r, width / 2], [0.1, width / 2]);
  const g = new THREE.LatheGeometry(prof.map(([a, b]) => new THREE.Vector2(a, b)), 64);
  g.rotateX(Math.PI / 2);   // lathe axis Y -> Z
  return shadow(new THREE.Mesh(g, mat));
}

export function buildR44(root, controls) {
  const anchors = {};
  const rotors = [];
  const pick = [];

  // ================= drive system =================
  // main gearbox: single-stage spiral bevel 11:57 (pinion aft of the apex, gear below it)
  const SH_Y = 2.0;                                // clutch shaft / tail shaft line
  const O = V(2.6, SH_Y, 0);
  const mg = bevelPair(root, O, V(-1, 0, 0), V(0, -1, 0), { z1: 11, z2: 57, m: 0.03, faceWidth: 0.24, rim1: 0.08, rim2: 0.1, names: ['Main gearbox pinion', 'Main rotor gear'] });
  mg.g1.k = 57 / 11; mg.g1.phase = 0;
  mg.solve();
  if (mg.g2.k < 0) { mg.g1.k = -mg.g1.k; mg.solve(); }       // mast must turn CCW seen from above
  rotors.push(mg.g1, mg.g2);
  const gearC = mg.g2.frame.position.clone();
  const MAST_TOP = 5.6;
  mg.g2.add(cylZ(0.13, -0.12, MAST_TOP - gearC.y, MAT.alu));            // mast (on the gear)
  mg.g2.add(cylZ(0.3, -0.12, 0.02, MAT.steelDark));
  // clutch shaft (pinion) -> forward flex coupling -> upper sheave (sprag clutch in hub) -> aft
  const pin = mg.g1;                                                    // frame axis +X, shaft along local -Z
  const pinX = pin.frame.position.x;
  const toLocal = (x) => -(pinX - x);                                   // world x -> pinion local z
  pin.add(cylZ(0.08, 0.05, toLocal(-0.35), MAT.steel));
  pin.add(flexCoupling(toLocal(1.0), 0.08, MAT.paint));
  const R_UP = 0.578, R_LO = R_UP * R44_DATA.SHEAVE_RATIO;
  const upSheave = sheave(R_UP, 0.44, 8);
  upSheave.position.z = toLocal(0.3);
  pin.add(upSheave);
  pin.add(tubeZ(0.2, 0.08, toLocal(0.52) + 0.01, toLocal(0.08) - 0.01, MAT.steelDark));   // clutch hub
  for (let i = 0; i < 12; i++) {                                         // sprags visible in the hub face
    const a = (i / 12) * Math.PI * 2;
    const sp = box(0.03, 0.08, 0.02, V(Math.cos(a) * 0.14, Math.sin(a) * 0.14, toLocal(0.52) + 0.02), MAT.sprag);
    sp.rotation.z = a + 0.4;
    pin.add(sp);
  }
  pin.add(flexCoupling(toLocal(-0.3), 0.08, MAT.paint));
  // tail drive shaft: long, no support bearings (one lightly-loaded damper bearing), flex coupling each end
  const TGB_X = -13.3;
  pin.add(cylZ(0.08, toLocal(-0.35), toLocal(TGB_X + 0.75), MAT.alu));
  pin.add(flexCoupling(toLocal(TGB_X + 0.85), 0.08, MAT.paint));
  // engine output shaft + lower sheave + direct-drive cooling fan (open belt drive: same direction)
  const lo = new Rotor(root, V(0.3, 0, 0), V(1, 0, 0), { name: 'Engine / lower sheave' });
  lo.k = pin.k * (R_UP / R_LO);
  rotors.push(lo);
  lo.add(sheave(R_LO, 0.44, 8));
  lo.add(cylZ(0.09, 0.25, -1.25, MAT.steel));
  {
    const fan = new THREE.Group();
    fan.position.z = -1.0;
    fan.add(cylZ(0.62, -0.02, 0.02, MAT.steelDark, { seg: 48 }));
    for (let i = 0; i < 28; i++) {
      const a = (i / 28) * Math.PI * 2;
      const vane = box(0.05, 0.18, 0.28, V(Math.cos(a) * 0.53, Math.sin(a) * 0.53, -0.15), MAT.steel);
      vane.rotation.z = a + 0.5;
      fan.add(vane);
    }
    lo.add(fan);
  }
  // fan scroll (see-through) and the engine
  const scroll = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.85, 0.85, 0.42, 48, 1, true), ghost(0.12)));
  scroll.rotation.z = Math.PI / 2;
  scroll.position.set(-0.85, 0, 0);
  scroll.userData.support = true;
  root.add(scroll);
  // ---------------- Lycoming IO-540: 6 cylinders horizontally opposed, 4-stroke ----------------
  // Cylinders 1-2 at the front (output end), odd numbers on the right bank, even on the left;
  // firing order 1-4-5-2-3-6 (Lycoming manual). Opposed pairs (1-2, 3-4, 5-6) reach top dead
  // centre together, one on compression and one on exhaust; a cylinder fires every 120° of crank.
  const eng = new THREE.Group();                 // housing (cut open along the crank plane)
  root.add(eng);
  eng.add(box(2.3, 0.7, 0.8, V(-2.2, 0, 0), MAT.engine));                       // crankcase
  eng.add(box(0.3, 0.5, 0.5, V(-1.0, 0, 0), MAT.engine));                       // front case / output
  const CR = 0.11, ROD = 0.4, CROWN = 0.08, HEAD_FACE = 0.68;                    // crank radius, rod, piston crown, chamber
  const FIRE = { 1: 0, 4: 120, 5: 240, 2: 360, 3: 480, 6: 600 };               // crank degrees of each firing (TDC compression)
  const cyls = [];
  const sgn = () => Math.sign(lo.k) || 1;
  const crank = new THREE.Group();               // crankshaft, turns with the engine output / lower sheave
  lo.add(crank);
  crank.add(cylZ(0.085, -1.1, -3.3, MAT.steelDark, { seg: 28 }));                // main journals
  const gasMat = (c) => new THREE.MeshPhysicalMaterial({ color: c, emissive: c, emissiveIntensity: 0, transparent: true, opacity: 0.35, depthWrite: false });
  for (const s of [1, -1]) {
    for (let i = 0; i < 3; i++) {
      const num = s > 0 ? 1 + 2 * i : 2 + 2 * i;
      const x = -1.45 - i * 0.62 + (s > 0 ? 0 : 0.2);
      // housing: barrel, cooling fins, head
      const barrel = tubeZ(0.245, 0.205, Math.min(0.36 * s, HEAD_FACE * s), Math.max(0.36 * s, HEAD_FACE * s), MAT.steelDark, 40);
      barrel.position.x = x;
      eng.add(barrel);
      for (let f = 0; f < 5; f++) {
        const z0 = s * (0.42 + f * 0.055);
        const fin = tubeZ(0.3, 0.244, Math.min(z0, z0 + 0.015 * s), Math.max(z0, z0 + 0.015 * s), MAT.steelDark, 40);
        fin.position.x = x;
        eng.add(fin);
      }
      eng.add(box(0.44, 0.48, 0.28, V(x, 0, s * (HEAD_FACE + 0.14)), MAT.engine));
      // crank throw for this cylinder: TDC when the pin points along this cylinder's bore
      const T = FIRE[num] % 360;
      const base = s > 0 ? Math.PI : 0;
      const gamma = base - (lo.phase || 0) - sgn() * THREE.MathUtils.degToRad(T);
      const zl = -(0.3 - x);                     // world x -> crank local z
      const pin = V(CR * Math.cos(gamma), CR * Math.sin(gamma), zl);
      const pinObj = new THREE.Object3D(); pinObj.position.copy(pin); crank.add(pinObj);
      for (const dz of [-0.07, 0.07]) {          // crank cheeks (webs) either side of the pin
        const cg = new THREE.Group();
        cg.position.set(0, 0, zl + dz);
        cg.rotation.z = gamma;
        const web = box(CR + 0.14, 0.13, 0.035, V((CR - 0.06) / 2 + 0.01, 0, 0), MAT.steelDark);
        cg.add(web);
        crank.add(cg);
      }
      const cp = cylZ(0.05, zl - 0.075, zl + 0.075, MAT.steel, { seg: 20 });   // crank pin
      cp.position.x = pin.x; cp.position.y = pin.y;
      crank.add(cp);
      // piston, connecting rod, valves, spark plugs, gas
      const piston = new THREE.Group();
      root.add(piston);
      piston.add(cylZ(0.196, -0.08, CROWN, MAT.alu, { seg: 32 }));
      for (const z of [0.045, 0.065]) piston.add(cylZ(0.199, z - 0.005, z + 0.005, MAT.steelBlack, { seg: 32 }));   // rings
      const rod = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.045, 1, 12), MAT.steel));
      root.add(rod);
      const valves = [-0.08, 0.08].map((dx, k) => {
        const vg = new THREE.Group();
        vg.position.set(x + dx, -0.06, 0);
        const head = cylZ(0.06, -0.012, 0.012, k ? MAT.steelDark : MAT.steel, { seg: 20 });
        const stem = cylZ(0.015, 0, 0.3 * s, MAT.steel, { seg: 8 });
        const tag = cylZ(0.03, 0.2 * s - 0.01, 0.2 * s + 0.01, new THREE.MeshStandardMaterial({ color: k ? 0xd0453a : 0x3a8ad0 }), { seg: 12 });
        vg.add(head, stem, tag);
        root.add(vg);
        return vg;                               // k = 0 intake (blue tag), 1 exhaust (red tag)
      });
      const plugs = [-0.13, 0.13].map((dx) => {
        const p = cylY(0.022, -0.32, -0.12, MAT.steelBlack, 10);
        p.position.set(x + dx * 0.0 + dx, 0, s * (HEAD_FACE + 0.05));
        root.add(p);
        const spark = new THREE.Mesh(new THREE.SphereGeometry(0.04, 12, 8), new THREE.MeshBasicMaterial({ color: 0xbfe6ff, transparent: true, opacity: 0 }));
        spark.position.set(x + dx, -0.11, s * (HEAD_FACE - 0.01));
        spark.userData.overlay = true;
        root.add(spark);
        return spark;
      });
      const gm = gasMat(0x7fc8ff);
      const gas = new THREE.Mesh(new THREE.CylinderGeometry(0.19, 0.19, 1, 28), gm);
      gas.rotation.x = Math.PI / 2;
      gas.userData.overlay = true;
      root.add(gas);
      cyls.push({ num, s, x, pinObj, piston, rod, valves, plugs, gas, gm });
    }
  }
  shadow(eng);
  const engCut = new THREE.Plane(V(0, -1, 0), 0.0);    // keep y < 0: the crank plane is cut open
  function setEngineCutaway(on) {
    eng.traverse((o) => {
      if (!o.isMesh) return;
      if (!o.userData.cutMat) {
        o.userData.cutMat = o.material.clone();
        Object.assign(o.userData.cutMat, { clippingPlanes: [engCut], clipShadows: true, side: THREE.DoubleSide });
        o.userData.fullMat = o.material;
      }
      o.material = on ? o.userData.cutMat : o.userData.fullMat;
    });
  }
  setEngineCutaway(true);
  const COLD = new THREE.Color(0x7fc8ff), HOT = new THREE.Color(0xff6a1a), SMOKE = new THREE.Color(0x8a7a70);
  const lift = (psi, open, close) => (psi > open && psi < close ? Math.sin(Math.PI * (psi - open) / (close - open)) : 0);
  function updateEngine() {
    const crankDeg = THREE.MathUtils.radToDeg(sgn() * (lo.angle - (lo.phase || 0)));
    root.updateMatrixWorld(true);
    let firing = null;
    for (const c of cyls) {
      const P = root.worldToLocal(c.pinObj.getWorldPosition(V(0, 0, 0)));
      const C = V(c.x, 0, 0), u = V(0, 0, c.s);
      const d = P.clone().sub(C), a = d.dot(u), perp2 = d.lengthSq() - a * a;
      const pinDist = a + Math.sqrt(Math.max(ROD * ROD - perp2, 0));
      const pp = C.clone().addScaledVector(u, pinDist);
      c.piston.position.copy(pp);
      c.piston.quaternion.setFromUnitVectors(V(0, 0, 1), u);
      const rd = pp.clone().sub(P);
      c.rod.position.copy(P).addScaledVector(rd, 0.5);
      c.rod.quaternion.setFromUnitVectors(Yax, rd.clone().normalize());
      c.rod.scale.y = rd.length();
      // 4-stroke phase: 0-180 power, 180-360 exhaust, 360-540 intake, 540-720 compression
      const psi = ((((crankDeg - FIRE[c.num]) % 720) + 720) % 720);
      const ex = lift(psi, 135, 375) || lift(psi + 720, 135, 375);
      const inl = lift(psi, 345, 590);
      c.valves[0].position.z = c.s * (HEAD_FACE - 0.012 - 0.05 * inl);
      c.valves[1].position.z = c.s * (HEAD_FACE - 0.012 - 0.05 * ex);
      const sparkOn = psi > 695 || psi < 2;
      for (const sp of c.plugs) sp.material.opacity = sparkOn ? 1 : 0;
      // gas between piston crown and head
      const crown = pinDist + CROWN, len = Math.max(HEAD_FACE - crown, 0.01);
      c.gas.position.set(c.x, 0, c.s * (crown + len / 2));
      c.gas.scale.set(1, len, 1);
      let col, op = 0.35, glow = 0;
      if (psi < 180) { const k = Math.exp(-psi / 70); col = SMOKE.clone().lerp(HOT, k); glow = 2.2 * k; op = 0.45 + 0.3 * k; if (psi < 25) firing = c.num; }
      else if (psi < 360) { col = SMOKE.clone(); op = 0.35; }
      else if (psi < 540) { col = COLD.clone(); op = 0.3; }
      else { col = COLD.clone().lerp(new THREE.Color(0xffffff), (psi - 540) / 180 * 0.6); op = 0.3 + (psi - 540) / 180 * 0.2; }
      c.gm.color.copy(col); c.gm.emissive.copy(col); c.gm.emissiveIntensity = glow; c.gm.opacity = op;
    }
    st.firing = firing ?? st.firing;
    st.crankDeg = ((crankDeg % 720) + 720) % 720;
  }
  // clutch actuator between the sheaves (raises the upper sheave to tension the belts)
  root.add(box(0.3, 0.3, 0.42, V(-0.12, 0, 0), MAT.paint));                     // lower bearing block
  root.add(box(0.3, 0.3, 0.42, V(-0.12, SH_Y, 0), MAT.paint));                  // upper bearing block
  root.add(box(0.12, 0.12, 1.5, V(-0.12, SH_Y, 0), MAT.paint));                 // side arms to the frame
  bar(root, V(-0.12, 0.15, 0.05), V(-0.12, 1.1, 0.05), 0.07, MAT.steel);         // actuator body
  bar(root, V(-0.12, 1.1, 0.05), V(-0.12, SH_Y - 0.15, 0.05), 0.035, MAT.alu);  // actuator screw
  // vee-belts (4 double belts) with moving index marks
  // open-belt path: over the TOP of the upper sheave, straight down, under the BOTTOM of the
  // lower sheave, straight up. Both tangent points sit `a` below horizontal (upper sheave larger).
  const beltPath = (() => {
    const pts = [];
    const a = Math.asin((R_UP - R_LO) / SH_Y);
    const N = 60;
    for (let i = 0; i <= N; i++) {                      // upper arc, left -> over the top -> right
      const t = -(Math.PI / 2 + a) + (Math.PI + 2 * a) * (i / N);
      pts.push(new THREE.Vector2(R_UP * Math.sin(t), SH_Y + R_UP * Math.cos(t)));
    }
    for (let i = 0; i <= N; i++) {                      // lower arc, right -> under the bottom -> left
      const s = (Math.PI / 2 - a) - (Math.PI - 2 * a) * (i / N);
      pts.push(new THREE.Vector2(R_LO * Math.sin(s), -R_LO * Math.cos(s)));
    }
    return pts; // (z, y) pairs around the loop; the closed curve adds both straight runs
  })();
  const belts = [];
  for (let b = 0; b < 4; b++) {
    const x = 0.3 - 0.165 + b * 0.11;
    const curve = new THREE.CatmullRomCurve3(beltPath.map((p) => V(x, p.y, p.x)), true);
    root.add(shadow(new THREE.Mesh(new THREE.TubeGeometry(curve, 240, 0.04, 6, true), MAT.engineDark)));
    const marks = new THREE.InstancedMesh(new THREE.BoxGeometry(0.1, 0.02, 0.02), MAT.gold, 16);
    root.add(marks);
    belts.push({ curve, marks, len: curve.getLength() });
  }
  let beltS = 0;

  // tail gearbox (90° spiral bevel, ratio ASSUMED to match the published tail rotor speed)
  const OT = V(TGB_X, SH_Y, 0);
  const tg = bevelPair(root, OT, V(1, 0, 0), V(0, 0, 1), { z1: R44_DATA.TAIL.input, z2: R44_DATA.TAIL.output, m: 0.03, faceWidth: 0.2, rim1: 0.1, rim2: 0.1, names: ['R44 tail gearbox input', 'R44 tail gearbox output'] });
  tg.g1.k = -pin.k; tg.g1.phase = 0;      // same shaft as the tail drive shaft, frame axis reversed
  tg.solve();
  rotors.push(tg.g1, tg.g2);
  tg.g1.add(cylZ(0.08, 0.02, -0.35, MAT.steel));
  const tgCase = support(shadow(new THREE.Mesh(new THREE.SphereGeometry(0.75, 32, 20), ghost(0.1))));
  tgCase.position.copy(OT);
  root.add(tgCase);

  // ================= airframe (ghost) + steel tube frame =================
  root.add(support(box(5.0, 0.12, 2.3, V(4.3, -1.0, 0), ghost(0.09))));                       // cabin floor
  const cone = (() => {
    const fwd = V(-0.9, SH_Y - 0.55, 0), aft = V(TGB_X + 0.3, SH_Y - 0.1, 0);
    const d = aft.clone().sub(fwd);
    const m = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.75, d.length(), 32), ghost(0.06));
    m.position.copy(fwd).addScaledVector(d, 0.5);
    m.quaternion.setFromUnitVectors(Yax, d.normalize());
    return support(m);
  })();
  root.add(cone);
  // welded steel-tube frame: gearbox mounts -> engine mounts / cabin
  const FR = [
    [V(2.1, 1.25, 0.55), V(-0.3, -0.3, 0.55)], [V(2.1, 1.25, -0.55), V(-0.3, -0.3, -0.55)],
    [V(3.1, 1.25, 0.55), V(3.6, -0.95, 0.85)], [V(3.1, 1.25, -0.55), V(3.6, -0.95, -0.85)],
    [V(2.1, 1.25, 0.55), V(3.1, 1.25, 0.55)], [V(2.1, 1.25, -0.55), V(3.1, 1.25, -0.55)],
    [V(-0.3, -0.3, 0.55), V(-3.1, -0.38, 0.45)], [V(-0.3, -0.3, -0.55), V(-3.1, -0.38, -0.45)],
    // (upper-sheave frame members start from side arms OUTSIDE the sheave + belt radius)
    [V(-0.3, -0.3, 0.55), V(-0.3, -0.3, -0.55)], [V(-0.12, SH_Y, 0.72), V(2.1, 1.25, 0.55)], [V(-0.12, SH_Y, -0.72), V(2.1, 1.25, -0.55)],
    [V(-0.3, -0.3, 0.55), V(3.6, -0.95, 0.85)], [V(-0.3, -0.3, -0.55), V(3.6, -0.95, -0.85)],
  ];
  for (const [a, b] of FR) bar(root, a, b, 0.045, MAT.paintLight);
  // main gearbox housing on 4 rubber mounts
  const mgCase = support(shadow(new THREE.Mesh(new THREE.SphereGeometry(1.05, 40, 24), ghost(0.1))));
  mgCase.position.copy(gearC).add(V(0, 0.15, 0));
  mgCase.scale.set(1, 0.75, 1);
  root.add(mgCase);
  for (const [x, z] of [[2.1, 0.55], [3.1, 0.55], [2.1, -0.55], [3.1, -0.55]]) {
    const m = cylY(0.09, 1.25, 1.45, MAT.steelBlack, 16);
    m.position.x = x; m.position.z = z;
    root.add(m);
  }

  // ================= main rotor: teetering hub, coning hinges, pitch housings =================
  const hub = new THREE.Group();                // rotates with the mast
  hub.position.set(gearC.x, MAST_TOP, gearC.z);
  root.add(hub);
  hub.add(cylY(0.17, -0.2, 0.12, MAT.steel));                              // mast top / teeter block
  const teeter = new THREE.Group();             // rotates about local Z (teeter hinge)
  hub.add(teeter);
  {
    const bolt = cylZ(0.05, -0.28, 0.28, MAT.steel);
    teeter.add(bolt);
    for (const s of [-1, 1]) { const n = hexNut(0.07, 0.05); n.rotation.x = Math.PI / 2; n.position.z = s * 0.29; teeter.add(n); }
    // underslung yoke below the teeter hinge, carrying the two coning hinges
    teeter.add(box(0.9, 0.12, 0.34, V(0, -0.16, 0), MAT.paint));
    for (const s of [-1, 1]) teeter.add(box(0.12, 0.22, 0.05, V(0, -0.06, s * 0.2), MAT.paint));
  }
  const HING = 0.42, HORN = { x: 0.42, y: -0.18, z: -0.36 };
  const HORN_R = Math.hypot(HING + HORN.x, -HORN.z);
  const HORN_LEAD = Math.atan2(-HORN.z, HING + HORN.x);
  const hornH = (t) => HORN.y * Math.cos(t) - HORN.z * Math.sin(t);
  const HA = -HORN.z, HB = HORN.y, HR = Math.hypot(HA, HB), H0 = Math.atan2(HB, HA);
  const pitchFromHornH = (h) => Math.asin(THREE.MathUtils.clamp(h / HR, -1, 1)) - H0;
  const ROTOR_R = 10.7, CHORD = 0.55;
  const blades = [0, 1].map((i) => {
    const arm = new THREE.Group();
    arm.rotation.y = i * Math.PI;
    teeter.add(arm);
    const coneG = new THREE.Group();            // coning hinge (about local Z)
    coneG.position.set(HING, -0.16, 0);
    arm.add(coneG);
    const hb = cylZ(0.05, -0.2, 0.2, MAT.steel); coneG.add(hb);                       // coning hinge bolt
    const pitchG = new THREE.Group();           // blade + pitch-bearing housing (turns in pitch)
    coneG.add(pitchG);
    pitchG.add(latheX([[0.0, 0.0], [0.14, 0.02], [0.16, 0.12], [0.16, 0.75], [0.12, 0.85], [0.0, 0.85]], MAT.paintLight, 32));   // oil-filled housing
    pitchG.add(latheX([[0.12, 0.6], [0.15, 0.62], [0.15, 0.7], [0.12, 0.72]], MAT.steelBlack, 24));                    // elastomeric boot
    // airfoil helper builds chord on ±Y (LE at +Y for leSign +1); rotating -90° about X puts the
    // chord on Z with the leading edge at -Z (the direction of rotation) and thickness vertical
    const bm = shadow(new THREE.Mesh(airfoilBlade(0.8, ROTOR_R - HING, CHORD, 1, { thick: 0.12 }), BLADE_MAT));
    bm.rotation.x = -Math.PI / 2;
    pitchG.add(bm);
    const hornTip = V(HORN.x, HORN.y, HORN.z);
    bar(pitchG, V(HORN.x, 0.0, -0.1), V(HORN.x, HORN.y + 0.12, HORN.z), 0.035, MAT.paint);
    const jHorn = new Joint(pitchG, root, hornTip, V(1, 0, 0), V(0, -1, 0), SMALL);
    return { phi: i * Math.PI, coneG, pitchG, jHorn, pitch: 0 };
  });

  // ================= swashplate + 3 servos =================
  const SW_Y = 4.55, DECK_Y = 2.72;
  const stat = new THREE.Group(), rot = new THREE.Group();
  root.add(stat, rot);
  stat.add(ringY(0.7, 0.3, -0.12, -0.03, MAT.paint));
  stat.add(ringY(0.62, 0.32, -0.03, 0.0, MAT.gold, 48));
  rot.add(ringY(0.78, 0.3, 0.0, 0.09, MAT.paint));
  const uniball = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.33, 28, 18), MAT.steel));
  root.add(uniball);
  const SERVO_AZ = [0, deg(120), deg(240)];
  const jServoTop = SERVO_AZ.map((a) => {
    const ear = box(0.24, 0.08, 0.2, V(0, 0, 0), MAT.paint);
    ear.position.copy(azDir(a).multiplyScalar(0.72)).setY(-0.1);
    ear.rotation.y = a;
    stat.add(ear);
    return new Joint(stat, root, azDir(a).multiplyScalar(0.78).setY(-0.32), tang(a), V(0, -1, 0), SMALL);
  });
  const lugs = blades.map((b) => {
    const a = b.phi + HORN_LEAD;
    const arm = box(0.4, 0.08, 0.16, V(0, 0, 0), MAT.paint);
    arm.position.copy(azDir(a).multiplyScalar(HORN_R - 0.22)).setY(0.045);
    arm.rotation.y = a;
    rot.add(arm);
    return new Joint(rot, root, azDir(a).multiplyScalar(HORN_R).setY(0.26), azDir(a), V(0, 1, 0), SMALL);
  });
  const LUG_Y = 0.26;
  // gearbox top deck carrying the servos
  const deck = ringY(1.0, 0.18, DECK_Y - 0.08, DECK_Y, MAT.gold, 64);
  deck.position.set(gearC.x, 0, gearC.z);
  root.add(deck);
  {
    const mastHousing = cylY(0.16, gearC.y + 0.1, DECK_Y, MAT.steelDark, 32);
    mastHousing.position.x = gearC.x; mastHousing.position.z = gearC.z;
    root.add(mastHousing);
  }
  const servos = SERVO_AZ.map((a, k) => {
    const p = azDir(a).multiplyScalar(0.78).add(V(gearC.x, DECK_Y + 0.2, gearC.z));
    const mount = box(0.3, 0.1, 0.3, p.clone().setY(DECK_Y + 0.05), MAT.paint);
    root.add(mount);
    const jBase = new Joint(root, root, p, tang(a), V(0, 1, 0), SMALL);
    const sv = new Servo(root, jBase.p(), a);
    return { a, sv, jBase, jTop: jServoTop[k] };
  });
  const pitchLinks = blades.map(() => new Link(root, { r: 0.032, eye: 0.06 }));
  // scissors (rotating: mast collar -> rotating plate; stationary: deck -> stationary plate)
  const RS_AZ = deg(90) + HORN_LEAD;
  hub.add(cylY(0.2, -0.75, -0.2, MAT.steel, 32));
  const jCollar = new Joint(hub, root, azDir(RS_AZ).multiplyScalar(0.32).setY(-0.6), tang(RS_AZ), azDir(RS_AZ).multiplyScalar(0.6).add(V(0, -1, 0)), SMALL);
  const jRotSw = new Joint(rot, root, azDir(RS_AZ).multiplyScalar(0.66).setY(0.25), tang(RS_AZ), V(0, 1, 0), SMALL);
  const SS_AZ = deg(180);
  const jDeckSc = new Joint(root, root, azDir(SS_AZ).multiplyScalar(0.7).add(V(gearC.x, DECK_Y + 0.2, gearC.z)), tang(SS_AZ), V(0, 1, 0), SMALL);
  root.add(box(0.26, 0.1, 0.26, azDir(SS_AZ).multiplyScalar(0.7).add(V(gearC.x, DECK_Y + 0.05, gearC.z)), MAT.paint));
  const jStatSw = new Joint(stat, root, azDir(SS_AZ).multiplyScalar(0.72).setY(-0.3), tang(SS_AZ), V(0, -1, 0), SMALL);
  const scL = () => new Link(root, { r: 0.04, eye: 0.06, mat: MAT.paint, turnbuckle: false });
  const rs = { a: scL(), b: scL() }, ss = { a: scL(), b: scL() };
  const tpp = new THREE.Mesh(new THREE.TorusGeometry(ROTOR_R - 0.4, 0.04, 8, 160), new THREE.MeshBasicMaterial({ color: 0xf0a83a, transparent: true, opacity: 0.55, depthWrite: false }));
  tpp.rotation.x = Math.PI / 2;
  tpp.userData.overlay = true;
  const tppG = new THREE.Group();
  tppG.add(tpp);
  root.add(tppG);

  // ================= tail rotor: 2-blade teetering, fixed coning, pitch control slider =================
  const TR_R = 1.57;
  const tr = new Rotor(root, OT.clone().add(V(0, 0, -0.62)), V(0, 0, -1), { name: 'R44 tail rotor' });
  tr.k = tg.g2.k;
  rotors.push(tr);
  tr.add(cylZ(0.07, -0.55, 0.12, MAT.steel));
  const trTeeter = new THREE.Group();
  tr.add(trTeeter);
  trTeeter.add(box(0.36, 0.12, 0.1, V(0, 0, 0.02), MAT.paint));
  const spinSign = Math.sign(tr.k) || 1;
  const le = spinSign;
  const trBlades = [0, 1].map((i) => {
    const arm = new THREE.Group();
    arm.rotation.z = i * Math.PI;
    trTeeter.add(arm);
    const coneG = new THREE.Group();
    coneG.rotation.y = deg(1.5);              // fixed coning angle (POH)
    arm.add(coneG);
    const pg = new THREE.Group();
    coneG.add(pg);
    pg.add(latheX([[0.0, 0.12], [0.06, 0.13], [0.06, 0.28], [0.0, 0.3]], MAT.paintLight, 20));
    pg.add(shadow(new THREE.Mesh(airfoilBlade(0.28, TR_R, 0.19, le, { thick: 0.1 }), BLADE_MAT)));
    const hornTip = V(0.24, le * 0.13, -0.16);                          // horn toward the slider (inboard side)
    bar(pg, V(0.24, le * 0.03, -0.02), hornTip.clone().add(V(0, 0, 0.09)), 0.02, MAT.paint);
    const jH = new Joint(pg, root, hornTip, V(1, 0, 0), V(0, 0, -1), { gap: 0.06, l: 0.08, h: 0.08, t: 0.015 });
    return { pg, jH, hornTip };
  });
  const slider = new THREE.Group();            // pitch control slider on the output shaft (turns with it)
  tr.add(slider);
  slider.add(cylZ(0.1, -0.03, 0.03, MAT.steel));
  const sliderJ = trBlades.map((b, i) => {
    const a = i * Math.PI;
    const tip = V(Math.cos(a) * 0.24 - Math.sin(a) * le * 0.13, Math.sin(a) * 0.24 + Math.cos(a) * le * 0.13, 0);
    bar(slider, V(0, 0, 0), tip.clone().multiplyScalar(0.7), 0.02, MAT.paint);
    return new Joint(slider, root, tip, V(Math.cos(a), Math.sin(a), 0), V(0, 0, 1), { gap: 0.06, l: 0.08, h: 0.08, t: 0.015 });
  });
  const trLinks = trBlades.map(() => new Link(root, { r: 0.018, eye: 0.03, turnbuckle: false }));
  const trHornZ = (th) => 0.13 * Math.sin(th) - 0.16 * Math.cos(th);

  // ================= cockpit controls (centre T-bar cyclic, collective with throttle, pedals) =================
  const FLOOR = -0.94;
  root.add(box(2.6, 0.03, 2.0, V(5.0, FLOOR - 0.015, 0), MAT.paintLight));
  const cycP = V(5.4, FLOOR + 0.1, 0);
  root.add(box(0.3, 0.1, 0.3, cycP.clone().setY(FLOOR + 0.05), MAT.steelDark));
  const post = new THREE.Group();
  post.position.copy(cycP);
  root.add(post);
  const GRIP_H = 1.05, GRIP_Z = 0.42;           // pilot (right seat) grip on the T-bar
  {
    const tube = bar(post, V(0, -0.35, 0), V(0, GRIP_H - 0.1, 0), 0.03, MAT.steelDark);
    const cross = bar(post, V(0, GRIP_H - 0.1, -GRIP_Z), V(0, GRIP_H - 0.1, GRIP_Z), 0.028, MAT.steelDark);
    const g1 = shadow(new THREE.Mesh(new THREE.CapsuleGeometry(0.05, 0.16, 6, 14), MAT.steelBlack));
    g1.position.set(0, GRIP_H, GRIP_Z);
    post.add(g1);
    const g2 = g1.clone(); g2.position.z = -GRIP_Z; post.add(g2);
    for (const m of [tube, cross, g1, g2]) { m.userData.ctrl = 'cyclic'; m.userData.pivot = cycP; m.userData.gripZ = GRIP_Z; pick.push(m); }
  }
  const jCyLon = new Joint(post, root, V(0, -0.35, -0.08), V(0, 0, 1), V(-1, 0, 0), SMALL);
  const jCyLat = new Joint(post, root, V(0, -0.35, 0.1), V(1, 0, 0), V(0, 0, 1), SMALL);
  const colP = V(4.65, FLOOR + 0.22, 0.1);
  root.add(box(0.22, 0.24, 0.14, colP.clone().setY(FLOOR + 0.12), MAT.steelDark));
  const coll = new THREE.Group();
  coll.position.copy(colP);
  root.add(coll);
  const twist = new THREE.Group();
  {
    const lev = bar(coll, V(0, 0, 0), V(1.0, 0, 0), 0.03, MAT.steelDark);
    twist.position.set(0.85, 0, 0);
    coll.add(twist);
    const tg2 = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.26, 18), MAT.steelBlack));
    tg2.rotation.z = Math.PI / 2;
    twist.add(tg2);
    twist.add(box(0.03, 0.02, 0.06, V(0, 0.042, 0), MAT.gold));         // throttle index set into the grip (shows rotation)
    bar(coll, V(0, 0, 0), V(0, -0.38, 0), 0.028, MAT.steelDark);       // horn
    for (const m of [lev, tg2]) { m.userData.ctrl = 'collective'; m.userData.pivot = colP; pick.push(m); }
  }
  const jCol = new Joint(coll, root, V(0, -0.49, 0), V(0, 0, 1), V(0, -1, 0), SMALL);
  const pedP = V(6.25, FLOOR + 0.06, 0.42);
  root.add(box(0.16, 0.12, 0.16, pedP.clone().setY(FLOOR + 0.06), MAT.steelDark));
  const rocker = new THREE.Group();
  rocker.position.copy(pedP);
  root.add(rocker);
  {
    bar(rocker, V(0, 0.05, -0.3), V(0, 0.05, 0.48), 0.03, MAT.steelDark);
    for (const s of [-1, 1]) {
      const z = s * 0.24;
      bar(rocker, V(0, 0.05, z), V(0.04, 0.3, z), 0.024, MAT.steelDark);
      const ped = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.3, 0.18), MAT.steelBlack));
      ped.position.set(0.05, 0.34, z);
      ped.rotation.z = -0.35;            // top leans forward (away from the pilot)
      ped.userData.ctrl = s < 0 ? 'pedalL' : 'pedalR';
      rocker.add(ped);
      pick.push(ped);
    }
  }
  const jPed = new Joint(rocker, root, V(-0.1, 0.05, 0.42), V(0, 1, 0), V(-1, 0, 0), SMALL);

  // under-floor jackshaft bank (inputs from the controls; outputs to the push-pull tubes below
  // each servo). The geometric mixing inside it is not drawn: outputs follow the servo commands.
  const JS = V(3.45, FLOOR - 0.45, 0);
  root.add(support(box(1.4, 0.55, 2.0, V(3.6, FLOOR - 0.35, 0), ghost(0.08))));
  root.add(box(0.5, 0.3, 1.2, JS.clone(), MAT.paint));
  const IN_Z = { lon: -0.15, lat: 0.15, coll: -0.4, yaw: 0.45 };
  const inLever = {};
  for (const [k, z] of Object.entries(IN_Z)) {
    inLever[k] = new Lever(root, root, V(JS.x + 0.32, JS.y, z), V(0, 0, 1), [{ dir: V(0, -1, 0), len: 0.22 }]);
    root.add(box(0.08, 0.1, 0.08, V(JS.x + 0.27, JS.y, z), MAT.steelDark));
  }
  const latBC = new Lever(root, root, V(5.4, FLOOR - 0.3, 0.6), Yax, [{ dir: V(1, 0, 0), len: 0.22 }, { dir: V(0, 0, -1), len: 0.22 }]);
  {
    const latPost = cylY(0.05, FLOOR - 0.36, FLOOR, MAT.steelDark, 12);   // lateral bellcrank pivot post
    latPost.position.x = 5.4; latPost.position.z = 0.6;
    root.add(latPost);
  }
  const outLever = servos.map((s) => {
    const p = azDir(s.a).multiplyScalar(1.1).add(V(gearC.x, FLOOR - 0.3, gearC.z));
    root.add(box(0.1, 0.1, 0.1, p.clone().add(V(0.06, 0, 0)), MAT.steelDark));
    return new Lever(root, root, p, V(0, 0, 1), [{ dir: V(-1, 0, 0), len: 0.3 }]);
  });
  // tail rotor push-pull tube (pedals connect directly to the tail rotor pitch control)
  const yawBC = new Lever(root, root, V(JS.x - 0.4, FLOOR - 0.3, 0.45), V(0, 0, 1), [{ dir: V(0, -1, 0), len: 0.2 }, { dir: V(0, 1, 0), len: 0.2 }]);
  const tailBC = new Lever(root, root, OT.clone().add(V(0.55, -0.45, -0.35)), V(0, 0, 1), [{ dir: V(0, 1, 0), len: 0.2 }, { dir: V(-1, 0, 0), len: 0.18 }]);
  // pilot-valve lever on a follow-up bracket that moves with the servo piston: in steady state the
  // input tube moves exactly as far as the piston and the valve returns to centre (irreversible servo)
  const follow = servos.map((s) => {
    const f = new THREE.Group();
    s.sv.g.add(f);
    f.add(box(0.05, 0.6, 0.04, V(0, 0.92, 0.3), MAT.steelDark));         // follow-up link along the body
    return f;
  });
  // lever arm horizontal (outward), so its tip moves up/down along the near-vertical input tube
  const valve = servos.map((s, i) => new Lever(follow[i], root, V(0, 0.92, 0.3), V(1, 0, 0), [{ dir: V(0, 0, 1), len: 0.2 }], { hub: 0.04 }));
  const rodL = (o = {}) => new Link(root, { r: 0.024, eye: 0.04, ...o });
  const RR = {
    lon: rodL(), lat1: rodL({ turnbuckle: false }), lat2: rodL(), coll: rodL(), yaw: rodL(),
    up: servos.map(() => rodL()), tail: rodL({ r: 0.03 }), tslide: rodL({ turnbuckle: false }),
  };
  const jSlideIn = new Joint(root, root, OT.clone().add(V(0.37, -0.45, -0.62)), V(0, 1, 0), V(1, 0, 0), SMALL);   // slider control rod end (non-rotating)
  const slideRod = bar(root, OT.clone().add(V(0.0, -0.45, -0.62)), OT.clone().add(V(0.26, -0.45, -0.62)), 0.03, MAT.alu);

  // ================= neutral lengths =================
  root.updateMatrixWorld(true);
  const D = (a, b) => a.distanceTo(b);
  const L = {
    lon: D(jCyLon.p(), inLever.lon.tipAt(0, 0)), lat1: D(jCyLat.p(), latBC.tipAt(0, 0)), lat2: D(latBC.tipAt(1, 0), inLever.lat.tipAt(0, 0)),
    coll: D(jCol.p(), inLever.coll.tipAt(0, 0)), yaw: D(jPed.p(), yawBC.tipAt(0, 0)), tail: D(yawBC.tipAt(1, 0), tailBC.tipAt(0, 0)),
  };
  let L_up = null, servoL0 = null, tailBC0 = 0;

  anchors.r44engine = V(-2.2, 0.75, -1.2);
  for (const c of cyls) anchors[`r44e_c${c.num}`] = V(c.x, 0.32, c.s * 1.05);
  anchors.r44e_crank = V(-2.95, 0.25, 0.0);
  anchors.r44e_piston = V(-1.45, 0.3, 0.55);
  anchors.r44e_valves = V(-2.07, 0.05, -0.95);
  anchors.r44e_plugs = V(-2.69, -0.45, 0.85);
  anchors.r44belts = V(0.3, 1.0, 0.55);
  anchors.r44clutch = V(-0.12, SH_Y + 0.45, -0.3);
  anchors.r44gearbox = gearC.clone().add(V(0.7, 0.9, 0.7));
  anchors.r44hub = V(gearC.x, MAST_TOP + 0.55, 0.3);
  anchors.r44swash = V(gearC.x + 0.9, SW_Y + 0.1, 0.3);
  anchors.r44servo = V(gearC.x - 0.6, DECK_Y + 1.1, 0.9);
  anchors.r44tail = OT.clone().add(V(0.2, 0.9, -0.8));
  anchors.r44shaft = V(-6.5, SH_Y + 0.35, 0);
  anchors.r44cyclic = cycP.clone().add(V(0, GRIP_H + 0.35, 0.3));
  anchors.r44collective = colP.clone().add(V(0.8, 0.3, -0.25));
  anchors.r44pedals = pedP.clone().add(V(0.15, 0.6, 0.1));

  // ================= runtime =================
  for (const r of rotors) { r.angle = r.phase; r.spinner.rotation.z = r.phase; }   // start meshed (solved tooth phase)
  const st = {
    controls, bladePitch: [0, 0], thetaColl: 0, cyclicAmp: 0, stickAz: 0, swashLowAz: 0, rotorAngle: 0,
    tailPitch: 0, discTilt: new THREE.Vector2(), coning: 0, throttle: 0, teeter: 0, trPitch: 0,
    firing: 1, crankDeg: 0,
  };
  const COLL_MIN = deg(-1), COLL_MAX = deg(14), CYC_MAX = deg(8);
  const hornH0 = hornH(0);

  function update(dTheta, dt) {
    const c = controls;
    for (const r of rotors) { r.angle += r.k * dTheta; r.spinner.rotation.z = r.angle; }
    updateEngine();
    const theta = mg.g2.angle - mg.g2.phase;                         // mast angle
    st.rotorAngle = theta;
    hub.rotation.y = theta;
    // belts: index marks travel at the sheave rim speed
    beltS += pin.k * dTheta * R_UP;
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
    for (const b of belts) {
      for (let i = 0; i < 16; i++) {
        const t = (((i / 16) + beltS / b.len) % 1 + 1) % 1;
        const p = b.curve.getPointAt(t), tg2 = b.curve.getTangentAt(t);
        q.setFromUnitVectors(V(0, 0, 1), tg2);
        m4.compose(p, q, V(1, 1, 1));
        b.marks.setMatrixAt(i, m4);
      }
      b.marks.instanceMatrix.needsUpdate = true;
    }
    // swashplate (no mixing unit on the R44: stick and collective only)
    st.thetaColl = COLL_MIN + (COLL_MAX - COLL_MIN) * c.collective;
    const mag = Math.min(Math.hypot(c.lon, c.lat), 1);
    st.cyclicAmp = CYC_MAX * mag;
    st.stickAz = Math.atan2(-c.lat, c.lon);
    st.swashLowAz = st.stickAz - Math.PI / 2 + HORN_LEAD;             // teetering rotor: 90° phase
    const swH = SW_Y + (hornH(st.thetaColl) - hornH0);
    const dh = (hornH(st.thetaColl + st.cyclicAmp) - hornH(st.thetaColl - st.cyclicAmp)) / 2;
    const tilt = Math.atan2(dh, HORN_R);
    const n = azDir(st.swashLowAz).multiplyScalar(Math.sin(tilt)).add(V(0, Math.cos(tilt), 0)).normalize();
    const qT = new THREE.Quaternion().setFromUnitVectors(Yax, n);
    stat.position.set(gearC.x, swH, gearC.z); stat.quaternion.copy(qT);
    rot.position.set(gearC.x, swH, gearC.z); rot.quaternion.copy(qT).multiply(new THREE.Quaternion().setFromAxisAngle(Yax, theta));
    uniball.position.set(gearC.x, swH - 0.04, gearC.z);
    root.updateMatrixWorld(true);
    // disc response (settles in 1/5 rev, at most ~0.5 s screen time)
    const discAz = st.swashLowAz - HORN_LEAD + Math.PI / 2;
    const target = new THREE.Vector2(Math.cos(discAz), Math.sin(discAz)).multiplyScalar(st.cyclicAmp);
    st.discTilt.lerp(target, 1 - Math.exp(-Math.max(Math.abs(dTheta) / (2 * Math.PI * 0.2), dt / 0.12)));
    const dA = st.discTilt.length(), dAz = Math.atan2(st.discTilt.y, st.discTilt.x);
    st.coning = deg(1.0) + deg(2.5) * c.collective;
    // teeter: the whole hub see-saws so blade 1 sits on the tilted disc, blade 2 opposite
    st.teeter = -dA * Math.cos(wrap(theta - dAz));
    teeter.rotation.z = st.teeter;
    blades.forEach((b, i) => {
      const h = lugs[i].p().y - (swH - (SW_Y - SW_Y)) ;
      const hTip = hornH0 + (lugs[i].p().y - (SW_Y + LUG_Y));
      b.pitch = pitchFromHornH(hTip);
      b.coneG.rotation.z = st.coning;
      b.pitchG.rotation.x = b.pitch;
      st.bladePitch[i] = b.pitch;
      void h;
    });
    root.updateMatrixWorld(true);
    blades.forEach((b, i) => pitchLinks[i].set(lugs[i].p(), b.jHorn.p(), lugs[i].ax(), b.jHorn.ax()));
    for (const s of servos) s.sv.set(s.jTop.p(), s.jTop.ax(), s.jBase.ax());
    const sc = (Lk, kb, jA, jB, l, out) => {
      const a = jA.p(), b = jB.p(), kn = knee(a, b, l, out), ax = jA.ax();
      Lk.a.set(a, kn.clone().addScaledVector(ax, 0.045), ax, ax);
      Lk.b.set(kn.clone().addScaledVector(ax, -0.045), b, ax, jB.ax());
    };
    sc(rs, null, jCollar, jRotSw, 0.5, azDir(theta + RS_AZ));
    sc(ss, null, jDeckSc, jStatSw, 1.1, azDir(SS_AZ));
    tppG.position.set(gearC.x, MAST_TOP - 0.16 + (ROTOR_R - 0.4) * Math.sin(st.coning), gearC.z);
    tppG.quaternion.setFromUnitVectors(Yax, azDir(dAz).multiplyScalar(Math.sin(dA)).add(V(0, Math.cos(dA), 0)).normalize());

    // controls
    const m2 = Math.min(Math.hypot(c.lon, c.lat), 1);
    if (m2 > 1e-4) post.quaternion.setFromAxisAngle(V(c.lat, 0, -c.lon).normalize(), m2 * deg(12));
    else post.quaternion.identity();
    coll.quaternion.setFromAxisAngle(V(0, 0, 1), deg(-6) + deg(26) * c.collective);
    st.throttle = 0.25 + 0.7 * c.collective;                          // throttle correlated to collective
    twist.rotation.x = -st.throttle * deg(80);
    rocker.quaternion.setFromAxisAngle(Yax, -c.pedal * deg(16));
    root.updateMatrixWorld(true);
    inLever.lon.solve(0, jCyLon.p(), L.lon);
    latBC.solve(0, jCyLat.p(), L.lat1);
    root.updateMatrixWorld(true);
    inLever.lat.solve(0, latBC.tipAt(1, latBC.u), L.lat2);
    inLever.coll.solve(0, jCol.p(), L.coll);
    yawBC.solve(0, jPed.p(), L.yaw);
    root.updateMatrixWorld(true);
    tailBC.solve(0, yawBC.tipAt(1, yawBC.u), L.tail);
    // servo outputs -> push-pull tubes -> servo input (pilot valve) levers
    const lens = servos.map((s) => s.jTop.p().distanceTo(s.jBase.p()));
    if (!servoL0) servoL0 = lens.slice();
    // servo piston travel = input tube travel: follow-up bracket rides with the piston, and the
    // jackshaft output lever turns so its tip rises by the same amount
    follow.forEach((f, i) => (f.position.y = lens[i] - servoL0[i]));
    outLever.forEach((lv, i) => lv.setAngle(-Math.asin(THREE.MathUtils.clamp((lens[i] - servoL0[i]) / 0.3, -0.95, 0.95))));
    root.updateMatrixWorld(true);
    if (!L_up) { valve.forEach((v) => v.setAngle(0)); root.updateMatrixWorld(true); L_up = outLever.map((lv, i) => lv.tipAt(0, lv.u).distanceTo(valve[i].tipAt(0, 0))); tailBC0 = tailBC.u; }
    valve.forEach((v, i) => v.solve(0, outLever[i].tipAt(0, outLever[i].u), L_up[i]));
    // tail rotor pitch from the pedal push-pull tube (tail bellcrank moves the pitch slider)
    st.tailPitch = c.pedal;
    const th = deg(7) + deg(11) * c.pedal;
    st.trPitch = th;
    const travel = trHornZ(th) - trHornZ(0);
    slider.position.z = -0.16 + travel - 0.0;
    for (const b of trBlades) b.pg.rotation.x = -le * th;               // pusher: thrust toward the tail boom (+Z)
    const slideX = (tailBC.u - tailBC0) * 0.18;
    slideRod.position.x = OT.x + 0.13 - 0 + slideX * 0;
    root.updateMatrixWorld(true);
    const J = (lv, k) => lv.joints[k];
    const set = (link, a, b) => link.set(a.p(), b.p(), a.ax(), b.ax());
    trBlades.forEach((b, i) => set(trLinks[i], sliderJ[i], b.jH));
    set(RR.lon, jCyLon, J(inLever.lon, 0));
    set(RR.lat1, jCyLat, J(latBC, 0));
    set(RR.lat2, J(latBC, 1), J(inLever.lat, 0));
    set(RR.coll, jCol, J(inLever.coll, 0));
    set(RR.yaw, jPed, J(yawBC, 0));
    set(RR.tail, J(yawBC, 1), J(tailBC, 0));
    set(RR.tslide, J(tailBC, 1), jSlideIn);
    RR.up.forEach((link, i) => set(link, J(outLever[i], 0), J(valve[i], 0)));
  }

  const speeds = () => {
    const r = (x) => Math.abs(x.k) * R44_DATA.MAST_RPM;
    return { engine: r(lo), upper: r(pin), mast: r(mg.g2), tailShaft: r(pin), tailRotor: r(tr) };
  };
  const check = () => [
    ['cyclic lon', RR.lon, jCyLon, inLever.lon.joints[0], L.lon],
    ['cyclic lat 1', RR.lat1, jCyLat, latBC.joints[0], L.lat1],
    ['cyclic lat 2', RR.lat2, latBC.joints[1], inLever.lat.joints[0], L.lat2],
    ['collective', RR.coll, jCol, inLever.coll.joints[0], L.coll],
    ['pedals', RR.yaw, jPed, yawBC.joints[0], L.yaw],
    ['tail push-pull tube', RR.tail, yawBC.joints[1], tailBC.joints[0], L.tail],
    ...RR.up.map((l, i) => [`servo ${i + 1} input`, l, outLever[i].joints[0], valve[i].joints[0], L_up && L_up[i]]),
  ];
  return {
    root, anchors, update, speeds, check, state: st, pick, rotors,
    pairs: { main: mg, tail: tg },
    geo: { cycP, colP, pedP, CYC_TILT: deg(12), COLL_U0: deg(-6), COLL_U1: deg(20), gripH: GRIP_H, gripZ: GRIP_Z },
    blades, lugs, hub, teeterGroup: teeter, trBladeGroups: trBlades.map((b) => b.pg), trLe: le,
    engine: { cyls, FIRE, CR, ROD, CROWN, HEAD_FACE, lo, setEngineCutaway },
  };
}
