// Pilot controls and the mechanical control path (UH-60 style, TM 1-1520-237-10 §2.37):
//   cyclic / collective / pedals -> push-pull rods under the floor -> bellcranks -> rods up the
//   control column -> MIXING UNIT -> rods to the pilot valve lever of each primary servo, and
//   cables (via a quadrant) aft to the tail rotor servo.
// Every rod has a FIXED length; each bellcrank / lever angle is solved from the rod driving it.
// (Pilot-assist servos and the copilot's controls are not shown.)
import * as THREE from 'three';
import { MAT, shadow, cylZ } from './parts.js';
import { Joint, Link, frameQuat } from './detail.js';

const V = (x, y, z) => new THREE.Vector3(x, y, z);
const Yax = V(0, 1, 0);
const deg = THREE.MathUtils.degToRad;

const FLOOR_Y = -4.5;     // cockpit floor top
const ROOF_Y = -0.85;     // cabin roof / upper deck top (main gearbox housing sits here)
const COL_X = 4.3;        // control column (behind the cockpit)
const MIX = V(3.7, 3.0, 0.62);   // mixing unit centre
const LANES = { coll: 0.25, lon: 0.5, lat: 0.75, yaw: 1.0 };      // z of each channel in the column
const SMALL = { gap: 0.09, l: 0.11, h: 0.11, t: 0.022 };

function box(sx, sy, sz, p, mat = MAT.paint) {
  const m = shadow(new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat));
  m.position.copy(p);
  return m;
}
function ghostMat(op = 0.08) { const g = MAT.glass.clone(); g.opacity = op; return g; }
function support(mesh) { mesh.userData.support = true; return mesh; }

/** Bar between two points (static or child geometry). */
function bar(parent, a, b, r, mat = MAT.paint) {
  const d = b.clone().sub(a);
  const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, d.length(), 14), mat));
  m.position.copy(a).addScaledVector(d, 0.5);
  m.quaternion.setFromUnitVectors(Yax, d.normalize());
  parent.add(m);
  return m;
}

/**
 * Lever / bellcrank rotating about `axis` through `pivot` (both in parent coordinates).
 * arms: [{ dir, len }] — each arm ends in a Joint (bolt parallel to the pivot axis).
 */
export class Lever {
  constructor(parent, root, pivot, axis, arms, { mat = MAT.paint, hub = 0.06 } = {}) {
    this.parent = parent; this.root = root;
    this.pivot = pivot.clone(); this.axis = axis.clone().normalize();
    this.arms = arms.map((a) => ({ dir: a.dir.clone().normalize(), len: a.len }));
    this.g = new THREE.Group();
    this.g.position.copy(pivot);
    parent.add(this.g);
    const hubM = shadow(new THREE.Mesh(new THREE.CylinderGeometry(hub, hub, 0.12, 20), MAT.steel));
    hubM.quaternion.setFromUnitVectors(Yax, this.axis);
    this.g.add(hubM);
    this.joints = this.arms.map((a) => {
      bar(this.g, V(0, 0, 0), a.dir.clone().multiplyScalar(a.len - SMALL.l), 0.03, mat);
      // fork opens outward (toward the rod); its lugs run back onto the arm
      return new Joint(this.g, root, a.dir.clone().multiplyScalar(a.len), this.axis, a.dir.clone(), SMALL);
    });
    this.u = 0;
  }
  setAngle(u) { this.u = u; this.g.quaternion.setFromAxisAngle(this.axis, u); }
  /** Root-frame tip of arm k at angle u (no state change). */
  tipAt(k, u) {
    const M = this.root.matrixWorld.clone().invert().multiply(this.parent.matrixWorld);
    const p = this.pivot.clone().applyMatrix4(M);
    const R = new THREE.Matrix3().setFromMatrix4(M);
    const ax = this.axis.clone().applyMatrix3(R).normalize();
    const d = this.arms[k].dir.clone().applyMatrix3(R).normalize();
    const e = ax.clone().cross(d);
    return p.addScaledVector(d, this.arms[k].len * Math.cos(u)).addScaledVector(e, this.arms[k].len * Math.sin(u));
  }
  /** Find the angle where arm k's tip is exactly L from point A (rod of fixed length). */
  solve(k, A, L) {
    const f = (u) => this.tipAt(k, u).distanceTo(A) - L;
    let u = this.u;
    for (let i = 0; i < 12; i++) {
      const h = 1e-4, fu = f(u), d = (f(u + h) - f(u - h)) / (2 * h);
      if (Math.abs(d) < 1e-9) break;
      const step = THREE.MathUtils.clamp(fu / d, -0.3, 0.3);
      u -= step;
      if (Math.abs(step) < 1e-7) break;
    }
    this.setAngle(u);
    return u;
  }
}

export function buildCockpit(root, { rh, tail }) {
  const anchors = {};
  const ghost = ghostMat(0.07);

  // ---------------- airframe ghosts (supports) ----------------
  // cockpit / cabin floor structure (rods and bellcranks run inside it)
  root.add(support(box(7.6, 0.9, 4.2, V(6.1, FLOOR_Y - 0.45, 0.2), ghost)));
  const floor = box(3.6, 0.04, 3.4, V(7.3, FLOOR_Y - 0.02, -0.05), MAT.paintLight);
  floor.material = MAT.paintLight.clone(); floor.material.color.set(0x4a5560);
  root.add(floor);
  // cabin roof / upper deck: the main gearbox housing and engine mounts sit on it
  root.add(support(box(11.0, 0.1, 9.4, V(2.0, ROOF_Y - 0.05, 0), ghost)));
  // control column from the floor structure up to the mixing unit
  root.add(support(box(0.6, MIX.y - FLOOR_Y + 0.5, 1.3, V(COL_X, (MIX.y + FLOOR_Y - 0.5) / 2, 0.62), ghostMat(0.1))));

  // ---------------- cyclic stick ----------------
  const cycP = V(6.9, FLOOR_Y + 0.12, 0.9);
  root.add(box(0.34, 0.12, 0.34, cycP.clone().setY(FLOOR_Y + 0.06), MAT.steelDark));      // gimbal housing
  const stick = new THREE.Group();
  stick.position.copy(cycP);
  root.add(stick);
  const pick = [];
  {
    const boot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.16, 0.22, 20), MAT.steelBlack));
    boot.position.y = 0.11;
    stick.add(boot);
    stick.add(shadow(new THREE.Mesh(new THREE.SphereGeometry(0.09, 20, 14), MAT.steel)));     // gimbal ball
    const tube = bar(stick, V(0, -0.42, 0), V(0, 1.25, 0), 0.032, MAT.steelDark);
    const grip = shadow(new THREE.Mesh(new THREE.CapsuleGeometry(0.065, 0.26, 6, 16), MAT.steelBlack));
    grip.position.set(0.02, 1.42, 0);
    grip.rotation.z = -0.15;
    stick.add(grip);
    for (const [x, y, c] of [[0.03, 1.53, 0xd03a2f], [-0.02, 1.5, 0xd8dde3], [0.04, 1.44, 0x3a3f45]]) {   // trim / buttons
      const b = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.03, 10), new THREE.MeshStandardMaterial({ color: c })));
      b.position.set(x, y, 0.06);                                  // set into the side of the grip
      b.rotation.x = Math.PI / 2;
      stick.add(b);
    }
    // lower arm cross-bar carrying the two output bolts
    bar(stick, V(0, -0.42, -0.12), V(0, -0.42, 0.12), 0.03, MAT.steelDark);
    for (const m of [tube, grip]) { m.userData.ctrl = 'cyclic'; pick.push(m); }
  }
  const jCycLon = new Joint(stick, root, V(0, -0.42, -0.07), V(0, 0, 1), V(-1, 0, 0), SMALL);
  const jCycLat = new Joint(stick, root, V(0, -0.42, 0.12), V(1, 0, 0), V(0, 0, 1), SMALL);
  const CYC_TILT = deg(14);

  // ---------------- collective lever ----------------
  const colP = V(6.15, FLOOR_Y + 0.32, LANES.coll + 0.05);
  root.add(box(0.26, 0.32, 0.16, colP.clone().setY(FLOOR_Y + 0.16), MAT.steelDark));      // pivot bracket
  const coll = new THREE.Group();
  coll.position.copy(colP);
  root.add(coll);
  {
    const lev = bar(coll, V(0, 0, 0), V(1.25, 0, 0), 0.035, MAT.steelDark);
    const head = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.13, 0.12), MAT.steelBlack));   // switch box
    head.position.set(1.33, 0.02, 0);
    coll.add(head);
    const throttle = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.32, 18), MAT.steelBlack)); // twist grip
    throttle.rotation.z = Math.PI / 2;
    throttle.position.set(1.0, 0, 0);
    coll.add(throttle);
    bar(coll, V(0, 0, 0), V(0, -0.45, 0), 0.03, MAT.steelDark);                                   // horn
    for (const m of [lev, head, throttle]) { m.userData.ctrl = 'collective'; pick.push(m); }
  }
  const hub = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.2, 18), MAT.steel));
  hub.rotation.x = Math.PI / 2; hub.position.copy(colP); root.add(hub);
  const jColl = new Joint(coll, root, V(0, -0.56, 0), V(0, 0, 1), V(0, -1, 0), SMALL);   // lugs run up onto the horn
  const COLL_U0 = deg(-8), COLL_U1 = deg(22);

  // ---------------- pedals (rocker on a vertical pivot) ----------------
  const pedP = V(8.35, FLOOR_Y + 0.08, 0.9);
  root.add(box(0.2, 0.16, 0.2, pedP.clone().setY(FLOOR_Y + 0.08), MAT.steelDark));
  const rocker = new THREE.Group();
  rocker.position.copy(pedP);
  root.add(rocker);
  {
    bar(rocker, V(0, 0.06, -0.42), V(0, 0.06, 0.68), 0.035, MAT.steelDark);
    for (const s of [-1, 1]) {
      const z = s * 0.36;
      bar(rocker, V(0, 0.06, z), V(0.05, 0.36, z), 0.028, MAT.steelDark);
      const pedal = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.38, 0.24), MAT.steelBlack));
      pedal.position.set(0.06, 0.42, z);
      pedal.rotation.z = -0.35;          // top leans forward (away from the pilot), face toward the pilot
      pedal.userData.ctrl = s < 0 ? 'pedalL' : 'pedalR';
      rocker.add(pedal);
      pick.push(pedal);
    }
  }
  const jPed = new Joint(rocker, root, V(-0.11, 0.06, 0.62), V(0, 1, 0), V(-1, 0, 0), SMALL);   // fork on the rocker end

  // ---------------- copilot controls (left seat), interconnected with the pilot's ----------------
  // TM 2.37: pilot and copilot controls are routed to a combining linkage for each axis.
  const cycP2 = cycP.clone().setZ(-0.9);
  root.add(box(0.34, 0.12, 0.34, cycP2.clone().setY(FLOOR_Y + 0.06), MAT.steelDark));
  const stick2 = new THREE.Group();
  stick2.position.copy(cycP2);
  root.add(stick2);
  {
    const boot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.16, 0.22, 20), MAT.steelBlack));
    boot.position.y = 0.11;
    stick2.add(boot);
    stick2.add(shadow(new THREE.Mesh(new THREE.SphereGeometry(0.09, 20, 14), MAT.steel)));
    const tube = bar(stick2, V(0, -0.42, 0), V(0, 1.25, 0), 0.032, MAT.steelDark);
    const grip = shadow(new THREE.Mesh(new THREE.CapsuleGeometry(0.065, 0.26, 6, 16), MAT.steelBlack));
    grip.position.set(0.02, 1.42, 0);
    grip.rotation.z = -0.15;
    stick2.add(grip);
    for (const m of [tube, grip]) { m.userData.ctrl = 'cyclic'; m.userData.pivot = cycP2; pick.push(m); }
  }
  // lateral interconnect rod between the two sticks, below the floor
  // (both bolts at the SAME offset on each stick, so the rod keeps its length as both sticks tilt)
  for (const s of [stick, stick2]) bar(s, V(0, -0.25, 0), V(-0.14, -0.25, 0), 0.025, MAT.steelDark);
  const jCycX1 = new Joint(stick, root, V(-0.14, -0.25, 0), V(0, 1, 0), V(0, 0, -1), SMALL);
  const jCycX2 = new Joint(stick2, root, V(-0.14, -0.25, 0), V(0, 1, 0), V(0, 0, 1), SMALL);
  // collective torque tube carrying both levers
  const colP2 = colP.clone().setZ(-1.45);
  root.add(box(0.26, 0.32, 0.16, colP2.clone().setY(FLOOR_Y + 0.16), MAT.steelDark));
  const torqueTube = new THREE.Group();
  torqueTube.position.copy(colP);
  root.add(torqueTube);
  {
    const t = cylZ(0.05, colP2.z - colP.z - 0.08, 0.08, MAT.steel, { seg: 20 });
    torqueTube.add(t);
    for (const z of [-0.6, -1.2]) torqueTube.add(box(0.06, 0.03, 0.03, V(0, 0.042, z), MAT.gold));   // index marks set into the tube (show rotation)
  }
  const coll2 = new THREE.Group();
  coll2.position.copy(colP2);
  root.add(coll2);
  {
    const lev = bar(coll2, V(0, 0, 0), V(1.25, 0, 0), 0.035, MAT.steelDark);
    const head = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.13, 0.12), MAT.steelBlack));
    head.position.set(1.33, 0.02, 0);
    coll2.add(head);
    const throttle = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.32, 18), MAT.steelBlack));
    throttle.rotation.z = Math.PI / 2;
    throttle.position.set(1.0, 0, 0);
    coll2.add(throttle);
    const hub2 = cylZ(0.07, -0.1, 0.1, MAT.steel);
    coll2.add(hub2);
    for (const m of [lev, head, throttle]) { m.userData.ctrl = 'collective'; m.userData.pivot = colP2; pick.push(m); }
  }
  // copilot pedals + interconnect rod between the two rockers (aft arms)
  const pedP2 = pedP.clone().setZ(-0.9);
  root.add(box(0.2, 0.16, 0.2, pedP2.clone().setY(FLOOR_Y + 0.08), MAT.steelDark));
  const rocker2 = new THREE.Group();
  rocker2.position.copy(pedP2);
  root.add(rocker2);
  {
    bar(rocker2, V(0, 0.06, -0.42), V(0, 0.06, 0.42), 0.035, MAT.steelDark);
    for (const s of [-1, 1]) {
      const z = s * 0.36;
      bar(rocker2, V(0, 0.06, z), V(0.05, 0.36, z), 0.028, MAT.steelDark);
      const pedal = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.38, 0.24), MAT.steelBlack));
      pedal.position.set(0.06, 0.42, z);
      pedal.rotation.z = -0.35;          // top leans forward (away from the pilot)
      pedal.userData.ctrl = s < 0 ? 'pedalL' : 'pedalR';
      rocker2.add(pedal);
      pick.push(pedal);
    }
  }
  for (const r of [rocker, rocker2]) bar(r, V(0, 0.06, 0), V(-0.3, 0.06, 0), 0.03, MAT.steelDark);   // aft arms
  const jPedX1 = new Joint(rocker, root, V(-0.3, 0.06, 0), V(0, 1, 0), V(0, 0, -1), SMALL);
  const jPedX2 = new Joint(rocker2, root, V(-0.3, 0.06, 0), V(0, 1, 0), V(0, 0, 1), SMALL);

  // ---------------- bellcranks + mixing unit input levers ----------------
  // B1: lateral bellcrank beside the cyclic (vertical axis) turns sideways motion into fore-aft
  const bLat1 = new Lever(root, root, V(6.9, FLOOR_Y - 0.4, 1.55), Yax, [{ dir: V(1, 0, 0), len: 0.3 }, { dir: V(0, 0, -1), len: 0.3 }]);
  // floor bellcranks at the column (axis Z): input arm down, output arm forward (tip moves vertically)
  const floorBC = {};
  for (const [k, z] of Object.entries(LANES)) {
    floorBC[k] = new Lever(root, root, V(MIX.x + 1.0, FLOOR_Y - 0.35, z), V(0, 0, 1),
      [{ dir: V(0, -1, 0), len: 0.3 }, { dir: V(-1, 0, 0), len: 0.3 }]);
  }
  // mixing unit
  root.add(box(0.7, 0.8, 1.1, MIX, MAT.paint));
  root.add(box(0.74, 0.08, 1.14, MIX.clone().add(V(0, 0.42, 0)), MAT.steelDark));
  const mixIn = {};
  for (const [k, z] of Object.entries(LANES)) {
    // pivot on the front face of the unit; arm forward so its tip moves up / down over the column
    mixIn[k] = new Lever(root, root, V(MIX.x + 0.4, MIX.y - 0.3, z), V(0, 0, 1), [{ dir: V(1, 0, 0), len: 0.3 }]);
    root.add(box(0.08, 0.12, 0.1, V(MIX.x + 0.37, MIX.y - 0.3, z), MAT.steelDark));      // pivot boss
  }
  const MIX_OUT_Z = [0.2, 0.45, 0.85];         // servo 0 (fwd), servo 1 (left aft), servo 2 (right aft)
  const mixOut = MIX_OUT_Z.map((z) => new Lever(root, root, V(MIX.x - 0.15, MIX.y + 0.48, z), V(0, 0, 1), [{ dir: V(0, 1, 0), len: 0.3 }]));
  // tail rotor cable quadrant on the side of the mixing unit
  const quad = new THREE.Group();
  quad.position.set(MIX.x, MIX.y, MIX.z + 0.62);
  root.add(quad);
  {
    const disc = cylZ(0.24, -0.04, 0.04, MAT.steel, { seg: 40 });
    quad.add(disc);
    quad.add(cylZ(0.05, -0.08, 0.08, MAT.steelDark));
    quad.add(box(0.05, 0.36, 0.03, V(0, 0.1, 0.045), MAT.gold));     // index, shows rotation
  }

  // ---------------- pilot-assist module (TM 2.38.5-6) ----------------
  // In each channel the column rod is split by a hydraulic unit whose piston follows the input
  // rod 1:1 (boost servos reduce stick forces; SAS actuators add small stability inputs — shown
  // at zero extension here). collective: boost · pitch: boost + SAS · roll: SAS · yaw: boost + SAS.
  const PA_Y = 0.75, PA_X = MIX.x + 0.7, HALF = 0.75;
  const UNITS = { coll: { boost: true, sas: false }, lon: { boost: true, sas: true }, lat: { boost: false, sas: true }, yaw: { boost: true, sas: true } };
  root.add(box(0.05, 0.95, 1.1, V(PA_X - 0.1, PA_Y, 0.62), MAT.paint));          // module back plate (aft side, servos in front of it)
  const piston = {}, jPaLo = {}, jPaHi = {};
  for (const [k, z] of Object.entries(LANES)) {
    const u = UNITS[k];
    if (u.boost) {
      const body = cylZ(0.11, -0.17, 0.17, MAT.steel, { seg: 24 });
      body.rotation.x = Math.PI / 2;
      body.position.set(PA_X, PA_Y, z);
      root.add(body);
      for (const s of [-1, 1]) { const cap = cylZ(0.12, -0.03, 0.03, MAT.steelDark, { seg: 24 }); cap.rotation.x = Math.PI / 2; cap.position.set(PA_X, PA_Y + s * 0.17, z); root.add(cap); }
    } else {
      for (const y of [-0.3, 0.05]) {                                                  // guide bushings on brackets
        const g = cylZ(0.085, -0.05, 0.05, MAT.steelDark, { seg: 20 }); g.rotation.x = Math.PI / 2; g.position.set(PA_X, PA_Y + y, z); root.add(g);
        root.add(box(0.12, 0.05, 0.06, V(PA_X - 0.1, PA_Y + y, z), MAT.steelDark));
      }
    }
    const pg = new THREE.Group();
    pg.position.set(PA_X, PA_Y, z);
    root.add(pg);
    bar(pg, V(0, -HALF + 0.11, 0), V(0, HALF - 0.11, 0), 0.035, MAT.alu);              // piston rod
    if (u.sas) {                                                                       // SAS actuator (in series, moves with the piston)
      const sas = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.18, 20), MAT.paintLight));
      sas.position.y = 0.55;
      pg.add(sas);
    }
    piston[k] = pg;
    jPaLo[k] = new Joint(pg, root, V(0, -HALF, 0), V(0, 0, 1), V(0, -1, 0), SMALL);
    jPaHi[k] = new Joint(pg, root, V(0, HALF, 0), V(0, 0, 1), V(0, 1, 0), SMALL);
  }

  // ---------------- servo pilot-valve levers ----------------
  const servos = rh.servos;
  const valve = servos.map((s) => new Lever(s.sv.g, root, V(0, 0.92, 0.3), V(1, 0, 0), [{ dir: V(0, 1, 0), len: 0.22 }], { hub: 0.045 }));

  // ---------------- rods (fixed length) ----------------
  const rod = (o = {}) => new Link(root, { r: 0.028, eye: 0.045, ...o });
  const R = {
    lon1: rod(), lat1: rod({ turnbuckle: false }), lat2: rod(), coll1: rod(), yaw1: rod(),
    lo: Object.fromEntries(Object.keys(LANES).map((k) => [k, rod()])),
    hi: Object.fromEntries(Object.keys(LANES).map((k) => [k, rod({ turnbuckle: false })])),
    out: MIX_OUT_Z.map(() => rod()),
    cycX: rod(), pedX: rod({ turnbuckle: false }),
  };

  // cables to the tail rotor servo (two runs, over pulleys supported by the ghost airframe)
  const tq = tail.servoBase.clone().add(V(0.35, -0.3, 0.3));
  // quadrant -> down to the cabin roof -> aft under the roof -> into the tail cone -> pulley in
  // the intermediate gearbox fairing (left side, clear of the gears) -> tail rotor servo quadrant
  const PATH = [
    V(MIX.x, MIX.y, MIX.z + 0.62), V(MIX.x, ROOF_Y - 0.15, MIX.z + 0.62), V(-3.3, ROOF_Y - 0.15, MIX.z + 0.62),
    V(-3.6, -0.55, 0.35), V(-18.2, -0.1, -0.45), V(-19.4, 0.9, -1.2), tq,
  ];
  for (const s of [-1, 1]) {
    const pts = PATH.map((p, i) => p.clone().add(V(0, i === 0 ? s * 0.24 : s * 0.06, 0)));
    const cp = new THREE.CurvePath();
    for (let i = 0; i < pts.length - 1; i++) cp.add(new THREE.LineCurve3(pts[i], pts[i + 1]));
    const cable = shadow(new THREE.Mesh(new THREE.TubeGeometry(cp, 400, 0.012, 6), MAT.steel));
    cable.userData.overlay = true;   // tension member between pulleys (skipped by the contact check)
    root.add(cable);
  }
  for (const p of PATH.slice(1, -1)) {
    const pul = cylZ(0.09, -0.03, 0.03, MAT.steelDark, { seg: 24 });
    pul.position.copy(p);
    root.add(pul);
  }
  const tailQuad = new THREE.Group();
  tailQuad.position.copy(tq);
  root.add(tailQuad);
  tailQuad.add(cylZ(0.12, -0.03, 0.03, MAT.steel, { seg: 32 }));
  tailQuad.add(box(0.03, 0.2, 0.02, V(0, 0.05, 0.035), MAT.gold));

  // ---------------- neutral rod lengths ----------------
  root.updateMatrixWorld(true);
  const L = {};
  const dist = (a, b) => a.distanceTo(b);
  L.lon1 = dist(jCycLon.p(), floorBC.lon.tipAt(0, 0));
  L.lat1 = dist(jCycLat.p(), bLat1.tipAt(0, 0));
  L.lat2 = dist(bLat1.tipAt(1, 0), floorBC.lat.tipAt(0, 0));
  L.coll1 = dist(jColl.p(), floorBC.coll.tipAt(0, 0));
  L.yaw1 = dist(jPed.p(), floorBC.yaw.tipAt(0, 0));
  L.cycX = dist(jCycX1.p(), jCycX2.p());
  L.pedX = dist(jPedX1.p(), jPedX2.p());
  L.lo = Object.fromEntries(Object.keys(LANES).map((k) => [k, dist(floorBC[k].tipAt(1, 0), jPaLo[k].p())]));
  L.hi = Object.fromEntries(Object.keys(LANES).map((k) => [k, dist(jPaHi[k].p(), mixIn[k].tipAt(0, 0))]));
  let L_out = null, servoL0 = null;

  anchors.cyclic = cycP.clone().add(V(0, 1.9, 0.25));
  anchors.collective = colP.clone().add(V(1.2, 0.35, -0.3));
  anchors.pedals = pedP.clone().add(V(0.2, 0.75, -0.4));
  anchors.mixing = MIX.clone().add(V(0.1, 0.75, -0.2));
  anchors.column = V(COL_X, -2.0, 1.3);
  anchors.cables = V(-9, -0.1, 0.4);
  anchors.assist = V(PA_X, PA_Y + 0.95, 1.25);
  anchors.copilot = cycP2.clone().add(V(0, 1.9, -0.25));

  // ---------------- runtime ----------------
  const tmpQ = new THREE.Quaternion();
  function update(st) {
    const c = st.controls;
    // control positions
    const m = Math.min(Math.hypot(c.lon, c.lat), 1);
    if (m > 1e-4) stick.quaternion.setFromAxisAngle(V(c.lat, 0, -c.lon).normalize(), m * CYC_TILT);
    else stick.quaternion.identity();
    coll.quaternion.setFromAxisAngle(V(0, 0, 1), COLL_U0 + (COLL_U1 - COLL_U0) * c.collective);
    rocker.quaternion.setFromAxisAngle(Yax, -c.pedal * deg(16));
    // copilot controls move with the pilot's through the interconnects
    stick2.quaternion.copy(stick.quaternion);
    coll2.quaternion.copy(coll.quaternion);
    torqueTube.quaternion.copy(coll.quaternion);
    rocker2.quaternion.copy(rocker.quaternion);
    root.updateMatrixWorld(true);

    // drive the linkage chains (each lever angle solved from its fixed-length rod)
    floorBC.lon.solve(0, jCycLon.p(), L.lon1);
    bLat1.solve(0, jCycLat.p(), L.lat1);
    root.updateMatrixWorld(true);
    floorBC.lat.solve(0, bLat1.tipAt(1, bLat1.u), L.lat2);
    floorBC.coll.solve(0, jColl.p(), L.coll1);
    floorBC.yaw.solve(0, jPed.p(), L.yaw1);
    root.updateMatrixWorld(true);
    for (const k of Object.keys(LANES)) {
      // piston slides vertically: place its lower bolt exactly L.lo from the bellcrank tip
      const T = floorBC[k].tipAt(1, floorBC[k].u);
      const dx = T.x - PA_X, dz = T.z - LANES[k];
      const yLo = T.y + Math.sqrt(Math.max(L.lo[k] ** 2 - dx * dx - dz * dz, 0));
      piston[k].position.y = yLo + HALF;
      piston[k].updateMatrixWorld(true);
      mixIn[k].solve(0, jPaHi[k].p(), L.hi[k]);
    }

    // mixing unit outputs follow the servo commands; servo valve levers solved from the rods
    const lens = servos.map((s) => s.jTop.p().distanceTo(s.jBase.p()));
    if (!servoL0) servoL0 = lens.slice();
    mixOut.forEach((lv, i) => lv.setAngle(-(lens[i] - servoL0[i]) * 2.2));
    quad.quaternion.setFromAxisAngle(V(0, 0, 1), st.tailPitch * 0.9);
    tailQuad.quaternion.copy(quad.quaternion);
    root.updateMatrixWorld(true);
    if (!L_out) {
      valve.forEach((v) => v.setAngle(0));
      root.updateMatrixWorld(true);
      L_out = mixOut.map((lv, i) => lv.tipAt(0, lv.u).distanceTo(valve[i].tipAt(0, 0)));
    }
    valve.forEach((v, i) => v.solve(0, mixOut[i].tipAt(0, mixOut[i].u), L_out[i]));
    root.updateMatrixWorld(true);

    // pose the rods on their bolts
    const J = (lv, k) => lv.joints[k];
    const set = (link, a, b) => link.set(a.p(), b.p(), a.ax(), b.ax());
    set(R.lon1, jCycLon, J(floorBC.lon, 0));
    set(R.lat1, jCycLat, J(bLat1, 0));
    set(R.lat2, J(bLat1, 1), J(floorBC.lat, 0));
    set(R.coll1, jColl, J(floorBC.coll, 0));
    set(R.yaw1, jPed, J(floorBC.yaw, 0));
    for (const k of Object.keys(LANES)) {
      set(R.lo[k], J(floorBC[k], 1), jPaLo[k]);
      set(R.hi[k], jPaHi[k], J(mixIn[k], 0));
    }
    R.out.forEach((link, i) => set(link, J(mixOut[i], 0), J(valve[i], 0)));
    set(R.cycX, jCycX1, jCycX2);
    set(R.pedX, jPedX1, jPedX2);
  }

  // rods for verification: [name, link, jointA, jointB, nominal length]
  const check = () => [
    ['cyclic lon', R.lon1, jCycLon, floorBC.lon.joints[0], L.lon1],
    ['cyclic lat 1', R.lat1, jCycLat, bLat1.joints[0], L.lat1],
    ['cyclic lat 2', R.lat2, bLat1.joints[1], floorBC.lat.joints[0], L.lat2],
    ['collective', R.coll1, jColl, floorBC.coll.joints[0], L.coll1],
    ['pedals', R.yaw1, jPed, floorBC.yaw.joints[0], L.yaw1],
    ...Object.keys(LANES).map((k) => [`column ${k} (lower)`, R.lo[k], floorBC[k].joints[1], jPaLo[k], L.lo[k]]),
    ...Object.keys(LANES).map((k) => [`column ${k} (upper)`, R.hi[k], jPaHi[k], mixIn[k].joints[0], L.hi[k]]),
    ...R.out.map((l, i) => [`servo ${i + 1} input`, l, mixOut[i].joints[0], valve[i].joints[0], L_out && L_out[i]]),
    ['cyclic interconnect', R.cycX, jCycX1, jCycX2, L.cycX],
    ['pedal interconnect', R.pedX, jPedX1, jPedX2, L.pedX],
  ];

  return {
    anchors, update, pick, check,
    geo: { cycP, colP, pedP, CYC_TILT, COLL_U0, COLL_U1, gripH: 1.42 },
  };
}
