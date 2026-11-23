// UH-60 tail: intermediate gearbox, pylon drive shaft, tail gearbox, 4-blade bearingless
// cross-beam tail rotor (right side, canted 20° up), pitch change beam + tail rotor servo,
// and the variable-incidence stabilator. Facts: RESEARCH.md A3, A5, A7.
// Gear tooth counts here are ASSUMED (chosen to match the published speed ratios).
import * as THREE from 'three';
import { bevelPair, MAST_RPM } from './uh60-transmission.js';
import { MAT, shadow, cylZ, tubeZ, boltRing } from './parts.js';
import { Joint, Link, Damper } from './detail.js';

const deg = THREE.MathUtils.degToRad;
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const Yax = V(0, 1, 0);

export const TAIL_TEETH = { igbIn: 25, igbOut: 31, tgbIn: 19, tgbOut: 53 }; // assumed
const IGB_ANGLE = deg(58);   // drive-angle change at the intermediate gearbox (Army handout)
const CANT = deg(20);        // tail rotor cant, upward (TM)
const TR_R = 3.5;            // tail rotor radius (display scale, same shrink as main blades)

class Rod {
  constructor(parent, r, mat) {
    this.m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, 1, 18), mat));
    parent.add(this.m);
  }
  set(a, b) {
    const d = b.clone().sub(a);
    this.m.position.copy(a).addScaledVector(d, 0.5);
    this.m.quaternion.setFromUnitVectors(Yax, d.clone().normalize());
    this.m.scale.set(1, d.length(), 1);
  }
}

/** Airfoil blade along +X (r0..r1), chord along ±Y (leading edge at leSign*Y), thickness along Z. */
export function airfoilBlade(r0, r1, chord, leSign, { thick = 0.12, twistRoot = 0, twistTip = 0, tipColor = 0xd8dde3 } = {}) {
  const nS = 16, nA = 28;
  const pos = [], col = [], idx = [];
  const base = new THREE.Color(0x3a3f45), tipC = new THREE.Color(tipColor);
  for (let i = 0; i <= nS; i++) {
    const s = i / nS, r = r0 + (r1 - r0) * s;
    const tw = twistRoot + (twistTip - twistRoot) * s;
    for (let j = 0; j <= nA; j++) {
      const u = j / nA;
      const x01 = 0.5 * (1 - Math.cos(2 * Math.PI * u));
      const t = thick * 5 * (0.2969 * Math.sqrt(x01) - 0.126 * x01 - 0.3516 * x01 ** 2 + 0.2843 * x01 ** 3 - 0.1036 * x01 ** 4);
      const zz = (u < 0.5 ? t : -t) * chord;
      const yy = leSign * (0.25 - x01) * chord;        // LE at +leSign*0.25c
      const y2 = yy * Math.cos(tw) - zz * Math.sin(tw) * leSign;
      const z2 = yy * Math.sin(tw) * leSign + zz * Math.cos(tw);
      pos.push(r, y2, z2);
      const c = r > r1 - 0.45 ? tipC : base;
      col.push(c.r, c.g, c.b);
    }
  }
  for (let i = 0; i < nS; i++) for (let j = 0; j < nA; j++) {
    const a = i * (nA + 1) + j, b = a + nA + 1;
    idx.push(a, b, a + 1, b, b + 1, a + 1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}
export const BLADE_MAT = new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0.15, roughness: 0.5, clearcoat: 0.3, side: THREE.DoubleSide });

/** Flexible graphite spar strip whose twist grows from 0 at the clamp to full pitch at the cuff. */
class FlexSpar {
  constructor(parent, r0, r1, w, t) {
    this.r0 = r0; this.r1 = r1;
    const g = new THREE.BoxGeometry(r1 - r0, w, t, 12, 1, 1);
    g.translate((r0 + r1) / 2, 0, 0);
    this.base = g.attributes.position.array.slice();
    this.geo = g;
    this.m = shadow(new THREE.Mesh(g, MAT.steelBlack));
    parent.add(this.m);
  }
  setTwist(theta) {
    const p = this.geo.attributes.position, b = this.base;
    for (let i = 0; i < p.count; i++) {
      const x = b[i * 3], y = b[i * 3 + 1], z = b[i * 3 + 2];
      const s = THREE.MathUtils.smoothstep(x, this.r0, this.r1);
      const a = theta * s, c = Math.cos(a), sn = Math.sin(a);
      p.setXYZ(i, x, y * c - z * sn, y * sn + z * c);
    }
    p.needsUpdate = true;
    this.geo.computeVertexNormals();
  }
}

export function buildTail(root, tx, td) {
  const anchors = { ...td.anchors };
  const tp = td.shaft;                  // airframe tail drive shaft (axis +X)
  const yT = td.y;
  const m = 0.06;

  // ---------------- intermediate gearbox (base of the pylon) ----------------
  const O1 = V(-19.2, yT, 0);
  const dPylon = V(-Math.cos(IGB_ANGLE), Math.sin(IGB_ANGLE), 0);   // up and aft
  const igb = bevelPair(root, O1, V(1, 0, 0), dPylon, {
    z1: TAIL_TEETH.igbIn, z2: TAIL_TEETH.igbOut, m, faceWidth: 0.32, names: ['IGB input gear', 'IGB output gear'],
  });
  igb.g1.k = -tp.k;   // same shaft as the tail drive shaft, frame axis reversed
  igb.g1.phase = 0;
  igb.solve();
  // short shaft joining the tail drive shaft
  const backIn = igb.A * Math.cos(igb.d1);
  const tailShaftEndX = -17.93;                       // end of the 6-section tail drive shaft
  igb.g1.add(cylZ(0.15, 0.05, -(tailShaftEndX - (O1.x + backIn)) - 0.05, MAT.alu));
  igb.g1.add(cylZ(0.3, -0.05, -0.22, MAT.steel));

  // ---------------- pylon drive shaft + tail gearbox ----------------
  const pylonLen = 7.0;
  const O2 = O1.clone().addScaledVector(dPylon, pylonLen);
  const dTR = V(0, Math.sin(CANT), Math.cos(CANT));                 // tail rotor shaft: right + 20° up
  const tgb = bevelPair(root, O2, dPylon.clone().negate(), dTR.clone().negate(), {
    z1: TAIL_TEETH.tgbIn, z2: TAIL_TEETH.tgbOut, m, faceWidth: 0.4, rim2: 0.12, names: ['TGB input gear', 'TGB output gear'],
  });
  // pylon shaft: from IGB output gear up to TGB input gear (same shaft, frames point opposite ways)
  const backOut = igb.A * Math.cos(igb.d2);
  const backTin = tgb.A * Math.cos(tgb.d1);
  const shaftLen = pylonLen - backOut - backTin;
  igb.g2.add(cylZ(0.14, 0.05, -shaftLen + 0.05, MAT.alu));
  igb.g2.add(cylZ(0.3, -0.05, -0.2, MAT.steel));
  tgb.g1.k = -igb.g2.k;
  tgb.g1.phase = 0;
  tgb.solve();
  tgb.g1.add(cylZ(0.26, -0.05, -0.2, MAT.steel));
  tgb.g2.add(tubeZ(1.3, 0.25, -0.12, -0.04, MAT.steelDark));
  tgb.g2.add(cylZ(0.3, -0.3, 0.1, MAT.steel));
  tgb.g2.add(boltRing(12, 0.85, -0.04, 0.035, MAT.steelDark, 1));
  for (const g of [igb.g1, igb.g2, tgb.g1, tgb.g2]) tx.rotors.push(g);
  for (const g of [igb.g2, tgb.g1, tgb.g2]) g.angle = g.phase;

  // gearbox housings (see-through) and pylon shaft hanger
  const glass = MAT.glass.clone();
  glass.opacity = 0.1;
  const igbCase = shadow(new THREE.Mesh(new THREE.SphereGeometry(1.35, 40, 24), glass));
  igbCase.position.copy(O1);
  igbCase.userData.support = true;
  root.add(igbCase);
  const tgbCase = shadow(new THREE.Mesh(new THREE.SphereGeometry(1.75, 40, 24), glass));
  tgbCase.position.copy(O2).addScaledVector(dTR, -0.7);
  tgbCase.userData.support = true;
  root.add(tgbCase);

  // ghost airframe (tail cone + pylon) — the hanger bearings, gearboxes and stabilator mount to it
  const ghost = MAT.glass.clone();
  ghost.opacity = 0.06;
  {
    const top = yT - 0.48;                 // hanger-bearing plates sit on the tail-cone top
    const fwd = V(-3.2, top - 0.95, 0), aft = V(-19.6, top - 0.5, 0);
    const d = aft.clone().sub(fwd);
    const cone = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.95, d.length(), 40, 1, false), ghost);
    cone.position.copy(fwd).addScaledVector(d, 0.5);
    cone.quaternion.setFromUnitVectors(Yax, d.normalize());
    cone.userData.support = true;
    root.add(cone);
    const s = new THREE.Shape();
    s.moveTo(-18.4, -0.4); s.lineTo(-21.95, 7.5); s.lineTo(-23.7, 7.5); s.lineTo(-20.9, -0.4); s.closePath();
    const pg = new THREE.ExtrudeGeometry(s, { depth: 0.7, bevelEnabled: false });
    pg.translate(0, 0, -0.35);
    const pylon = new THREE.Mesh(pg, ghost);
    pylon.userData.support = true;
    root.add(pylon);
  }

  // ---------------- tail rotor (on the TGB output shaft) ----------------
  const hubPos = O2.clone().addScaledVector(dTR, 0.85);
  const tr = new (tgb.g2.constructor)(root, hubPos, dTR, { name: 'Tail rotor' });
  tr.k = tgb.g2.k; tr.phase = 0; tr.angle = 0;
  tx.rotors.push(tr);
  const spinSign = Math.sign(tr.k);           // + = counter-clockwise about the outward shaft axis
  const le = spinSign;                         // leading edge direction along local ±Y
  tr.add(cylZ(0.22, -2.0, 0.15, MAT.steel));   // output shaft (hollow: the servo rod runs inside)
  tr.add(cylZ(0.38, -0.12, 0.12, MAT.paint, { seg: 4 }));
  const trBlades = [];
  for (let i = 0; i < 4; i++) {
    const beam = i % 2;                                   // two cross-beams, stacked along the shaft
    const zOff = beam ? 0.09 : -0.09;
    const arm = new THREE.Group();
    arm.rotation.z = (i * Math.PI) / 2;
    arm.position.z = zOff;
    tr.add(arm);
    const spar = new FlexSpar(arm, 0.25, 0.95, 0.16, 0.035);
    const pitchG = new THREE.Group();                     // rigid part beyond the flexible zone
    arm.add(pitchG);
    pitchG.add(shadow(new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.26, 0.12), MAT.paint)).translateX(1.08));
    const bl = shadow(new THREE.Mesh(airfoilBlade(1.2, TR_R, 0.42, le, { twistRoot: deg(4), twistTip: deg(-4) }), BLADE_MAT));
    pitchG.add(bl);
    // pitch horn on the cuff, reaching outboard toward the pitch change beam
    const hornTip = new THREE.Object3D();
    hornTip.position.set(1.0, le * 0.26, 0.42 - zOff);
    pitchG.add(hornTip);
    const horn = new Rod(pitchG, 0.035, MAT.paint);
    horn.set(V(1.0, le * 0.1, 0.05), hornTip.position.clone().add(V(0, 0, -0.12)));
    // horn bolt: spanwise, fork opens outboard toward the pitch change beam
    const jHorn = new Joint(pitchG, root, hornTip.position.clone(), V(1, 0, 0), V(0, 0, 1), { gap: 0.09, l: 0.12, h: 0.11, t: 0.022 });
    trBlades.push({ arm, spar, pitchG, hornTip, zOff, jHorn });
  }
  // hub clamp plates
  tr.add(cylZ(0.42, -0.2, -0.14, MAT.paint, { seg: 4 }));
  tr.add(cylZ(0.42, 0.14, 0.2, MAT.paint, { seg: 4 }));
  // pitch change beam (rotating) + control rod through the hollow shaft (non-rotating)
  const pcb = new THREE.Group();
  tr.add(pcb);
  pcb.add(cylZ(0.18, -0.06, 0.06, MAT.steel));
  const beamArms = [];
  for (let i = 0; i < 4; i++) {
    const a = (i * Math.PI) / 2;
    const armM = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.09, 0.07), MAT.paint));
    armM.position.set(Math.cos(a) * 0.47, Math.sin(a) * 0.47, 0);
    armM.rotation.z = a;
    pcb.add(armM);
    const tipPos = V(Math.cos(a) * 1.0 - Math.sin(a) * le * 0.26, Math.sin(a) * 1.0 + Math.cos(a) * le * 0.26, 0);
    // beam arm extension out to the fork
    const ext = new Rod(pcb, 0.04, MAT.paint);
    ext.set(V(Math.cos(a) * 0.9, Math.sin(a) * 0.9, 0), tipPos.clone().add(V(0, 0, 0.1)));
    beamArms.push(new Joint(pcb, root, tipPos, V(Math.cos(a), Math.sin(a), 0), V(0, 0, -1), { gap: 0.09, l: 0.12, h: 0.11, t: 0.022 }));
  }
  const pitchLinks = trBlades.map(() => new Link(root, { r: 0.025, eye: 0.045, turnbuckle: false }));
  const servoBase = O2.clone().addScaledVector(dTR, -2.15);
  const servo = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.75, 24), MAT.steel));
  servo.position.copy(servoBase).addScaledVector(dTR, 0.37);
  servo.quaternion.setFromUnitVectors(Yax, dTR);
  root.add(servo);
  const servoRod = new Rod(root, 0.05, MAT.alu);

  // ---------------- stabilator ----------------
  const stab = new THREE.Group();
  const hinge = V(-21.25, 1.45, 0);
  stab.position.copy(hinge);
  root.add(stab);
  const stabChord = 1.7, stabSpan = 4.4;
  {
    const shape = new THREE.Shape();
    const n = 30;
    for (let j = 0; j <= n; j++) {
      const u = j / n, x01 = 0.5 * (1 - Math.cos(2 * Math.PI * u));
      const t = 0.12 * 5 * (0.2969 * Math.sqrt(x01) - 0.126 * x01 - 0.3516 * x01 ** 2 + 0.2843 * x01 ** 3 - 0.1036 * x01 ** 4);
      const camber = -0.02 * Math.sin(Math.PI * x01);                 // inverted camber (down-load)
      const x = -(x01 - 0.25) * stabChord;                             // LE forward (+X), hinge at 25 % chord
      const y = ((u < 0.5 ? t : -t) + camber) * stabChord;
      if (j === 0) shape.moveTo(x, y); else shape.lineTo(x, y);
    }
    const g = new THREE.ExtrudeGeometry(shape, { depth: stabSpan * 2, bevelEnabled: false, steps: 1 });
    g.translate(0, 0, -stabSpan);
    stab.add(shadow(new THREE.Mesh(g, MAT.paintLight)));
    for (const s of [-1, 1]) {
      const cap = shadow(new THREE.Mesh(new THREE.BoxGeometry(stabChord * 0.95, 0.04, 0.05), MAT.paint));
      cap.position.set(-0.1, 0, s * stabSpan);
      stab.add(cap);
    }
    stab.add(cylZ(0.07, -stabSpan * 0.6, stabSpan * 0.6, MAT.steelDark)); // hinge tube
  }
  // two electric actuators (dual) from a fixed bracket to stabilator horns
  const actBase = [V(-21.45, 3.0, -0.35), V(-21.45, 3.0, 0.35)];
  const actHorn = [V(-0.55, 0.18, -0.35), V(-0.55, 0.18, 0.35)];
  // dual electric actuators: rod-end at a fixed bracket (bolt spanwise) and at a horn on the stabilator
  const actuators = actBase.map(() => { const d = new Damper(root); d.res.visible = false; return d; });
  const bracket = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.2, 1.0), MAT.paint));
  bracket.position.set(-21.45, 3.27, 0);
  root.add(bracket);
  const jBracket = actBase.map((p) => new Joint(root, root, p, V(0, 0, 1), V(0, -1, 0)));
  const jStabHorn = actHorn.map((p) => {
    const lugBase = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.08, 0.22), MAT.paint));
    lugBase.position.copy(p).add(V(0, -0.15, 0));
    stab.add(lugBase);
    return new Joint(stab, root, p, V(0, 0, 1), V(0, 1, 0));
  });

  // ---------------- downwash streaks (low speed: straight down; with airspeed: skewed aft) ----------------
  const N = 150;
  const streakGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.9, 6);
  const streaks = new THREE.InstancedMesh(streakGeo, new THREE.MeshBasicMaterial({ color: 0xbfe0ff, transparent: true, opacity: 0.55, depthWrite: false }), N);
  const pts = Array.from({ length: N }, () => V((Math.random() - 0.1) * -22, 8 - Math.random() * 10, (Math.random() - 0.5) * 12));
  root.add(streaks);
  streaks.visible = false;

  anchors.igb = O1.clone().add(V(0.6, 1.6, 0));
  anchors.pylonShaft = O1.clone().addScaledVector(dPylon, 3.6).add(V(0.5, 0, 0));
  anchors.tgb = O2.clone().add(V(0.6, 1.3, -1.2));
  anchors.trotor = hubPos.clone().add(V(0, TR_R + 0.5, 0.4));
  anchors.pcb = hubPos.clone().addScaledVector(dTR, 0.55).add(V(0.4, -0.6, 0));
  anchors.trservo = servoBase.clone().add(V(0.4, 0.55, 0));
  anchors.stab = hinge.clone().add(V(-0.3, 0.6, 3.2));

  // ---------------- runtime ----------------
  const st = { trPitch: 0, stabTED: deg(39), airspeed: 0, showDownwash: false };
  // axial position of a horn tip vs blade pitch (horn: 0.26 toward the leading edge, 0.42 outboard)
  const hornZ = (th) => 0.26 * Math.sin(th) + 0.42 * Math.cos(th);
  const tmpQ = new THREE.Quaternion();

  /** tailPitchCmd: -1..1 from pedals + mixing; airspeed in knots. */
  function update(tailPitchCmd, airspeed, dt) {
    // ---- tail rotor pitch (illustrative range -6°..+20°) ----
    const th = deg(7) + deg(13) * tailPitchCmd;
    st.trPitch = th;
    const travel = hornZ(th) - hornZ(0);             // axial travel of the pitch change beam
    pcb.position.z = 0.7 + travel;
    for (const b of trBlades) {
      b.pitchG.rotation.x = le * th;                  // +pitch → thrust outward (tractor, to the right)
      b.spar.setTwist(le * th);
    }
    root.updateMatrixWorld(true);
    trBlades.forEach((b, i) => {
      pitchLinks[i].set(beamArms[i].p(), b.jHorn.p(), beamArms[i].ax(), b.jHorn.ax());
    });
    const beamC = root.worldToLocal(pcb.getWorldPosition(V(0, 0, 0)));
    servoRod.set(servoBase.clone().addScaledVector(dTR, 0.7), beamC);

    // ---- stabilator schedule (shape illustrative; limits 39° TED / 9° TEU from the TM) ----
    st.airspeed = airspeed;
    let ted;
    if (airspeed <= 30) ted = 39;
    else if (airspeed <= 80) ted = 39 - (airspeed - 30) * (33 / 50);
    else ted = 6 - (airspeed - 80) * (9 / 70);
    ted = THREE.MathUtils.clamp(ted, -9, 39);
    st.stabTED = THREE.MathUtils.lerp(st.stabTED, deg(ted), 1 - Math.exp(-dt * 2.5));
    stab.rotation.z = st.stabTED;                     // + = trailing edge down
    root.updateMatrixWorld(true);
    actuators.forEach((a, i) => a.set(jBracket[i].p(), jStabHorn[i].p(), jBracket[i].ax(), jStabHorn[i].ax()));

    // ---- downwash ----
    streaks.visible = st.showDownwash;
    if (st.showDownwash) {
      const chi = Math.atan2(airspeed, 40);            // wake skew angle from vertical
      const vel = V(-Math.sin(chi), -Math.cos(chi), 0).multiplyScalar(7);
      tmpQ.setFromUnitVectors(Yax, vel.clone().normalize());
      const m4 = new THREE.Matrix4();
      pts.forEach((p, i) => {
        p.addScaledVector(vel, dt);
        if (p.y < -1.5 || p.x < -26) p.set(-Math.random() * 20 + 1, 7.6, (Math.random() - 0.5) * 12);
        m4.compose(p, tmpQ, V(1, 1, 1));
        streaks.setMatrixAt(i, m4);
      });
      streaks.instanceMatrix.needsUpdate = true;
    }
  }

  return {
    anchors, update, state: st, servoBase, trBladeGroups: trBlades.map((b) => b.pitchG), trLe: le,
    rpm: () => Math.abs(tr.k) * MAST_RPM,
    pairs: { igb, tgb },
  };
}
