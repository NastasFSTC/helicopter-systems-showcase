// Mechanical detail parts with a consistent JOINT model:
//   * A Joint is a bolt: an exact point + a bolt axis, fixed to some parent part.
//   * jointClevis() builds the fork (two lugs + bolt + nut) centred exactly on that bolt.
//   * Link / Damper / Servo ends are spherical rod-ends: every frame each eye is placed on the
//     bolt point and turned so the bolt passes through the ball (eye plane contains the link).
import * as THREE from 'three';
import { MAT, shadow } from './parts.js';

const Y = new THREE.Vector3(0, 1, 0);
const _m = new THREE.Matrix4();
const _q = new THREE.Quaternion();

export function hexNut(r, h, mat = MAT.steelDark) {
  return shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 6), mat));
}

/** Quaternion whose local +Y = yDir and local +Z = zHint made perpendicular to yDir. */
export function frameQuat(yDir, zHint, out = new THREE.Quaternion()) {
  const y = yDir.clone().normalize();
  const z = zHint.clone().addScaledVector(y, -zHint.dot(y));
  if (z.lengthSq() < 1e-8) { z.set(1, 0, 0).addScaledVector(y, -y.x); if (z.lengthSq() < 1e-8) z.set(0, 0, 1); }
  z.normalize();
  const x = new THREE.Vector3().crossVectors(y, z);
  _m.makeBasis(x, y, z);
  return out.setFromRotationMatrix(_m);
}

/**
 * A bolt fixed to `parent`: position `pos` and axis `axis` in parent coordinates.
 * The clevis fork opens toward `openDir` (where the rod comes from).
 * `root` is the frame links are drawn in; p()/ax() return the bolt in root coordinates.
 */
export class Joint {
  constructor(parent, root, pos, axis, openDir, { gap = 0.14, l = 0.17, t = 0.032, h = 0.17, mat = MAT.paint, fork = true } = {}) {
    this.root = root;
    this.node = new THREE.Object3D();
    this.node.position.copy(pos);
    this.node.quaternion.copy(frameQuat(openDir, axis));   // node: +Y = open dir, +Z = bolt
    parent.add(this.node);
    if (fork) {
      for (const s of [-1, 1]) {
        const z = s * (gap / 2 + t / 2);
        const lug = shadow(new THREE.Mesh(new THREE.BoxGeometry(h, l, t), mat));
        lug.position.set(0, -l / 2, z);
        this.node.add(lug);
        const boss = shadow(new THREE.Mesh(new THREE.CylinderGeometry(h / 2, h / 2, t, 24), mat));
        boss.rotation.x = Math.PI / 2;
        boss.position.z = z;
        this.node.add(boss);
      }
      const base = shadow(new THREE.Mesh(new THREE.BoxGeometry(h, 0.05, gap + 2 * t), mat));
      base.position.y = -l;
      this.node.add(base);
    }
    const bl = gap + 2 * t + 0.09;
    const bolt = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, bl, 12), MAT.steel));
    bolt.rotation.x = Math.PI / 2;
    this.node.add(bolt);
    const head = hexNut(0.045, 0.035);
    head.rotation.x = Math.PI / 2;
    head.position.z = -(gap / 2 + t + 0.018);
    const nut = head.clone();
    nut.position.z = gap / 2 + t + 0.018;
    this.node.add(head, nut);
  }
  p() { return this.root.worldToLocal(this.node.getWorldPosition(new THREE.Vector3())); }
  ax() {
    const q = this.node.getWorldQuaternion(new THREE.Quaternion());
    const rq = this.root.getWorldQuaternion(_q).invert();
    return new THREE.Vector3(0, 0, 1).applyQuaternion(q).applyQuaternion(rq);
  }
}

/** Spherical rod-end: eye ring (plane XY, contains the link) + ball; bolt goes along local Z. */
function rodEnd(eye, mat) {
  const g = new THREE.Group();
  g.add(shadow(new THREE.Mesh(new THREE.TorusGeometry(eye, eye * 0.36, 10, 28), mat)));
  const ball = shadow(new THREE.Mesh(new THREE.SphereGeometry(eye * 0.66, 18, 12), MAT.steel));
  ball.scale.z = 0.8;
  g.add(ball);
  // shank from the eye toward the rod
  const sh = shadow(new THREE.Mesh(new THREE.CylinderGeometry(eye * 0.42, eye * 0.5, eye * 0.9, 12), mat));
  sh.position.y = eye * 1.25;
  g.add(sh);
  return g;
}
/** Place rod-end `e` on bolt point p with bolt axis ax; `toward` = direction to the other end. */
function poseEnd(e, p, ax, toward) {
  e.position.copy(p);
  frameQuat(toward, ax, e.quaternion);
}

/** Push-pull rod with rod ends; optional turnbuckle (adjustable length) in the middle. */
export class Link {
  constructor(parent, { r = 0.04, eye = 0.07, mat = MAT.steelDark, turnbuckle = true } = {}) {
    this.g = new THREE.Group();
    parent.add(this.g);
    this.eye = eye;
    const sg = new THREE.CylinderGeometry(r, r, 1, 14);
    sg.translate(0, 0.5, 0);
    this.shaft = shadow(new THREE.Mesh(sg, mat));
    this.g.add(this.shaft);
    this.e0 = rodEnd(eye, mat);
    this.e1 = rodEnd(eye, mat);
    parent.add(this.e0, this.e1);
    this.tb = null;
    if (turnbuckle) {
      this.tb = new THREE.Group();
      this.tb.add(shadow(new THREE.Mesh(new THREE.CylinderGeometry(r * 1.9, r * 1.9, 0.26, 6), MAT.steel)));
      for (const s of [-1, 1]) {
        const jam = hexNut(r * 2.1, 0.045, MAT.steelDark);
        jam.position.y = s * 0.17;
        this.tb.add(jam);
      }
      this.g.add(this.tb);
    }
  }
  /** a, b = bolt centres; axA, axB = bolt axes (all in the parent frame). */
  set(a, b, axA, axB = axA) {
    const d = b.clone().sub(a);
    const len = d.length();
    const u = d.clone().divideScalar(len);
    this.g.position.copy(a);
    this.g.quaternion.copy(frameQuat(u, axA));
    const off = this.eye * 1.6;                                  // shaft starts at the shank
    this.shaft.position.y = off;
    this.shaft.scale.y = Math.max(len - 2 * off, 0.001);
    if (this.tb) this.tb.position.y = len / 2;
    poseEnd(this.e0, a, axA, u);
    poseEnd(this.e1, b, axB, u.clone().negate());
  }
}

/** Hydraulic lead-lag damper: body + piston rod + side reservoir with fluid indicator, rod-end eyes. */
export class Damper {
  constructor(parent) {
    this.g = new THREE.Group();
    parent.add(this.g);
    this.e0 = rodEnd(0.075, MAT.steelDark);
    this.e1 = rodEnd(0.075, MAT.steelDark);
    parent.add(this.e0, this.e1);
    const bg = new THREE.CylinderGeometry(0.11, 0.11, 1, 24);
    bg.translate(0, 0.5, 0);
    this.body = shadow(new THREE.Mesh(bg, MAT.steelBlack));
    const rg = new THREE.CylinderGeometry(0.042, 0.042, 1, 14);
    rg.translate(0, 0.5, 0);
    this.rod = shadow(new THREE.Mesh(rg, MAT.alu));
    this.res = new THREE.Group();
    const res = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.24, 16), MAT.paintLight));
    res.position.set(0.16, 0, 0);
    const ind = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.062, 0.062, 0.05, 16), MAT.gold));
    ind.position.set(0.16, 0.07, 0);
    const pipe = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.1, 8), MAT.steel));
    pipe.rotation.z = Math.PI / 2;
    pipe.position.set(0.1, -0.06, 0);
    this.res.add(res, ind, pipe);
    this.capA = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.125, 0.125, 0.05, 24), MAT.steelDark));
    this.capB = this.capA.clone();
    this.g.add(this.body, this.rod, this.res, this.capA, this.capB);
  }
  set(a, b, axA, axB = axA) {
    const d = b.clone().sub(a);
    const len = d.length();
    const u = d.clone().divideScalar(len);
    this.g.position.copy(a);
    this.g.quaternion.copy(frameQuat(u, axA));
    const s0 = 0.14, bodyEnd = Math.max(len * 0.58, s0 + 0.2);
    this.body.position.y = s0; this.body.scale.y = bodyEnd - s0;
    this.capA.position.y = s0; this.capB.position.y = bodyEnd;
    this.rod.position.y = bodyEnd; this.rod.scale.y = Math.max(len - bodyEnd - 0.12, 0.01);
    this.res.position.y = (s0 + bodyEnd) / 2;
    poseEnd(this.e0, a, axA, u);
    poseEnd(this.e1, b, axB, u.clone().negate());
  }
}

/**
 * Two-stage primary servo: twin cylinders, manifold block, hydraulic lines, piston rod.
 * Its lower rod-end pivots on a deck joint, its upper rod-end on the swashplate joint.
 * yaw = azimuth the manifold should face.
 */
export class Servo {
  constructor(parent, base, yaw = 0) {
    this.base = base.clone();
    this.yawDir = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw));
    this.g = new THREE.Group();
    this.g.position.copy(base);
    parent.add(this.g);
    this.e0 = rodEnd(0.08, MAT.steelDark);
    this.e1 = rodEnd(0.08, MAT.steelDark);
    parent.add(this.e0, this.e1);
    const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.12, 0.12, 18), MAT.steelDark));
    foot.position.y = 0.2;
    this.g.add(foot);
    for (const s of [-1, 1]) {
      const barrel = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.9, 24), MAT.steel));
      barrel.position.set(s * 0.115, 0.72, 0);
      this.g.add(barrel);
      for (const y of [0.27, 1.17]) {
        const cap = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.125, 0.125, 0.06, 24), MAT.steelDark));
        cap.position.set(s * 0.115, y, 0);
        this.g.add(cap);
      }
    }
    const manifold = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.34, 0.24), MAT.paintLight));
    manifold.position.set(0, 0.68, 0.14);   // bolted against both cylinders
    this.g.add(manifold);
    const valve = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.3, 14), MAT.steelDark));
    valve.rotation.z = Math.PI / 2;
    valve.position.set(0, 0.75, 0.29);
    this.g.add(valve);
    for (const [x, h] of [[-0.14, 0.58], [0.14, 0.62]]) {           // system 1 / system 2 lines
      const curve = new THREE.CatmullRomCurve3([
        // from the manifold down into a deck fitting (the deck top is 0.27 below the servo pivot)
        new THREE.Vector3(x, h, 0.24), new THREE.Vector3(x, h - 0.25, 0.38), new THREE.Vector3(x * 1.5, 0.0, 0.45), new THREE.Vector3(x * 1.8, -0.32, 0.45),
      ]);
      this.g.add(shadow(new THREE.Mesh(new THREE.TubeGeometry(curve, 24, 0.022, 8), MAT.steelBlack)));
    }
    const rg = new THREE.CylinderGeometry(0.055, 0.055, 1, 16);
    rg.translate(0, 0.5, 0);
    this.rod = shadow(new THREE.Mesh(rg, MAT.alu));
    this.g.add(this.rod);
    this.gland = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.08, 18), MAT.steelDark));
    this.gland.position.y = 1.22;
    this.g.add(this.gland);
  }
  /** top / axes in the parent frame: axBase = deck bolt axis, axTop = swashplate bolt axis. */
  set(top, axTop, axBase) {
    const d = top.clone().sub(this.base);
    const len = d.length();
    const u = d.clone().divideScalar(len);
    this.g.quaternion.copy(frameQuat(u, this.yawDir));                 // manifold (+Z) faces outward
    this.rod.position.y = 1.2;
    this.rod.scale.y = Math.max(len - 1.2 - 0.12, 0.01);
    poseEnd(this.e0, this.base, axBase, u);
    poseEnd(this.e1, top, axTop, u.clone().negate());
  }
}

/** Revolve a profile [[r, x], ...] around the X axis. */
export function latheX(profile, mat, seg = 40) {
  const g = new THREE.LatheGeometry(profile.map(([r, x]) => new THREE.Vector2(r, x)), seg);
  g.rotateZ(-Math.PI / 2); // lathe axis Y -> X
  return shadow(new THREE.Mesh(g, mat));
}

/** Stadium (rounded slot) path between x0 and x1 with half-width h, in the XY plane. */
export function stadiumPath(x0, x1, h, cw = false) {
  const p = new THREE.Path();
  if (!cw) {
    p.absarc(x1, 0, h, -Math.PI / 2, Math.PI / 2, false);
    p.absarc(x0, 0, h, Math.PI / 2, (3 * Math.PI) / 2, false);
  } else {
    p.absarc(x0, 0, h, (3 * Math.PI) / 2, Math.PI / 2, true);
    p.absarc(x1, 0, h, Math.PI / 2, -Math.PI / 2, true);
  }
  return p;
}

/** Plate shape extruded along Y (horizontal plate between y0 and y1) with rounded edges. */
export function plateY(shape, y0, y1, mat, bevel = 0.04) {
  const g = new THREE.ExtrudeGeometry(shape, {
    depth: y1 - y0 - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 3, curveSegments: 32,
  });
  g.rotateX(-Math.PI / 2);        // extrude Z -> Y ; shape XY -> XZ (shape Y -> -Z)
  g.translate(0, y0 + bevel, 0);
  return shadow(new THREE.Mesh(g, mat));
}
