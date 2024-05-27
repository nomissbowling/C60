#![doc(html_root_url = "https://docs.rs/c60/0.5.3")]
/*
  cc-rs https://crates.io/crates/cc
  bindgen https://crates.io/crates/bindgen

  dependencies asciiz ode-rs

  in the current directory
    drawstuff.dll
    ode.dll
    libstdc++-6.dll
    libgcc_s_seh-1.dll
    libwinpthread-1.dll
*/

use ph_faces::{prec_eq_f, avg_f4};
use trimesh::polyhedron::{
  self, tetra::*, cube::*, octa::*,
  sphere::*, cylinder::*, capsule::*, cone::*,
  torus::*, pipe::*, // polyhedron::pin
  revolution::*,
  Icosahedron,
  {Dodecahedron, DodecahedronCenter},
  {C60, C60Center}};
use trimesh::tmm::*;

use anyslot::anyslot::*;

use ode_rs::ds::Drawstuff;
use ode_rs::colors::*;
use ode_rs::ode::*;

use std::ffi::{c_void}; // used by impl_sim_fn
use impl_sim::{impl_sim_fn, impl_sim_derive};
use rand::{Rng, rngs};
use std::collections::HashMap;
use std::time;
use regex::Regex;

const APP_HELP: &str = "
  application defined key set (this app)
  '0': drop trimesh bunny
  '1': drop trimesh tetra
  '2': drop trimesh cube
  '3': drop trimesh icosahedron
  '4': drop tmball (tmbunny) (over calc when 3 tmbunnys at the same position)
  '5': drop test composite
  '6': drop test box small
  '7': drop c60 icosahedron
  '8': drop c60 dodecahedron
  '9': drop c60 fullerene
  '@': drop polyhedron (sequence)
  'h': left
  'j': front
  'k': back
  'l': right
  'c': collision info
  'x': collision info sub
  ' ': drop apple ball
  't': torque
  'o': big ball info
  'b': test mut (big ball)
  'a': test cmd (all info)";

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub enum Phase {
  PEmpty, PHold, PRelease, PDown, PEnd
}
pub use Phase::*;

impl Phase {
  pub fn from_usize(u: usize) -> Option<Self> {
    const PS: [Phase; Phase::PEnd as usize] = [
      PEmpty, PHold, PRelease, PDown];
    if u >= Phase::PEnd as usize { return None; }
    Some(PS[u])
  }
}

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub enum PE {
  ETetra, ECube, ECubeCenter, EOcta, ERSphere, ECylinder, ECapsule, ECone,
  ETorus, ERTorus, ERing, ETube, EHalfPipe, EPin,
  ERevolutionN0, ERevolutionN1,
  EIcosahedronN0, EIcosahedronN1,
  EDodecahedronN0, EDodecahedronN1,
  EDodecahedronCenterN0, EDodecahedronCenterN1,
  EC60N0, EC60N1,
  EC60CenterN0, EC60CenterN1,
  End
}
pub use PE::*;

impl PE {
  pub fn from_usize(u: usize) -> Option<Self> {
    const PS: [PE; PE::End as usize] = [
      ETetra, ECube, ECubeCenter, EOcta, ERSphere, ECylinder, ECapsule, ECone,
      ETorus, ERTorus, ERing, ETube, EHalfPipe, EPin,
      ERevolutionN0, ERevolutionN1,
      EIcosahedronN0, EIcosahedronN1,
      EDodecahedronN0, EDodecahedronN1,
      EDodecahedronCenterN0, EDodecahedronCenterN1,
      EC60N0, EC60N1,
      EC60CenterN0, EC60CenterN1];
    if u >= PE::End as usize { return None; }
    Some(PS[u])
  }
}

#[macro_export]
macro_rules! cp {
  ($slf: expr, $col: expr, $pos: expr, $q: expr,
    $tm: ident, $hm: ident, $k: expr) => {{
    let (s, t) = tmg!($tm, $hm, $k);
    let d = 1e-2;
    let dm = if prec_eq_f(t.ph.vol, 1e-6, 0.0) { d } else { d / t.ph.vol };
    let krp = Krp::new(false, false, false, 0.2, 0.3); // set true later
    let mi_tm = MetaTriMesh::new(false, dm, &mut t.ph.tmv, krp, 0, $col);
    let k = $slf.ts(s.as_str());
    let (body, _, _) = $slf.super_mut().creator(k.as_str(), mi_tm);
    $slf.set_pos_Q(body, $pos, $q);
    k
  }}
}
// pub use cp;

pub struct SimApp {
  /// phase
  phase: Phase,
  /// current hold key
  current: String,
  /// current hold pos
  pos: dVector3,
  /// next drop pe disp pos
  nexpos: dVector3,
  /// next drop pe
  nexpe: PE,
  /// next drop pe choose from rand::thread_rng()
  rng: rngs::ThreadRng,
  /// pre evolution drop
  ped: Vec<PE>,
  /// evolution &lt;key, next&gt;
  evo: HashMap<String, PE>,
  /// erase body pairs
  ebps: Vec<(dBodyID, dBodyID, String)>,
  /// collision info
  i: bool,
  /// collision info sub
  j: bool,
  t: time::Instant,
  n: usize,
  u: usize,
  cnt: usize
}

impl SimApp {

pub fn ts(&mut self, s: &str) -> String {
  format!("{}_{:016x}", s, self.t.elapsed().as_nanos())
}

pub fn ts_rm(&self, s: &str) -> Option<String> {
  let re = r"
(?P<o>[0-9A-Za-z_]+)
_
(?P<p>[0-9]+)
_
(?P<q>[0-9A-Fa-f]+)
".replace("\n", "");
  let re = Regex::new(&re).unwrap();
  match re.captures(s) {
  None => None,
  Some(caps) => Some(caps["o"].to_string())
  }
}

pub fn kgc(&mut self, s: &str) {
  let rode = self.super_mut();
  let Ok(o) = rode.find(s.to_string()) else { return; }; // get not mut
  let krp = rode.get_krp_mut(o.geom());
  krp.k = true;
  krp.g = true;
  krp.c = true;
  let Ok(o) = rode.find_mut(s.to_string()) else { return; }; // re get mut
  o.enable();
}

pub fn trans(&mut self) {
  let ck = self.current.clone(); // clone to skip borrow
  let q = self.pos.clone(); // clone to skip borrow
  let rode = self.super_mut();
  let Ok(o) = rode.find_mut(ck) else { return; };
  let p = o.pos_();
  for i in 0..4 { p[i] = q[i]; }
}

pub fn objs_mut(&mut self, f: bool, s: &str) {
  let rode = self.super_mut();
  if f || rode.is_modified(false) {
    self.cnt = rode.num();
    println!("obgs: {} in {}", self.cnt, s);
    let rode = self.super_get(); // must re get
    let ids = rode.each_id(|_key, _id| { true }); // lambda may return false
    for id in ids {
      if id == 0 as dBodyID { continue; } // skipped by result of each_id
      let rode = self.super_mut(); // must re get
      match rode.get_mut(id) {
        Err(e) => { println!("{}", e); },
        Ok(obg) => {
          // This is test code using each_id with get_mut, but high cost.
          // Better to use self.super_mut().find_mut("ball_big".to_string())
          if obg.key == "ball_big" { obg.col = [1.0, 0.0, 0.0, 0.8]; }
          println!("{}: {:018p} {:?}", obg.key, id, obg.col);
          // get_tcm_mut must be after accessing to obg members
          if obg.key == "ball_big" {
            let geom = obg.geom(); // must assign before get_tcm_mut
            let mgm = rode.get_mgm_mut(geom).unwrap(); // must care ok_or
            mgm.get_tcm_mut().col = [1.0, 0.0, 0.0, 0.8];
          }
        }
      }
    }
  }
}

pub fn objs_info(&mut self, f: bool, s: &str) {
  let rode = self.super_mut();
  if f || rode.is_modified(false) {
    self.cnt = rode.num();
    println!("obgs: {} in {}", self.cnt, s);
    if !f { return; }
    let rode = self.super_get(); // must re get because borrow later self.cnt
    rode.each(|key, id, obg| {
      println!("{}: {:018p} {:?}", key, id, obg.col);
      true
    });
  }
}

/// create test balls
pub fn create_test_balls(&mut self) {
  let m: dReal = 0.8;
  let r: dReal = 0.2;
  for i in 0..16 {
    let c: dVector4 = vec4_from_u32(COLORS[i]);
    let p: dVector3 = [(i%4) as dReal - 1.5, (i/4) as dReal - 1.5, 2.0, 1.0];
    let mib = MetaSphere::new(m, r, KRP095, 0, c);
    let (body, _, _) = self.super_mut().creator_m(
      format!("ball_{:08X}", i).as_str(), mib);
    self.set_pos_Q(body, p, QI);
  }
}

/// create test ball big
pub fn create_test_ball_big(&mut self) {
  let c: dVector4 = [1.0, 1.0, 0.0, 0.8];
  let p: dVector3 = [0.0, 0.0, 10.0, 1.0];
  let mib = MetaSphere::new(0.08 / (125.0 * PIt4), 1.0, KRP095, 0, c);
  let (body, _, _) = self.super_mut().creator("ball_big", mib);
  self.set_pos_Q(body, p, QI);
}

/// create test box small
pub fn create_test_box_small(&mut self) {
  let mibox_small = MetaBox::new(0.1, [1.0, 1.0, 1.0, 0.0],
    KRP095, 0, [0.0, 1.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("box_small", mibox_small);
  self.set_pos_Q(body, [-5.0, 5.0, 2.0, 1.0], QI);
}

/// create test box frames
pub fn create_test_box_frames(&mut self) {
  let mibox_big_0 = MetaBox::new(0.1, [1.0, 5.0, 0.5, 0.0],
    KRP095, 0, [1.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("box_big_0", mibox_big_0);
  self.set_pos_R(body, [-9.0, -11.0, 2.0, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 0.0, 1.0], PIx));

  let mibox_big_1 = MetaBox::new(0.1, [1.0, 12.0, 0.5, 0.0],
    KRP095, 0, [0.0, 1.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("box_big_1", mibox_big_1);
  self.set_pos_R(body, [12.0, -12.0, 2.0, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 0.0, 1.0], -PIq));

  let mibox_big_2 = MetaBox::new(0.1, [1.0, 12.0, 0.5, 0.0],
    KRP095, 0, [0.0, 1.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("box_big_2", mibox_big_2);
  self.set_pos_R(body, [12.0, 12.0, 2.0, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 0.0, 1.0], PIq));

  let mibox_big_3 = MetaBox::new(0.1, [1.0, 12.0, 0.5, 0.0],
    KRP095, 0, [0.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("box_big_3", mibox_big_3);
  self.set_pos_R(body, [-12.0, 12.0, 2.0, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 0.0, 1.0], -PIq));
}

/// create test capsule frames
pub fn create_test_capsule_frames(&mut self) {
  let micap_0 = MetaCapsule::new(0.001, 0.5, 16.0,
    KRP080, 0, [0.0, 1.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("p_capsule_0", micap_0);
  self.set_pos_R(body, [-8.6, 0.0, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([1.0, 0.0, 0.0], PIh));

  let micap_1 = MetaCapsule::new(0.001, 0.5, 16.0,
    KRP080, 0, [0.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("p_capsule_1", micap_1);
  self.set_pos_R(body, [8.6, 0.0, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([1.0, 0.0, 0.0], PIh));
}

/// create test cylinder frames
pub fn create_test_cylinder_frames(&mut self) {
  let micyl_0 = MetaCylinder::new(0.001, 0.5, 16.0,
    KRP080, 0, [1.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("p_cylinder_0", micyl_0);
  self.set_pos_R(body, [0.0, 8.6, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 1.0, 0.0], PIh));

  let micyl_1 = MetaCylinder::new(0.001, 0.5, 16.0,
    KRP080, 0, [0.0, 1.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("p_cylinder_1", micyl_1);
  self.set_pos_R(body, [0.0, -8.6, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 1.0, 0.0], PIh));
}

/// create test composite
pub fn create_test_composite(&mut self) {
  let micmp_0 = MetaComposite::new(
    vec![
      MetaBox::new(0.1, [0.5, 0.5, 0.5, 0.0], KRP095, 0, [1.0, 0.0, 0.0, 0.8]),
      MetaBox::new(0.1, [0.5, 0.5, 0.5, 0.0], KRP095, 0, [0.0, 0.0, 1.0, 0.8]),
      MetaSphere::new(0.6 / PI, 0.5, KRP095, 0, [0.0, 1.0, 0.0, 0.8]),
      MetaSphere::new(0.0001, 0.1, KRPnk, 0, [1.0, 0.0, 1.0, 0.8])],
    vec![QI, QI, QI, QI],
    vec![
      [-0.4, -0.4, -0.4, 1.0],
      [0.4, 0.4, 0.4, 1.0],
      [0.0, 0.0, 0.0, 1.0],
      [0.0, 0.0, 0.0, 1.0]],
    KRPnk, 0, [1.0, 0.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator_composite("composite_0", micmp_0);
  self.set_pos_Q(body, [-12.0, -2.0, 2.0, 1.0],
    dQuaternion::from_axis_and_angle([0.0, 0.0, 1.0], -PIq3));

  let micmp_1 = MetaComposite::new(
    vec![
      MetaBox::new(0.1, [0.5, 0.5, 0.5, 0.0], KRP095, 0, [1.0, 0.0, 0.0, 0.8]),
      MetaBox::new(0.1, [0.5, 0.5, 0.5, 0.0], KRP095, 0, [0.0, 0.0, 1.0, 0.8]),
      MetaSphere::new(0.6 / PI, 0.5, KRP095, 0, [0.0, 1.0, 0.0, 0.8])],
    vec![
      dQuaternion::from_axis_and_angle([-0.707, 0.707, 0.0], PIq),
      dQuaternion::from_axis_and_angle([0.707, -0.707, 0.0], -PIq),
      dQuaternion::new()],
    vec![
      [-0.4, -0.4, -0.4, 1.0],
      [0.4, 0.4, 0.4, 1.0],
      [0.0, 0.0, 0.0, 1.0]],
    KRP100, 0, [1.0, 0.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator_composite("composite_1", micmp_1);
  self.set_pos_Q(body, [-12.0, 0.0, 2.0, 1.0],
    dQuaternion::from_axis_and_angle([0.0, 0.0, 1.0], -PIq3));
}

/// create test tetra
pub fn create_test_tetra(&mut self) {
  let mitmv_tetra_0 = MetaTriMesh::new(false, 0.1, unsafe { &mut *tetra::tmv },
    KRP095, 0, [1.0, 0.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("tmv_tetra_0", mitmv_tetra_0);
  self.set_pos_Q(body, [-13.0, -6.0, 2.0, 1.0], QI);

  let mifvp_tetra_0 = MetaConvex::new(false, 0.1, unsafe { &mut *tetra::fvp },
    KRP095, 0, [0.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("fvp_tetra_0", mifvp_tetra_0);
  self.set_pos_Q(body, [-13.0, -8.0, 2.0, 1.0], QI);
}

/// create test cube
pub fn create_test_cube(&mut self) {
  let mitmv_cube_0 = MetaTriMesh::new(false, 0.1, unsafe { &mut *cube::tmv },
    KRP095, 0, [1.0, 1.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("tmv_cube_0", mitmv_cube_0);
  self.set_pos_Q(body, [-7.0, 1.0, 2.0, 1.0], QI);

  let mifvp_cube_0 = MetaConvex::new(false, 0.1, unsafe { &mut *cube::fvp },
    KRP095, 0, [1.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("fvp_cube_0", mifvp_cube_0);
  self.set_pos_Q(body, [-7.0, -1.0, 2.0, 1.0], QI);
}

/// create test icosahedron
pub fn create_test_icosahedron(&mut self) {
  let mitmv_ih_0 = MetaTriMesh::new(false, 0.1,
    unsafe { &mut *icosahedron::tmv },
    KRP095, 0, [0.0, 1.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("tmv_icosahedron_0", mitmv_ih_0);
  self.set_pos_Q(body, [-7.0, 3.0, 2.0, 1.0], QI);

  let mifvp_ih_0 = MetaConvex::new(false, 0.1,
    unsafe { &mut *icosahedron::fvp },
    KRP095, 0, [1.0, 1.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("fvp_icosahedron_0", mifvp_ih_0);
  self.set_pos_Q(body, [-7.0, -3.0, 2.0, 1.0], QI);
}

/// create test plane
pub fn create_test_plane(&mut self) {
  let rode = self.super_mut();
  let dm: dReal = 0.1;
  let lxyz: dVector3 = [10.0, 10.0, 0.05, 0.0];
  let norm: dVector4 = [0.0, 0.0, 1.0, 0.0];
  let col: dVector4 = vec4_from_u32(COLORS[0]);
  let pos: dVector3 = [-5.0, -5.0, 5.0, 0.0];
  let mip = MetaPlane::new(dm, lxyz, norm, KRPnk, 0, col);
  let (body, _, _) = rode.creator("plane", mip);
  let q = dQuaternion::from_axis_and_angle([1.0, 1.0, 0.0], PIq);
  rode.get_mut(body).expect("fail reg").set_pos(pos)
    // .set_rot(dMatrix3::from_z_axis([0.7, 0.7, 0.0]));
    // .set_rot(dMatrix3::from_2_axes([-0.7, 0.7, 0.0], [0.7, 0.7, 0.0]));
    // .set_rot(dMatrix3::from_euler_angles(PIq, PIq, PIq));
    // .set_rot(dMatrix3::from_axis_and_angle([0.0, 0.0, 1.0], PIq));
    // .set_rot(dMatrix3::new());
    // .set_rot(dMatrix3::from_Q(dQuaternion::new()));
    // .set_rot(dQuaternion::new().to_R());
    // .set_quaternion(dMatrix3::new().to_Q());
    // .set_quaternion(dQuaternion::from_R(dMatrix3::new()));
    // .set_quaternion(dQuaternion::new());
    // .set_quaternion(q);
    .set_rot(q.to_R());
}

/// create
pub fn create_tmball(&mut self) {
  let mi_tmball = MetaComposite::new(
    vec![
      MetaTriMesh::new(false, 0.1, unsafe { &mut *bunny::tmv },
        KRP095, 0, [1.0, 0.8, 0.2, 0.6]),
      MetaSphere::new(0.1, 1.2, KRP095, 0, [0.2, 1.0, 0.8, 0.4])],
    vec![QI, QI],
    vec![[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]],
    KRP100, 0, [1.0, 0.0, 0.0, 0.8]);
  let k = self.ts("tmball");
  let (body, _, _) = self.super_mut().creator_composite(k.as_str(), mi_tmball);
  let p = dQuaternion::from_axis_and_angle([0.0, 0.0, 1.0], PIh);
  let q = dQuaternion::from_axis_and_angle([1.0, 0.0, 0.0], PIh);
  let o = dQuaternion::multiply0(p, q);
  self.set_pos_Q(body, [-29.0, -3.0, 3.0, 1.0], o);
}

/// create
pub fn create_slope(&mut self) {
  let mi_slope = MetaComposite::new(
    vec![
      MetaBox::new(1.0, [6.0, 0.1, 8.0, 0.0], KRP001, 0, [1.0, 0.8, 0.2, 0.6]),
      MetaCylinder::new(1.0, 1.0, 2.0, KRP001, 0, [0.2, 1.0, 0.8, 0.4])],
    vec![QI, QI],
    vec![[0.0, 0.0, 0.0, 0.0], [-3.0, 0.0, 0.0, 0.0]],
    KRP100, 0, [1.0, 0.0, 0.0, 0.8]);
  let (body, _, _) = self.super_mut().creator_composite("slope", mi_slope);
  let p = dQuaternion::from_axis_and_angle([0.0, 1.0, 0.0], PIx / 3.0);
  let q = dQuaternion::from_axis_and_angle([1.0, 0.0, 0.0], PIh);
  let o = dQuaternion::multiply0(p, q);
  self.set_pos_Q(body, [-28.5, 0.0, 1.2, 1.0], o);
}

/// create x, y on the bunny
pub fn create_sphere_apple(&mut self) {
  let krp = Krp::new(true, false, true, 0.95, 0.1);
  let mi_apple = MetaSphere::new(0.1, 0.2, krp, 0, [0.8, 0.4, 0.4, 0.8]);
  let (body, _, _) = self.super_mut().creator("apple", mi_apple);
  self.set_pos_Q(body, [-15.15, 0.31, 2.5, 1.0], QI);
}

/// create
pub fn create_sphere_ball(&mut self) {
  let mi_ball = MetaSphere::new(0.1, 0.1, KRP080, 0, [0.4, 0.4, 0.8, 0.8]);
  let z = mi_ball.r;
  let (body, _, _) = self.super_mut().creator("ball", mi_ball);
  self.set_pos_Q(body, [-14.5, 0.0, z, 1.0], QI);
}

/// create on the slope
pub fn create_sphere_roll(&mut self) {
  let mi_roll = MetaSphere::new(0.1, 0.2, KRP080, 0, [0.4, 0.8, 0.4, 0.8]);
  let (body, _, _) = self.super_mut().creator("roll", mi_roll);
  self.set_pos_Q(body, [-27.0, 0.0, 1.2, 1.0], QI);
}

/// create
pub fn create_tmtetra(&mut self) {
  let mi_tmtetra = MetaTriMesh::new(false, 1.0, unsafe { &mut *tetra::tmv },
    KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
  let k = self.ts("tmtetra");
  let (body, _, _) = self.super_mut().creator(k.as_str(), mi_tmtetra);
  self.set_pos_Q(body, [-15.0, -1.5, 0.5, 1.0], QI);
}

/// create
pub fn create_tmcube(&mut self) {
  let mi_tmcube = MetaTriMesh::new(false, 1.0, unsafe { &mut *cube::tmv },
    KRP095, 0, [0.6, 0.8, 0.2, 1.0]);
  let k = self.ts("tmcube");
  let (body, _, _) = self.super_mut().creator(k.as_str(), mi_tmcube);
  self.set_pos_Q(body, [-16.5, -3.0, 0.5, 1.0],
    dQuaternion::from_axis_and_angle([1.0, 1.0, 1.0], PIq));
}

/// create
pub fn create_tmicosahedron(&mut self) {
  let mi_tmih = MetaTriMesh::new(false, 1.0, unsafe { &mut *icosahedron::tmv },
    KRP095, 0, [0.2, 0.8, 0.6, 1.0]);
  let k = self.ts("tmicosahedron");
  let (body, _, _) = self.super_mut().creator(k.as_str(), mi_tmih);
  self.set_pos_Q(body, [-16.5, 3.0, 0.5, 1.0], QI);
}

/// create
pub fn create_tmbunny(&mut self) {
  let mi_tmbunny = MetaTriMesh::new(false, 1.0, unsafe { &mut *bunny::tmv },
    KRP095, 0, [0.8, 0.2, 0.6, 1.0]);
  let k = self.ts("tmbunny");
  let (body, _, _) = self.super_mut().creator(k.as_str(), mi_tmbunny);
  // phi=-x, theta=-y, psi=-z
  let m = dMatrix3::from_euler_angles(-PIh, 0.0, 0.0);
  self.set_pos_Q(body, [-15.0, 0.25, 0.88, 1.0], dQuaternion::from_R(m));
  // to (-0.109884, 0.304591, 1.217693)
}

/// create c60 icosahedron
pub fn create_c60_icosahedron(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
      let nk = cp!(self,
        [0.8, 0.6, 0.2, 0.8],
        [-4.0 + 2.0 * i as f64, 0.0, 2.0, 1.0],
        QI,
        tm, icosahedron, i);
      self.kgc(&nk);
    });
  }
}

/// create c60 dodecahedron
pub fn create_c60_dodecahedron(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
      let nk = cp!(self,
        [0.8, 0.6, 0.2, 0.8],
        [-4.0 + 2.0 * i as f64, -4.0, 2.0, 1.0],
        QI,
        tm, dodecahedron, i);
      self.kgc(&nk);
    });
  }
}

/// create c60 dodecahedron center
pub fn create_c60_dodecahedron_center(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
      let nk = cp!(self,
        [0.8, 0.6, 0.2, 0.8],
        [-4.0 + 2.0 * i as f64, -2.0, 2.0, 1.0],
        QI,
        tm, dodecahedron_center, i);
      self.kgc(&nk);
    });
  }
}

/// create c60 fullerene
pub fn create_c60_fullerene(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
      let nk = cp!(self,
        [0.8, 0.6, 0.2, 0.8],
        [-4.0 + 2.0 * i as f64, 4.0, 2.0, 1.0],
        QI,
        tm, c60, i);
      self.kgc(&nk);
    });
  }
}

/// create c60 fullerene center
pub fn create_c60_fullerene_center(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
      let nk = cp!(self,
        [0.8, 0.6, 0.2, 0.8],
        [-4.0 + 2.0 * i as f64, 2.0, 2.0, 1.0],
        QI,
        tm, c60_center, i);
      self.kgc(&nk);
    });
  }
}

/// create polyhedron
pub fn create_polyhedron(&mut self, i: usize, p: dVector3) -> String {
  let col = vec![
    [0.8, 0.6, 0.2, 0.8],
    [0.2, 0.8, 0.6, 0.8],
    [0.6, 0.2, 0.8, 0.8],
    [0.8, 0.8, 0.2, 0.8],
    [0.8, 0.2, 0.8, 0.8],
    [0.2, 0.8, 0.8, 0.8],
    [0.8, 0.2, 0.6, 0.8],
    [0.6, 0.8, 0.2, 0.8],
    [0.2, 0.6, 0.8, 0.8]];
  let c = col[self.t.elapsed().as_secs() as usize % col.len()];
  let q = vec![
    QI, // +Y
    dQuaternion::from_axis_and_angle([1.0, 0.0, 0.0], PIh), // +Z
    // dMatrix3 phi=-x, theta=-y, psi=-z
    dQuaternion::from_R(dMatrix3::from_euler_angles(-PIh, 0.0, 0.0)), // +Z
    dQuaternion::from_R(dMatrix3::from_euler_angles(0.0, -PIh, 0.0)), // +YZX
    dQuaternion::from_R(dMatrix3::from_euler_angles(0.0, 0.0, -PIh))]; // -X
  let mut k: String = "".to_string(); // result
  any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
    k = match PE::from_usize(i % self.n).unwrap() {
    ETetra => cp!(self, c, p, q[0], tm, tetra, 0),
    ECube => cp!(self, c, p, q[1], tm, cube, 0),
    ECubeCenter => cp!(self, c, p, q[2], tm, cube_center, 0),
    EOcta => cp!(self, c, p, q[1], tm, octa, 0),
    ERSphere => cp!(self, c, p, q[1], tm, r_sphere, 0),
    ECylinder => cp!(self, c, p, q[1], tm, cylinder, 0),
    ECapsule => cp!(self, c, p, q[1], tm, capsule, 0),
    ECone => cp!(self, c, p, q[1], tm, cone, 0),
    ETorus => cp!(self, c, p, q[0], tm, torus, 0),
    ERTorus => cp!(self, c, p, q[4], tm, r_torus, 0),
    ERing => cp!(self, c, p, q[1], tm, ring, 0),
    ETube => cp!(self, c, p, q[1], tm, tube, 0),
    EHalfPipe => cp!(self, c, p, q[3], tm, half_pipe, 0),
    EPin => cp!(self, c, p, q[1], tm, pin, 0),
    ERevolutionN0 => cp!(self, c, p, q[2], tm, revolution, 0),
    ERevolutionN1 => cp!(self, c, p, q[4], tm, revolution, 1),
    EIcosahedronN0 => cp!(self, c, p, q[1], tm, icosahedron, 0),
    EIcosahedronN1 => cp!(self, c, p, q[1], tm, icosahedron, 1),
    EDodecahedronN0 => cp!(self, c, p, q[1], tm, dodecahedron, 0),
    EDodecahedronN1 => cp!(self, c, p, q[1], tm, dodecahedron, 1),
    EDodecahedronCenterN0 => cp!(self, c, p, q[1], tm, dodecahedron_center, 0),
    EDodecahedronCenterN1 => cp!(self, c, p, q[1], tm, dodecahedron_center, 1),
    EC60N0 => cp!(self, c, p, q[1], tm, c60, 0),
    EC60N1 => cp!(self, c, p, q[1], tm, c60, 1),
    EC60CenterN0 => cp!(self, c, p, q[1], tm, c60_center, 0),
    EC60CenterN1 => cp!(self, c, p, q[1], tm, c60_center, 1),
    _ => "nothing".to_string()
    };
    println!("polyhedron: {}", k);
  });
  k
}

}

#[impl_sim_derive(draw_geom, stop_callback)] // near_callback
impl Sim for SimApp {

fn draw_objects(&mut self) {
  self.objs_info(false, "draw"); // twice (after step)
/*
  let ds = ODE::ds_as_ref();
  ds.SetDrawMode(1); // test always wireframe
*/
  self.super_mut().draw_objects();
}

fn start_callback(&mut self) {
  let t_delta = &mut self.super_mut().t_delta;
  *t_delta = 0.002;
  self.create_test_balls();
  self.create_test_ball_big();
  self.create_test_box_small();
  self.create_test_box_frames();
  self.create_test_capsule_frames();
  self.create_test_cylinder_frames();
  self.create_test_composite();
  self.create_test_tetra();
  self.create_test_cube();
  self.create_test_icosahedron();
//  self.create_test_plane();

  self.create_tmball();
  self.create_slope();
  self.create_sphere_apple();
  self.create_sphere_ball(); self.create_sphere_roll();
  self.create_tmtetra();
  self.create_tmcube();
  self.create_tmicosahedron();
  self.create_tmbunny();

  any_pinned_with_bg_mut!(TriMeshManager<f64>, 0, |tm| {
    let r = 0.2;
    let tf = false;
    tms!(tm, tetra, Tetra::<f64>, 0).setup(1.0, tf); // 0
    tms!(tm, cube, Cube::<f64>, 0).setup(r, tf); // 1
    tms!(tm, cube_center, CubeCenter::<f64>, 0).setup(r, tf); // 2
    tms!(tm, octa, Octa::<f64>, 0).setup(1.0, tf); // 3
    tms!(tm, r_sphere, RSphere::<f64>, 0).setup(r, 6, tf); // 4
    tms!(tm, cylinder, Cylinder::<f64>, 0).setup(r, 2.0, 6, tf); // 5
    tms!(tm, capsule, Capsule::<f64>, 0).setup(r, 2.0, 6, tf); // 6
    tms!(tm, cone, Cone::<f64>, 0).setup(r, 2.0, 6, tf); // 7
    tms!(tm, torus, Torus::<f64>, 0).setup(2.0, 0.5, 6, 6, tf); // 8
    tms!(tm, r_torus, RTorus::<f64>, 0).setup(2.0, 0.5, 12, 6, tf); // 9
    tms!(tm, ring, Ring::<f64>, 0).setup(2.0, 0.1, 0.4, 12, 6, tf); // 10
    tms!(tm, tube, Tube::<f64>, 0).setup(3.0, 2.8, 4.0, 6, tf); // 11
    tms!(tm, half_pipe, HalfPipe::<f64>, 0)
    .setup(3.141592654, 3.0, 2.8, 4.0, 6, tf); // 12
    tms!(tm, pin, polyhedron::pin::Pin::<f64>, 0).setup(r, 8, 6, tf); // 13
    tms!(tm, revolution, Revolution::<f64>, 0)
    .setup(1.0, 2, 6, (true, true), |n, m| {
      (n as f64 / (m - 1) as f64, 1.0) }, tf); // 14
    tms!(tm, revolution, Revolution::<f64>, 1)
    .setup_from_tbl(1.0, 2, 6, (true, true), &vec![
      (0.0, 1.0), (0.2, 1.0), (0.4, 1.0), (0.6, 1.0), (0.8, 1.0)], tf); // 15
    for i in 0..2 { // (16, 17), (18, 19, 20, 21), (22, 23, 24, 25)
      let tf = i == 0; // true: on the one texture, false: texture each face
      tms!(tm, icosahedron, Icosahedron::<f64>, i).setup(r, tf);
      tms!(tm, dodecahedron, Dodecahedron::<f64>, i).setup(r, tf);
      tms!(tm, dodecahedron_center, DodecahedronCenter::<f64>, i).setup(r, tf);
      tms!(tm, c60, C60::<f64>, i).setup(r, tf);
      tms!(tm, c60_center, C60Center::<f64>, i).setup(r, tf);
    }
  });
  self.create_c60_icosahedron();
  self.create_c60_dodecahedron();
  self.create_c60_dodecahedron_center();
  self.create_c60_fullerene();
  self.create_c60_fullerene_center();
  self.current = self.create_polyhedron(self.nexpe as usize, self.pos);

  self.super_mut().start_callback();
}

fn near_callback(&mut self, dat: *mut c_void, o1: dGeomID, o2: dGeomID) {
  self.super_mut().near_callback(dat, o1, o2);

  let (info, info_sub) = (self.i, self.j); // tmp copy
  let rode = self.super_mut(); // must re get mut (for get_contacts)
  if rode.is_space(o1) || rode.is_space(o2) { return; } // skip when space
  let ground = rode.get_ground();
  if ground == o1 || ground == o2 { return; } // vs ground
  let _contactgroup = rode.get_contactgroup(); // now do nothing
  let n = rode.get_contacts(o1, o2);
  if n == 0 { return; } // skip no collision
  let contacts = rode.ref_contacts(); // or rode.ref_contacts_mut()
  let (b1p, _b1gp) = rode.get_ancestor(o1);
  let (b2p, _b2gp) = rode.get_ancestor(o2);
  if b1p == b2p { return; } // may not arrive here
  let skip = ["box", "p_", "plane", "slope", "apple", "ball", "roll",
    "fvp", "tmv", "tm"];
  let mut qpk: Vec<(dBodyID, dBodyID, String)> = vec![];
  let _ids = rode.each_id(|key, id| {
    for skp in skip {
      let (mut slen, klen) = (skp.len(), key.len());
      if klen < slen { slen = klen; }
      if key[..slen] == skp[..slen] { return false; }
    }
    if id == b1p { qpk.push((b2p, id, key.to_string())); return true; }
    if id == b2p { qpk.push((b1p, id, key.to_string())); return true; }
    false // lambda returns bool
  });
  if qpk.len() == 0 { return; } // when one of skip target
  if info {
    for (q, p, key) in &qpk { println!(" {:?} {:04} {:?} {}", q, n, p, key); }
    if info_sub {
      for (i, c) in contacts.iter().enumerate() {
        if i >= n as usize { break; }
        // &Vec<dContact> dContactGeom dGeomID dReal
        println!("  {:04} {:?}({:?}) {:?}({:?}) {:8.3e}",
          i,
          c.geom.g1, rode.get_grand_parent(c.geom.g1),
          c.geom.g2, rode.get_grand_parent(c.geom.g2),
          c.geom.depth);
      }
    }
  }
  // this code must be after contacts.iter() because of borrow mut self
  if qpk.len() == 2 { // may be always 2
    if qpk[0].0 != qpk[1].1 || qpk[0].1 != qpk[1].0 { return; }
    let (pk, qk) = (self.ts_rm(&qpk[0].2), self.ts_rm(&qpk[1].2));
    if pk != None && qk != None {
      let (pk, qk) = (pk.unwrap(), qk.unwrap());
      if pk == qk { // check same meta object
        self.ebps.push((qpk[0].1, qpk[1].1, pk));
      }
    }
  }
}

fn step_callback(&mut self, pause: i32) {
  self.objs_info(false, "step"); // twice (before draw)
  self.super_mut().step_callback(pause);
  for (q, p, k) in self.ebps.clone().into_iter() { // must clone and into_iter
    println!("disappear {:?} {:?} {}", q, p, k);
    let mut pos = vec![];
    let rode = self.super_mut(); // must mut (and in the loop)
    for o in [p, q] {
      pos.push(Obg::get_pos_mut_by_id(o));
      rode.unregister_obg_by_id(o, true);
    } // with destroy
    let c = avg_f4(&pos);
    // println!("{:?}", c);
    let u = match self.evo.get(&k) {
    None => self.u,
    Some(&u) => u as usize
    };
    let nk = self.create_polyhedron(u, c.try_into().unwrap()); // borrow temp
    self.kgc(&nk);
  }
  self.ebps.clear();
}

fn command_callback(&mut self, cmd: i32) {
  match cmd as u8 as char {
    '0' => { self.create_tmbunny(); },
    '1' => { self.create_tmtetra(); },
    '2' => { self.create_tmcube(); },
    '3' => { self.create_tmicosahedron(); },
    '4' => { self.create_tmball(); },
    '5' => { self.create_test_composite(); },
    '6' => { self.create_test_box_small(); },
    '7' => { self.create_c60_icosahedron(); },
    '8' => {
      self.create_c60_dodecahedron();
      self.create_c60_dodecahedron_center();
    },
    '9' => {
      self.create_c60_fullerene();
      self.create_c60_fullerene_center();
    },
    '@' => {
      let ck = self.current.clone(); // clone to skip borrow
      self.kgc(&ck);
      let u: usize = self.rng.gen();
      self.u = (self.u + u) % self.n;
      self.nexpe = PE::from_usize(self.u).unwrap();
      self.current = self.create_polyhedron(self.nexpe as usize, self.pos);
    },
    'h' => { self.pos[1] -= 0.1; self.trans(); }, // -Y left
    'j' => { self.pos[0] += 0.1; self.trans(); }, // +X front
    'k' => { self.pos[0] -= 0.1; self.trans(); }, // -X back
    'l' => { self.pos[1] += 0.1; self.trans(); }, // +Y right
    'c' => { self.i = !self.i; }, // collision info
    'x' => { self.j = !self.j; }, // collision info sub
    ' ' => {
      let k = "apple";
      match self.super_mut().find_mut(k.to_string()) {
        Err(e) => { println!("{}", e); },
        Ok(obg) => {
          if obg.is_enabled() { obg.disable(); } else { obg.enable(); }
        }
      }
    },
    't' => {
      for k in ["ball_big", "box_small",
        "apple", "roll", "tmball"] { // TODO: tmXX_timestamp will not be found
        match self.super_mut().find_mut(k.to_string()) {
          Err(e) => { println!("{}", e); },
          Ok(obg) => {
            // obg.add_rel_torque([-0.5, 0.0, 0.0]);
            obg.add_rel_force_rel([-0.5, 0.0, 0.0], [0.0, 0.0, 0.5]);
          }
        }
      }
      for k in ["ball", "tmbunny", "tmtetra",
        "tmcube", "tmicosahedron"] { // TODO: tmXX_timestamp will not be found
        match self.super_mut().find_mut(k.to_string()) {
          Err(e) => { println!("{}", e); },
          Ok(obg) => {
            // obg.add_rel_torque([0.0, 0.0, 0.5]);
            obg.add_rel_force_rel([0.0, 0.0, 0.5], [0.5, 0.0, 0.0]);
          }
        }
      }
    },
    'o' => {
      let k = "ball_big";
      match self.super_mut().find_mut(k.to_string()) {
        Err(e) => { println!("{}", e); },
        Ok(obg) => {
          println!("{}: {:018p} {:?}", k, obg.body(), obg.col);
          println!(" pos: {}", obg.pos_vec());
          println!(" rot: {}", obg.rot_mat3());
          let pos: &mut [dReal] = obg.pos_(); // re get mut
          pos[0] += 0.2;
          pos[1] += 0.2;
          pos[2] = 5.0;
        }
      }
    },
    'b' => {
      self.objs_mut(true, "mut");
    },
    'a' => {
      self.objs_info(true, "cmd");
    },
    '?' => {
      println!("{}", APP_HELP);
    },
    _ => {}
  }
  self.super_mut().command_callback(cmd);
}

} // impl Sim for SimApp

fn main() {
  any_pinned_init_slots!(16);
  any_pinned_set_bg_mut!(TriMeshManager<f64>, 0); // polyhedron sequence

  // default values
  // drawstuff: select drawstuff module
  // delta: dReal 0.002
  // QuickStepW: over_relaxation: dReal 1.3
  // QuickStepNumIterations: usize (c_int) 20
  // ContactMaxCorrectingVel: vel: dReal 1e-3 (1e-3, 1e-2, 0.0 or inf, ...)
  // ContactSurfaceLayer: depth: dReal 0.0
  // num_contact: 256
  ODE::open(Drawstuff::new(), 0.002, 1.3, 20, 1e-3, 0.0, 256);
  ODE::sim_loop(
    640, 480, // 800, 600,
    Some(Box::new(SimApp{
      phase: PHold, current: "".to_string(), pos: [-2.0, 0.0, 6.0, 1.0],
      nexpos: [-20.0, 0.0, 6.0, 1.0],
      nexpe: PE::ERSphere,
      rng: rand::thread_rng(),
      ped: vec![ERSphere, ETetra, ECubeCenter, EOcta, ECone],
      evo: vec![
        ("r_sphere", ETetra),
        ("tetra", ECubeCenter),
        ("cube_center", EOcta),
        ("octa", ECone),
        ("cone", EIcosahedronN0),
        ("icosahedron", EPin),
        ("pin", EDodecahedronCenterN0),
        ("dodecahedron_center", ERing),
        ("ring", EC60CenterN0),
        ("c60_center", ERSphere)
      ].into_iter().map(|(s, u)| (s.to_string(), u)).into_iter().collect(),
      ebps: vec![], i: false, j: false,
      t: time::Instant::now(), n: PE::End as usize, u: 0, cnt: 0})),
    b"./resources");
  ODE::close();

  any_pinned_dispose_slots!();
}
