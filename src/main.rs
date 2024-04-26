#![doc(html_root_url = "https://docs.rs/c60/0.0.6")]
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

use trimesh::polyhedron::{
  Icosahedron,
  {Dodecahedron, DodecahedronCenter},
  {C60, C60Center}};

use anyslot::anyslot::*;

use ode_rs::ds::Drawstuff;
use ode_rs::colors::*;
use ode_rs::ode::*;

use std::ffi::{c_void}; // used by impl_sim_fn
use impl_sim::{impl_sim_fn, impl_sim_derive};

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
  ' ': drop apple ball
  't': torque
  'o': big ball info
  'b': test mut (big ball)
  'a': test cmd (all info)";

pub struct SimApp {
  cnt: usize
}

impl SimApp {

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
  let (body, _, _) = self.super_mut().creator("capsule_0", micap_0);
  self.set_pos_R(body, [-8.6, 0.0, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([1.0, 0.0, 0.0], PIh));

  let micap_1 = MetaCapsule::new(0.001, 0.5, 16.0,
    KRP080, 0, [0.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("capsule_1", micap_1);
  self.set_pos_R(body, [8.6, 0.0, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([1.0, 0.0, 0.0], PIh));
}

/// create test cylinder frames
pub fn create_test_cylinder_frames(&mut self) {
  let micyl_0 = MetaCylinder::new(0.001, 0.5, 16.0,
    KRP080, 0, [1.0, 0.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("cylinder_0", micyl_0);
  self.set_pos_R(body, [0.0, 8.6, 1.5, 1.0],
    dMatrix3::from_axis_and_angle([0.0, 1.0, 0.0], PIh));

  let micyl_1 = MetaCylinder::new(0.001, 0.5, 16.0,
    KRP080, 0, [0.0, 1.0, 1.0, 0.8]);
  let (body, _, _) = self.super_mut().creator("cylinder_1", micyl_1);
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
  let (body, _, _) = self.super_mut().creator_composite("tmball", mi_tmball);
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
  let krp = Krp::new(true, false, true, 0.95);
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
  let (body, _, _) = self.super_mut().creator("tmtetra", mi_tmtetra);
  self.set_pos_Q(body, [-15.0, -1.5, 0.5, 1.0], QI);
}

/// create
pub fn create_tmcube(&mut self) {
  let mi_tmcube = MetaTriMesh::new(false, 1.0, unsafe { &mut *cube::tmv },
    KRP095, 0, [0.6, 0.8, 0.2, 1.0]);
  let (body, _, _) = self.super_mut().creator("tmcube", mi_tmcube);
  self.set_pos_Q(body, [-16.5, -3.0, 0.5, 1.0],
    dQuaternion::from_axis_and_angle([1.0, 1.0, 1.0], PIq));
}

/// create
pub fn create_tmicosahedron(&mut self) {
  let mi_tmih = MetaTriMesh::new(false, 1.0, unsafe { &mut *icosahedron::tmv },
    KRP095, 0, [0.2, 0.8, 0.6, 1.0]);
  let (body, _, _) = self.super_mut().creator("tmicosahedron", mi_tmih);
  self.set_pos_Q(body, [-16.5, 3.0, 0.5, 1.0], QI);
}

/// create
pub fn create_tmbunny(&mut self) {
  let mi_tmbunny = MetaTriMesh::new(false, 1.0, unsafe { &mut *bunny::tmv },
    KRP095, 0, [0.8, 0.2, 0.6, 1.0]);
  let (body, _, _) = self.super_mut().creator("tmbunny", mi_tmbunny);
  // phi=-x, theta=-y, psi=-z
  let m = dMatrix3::from_euler_angles(-PIh, 0.0, 0.0);
  self.set_pos_Q(body, [-15.0, 0.25, 0.88, 1.0], dQuaternion::from_R(m));
  // to (-0.109884, 0.304591, 1.217693)
}

/// create c60 icosahedron
pub fn create_c60_icosahedron(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(Icosahedron<f64>, 0 + i, |icosa| {
      let mi_tm_icosa = MetaTriMesh::new(false, 1.0, &mut icosa.ph.tmv,
        KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
      let s = format!("c60 icosahedron {}", i);
      let (body, _, _) = self.super_mut().creator(s.as_str(), mi_tm_icosa);
      self.set_pos_Q(body, [-4.0 + 2.0 * i as f64, 0.0, 2.0, 1.0], QI);
    });
  }
}

/// create c60 dodecahedron
pub fn create_c60_dodecahedron(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(Dodecahedron<f64>, 2 + i, |dodeca| {
      let mi_tm_dodeca = MetaTriMesh::new(false, 1.0, &mut dodeca.ph.tmv,
        KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
      let s = format!("c60 dodecahedron {}", i);
      let (body, _, _) = self.super_mut().creator(s.as_str(), mi_tm_dodeca);
      self.set_pos_Q(body, [-4.0 + 2.0 * i as f64, -4.0, 2.0, 1.0], QI);
    });
  }
}

/// create c60 dodecahedron center
pub fn create_c60_dodecahedron_center(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(DodecahedronCenter<f64>, 4 + i, |dodecac| {
      let mi_tm_dodecac = MetaTriMesh::new(false, 1.0, &mut dodecac.ph.tmv,
        KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
      let s = format!("c60 dodecahedron center {}", i);
      let (body, _, _) = self.super_mut().creator(s.as_str(), mi_tm_dodecac);
      self.set_pos_Q(body, [-4.0 + 2.0 * i as f64, -2.0, 2.0, 1.0], QI);
    });
  }
}

/// create c60 fullerene
pub fn create_c60_fullerene(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(C60<f64>, 6 + i, |c60| {
      let mi_tm_c60 = MetaTriMesh::new(false, 1.0, &mut c60.ph.tmv,
        KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
      let s = format!("c60 fullerene {}", i);
      let (body, _, _) = self.super_mut().creator(s.as_str(), mi_tm_c60);
      self.set_pos_Q(body, [-4.0 + 2.0 * i as f64, 4.0, 2.0, 1.0], QI);
    });
  }
}

/// create c60 fullerene center
pub fn create_c60_fullerene_center(&mut self) {
  for i in 0..2 {
    any_pinned_with_bg_mut!(C60Center<f64>, 8 + i, |c60c| {
      let mi_tm_c60c = MetaTriMesh::new(false, 1.0, &mut c60c.ph.tmv,
        KRP095, 0, [0.8, 0.6, 0.2, 1.0]);
      let s = format!("c60 fullerene center {}", i);
      let (body, _, _) = self.super_mut().creator(s.as_str(), mi_tm_c60c);
      self.set_pos_Q(body, [-4.0 + 2.0 * i as f64, 2.0, 2.0, 1.0], QI);
    });
  }
}

}

#[impl_sim_derive(draw_geom, near_callback, stop_callback)]
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
  self.create_test_plane();

  self.create_tmball();
  self.create_slope();
  self.create_sphere_apple();
  self.create_sphere_ball(); self.create_sphere_roll();
  self.create_tmtetra();
  self.create_tmcube();
  self.create_tmicosahedron();
  self.create_tmbunny();

  for i in 0..2 {
    let tf = i == 0; // true: on the one texture, false: texture each face
    any_pinned_with_bg_mut!(Icosahedron<f64>, 0 + i, |icosa| {
      icosa.setup(0.2, tf);
    });
    any_pinned_with_bg_mut!(Dodecahedron<f64>, 2 + i, |dodeca| {
      dodeca.setup(0.2, tf);
    });
    any_pinned_with_bg_mut!(DodecahedronCenter<f64>, 4 + i, |dodecac| {
      dodecac.setup(0.2, tf);
    });
    any_pinned_with_bg_mut!(C60<f64>, 6 + i, |c60| {
      c60.setup(0.2, tf);
    });
    any_pinned_with_bg_mut!(C60Center<f64>, 8 + i, |c60c| {
      c60c.setup(0.2, tf);
    });
  }
  self.create_c60_icosahedron();
  self.create_c60_dodecahedron();
  self.create_c60_dodecahedron_center();
  self.create_c60_fullerene();
  self.create_c60_fullerene_center();

  self.super_mut().start_callback();
}

fn step_callback(&mut self, pause: i32) {
  self.objs_info(false, "step"); // twice (before draw)
  self.super_mut().step_callback(pause);
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
        "apple", "roll", "tmball"] {
        match self.super_mut().find_mut(k.to_string()) {
          Err(e) => { println!("{}", e); },
          Ok(obg) => {
            // obg.add_rel_torque([-0.5, 0.0, 0.0]);
            obg.add_rel_force_rel([-0.5, 0.0, 0.0], [0.0, 0.0, 0.5]);
          }
        }
      }
      for k in ["ball", "tmbunny", "tmtetra",
        "tmcube", "tmicosahedron"] {
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
  any_pinned_set_bg_mut!(Icosahedron<f64>, 0);
  any_pinned_set_bg_mut!(Icosahedron<f64>, 1);
  any_pinned_set_bg_mut!(Dodecahedron<f64>, 2);
  any_pinned_set_bg_mut!(Dodecahedron<f64>, 3);
  any_pinned_set_bg_mut!(DodecahedronCenter<f64>, 4);
  any_pinned_set_bg_mut!(DodecahedronCenter<f64>, 5);
  any_pinned_set_bg_mut!(C60<f64>, 6);
  any_pinned_set_bg_mut!(C60<f64>, 7);
  any_pinned_set_bg_mut!(C60Center<f64>, 8);
  any_pinned_set_bg_mut!(C60Center<f64>, 9);

  ODE::open(Drawstuff::new());
  ODE::sim_loop(
    640, 480, // 800, 600,
    Some(Box::new(SimApp{cnt: 0})),
    b"./resources");
  ODE::close();

  any_pinned_dispose_slots!();
}
