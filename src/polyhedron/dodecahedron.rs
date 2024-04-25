//! Trimesh Dodecahedron from Fullerene
//!

use anyslot::anyslot::*;
use num::Float;

use ode_rs::ode::*;

use crate::polyhedron::*;

/// Dodecahedron
#[derive(Debug, Clone)]
pub struct Dodecahedron<F: Float> {
  /// indices n * 3 flat (keep lifetime)
  pub indices: Vec<dTriIndex>,
  /// planes n * 4 flat (keep lifetime)
  pub planes: Vec<dReal>,
  /// vtx n * 3 flat (keep lifetime)
  pub vtx: Vec<dReal>,
  /// polygons n * (1 + 3) flat (keep lifetime)
  pub polygons: Vec<u32>,
  /// tmv (create from vtx, indices)
  pub tmv: trimeshvi,
  /// fvp (create from planes, vtx, polygons)
  pub fvp: convexfvp,
  /// ratio
  pub r: F
}

/// TBridgeGlobal for Dodecahedron
impl<F: Float> TBridgeGlobal for Dodecahedron<F> {
  /// constructor
  fn void() -> Self {
    let tmv = void_tmv();
    let fvp = void_fvp();
    let r = <F>::from(1).unwrap();
    Dodecahedron::<F>{
      indices: vec![],
      planes: vec![],
      vtx: vec![],
      polygons: vec![],
      tmv, fvp, r}
  }
}

/// Dodecahedron
impl<F: Float> Dodecahedron<F> {
  /// make trimeshvi and convexfvp
  pub fn setup(&mut self) {
    self.indices = vec![
      3, 1, 0,
      3, 2, 1,
      3, 0, 2,
      2, 0, 1];
    self.planes = vec![
      0.0, 0.0, -1.0, 0.2041,
      -0.9107, 0.2440, 0.3333, 0.2041,
      0.2440, -0.9107, 0.3333, 0.2041,
      0.6667, 0.6667, 0.3334, 0.2042];
    self.vtx = vec![
      0.5577, -0.1494, -0.2041,
      -0.1494, 0.5577, -0.2041,
      0.0, 0.0, 0.6124,
      -0.4082, -0.4082, -0.2041];
    self.polygons = vec![
      3, 3, 1, 0,
      3, 3, 2, 1,
      3, 3, 0, 2,
      3, 2, 0, 1];
    self.tmv = trimeshvi::new(&mut self.vtx, &mut self.indices);
    self.fvp = convexfvp::new(&mut self.planes, &mut self.vtx, &mut self.polygons);
  }
}
