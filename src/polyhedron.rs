//! Polyhedron from Fullerene
//!

pub mod icosahedron;
pub use icosahedron::*;

pub mod dodecahedron;
pub use dodecahedron::*;

pub mod c60;
pub use c60::*;

use ode_rs::ode::*;

use num::Float;
use fullerene::{self, TUV};

/// Polyhedron
#[derive(Debug, Clone)]
pub struct Polyhedron<F: Float> {
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

/// Polyhedron
impl<F: Float> Polyhedron<F> {
  /// constructor
  fn void() -> Self {
    let tmv = Polyhedron::<F>::void_tmv();
    let fvp = Polyhedron::<F>::void_fvp();
    let r = <F>::from(1).unwrap();
    Polyhedron::<F>{
      indices: vec![],
      planes: vec![],
      vtx: vec![],
      polygons: vec![],
      tmv, fvp, r}
  }
  /// make void tmv (this should be implemented trait for trimeshvi)
  /// - vtx n * 3 flat (keep lifetime)
  /// - indices n * 3 flat (keep lifetime)
  pub fn void_tmv() -> trimeshvi {
    trimeshvi{
      vtxCount: 0,
      vtx: 0 as *mut dReal,
      indices: 0 as *mut dTriIndex,
      indexCount: 0}
  }
  /// make void fvp (this should be implemented trait for convexfvp)
  /// - planes n * 4 flat (keep lifetime)
  /// - vtx n * 3 flat (keep lifetime)
  /// - polygons n * (1 + 3) flat (keep lifetime)
  pub fn void_fvp() -> convexfvp {
    convexfvp{
      faceCount: 0,
      faces: 0 as *mut dReal,
      vtxCount: 0,
      vtx: 0 as *mut dReal,
      polygons: 0 as *mut u32}
  }
  /// polyhedron from PHF
  fn from_phf(&mut self, phf: &fullerene::PHF<F>) {
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
