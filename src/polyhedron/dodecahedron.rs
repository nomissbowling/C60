//! Trimesh and Convex Dodecahedron from Fullerene
//!

use anyslot::anyslot::*;
use num::Float;
use fullerene;
use crate::polyhedron::*;

/// Dodecahedron
#[derive(Debug, Clone)]
pub struct Dodecahedron<F: Float> {
  /// polyhedron
  pub ph: Polyhedron<F>
}

/// TBridgeGlobal for Dodecahedron
impl<F: Float> TBridgeGlobal for Dodecahedron<F> {
  /// constructor
  fn void() -> Self {
    Dodecahedron::<F>{ph: Polyhedron::<F>::void()}
  }
}

/// Dodecahedron
impl<F: Float> Dodecahedron<F> {
  /// make trimeshvi and convexfvp
  pub fn setup(&mut self, r: F, tf: bool) {
    self.ph.from_phf(&fullerene::Dodecahedron::new(r).with_uv(tf));
  }
}

/// DodecahedronCenter
#[derive(Debug, Clone)]
pub struct DodecahedronCenter<F: Float> {
  /// polyhedron
  pub ph: Polyhedron<F>
}

/// TBridgeGlobal for DodecahedronCenter
impl<F: Float> TBridgeGlobal for DodecahedronCenter<F> {
  /// constructor
  fn void() -> Self {
    DodecahedronCenter::<F>{ph: Polyhedron::<F>::void()}
  }
}

/// DodecahedronCenter
impl<F: Float> DodecahedronCenter<F> {
  /// make trimeshvi and convexfvp
  /// - tf: true: on the one texture, false: texture each face
  pub fn setup(&mut self, r: F, tf: bool) {
    self.ph.from_phf(&fullerene::DodecahedronCenter::new(r).with_uv(tf));
  }
}
