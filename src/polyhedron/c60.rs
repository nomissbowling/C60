//! Trimesh and Convex C60 from Fullerene
//!

use anyslot::anyslot::*;
use num::Float;
use fullerene;
use crate::polyhedron::*;

/// C60
#[derive(Debug, Clone)]
pub struct C60<F: Float> {
  /// polyhedron
  pub ph: Polyhedron<F>
}

/// TBridgeGlobal for C60
impl<F: Float> TBridgeGlobal for C60<F> {
  /// constructor
  fn void() -> Self {
    C60::<F>{ph: Polyhedron::<F>::void()}
  }
}

/// C60
impl<F: Float> C60<F> {
  /// make trimeshvi and convexfvp
  pub fn setup(&mut self, r: F, tf: bool) {
    self.ph.from_phf(&fullerene::C60::new(r).with_uv(tf));
  }
}

/// C60Center
#[derive(Debug, Clone)]
pub struct C60Center<F: Float> {
  /// polyhedron
  pub ph: Polyhedron<F>
}

/// TBridgeGlobal for C60Center
impl<F: Float> TBridgeGlobal for C60Center<F> {
  /// constructor
  fn void() -> Self {
    C60Center::<F>{ph: Polyhedron::<F>::void()}
  }
}

/// C60Center
impl<F: Float> C60Center<F> {
  /// make trimeshvi and convexfvp
  /// - tf: true: on the one texture, false: texture each face
  pub fn setup(&mut self, r: F, tf: bool) {
    self.ph.from_phf(&fullerene::C60Center::new(r).with_uv(tf));
  }
}
