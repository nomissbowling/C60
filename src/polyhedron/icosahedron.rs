//! Trimesh and Convex Icosahedron from Fullerene
//!

use anyslot::anyslot::*;
use num::Float;
use fullerene;
use crate::polyhedron::*;

/// Icosahedron
#[derive(Debug, Clone)]
pub struct Icosahedron<F: Float> {
  /// polyhedron
  pub ph: Polyhedron<F>
}

/// TBridgeGlobal for Icosahedron
impl<F: Float> TBridgeGlobal for Icosahedron<F> {
  /// constructor
  fn void() -> Self {
    Icosahedron::<F>{ph: Polyhedron::<F>::void()}
  }
}

/// Icosahedron
impl<F: Float> Icosahedron<F> {
  /// make trimeshvi and convexfvp
  /// - tf: true: on the one texture, false: texture each face
  pub fn setup(&mut self, r: F, tf: bool) {
    self.ph.from_phf(&fullerene::Icosahedron::new(r).with_uv(tf));
  }
}
