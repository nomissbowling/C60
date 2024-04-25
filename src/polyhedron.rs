//! Polyhedron from Fullerene
//!

pub mod icosahedron;
pub use icosahedron::*;

pub mod dodecahedron;
pub use dodecahedron::*;

pub mod c60;
pub use c60::*;

use ode_rs::ode::*;

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
