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
    self.indices.clear();
    self.planes.clear();
    self.vtx.clear();
    self.polygons.clear();
    let mut cf = 0; // count faces (not use fi * f.len()) c60 contains 5 and 6
    for (_fi, f) in phf.iter().enumerate() {
      for _k in 0..4 { self.planes.push(0.0); } // flatten (auto set later)
      for (vi, v) in f.iter().enumerate() {
        let nv = v.len();
        self.polygons.push(nv as u32); // now all triangle
        for (ii, (p, uv)) in v.iter().enumerate() {
          let p = fullerene::f_to_f64(p);
          let _uv = fullerene::f_to_f64(uv);
          for k in 0..3 { self.vtx.push(p[k]); } // flatten
          let j = (cf + vi) * nv + ii;
          self.indices.push(j as dTriIndex);
          self.polygons.push(j as u32); // now all triangle
        }
      }
      cf += f.len();
    }
/*
    let nfaces = phf.len();
    self.planes = (0..nfaces).into_iter().flat_map(|_f|
      vec![0.0, 0.0, 0.0, 0.0]).collect(); // auto set later
*/
/*
    println!("{} indices\n{:?}", self.indices.len(), self.indices);
//    println!("{} planes\n{:?}", self.planes.len() / 4, self.planes);
//    println!("{} vtx\n{:?}", self.vtx.len() / 3, self.vtx);
    println!("{} polygons\n{:?}", self.polygons.len(), self.polygons);
*/
    self.tmv = self.to_trimeshvi();
    self.fvp = self.to_convexfvp();
  }
  /// to trimeshvi
  fn to_trimeshvi(&mut self) -> trimeshvi {
    trimeshvi::new(&mut self.vtx, &mut self.indices)
  }
  /// to convexfvp
  fn to_convexfvp(&mut self) -> convexfvp {
    convexfvp::new(&mut self.planes, &mut self.vtx, &mut self.polygons)
  }
}
