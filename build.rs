/*
  build.rs for c60

  (skip C/C++ Bridge)

  cc-rs bindgen
  and generate link option
*/

extern crate cc;
extern crate bindgen;

use std::path::PathBuf;

fn main() {
  let s_path: String = if cfg!(docsrs) {
    std::env::var("OUT_DIR").unwrap()
  }else{
    ".".to_string()
  }; // to keep lifetime
  let o_path: &str = s_path.as_str();
  let c_opt = if o_path == "." { "-std:c11" }else{ "-std=c++11" };

  // default bindgen should not be inline otherwise link error
  // cc with no default inline when bindgen .generate_inline_functions(true)
  // -fno-implement-inlines : #pragma implementation
  // -fkeep-inline-functions : ***CAUTION***
  // -finline-functions : assume all inline ***CAUTION***
  // -fno-inline : ignore all inline ***CAUTION***
  // -fno-default-inline : not assume inline defined in the class
  // let c_inl = if o_path == "." { "-Ob0" }else{ "-fno-default-inline" };
  // let c_asm = if o_path == "." { "-Fa" }else{ "-S" };

  let _mk_cc = |dname: &str, sname: &str, iname: &str, oname: &str| {
    let sd = PathBuf::from(dname);
    let fname = format!("{}", sd.join(sname).to_str().expect("invalid path"));
    println!("cargo:rerun-if-changed={}", fname);
    cc::Build::new()
      .file(fname)
      .flag(c_opt)
      // .flag("-std=c++11") // gcc
      // .flag("-std:c11") // cl
      // .flag("-std:c++14") // cl
      // .flag(c_inl)
      // .flag(c_asm)
      .include(iname)
      .compile(oname)
  };
/*
  mk_cc("./src", "bridge.cpp", "./include", "bridge");
*/

  let _mk_bindings = |hdd: &str, header: &str, rsd: &str, rsfile: &str,
    binl: bool, bcmt: bool| { // inline, comment
    let hd = PathBuf::from(hdd);
    let hf = format!("{}", hd.join(header).to_str().expect("invalid path"));
    println!("cargo:rerun-if-changed={}", hf);
    let bindings = bindgen::Builder::default()
      .header(hf)
      .generate_inline_functions(binl) // default: false (.hpp inline if true)
      .generate_comments(bcmt) // default: true
      .parse_callbacks(Box::new(bindgen::CargoCallbacks))
      .generate()
      .expect("Unable to generate bindings");
    let rs = PathBuf::from(rsd);
    bindings
      .write_to_file(rs.join(rsfile))
      .expect("Could not write bindings!");
  };
/*
  if o_path == "." {
    mk_bindings("./include", "bridge.hpp", "./include", "bridge_bindings.rs",
      false, true); // cc should be compiled with option no default inline
    mk_bindings("./ode", "drawstuff.h", "./ode", "drawstuff_bindings.rs",
      false, true);
    mk_bindings("./ode", "ode.hpp", "./ode", "ode_bindings.rs",
      false, true);
  }
*/

  println!("cargo:rustc-link-search=./ode/lib");
  println!("cargo:rustc-link-lib=drawstuff");
  println!("cargo:rustc-link-lib=ode");
}
