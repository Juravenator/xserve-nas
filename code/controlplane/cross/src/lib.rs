#![cfg_attr(not(test), no_std)]
#![feature(
    type_alias_impl_trait,
    generic_const_exprs,
    iter_array_chunks,
    generic_arg_infer
)]

#[macro_use]
extern crate arrayref;

pub mod led;
pub mod led_driver;
pub mod shift_register;
