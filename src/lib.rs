// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

//! # embassy-ssd1306-physics
//!
//! Dessins 2D `no_std` d'objets physiques pour écrans OLED SSD1306 (128×64),
//! construite au-dessus de `[`embassy-ssd1306-graphics`]`.
//!
//! ## Objets disponibles
//!
//! | Module          | Struct         | Description                                  |
//! |-----------------|----------------|----------------------------------------------|
//! | [`robotic_arm`] | [`RoboticArm`] | Bras industriel 2D (socle + pince pneumatique)  |
//! | [`pendulum`]    | [`Pendulum`]   | Pendule simple (encastrement + tige + masse) |
//! | [`spring_mass`] | [`SpringMass`] | Système ressort-masse vertical               |
//! | [`compass`]     | [`Compass`]    | Boussole (cadran + cardinaux + aiguille)     |
//! | [`gear`]        | [`Gear`]       | Engrenage (dents + moyeu creux)              |
//! | [`piston`]       | [`Piston`]      | Piston 2D mécanique (chambre + tige guidée)                    |
//!
//! ## Patron sin/cos injecté
//!
//! Chaque `draw()` accepte `cos_fn` et `sin_fn` de type `fn(f32) -> f32`.
//! Passez votre crate trig `no_std` :
//!
//! ```rust,no_run
//! use embedded_trig_f32 as trig;
//! arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
//! ```
//!
//! ## Ajouter un nouvel objet
//!
//! 1. Créer `src/mon_objet.rs` : impl struct + `draw()` + `erase()`
//! 2. Déclarer `pub mod mon_objet;` dans `src/lib.rs`
//! 3. Ajouter `pub use mon_objet::MonObjet;` si re-export souhaité
//!
//! Aucun autre fichier à modifier.

pub mod draw_utils;
pub mod robotic_arm;
pub mod pendulum;
pub mod spring_mass;
pub mod compass;
pub mod gear;
pub mod piston;


pub use robotic_arm::RoboticArm;
pub use pendulum::Pendulum;
pub use spring_mass::SpringMass;
pub use compass::Compass;
pub use gear::Gear;
pub use gear::GearPair;
pub use gear::GearTrain;
pub use piston::Piston;
pub use robotic_arm::Facing;