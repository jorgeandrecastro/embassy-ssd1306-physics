// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # robotic_arm
//!
//! Bras robotique plan à deux segments rigides.
//!
//! ## Géométrie
//!
//! ```text
//!   base ──── seg1 ──── coude ──── seg2 ──── effecteur
//!          θ_shoulder             θ_elbow (relatif à seg1)
//! ```
//!
//! Les angles sont mesurés depuis l'axe **X+ horizontal** (écran,
//! Y croît vers le bas). Angle positif → rotation vers Y+.
//!
//! ## Dessin
//!
//! | Élément          | Rendu                            |
//! |------------------|----------------------------------|
//! | Base             | Disque plein r=3                 |
//! | Coude            | Disque plein r=2                 |
//! | Effecteur        | Disque plein r=2                 |
//! | Segments         | Ligne épaissie (×2 lignes)       |
//!
//! ## Exemple
//!
//! ```rust,no_run
//! use embassy_ssd1306_physics::RoboticArm;
//! use embedded_trig_f32 as trig;
//!
//! let arm = RoboticArm::new(20, 50, 28, 18);
//! arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
//! arm.erase(&mut gfx, 0.785, -0.524, trig::cos, trig::sin);
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{polar_end, thick_segment, filled_disk};

/// Bras robotique plan à deux segments (épaule + coude).
#[derive(Clone, Copy, Debug)]
pub struct RoboticArm {
    /// X de la base (pixel).
    pub base_x: i32,
    /// Y de la base (pixel).
    pub base_y: i32,
    /// Longueur du segment épaule → coude (pixels).
    pub seg1_len: i32,
    /// Longueur du segment coude → effecteur (pixels).
    pub seg2_len: i32,
}

impl RoboticArm {
    /// Construit un bras robotique.
    ///
    /// - `base_x`, `base_y` : position de la base
    /// - `seg1_len` : longueur épaule → coude
    /// - `seg2_len` : longueur coude → effecteur
    #[inline]
    pub fn new(base_x: i32, base_y: i32, seg1_len: i32, seg2_len: i32) -> Self {
        Self { base_x, base_y, seg1_len, seg2_len }
    }

    /// Dessine le bras.
    ///
    /// - `angle_shoulder` : angle seg1 en radians depuis X+ (0 = horizontal)
    /// - `angle_elbow`    : angle seg2 **relatif** à seg1
    /// - `on`             : `true` allume, `false` efface
    /// - `cos_fn`, `sin_fn` : fonctions trig (ex. `embedded_trig_f32::cos`)
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let cs = cos_fn(angle_shoulder);
        let ss = sin_fn(angle_shoulder);

        let (ex, ey) = polar_end(self.base_x, self.base_y, self.seg1_len, cs, ss);

        let abs_elbow = angle_shoulder + angle_elbow;
        let ce = cos_fn(abs_elbow);
        let se = sin_fn(abs_elbow);

        let (fx, fy) = polar_end(ex, ey, self.seg2_len, ce, se);

        // Segments épaissis
        thick_segment(gfx, self.base_x, self.base_y, ex, ey, on);
        thick_segment(gfx, ex, ey, fx, fy, on);

        // Articulations
        filled_disk(gfx, self.base_x, self.base_y, 3, on);
        filled_disk(gfx, ex, ey, 2, on);
        filled_disk(gfx, fx, fy, 2, on);
    }

    /// Efface le bras (alias `draw(..., false)`).
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle_shoulder, angle_elbow, false, cos_fn, sin_fn);
    }
}