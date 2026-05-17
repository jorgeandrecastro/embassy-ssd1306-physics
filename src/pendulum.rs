// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # pendulum
//!
//! Pendule simple : pivot fixe + tige + masse circulaire.
//!
//! ## Géométrie
//!
//! L'angle est mesuré **depuis la verticale descendante** (0 = au repos).
//! Angle positif → penche à droite.
//!
//! ```text
//!          ╫  ← pivot (croix 5px)
//!          │   angle = 0
//!         \│/  angle > 0 → droite
//!          ●   masse (disque r=3)
//! ```
//!
//! ## Dessin
//!
//! | Élément | Rendu              |
//! |---------|--------------------|
//! | Pivot   | Croix 5×5 pixels   |
//! | Tige    | Ligne simple       |
//! | Masse   | Disque plein r=3   |
//!
//! ## Exemple
//!
//! ```rust,no_run
//! use embassy_ssd1306_physics::Pendulum;
//! use embedded_trig_f32 as trig;
//!
//! let pend = Pendulum::new(64, 6, 28);
//! pend.draw(&mut gfx, 0.4, true, trig::cos, trig::sin);
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, filled_disk};

/// Pendule simple : pivot + tige + masse.
#[derive(Clone, Copy, Debug)]
pub struct Pendulum {
    /// X du pivot.
    pub pivot_x: i32,
    /// Y du pivot.
    pub pivot_y: i32,
    /// Longueur de la tige en pixels.
    pub length: i32,
}

impl Pendulum {
    /// Construit un pendule.
    #[inline]
    pub fn new(pivot_x: i32, pivot_y: i32, length: i32) -> Self {
        Self { pivot_x, pivot_y, length }
    }

    /// Dessine le pendule.
    ///
    /// - `angle` : angle depuis la verticale, en radians (0 = repos)
    /// - `on`    : `true` allume, `false` efface
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        // sin → déplacement X, cos → déplacement Y (pendule vers le bas)
        let mx = self.pivot_x + ((self.length as f32) * sin_fn(angle) + 0.5) as i32;
        let my = self.pivot_y + ((self.length as f32) * cos_fn(angle) + 0.5) as i32;

        // Tige
        segment(gfx, self.pivot_x, self.pivot_y, mx, my, on);

        // Pivot :croix 5×5
        gfx.pixel(self.pivot_x,     self.pivot_y,     on);
        gfx.pixel(self.pivot_x - 1, self.pivot_y,     on);
        gfx.pixel(self.pivot_x + 1, self.pivot_y,     on);
        gfx.pixel(self.pivot_x - 2, self.pivot_y,     on);
        gfx.pixel(self.pivot_x + 2, self.pivot_y,     on);
        gfx.pixel(self.pivot_x,     self.pivot_y - 1, on);
        gfx.pixel(self.pivot_x,     self.pivot_y + 1, on);

        // Masse
        filled_disk(gfx, mx, my, 3, on);
    }

    /// Efface le pendule.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle, false, cos_fn, sin_fn);
    }
}