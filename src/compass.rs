// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # compass
//!
//! Boussole avec cadran, points cardinaux et aiguille bicolore.
//!
//! ## Géométrie
//!
//! L'angle (`heading`) est mesuré depuis le **Nord** (haut de l'écran,
//! Y décroissant). Il croît dans le sens horaire.
//!
//! ```text
//!       N  (pixel intérieur)
//!    ┌──┼──┐
//!  O ┤  ↑  ├ E    ← heading = 0 (aiguille pointe Nord)
//!    └──┼──┘
//!       S
//! ```
//!
//! ## Dessin
//!
//! | Élément         | Rendu                                          |
//! |-----------------|------------------------------------------------|
//! | Cadran          | Cercle creux (rayon = `radius`)                |
//! | Points cardinaux| 1 pixel à r-2 sur chaque axe                   |
//! | Aiguille Nord   | Segment épaissi vers la pointe                 |
//! | Aiguille Sud    | Segment simple (queue)                         |
//! | Centre          | Disque plein r=1                               |
//!
//! ## Exemple
//!
//! ```rust,no_run
//! use embassy_ssd1306_physics::Compass;
//! use embedded_trig_f32 as trig;
//!
//! let compass = Compass::new(112, 32, 14);
//! compass.draw(&mut gfx, 0.785, true, trig::cos, trig::sin); // NE
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, filled_disk, ring};

/// Boussole : cadran + points cardinaux + aiguille.
#[derive(Clone, Copy, Debug)]
pub struct Compass {
    /// X du centre.
    pub cx: i32,
    /// Y du centre.
    pub cy: i32,
    /// Rayon du cadran en pixels.
    pub radius: i32,
}

impl Compass {
    /// Construit une boussole.
    #[inline]
    pub fn new(cx: i32, cy: i32, radius: i32) -> Self {
        Self { cx, cy, radius }
    }

    /// Dessine la boussole.
    ///
    /// `heading` : cap en radians depuis le Nord, sens horaire
    /// `on`      : `true` allume, `false` efface
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        heading: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let r = self.radius;

        // Cadran
        ring(gfx, self.cx, self.cy, r, on);

        // Points cardinaux (pixels à r-2)
        let inner = r - 2;
        gfx.pixel(self.cx,         self.cy - inner, on); // N
        gfx.pixel(self.cx + inner, self.cy,         on); // E
        gfx.pixel(self.cx,         self.cy + inner, on); // S
        gfx.pixel(self.cx - inner, self.cy,         on); // O

        let sh = sin_fn(heading);
        let ch = cos_fn(heading);

        // Aiguille Nord (épaisse, jusqu'à r-3)
        let needle = r - 3;
        let nx = self.cx + ((needle as f32) * sh + 0.5) as i32;
        let ny = self.cy - ((needle as f32) * ch + 0.5) as i32;

        segment(gfx, self.cx, self.cy, nx, ny, on);
        // Épaississement perpendiculaire
        let dx = (nx - self.cx).abs();
        let dy = (ny - self.cy).abs();
        if dx >= dy {
            segment(gfx, self.cx, self.cy + 1, nx, ny + 1, on);
        } else {
            segment(gfx, self.cx + 1, self.cy, nx + 1, ny, on);
        }

        // Queue Sud (simple, jusqu'à r/2-1)
        let tail = (r / 2) - 1;
        let tx = self.cx - ((tail as f32) * sh + 0.5) as i32;
        let ty = self.cy + ((tail as f32) * ch + 0.5) as i32;
        segment(gfx, self.cx, self.cy, tx, ty, on);

        // Pivot central
        filled_disk(gfx, self.cx, self.cy, 1, on);
    }

    /// Efface la boussole.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        heading: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, heading, false, cos_fn, sin_fn);
    }
}