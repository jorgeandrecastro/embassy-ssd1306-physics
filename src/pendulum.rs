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
//!  ▓▓▓▓▓▓▓▓▓  ← encastrement (rectangle plein w×h + hachures diagonales)
//!  ─────────  ← trait de séparation mur/pivot
//!      │       ← tige (angle = 0)
//!     \│/      ← tige déviée (angle > 0)
//!      ●       ← masse (disque plein r=3)
//! ```
//!
//! ## Dessin
//!
//! | Élément       | Rendu                                        |
//! |---------------|----------------------------------------------|
//! | Encastrement  | Rectangle plein `wall_w × wall_h` + hachures |
//! | Trait de base | Ligne horizontale sous le rectangle           |
//! | Pivot         | Petit disque plein r=1 (axe de rotation)     |
//! | Tige          | Ligne simple                                 |
//! | Masse         | Disque plein r=3                             |
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
use crate::draw_utils::{segment, filled_disk, filled_rect};

/// Pendule simple : encastrement mural + pivot + tige + masse.
#[derive(Clone, Copy, Debug)]
pub struct Pendulum {
    /// X du pivot (centre de rotation, milieu horizontal de l'encastrement).
    pub pivot_x: i32,
    /// Y du pivot (bord bas de l'encastrement, point d'attache de la tige).
    pub pivot_y: i32,
    /// Longueur de la tige en pixels.
    pub length: i32,
    /// Largeur de l'encastrement en pixels (centré sur `pivot_x`).
    pub wall_w: i32,
    /// Hauteur de l'encastrement en pixels.
    pub wall_h: i32,
}

impl Pendulum {
    /// Construit un pendule avec encastrement par défaut (wall_w=16, wall_h=5).
    #[inline]
    pub fn new(pivot_x: i32, pivot_y: i32, length: i32) -> Self {
        Self { pivot_x, pivot_y, length, wall_w: 16, wall_h: 5 }
    }

    /// Construit un pendule avec encastrement personnalisé.
    #[inline]
    pub fn with_wall(pivot_x: i32, pivot_y: i32, length: i32, wall_w: i32, wall_h: i32) -> Self {
        Self { pivot_x, pivot_y, length, wall_w, wall_h }
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
        let hw = self.wall_w / 2;
        let wall_x = self.pivot_x - hw;
        let wall_y = self.pivot_y - self.wall_h;

        //  Encastrement : rectangle plein
        filled_rect(gfx, wall_x, wall_y, self.wall_w, self.wall_h, on);

        //  Hachures diagonales ╲ (texture béton/mur) espacées de 4px
        // On éteint les pixels sur les diagonales pour créer la texture.
        let mut d = 0i32;
        while d < self.wall_w + self.wall_h {
            for dx in 0..self.wall_w {
                let dy = d - dx;
                if dy >= 0 && dy < self.wall_h {
                    gfx.pixel(wall_x + dx, wall_y + dy, !on);
                }
            }
            d += 4;
        }

        //  Trait de séparation horizontal sous l'encastrement
        segment(
            gfx,
            self.pivot_x - hw - 1, self.pivot_y,
            self.pivot_x + hw + 1, self.pivot_y,
            on,
        );

        // Tige (sin → X, cos → Y : pendule pend vers le bas)
        let mx = self.pivot_x + ((self.length as f32) * sin_fn(angle) + 0.5) as i32;
        let my = self.pivot_y + ((self.length as f32) * cos_fn(angle) + 0.5) as i32;
        segment(gfx, self.pivot_x, self.pivot_y, mx, my, on);

        //  Pivot : petit disque r=1 (axe de rotation visible)
        filled_disk(gfx, self.pivot_x, self.pivot_y, 1, on);

        //  Masse
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