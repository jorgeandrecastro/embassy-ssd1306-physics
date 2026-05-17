// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # spring_mass
//!
//! Système ressort-masse vertical.
//!
//! Le ressort est ancré en haut (`anchor_x`, `anchor_y`) et s'étend
//! vers le bas. La masse est un rectangle plein dessiné sous le ressort.
//!
//! Le ressort est rendu par un zigzag integer-only (alternance ±amplitude).
//!
//! ## Dessin
//!
//! ```text
//!  ─── ancrage (trait horizontal 5px)
//!   |   ← trait droit haut 2px
//!  /\/\/\/  ← zigzag (N demi-spires, amplitude ±4px)
//!   |   ← trait droit bas 2px
//!  ┌───────┐ ← bloc masse (hauteur fixe 6px)
//!  └───────┘
//! ```
//!
//! ## Paramètres
//!
//! | Champ       | Description                                         |
//! |-------------|-----------------------------------------------------|
//! | `anchor_x`  | X du milieu de l'ancrage                            |
//! | `anchor_y`  | Y du point d'ancrage                                |
//! | `rest_len`  | Longueur du ressort au repos (pixels)               |
//! | `block_w`   | Largeur du bloc de masse (pixels, centré)           |
//! | `coils`     | Nombre de demi-spires (défaut : 6)                  |
//! | `amplitude` | Amplitude du zigzag ±pixels (défaut : 4)            |
//!
//! ## Exemple
//!
//! ```rust,no_run
//! use embassy_ssd1306_physics::SpringMass;
//!
//! let sm = SpringMass::new(64, 4, 22, 12);
//! sm.draw(&mut gfx, 6, true);   // ressort étiré de 6px
//! sm.draw(&mut gfx, -4, true);  // ressort comprimé de 4px
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::segment;

/// Système ressort-masse vertical.
#[derive(Clone, Copy, Debug)]
pub struct SpringMass {
    /// X du point d'ancrage (milieu).
    pub anchor_x: i32,
    /// Y du point d'ancrage (haut du ressort).
    pub anchor_y: i32,
    /// Longueur au repos en pixels.
    pub rest_len: i32,
    /// Largeur du bloc de masse en pixels.
    pub block_w: i32,
    /// Nombre de demi-spires du zigzag.
    pub coils: i32,
    /// Amplitude ±pixels des dents.
    pub amplitude: i32,
}

impl SpringMass {
    /// Construit un système ressort-masse avec coils=6, amplitude=4.
    #[inline]
    pub fn new(anchor_x: i32, anchor_y: i32, rest_len: i32, block_w: i32) -> Self {
        Self { anchor_x, anchor_y, rest_len, block_w, coils: 6, amplitude: 4 }
    }

    /// Construit un système avec nombre de spires et amplitude personnalisés.
    #[inline]
    pub fn with_coils(mut self, coils: i32, amplitude: i32) -> Self {
        self.coils = coils;
        self.amplitude = amplitude;
        self
    }

    /// Dessine le système.
    ///
    /// - `extension` : élongation signée en pixels (positif = étire, négatif = comprime)
    /// - `on`        : `true` allume, `false` efface
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        extension: i32,
        on: bool,
    ) {
        let total_len = (self.rest_len + extension).max(self.coils + 4);

        // Trait d'ancrage horizontal
        for dx in -(self.block_w / 2)..=(self.block_w / 2) {
            gfx.pixel(self.anchor_x + dx, self.anchor_y, on);
        }

        // Trait droit haut
        let spring_top = self.anchor_y + 2;
        segment(gfx, self.anchor_x, self.anchor_y, self.anchor_x, spring_top, on);

        // Zigzag
        let spring_bot = self.anchor_y + total_len - 2;
        if spring_bot > spring_top {
            let h = spring_bot - spring_top;
            let mut prev_x = self.anchor_x;
            let mut prev_y = spring_top;

            for i in 1..=self.coils {
                let ny = spring_top + h * i / self.coils;
                let nx = if i % 2 == 1 {
                    self.anchor_x + self.amplitude
                } else {
                    self.anchor_x - self.amplitude
                };
                segment(gfx, prev_x, prev_y, nx, ny, on);
                prev_x = nx;
                prev_y = ny;
            }
            // Retour au centre
            segment(gfx, prev_x, prev_y, self.anchor_x, spring_bot, on);
        }

        // Trait droit bas
        let mass_top = self.anchor_y + total_len;
        segment(gfx, self.anchor_x, spring_bot, self.anchor_x, mass_top, on);

        // Bloc de masse (rectangle plein, hauteur 6px)
        let block_h = 6i32;
        let half_w = self.block_w / 2;
        for dy in 0..block_h {
            for dx in -half_w..=half_w {
                gfx.pixel(self.anchor_x + dx, mass_top + dy, on);
            }
        }
    }

    /// Efface le système.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        extension: i32,
    ) {
        self.draw(gfx, extension, false);
    }
}