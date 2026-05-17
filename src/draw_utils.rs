// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # draw_utils
//!
//! Primitives de dessin partagées entre tous les modules physiques.
//!
//! Ces fonctions sont de simples adaptateurs vers `embassy-ssd1306-graphics`
//! qui évitent la répétition de `use` dans chaque module objet.

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

/// Trace une ligne entre deux points (Bresenham via graphics).
#[inline]
pub fn segment<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x0: i32, y0: i32,
    x1: i32, y1: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::line(gfx, x0, y0, x1, y1, on);
}

/// Dessine un disque plein (articulation, masse, moyeu).
#[inline]
pub fn filled_disk<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::fill_circle(gfx, cx, cy, r, on);
}

/// Dessine un cercle creux (cadran, jante).
#[inline]
pub fn ring<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::circle(gfx, cx, cy, r, on);
}

/// Dessine un rectangle plein (bloc masse, dent d'engrenage).
#[inline]
pub fn filled_rect<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x: i32, y: i32, w: i32, h: i32,
    on: bool,
) {
    // Scanline simple : draw_filled_rect du driver fonctionne en u8,
    // on passe par pixel() pour rester en i32 avec clipping.
    for dy in 0..h {
        for dx in 0..w {
            gfx.pixel(x + dx, y + dy, on);
        }
    }
}

/// Calcule l'extrémité d'un vecteur polaire.
///
/// Depuis `(ox, oy)`, longueur `len`, direction `(cos_a, sin_a)`.
#[inline]
pub fn polar_end(ox: i32, oy: i32, len: i32, cos_a: f32, sin_a: f32) -> (i32, i32) {
    let x = ox + ((len as f32) * cos_a + 0.5) as i32;
    let y = oy + ((len as f32) * sin_a + 0.5) as i32;
    (x, y)
}

/// Trace un segment épaissi (ligne + décalage d'un pixel).
///
/// L'axe de décalage est perpendiculaire à la direction dominante.
#[inline]
pub fn thick_segment<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x0: i32, y0: i32,
    x1: i32, y1: i32,
    on: bool,
) {
    segment(gfx, x0, y0, x1, y1, on);
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    if dx >= dy {
        segment(gfx, x0, y0 + 1, x1, y1 + 1, on);
    } else {
        segment(gfx, x0 + 1, y0, x1 + 1, y1, on);
    }
}