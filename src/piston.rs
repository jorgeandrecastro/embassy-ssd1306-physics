// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # piston
//!
//! Piston 2D mécanique propre (sans gaz).
//!
//! Modèle physique simplifié :
//! - cylindre fermé (chambre)
//! - piston contraint dans la chambre
//! - tige guidée (liaison rigide)
//!
//! Objectif : base future pour moteur thermique / crankshaft.

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

use crate::draw_utils::{segment, filled_rect};

/// Piston 2D vertical contraint dans une chambre.
#[derive(Clone, Copy, Debug)]
pub struct Piston {
    /// Centre X du cylindre
    pub cx: i32,
    /// Haut du cylindre
    pub cy: i32,

    /// Largeur du cylindre
    pub w: i32,
    /// Hauteur totale du cylindre
    pub h: i32,

    /// Épaisseur du piston
    pub piston_h: i32,

    /// Position normalisée [0..h - piston_h]
    pub pos: i32,
}

impl Piston {
    /// Crée un piston.
    ///
    /// `cx, cy` : centre haut de la chambre
    /// `w`      : largeur du cylindre
    /// `h`      : hauteur de la chambre
    pub fn new(cx: i32, cy: i32, w: i32, h: i32) -> Self {
        Self {
            cx,
            cy,
            w,
            h,
            piston_h: 6,
            pos: 0,
        }
    }

    /// Force la position du piston dans la chambre.
    #[inline]
    pub fn set_pos(&mut self, pos: i32) {
        let max = self.h - self.piston_h;
        self.pos = pos.clamp(0, max);
    }

    /// Position “physique” normalisée (0.0 → haut, 1.0 → bas)
    #[inline]
    pub fn normalized(&self) -> f32 {
        let max = (self.h - self.piston_h).max(1);
        self.pos as f32 / max as f32
    }

    /// Dessine le système piston + chambre.
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        _t: f32,
        on: bool,
        _cos_fn: fn(f32) -> f32,
        _sin_fn: fn(f32) -> f32,
    ) {
        let left = self.cx - self.w / 2;
        let right = self.cx + self.w / 2;

        let top = self.cy;
        let bottom = self.cy + self.h;

        // ─────────────────────────────
        // CHAMBRE (rectangle fermé)
        // ─────────────────────────────
        segment(gfx, left, top, right, top, on);
        segment(gfx, left, bottom, right, bottom, on);
        segment(gfx, left, top, left, bottom, on);
        segment(gfx, right, top, right, bottom, on);

        // ─────────────────────────────
        // PISTON (contraint)
        // ─────────────────────────────
        let max = self.h - self.piston_h;
        let y = self.cy + self.pos.clamp(0, max);

        filled_rect(
            gfx,
            left + 2,
            y,
            self.w - 4,
            self.piston_h,
            on,
        );

        // ─────────────────────────────
        // TIGE GUIDÉE (sans débordement visuel)
        // ─────────────────────────────
        let rod_x = self.cx;
        let rod_top = y + self.piston_h / 2;
        let rod_bottom = bottom;

        segment(gfx, rod_x, rod_top, rod_x, rod_bottom, on);

        // ─────────────────────────────
        // CAPUCHON BAS (stabilité visuelle)
        // ─────────────────────────────
        segment(gfx, left, bottom, right, bottom, on);
    }

    /// Efface
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        t: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, t, false, cos_fn, sin_fn);
    }
}