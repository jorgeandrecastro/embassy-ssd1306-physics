// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # gear
//!
//! Engrenage complet (dents + corps + moyeu + clavetage optionnel).
//!
//! ## Géométrie d'un engrenage
//!
//! ```text
//!           ┌─┐ ┌─┐ ┌─┐   ← dents (tooth_h pixels de haut)
//!         ──┘ └─┘ └─┘ └──
//!        /                \
//!       │   ┌───┐  corps   │  ← cercle primitif (rayon = pitch_r)
//!       │   │ ◎ │  moyeu   │  ← cercle intérieur (rayon = hub_r)
//!       │   └───┘          │
//!        \                /
//!         ──┐ ┌─┐ ┌─┐ ┌──
//!           └─┘ └─┘ └─┘
//! ```
//!
//! ### Paramètres principaux
//!
//! | Champ       | Description                                              |
//! |-------------|----------------------------------------------------------|
//! | `cx`, `cy`  | Centre de l'engrenage                                    |
//! | `pitch_r`   | Rayon primitif (racine des dents), pixels                |
//! | `teeth`     | Nombre de dents                                          |
//! | `tooth_h`   | Hauteur d'une dent (addendum), pixels                    |
//! | `tooth_w`   | Largeur angulaire d'une dent en % du pas (0–100)         |
//! | `hub_r`     | Rayon du moyeu intérieur (0 = pas de moyeu)              |
//!
//! ### Algorithme des dents
//!
//! Pour chaque dent `i ∈ [0, teeth[` :
//! 1. Calculer l'angle du centre de la dent : `θ = angle + i * (TAU / teeth)`
//! 2. Calculer les 4 coins de la dent rectangulaire en coordonnées polaires
//!    (addendum = pitch_r + tooth_h, dédendum ≈ pitch_r).
//! 3. Tracer les 4 segments du rectangle de la dent.
//! 4. Tracer l'arc du corps entre deux dents consécutives via `circle()` masqué
//!    (approché par des pixels du cercle dans l'intervalle angulaire).
//!
//! Tout est integer-only via virgule fixe ×1024 pour les coordonnées.
//!
//! ## Engrenages appariés
//!
//! [`GearPair`] calcule automatiquement la position du second engrenage
//! pour qu'il soit en prise avec le premier (centre à `pitch_r1 + pitch_r2`).
//!
//! La vitesse de rotation du second engrenage est inversée et réduite
//! dans le rapport `teeth1 / teeth2`.
//!
//! ## Exemple
//!
//! ```rust,no_run
//! use embassy_ssd1306_physics::Gear;
//! use embedded_trig_f32 as trig;
//!
//! // Engrenage simple
//! let g = Gear::new(40, 32, 14, 10, 4, 70, 4);
//! g.draw(&mut gfx, 0.0, true, trig::cos, trig::sin);
//!
//! // Paire engrenée
//! use embassy_ssd1306_physics::gear::GearPair;
//! let pair = GearPair::new(
//!     Gear::new(30, 32, 12, 8, 4, 70, 3),  // moteur
//!     Gear::new(0, 0, 18, 12, 4, 70, 5),   // récepteur (cx/cy ignorés)
//!     0,                                     // axe de contact : 0 = droite
//!     trig::cos, trig::sin,
//! );
//! pair.draw(&mut gfx, 0.3, true, trig::cos, trig::sin);
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, ring, filled_disk};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes virgule fixe
// ─────────────────────────────────────────────────────────────────────────────

/// TAU × 1024 (virgule fixe ×1024 pour éviter les flottants dans les boucles).
const TAU_FP: i32 = 6434; // 2π × 1024 ≈ 6433.98 → 6434

// ─────────────────────────────────────────────────────────────────────────────
// Gear
// ─────────────────────────────────────────────────────────────────────────────

/// Engrenage complet : corps circulaire + dents rectangulaires + moyeu.
#[derive(Clone, Copy, Debug)]
pub struct Gear {
    /// X du centre.
    pub cx: i32,
    /// Y du centre.
    pub cy: i32,
    /// Rayon primitif (base des dents) en pixels.
    pub pitch_r: i32,
    /// Nombre de dents.
    pub teeth: i32,
    /// Hauteur d'une dent (addendum) en pixels.
    pub tooth_h: i32,
    /// Largeur de la dent en % du pas angulaire (1–99). 70 = dent large.
    pub tooth_w_pct: i32,
    /// Rayon du moyeu intérieur. 0 = plein, >0 = moyeu creux.
    pub hub_r: i32,
}

impl Gear {
    /// Construit un engrenage.
    ///
    /// # Paramètres
    ///
    /// - `cx`, `cy`     : centre
    /// - `pitch_r`      : rayon primitif (pixels)
    /// - `teeth`        : nombre de dents (≥ 3)
    /// - `tooth_h`      : hauteur de dent en pixels (addendum)
    /// - `tooth_w_pct`  : largeur dent en % du pas (30–80 recommandé)
    /// - `hub_r`        : rayon du moyeu (0 = disque plein, 2+ = anneau)
    #[inline]
    pub fn new(
        cx: i32, cy: i32,
        pitch_r: i32,
        teeth: i32,
        tooth_h: i32,
        tooth_w_pct: i32,
        hub_r: i32,
    ) -> Self {
        Self { cx, cy, pitch_r, teeth, tooth_h, tooth_w_pct: tooth_w_pct.max(1).min(99), hub_r }
    }

    /// Dessine l'engrenage à l'angle `angle` (radians).
    ///
    ///  `angle`    : rotation de l'engrenage (radians)
    ///  `on`       : `true` allume, `false` efface
    ///  `cos_fn`, `sin_fn` : fonctions trig
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        if self.teeth < 3 { return; }

        let outer_r = self.pitch_r + self.tooth_h;

        // Corps circulaire (cercle au rayon primitif)
        ring(gfx, self.cx, self.cy, self.pitch_r, on);

        //  Dents
        // Pas angulaire entre deux dents (en radians, flottant uniquement ici)
        let step = core::f32::consts::TAU / (self.teeth as f32);
        // Demi-largeur angulaire de la dent
        let half_tooth_angle = step * (self.tooth_w_pct as f32) / 200.0;

        for i in 0..self.teeth {
            let center_angle = angle + (i as f32) * step;

            // Angles des bords gauche/droit de la dent
            let a_left  = center_angle - half_tooth_angle;
            let a_right = center_angle + half_tooth_angle;

            // 4 coins de la dent en coordonnées pixel
            // Bas-gauche, Bas-droit  (sur le cercle primitif)
            // Haut-gauche, Haut-droit (sur le cercle extérieur)
            let (blx, bly) = polar_px(self.cx, self.cy, self.pitch_r, a_left,  cos_fn, sin_fn);
            let (brx, bry) = polar_px(self.cx, self.cy, self.pitch_r, a_right, cos_fn, sin_fn);
            let (tlx, tly) = polar_px(self.cx, self.cy, outer_r,      a_left,  cos_fn, sin_fn);
            let (trx, try_) = polar_px(self.cx, self.cy, outer_r,     a_right, cos_fn, sin_fn);

            // Côtés gauche et droit de la dent
            segment(gfx, blx, bly, tlx, tly, on);
            segment(gfx, brx, bry, trx, try_, on);
            // Sommet de la dent
            segment(gfx, tlx, tly, trx, try_, on);

            // Arc entre cette dent et la suivante (sur le cercle primitif)
            // Approché par des points discrets le long de l'arc
            let a_next_left = center_angle + step - half_tooth_angle;
            draw_arc(gfx, self.cx, self.cy, self.pitch_r,
                     a_right, a_next_left, on, cos_fn, sin_fn);
        }

        // Moyeu
        if self.hub_r > 0 {
            ring(gfx, self.cx, self.cy, self.hub_r, on);
            // Point central
            gfx.pixel(self.cx, self.cy, on);
        } else {
            // Moyeu plein : petit disque
            filled_disk(gfx, self.cx, self.cy, 2, on);
        }
    }

    /// Efface l'engrenage.
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

    /// Rayon extérieur (sommet des dents).
    #[inline]
    pub fn outer_r(&self) -> i32 {
        self.pitch_r + self.tooth_h
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GearPair
// ─────────────────────────────────────────────────────────────────────────────

/// Paire d'engrenages en contact.
///
/// Le second engrenage est positionné automatiquement en prise avec le premier.
/// Son centre est calculé à la distance `pitch_r1 + pitch_r2` du centre du premier,
/// dans la direction `axis_angle`.
///
/// La rotation est inversée et mise à l'échelle par le rapport de denture.
///
/// ## Rapport de transmission
///
/// `ω2 = -ω1 × (teeth1 / teeth2)` : calculé en virgule fixe (×1024).
///
/// ## Mise en phase
///
/// Le second engrenage est automatiquement déphasé de `π / teeth2`
/// pour que les dents se croisent proprement en x=0.
///
/// ## Exemple
///
/// ```rust,no_run
/// use embassy_ssd1306_physics::gear::{Gear, GearPair};
/// use embedded_trig_f32 as trig;
///
/// let pair = GearPair::new(
///     Gear::new(30, 32, 12, 8, 4, 70, 3),
///     Gear::new(0, 0, 18, 12, 4, 70, 5),
///     0.0,   // axe horizontal (second à droite du premier)
///     trig::cos, trig::sin,
/// );
/// pair.draw(&mut gfx, 0.3, true, trig::cos, trig::sin);
/// ```
#[derive(Clone, Copy, Debug)]
pub struct GearPair {
    /// Premier engrenage (moteur).
    pub g1: Gear,
    /// Second engrenage (récepteur), cx/cy calculés automatiquement.
    pub g2: Gear,
    /// Angle de l'axe reliant les deux centres (radians).
    pub axis_angle: f32,
}

impl GearPair {
    /// Crée une paire d'engrenages engrenés.
    ///
    /// `g1`         : engrenage moteur (cx/cy définissent le centre réel)
    /// `g2`         : engrenage récepteur (cx/cy **ignorés**, calculés ici)
    /// `axis_angle` : angle de l'axe de contact (0.0 = g2 à droite de g1)
    /// `cos_fn`, `sin_fn` : fonctions trig pour le calcul du centre
    pub fn new(
        g1: Gear,
        mut g2: Gear,
        axis_angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) -> Self {
        let dist = g1.pitch_r + g2.pitch_r;
        g2.cx = g1.cx + ((dist as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g2.cy = g1.cy + ((dist as f32) * sin_fn(axis_angle) + 0.5) as i32;
        Self { g1, g2, axis_angle }
    }

    /// Dessine la paire.
    ///
    /// `angle1` : angle de rotation du premier engrenage (radians)
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        // Rapport de transmission en flottant
        let ratio = (self.g1.teeth as f32) / (self.g2.teeth as f32);
        // Mise en phase : dent du g2 alignée au point de contact
        let phase2 = self.axis_angle + core::f32::consts::PI / (self.g2.teeth as f32);
        let angle2 = phase2 - angle1 * ratio;

        self.g1.draw(gfx, angle1, on, cos_fn, sin_fn);
        self.g2.draw(gfx, angle2, on, cos_fn, sin_fn);
    }

    /// Efface la paire.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle1, false, cos_fn, sin_fn);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GearTrain : chaîne de N engrenages
// ─────────────────────────────────────────────────────────────────────────────

/// Chaîne de 3 engrenages en ligne (moteur → intermédiaire → sortie).
///
/// Chaque engrenage est en contact avec le suivant. Les centres et
/// les angles de rotation sont calculés automatiquement.
///
/// ```text
///   G1 ──── G2 ──── G3
/// ```
///
/// ## Exemple
///
/// ```rust,no_run
/// use embassy_ssd1306_physics::gear::{Gear, GearTrain};
/// use embedded_trig_f32 as trig;
///
/// let train = GearTrain::new(
///     Gear::new(18, 32, 10, 8, 3, 70, 2),   // G1
///     Gear::new(0, 0,  14, 12, 3, 70, 3),   // G2
///     Gear::new(0, 0,   8,  6, 3, 70, 2),   // G3
///     0.0,   // alignement horizontal
///     trig::cos, trig::sin,
/// );
/// train.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
/// ```
#[derive(Clone, Copy, Debug)]
pub struct GearTrain {
    /// Engrenage moteur.
    pub g1: Gear,
    /// Engrenage intermédiaire.
    pub g2: Gear,
    /// Engrenage de sortie.
    pub g3: Gear,
    /// Angle de l'axe (0 = horizontal).
    pub axis_angle: f32,
}

impl GearTrain {
    /// Construit une chaîne de 3 engrenages.
    pub fn new(
        g1: Gear,
        mut g2: Gear,
        mut g3: Gear,
        axis_angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) -> Self {
        // Positionner g2 en prise avec g1
        let d12 = g1.pitch_r + g2.pitch_r;
        g2.cx = g1.cx + ((d12 as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g2.cy = g1.cy + ((d12 as f32) * sin_fn(axis_angle) + 0.5) as i32;

        // Positionner g3 en prise avec g2
        let d23 = g2.pitch_r + g3.pitch_r;
        g3.cx = g2.cx + ((d23 as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g3.cy = g2.cy + ((d23 as f32) * sin_fn(axis_angle) + 0.5) as i32;

        Self { g1, g2, g3, axis_angle }
    }

    /// Dessine le train d'engrenages.
    ///
    /// - `angle1` : angle de rotation du premier engrenage
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let r12 = (self.g1.teeth as f32) / (self.g2.teeth as f32);
        let phase2 = self.axis_angle + core::f32::consts::PI / (self.g2.teeth as f32);
        let angle2 = phase2 - angle1 * r12;

        let r23 = (self.g2.teeth as f32) / (self.g3.teeth as f32);
        let phase3 = self.axis_angle + core::f32::consts::PI / (self.g3.teeth as f32);
        let angle3 = phase3 - angle2 * r23;

        self.g1.draw(gfx, angle1, on, cos_fn, sin_fn);
        self.g2.draw(gfx, angle2, on, cos_fn, sin_fn);
        self.g3.draw(gfx, angle3, on, cos_fn, sin_fn);
    }

    /// Efface le train.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle1, false, cos_fn, sin_fn);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers internes
// ─────────────────────────────────────────────────────────────────────────────

/// Coordonnées pixel d'un point polaire.
#[inline]
fn polar_px(
    cx: i32, cy: i32, r: i32, angle: f32,
    cos_fn: fn(f32) -> f32,
    sin_fn: fn(f32) -> f32,
) -> (i32, i32) {
    let x = cx + ((r as f32) * cos_fn(angle) + 0.5) as i32;
    let y = cy + ((r as f32) * sin_fn(angle) + 0.5) as i32;
    (x, y)
}

/// Dessine un arc de cercle par interpolation angulaire.
///
/// Trace des pixels discrets du cercle de rayon `r` entre `a_start` et `a_end`.
/// Le nombre de pas est proportionnel à l'angle couvert × rayon
/// pour garantir des pixels contigus.
fn draw_arc<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    a_start: f32, a_end: f32,
    on: bool,
    cos_fn: fn(f32) -> f32,
    sin_fn: fn(f32) -> f32,
) {
    // Nombre de pas : au moins 1 par pixel de circonférence couverte
    let span = a_end - a_start;
    if span <= 0.0 { return; }

    // Circonférence totale / pas angulaire → nombre de pixels ~arc
    let steps = ((r as f32) * span + 1.5) as i32;
    if steps <= 0 { return; }

    for i in 0..=steps {
        let a = a_start + span * (i as f32) / (steps as f32);
        let x = cx + ((r as f32) * cos_fn(a) + 0.5) as i32;
        let y = cy + ((r as f32) * sin_fn(a) + 0.5) as i32;
        gfx.pixel(x, y, on);
    }
}

// Supprimer l'avertissement unused :TAU_FP est conservé pour extensions futures.
#[allow(dead_code)]
const _: i32 = TAU_FP;