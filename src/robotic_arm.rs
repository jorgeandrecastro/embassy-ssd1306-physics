// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # RoboticArm — bras industriel 2D
//!
//! Ce module implémente un bras robotique plan simplifié conçu pour affichage OLED
//! et simulation embarquée `no_std`.
//!
//! ## Architecture mécanique
//!
//! Le système est composé de :
//! - un **socle industriel** fixé au sol (rectangle hachuré)
//! - une **articulation d'épaule** (pivot fixe, juste au-dessus du socle)
//! - un **premier segment** (épaule → coude)
//! - un **second segment** (coude → effecteur)
//! - une **pince terminale** industrielle à mâchoires parallèles (type pneumatique)
//!
//! ## Convention de repère
//!
//! ```text
//! (0,0) ──── X+
//!   │
//!   Y+   (Y croît vers le bas, convention écran)
//! ```
//!
//! Le bras est **ancré en bas** (`base_y` = niveau sol) et s'étend **vers le haut**.
//! L'orientation gauche/droite est contrôlée par [`Facing`].
//!
//! ## Modèle visuel (facing = Right, angles nuls)
//!
//! ```text
//!      ════╗  ← mâchoire sup
//!        ●   ← effecteur
//!       /
//!      ●     ← coude
//!       \
//!        ●   ← épaule
//!        │
//!    ██████  ← socle industriel
//! ──────────  sol
//! ```
//!
//! ## Convention angulaire
//!
//! Les angles sont mesurés depuis la **verticale montante** (axe −Y écran) :
//!
//! ```text
//!        0°
//!        ↑
//! −90° ←   → +90°
//! ```
//!
//! - `angle_shoulder` : orientation absolue du segment 1 depuis la verticale.
//!   - `0.0`  → segment droit vers le haut
//!   - positif → incliné vers le côté [`Facing`]
//! - `angle_elbow` : rotation **relative** du segment 2 par rapport au segment 1.
//!   - `0.0`  → segments alignés (bras tendu)
//!   - positif → coude plié vers le côté [`Facing`]
//!
//! ## Dépendances internes
//!
//! Ce module utilise exclusivement les primitives de [`crate::draw_utils`] :
//! [`segment`], [`thick_segment`], [`filled_disk`], [`filled_rect`].
//! Il n'importe plus directement `embassy-ssd1306-graphics`.
//!
//! ## Propriétés physiques
//!
//! Modèle purement géométrique (pas d'inertie, pas de dynamique).
//! Conçu pour rendu temps réel embarqué sur afficheur SSD1306 128×64.

use crate::draw_utils::{filled_disk, filled_rect, segment, thick_segment};
use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

// ─────────────────────────────────────────────────────────────────────────────
// Facing
// ─────────────────────────────────────────────────────────────────────────────

/// Orientation de la pince — sens d'ouverture des mâchoires.
///
/// Contrôle le signe de la composante latérale dans le calcul cinématique.
/// Toutes les directions perpendiculaires (`perp_x`, `perp_y`) sont multipliées
/// par [`Facing::sign`], ce qui reflète la pince de façon cohérente.
///
/// # Exemple
///
/// ```rust
/// let sign = Facing::Right.sign(); // +1.0
/// let sign = Facing::Left.sign();  // -1.0
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Facing {
    /// Mâchoires s'ouvrent vers la droite (robot orienté droite).
    Right,
    /// Mâchoires s'ouvrent vers la gauche (robot orienté gauche).
    Left,
}

impl Facing {
    /// Retourne le facteur directionnel latéral.
    ///
    /// - `Right` → `+1.0` (X croît vers la droite)
    /// - `Left`  → `−1.0` (X décroît vers la gauche)
    #[inline]
    fn sign(self) -> f32 {
        match self {
            Facing::Right => 1.0,
            Facing::Left => -1.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RoboticArm
// ─────────────────────────────────────────────────────────────────────────────

/// Bras robotique industriel 2D à deux segments avec pince pneumatique.
///
/// Toutes les dimensions sont en **pixels écran** (entiers `i32`).
/// La position du socle est définie par (`base_x`, `base_y`) où `base_y`
/// correspond au **niveau du sol** (bas du socle).
///
/// # Construction
///
/// ```rust
/// let arm = RoboticArm::new(64, 63, 20, 18)
///     .with_wall(24, 8)
///     .with_gripper(12, 2);
/// ```
///
/// # Rendu
///
/// ```rust
/// arm.draw(
///     &mut gfx,
///     0.3,          // angle_shoulder (rad)
///     -0.5,         // angle_elbow (rad)
///     0.0,          // pince fermée
///     Facing::Right,
///     true,         // on = dessiner
///     libm::cosf,
///     libm::sinf,
/// );
/// ```
#[derive(Clone, Copy, Debug)]
pub struct RoboticArm {
    /// Centre horizontal du socle et de l'épaule (pixels).
    pub base_x: i32,

    /// Niveau du sol — bas du socle industriel (pixels, Y croît vers le bas).
    pub base_y: i32,

    /// Longueur du segment 1 : épaule → coude (pixels).
    pub seg1_len: i32,

    /// Longueur du segment 2 : coude → effecteur (pixels).
    pub seg2_len: i32,

    /// Largeur du socle industriel (pixels).
    pub wall_w: i32,

    /// Hauteur du socle industriel (pixels).
    pub wall_h: i32,

    /// Longueur des tiges de mâchoire, dans l'axe du segment 2 (pixels).
    pub gripper_len: i32,

    /// Demi-écartement minimal des mâchoires à pince fermée (pixels).
    ///
    /// Correspond à l'épaisseur mécanique d'une mâchoire.
    /// Utilisé aussi comme taille des embouts de serrage.
    pub gripper_thickness: i32,
}

impl RoboticArm {
    /// Crée un bras avec le socle et la pince aux valeurs par défaut.
    ///
    /// Valeurs par défaut :
    /// - socle : 20 × 6 px
    /// - pince : longueur 10 px, épaisseur 2 px
    ///
    /// # Paramètres
    ///
    /// | Paramètre  | Description                            |
    /// |------------|----------------------------------------|
    /// | `base_x`   | Centre horizontal du robot (px)        |
    /// | `base_y`   | Niveau du sol, bas du socle (px)       |
    /// | `seg1_len` | Longueur épaule → coude (px)           |
    /// | `seg2_len` | Longueur coude → effecteur (px)        |
    pub fn new(base_x: i32, base_y: i32, seg1_len: i32, seg2_len: i32) -> Self {
        Self {
            base_x,
            base_y,
            seg1_len,
            seg2_len,
            wall_w: 20,
            wall_h: 6,
            gripper_len: 10,
            gripper_thickness: 2,
        }
    }

    /// Personnalise la géométrie du socle industriel.
    ///
    /// Le socle est rendu comme un rectangle plein avec texture de hachures
    /// diagonales (pas de 4 px) simulant une surface métallique.
    ///
    /// # Paramètres
    ///
    /// - `wall_w` : largeur totale du socle (px)
    /// - `wall_h` : hauteur totale du socle (px) — s'étend vers le haut depuis `base_y`
    pub fn with_wall(mut self, wall_w: i32, wall_h: i32) -> Self {
        self.wall_w = wall_w;
        self.wall_h = wall_h;
        self
    }

    /// Personnalise la géométrie des mâchoires de la pince industrielle.
    ///
    /// La pince est de type **pneumatique à translation parallèle** :
    /// les deux mâchoires glissent perpendiculairement à l'axe du segment 2.
    ///
    /// # Paramètres
    ///
    /// - `gripper_len`       : longueur de chaque tige de mâchoire dans l'axe du segment 2 (px)
    /// - `gripper_thickness` : demi-écartement minimal (pince fermée) et taille des embouts (px)
    pub fn with_gripper(mut self, gripper_len: i32, gripper_thickness: i32) -> Self {
        self.gripper_len = gripper_len;
        self.gripper_thickness = gripper_thickness;
        self
    }

    // ─────────────────────────────────────────────────────────────────────────
    // draw
    // ─────────────────────────────────────────────────────────────────────────

    /// Dessine le bras robotique complet dans son état courant.
    ///
    /// # Algorithme de rendu (ordre d'appel)
    ///
    /// 1. **Socle** — rectangle plein + hachures diagonales + ligne de sol
    /// 2. **Cinématique directe** — calcul des positions épaule / coude / effecteur
    /// 3. **Segments épais** — épaule→coude et coude→effecteur via [`thick_segment`]
    /// 4. **Articulations** — disques pleins aux trois pivots via [`filled_disk`]
    /// 5. **Pince** — corps, rail de translation, tiges et embouts via [`segment`]
    ///
    /// # Cinématique directe
    ///
    /// Le repère angulaire est la **verticale montante** (−Y écran).
    /// Pour le segment 1 (angle absolu `a1 = angle_shoulder`) :
    ///
    /// ```text
    /// dir1_x =  sin(a1) × sign
    /// dir1_y = −cos(a1)
    /// ```
    ///
    /// Pour le segment 2 (angle absolu `a2 = angle_shoulder + angle_elbow`) :
    ///
    /// ```text
    /// dir2_x =  sin(a2) × sign
    /// dir2_y = −cos(a2)
    /// ```
    ///
    /// # Géométrie de la pince
    ///
    /// ```text
    ///   end ──── guide      (corps de longueur body_len = 3 px)
    ///             │
    ///         ╠═════╣       (rail de translation)
    ///         ║     ║
    ///        jaw1  jaw2     (tiges parallèles à dir2)
    ///        [▓]   [▓]      (embouts perpendiculaires)
    /// ```
    ///
    /// L'écartement des mâchoires est interpolé linéairement :
    ///
    /// ```text
    /// half_gap = half_gap_closed + (half_gap_max − half_gap_closed) × open
    /// ```
    ///
    /// # Paramètres
    ///
    /// | Paramètre          | Type      | Description                                        |
    /// |--------------------|-----------|----------------------------------------------------|
    /// | `gfx`              | `&mut Graphics` | Contexte de rendu SSD1306                    |
    /// | `angle_shoulder`   | `f32`     | Angle épaule depuis la verticale (rad)             |
    /// | `angle_elbow`      | `f32`     | Rotation relative du coude (rad)                   |
    /// | `gripper_opening`  | `f32`     | Ouverture [0.0 = fermé … 1.0 = ouvert max]         |
    /// | `facing`           | [`Facing`]| Orientation droite/gauche de la pince              |
    /// | `on`               | `bool`    | `true` = allumer pixels, `false` = éteindre (erase)|
    /// | `cos_fn`           | `fn(f32)->f32` | Cosinus no_std (ex. `libm::cosf`)             |
    /// | `sin_fn`           | `fn(f32)->f32` | Sinus no_std (ex. `libm::sinf`)               |
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        gripper_opening: f32,
        facing: Facing,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        // ── 1. SOCLE ─────────────────────────────────────────────────────────
        //
        // Le socle s'étend de (sx, sy) jusqu'à (sx + wall_w, base_y).
        // sy = base_y - wall_h  (sommet du socle = point d'articulation épaule)

        let half = self.wall_w / 2;
        let sx = self.base_x - half;
        let sy = self.base_y - self.wall_h; // sommet du socle

        // Bloc plein (remplace la double boucle pixel manuelle)
        filled_rect(gfx, sx, sy, self.wall_w, self.wall_h, on);

        // Texture hachures diagonales (pas de 4 px), inversées pour simuler le métal
        let mut d = 0;
        while d < self.wall_w + self.wall_h {
            for dx in 0..self.wall_w {
                let dy = d - dx;
                if dy >= 0 && dy < self.wall_h {
                    gfx.pixel(sx + dx, sy + dy, !on);
                }
            }
            d += 4;
        }

        // Ligne de sol légèrement plus large que le socle (±4 px)
        segment(
            gfx,
            sx - 4,
            self.base_y,
            sx + self.wall_w + 4,
            self.base_y,
            on,
        );

        // ── 2. CINÉMATIQUE DIRECTE ────────────────────────────────────────────
        //
        // Origine de la chaîne : sommet du socle (= épaule).
        let shoulder_x = self.base_x;
        let shoulder_y = sy;

        // Direction segment 1 dans le repère écran (verticale montante = 0°)
        let dir1_x = sin_fn(angle_shoulder) * facing.sign();
        let dir1_y = -cos_fn(angle_shoulder);

        let elbow_x = shoulder_x + (dir1_x * self.seg1_len as f32) as i32;
        let elbow_y = shoulder_y + (dir1_y * self.seg1_len as f32) as i32;

        // Direction segment 2 (angle absolu = somme des deux rotations)
        let abs_angle = angle_shoulder + angle_elbow;
        let dir2_x = sin_fn(abs_angle) * facing.sign();
        let dir2_y = -cos_fn(abs_angle);

        let end_x = elbow_x + (dir2_x * self.seg2_len as f32) as i32;
        let end_y = elbow_y + (dir2_y * self.seg2_len as f32) as i32;

        // ── 3. SEGMENTS ÉPAIS ────────────────────────────────────────────────
        //
        // thick_segment trace la ligne principale + une ligne décalée d'un pixel
        // perpendiculairement à la direction dominante (voir draw_utils).
        thick_segment(gfx, shoulder_x, shoulder_y, elbow_x, elbow_y, on);
        thick_segment(gfx, elbow_x, elbow_y, end_x, end_y, on);

        // ── 4. ARTICULATIONS ─────────────────────────────────────────────────
        //
        // Disques pleins aux trois pivots (épaule r=3, coude r=3, effecteur r=2).
        filled_disk(gfx, shoulder_x, shoulder_y, 3, on);
        filled_disk(gfx, elbow_x, elbow_y, 3, on);
        filled_disk(gfx, end_x, end_y, 2, on);

        // ── 5. PINCE PNEUMATIQUE ─────────────────────────────────────────────
        //
        // Perpendiculaire à dir2 : vecteur de translation des mâchoires.
        let perp_x = -dir2_y;
        let perp_y = dir2_x;

        // Corps de pince : prolongement depuis l'effecteur pour placer le rail.
        let body_len = 3_i32;
        let guide_x = end_x + (dir2_x * body_len as f32) as i32;
        let guide_y = end_y + (dir2_y * body_len as f32) as i32;
        segment(gfx, end_x, end_y, guide_x, guide_y, on);

        // Interpolation de l'écartement courant des mâchoires.
        let open = gripper_opening.clamp(0.0, 1.0);
        let half_gap_max = (self.gripper_len as f32 * 0.6) as i32; // écartement max
        let half_gap_closed = self.gripper_thickness; // écartement min (fermé)
        let half_gap =
            half_gap_closed + ((half_gap_max - half_gap_closed) as f32 * open) as i32;

        // Origines des mâchoires sur le rail (symétrie par rapport à l'axe dir2).
        let jaw1_ox = guide_x + (perp_x * half_gap as f32) as i32;
        let jaw1_oy = guide_y + (perp_y * half_gap as f32) as i32;

        let jaw2_ox = guide_x - (perp_x * half_gap as f32) as i32;
        let jaw2_oy = guide_y - (perp_y * half_gap as f32) as i32;

        // Extrémités des tiges (avancent dans la direction dir2).
        let jaw1_ex = jaw1_ox + (dir2_x * self.gripper_len as f32) as i32;
        let jaw1_ey = jaw1_oy + (dir2_y * self.gripper_len as f32) as i32;

        let jaw2_ex = jaw2_ox + (dir2_x * self.gripper_len as f32) as i32;
        let jaw2_ey = jaw2_oy + (dir2_y * self.gripper_len as f32) as i32;

        // Rail de guidage (ligne transversale reliant les deux origines).
        segment(gfx, jaw1_ox, jaw1_oy, jaw2_ox, jaw2_oy, on);

        // Tiges des mâchoires (parallèles à dir2).
        segment(gfx, jaw1_ox, jaw1_oy, jaw1_ex, jaw1_ey, on);
        segment(gfx, jaw2_ox, jaw2_oy, jaw2_ex, jaw2_ey, on);

        // Embouts de serrage (perpendiculaires aux tiges, ±tip px de chaque côté).
        let tip = self.gripper_thickness.max(2);
        let tip_off_x = (perp_x * tip as f32) as i32;
        let tip_off_y = (perp_y * tip as f32) as i32;

        segment(
            gfx,
            jaw1_ex + tip_off_x,
            jaw1_ey + tip_off_y,
            jaw1_ex - tip_off_x,
            jaw1_ey - tip_off_y,
            on,
        );
        segment(
            gfx,
            jaw2_ex + tip_off_x,
            jaw2_ey + tip_off_y,
            jaw2_ex - tip_off_x,
            jaw2_ey - tip_off_y,
            on,
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // erase
    // ─────────────────────────────────────────────────────────────────────────

    /// Efface le bras robotique dans l'état donné.
    ///
    /// Appelle [`draw`](Self::draw) avec `on = false`.
    /// Les paramètres doivent être **identiques** à ceux du dernier appel à `draw`
    /// pour garantir un effacement pixel-perfect sans artefact.
    ///
    /// # Usage typique (animation)
    ///
    /// ```rust
    /// // Effacer l'ancienne position
    /// arm.erase(&mut gfx, old_shoulder, old_elbow, 0.0, Facing::Right, cosf, sinf);
    /// // Dessiner la nouvelle
    /// arm.draw(&mut gfx, new_shoulder, new_elbow, 0.0, Facing::Right, true, cosf, sinf);
    /// gfx.flush().await?;
    /// ```
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        gripper_opening: f32,
        facing: Facing,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(
            gfx,
            angle_shoulder,
            angle_elbow,
            gripper_opening,
            facing,
            false, // on = false → efface
            cos_fn,
            sin_fn,
        );
    }
}