// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # RoboticArm bras industriel 2D
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
//! L'orientation est contrôlée par [`Facing`], appliqué comme **transformation
//! globale** (miroir 2D homogène) et non comme patch sur X.
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
//! ## Convention angulaire (espace local, avant miroir)
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
//!   - positif → incliné vers la droite en espace local (avant miroir)
//! - `angle_elbow` : rotation **relative** du segment 2 par rapport au segment 1.
//!   - `0.0`  → segments alignés (bras tendu)
//!   - positif → coude plié vers la droite en espace local
//!
//! ## Modèle mathématique — Facing comme transformation globale
//!
//! Toutes les directions sont d'abord calculées en **espace local** (Facing = Right),
//! puis transformées par [`Transform2D`] avant d'être converties en pixels.
//!
//! ```text
//! espace local  →  Transform2D::apply()  →  espace écran
//!
//! Right : identité        (sx=+1)
//! Left  : miroir sur X    (sx=−1)
//! ```
//!
//! Le vecteur perpendiculaire (`perp`) est dérivé **après** transformation,
//! ce qui garantit la cohérence géométrique de la pince dans les deux orientations.
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
// Transform2D :transformation affine 2D minimale (rotation + miroir)
// ─────────────────────────────────────────────────────────────────────────────

/// Transformation affine 2D minimaliste pour l'espace écran.
///
/// Représente une matrice 2×2 appliquée à des vecteurs directeurs `(vx, vy)`.
/// Pas de translation (elle est gérée explicitement via les origines de segment).
///
/// # Matrice
///
/// ```text
/// | sx  shx | × | vx |
/// | shy  sy | × | vy |
/// ```
///
/// Pour [`Facing::Right`] : identité `(sx=1, sy=1, shx=0, shy=0)`.
/// Pour [`Facing::Left`]  : miroir X  `(sx=−1, sy=1, shx=0, shy=0)`.
///
/// # Pourquoi pas juste `sign` ?
///
/// Un simple `sign` sur X est insuffisant dès que la perpendiculaire
/// est calculée depuis le vecteur transformé : le produit vectoriel
/// `perp = (−dir_y, dir_x)` reste cohérent si et seulement si
/// `dir` est déjà dans l'espace global.
/// [`Transform2D`] garantit que **toutes** les directions passent
/// par la même transformation avant usage.
#[derive(Clone, Copy, Debug)]
 pub struct Transform2D {
    /// Facteur d'échelle sur X (±1.0 pour miroir, 1.0 pour identité).
     sx: f32,
    /// Facteur d'échelle sur Y (toujours 1.0 dans ce modèle).
     sy: f32,
    /// Cisaillement X→Y (0.0 dans ce modèle).
     shx: f32,
    /// Cisaillement Y→X (0.0 dans ce modèle).
     shy: f32,
}

impl Transform2D {
    /// Transformation identité (Facing::Right).
    #[inline]
    pub const fn identity() -> Self {
        Self { sx: 1.0, sy: 1.0, shx: 0.0, shy: 0.0 }
    }

    /// Miroir sur l'axe X (Facing::Left).
    ///
    /// Inverse la composante X de tout vecteur transformé,
    /// ce qui reflète le bras entier (segments + pince) de façon homogène.
    #[inline]
    pub const fn mirror_x() -> Self {
        Self { sx: -1.0, sy: 1.0, shx: 0.0, shy: 0.0 }
    }

    /// Applique la transformation à un vecteur directeur `(vx, vy)`.
    ///
    /// Retourne `(vx', vy')` dans l'espace écran.
    ///
    /// ```text
    /// vx' = sx × vx + shx × vy
    /// vy' = shy × vx + sy  × vy
    /// ```
    #[inline]
    fn apply(self, vx: f32, vy: f32) -> (f32, f32) {
        (
            self.sx * vx + self.shx * vy,
            self.shy * vx + self.sy * vy,
        )
    }

    /// Calcule l'extrémité d'un segment depuis une origine, dans cette transformation.
    ///
    /// `(ox, oy)` : origine en pixels.
    /// `(lx, ly)` : direction locale (non transformée).
    /// `len`      : longueur du segment en pixels.
    ///
    /// Retourne le point d'arrivée `(x, y)` en pixels.
    #[inline]
    fn endpoint(self, ox: i32, oy: i32, lx: f32, ly: f32, len: i32) -> (i32, i32) {
        let (dx, dy) = self.apply(lx, ly);
        (
            ox + (dx * len as f32 + 0.5) as i32,
            oy + (dy * len as f32 + 0.5) as i32,
        )
    }

    /// Calcule le vecteur **perpendiculaire** à un vecteur global `(gx, gy)`.
    ///
    /// Le vecteur `(gx, gy)` doit déjà être dans l'espace écran (post-transformation).
    /// La perpendiculaire est `(−gy, gx)`, cohérente quel que soit le [`Facing`].
    ///
    /// C'est ici que réside le correctif principal par rapport à l'ancienne version :
    /// `perp` est toujours calculé **après** [`Transform2D::apply`], jamais avant.
    #[inline]
    fn perp(gx: f32, gy: f32) -> (f32, f32) {
        (-gy, gx)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Facing
// ─────────────────────────────────────────────────────────────────────────────

/// Orientation du bras — détermine la transformation globale appliquée au rendu.
///
/// [`Facing`] n'est plus un simple `sign()` sur X : il produit une [`Transform2D`]
/// complète qui garantit la cohérence géométrique de tous les vecteurs
/// (segments, perpendiculaires, pince).
///
/// # Exemple
///
/// ```rust
/// let t = Facing::Left.transform();
/// let (gx, gy) = t.apply(0.5, -0.866); // direction locale → espace écran
/// let (px, py) = Transform2D::perp(gx, gy); // perpendiculaire correcte
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Facing {
    /// Bras orienté vers la droite — transformation identité.
    Right,
    /// Bras orienté vers la gauche — miroir sur l'axe X.
    Left,
}

impl Facing {
    /// Retourne la transformation 2D associée à cette orientation.
    ///
    /// - `Right` → [`Transform2D::identity`]
    /// - `Left`  → [`Transform2D::mirror_x`]
    #[inline]
    pub fn transform(self) -> Transform2D {
        match self {
            Facing::Right => Transform2D::identity(),
            Facing::Left  => Transform2D::mirror_x(),
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
/// Le rendu est mathématiquement cohérent dans les deux orientations :
/// [`Facing`] agit comme transformation globale via [`Transform2D`],
/// et non comme un patch ponctuel sur la composante X.
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
///     0.3,           // angle_shoulder (rad, espace local)
///     -0.5,          // angle_elbow (rad, relatif)
///     0.0,           // pince fermée
///     Facing::Right,
///     true,          // on = dessiner
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
    /// les deux mâchoires glissent perpendiculairement à l'axe du segment 2,
    /// après transformation globale par [`Transform2D`].
    ///
    /// # Paramètres
    ///
    /// - `gripper_len`       : longueur de chaque tige dans l'axe du segment 2 (px)
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
    /// 2. **Cinématique directe** — directions locales transformées par [`Transform2D`]
    /// 3. **Segments épais** — épaule→coude et coude→effecteur via [`thick_segment`]
    /// 4. **Articulations** — disques pleins aux trois pivots via [`filled_disk`]
    /// 5. **Pince** — corps, rail, tiges et embouts via [`segment`]
    ///
    /// # Cinématique directe
    ///
    /// Les directions sont d'abord exprimées en **espace local** (Facing = Right),
    /// puis transformées en espace écran via `t = facing.transform()`.
    ///
    /// ```text
    /// local :   lx =  sin(a),  ly = −cos(a)
    /// écran :  (gx, gy) = t.apply(lx, ly)
    /// ```
    ///
    /// La perpendiculaire est calculée **après** transformation :
    ///
    /// ```text
    /// (perp_x, perp_y) = Transform2D::perp(gx, gy) = (−gy, gx)
    /// ```
    ///
    /// Ce point est le correctif central par rapport à l'ancienne version :
    /// l'ancienne perpendiculaire `(−dir2_y, dir2_x)` était incohérente
    /// sur `Facing::Left` car `dir2` n'était pas encore dans l'espace global.
    ///
    /// # Géométrie de la pince
    ///
    /// ```text
    ///   end ──── guide      (corps body_len = 3 px dans dir2_g)
    ///             │
    ///         ╠═════╣       (rail de translation, axe perp)
    ///         ║     ║
    ///        jaw1  jaw2     (tiges parallèles à dir2_g)
    ///        [▓]   [▓]      (embouts perpendiculaires à dir2_g)
    /// ```
    ///
    /// Interpolation linéaire de l'écartement :
    ///
    /// ```text
    /// half_gap = half_gap_closed + (half_gap_max − half_gap_closed) × open
    /// ```
    ///
    /// # Paramètres
    ///
    /// | Paramètre          | Type                | Description                              |
    /// |--------------------|---------------------|------------------------------------------|
    /// | `gfx`              | `&mut Graphics`     | Contexte de rendu SSD1306                |
    /// | `angle_shoulder`   | `f32`               | Angle épaule depuis la verticale (rad)   |
    /// | `angle_elbow`      | `f32`               | Rotation relative du coude (rad)         |
    /// | `gripper_opening`  | `f32`               | Ouverture \[0.0 fermé … 1.0 ouvert max\] |
    /// | `facing`           | [`Facing`]          | Orientation droite/gauche                |
    /// | `on`               | `bool`              | `true` = dessiner, `false` = effacer     |
    /// | `cos_fn`           | `fn(f32) -> f32`    | Cosinus no_std (ex. `libm::cosf`)        |
    /// | `sin_fn`           | `fn(f32) -> f32`    | Sinus no_std (ex. `libm::sinf`)          |
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
        // Transformation globale : identité (Right) ou miroir X (Left).
        // Toutes les directions passent par `t` avant d'être utilisées.
        let t = facing.transform();

        // ── 1. SOCLE ─────────────────────────────────────────────────────────
        //
        // Le socle n'est pas affecté par le miroir (il est symétrique par
        // construction et centré sur base_x).
        // sy = base_y − wall_h  →  sommet du socle = point d'articulation épaule.

        let half = self.wall_w / 2;
        let sx   = self.base_x - half;
        let sy   = self.base_y - self.wall_h;

        // Bloc plein via filled_rect (remplace la double boucle pixel).
        filled_rect(gfx, sx, sy, self.wall_w, self.wall_h, on);

        // Texture hachures diagonales (pas = 4 px), inversées pour effet métal.
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

        // Ligne de sol légèrement plus large que le socle (±4 px).
        segment(gfx, sx - 4, self.base_y, sx + self.wall_w + 4, self.base_y, on);

        // ── 2. CINÉMATIQUE DIRECTE ────────────────────────────────────────────
        //
        // Étape 1 : direction locale du segment 1 (espace Right, verticale = 0°).
        //   lx =  sin(a),  ly = −cos(a)
        // Étape 2 : transformation vers espace écran via t.apply().

        let shoulder_x = self.base_x;
        let shoulder_y = sy;

        // Segment 1 — direction locale puis globale.
        let (l1x, l1y) = (sin_fn(angle_shoulder), -cos_fn(angle_shoulder));
        let (g1x, g1y) = t.apply(l1x, l1y);
        let (elbow_x, elbow_y) = t.endpoint(shoulder_x, shoulder_y, l1x, l1y, self.seg1_len);

        // Segment 2 — angle absolu = somme des deux rotations (espace local).
        let abs_angle = angle_shoulder + angle_elbow;
        let (l2x, l2y) = (sin_fn(abs_angle), -cos_fn(abs_angle));
        let (g2x, g2y) = t.apply(l2x, l2y);
        let (end_x, end_y) = t.endpoint(elbow_x, elbow_y, l2x, l2y, self.seg2_len);

        // Perpendiculaire à dir2 dans l'espace global — calculée APRÈS transformation.
        // C'est ici le correctif mathématique central : perp doit être dérivé
        // depuis le vecteur déjà transformé (g2x, g2y), pas depuis le vecteur local.
        let (perp_x, perp_y) = Transform2D::perp(g2x, g2y);

        // g1x/g1y utilisé uniquement pour vérification future (IK, debug).
        // On le marque explicitement pour éviter l'avertissement unused.
        let _ = (g1x, g1y);

        // ── 3. SEGMENTS ÉPAIS ────────────────────────────────────────────────
        //
        // thick_segment = ligne principale + décalage d'1 px perpendiculaire
        // à la direction dominante (voir draw_utils::thick_segment).
        thick_segment(gfx, shoulder_x, shoulder_y, elbow_x, elbow_y, on);
        thick_segment(gfx, elbow_x, elbow_y, end_x, end_y, on);

        // ── 4. ARTICULATIONS ─────────────────────────────────────────────────
        //
        // Disques pleins aux trois pivots.
        // Rayons : épaule r=3, coude r=3, effecteur r=2.
        filled_disk(gfx, shoulder_x, shoulder_y, 3, on);
        filled_disk(gfx, elbow_x,    elbow_y,    3, on);
        filled_disk(gfx, end_x,      end_y,      2, on);

        // ── 5. PINCE PNEUMATIQUE ─────────────────────────────────────────────
        //
        // Corps de pince : prolongement de 3 px depuis l'effecteur dans dir2_global.
        let body_len  = 3_i32;
        let guide_x   = end_x + (g2x * body_len as f32 + 0.5) as i32;
        let guide_y   = end_y + (g2y * body_len as f32 + 0.5) as i32;
        segment(gfx, end_x, end_y, guide_x, guide_y, on);

        // Interpolation de l'écartement courant.
        let open           = gripper_opening.clamp(0.0, 1.0);
        let half_gap_max    = (self.gripper_len as f32 * 0.6) as i32;
        let half_gap_closed = self.gripper_thickness;
        let half_gap        = half_gap_closed
            + ((half_gap_max - half_gap_closed) as f32 * open) as i32;

        // Origines des mâchoires — symétrie par rapport à l'axe dir2_global.
        let jaw1_ox = guide_x + (perp_x * half_gap as f32 + 0.5) as i32;
        let jaw1_oy = guide_y + (perp_y * half_gap as f32 + 0.5) as i32;
        let jaw2_ox = guide_x - (perp_x * half_gap as f32 + 0.5) as i32;
        let jaw2_oy = guide_y - (perp_y * half_gap as f32 + 0.5) as i32;

        // Extrémités des tiges (avancent dans dir2_global).
        let jaw1_ex = jaw1_ox + (g2x * self.gripper_len as f32 + 0.5) as i32;
        let jaw1_ey = jaw1_oy + (g2y * self.gripper_len as f32 + 0.5) as i32;
        let jaw2_ex = jaw2_ox + (g2x * self.gripper_len as f32 + 0.5) as i32;
        let jaw2_ey = jaw2_oy + (g2y * self.gripper_len as f32 + 0.5) as i32;

        // Rail de translation (relie les deux origines).
        segment(gfx, jaw1_ox, jaw1_oy, jaw2_ox, jaw2_oy, on);

        // Tiges des mâchoires (parallèles à dir2_global).
        segment(gfx, jaw1_ox, jaw1_oy, jaw1_ex, jaw1_ey, on);
        segment(gfx, jaw2_ox, jaw2_oy, jaw2_ex, jaw2_ey, on);

        // Embouts de serrage (perpendiculaires aux tiges, ±tip px).
        let tip       = self.gripper_thickness.max(2);
        let tip_off_x = (perp_x * tip as f32 + 0.5) as i32;
        let tip_off_y = (perp_y * tip as f32 + 0.5) as i32;

        segment(gfx,
            jaw1_ex + tip_off_x, jaw1_ey + tip_off_y,
            jaw1_ex - tip_off_x, jaw1_ey - tip_off_y,
            on,
        );
        segment(gfx,
            jaw2_ex + tip_off_x, jaw2_ey + tip_off_y,
            jaw2_ex - tip_off_x, jaw2_ey - tip_off_y,
            on,
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // erase
    // ─────────────────────────────────────────────────────────────────────────

    /// Efface le bras robotique dans l'état donné.
    ///
    /// Délègue à [`draw`](Self::draw) avec `on = false`.
    /// Les paramètres doivent être **identiques** à ceux du dernier `draw`
    /// pour garantir un effacement pixel-perfect sans artefact.
    ///
    /// # Usage typique (boucle d'animation)
    ///
    /// ```rust
    /// // 1. Effacer l'ancienne position
    /// arm.erase(&mut gfx, old_shoulder, old_elbow, 0.0, Facing::Right, cosf, sinf);
    /// // 2. Dessiner la nouvelle
    /// arm.draw(&mut gfx, new_shoulder, new_elbow, 0.0, Facing::Right, true, cosf, sinf);
    /// // 3. Envoyer le framebuffer
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