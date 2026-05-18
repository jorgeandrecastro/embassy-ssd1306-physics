# embassy-ssd1306-physics

[![docs](https://docs.rs/embassy-ssd1306-physics/badge.svg)](https://docs.rs/embassy-ssd1306-physics/) [![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html) [![physics-objects](https://img.shields.io/badge/physics--objects-embassy--ssd1306--physics-blue.svg)](https://crates.io/crates/embassy_ssd1306_physics)

Dessins 2D `no_std` d'objets physiques pour écrans OLED SSD1306 (128×64),  
construite au-dessus de [`embassy-ssd1306-graphics`](https://crates.io/crates/embassy-ssd1306-graphics).

---

## Objets disponibles

| Module          | Struct(s)               | Description                                      |
|-----------------|-------------------------|--------------------------------------------------|
| `robotic_arm`   | `RoboticArm`, `Facing`  | Bras industriel 2D (socle + pince pneumatique)   |
| `pendulum`      | `Pendulum`              | Pendule simple (encastrement + tige + masse)     |
| `piston`        | `Piston`                | Piston 2D mécanique (chambre + tige guidée)      |
| `spring_mass`   | `SpringMass`            | Système ressort-masse vertical (zigzag + bloc)   |
| `compass`       | `Compass`               | Boussole (cadran + cardinaux + aiguille)         |
| `gear`          | `Gear`, `GearPair`, `GearTrain` | Engrenages simples, paires et trains     |
| `draw_utils`    | *(interne)*             | Primitives partagées (segment, arc, disque…)     |


---

## Ajouter au projet

```toml
[dependencies]
embassy-ssd1306          = "0.6"
embassy-ssd1306-graphics  = "0.1"
embassy-ssd1306-physics   = "0.2"
embedded-trig-f32         = "0.1"   
embedded-hal-async        = "1.0"
```

---

## Patron sin/cos injecté

Chaque `draw()` accepte `cos_fn: fn(f32) -> f32` et `sin_fn: fn(f32) -> f32`.  
Passez vos fonctions trig `no_std` favorites — zéro couplage, zéro overhead :

```rust
use embedded_trig_f32 as trig;
arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
```

---

## Convention de repère et angles

### Repère écran

```text
(0,0) ──── X+
  │
  Y+   (Y croît vers le bas — convention écran standard)
```

### Conventions angulaires par objet

#### `RoboticArm` — Angles depuis la verticale montante

L'angle est mesuré **depuis la verticale montante** (axe −Y) :

```text
       0°
       ↑
−90° ←   → +90°
```

- `angle_shoulder` : orientation absolue du segment 1
  - `0.0` → segment droit vers le haut
  - positif → incliné vers le côté `Facing`
- `angle_elbow` : rotation relative du segment 2 par rapport au segment 1
  - `0.0` → segments alignés (bras tendu)
  - positif → coude plié vers le côté `Facing`

#### `Pendulum` — Angles depuis la verticale descendante

L'angle est mesuré **depuis la verticale descendante** (axe +Y) :

```text
  −90° ← 0° → +90°
         ↓
```

- `0.0` → pendule au repos (tige pend vers le bas)
- positif → penche à droite
- négatif → penche à gauche

#### `Piston` — Position linéaire

Le piston se déplace **verticalement** (axe Y) dans sa chambre :

```text
pos = 0     ← sommet (chambre fermée)
  │
  │ chambre
  │
pos = max   ← bas (piston complètement sortant)
```

- `pos ∈ [0, h − piston_h]` : position contrainte
- `normalized() ∈ [0.0, 1.0]` : position normalisée (0 = haut, 1 = bas)

---

## Structure des modules

```
src/
├── lib.rs           ← déclarations + re-exports
├── draw_utils.rs    ← segment, arc, disque, rect (partagés)
├── robotic_arm.rs   ← RoboticArm
├── pendulum.rs      ← Pendulum
├── piston.rs        ← Piston
├── spring_mass.rs   ← SpringMass
├── compass.rs       ← Compass
└── gear.rs          ← Gear · GearPair · GearTrain
```

### Ajouter un nouvel objet

1. Créer `src/mon_objet.rs`
2. Déclarer `pub mod mon_objet;` dans `src/lib.rs`
3. (Optionnel) `pub use mon_objet::MonObjet;` pour re-export

Aucun autre fichier à toucher.

---

## Exemples

### Bras robotique

Bras industriel 2D à deux segments avec pince pneumatique.  
**L'angle est mesuré depuis la verticale montante** (0 = droit vers le haut, positif = incliné selon `Facing`).

**Architecture :** socle industriel (rect + hachures) + pivot d'épaule + segment 1 + segment 2 + effecteur + pince à mâchoires.

**Facing** : énumération `Right` / `Left` qui contrôle l'orientation et le sens d'ouverture de la pince.

#### Constructeur par défaut

```rust
use embassy_ssd1306_physics::RoboticArm;
use embedded_trig_f32 as trig;

// Base à (64, 63), segment 1: 20px, segment 2: 18px
let arm = RoboticArm::new(64, 63, 20, 18);

// Épaule à 45°, coude à -30° relatif, pince fermée (0.0)
arm.draw(&mut gfx, 0.785, -0.524, 0.0, Facing::Right, true, trig::cos, trig::sin);

// Même bras à gauche
arm.draw(&mut gfx, 0.785, -0.524, 0.0, Facing::Left, true, trig::cos, trig::sin);
```

#### Socle et pince personnalisés

```rust
use embassy_ssd1306_physics::{RoboticArm, Facing};
use embedded_trig_f32 as trig;

let arm = RoboticArm::new(32, 60, 18, 16)
    .with_wall(20, 6)           // socle plus grand : 20×6
    .with_gripper(10, 2);       // pince : 10px long, 2px hauteur

arm.draw(&mut gfx, 0.3, 0.4, 0.0, Facing::Right, true, trig::cos, trig::sin);
```

#### Animation simple

```rust
use embassy_ssd1306_physics::{RoboticArm, Facing};
use embedded_trig_f32 as trig;

let arm = RoboticArm::new(64, 63, 20, 18);
let mut angle_shoulder: f32 = 0.0;
let mut angle_elbow: f32 = 0.0;

loop {
    arm.erase(&mut gfx, angle_shoulder, angle_elbow, 0.0, Facing::Right, trig::cos, trig::sin);
    
    // Balancer l'épaule et le coude
    angle_shoulder += 0.05;
    angle_elbow -= 0.03;
    
    arm.draw(&mut gfx, angle_shoulder, angle_elbow, 0.0, Facing::Right, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}

### Pendule

Pendule simple : pivot fixe + tige + masse circulaire.  
**L'angle est mesuré depuis la verticale descendante** (0 = repos, positif = penche à droite).

**Rendus :** encastrement mural (rect + hachures) + tige + pivot (r=1) + masse (r=3).

#### Constructeur par défaut

```rust
use embassy_ssd1306_physics::Pendulum;
use embedded_trig_f32 as trig;

// Pivot à (64, 6), tige 28px, encastrement par défaut 16×5
let pend = Pendulum::new(64, 6, 28);
pend.draw(&mut gfx, 0.4, true, trig::cos, trig::sin);   // ~23° à droite
pend.draw(&mut gfx, -0.4, true, trig::cos, trig::sin);  // ~23° à gauche
```

#### Encastrement personnalisé

```rust
use embassy_ssd1306_physics::Pendulum;
use embedded_trig_f32 as trig;

// Encastrement plus grand : 20×8 au lieu de 16×5
let pend = Pendulum::with_wall(32, 10, 30, 20, 8);
pend.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
```

#### Animation simple

```rust
let mut angle: f32 = 0.0;
loop {
    pend.erase(&mut gfx, angle, trig::cos, trig::sin);
    angle += 0.05;  // incrémenter l'angle progressivement
    pend.draw(&mut gfx, angle, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}
```

### Ressort-masse

```rust
use embassy_ssd1306_physics::SpringMass;

let sm = SpringMass::new(64, 4, 22, 12);
// Ressort étiré de 6px
sm.draw(&mut gfx, 6, true);
// Ressort comprimé de 4px
sm.draw(&mut gfx, -4, true);

// Personnalisation : 8 demi-spires, amplitude ±6px
let sm2 = SpringMass::new(64, 4, 22, 12).with_coils(8, 6);
sm2.draw(&mut gfx, 0, true);
```

### Boussole

```rust
use embassy_ssd1306_physics::Compass;
use embedded_trig_f32 as trig;

let compass = Compass::new(112, 32, 14);
compass.draw(&mut gfx, 0.785, true, trig::cos, trig::sin); // NE
```

### Piston

```rust
use embassy_ssd1306_physics::Piston;

let mut piston = Piston::new(64, 10, 20, 40);

// Position au sommet (fermé)
piston.set_pos(0);
piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);

// Position au milieu
piston.set_pos(20);
piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);

// Animation simple : va-et-vient
let max_pos = piston.h - piston.piston_h;
let mut pos: i32 = 0;
loop {
    piston.erase(&mut gfx, 0.0, |_| 0.0, |_| 0.0);
    pos = (pos + 1) % (max_pos * 2);
    let actual_pos = if pos > max_pos { max_pos * 2 - pos } else { pos };
    piston.set_pos(actual_pos);
    piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);
    oled.flush().await.unwrap();
}
```

### Engrenage simple

```rust
use embassy_ssd1306_physics::Gear;
use embedded_trig_f32 as trig;

//                      cx  cy  pitch_r  teeth  tooth_h  tooth_w%  hub_r
let g = Gear::new(      40, 32,    14,     10,      4,      70,      4);
g.draw(&mut gfx, 0.0, true, trig::cos, trig::sin);
```

### Paire d'engrenages

```rust
use embassy_ssd1306_physics::gear::{Gear, GearPair};
use embedded_trig_f32 as trig;

let pair = GearPair::new(
    Gear::new(30, 32, 12, 8, 4, 70, 3),   // moteur (cx/cy réels)
    Gear::new(0, 0, 18, 12, 4, 70, 5),    // récepteur (cx/cy calculés)
    0.0,                                    // g2 à droite de g1
    trig::cos, trig::sin,
);

let mut angle: f32 = 0.0;
loop {
    pair.erase(&mut gfx, angle, trig::cos, trig::sin);
    angle += 0.05;
    pair.draw(&mut gfx, angle, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}
```

### Train de 3 engrenages

```rust
use embassy_ssd1306_physics::gear::{Gear, GearTrain};
use embedded_trig_f32 as trig;

let train = GearTrain::new(
    Gear::new(18, 32, 10, 8, 3, 70, 2),  // G1 moteur
    Gear::new(0, 0,  14, 12, 3, 70, 3),  // G2 intermédiaire
    Gear::new(0, 0,   8,  6, 3, 70, 2),  // G3 sortie
    0.0,                                   // alignement horizontal
    trig::cos, trig::sin,
);
train.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
```

---

## Géométrie des engrenages

```
         ┌─┐ ┌─┐ ┌─┐   ← sommet (pitch_r + tooth_h)
       ──┘ └─┘ └─┘ └──
      /                \
     │    ┌──────┐      │  ← cercle primitif (pitch_r)
     │    │  ◎   │      │  ← moyeu (hub_r)
     │    └──────┘      │
      \                /
       ──┐ ┌─┐ ┌─┐ ┌──
         └─┘ └─┘ └─┘
```

### Rapport de transmission GearPair / GearTrain

```
ω2 = -ω1 × (teeth1 / teeth2)
ω3 =  ω1 × (teeth1 / teeth2) × (teeth2 / teeth3)   [GearTrain]
```

La mise en phase est automatique : les dents se croisent proprement au point de contact.

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                Votre application                │
│  RoboticArm / Pendulum / Gear / GearPair …     │
└──────────────────────┬──────────────────────────┘
                       │ &mut Graphics
┌──────────────────────▼──────────────────────────┐
│        embassy-ssd1306-physics                  │
│   robotic_arm · pendulum · spring_mass          │
│   compass · gear (Gear / GearPair / GearTrain)  │
│                  ↓ draw_utils                   │
│   segment · arc · ring · filled_disk · rect     │
└──────────────────────┬──────────────────────────┘
                       │ line() / circle() / fill_circle()
┌──────────────────────▼──────────────────────────┐
│        embassy-ssd1306-graphics                 │
│   clipping · pixel()                            │
└──────────────────────┬──────────────────────────┘
                       │ draw_pixel()
┌──────────────────────▼──────────────────────────┐
│        embassy-ssd1306 (driver)                 │
│   framebuffer · I2C · flush()                   │
└─────────────────────────────────────────────────┘
```

---

## Disposition suggérée sur 128×64

```
┌────────────────────────────────────────────────────────────────┐
│ RoboticArm (x 0–55) │ GearPair (x 56–95) │ Pendule │ Boussole │
│ base=(20,50)         │ g1=(66,32) r=10    │ (104,6) │ (120,32) │
│ seg1=28 seg2=18      │ g2=(84,32) r=8     │  l=20   │  r=10    │
└────────────────────────────────────────────────────────────────┘
```

---

## Convention des angles

| Objet         | Référence 0°             | Sens positif       | Type       |
|---------------|--------------------------|--------------------|------------|
| `RoboticArm`  | Verticale montante (Y−)  | Vers `Facing`      | Angulaire  |
| `Pendulum`    | Verticale descendante (Y+) | Droite           | Angulaire  |
| `Piston`      | Sommet chambre (pos=0)   | Vers le bas (Y+)   | Linéaire   |
| `Compass`     | Nord (Y−)                | Horaire (Est)      | Angulaire  |
| `Gear`        | X+ horizontal            | Horaire            | Angulaire  |
| `SpringMass`  | —                        | Extension vers Y+  | Linéaire   |

---

# Licence

GNU GPL v2 ou ultérieure — voir `LICENSE`.

**Copyright (C) 2026 Jorge Andre Castro**