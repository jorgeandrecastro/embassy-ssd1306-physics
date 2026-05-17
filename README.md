# embassy-ssd1306-physics

Dessins 2D `no_std` d'objets physiques pour écrans OLED SSD1306 (128×64),  
construite au-dessus de [`embassy-ssd1306-graphics`](https://crates.io/crates/embassy-ssd1306-graphics).

---

## Objets disponibles

| Module          | Struct(s)               | Description                                      |
|-----------------|-------------------------|--------------------------------------------------|
| `robotic_arm`   | `RoboticArm`            | Bras robotique 2 segments (épaule + coude)       |
| `pendulum`      | `Pendulum`              | Pendule simple (pivot + tige + masse)            |
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

## Structure des modules

```
src/
├── lib.rs           ← déclarations + re-exports
├── draw_utils.rs    ← segment, arc, disque, rect (partagés)
├── robotic_arm.rs   ← RoboticArm
├── pendulum.rs      ← Pendulum
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

```rust
use embassy_ssd1306_physics::RoboticArm;
use embedded_trig_f32 as trig;

let arm = RoboticArm::new(20, 50, 28, 18);
// Épaule à 45°, coude à -30° relatif
arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
```

### Pendule

```rust
use embassy_ssd1306_physics::Pendulum;
use embedded_trig_f32 as trig;

let pend = Pendulum::new(64, 6, 28);
pend.draw(&mut gfx, 0.4, true, trig::cos, trig::sin); // ~23° à droite
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

| Objet         | Référence 0              | Sens positif       |
|---------------|--------------------------|--------------------|
| `RoboticArm`  | Horizontal droit (X+)    | Horaire (vers Y+)  |
| `Pendulum`    | Verticale (repos en bas) | Droite             |
| `Compass`     | Nord (Y–)                | Horaire (Est)      |
| `Gear`        | X+ horizontal            | Horaire            |
| `SpringMass`  | Sans angle               | Extension vers Y+  |

---

# Licence

GNU GPL v2 ou ultérieure — voir `LICENSE`.

**Copyright (C) 2026 Jorge Andre Castro**