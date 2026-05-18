# Changelog

Tous les changements notables de ce projet sont documentés dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet respecte la [Gestion sémantique de version](https://semver.org/spec/v2.0.0.html).

---

## [0.2.0] — 2026-05-18

### Ajouts

- **Module `piston`** : Nouvelle struct `Piston` pour animation mécanique 2D
  - Chambre cylindrique fermée avec piston contraint
  - Tige guidée (liaison rigide)
  - Dimensions et position du piston configurables
  - Fondation future pour moteur thermique / vilebrequin
  - Exemple : `Piston::new(64, 10, 20, 40)` avec `set_pos()` pour mouvement vertical

### Modifications

- **Documentation `pendulum`** : Clarté et complétude considérablement améliorées
  - Explication détaillée de la géométrie : « angle mesuré depuis la verticale descendante » (0 = repos)
  - Rendus clarifiés : encastrement (rect + hachures diagonales), tige, pivot (r=1), masse (r=3)
  - Exemples enrichis :
    - Constructeur par défaut avec angles positifs/négatifs
    - Encastrement personnalisé via `with_wall()`
    - Boucle d'animation simple avec `erase()` et `draw()`
  - Correction de la description dans la table « Objets disponibles »

---

## [0.1.0] — Première publication

- `RoboticArm` : Bras robotique 2 segments (épaule + coude)
- `Pendulum` : Pendule simple (pivot fixe + tige + masse)
- `SpringMass` : Système ressort-masse vertical
- `Compass` : Rose des vents avec cardinaux
- `Gear`, `GearPair`, `GearTrain` : Animations d'engrenages avec synchronisation
- `draw_utils` : Primitives partagées (segment, arc, disque, rect)
