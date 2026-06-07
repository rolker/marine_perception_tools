# ADR-0001: Shared Scalar Colormap Library (`marine_colormap`)

## Status

Accepted (2026-06-07). Tracked by
[rolker/marine_perception_tools#4](https://github.com/rolker/marine_perception_tools/issues/4).

This is the first ADR in this repository; it establishes `docs/decisions/` here
for marine-software architecture decisions (distinct from the workspace's own
`docs/decisions/`, which records agent-framework decisions).

## Context

Scalar-field colormapping — mapping a value to a display color through a named
palette and a transfer function (range normalize → gain → contrast/gamma →
alpha) — is reimplemented in at least three places, all slightly differently:

- **`rviz_sonar_image::ColorMap`** (Ogre/rviz, `jazzy` branch): does the mapping
  on the **CPU** (`color_map_->lookup(value)` per sample) to fill an RGBA Ogre
  texture, re-baked whenever the range changes.
- **`rqt_sonar_waterfall::ColorMap`** (Qt, `rqt_operator_tools`): a second copy;
  its Thermal palette is literally "adapted from rviz_sonar_image".
- **`camp::map::ColorMap`** (proposed, [camp#63](https://github.com/rolker/camp/issues/63)):
  a third, still-unbuilt copy wanting `grayscale` + `viridis`/`turbo` with
  per-layer range.

Two forces motivate consolidation:

1. **Duplication.** Three copies of the same value→color logic drift apart;
   colors differ subtly between viewers of the same data.
2. **Bit depth.** We want a path that is not bound to 8-bit sonar data. The
   current CPU approach quantizes intensity to 8-bit before colormapping; a GPU
   path can keep full-precision input.

The hard constraint is that the three consumers sit on **three different
rendering substrates**, so a single shared *runtime renderer* is not possible:

| Consumer | Substrate | GPU path |
|---|---|---|
| rqt plugins | Qt widgets | `QOpenGLWidget` + GLSL (raw GL via Qt) |
| rviz displays | Ogre3D | Ogre material + Ogre-managed shader |
| CAMP | `QGraphicsScene` (CPU/QPainter) | needs a GL viewport, or stays CPU |

Live GL/texture objects cannot be shared across Qt-GL, Ogre, and QGraphicsScene.

## Decision

Extract colormapping into a layered, framework-agnostic library, and share only
the layers *below* the renderer.

### Tier 1 — `marine_colormap` core (no Qt / Ogre / GL)

A new `marine_colormap` package **inside `marine_perception_tools`**, kept as its
own package so consumers depend on `marine_colormap` and not the tuner. It
provides:

- Named palettes: `grayscale`, `bronze`, `thermal` (from the existing sonar
  copies) plus `viridis`, `turbo` (camp#63), with a name⇄index registry.
- The transfer function: normalize(min, max) → gain → contrast/gamma → optional
  alpha range, matching the existing `scale_intensity` semantics so colors do
  not shift on migration.
- `bake_lut(colormap, N) -> std::vector<Rgba>` (the 1-D LUT the GPU phase
  uploads as a texture) and a CPU `lookup(value, min, max, ...) -> Rgba`.
- A plain `Rgba` value type. Consumers convert to `QColor` /
  `Ogre::ColourValue` in one line — **the core depends on neither Qt nor Ogre**.

Pure C++, unit-testable with **no GL context** (so it runs in headless CI). This
is the single source of truth for appearance: identical colors in rqt, rviz, and
CAMP.

### Tier 2 — shared GLSL source + LUT-as-texture (later phase)

One fragment shader (scalar sampler + 1-D LUT sampler + `min`/`max`/`gain`/
`contrast` uniforms), shipped as a resource string with a `#version` shim for
desktop GL vs GLES. The shader *math* and the baked LUT *bytes* are shared; the
GL/Ogre **binding boilerplate stays per-consumer**.

Full-precision input is achieved by carrying raw samples in an **R16/R32F**
scalar texture and normalizing in the shader — input is never quantized to 8-bit
before colormapping. The LUT only quantizes the 8-bit color *output* (fine for
display). The CPU fallback necessarily quantizes input — the accepted tradeoff
for CAMP without a GL viewport.

### Seam

Each consumer keeps its own small renderer integration but depends on the same
Tier 1 core (and, in the GPU phase, the same Tier 2 shader source). We do **not**
build a shared runtime renderer; forcing one would couple rviz to Qt-GL (or vice
versa).

### Sequencing

1. **This effort (first cut):** Tier 1 core + migrate the existing **CPU**
   consumers (`rqt_sonar_waterfall`, `rviz_sonar_image` on `jazzy`) onto it;
   satisfy camp#63 by adopting the shared core (CPU) rather than a camp-local
   `ColorMap`. No GPU yet.
2. **Follow-up phase:** Tier 2 GLSL renderer for the rqt family
   (`QOpenGLWidget`) — the >8-bit waterfall; then an Ogre GPU material for
   `rviz_sonar_image`; CAMP GPU only if it moves to a GL viewport.

## Consequences

- **Positive:** one palette/transfer definition; identical colors across viewers;
  headless-testable core; a clean place to add palettes; the >8-bit GPU path has
  a shared shader + LUT to build on without re-litigating the colormap.
- **Cost / risk:** `rviz_sonar_image` and `camp` now depend on
  `marine_perception_tools`. Mitigated by keeping `marine_colormap` a standalone
  package within the repo (depend on the package, not the tuner). A cleaner
  long-term option is its own repo; revisit if the dependency proves awkward.
- **Substrate divergence remains:** the GPU phase still requires per-consumer
  renderer work (Qt-GL, Ogre, QGraphicsScene). Tier 2 shares the shader source
  and LUT, not the plumbing.
- **CPU fallback quantizes input** to 8-bit; only the GPU path is truly
  bit-depth-independent.

## References

- Umbrella: [rolker/marine_perception_tools#4](https://github.com/rolker/marine_perception_tools/issues/4)
- Core lib: [rolker/marine_perception_tools#5](https://github.com/rolker/marine_perception_tools/issues/5)
- rqt migration: [rolker/rqt_operator_tools#44](https://github.com/rolker/rqt_operator_tools/issues/44)
- rviz migration (jazzy): [rolker/rviz_sonar_image#4](https://github.com/rolker/rviz_sonar_image/issues/4)
- CAMP adoption: [rolker/camp#63](https://github.com/rolker/camp/issues/63)
