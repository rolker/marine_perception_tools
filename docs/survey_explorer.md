# Survey Explorer — Design

The durable half of the survey explorer: what it is for, and the decisions that
constrain it. Sequencing lives in the umbrella issue
([#36](https://github.com/rolker/marine_perception_tools/issues/36)), which
changes constantly; this document holds what should not.

Its predecessor was
[unh_marine_autonomy#258](https://github.com/rolker/unh_marine_autonomy/issues/258),
a five-stage umbrella that closed on 2026-07-14 — the day stage 1 merged — while
the work it described continued for another seven weeks. The decisions below were
recorded there and in its 2026-07-16 amendment comment; they are reproduced here
because a closed issue in another repository is not a discoverable home for a
tool's founding reasoning.

**Cross-repo citation convention** (from `uma-ADR-0013`): ADR references are
repo-qualified — `uma-ADR-00NN` for `unh_marine_autonomy`, `camp-ADR-00NN` for
`camp`. Numbers collide across repos; a bare "ADR-0013" is ambiguous.

## What it is for

Finding and examining the **raw, un-averaged data behind a location**.

The stores are averaged products — CUBE fusion, best-source compositing. They
answer "what did we accept". Target-level work asks a different question: given a
spot on the map, which passes ensonified it, what did the raw soundings and the
single-pass sidescan look like, and is there something there worth re-surveying
or diving on.

The motivating case (2026-07-13) was the Massabesic ball-turret search: `camp`
and the stores showed the area was partially covered, and there was no way to get
from that to the data files worth examining. That specific job is done. The tool
it produced is now the working interface to the world model for anything that is
not realtime.

## Constraining decisions

Each of these has a reason. Change them deliberately, not by drift.

### Tile-based, not bag-based

The unit of navigation is a geographic tile, not a recording. A location is the
question; bags are an implementation detail of where the answer is stored.

### Index from ping geometry, not store acceptance

The survey index records **where the sensor looked**, not what the stores
accepted. This is the difference that makes the tool work: pings that never
landed in a store cell — coverage gaps, filter rejections, CUBE blunder
rejections, false detections — are exactly the ones worth examining when
something is missing from a surface. An index built from store contributions
could not find them.

### Bags are the data of record

The index and the stores are both derived caches, regenerable from the bags
(the `cube_bathymetry#96` philosophy). The bags themselves are never rewritten.

**Corollary settled 2026-09-03:** soundings are *not* copied into the stores.
The bags stay on local disk or NAS and remain reachable, and measurement shows
random access is cheap — seeking anywhere in a 7.6-hour, 17.7 GB MCAP bag costs a
flat ~22 ms regardless of position (the index is used, not a scan), with ~285 ms
to open a bag and ~1 GB/s sequential reads afterwards. A typical tile is touched
by one or two bags. There is therefore no performance case for a per-tile
sounding store, and the durability case does not apply while the bags are online.

Had one been built, the right form would have been **detections with two-way
travel time**, never transducer-relative point clouds: a point cloud has already
divided travel time by a sound speed, which is precisely the quantity the
sound-speed inversion work (`uma#300`) re-estimates. Recorded here so the
question is not reopened from scratch.

### A single pass is the unit of sidescan interpretation

Blending passes destroys shadows, and shadows are how targets are read.
Single-pass display everywhere raw sidescan is shown; composites are for coverage
overview only, and resolve per-cell conflicts by picking a best pixel rather than
averaging.

This decision does more work than it looks: it is also why **zoom cannot drive
the detail panes**. A viewport contains hundreds of passes and "load the sidescan
for this view" has no defined answer. Only a person can choose the pass.

### The map *is* the index

The overview map showing georeferenced store imagery is the index interface.
There is no separate index widget. Selecting tiles on the map loads the relevant
raw data into the other views; the pass list is a **detail readout of the
selection, not the primary picker**. (uma#258 amendment, 2026-07-16.)

### Zoom drives rasters; selection drives detail

Following from the two decisions above. Raster layers — depths, backscatter,
reference, chart — load by viewport like a chart plotter. Observation views —
waterfall, echogram, MBES cloud — load only on selection.

Between them sits a cheap **availability tier** that makes selection possible:
pass density, nav track with direction, sensor and date-range readout, all
derived from the index without opening a bag. The map shows there is something
there; the user selects it; the panes fill.

Auto-loading when only one pass is present is deliberately rejected: "sometimes
it loads" is harder to learn than "it never loads until you ask", and it makes
zoom occasionally destructive of pane state.

### Orientation is not navigation

A collection-wide view is a few specks of store imagery on black: the operator
cannot tell one survey area from another without already knowing where they
are. Every background source `camp` offers is a network service, and the
explorer runs on the operator station and in the field, where none of them are
reachable. So the coastline is **compiled into the application** — public-domain
Natural Earth 1:50m, vendored under `data/coastline/`, never fetched at build
or run time.

What that buys is orientation, and nothing else. A generalised world coastline
is wrong by hundreds of metres to a couple of kilometres at survey scale, and
this is a tool whose other layers are trusted to the centimetre. The rule that
keeps the two from being confused is a **scale rule, not a style choice**: the
layer draws beneath every real layer, and its opacity falls to zero before the
scales at which the store basemap and the index answer the question
(`coastlineFadeAlpha`, `src/coastline_data.hpp`). Anything that would let it
read as authoritative at survey zoom is a defect. The toggle beside track and
grid is a declutter control, not the mechanism — an operator must not have to
switch it off to be safe.

The same reasoning is why a network chart layer is a separate question and not
a variation on this one: a chart that is only sometimes reachable cannot carry
orientation, and one that is reachable is authoritative enough to be drawn on
its own terms.

### One window, docked panes

Separate top-level windows were scaffolding. The end state is a single main
window whose map/index pane drives selection, with the other views as docks.

### The survey index is regenerable; an import ledger is not

The survey index may be deleted and rebuilt from the bags at any time. Anything
that records an **irreversible** act — which bags have been folded into an
accumulating store — must not live in it, because a routine reindex would destroy
it. CUBE accumulation is not idempotent: re-importing a bag adds its soundings to
the hypotheses again and yields a more confident wrong answer with no visible
symptom. Regenerability is the discriminator, and it decides where such state
lives (with the store, beside its provenance sidecar), not convenience.

## What the explorer is not

- **Not a realtime tool.** Monitoring, mission planning and live coverage stay in
  `camp`. `camp` does not grow exploration features and the explorer does not
  grow a live view. Neither half is diluted by pretending to be the other.
- **Not a general GIS.** It inherits the scope `uma-ADR-0010` D12 sets for the
  world model. What that excludes is D12's to state and is deliberately not
  enumerated here — for the reason the next bullet gives.
- **Not a second source of truth about the stores.** The store contract, the
  world-model taxonomy and the LOD model live in `unh_marine_autonomy`'s ADRs.
  This document references them; it must never restate them, or it becomes a
  fourth namespace disagreeing about the same data.

### Not a store-editing tool

Ingest (umbrella #36, direction 4) puts a UI on `import_bag` — a
`cube_bathymetry` tool, not part of this repo — which already writes to the
stores. It gives the operator a way to invoke
it — discovery, confirmation, progress — rather than asking an agent to run it.
The explorer does not otherwise modify store contents: the CUBE lab's surfaces
are in-memory and export to a file the user names, and nothing edits tiles in
place.

Two obligations follow from the ingest path specifically, and are worth stating
because they are easy to miss when adding the UI: an import ledger before
discovery is automated (see the regenerability decision above), and — because
this is the first time a fold runs *while the operator is exploring the same
store* — atomic tile writes
([cube_bathymetry#135](https://github.com/rolker/cube_bathymetry/issues/135))
plus a refresh story for tiles that change under a composited view.

## Directions

Stated as constraints and open questions. Sequencing and sub-issues are in #36.

**Navigating all the world stores.** The explorer renders one layer at a time;
the world model is a taxonomy. Reaching `camp` parity means layer lists, per-layer
LOD state and compositing — all of which `camp` already has. `src/basemap_lod.cpp`
is a hand-port of camp's level selector made three days before `uma-ADR-0013`,
whose **D7** requires a single renderer-agnostic selection core precisely to stop
this. *Open question: build the D7 core, or accept a second divergent
implementation.* This is a decision, not a task.

**Filtering by track geometry.** Straightness is already computed per ping in
`drape_pass()` and used only to weight conflicts. Exposing it as a filter — and
precomputing it into the index from the decimated nav track — turns "show me the
straight-running data" into a query answerable without opening a bag. Generalises
to speed, altitude and heading stability.

**A processing lab with a way out.** The CUBE lab runs the real library on an
operator-chosen box with editable parameters. Its purpose has shifted from
exploration to *deciding whether to process the whole dataset a given way*, which
requires the parameters to become a serializable **recipe** rather than dialog
state, and a promotion path from a tuned box to a batch run.
*Open question: what a recipe is, and whether it is also what an ingest run
carries.*

**Ingest and dataset grouping.** A UI over `cube_bathymetry`'s existing
`import_bag`: discovery of new bags, confirmation, background fold. Prerequisite: the import
ledger. The same ledger carries the campaign per
import record, which retires the store-level `survey` scalar that was overwritten
on 2026-08-25 (`massabesic-jun2026` → `shoals-aug2026`, with both campaigns'
data present and only one named). Project grouping is then a tag on ledger
entries — additional metadata, no new stores. *Open questions: bag identity
across a NAS move; how an interrupted import is represented.* The ledger itself
is a store-contract change and belongs in `unh_marine_autonomy`.

## References

- [#36](https://github.com/rolker/marine_perception_tools/issues/36) — umbrella,
  sequencing and sub-issues
- [unh_marine_autonomy#258](https://github.com/rolker/unh_marine_autonomy/issues/258)
  — the original umbrella (closed) and its 2026-07-16 amendment comment
- `uma-ADR-0010` — the geospatial world model: taxonomy, datum invariant,
  per-layer LOD
- `uma-ADR-0011` — the overview pyramid sidecar
- `uma-ADR-0013` — bounded LOD navigation: geometric error, declared coverage,
  and the D7 shared selection core
- `uma-ADR-0005` — cross-store provenance; its re-introduction trigger is
  relevant to the ledger
- `unh_marine_autonomy/docs/survey_index_schema.md` — the index schema contract
- `unh_marine_autonomy/docs/sonar_ecosystem.md` — where the explorer sits in the
  sonar chain
