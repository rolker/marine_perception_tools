# Vendored world coastline

`ne_50m_coastline.txt` is the world coastline `survey_explorer` draws underneath
the index map so an operator can tell where they are at collection-wide zoom
(issue #41). It is compiled into the installed package: neither the build nor
the application ever reaches the network, because the operator station and the
boat have no route to one.

## Provenance

| | |
|---|---|
| Dataset | Natural Earth 1:50m physical coastline (`ne_50m_coastline`) |
| Version | 4.1.0 (`ne_50m_coastline.VERSION.txt` in the upstream archive) |
| Source | <https://naturalearth.s3.amazonaws.com/50m_physical/ne_50m_coastline.zip> |
| Retrieved | 2026-09-09 |
| Terms | Public domain. Natural Earth places all its map data in the public domain: "no permission is needed to use Natural Earth. Crediting the authors is unnecessary." See <https://www.naturalearthdata.com/about/terms-of-use/>. |
| Converted by | `regenerate.sh 50m`, which calls `encode_coastline.py` |
| Committed artifact | 1429 polylines, 60416 points, sha256 `7ad01d2e7e70e9e0e90a00e05afb7283d0821420c7a2138b445c10e416649dcc` |

`regenerate.sh` is a hand-run re-vendoring tool, not a build step. Re-run it
only to take a newer Natural Earth release, and update this table with the
version, date and checksum it prints.

## What this data is, and is not

It is **orientation, not navigation**. A 1:50m generalised coastline is wrong by
hundreds of metres to a couple of kilometres at survey scale, and the vendored
form quantises it further to milli-degrees (~111 m of latitude — an order of
magnitude finer than the source generalisation, so the quantisation is not the
limiting error). The explorer therefore draws it beneath every real layer and
fades it out entirely before survey zoom (`coastlineFadeAlpha` in
`src/coastline_data.hpp`). Nothing about it may be treated as chart detail.

## Format

Plain ASCII text, one record per line:

```
# comment — ignored (the committed file carries its provenance in a header)
> <lat> <lon>     start a polyline; absolute position in milli-degrees
<dlat> <dlon>     the next point, as a delta in milli-degrees from the previous
```

The parser is `parseCoastline()` in `src/coastline_data.hpp`; a malformed or
out-of-range record drops the polyline it belongs to and parsing resumes at the
next `>`, so a damaged file degrades to less coastline rather than to a crash.

**Why not vendor the GeoJSON.** GDAL is already a dependency, so OGR could read
a GeoJSON directly, and that was the first choice. The same 60416 points come
to ~1.5 MB as GeoJSON — three times the repository's 500 kB large-file limit —
and getting under that limit as GeoJSON costs real geometry: simplifying to
0.05° (5 km) still leaves ~600 kB. Delta-encoded integers keep the source
geometry intact at 453 kB, at the cost of a ~40-line parser that is unit-tested
(`test/test_coastline_data.cpp`) including the empty and malformed cases. The
tradeoff is fidelity of a third-party dataset against a small amount of
parsing code, and this directory's `regenerate.sh` keeps the conversion
reproducible from the published source.
