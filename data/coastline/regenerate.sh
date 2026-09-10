#!/usr/bin/env bash
# Regenerate the vendored world coastline from Natural Earth.
#
# This script is NOT run by the build or by the application: the converted
# artifact is committed, and neither the build nor survey_explorer ever
# reaches the network (issue #41). Run it by hand only to re-vendor a newer
# Natural Earth release, and commit the result together with the version and
# retrieval date recorded in README.md.
#
# Requires: curl, unzip, ogr2ogr (GDAL), python3.

set -euo pipefail

SCALE="${1:-50m}"          # 50m (vendored) or 110m (coarser fallback)
OUT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${OUT_DIR}/ne_${SCALE}_coastline.txt"
URL="https://naturalearth.s3.amazonaws.com/${SCALE}_physical/ne_${SCALE}_coastline.zip"

WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

echo "Downloading ${URL}"
curl -sSL -o "${WORK}/ne.zip" "${URL}"
unzip -q "${WORK}/ne.zip" -d "${WORK}/ne"
# The upstream VERSION.txt is CRLF-terminated; the vendored file is not.
VERSION="$(tr -d '\r\n' < "${WORK}/ne/ne_${SCALE}_coastline.VERSION.txt" 2>/dev/null || echo unknown)"

# GeoJSON is only an intermediate: RFC7946 normalises to WGS84 lon/lat and
# splits any geometry crossing the antimeridian, which the explorer's
# equirectangular canvas cannot span (it has no wrap handling).
ogr2ogr -f GeoJSON -select "" -lco RFC7946=YES -lco COORDINATE_PRECISION=6 \
  "${WORK}/coastline.json" "${WORK}/ne/ne_${SCALE}_coastline.shp"

python3 "${OUT_DIR}/encode_coastline.py" \
  --input "${WORK}/coastline.json" \
  --output "${OUT}" \
  --source "${URL}" \
  --dataset "Natural Earth 1:${SCALE} physical coastline" \
  --version "${VERSION}"

echo "Wrote ${OUT} ($(stat -c%s "${OUT}") bytes)"
echo "sha256: $(sha256sum "${OUT}" | cut -d' ' -f1)"
