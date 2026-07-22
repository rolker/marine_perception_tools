// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>
#include <QCommandLineParser>
#include <QDateTime>
#include <QString>
#include <QStringList>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

#include "sidescan_viewer_window.hpp"

namespace
{

// Any real bag stamp in ns is ~1.7e18 (2020s); an integer below 1e18 (before
// Sep 2001) is not a plausible ns stamp — more likely a mistyped or
// basic-format ISO date (e.g. `20260714`), which must not be silently taken
// as nanoseconds and "cue" to 1970.
constexpr int64_t kMinPlausibleNs = 1000000000000000000LL;

// Parse a cue bound: UNIX nanoseconds (bare integer) or ISO-8601. An ISO
// string without a UTC offset is treated as UTC — bag stamps are UTC, and
// silently interpreting operator input as local time would cue hours off.
// Returns 0 and sets ok=false on unparseable (or implausible-ns) input; 0 is
// also the "no cue" sentinel, which cannot collide with real data — an
// epoch-0 stamp is no plausible bag time, and a parse yielding it fails here.
int64_t parseCueBound(const QString & text, bool & ok)
{
  ok = true;
  bool is_int = false;
  const qlonglong ns = text.toLongLong(&is_int);
  if (is_int && ns >= kMinPlausibleNs) {
    return static_cast<int64_t>(ns);
  }
  QDateTime dt = QDateTime::fromString(text, Qt::ISODateWithMs);
  if (!dt.isValid()) {
    ok = false;
    return 0;
  }
  if (dt.timeSpec() == Qt::LocalTime) {   // no offset in the string
    dt.setTimeSpec(Qt::UTC);
  }
  return static_cast<int64_t>(dt.toMSecsSinceEpoch()) * 1000000LL;
}

}  // namespace

// Offline georeferenced sidescan viewer + target tracker. Optionally takes a
// bag directory to open on launch, and a --start/--end time window to cue the
// scrub to once indexing completes (the survey-index jump-to-pass bridge —
// pass the interval straight from a `survey_index_query --json` row).
int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  QApplication::setApplicationName("sidescan_target_viewer");

  QCommandLineParser parser;
  parser.setApplicationDescription(
    "Offline georeferenced sidescan/MBES viewer + target tracker.\n"
    "With --start/--end, cues the scrub to that time window once the bag is\n"
    "indexed (jump-to-pass: values come from survey_index_query output).");
  parser.addHelpOption();
  parser.addPositionalArgument("bag", "Bag directory to open on launch.", "[bag]");
  const QCommandLineOption start_opt(
    "start", "Cue window start: UNIX nanoseconds or ISO-8601 (UTC assumed "
    "when no offset is given).", "time");
  const QCommandLineOption end_opt(
    "end", "Cue window end: UNIX nanoseconds or ISO-8601 (UTC assumed when "
    "no offset is given).", "time");
  const QCommandLineOption index_opt(
    "index", "survey_index.db to open in survey-explorer mode (#24): the map "
    "becomes the index — store-tile basemap, nav track, selectable tiles; "
    "selecting tiles loads their mbes-bathy passes into the 3D cloud.",
    "db");
  const QCommandLineOption stores_opt(
    "stores", "Directory of GGGS store GeoTIFF tiles for the explorer basemap "
    "(default: <index dir>/bathymetry/survey).", "dir");
  parser.addOption(start_opt);
  parser.addOption(end_opt);
  parser.addOption(index_opt);
  parser.addOption(stores_opt);
  parser.process(app);

  const bool has_start = parser.isSet(start_opt);
  const bool has_end = parser.isSet(end_opt);
  const QStringList positional = parser.positionalArguments();
  if (has_start != has_end) {
    std::fprintf(stderr, "error: --start and --end must be given together\n");
    return 2;
  }
  if (has_start && positional.isEmpty()) {
    std::fprintf(stderr, "error: --start/--end need a bag argument to cue into\n");
    return 2;
  }

  int64_t cue_start_ns = 0;
  int64_t cue_end_ns = 0;
  if (has_start) {
    bool ok = false;
    cue_start_ns = parseCueBound(parser.value(start_opt), ok);
    if (!ok || cue_start_ns == 0) {
      std::fprintf(stderr, "error: --start '%s' is not UNIX ns or ISO-8601\n",
        parser.value(start_opt).toUtf8().constData());
      return 2;
    }
    cue_end_ns = parseCueBound(parser.value(end_opt), ok);
    if (!ok || cue_end_ns == 0) {
      std::fprintf(stderr, "error: --end '%s' is not UNIX ns or ISO-8601\n",
        parser.value(end_opt).toUtf8().constData());
      return 2;
    }
  }

  // One window for both modes (#24): --index turns the map into the survey
  // explorer's index map; a bag positional (with or without --index) opens in
  // the same window.
  marine_perception_tools::SidescanViewerWindow window;
  if (parser.isSet(index_opt)) {
    const std::string index_path = parser.value(index_opt).toStdString();
    std::string stores_dir;
    if (parser.isSet(stores_opt)) {
      stores_dir = parser.value(stores_opt).toStdString();
    } else {
      stores_dir = (std::filesystem::path(index_path).parent_path() /
        "bathymetry" / "survey").string();
    }
    try {
      window.openSurveyIndex(index_path, stores_dir);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "error: opening survey index: %s\n", e.what());
      return 1;
    }
  }
  window.show();
  if (!positional.isEmpty()) {
    window.openBag(positional.first().toStdString(), cue_start_ns, cue_end_ns);
  }
  return app.exec();
}
