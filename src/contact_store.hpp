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

#ifndef CONTACT_STORE_HPP_
#define CONTACT_STORE_HPP_

#include <string>
#include <vector>

#include "marine_interfaces/msg/contact.hpp"

// Manual-curation contact store for the sidescan viewer (PR3). Holds human-drawn
// contacts as the unified marine_interfaces/msg/Contact, persists them as a CDR
// ContactArray file, and answers a map-frame bounding-box query for the cross-pass
// overlay. Qt-free so the store + box geometry + serialization round-trip are
// unit-tested without a display. The CDR ContactArray format is forward-compatible
// with the contact_manager standalone store (unh_marine_autonomy#167).

namespace marine_perception_tools
{

// A 2D point in the map (bizzy/map ENU) plane, metres.
struct MapPoint
{
  double x = 0.0;
  double y = 0.0;
};

// Build a human-origin BOX Contact from the map-frame footprint of a drawn box.
// `points` are the box corners (or any point set) in map metres; the contact's
// kinematics pose is their centroid, the BOX dimensions their extent. The contact
// is STATUS_PROPOSED, existence_probability 1.0, frame_id `frame`. `geo_pose` is
// left unresolved (latitude = NaN) — lat/lon resolution is a follow-up; same-datum
// overlay uses the map-frame pose directly.
marine_interfaces::msg::Contact make_box_contact(
  const std::vector<MapPoint> & points, const std::string & id,
  const std::string & source, const std::string & frame, double stamp_s);

class ContactStore
{
public:
  void add(const marine_interfaces::msg::Contact & contact) {contacts_.push_back(contact);}
  void clear() {contacts_.clear();}
  const std::vector<marine_interfaces::msg::Contact> & contacts() const {return contacts_;}
  std::size_t size() const {return contacts_.size();}

  // Contacts whose map-frame position lies within [min_x, max_x] x [min_y, max_y].
  std::vector<const marine_interfaces::msg::Contact *> inBox(
    double min_x, double min_y, double max_x, double max_y) const;

  // Persist / load all contacts as a CDR-serialized ContactArray. Returns false on
  // any file or (de)serialization error.
  bool save(const std::string & path) const;
  bool load(const std::string & path);

private:
  std::vector<marine_interfaces::msg::Contact> contacts_;
};

}  // namespace marine_perception_tools

#endif  // CONTACT_STORE_HPP_
