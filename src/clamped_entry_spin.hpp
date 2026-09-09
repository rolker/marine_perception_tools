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

#ifndef CLAMPED_ENTRY_SPIN_HPP_
#define CLAMPED_ENTRY_SPIN_HPP_

#include <QAbstractSpinBox>
#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QLineEdit>
#include <QString>
#include <QWidget>

#include <cmath>
#include <functional>
#include <utility>

namespace marine_perception_tools
{

// A spin box that SAYS SO when it rewrites what was typed (#42).
//
// Qt treats spin-box text below the minimum as not-yet-valid (Intermediate:
// "0.01" could still grow into something in range) and quietly corrects it
// when the edit is committed — on focus-out, which is exactly the moment the
// operator clicks the button that consumes the value. The operator's report:
// typing 0.01 into the CUBE cell size, whose floor was 0.02, and pressing Run
// CUBE, which ran at 0.1 — the value from before the edit — with nothing on
// screen saying the entry had been dropped. Two earlier investigations went
// hunting for a stray setValue(); there was none. The silence is the defect.
//
// (Text ABOVE the maximum is different, and does not need this: "60" in a box
// that stops at 50 cannot grow into anything valid, so Qt rejects the
// keystroke outright and the digit visibly never appears.)
//
// Widening a range only moves where the correction happens, so the correction
// itself is made audible: the commit paths — focus-out and Return/Enter —
// compare the text as typed against the value that resulted and hand both to
// a notice callback when they differ. The caller phrases the message in its
// own units and puts it wherever that window reports things (the status line
// here). Never a modal: a correction the operator can read is not worth one.
//
// Deliberately not a QObject: a std::function member needs no moc, and the
// notice has exactly one owner — the window that built the widget.
class ClampedEntryDoubleSpinBox : public QDoubleSpinBox
{
public:
  explicit ClampedEntryDoubleSpinBox(QWidget * parent = nullptr)
  : QDoubleSpinBox(parent)
  {
    // Qt's default is CorrectToPreviousValue: an entry it will not accept is
    // replaced by whatever was in the box BEFORE the edit — a value the
    // operator may have set minutes ago and is no longer thinking about. The
    // nearest valid value is at least an answer to what was typed, and it is
    // never adopted silently: the notice fires on the same commit.
    setCorrectionMode(QAbstractSpinBox::CorrectToNearestValue);
  }

  // Called with (value as typed, value actually applied) whenever a commit
  // turned one into the other. Not called when the entry survived intact.
  void setClampNotice(std::function<void(double, double)> notice)
  {
    notice_ = std::move(notice);
  }

protected:
  void focusOutEvent(QFocusEvent * event) override
  {
    reportCorrectionAround([this, event]() {QDoubleSpinBox::focusOutEvent(event);});
  }

  void keyPressEvent(QKeyEvent * event) override
  {
    // Only the keys that COMMIT the text: any other keystroke is mid-edit,
    // where the text legitimately does not yet match the value.
    if (event->key() != Qt::Key_Enter && event->key() != Qt::Key_Return) {
      QDoubleSpinBox::keyPressEvent(event);
      return;
    }
    reportCorrectionAround([this, event]() {QDoubleSpinBox::keyPressEvent(event);});
  }

private:
  // The typed text as a number, in the widget's own locale and without its
  // prefix/suffix decoration. Fails on text Qt would not read as a number
  // either — there is nothing to report about that.
  bool typedValue(double * out) const
  {
    const QLineEdit * edit = lineEdit();
    if (!edit) {
      return false;
    }
    QString text = edit->text();
    if (!prefix().isEmpty() && text.startsWith(prefix())) {
      text.remove(0, prefix().size());
    }
    if (!suffix().isEmpty() && text.endsWith(suffix())) {
      text.chop(suffix().size());
    }
    bool ok = false;
    const double parsed = locale().toDouble(text.trimmed(), &ok);
    if (ok) {
      *out = parsed;
    }
    return ok;
  }

  template<typename Commit>
  void reportCorrectionAround(Commit && commit)
  {
    double typed = 0.0;
    const bool readable = notice_ && typedValue(&typed);
    commit();
    if (!readable) {
      return;
    }
    // Below the smallest difference the box can display, "typed" and
    // "applied" are one number shown two ways, not a correction.
    if (std::fabs(typed - value()) < 0.5 * std::pow(10.0, -decimals())) {
      return;
    }
    notice_(typed, value());
  }

  std::function<void(double, double)> notice_;
};

}  // namespace marine_perception_tools

#endif  // CLAMPED_ENTRY_SPIN_HPP_
