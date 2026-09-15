// Copperline: Gerber RS-274X import (minimal but real).
//
// Documented subset (anything else warns, never silently):
//   %FS..*% format (zero omission T/L, absolute/incremental, X/Y digits);
//   %MO..*% units (MM/IN); %AD..*% apertures D10+ (C/R/O/P shapes +
//   macro subset: primitives 1 circle / 21 rect as bbox, others warn and
//   approximate as a circle of their first parameter); %AM..*% macro
//   definitions (stored, only the subset above is honored); %LP..*%
//   polarity (dark = copper, clear = cutout keepout); G01/G02/G03/G75
//   interpolation (arcs linearized, <= 256 segments, warned); G36/G37
//   regions (dark = PlaneZone pour, clear = bbox keepout); G04 comments
//   (COPPERLINE:PROFILE forces profile mode); %TF.FileFunction,..*%
//   (Profile => outline-only; non-copper functions warn, still parsed);
//   %SR..*% step-repeat (replicated, bounded); D01 draw / D02 move /
//   D03 flash; M02 / M00 end.
//
// Honest limits: a Gerber file carries no netlist, so ALL dark copper
// lands on one net ("COPPER", id 0): flashes -> pads, draws -> traces,
// dark regions -> PlaneZone pours (own island each). Clear features
// become rect keepouts. No buried/blind structure, no soldermask/paste
// semantics. Boards without a profile get outline = copper bbox + 1mm
// (warned). Streaming line parse; polygon vertices capped (32k, truncate
// warned); total primitives capped (2M, hard error).
#pragma once

#include <string>

#include "router/board.h"

namespace copperline {

// Gerber RS-274X importer (registered in import_board_auto for Gerber
// extensions / RS-274X content markers).
class GerberImporter : public BoardImporter {
  public:
    std::string format_name() const override { return "gerber"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override;
    ImportResult import_file(const std::string& path) const override;
    ImportResult import_text(const std::string& text, const std::string& path) const;
};

// Registry singleton (defined in gerber.cpp; referenced by board.cpp).
const BoardImporter& gerber_importer_singleton();

}  // namespace copperline
