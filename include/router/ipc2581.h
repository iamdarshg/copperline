// Copperline: IPC-2581C import (minimal but real).
//
// Documented subset (anything else warns, never silently):
//   <Ipc2581|Ipc-2581|Ecads|Ecad|Content unit|units|uom="mm|inch|mil|um|nm">
//   <Datum width|size_x|boardWidth height|size_y|boardHeight/> (board size;
//     absent => profile bbox, else copper bbox + 1mm margin, warned)
//   <Layer|SignalLayer|CopperLayer|ConductorLayer name type>
//     (type plane|power|ground => plane layer, else signal)
//   <Net|LogicalNet|PhysicalNet|Signal name|net>
//   <NetClass|Class name width|traceWidth clearance|spacing>
//     with <Net|Member|NetRef name/> children (maps to min_width /
//     min_clearance floors)
//   <Component|Part|Footprint|Placement refDes|ref|designator|name>
//     with <Pad|Pin|Terminal net|netName x|posX|cx y|posY|cy
//     width|sizeX|sx|dx|dia|diameter|size height|sizeY|sy|dy shape
//     layer|side/> children; standalone <Pad refDes .../> also accepted.
//     Unknown nets/layers skip the pad with an explicit warning.
//   <Trace|Wire|Track|Conductor|Route net layer
//     x1|startX y1|startY x2|endX y2|endY width|w/> (unknown net => warn
//     + skip; unknown layer => default Top + warn)
//   <Via|PlatedHole net x y outer|diameter|size hole|drill
//     top|topLayer bottom|bottomLayer/>
//   <Profile|Outline|BoardOutline|Perimeter> with <Polygon
//     points="x1,y1 x2,y2 ..."/> | <Rect x1 y1 x2 y2/> | raw
//     <Point|Vertex|Xy x y/> children (outline only).
//
// Honest limits: flat-subset only. No stackup dielectrics, padstack
// libraries, embedded components, hierarchy, DFM rules, BOM data, or
// multi-step panels (first <Step> only, warned). Tag/attribute matching
// is case-insensitive. Streaming chunk scan (64KB, no DOM); polygon
// vertices capped (32k, truncate warned); primitives capped (2M, error);
// unknown top-level elements summarized in one warning (cap 12 names).
#pragma once

#include <string>

#include "router/board.h"

namespace copperline {

// IPC-2581C importer (registered in import_board_auto for IPC-2581 XML
// content markers / extensions).
class Ipc2581Importer : public BoardImporter {
  public:
    std::string format_name() const override { return "ipc-2581"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override;
    ImportResult import_file(const std::string& path) const override;
    ImportResult import_text(const std::string& text, const std::string& path) const;
};

// Registry singleton (defined in ipc2581.cpp; referenced by board.cpp).
const BoardImporter& ipc2581_importer_singleton();

}  // namespace copperline
