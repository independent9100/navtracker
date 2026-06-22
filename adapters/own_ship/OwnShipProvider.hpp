#pragma once

// OwnShipProvider has moved to core/ (Phase 8 architecture fix —
// it's a pure domain type with no I/O, so it belongs alongside the
// other core/own_ship/ helpers, not under adapters/). This shim
// preserves the existing include path used by ~37 callers; new code
// should include "core/own_ship/OwnShipProvider.hpp" directly.
#include "core/own_ship/OwnShipProvider.hpp"
