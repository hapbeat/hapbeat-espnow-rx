// ---------------------------------------------------------------------------
// HapbeatOpusBackend.cpp — the backend registry (see header).
//
// Deliberately contains no Opus code and never includes <opus.h>: the concrete
// decoder lives in HapbeatOpusBackendImpl.h and is compiled into the sketch,
// which is the only translation unit that gets the project's include paths.
// Keeping this file dependency-free is what lets an ADPCM-only user build the
// library without libopus present at all.
// ---------------------------------------------------------------------------

#include "HapbeatOpusBackend.h"

namespace hapbeat {
namespace {
OpusBackend* g_backend = nullptr;
}

void registerOpusBackend(OpusBackend* backend) { g_backend = backend; }

OpusBackend* defaultOpusBackend() { return g_backend; }

bool opusAvailable() { return g_backend != nullptr; }

} // namespace hapbeat
