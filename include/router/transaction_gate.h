// Copperline: shared transactional-mutation gate (D6).
//
// One home for the snapshot/mutate/verify-or-revert pattern previously
// cloned across the optimizer gate, the optimizer per-mutation attempt,
// and the tuner accept (optimizer.cpp gate_ok + attempt, tuning.cpp
// accept). engine.cpp/main.cpp copies belong to Stream-3 and are
// deliberately untouched.
//
// Semantics (behavior-preserving):
//   - verification is always the independent BoardVerifier over committed
//     copper (no engine-state coupling);
//   - on any verify failure the board is restored verbatim from the
//     snapshot (deterministic, exception-neutral: callers stay noexcept
//     as before; std::bad_alloc aside the board is never half-mutated);
//   - the optional accept predicate covers caller-specific extras
//     (optimizer must_shorten, tuner in_window) evaluated AFTER verify;
//     revert covers both rejections identically to the inlined clones.
// Header-only so no build re-configure is needed.
#pragma once

#include <utility>

#include "router/board.h"
#include "router/rules.h"
#include "router/verifier.h"

namespace copperline {

class TransactionGate {
  public:
    TransactionGate(Board* board, const RuleResolver* resolver,
                    const ElectricalContext* ctx)
        : board_(board), resolver_(resolver), ctx_(ctx) {}

    // Full independent verify of the current copper (the FINAL gate always
    // uses this; delta pre-checks in S3 never replace it).
    VerifyResult verify() const {
        BoardVerifier verifier;
        return verifier.verify(*board_, *resolver_, *ctx_);
    }

    bool check() const { return verify().ok; }

    // Snapshot, mutate, full-verify, revert unless (verify ok && accept()).
    // accept defaults to "always" (pure verify gate). Returns true when the
    // mutation was kept. When vr_out is non-null it receives the
    // post-mutate VerifyResult in all cases.
    template <typename MutateFn, typename AcceptFn>
    bool try_apply(MutateFn&& mutate, AcceptFn&& accept,
                   VerifyResult* vr_out = nullptr) {
        Board backup = *board_;
        mutate();
        VerifyResult vr = verify();
        bool kept = vr.ok && static_cast<bool>(accept());
        if (!kept) *board_ = backup;
        if (vr_out) *vr_out = vr;
        return kept;
    }

    template <typename MutateFn>
    bool try_apply(MutateFn&& mutate, VerifyResult* vr_out = nullptr) {
        return try_apply(std::forward<MutateFn>(mutate), [] { return true; },
                         vr_out);
    }

    // S3 delta pre-check: snapshot, mutate, then run the scoped sound-reject
    // (touched net near touched bbox) BEFORE the full verify. A scoped hit
    // reverts immediately (the hit is a definite violation); otherwise the
    // FINAL full verify + accept decide exactly as in try_apply. No behavior
    // change: scoped-pass always falls through to the full gate.
    template <typename MutateFn, typename AcceptFn>
    bool try_apply_scoped(MutateFn&& mutate, AcceptFn&& accept, NetId touched_net,
                          const Rect& touched_area, VerifyResult* vr_out = nullptr) {
        Board backup = *board_;
        mutate();
        BoardVerifier scoped;
        if (scoped.has_local_violation(*board_, *resolver_, *ctx_, touched_net,
                                       touched_area)) {
            *board_ = backup;
            return false;
        }
        VerifyResult vr = verify();
        bool kept = vr.ok && static_cast<bool>(accept());
        if (!kept) *board_ = backup;
        if (vr_out) *vr_out = vr;
        return kept;
    }

  private:
    Board* board_;
    const RuleResolver* resolver_;
    const ElectricalContext* ctx_;
};

}  // namespace copperline
