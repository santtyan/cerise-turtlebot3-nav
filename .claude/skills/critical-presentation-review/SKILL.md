---
name: critical-presentation-review
description: Apply an extremely critical, no-blind-spots review to an academic presentation/slide deck for this project (CERISE, LARS, LAFusion) — every claim justified, architecture and benchmarks shown, hard questions pre-empted, Portuguese prose audited for logic and AI-tells. Use whenever building or revising a slide deck, especially before presenting to an advisor or committee. Triggers on requests like "faça um slide", "revise a apresentação", "seja crítico com o slide", "prepare a apresentação para o CERISE".
---

# Critical Presentation Review

## Why this exists

Built on 2026-08-16 when the user asked for a CERISE progress slide
covering the full LARS and LAFusion journeys and then escalated the
request mid-task: every claim must be justified, decisions must not
look arbitrary, code architecture must be visible, benchmarks must be
explicit, and the deck must survive hard questions from an advisor or
committee without the presenter getting caught flat-footed. This
mirrors the same rigor already enforced on the papers themselves
(`verify-citation`, `verify-claim-against-code`) but applied to the
narrative and defensibility of a presentation, not just its numbers.

## Checklist (apply while drafting, not only after)

1. **Every "what" carries a "why" and a "how".** A design decision
   stated without justification (why EKF and not UKF/PF; why this
   reward term and not another; why this hyperparameter value) reads as
   arbitrary to a reviewer even when it isn't. If a slide states a
   choice, the choice needs a visible reason next to it — a full
   sentence, not a bullet fragment implying it should be obvious.
2. **Code architecture is a slide, not a footnote.** For each project
   covered, name the real modules/files behind the pipeline (not only
   the conceptual diagram) — e.g. `ekf_fusion_node.py`,
   `association.py`, `allocation_env.py`. A reviewer who asks "onde
   isso está implementado?" needs an answer already on screen.
3. **Testing/benchmarks get their own explicit slide.** Statistical
   tests used (which, why, what they ruled out), validation stages
   (synthetic before real data), and — critically — an honest note on
   any public benchmark considered and why it wasn't used, if that gap
   exists. Silence on this invites the question "comparou com quê?".
4. **A references-and-next-steps slide is mandatory**, listing only the
   citations that actually back a claim made earlier in the deck (not
   the full bibliography), plus concrete next steps per project — not
   generic ones like "melhorar o modelo".
5. **Open with a roadmap slide** right after the title, stating what
   the presentation will cover and in what order, so the audience can
   track where they are.
6. **Pre-empt hard questions before calling the deck done.** For every
   central result, ask: what would a skeptical advisor ask right after
   this slide? Common categories in this project's history:
   - *Scope/generalization*: does this hold for more robots/scenarios
     than tested?
   - *Alternatives not tried*: why this algorithm/method and not the
     obvious competitor (e.g., another RL algorithm, another filter)?
   - *Negative results*: does this finding undercut the whole
     contribution, or is it a scoped, theoretically-grounded limitation?
   - *Arbitrary-looking parameters*: any hardcoded value, threshold, or
     magic number needs its provenance stated (calibrated how, against
     what).
   If a question doesn't have a sourced answer, don't invent one —
   flag it to the user as an open gap before considering the deck done
   (same non-negotiable rule as `verify-citation`: never force an
   unverified claim to close a gap).

## The four Aristotelian causes as a justification frame

For each design decision worth explaining, check it against all four
before writing the justification sentence — not as a labeled rubric on
the slide (that would read as artificial), but as an internal
completeness check:

- **Material cause** — what raw input/structure the decision consumes
  (e.g., Mahalanobis gating consumes the filter's own covariance).
- **Formal cause** — what structure/shape defines the method (e.g., the
  EKF's state `[px,py,θ]` and the linearized predict/correct form).
- **Efficient cause** — what produced or motivated the decision (e.g.,
  a root-cause bug in the reward motivated switching from
  `travel_time` to `response_time`; discovering Zhang's degenerate case
  motivated fixing the camera calibration).
- **Final cause** — what purpose it serves (e.g., the covariance
  ceiling exists to keep Mahalanobis-gated association stable, not to
  make the state estimate itself more accurate — a real distinction
  already present in the LAFusion paper's Discussion).

A justification that only answers "what it's made of" or only "what
purpose it serves" without the others tends to read as incomplete or
arbitrary to a critical reviewer — this is why the check runs all four,
even though the resulting sentence should read naturally in Portuguese,
not as a four-part list.

## Two-pass review (run on the finished draft, before compiling)

1. **Senior logician/grammarian pass (Portuguese)**: read every
   sentence for cohesion, coherence, fluency. Cut sentences that don't
   support the conclusion they announce. Every number needs a unit and
   a comparison point — no bare numbers.
2. **Conference-reviewer / anti-AI-pattern pass**: strip generative
   filler — chains of "além disso"/"portanto", empty adjectives
   ("robusto", "eficiente") with no evidence behind them, generic
   bullet lists with no concrete numbers, inflated conclusions not
   backed by the data shown. Every claim must survive being questioned
   on the spot.

## How to apply

Run both the checklist and the two-pass review against the actual
draft content (`.tex`, `.md`, whatever the deck's source is) — this is
prose/content review, not a one-time consultation. Re-run after any
substantial edit, not just once at the end.
