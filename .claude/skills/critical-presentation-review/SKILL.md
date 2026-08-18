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
2. **Code architecture is a slide, not a footnote, and needs an actual
   diagram.** For each project covered, name the real modules/files
   behind the pipeline AND draw the pipeline as a block diagram (TikZ
   in Beamer, Mermaid elsewhere) with the file/module names inside or
   next to each box — not a text-only arrow chain
   (`A $\rightarrow$ B $\rightarrow$ C`), which reads as a list, not an
   architecture. A reviewer who asks "onde isso está implementado?"
   needs a picture with the file names on screen, not just prose bullets.
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
6. **A thesis/argument/contributions slide is mandatory**, placed after
   the roadmap and before diving into individual projects. It must state
   explicitly: the central thesis in one or two sentences, how each
   project's result supports that thesis (not just what each project
   did), and a numbered list of concrete contributions. Without this
   slide, a multi-project deck reads as two unrelated demos with no
   argument tying them together — a reviewer will ask "e daí, qual é o
   argumento geral?" if it's missing.
7. **Pre-empt hard questions before calling the deck done.** For every
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

8. **Verify every diagram and figure against the actual code/data, not
   just the prose.** Found repeatedly in this project's history
   (2026-08-17 session): a TikZ architecture diagram drew training and
   inference as one linear chain when the code has them as two
   separate paths (`AllocationEnv` never touches the camera); another
   diagram placed `projection.py` between the wrong two pipeline
   stages because no one traced the actual imports
   (`grep "from cerise_nav.projection"` showed it's used inside
   `yolo_detector.py`, not `association.py`). A reused paper figure
   showed 6 policies (including one, `masked`, not yet introduced to
   the audience) while the adjacent table only reported 3 — figures
   get silently out of sync with the slide's current scope when
   copy-pasted from the paper. Concretely: for every box/arrow in a
   diagram, `grep` the module it names to confirm what actually calls
   what; for every reused figure, check its data/legend against the
   text and table right next to it, not just against the paper it
   came from.
9. **Any state/number that can go stale must be re-checked before
   calling the deck done**, not trusted from when it was first
   written. Page counts, submission deadlines, and "N pages written"
   claims drift as the underlying document changes — this session's
   deck said "8 páginas escritas" after the paper had already grown to
   10 pages three commits earlier. Re-run `pdfinfo main.pdf | grep
   Pages` on every referenced document as part of the final pass, not
   only when a page count is first written.
10. **Terms of art get defined at first mention, not assumed.**
   Mahalanobis distance, ATE/RPE, Gymnasium's `render()` contract —
   this session's user repeatedly asked "o que é X?" for terms already
   used 3-4 times earlier in the deck. A one-clause parenthetical at
   the first occurrence (not a full glossary) is enough; skipping it
   forces the audience to interrupt or silently disengage.

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
   filler — chains of "além disso"/"portanto", the antithesis pattern
   ("não X, mas Y" / "não X — Y"), em-dash used as the default
   connector in place of varied punctuation (comma, period,
   semicolon — count occurrences; more than roughly one per 10 lines,
   or any sentence with two or more, is the density that reads as
   AI-generated), empty adjectives ("robusto", "eficiente") with no
   evidence behind them, generic bullet lists with no concrete numbers,
   inflated conclusions not backed by the data shown. Every claim must
   survive being questioned on the spot.

## Page budget

A deck covering an already-submitted project alongside an in-progress
one should not give both equal page weight. Compress the finished
project (fuse motivation+architecture, results+robustness-figures,
etc. into denser multi-panel slides) and keep the in-progress project
at full detail — the audience needs the most scrutiny where the work
is still being shaped, not where it already shipped. If a deck exceeds
roughly 20–25 slides for a two-project progress update, treat that as
a signal to re-run this compression pass before considering it done,
not just to add more slides.

## How to apply

Run both the checklist and the two-pass review against the actual
draft content (`.tex`, `.md`, whatever the deck's source is) — this is
prose/content review, not a one-time consultation. Re-run after any
substantial edit, not just once at the end.
