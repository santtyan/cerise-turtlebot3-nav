---
name: critical-presentation-review
description: Apply an extremely critical, no-blind-spots review to an academic presentation/slide deck for this project (CERISE, LARS, LAFusion) — every claim justified, architecture and benchmarks shown, hard questions pre-empted, Portuguese prose audited for logic and AI-tells, plus research-backed visual design (assertion-evidence titles, zero overfull content, appendix/backup slides). Use whenever building or revising a slide deck, especially before presenting to an advisor or committee. Triggers on requests like "faça um slide", "revise a apresentação", "seja crítico com o slide", "prepare a apresentação para o CERISE".
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
   a comparison point — no bare numbers. Concrete patterns found
   repeatedly in this project's history that a first pass misses because
   each sentence is individually correct:
   - **Two clauses glued by a bare comma+"e"** instead of split into
     separate sentences (e.g. "...torna o modelo quase linear, e o ganho
     de um UKF seria marginal" reads more naturally as two sentences).
   - **A trailing gerund clause dangling off an already-long sentence**
     ("...associa cada detecção ao vizinho mais próximo" tacked onto a
     sentence that already named the mechanism) — promote it to its own
     sentence.
   - **The same verb repeated across adjacent clauses** ("já converte...
     e publica..." followed immediately by "O EKF publica...") — reads
     as mechanical repetition even though each use is correct; vary the
     verb or restructure.
   - **A qualifier that repeats scope already established by the
     paragraph** ("...seria marginal nesse caso" when the whole
     paragraph is already about that one case) — cut it, it adds nothing.
2. **Conference-reviewer / anti-AI-pattern pass**: strip generative
   filler — chains of "além disso"/"portanto", the antithesis pattern
   ("não X, mas Y" / "não X — Y"), em-dash used as the default
   connector in place of varied punctuation (comma, period,
   semicolon — count occurrences; more than roughly one per 10 lines,
   or any sentence with two or more, is the density that reads as
   AI-generated), empty adjectives ("robusto", "eficiente") with no
   evidence behind them, generic bullet lists with no concrete numbers,
   inflated conclusions not backed by the data shown. Every claim must
   survive being questioned on the spot. Not every "mas" is antithesis —
   a plain causal contrast ("a abordagem funciona para N=3, mas se dois
   robôs ficam próximos, a associação pode trocar identidades") is
   ordinary Portuguese connective tissue, not the AI-tell pattern; don't
   over-correct by stripping every contrastive conjunction on sight.
3. **Calibrate explanation level to the stated audience explicitly**,
   and treat "leigo"/lay audience as ambiguous until the user narrows it.
   This project's user asked for "público leigo" and, after receiving a
   fully-lay rewrite (defining EKF, "visão computacional" from scratch,
   analogies for basic concepts), clarified: "leigo que não conhece o
   projeto, mas ainda são engenheiros" — jargon (EKF, Mahalanobis,
   Joseph form, UKF) stays, but *this project's specific design
   decisions and their reasoning* need spelling out, since that's what a
   newcomer engineer lacks. The fix was recalibrating, not re-simplifying
   further: same technical vocabulary, decisions explained instead of
   assumed. When a user says "leigo" for a technical deck, ask (or infer
   from context, e.g. an internal engineering audience vs. a general
   public talk) whether they mean *lay in this domain generally* or *lay
   in this specific project* — the right rewrite differs substantially.

## The "is this gold-standard?" prompt pattern (added 2026-08-28)

This project's user developed an effective iterative pattern for tightening
slide prose that's worth recognizing and replaying without waiting to be
asked again. It repeats per paragraph/claim, in this order:

1. **"Essa frase está confusa/pouco explicativa" / "não está fluido"** —
   flag one specific sentence, not the whole slide. Fix cohesion/fluency
   for that sentence alone (see the two-pass review above), without
   expanding scope to neighboring sentences unless they're actually part
   of the same problem.
2. **"Isso é padrão-ouro/estado da arte?"** — before adding more
   explanation, verify whether the technique named in that sentence
   (Mahalanobis gating, Joseph form, EKF vs. UKF, Monte Carlo NEES/NIS
   validation) is actually the field's established best practice, and
   *why* — not just restate the code's own docstring. Check the
   project's source (`grep` the comment/reference in the actual `.py`
   file) before asserting a citation backs it; this project's code
   docstrings already cite Bar-Shalom, Sinopoli, Chen et al. 2023, etc.,
   so confirming is usually fast, but never skip the check.
3. **"Como explicar isso no trecho?"** — only after 1–2 are settled,
   fold the verified justification into the sentence, in prose, not as a
   citation dump. This is where a claim moves from "we do X" to "we do X
   because the field agrees this is correct when Y holds, and our system
   has Y."
4. **"Para um leigo isso não está bom" → clarify calibration (see item 3
   above), then rewrite once, not repeatedly guessing.** If a mechanism
   explanation ends up attributing the right effect to the wrong cause
   (e.g. "câmera fixa" as the reason a projection is linear, when the
   real cause is the camera looking straight down at a flat scene from
   above — fixed-but-angled would not have the same property), catch and
   correct the causal claim itself, not just the phrasing. A confident,
   fluent sentence with the wrong mechanism is worse than an awkward one
   with the right mechanism.
5. **"Muito grande" → cut, don't re-explain.** Once content and
   causality are both verified correct, length cuts should preserve the
   verified mechanism and drop secondary clauses/examples, not
   re-derive the explanation from scratch. Expect 2-4 rounds of "menor"
   before a paragraph reaches presentation length — this is normal for
   the leigo-mas-engenheiro register (item 3 of the two-pass section),
   which needs the mechanism spelled out but not padded.

Applying this pattern proactively (checking gold-standard status and
causal correctness before the user asks) is higher-value than waiting for
each numbered prompt — but when a user's request matches step 1 or 4
above, that's the signal to also silently run step 2 (verify against the
literature/code) before answering, even if they didn't explicitly ask for
it this time.

## Visual design and cognitive load (research-backed, added 2026-08-28)

The checklist above covers argumentative rigor; these items cover *how the
slide communicates*, based on a literature review (Alley's assertion-evidence
model, Mayer's multimedia learning principles, Beamer defense conventions).
Sources cited inline so claims stay checkable.

11. **Zero overfull boxes before calling a Beamer deck done — this is not
    cosmetic.** `pdflatex`'s "Overfull \vbox" warning means content is
    physically rendering past the frame boundary; at least once in this
    project's history (2026-08-28 session) a 103pt overfull was silently
    cutting the "why EKF and not UKF" justification and half a module list
    off the bottom of the slide, invisible in the `.tex` source. Compile,
    `grep "Overfull \\\\vbox" *.log`, and for anything above ~15pt render
    that page to an image and look at it — don't trust the point count
    alone. Fix order (cheapest first): move content to an appendix slide →
    cut redundant sentences → shrink `\vspace`/table row height → only as
    last resort shrink font size (already-small text going smaller is often
    the wrong fix).
12. **Split a slide instead of shrinking it once two dense elements compete
    for the same frame** (e.g., a TikZ diagram *and* a paragraph explaining
    it; two figures *and* two explanatory paragraphs). A rule of thumb: if
    fixing an overfull requires touching more than one `\vspace` or cutting
    a full sentence, the slide has too much content for one frame, not too
    little space.
13. **Titles on result slides should assert the finding, not name the
    topic**, per Alley's assertion-evidence model (Alley, Schreiber,
    Ramsdell & Muffo, *Technical Communication* 53(2), 2006; Garner et al.,
    *Int. J. Engineering Education* 29(6), 2013 — controlled study, same
    spoken content, assertion-evidence audiences understood and recalled
    more, p<.01, effect larger for complex concepts). Concretely: "LARS —
    Resultado: achado negativo" → "O guloso nearest-free vence o PPO em
    ~12% no tempo de resposta". Apply this **only to result/finding slides**
    (typically 4–6 in a two-project deck) — forcing it onto
    method/architecture slides, which don't have one single claim, reads as
    artificial. Low cost, high perceived-quality gain; do this before
    anything else if time is short.
14. **A defense/progress deck needs a `\appendix` with backup slides**,
    placed after a closing "Obrigado/Perguntas" frame. Anything that is
    (a) a legitimate answer to a hard question but not needed to follow the
    main argument, or (b) the overflow content from item 11/12, belongs
    here — hyperparameter sweeps, benchmark-not-used justifications,
    rejected-alternative comparisons (e.g. fading factor vs. covariance
    clip). In Beamer, frame numbering should exclude the appendix from the
    displayed total (defense convention: the audience sees "24/24" at the
    close, backup slides then continue "25/24", "26/24", signaling clearly
    they're extra) — `\setbeamertemplate{footline}` with a fixed
    `\LASTFRAME` macro set to the last main-deck frame number is simpler
    and more robust than trying to capture `\value{framenumber}` at the
    `\appendix` boundary (that value does not reliably persist across the
    `\appendix` command in practice — verified failing in this project
    2026-08-28, do not re-attempt the counter-capture approach without new
    evidence it works).
15. **The redundancy principle only penalizes text the presenter reads
    aloud verbatim while it's also on screen** (Mayer's multimedia learning
    principles — narration + identical on-screen text splits attention and
    measurably hurts retention). It does **not** penalize dense reference
    text the presenter does not narrate — a deck that doubles as a
    leave-behind document for an advisor is a legitimate use case, not a
    violation. Don't strip dense slides on sight; check whether the density
    is presenter-narrated redundancy (cut it) or reference material for
    later reading (fine as-is, or move to appendix per item 14 if it's
    fighting for space with an on-screen diagram).
16. **Don't switch Beamer themes (e.g. `default` → `metropolis`) under time
    pressure.** `metropolis` needs XeLaTeX/LuaLaTeX plus the Fira font
    installed to render as intended; falling back to pdfLaTeX degrades it
    to Computer Modern Sans and loses most of the visual gain, and a theme
    switch changes every box's metrics — which reintroduces overfull
    problems just fixed under item 11, and breaks any hand-tuned TikZ
    `scale=` values. This is a real example of chasing cosmetic polish at
    the cost of the actual deadline; see "what's not worth doing" framing
    in `senior-repo-refactor`.

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
