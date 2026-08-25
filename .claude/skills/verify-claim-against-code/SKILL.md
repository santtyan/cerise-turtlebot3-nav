---
name: verify-claim-against-code
description: Verify every technical/numerical claim in a paper draft against the actual code or a fresh execution, not against memory or a prior session's notes. Use before writing or finalizing any Methodology, Results, or Experimental Setup section. Triggers on requests like "write the results section", "add the numbers for X", "finalize the paper", or when a draft cites a hyperparameter, metric, or dataset statistic.
---

# Verify Claim Against Code

## Why this exists

This pattern was validated twice in this repo's history: once on the
LARS 2026 paper (found 10 hallucinated/stale numbers that reading the
text alone would not have caught — e.g., an abstract inconsistency
where 18%/15% was claimed but the real single-run number was 12%/11%),
and again on LAFusion 2026 (the paper's central number — continuous
error at -12.7% — had never been computed as an explicit value before
being needed for Results; only a qualitative "EKF is worse" observation
existed in a figure).

**Never write a specific number, hyperparameter value, or dataset
statistic into a paper from memory of a prior session, a plan file, or
a casual mention in conversation.** Re-derive it from the source.

## Procedure

1. **For any hyperparameter** (Q, R, thresholds, learning rates, etc.):
   read it directly from the current source file
   (`scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py`, `src/cerise_nav/...`) — do not
   trust a comment, a memory file, or a previous paper draft, since
   values get recalibrated between sessions (Q was recalibrated at
   least twice in this project's history).

2. **For any result number** (accuracy, error reduction %, p-values):
   re-run the actual evaluation script and read the fresh output. If the
   exact number needed (e.g., a specific aggregation or metric) was
   never explicitly computed before, compute it now — do not
   extrapolate or estimate it from a related number that was computed.

3. **For any dataset/experimental statistic** (frame counts, coverage
   percentages, durations): query the actual data (bag files, logs,
   dataset directory) rather than repeating a number from a previous
   session's summary, which may have been rounded, approximate, or
   about a different run.

4. **Cross-check consistency**: if the same number appears in multiple
   places (Abstract, Introduction, Results, Conclusion), confirm they
   all trace to the same computation/run — a documented past bug in
   this repo was an Abstract percentage that came from a different run
   than the one the rest of the paper described.

5. **Mark anything you could not verify** explicitly (in a code comment
   or a TODO in the `.tex`) rather than presenting an unverified number
   as fact — this project's convention is to close these TODOs before
   submission, not leave them silently unresolved.

## When this applies

Any time you are about to write or edit: Methodology (hyperparameters),
Experimental Setup (dataset stats), Results (all numbers, all
tables/figures), Abstract/Conclusion (numbers repeated from Results).
