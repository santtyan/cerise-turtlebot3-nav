---
name: verify-citation
description: Verify a bibliographic citation against a primary source (CrossRef, arXiv, or full PDF) before adding it to a LaTeX paper's \bibitem/\cite. Use whenever adding a new reference, or when reviewing existing citations before submission. Triggers on requests like "add a citation for X", "cite a paper about Y", "verify this reference", or before any paper submission/camera-ready.
---

# Verify Citation

## Why this exists

During the LAFusion 2026 paper (this repo), a fabricated citation
("Vigni et al., 31.48% desynchronization") made it into a draft before
being caught — the author name and number did not exist anywhere. This
was found by explicitly searching for the claimed paper and failing to
find it. The same session also found that a citation attributed a
mechanism to a paper (Altendorfer & Wirkert) whose abstract confirmed
the general topic but not the exact claimed mechanism word-for-word.

**Never add a `\bibitem` or a specific claimed number/detail from a
paper based on memory, plausible-sounding recall, or a name that "feels
right".** Every citation must be verified against a primary source
before it enters the manuscript.

## Procedure

1. **Identify what needs verifying**: the paper's existence (title,
   authors, venue, year) AND, separately, any specific claim you intend
   to attribute to it (a number, a mechanism, a configuration detail).
   These are two different verification steps — a real paper can still
   be misquoted.

2. **Verify existence** via one of, in order of preference:
   - CrossRef API (`https://api.crossref.org/works?query.bibliographic=...`) —
     fast, gives exact DOI/authors/venue/pages for indexed journal papers.
   - arXiv abstract page (`arxiv.org/abs/<id>`) — for preprints.
   - Direct search if the ID/DOI is unknown — do not guess a DOI or
     arXiv ID from pattern-matching; find it.

3. **Verify specific claims** (numbers, configurations, mechanisms) by
   reading the **full text**, not just the abstract. Abstracts often
   omit the specific detail you want to cite (e.g., exact team
   composition, an exact metric value). If you cannot access the full
   text, mark the claim as unconfirmed in a comment and use only what
   the abstract actually supports.

4. **If verification fails** (paper not found, claim not supported):
   do not force the citation. Either find a real alternative reference
   for the same point, or rephrase the sentence to remove the
   unsupported specific claim while keeping the argument.

5. **Record uncertainty honestly in the text**, not just in your own
   notes — e.g., "a coincidence we note but do not lean on" when a
   numeric match between two unrelated papers isn't backed by a shared
   method (see `ahmed2026maritime` in `docs/lafusion_paper/main.tex` for
   a real example of this pattern).

## Before final submission

Re-run this check on every `\bibitem` in the paper, not just newly
added ones — citations added early in a long writing session are as
likely to have been under-verified as ones added under deadline
pressure at the end.
