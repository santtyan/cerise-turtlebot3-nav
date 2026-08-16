---
name: anonymize-paper
description: Checklist to anonymize a LaTeX paper manuscript and its compiled PDF for double-blind review, and to restore authorship for the camera-ready version. Use before any double-blind submission (e.g., CMT, EasyChair with blind review enabled), or when preparing a camera-ready after acceptance. Triggers on requests like "anonymize the paper", "prepare for double-blind submission", "restore author info for camera-ready".
disable-model-invocation: true
---

# Anonymize Paper for Double-Blind Review

This has side effects on a manuscript intended for submission — only
run this when the user explicitly asks for anonymization or
camera-ready restoration, not proactively.

## Checklist to anonymize (before double-blind submission)

1. **Author block**: replace `\author{...}`, `\authorrunning{...}`,
   `\institute{...}` with placeholders (e.g., `Anonymous Author(s)`,
   `Anonymous et al.`, `Institution withheld for double-blind review`).
   Leave a `%` comment noting this was intentional and where to restore
   from, for the camera-ready pass.

2. **Project/system names in body text**: many venues' anonymity
   policies explicitly cover project names, not just author/institution
   (confirmed on LAFusion 2026's CMT submission page: "including
   self-identifying acknowledgments, affiliations, project names,
   funding information..."). Grep the full `.tex` for any internal
   project codename (e.g., a system name unique to your lab) and
   replace with a generic description ("the multi-robot digital twin
   studied in this work").

3. **Acknowledgments / funding**: remove entirely, or leave a
   placeholder to restore later.

4. **URLs and repository links**: if a GitHub repo, personal site, or
   institutional page is linked anywhere (including in a "reproducibility
   package" description), either remove the link or point to an
   anonymized mirror per the venue's policy.

5. **Internal planning comments in the `.tex` source**: LaTeX `%`
   comments don't render in the compiled PDF, but they DO appear if the
   venue requests LaTeX source files (common for camera-ready). Remove
   session-planning comments, internal memory references, or anything
   that names people/institutions before submitting sources.

6. **Recompile and grep the rendered PDF text** (not just the `.tex`)
   for the author's name, institution, and any project codename —
   `pdftotext main.pdf - | grep -i <name>` — a leftover mention can
   survive an incomplete edit.

7. **Check PDF metadata**, which is a commonly-missed leak vector:
   `pdfinfo main.pdf | grep -iE "author|creator|title"` — LaTeX/pdfTeX
   often leaves the Author field empty by default, but verify rather
   than assume. If populated, strip it (e.g., via `exiftool` or by
   ensuring no `\pdfinfo`/`hyperref` `pdfauthor` field is set).

## Checklist to restore (camera-ready, post-acceptance only)

1. Restore real `\author`, `\authorrunning`, `\institute` from project
   records.
2. Restore acknowledgments/funding if applicable.
3. Restore any project name/links that were genericized for review.
4. Recompile and verify the PDF now correctly shows real authorship.

## Reference example

`docs/lafusion_paper/main.tex` in this repo went through this exact
process for LAFusion 2026 (CMT, double-blind) — see its commit history
for a real before/after example, including the specific project-name
substitution and internal-comment removal.
