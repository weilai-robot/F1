# Knowledge Base
_Source: {PAPER_TITLE} — {AUTHORS} ({YEAR})_
_Venue: {VENUE}_
_Extracted: {DATE}_
_Status: DRAFT — becomes LOCKED after $requirement Phase 5_

<!--
  USAGE:
  - Written by $requirement (Phase 2 synthesis from paper).
  - Read by $design, $tune, $consolidate.
  - LOCKED (read-only) after $requirement completes.
  - Do NOT modify this file post-lock. Open a new $requirement session if the
    reference paper changes.
-->

## Problem Formulation
- Task: {one-sentence description from abstract}
- Input: {modality, format, example shape}
- Output: {prediction type and format}
- Core challenge addressed: {what existing methods fail to do, per paper}

## Algorithm Landscape

| Family | Representative Papers | Strength | Weakness |
|--------|----------------------|---------|---------|
| {e.g. gradient-boosted trees} | {paper1, paper2} | {strength} | {weakness} |
| {e.g. neural networks} | {paper1, paper2} | {strength} | {weakness} |

**Landscape synthesis**: {1–2 sentences on which families dominate this problem
and why — this informs $design's candidate selection.}

## Innovation Claims
What this paper does differently from prior work:
- {Claim 1 — specific and falsifiable, e.g. "proposes a sparse attention mechanism
  that reduces O(n²) complexity to O(n log n) while preserving accuracy"}
- {Claim 2}
- {Claim 3}

> Note for $design: These claims represent one valid solution direction.
> They are NOT mandatory starting points — they are reference points.

## Evaluation Protocol
<!-- This is the most important section for pre-filling requirements.md -->

- **Dataset**: {name, description, size, source/URL if public}
- **Train / Val / Test split**: {ratio or absolute sizes}
- **Primary metric**: {name} — {formula or citation if non-standard}
- **Secondary metrics**: {list}
- **Baselines reported in paper**:

| Baseline | {primary metric} | Notes |
|---------|-----------------|-------|
| {Method A} | {value} | {e.g. "reproduced by authors"} |
| {Method B} | {value} | |
| **This paper** | **{value}** | **best result** |

- **Evaluation protocol details**: {e.g. "averaged over 5 random seeds",
  "single train/test split", "10-fold cross-validation"}
- **Statistical reporting**: {e.g. "mean ± std", "median", "best of N runs"}

## Known Limitations
| Limitation | Source Section | Implication for Our Work |
|-----------|---------------|--------------------------|
| {e.g. "only evaluated on English text"} | Limitations §6 | {e.g. "if our data is multilingual, this approach may not transfer"} |
| {limitation 2} | | |

## Reproduction Constraints
| Constraint | Value | Impact |
|-----------|-------|--------|
| Compute reported | {e.g. "8×A100, 72 hours"} | {high/medium/low} |
| Data availability | {public / proprietary / partial} | |
| Code released | {yes — {URL} / no / partial} | |
| Special hardware | {e.g. "TPU v4 required" or "none"} | |
| Key hyperparameters reported | {yes / partial / no} | |

## Dead End Seeds
<!-- $consolidate will transfer these to memory.md on first run -->
Directions the paper explicitly found not to work:
| Direction | Failure Mode | Paper Location |
|-----------|-------------|---------------|
| {e.g. "naive concatenation of features"} | {why it failed} | {Ablation §5.3} |
