# Requirements — {PROJECT_NAME}
_Created: {DATE}_
_Status: DRAFT — becomes LOCKED after $requirement Phase 5_
_Knowledge source: {knowledge.md paper reference or "no paper — manual interview"}_

<!--
  USAGE:
  - Written by $requirement. Read by ALL downstream skills.
  - Field source tags:
      [from paper]      — taken directly from paper's Experiments section
      [proposed]        — reasonable inference from paper context
      [user confirmed]  — confirmed or overridden by user in Phase 4 interview
      [NEEDS USER INPUT]— must be answered before this file can be locked
  - LOCKED after $requirement Phase 5 completes.
  - Changing requirements after lock requires re-running $requirement.
-->

## Framing
- Objective: {A / B / C from Phase 4 framing question}
  - A = replicate paper's exact setup
  - B = use paper as reference, apply to own data/domain
  - C = beat paper's result on same benchmark
- Paper reference: {title} ({year}) [from paper]

## Problem Definition
- Task type: {e.g. binary classification} [from paper / user confirmed]
- Input: {modality and format} [from paper]
- Output: {prediction format} [from paper]

## Data
- Dataset: {name or "proprietary"} [from paper / user confirmed]
- Source: {URL or description} [from paper / user confirmed]
- Size: {approximate} [from paper / NEEDS USER INPUT]
- Split: {train/val/test — ratio or fixed files} [from paper / proposed]
- Known issues: {class imbalance, missing values, etc.} [user confirmed]
- Data version: {hash, tag, or date} [user confirmed]
- Data is frozen for this cycle: {yes / no} [user confirmed]

## Success Criteria
<!-- Baseline = paper's best reported result (pre-filled from knowledge.md) -->
| Metric | Baseline | Threshold (deploy gate) | Stretch Target |
|--------|----------|------------------------|----------------|
| {primary metric} | {paper value} [from paper] | {value} [user confirmed] | {value} [user confirmed] |
| {secondary metric} | | | |

- Primary ranking metric (leaderboard sort): {metric name} [from paper]
- Higher is better: {yes / no} [from paper]

## Constraints
| Constraint | Value | Source | Hard / Soft |
|-----------|-------|--------|------------|
| Max GPU-hours / experiment | {value} | [user confirmed] | hard |
| Total GPU-hours budget | {value} | [user confirmed] | hard |
| Inference latency p95 | {ms or "not constrained"} | [user confirmed] | {hard/soft} |
| Peak inference memory | {GB or "not constrained"} | [user confirmed] | {hard/soft} |
| Seeds per configuration (N) | 3 | [proposed] | hard |

## Evaluation Protocol
- Held-out test set isolated: {yes — describe guarantee} [from paper / user confirmed]
- Test set used only in: $evaluate (never in $train or $tune)
- Statistical reporting: mean ± std across N seeds
- Significance threshold: delta > 1 std of baseline = meaningful improvement
- Paper's evaluation protocol: {e.g. "5 seeds, single held-out test set"} [from paper]

## Non-Goals (this cycle)
<!-- Explicit scope boundaries prevent drift during $tune -->
- {e.g. "Multilingual support is out of scope"} [user confirmed]
- {e.g. "Model interpretability not required this cycle"} [user confirmed]

---
_Provenance: {knowledge.md paper extraction / manual interview}_
_Locked by: $requirement on {date}_
_Next step: $design_
