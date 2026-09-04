# Safety-Model-Guided Abstraction and Refinement

This document describes the method implemented in this repository: using a
formal driver/safety model (rather than an arbitrary metric grid) to drive
the abstraction granularity of a Car Position Diagram (CPD) built from a
real Autoware driving log.

## 1. Idea

A CPD verification pipeline partitions a continuous trajectory (the
relative position of the ego vehicle and an NPC over time) into a finite
sequence of discrete "boxes." The central question is: **what should decide
where the box boundaries go?**

The naive approach is a metric grid — pick some cell width and tile space
with it. This repository instead drives abstraction from the *safety
model under verification itself*:

1. Wherever the safety model's own state actually distinguishes situations
   (e.g., "before vs. after the model's risk/violation onset," "in contact
   vs. not"), the abstraction should distinguish them too.
2. Wherever the safety model does not care about the exact position (the
   "far" region, well outside the range the model reasons about), all such
   frames should collapse into a **single** box, regardless of how far
   apart they are metrically.

This is *predicate abstraction*: box identity is a label computed from the
safety model's own predicates, not from a position bucketed by a
distance-based cell size. It is implemented in
`logverify/safety_predicate_abstraction.py`.

## 2. Two driver/safety models compared

- **JAMA C&C (Competent and Careful) driver model**
  (`logverify/jama_cc_model.py`): perception response time 0.75s,
  time-to-max-deceleration 0.6s, max deceleration 0.774G. Risk is flagged
  at the first of two thresholds to trigger: a lateral risk boundary
  (1.8 m/s x 0.4s = 0.72m) or a longitudinal TTC risk boundary (2.0s),
  filtered so a boundary must hold for `persist_frames` (default 3)
  consecutive frames before being accepted as a genuine onset (this
  persistence filter removes single-frame TTC/measurement noise).

- **RSS (Responsibility-Sensitive Safety)** (`logverify/rss_model.py`):
  longitudinal minimum safe distance

  ```
  d_min = v_r*rho + (1/2)*a_max_accel*rho^2
          + (v_r + rho*a_max_accel)^2 / (2*b_min)
          - v_f^2 / (2*b_max)
  ```

  with rho=1.0s, a_max_accel=2.0 m/s^2, b_min=4.0 m/s^2, b_max=8.0 m/s^2
  (illustrative parameter values from Shalev-Shwartz et al., 2017,
  arXiv:1708.06374). Only the longitudinal formula is implemented; the
  lateral (merge) formula is future work. Onset-finding uses the same
  persistence-filtered structure as the C&C model
  (`_first_persistent_trigger`, reused from `jama_cc_model.py`).

Both models' onset frames are genuinely different quantities, not just
different numbers: on the pilot log (AJISAI `TD-NI-AR-SD-N04-CI-0067`),
JAMA C&C's onset is frame 588 and RSS's is frame 548 (about 1 second /
40 frames apart), and the two models respond differently to speed (C&C's
thresholds are roughly speed-independent; RSS's `d_min` scales
quadratically with speed). That the counterfactual "was this collision
preventable?" simulation happens to agree on this one log is not evidence
that the two models are equivalent in general — comparing both across a
larger log set is future work.

## 3. Comparison variants

Five abstraction variants are compared on the pilot log:

| # | Variant | Definition | True box count |
|---|---|---|---|
| 1 | Vehicle-physical-size basis | Uniform grid, cell = 0.9526m (sum of ego/NPC half-lengths / 5), no near/far split | 320 |
| 2 | Uniform-grid baseline | Uniform grid, cell = 2.0m (arbitrary, not derived from vehicle size or any safety model) | 158 |
| 3 | JAMA C&C predicate abstraction | Near region partitioned by C&C's own onset/contact state; far region (\|rx\| > 40m) collapsed to a single box | **13** |
| 4 | RSS predicate abstraction | Same as (3), using RSS's onset/contact state | **13** |
| 5 | Reference: pre-correction C&C-guided metric grid | Only `near_range` tied to C&C's onset frame; `near_cell`/`far_cell` still vehicle-size-derived | 91 |

"True box count" is the number of *distinct* boxes
(`len(box_id_of)`/`len(label_of_box)`), not the number of maximal
same-box runs — `gcpd.Model` identifies a box by `(lane, position)` (or,
for predicate abstraction, by its label), and a revisit to an
already-seen box reuses its id rather than creating a new one.

## 4. Purity as an evaluation metric

Given a safety model's onset frame f, an abstraction is called **pure**
at f if the box containing frame f starts exactly at f — i.e., no
pre-onset and post-onset frames are merged into the same box. If they are
merged, the abstraction is **impure**, and the **smear** is the rx-span
(in meters) of frames incorrectly grouped together. Purity measures
whether the abstraction accidentally erases the one boundary that matters
most for verification: the moment the safety model's own verdict flips.

Purity at a metric grid's cell boundaries is a matter of luck (whether the
onset frame happens to land on a cell edge) and is non-monotonic in cell
size — shrinking cells does not monotonically improve it, and searching
for the minimal cell size preserving purity risks overfitting to one log
(see `logverify/auto_grid.search_minimal_purity_grid`, kept in this
repository as a comparison baseline). Predicate abstraction, by contrast,
is pure **by construction** at its own model's onset: the label itself is
defined to change exactly at that frame.

**Note on cross-model purity.** An earlier draft of this analysis treated
"purity for model A does not imply purity for model B" as a noteworthy
finding. On reflection this is not actually meaningful: each safety
model's predicate abstraction is only ever interpreted within that same
model's own context (a C&C-guided abstraction is read in C&C's terms, an
RSS-guided one in RSS's terms), so its impurity at a *different* model's
onset is neither a defect nor a fair comparison point. The results below
therefore report purity only at each abstraction's own relevant onset.

## 5. Results (pilot log, single log: AJISAI `TD-NI-AR-SD-N04-CI-0067`)

| # | Variant | True box count | Purity (own-model onset) |
|---|---|---|---|
| 1 | Vehicle-size uniform | 320 | impure at both onsets (smear 0.55m / 0.81m) |
| 2 | Uniform baseline (2.0m) | 158 | impure at C&C onset (1.65m); pure at RSS onset |
| 3 | JAMA C&C predicate | 13 | pure (by construction) |
| 4 | RSS predicate | 13 | pure (by construction) |
| 5 | Reference: pre-correction C&C metric grid | 91 | pure at C&C onset (not evaluated against RSS — different model's context) |

Key findings:

1. **Predicate abstraction is dramatically more compact.** Both (3) and
   (4) use 13 true boxes, versus 91-320 for any metric-grid variant
   tried. Restricting the abstraction to the region and the state
   distinctions the safety model actually reasons about — and collapsing
   everything else to one box — has a large quantitative effect, not just
   a conceptual one.
2. **Purity is structural, not incidental, for predicate abstraction.**
   (3) and (4) are pure at their own onset by construction; metric grids
   achieve purity only when cell size happens to align with the onset
   frame, which is non-monotonic and log-specific.
3. **A genuine "vehicle-size basis" is a uniform grid, not a near/far
   split.** If vehicle size alone is the justification for the cell size,
   there is no principled reason to use a different cell size far away —
   doing so (variant 1's predecessor, an earlier near/far version) needs
   an additional, separately justified design decision. Applied
   uniformly, the vehicle-size cell in fact produces *more* boxes (320)
   than the near/far version it replaced.
4. **Cross-model purity is not a practically important comparison** (see
   note above) — retracted from the original draft of this analysis after
   review.

## 6. Limitations / future work

- ~~Single-log pilot~~ **Update:** the comparison has since been reproduced
  across 10 logs (5 collision + 5 non-collision, the same selection as an
  earlier 12.24-era batch analysis). See
  [`docs/multi_log_results.md`](multi_log_results.md) for the full results.
  The headline correction: predicate abstraction's purity guarantee holds
  unconditionally (100% pure at its own onset across all 10 logs), but its
  *compactness* is conditional — it stays compact (9-15 boxes) only on logs
  where the safety model's risk-detection window is short; on logs with a
  long near-range dwell time or heavy lateral movement, it grows as large
  as (or larger than) the metric-grid variants (up to 266 boxes).
- The C&C predicate abstraction's `RISK` state currently splits into
  lane buckets (`lane_k`) via a naive grid (`grid_index_centered`), with no
  hysteresis applied — this is the one part of the construction that is not
  itself safety-model-guided, and is the leading suspect for the
  compactness blow-up observed on some logs (see
  `docs/multi_log_results.md` section 5).
- The RSS model implements only the longitudinal formula; the lateral
  (merge) formula is not yet implemented.
- Scalability (Z3 membership-check cost) has now been measured directly
  (`docs/multi_log_results.md` section 3): cost tracks box count, not
  abstraction method — predicate abstraction is only cheaper on the logs
  where it stays compact.

## 7. Implementation map

| File | Role |
|---|---|
| `logverify/jama_cc_model.py` | JAMA C&C driver/safety model and onset-finding |
| `logverify/rss_model.py` | RSS (longitudinal) safety model and onset-finding |
| `logverify/safety_predicate_abstraction.py` | Predicate abstraction (variants 3, 4) |
| `logverify/auto_grid.py` | Metric-grid parameter derivation and grid-search baselines (variants 1, 2, 5) |
| `logverify/grid_bridge.py` | Compresses a frame sequence into grid-based box states |
| `logverify/compare_safety_model_abstractions.py` | Builds/compares metric-grid variants; purity computation |
| `logverify/visualize_five_abstractions.py` | Box-sequence (scenario-style) figures for variants 1-5 |
| `logverify/plot_five_abstractions_summary.py` | Box-count bar chart across variants 1-5 |
| `logverify/plot_five_abstractions_purity.py` | Purity/smear bar chart across variants 1-5 |
| `logverify/multi_log_five_abstractions.py` | 10-log reproduction: box count, purity, Z3 membership-check cost per variant per log |
| `logverify/plot_multi_log_five_abstractions.py` | Aggregate charts/summary table for the 10-log reproduction (see `docs/multi_log_results.md`) |
| `logverify/multi_log_model.py`, `logverify/membership.py` | Underlying CPD model / Z3 membership-check infrastructure used by the above |
| `gcpd.py` | Core CPD (Car Position Diagram) model |
