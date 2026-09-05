# Section 13.2: folding Section 13.1's events into the predicate label itself

## Motivation

Section 13.1 found that 12 independently-derived events (real Ego brake
onset, real deceleration changes, TTC/distance/speed criticality
milestones) all land *inside* an existing box of the C&C/RSS predicate
abstraction rather than at a box boundary. The user's follow-up question:
if an important transition is buried, shouldn't the abstraction split a
box there -- i.e., abstract using the C&C model *and* this event/metric
data mapping together?

This is structurally correct: this project's predicate-abstraction purity
(Section 12.25) holds *by construction* because the label is defined to
change exactly at the frame in question. The same construction works for
any frame we choose to fold into the label -- including Section 13.1's
events -- exactly as Section 12.28 already did for RSS's onset alongside
C&C's.

**This is not quite the same kind of move as Section 12.28**, though.
C&C and RSS are both independently, formally established safety
standards; combining them combines two principled criteria. TTC's
1.5s/3.0s thresholds, the 20m/10m/5m distance milestones, and the
-0.15 m/s^2 brake-onset threshold are numbers this project chose --
exactly the kind of externally-chosen metric-grid parameter that Sections
12.9-12.25 deliberately moved away from. Folding them into the label is a
legitimate choice for a research question that specifically cares about
those thresholds, but it re-admits arbitrary numeric choices that the
predicate-abstraction redesign had eliminated, and this section exists to
make that cost visible and quantified, not to hide it.

## Design

`logverify/event_augmented_predicate_abstraction.py` turns each Section
13.1 signal into a monotonically non-decreasing "phase" index over the
log, so that (a) purity holds by construction at each of that signal's
transitions and (b) a raw signal that could otherwise oscillate near a
threshold (TTC especially) does not multiply boxes:

- `real_behavior_phase`: 0, +1 at the real brake-onset frame, +1 again at
  each real deceleration-change frame.
- `ttc_ratchet_zone`: "safe" -> "caution" -> "danger", using the same
  persistence-filtered first-arrival frames as Section 13.1 (never
  reverts).
- `distance_ratchet_zone`: 0 -> 1 -> 2 -> 3 at the 20m/10m/5m closing-gap
  milestones.
- `speed_ratchet_zone`: 0 -> 1 -> 2 at the half-cruising-speed and
  near-stop milestones (tracked from the cut-in onset frame onward).

The augmented label is `(cc_label, real_behavior_phase, ttc_ratchet_zone,
distance_ratchet_zone, speed_ratchet_zone)`.

**The FAR-region caveat.** A naive version of this (`freeze_far=False`)
applies the full tuple everywhere, including frames where `cc_label` is
already `("FAR",)` -- the region the C&C model, by the whole premise of
Section 12.25, does not attend to and collapses to a single box. But the
auxiliary phases are computed independently of `cc_label` and keep
changing even while far away (an irrelevant pre-scenario deceleration
blip at frame 53, an irrelevant post-collision deceleration change at
frame 2509), so the naive version **splits the FAR region into multiple
boxes** -- a real regression of the property Section 12.25 was built to
guarantee. Measured on log 0067: 4 of the naive version's 27 boxes were
such FAR-region splits.

The recommended version (`freeze_far=True`, the default) collapses the
label to `(("FAR",),)` whenever `cc_label` is `("FAR",)`, regardless of
the auxiliary phases' values, restoring "far is exactly one box" while
still fully resolving every event inside the near region.

## Results (pilot log 0067)

| Variant | True box count | vs. baseline |
|---|---|---|
| Baseline: C&C predicate abstraction alone (12.25-12.28, unchanged) | 13 | - |
| Naive augmentation (all phases everywhere) | 27 | x2.1 (incl. 4 FAR-region splits) |
| **Recommended augmentation (FAR frozen)** | **24** | **x1.8** |

Purity of the recommended (FAR-frozen) version at each Section 13.1 event:
**10 of 12 are PURE by construction** (every event inside the near
region: both TTC milestones, all 3 distance milestones, the real brake
onset, both speed milestones, and 2 of the 4 real-deceleration-change
frames). The 2 IMPURE cases are exactly the 2 deceleration-change events
that occur *inside* the FAR region (frames 53 and 2509) -- correctly
folded into the single FAR box by design, not a defect. C&C's own onset
(frame 588) remains PURE, unchanged from the baseline.

## Interpretation

Whether to adopt this augmented label instead of (or alongside) the
plain C&C/RSS predicate abstraction is a research-question-dependent
tradeoff, not a strict improvement:

- **For**: every event a researcher wants to locate in the box sequence
  now corresponds to an actual box boundary, at the cost of specific,
  documented, and now-measured extra granularity (13 -> 24 boxes on this
  log, +85%) rather than an unbounded metric grid.
- **Against**: the extra granularity is driven by four externally-chosen
  numeric thresholds (TTC zones, distance milestones, brake-onset
  threshold, and implicitly the jerk threshold for deceleration-change
  detection) that the predicate-abstraction redesign (Section 12.25) was
  specifically motivated to avoid. Different threshold choices would give
  different box counts and different box boundaries -- the same kind of
  threshold-sensitivity concern raised (and set aside) for metric grids.

This was tested on a single log only. A multi-log measurement (matching
the 10-log reproduction in `docs/multi_log_results.md`) would show
whether the ~1.8x box-count cost is representative or log-specific, and
whether it stays cheap enough for Z3 membership checking as box count
grows.

## Implementation

- `logverify/event_augmented_predicate_abstraction.py` -- phase functions
  (`real_behavior_phase_fn`, `ttc_ratchet_zone_fn`,
  `distance_ratchet_zone_fn`, `speed_ratchet_zone_fn`),
  `event_augmented_label_fn` (with the `freeze_far` option), and the
  baseline/naive/recommended comparison in `run()`.
- Figure: `out_gif/event_augmented_comparison.png` (box-boundary strips,
  baseline vs. recommended augmentation, near the risk-perception-to-
  collision window).
