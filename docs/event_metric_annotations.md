# Section 13.1: Event/criticality-metric annotation of the predicate abstraction (pilot log 0067)

## Motivation

The safety-model-guided predicate abstraction (Sections 12.25-12.28,
`logverify/safety_predicate_abstraction.py`) draws box boundaries purely
from each safety model's own state (JAMA C&C's risk-perceived frame /
RSS's violation-onset frame, plus contact). That design choice is
deliberate (see `docs/method.md`), but it left an open question raised in
discussion 2026-09-05: after abstraction, can we still locate, from the
BOX sequence alone, other things a researcher would want to check --

1. **real driver-behavior events** actually observed in the log (Ego's
   real brake onset, real deceleration-profile changes) -- as distinct
   from the safety model's own *simulated/counterfactual* deceleration
   profile (`jama_cc_model.cc_deceleration_at`,
   `rss_model.simulate_rss_reference`), which is a different quantity and
   must not be conflated with what Ego actually did; and
2. **criticality-metric milestones** (TTC zone crossings, closing-distance
   milestones, Ego speed milestones) computed independently of any safety
   model?

This section implements a prototype, `logverify/event_metric_annotations.py`,
that computes these events independently from the log and checks each one
against the *existing* (unmodified) C&C/RSS predicate abstraction's box
runs -- not as a new box-boundary criterion, which would reintroduce the
metric-grid problem Section 12.25 moved away from, but purely as an
after-the-fact query: does this event land on a box boundary (the
abstraction "sees" it), or is it buried inside a larger box (invisible at
the current granularity)?

## Method

Detected events (log 0067, JAMA C&C risk-perceived frame 588, RSS
violation-onset frame 548):

- **Real brake onset**: first frame Ego's real longitudinal acceleration
  (`groundtruth_ego.acceleration.linear.x`, the actually-achieved value,
  not any model's prescription) persistently drops below -0.15 m/s^2.
  *Calibration note*: Ego's actual deceleration during the critical
  window here never exceeds about -0.5 m/s^2 -- far short of JAMA C&C's
  0.774G (~7.6 m/s^2) reference response. A naive -0.5 m/s^2 threshold
  would miss Ego's real (weak) braking attempt and instead catch only an
  unrelated -0.9 m/s^2 blip at frame 53 (an initial speed-settling
  maneuver at the very start of the log, unrelated to the cut-in).
- **Real deceleration change**: frames where the jerk (d(accel)/dt) of
  Ego's real acceleration persistently exceeds 3.0 m/s^3, with nearby
  detections merged.
- **TTC zone milestones**: first frame TTC persistently (3-frame,
  matching the noise-filtering pattern already used for
  `find_risk_perceived_frame`, Section 12.24) reaches the "caution"
  (<3.0s) and "danger" (<1.5s) zones. A naive frame-to-frame zone-change
  check was tried first and produced 90+ spurious "transitions" from TTC
  oscillating near a threshold -- the same noise problem Section 12.24
  found in TTC generally -- so it was replaced with a persistence-filtered,
  once-per-zone version.
- **Distance milestones**: first frame the longitudinal gap to the
  contact boundary drops below 20m, 10m, 5m.
- **Speed milestones**: first frame (at or after the cut-in onset frame,
  not frame 0 -- see note below) Ego's speed drops below half its
  pre-cut-in cruising speed, and below 1.0 m/s (near-stop).

*Implementation note*: the speed-milestone scan must start at the cut-in
onset frame, not frame 0 -- log 0067, like most of this dataset, begins
before Ego has reached cruising speed, and scanning from frame 0 falsely
flags that initial acceleration-to-cruise phase as "half speed" / "near
stop".

## Result

All 12 detected events were checked against both the C&C and RSS
predicate abstractions' box runs (13 true boxes each, matching Section
12.28's count exactly -- this annotation pass does not change the
abstraction itself).

**None of the 12 events land on a box boundary; all 12 are buried inside
an existing box**, for both the C&C and RSS abstractions. In particular:

- TTC's first entry into "caution" (frame 559) and "danger" (frame 597)
  both fall inside the pre-existing `SAFE`/`RISK` (or `VIOLATION`) boxes,
  not at their start.
- The real brake onset (frame 716) and the 5m/10m/20m distance milestones
  all fall inside the single `RISK`/`VIOLATION` box spanning frames
  588-692 (or 548-692 for RSS) -- i.e., that one box covers the entire
  window from risk perception through the closest approach, and gives no
  finer-grained handle on which sub-interval contains the real braking
  attempt or any given distance milestone.
- A very large box (frames 932-1930, ~1000 frames / ~30s) covers the
  post-collision plateau and swallows two real deceleration-change events
  (frames 1576, 1810) and the near-stop speed milestone (frame 1462)
  entirely.

## Interpretation

This is a real, structural limitation of the current predicate
abstraction, not a bug: the abstraction is deliberately built to be
minimal with respect to what its own safety model attends to (Section
12.25's whole point), so it is not designed to also resolve independent
criticality metrics or the real driver-behavior timeline. The practical
upshot is that if a researcher wants to inspect, after abstraction,
*where* in the box sequence Ego's real brake onset or a TTC-danger
crossing occurred, the box identity alone does not answer that -- a
supplementary per-frame annotation (as implemented here) is needed
alongside the abstraction, not a replacement for it.

This was tested on a single log (0067) only; whether the same
"everything buried, nothing at a boundary" pattern holds across the wider
10-log (or full 94-log) set is future work, as is deciding whether any of
these auxiliary distinctions are worth folding into the predicate label
itself for specific research questions (at the cost of additional boxes,
the same tradeoff already documented for the combined C&C+RSS label in
Section 12.28).

## Implementation

- `logverify/event_metric_annotations.py` -- event detection functions
  (`real_brake_onset_frame`, `real_deceleration_change_frames`,
  `ttc_zone_transition_frames`, `distance_milestone_frames`,
  `speed_milestone_frames`), the box-lookup/annotation logic
  (`locate_event_in_runs`, `annotate`), and the 3-panel figure
  (`plot_annotations`: BOX-boundary strip, real acceleration with event
  markers, TTC with zone shading and event markers).
- Figure: `out_gif/event_metric_annotations.png`
