# Section 13.4: two-phase analysis reproduced across 10 logs

Extends Section 13.3's two-phase framework from the single pilot log
(0067) to the same 10-log set (5 collision + 5 non-collision) used in
`docs/multi_log_results.md` / Section 12.24. Every detection/labeling
function is reused unchanged from `event_metric_annotations.py` and
`event_augmented_predicate_abstraction.py`; only the drive-over-10-logs
loop (`logverify/multi_log_event_augmented_analysis.py`) is new.

## Results

| log | collision | events | at boundary | buried | Phase 1 boxes | Phase 2 naive | Phase 2 recommended | ratio | near-region events | pure |
|---|---|---|---|---|---|---|---|---|---|---|
| #0002 | yes | 10 | 0 | 10 | 12 | 22 | 20 | 1.67 | 8 | 8 |
| #0036 | yes | 10 | 0 | 10 | 11 | 23 | 19 | 1.73 | 7 | 7 |
| #0067 | yes | 12 | 0 | 12 | 13 | 27 | 24 | 1.85 | 10 | 10 |
| #0071 | yes | 14 | 0 | 14 | 12 | 29 | 23 | 1.92 | 9 | 9 |
| #0093 | yes | 10 | 1 | 9 | 9 | 21 | 16 | 1.78 | 6 | 6 |
| #0001 | no | 10 | 0 | 10 | 14 | 31 | 28 | 2.00 | 8 | 8 |
| #0030 | no | 9 | 0 | 9 | 12 | 23 | 19 | 1.58 | 6 | 6 |
| #0044 | no | 14 | 0 | 14 | 14 | 29 | 21 | 1.50 | 7 | 7 |
| #0065 | no | 9 | 0 | 9 | 16 | 27 | 22 | 1.38 | 5 | 5 |
| #0090 | no | 11 | 0 | 11 | 21 | 35 | 29 | 1.38 | 6 | 6 |

**Phase 1 (baseline C&C predicate abstraction, unchanged):** across all
10 logs, only **1 of 109** independently-derived events (real brake
onset, real deceleration changes, TTC/distance/speed milestones) lands on
a box boundary; the other 108 are buried inside a box. The single
exception is on log #0093. This generalizes the single-log-0067 finding
from Section 13.1 (0/12 there) essentially unchanged: the minimal,
safety-model-only abstraction is consistently blind to this auxiliary
event timeline, not just on the pilot log.

**Phase 2 (event-augmented, FAR-frozen recommended variant):** box count
grows by a mean factor of **x1.68** (range x1.38-x2.00) over the Phase 1
baseline, and **72 of 72** near-region events are pure by construction on
every log (the only impure cases anywhere, as in Section 13.2, are
events that fall inside the FAR region, which is intentionally collapsed
to a single box). Purity of each log's own C&C onset frame is unaffected
(still pure on all 10 logs).

No systematic difference between collision and non-collision logs is
visible in either the box-count ratio or the events-buried count -- both
phases behave consistently across the two groups.

## Interpretation

This is a genuine multi-log confirmation, not just a property of the
pilot log: Phase 1's "everything is buried" result and Phase 2's
"~1.4x-2x box-count cost, full purity of near-region events" result both
hold consistently across 5 collision and 5 non-collision logs of the same
scenario family. The box-count growth factor (x1.68 average) is the
concrete, now-measured price of Section 13.2's tradeoff (Section
12.25-style safety-model-only minimality vs. resolving auxiliary
criticality-metric/behavioral events at box granularity) -- consistent
enough across logs that it is not an artifact of any one log's geometry.

## Limitations / future work

- Z3 membership-check cost at the Phase 2 (24-35 box) scale has not been
  measured here -- `docs/multi_log_results.md` section 6 found cost
  tracks box count for the *existing* predicate abstraction (9-21 boxes,
  0% timeout after the near_ry fix); whether the larger Phase 2
  abstraction (up to 35 boxes here) stays within the same budget is an
  open question for a follow-up measurement.
- This 10-log set is the same "cut-in" scenario family used throughout
  Sections 12.19-13.3; the full 94-log AJISAI cut-in set (or other
  scenario types -- cutout, deceleration, swerve, u-turn) has not been
  tried.
- The event-detection thresholds themselves (TTC 1.5s/3.0s, distance
  20/10/5m, brake-onset -0.15 m/s^2, jerk 3.0 m/s^3) were calibrated on
  log 0067 in Section 13.1 and applied unchanged to all 10 logs here;
  they were not re-tuned per log, which is itself worth noting as a
  choice (uniform thresholds across a scenario family, not per-log
  optimization).

## Implementation

- `logverify/multi_log_event_augmented_analysis.py` -- the 10-log driver;
  `out_gif/multi_log_event_augmented_analysis/results.csv` (raw per-log
  data) and `summary.png` (box-count and event-buried bar charts).
