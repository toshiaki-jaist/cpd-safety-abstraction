"""Section 13.4: extend the two-phase analysis (Section 13.3) from the
single pilot log (0067) to the same 10-log set (5 collision + 5
non-collision) used in `docs/multi_log_results.md`.

For each log, this measures:

  Phase 1 (event_metric_annotations.py): against the plain C&C predicate
  abstraction, how many of the log's independently-derived events (real
  brake onset, real deceleration changes, TTC/distance/speed criticality
  milestones) land on a box boundary vs. are buried inside a box.

  Phase 2 (event_augmented_predicate_abstraction.py): the box-count cost
  of folding those same events into the label (naive vs. FAR-frozen
  recommended variant), and what fraction of near-region events become
  pure by construction.

This reuses every detection/labeling function from the two pilot modules
unchanged -- only the "loop over one hardcoded log" driver is new -- so
Section 13.1/13.2's log-0067 numbers are reproduced exactly as row one of
this table, not recomputed differently.

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.multi_log_event_augmented_analysis
"""

import csv
import os

from logverify.paths import DATA_DIR
from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame, cutin_onset_frame,
)
from logverify.reference_model_comparison import (
    ego_speed_series, actual_accel_series, compute_ttc,
)
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_near_range_from_risk_frame
from logverify.safety_predicate_abstraction import (
    cc_predicate_label_fn, rss_predicate_label_fn, compress_by_label,
    _purity_of_predicate_abstraction,
)
from logverify.event_metric_annotations import (
    real_brake_onset_frame, real_deceleration_change_frames,
    ttc_zone_transition_frames, distance_milestone_frames, speed_milestone_frames,
    locate_event_in_runs, Event,
)
from logverify.event_augmented_predicate_abstraction import (
    real_behavior_phase_fn, ttc_ratchet_zone_fn, distance_ratchet_zone_fn,
    speed_ratchet_zone_fn, event_augmented_label_fn,
)

OUT_DIR = "out_gif/multi_log_event_augmented_analysis"
RESULTS_CSV = f"{OUT_DIR}/results.csv"

# Same 10-log selection as docs/multi_log_results.md / Section 12.24.
COLLISION_LOGS = [
    "TD-NI-AR-SD-N04-CI-0002.json",
    "TD-NI-AR-SD-N04-CI-0036.json",
    "TD-NI-AR-SD-N04-CI-0067.json",
    "TD-NI-AR-SD-N04-CI-0071.json",
    "TD-NI-AR-SD-N04-CI-0093.json",
]
NON_COLLISION_LOGS = [
    "TD-NI-AR-SD-N04-CI-0001.json",
    "TD-NI-AR-SD-N04-CI-0030.json",
    "TD-NI-AR-SD-N04-CI-0044.json",
    "TD-NI-AR-SD-N04-CI-0065.json",
    "TD-NI-AR-SD-N04-CI-0090.json",
]


def analyze_log(json_path, is_collision):
    data = _load(json_path)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    accel = actual_accel_series(gk)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    closest_frame, _risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    cutin_frame = cutin_onset_frame(rys, closest_frame)

    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, rys, timestamps, ego_speed, npc_speed)
    gy, near_rx = 0.364, 40.0
    near_ry_cc = auto_near_range_from_risk_frame(rys, cc_risk_frame, margin_factor=1.2, default=10.0)
    cc_label_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx, gy, near_ry=near_ry_cc)

    # --- Baseline (Phase 1's abstraction, unchanged) ---
    baseline_runs, baseline_boxes = compress_by_label(n, valid, cc_label_fn)

    # --- Events (Section 13.1, unchanged) ---
    brake_frame = real_brake_onset_frame(accel)
    decel_change_frames = real_deceleration_change_frames(accel, timestamps)
    ttc_transitions = ttc_zone_transition_frames(ttcs)
    distance_events = distance_milestone_frames(rxs, eh_l + nh_l)
    speed_events = speed_milestone_frames(ego_speed, cutin_frame)

    events = []
    if brake_frame is not None:
        events.append(Event("real_brake_onset", brake_frame, "real_behavior"))
    for f in decel_change_frames:
        events.append(Event("real_decel_change", f, "real_behavior"))
    for f, frm, to in ttc_transitions:
        events.append(Event(f"ttc_{frm}_to_{to}", f, "criticality_metric"))
    for f, m in distance_events:
        events.append(Event(f"distance_lt_{m:.0f}m", f, "criticality_metric"))
    for f, label in speed_events:
        events.append(Event(f"speed_{label}", f, "criticality_metric"))

    # --- Phase 1: annotate baseline abstraction ---
    n_boundary = n_buried = 0
    for ev in events:
        run = locate_event_in_runs(ev.frame, baseline_runs)
        if run is None:
            continue
        if ev.frame == run.start_frame:
            n_boundary += 1
        else:
            n_buried += 1

    # --- Phase 2: event-augmented abstraction ---
    real_phase_fn = real_behavior_phase_fn(brake_frame, decel_change_frames)
    ttc_zone_fn = ttc_ratchet_zone_fn(ttc_transitions)
    dist_zone_fn = distance_ratchet_zone_fn(distance_events)
    speed_zone_fn = speed_ratchet_zone_fn(speed_events, cutin_frame)

    naive_fn = event_augmented_label_fn(cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn, speed_zone_fn, freeze_far=False)
    _, naive_boxes = compress_by_label(n, valid, naive_fn)
    rec_fn = event_augmented_label_fn(cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn, speed_zone_fn, freeze_far=True)
    rec_runs, rec_boxes = compress_by_label(n, valid, rec_fn)

    n_pure_rec = 0
    n_near_events = 0
    for ev in events:
        in_far = cc_label_fn(ev.frame) == ("FAR",)
        if in_far:
            continue
        n_near_events += 1
        p = _purity_of_predicate_abstraction(rec_runs, ev.frame)
        if p.get("applicable") and p.get("pure"):
            n_pure_rec += 1

    p_cc_own = _purity_of_predicate_abstraction(rec_runs, cc_risk_frame)

    return dict(
        log=os.path.basename(json_path), is_collision=is_collision,
        n_events=len(events), n_boundary=n_boundary, n_buried=n_buried,
        baseline_boxes=len(baseline_boxes), naive_boxes=len(naive_boxes),
        rec_boxes=len(rec_boxes),
        rec_ratio=round(len(rec_boxes) / len(baseline_boxes), 2) if baseline_boxes else None,
        n_near_events=n_near_events, n_pure_rec=n_pure_rec,
        cc_onset_pure=bool(p_cc_own.get("pure")),
        cc_risk_frame=cc_risk_frame, rss_risk_frame=rss_risk_frame,
    )


def run():
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    for name in COLLISION_LOGS:
        rows.append(analyze_log(str(DATA_DIR / name), True))
    for name in NON_COLLISION_LOGS:
        rows.append(analyze_log(str(DATA_DIR / name), False))

    fieldnames = list(rows[0].keys())
    with open(RESULTS_CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"{'log':32s} {'coll':5s} {'evt':4s} {'bnd':4s} {'buried':6s} "
          f"{'base':5s} {'naive':6s} {'rec':4s} {'ratio':6s} {'near':5s} {'pure':5s} {'cc_pure'}")
    for r in rows:
        print(f"{r['log']:32s} {str(r['is_collision']):5s} {r['n_events']:4d} "
              f"{r['n_boundary']:4d} {r['n_buried']:6d} {r['baseline_boxes']:5d} "
              f"{r['naive_boxes']:6d} {r['rec_boxes']:4d} {str(r['rec_ratio']):6s} "
              f"{r['n_near_events']:5d} {r['n_pure_rec']:5d} {r['cc_onset_pure']}")

    total_events = sum(r["n_events"] for r in rows)
    total_boundary = sum(r["n_boundary"] for r in rows)
    total_near = sum(r["n_near_events"] for r in rows)
    total_pure_rec = sum(r["n_pure_rec"] for r in rows)
    avg_ratio = sum(r["rec_ratio"] for r in rows if r["rec_ratio"]) / len(rows)
    print()
    print(f"Phase 1 (baseline abstraction): {total_boundary}/{total_events} events at a box boundary across all 10 logs")
    print(f"Phase 2 (recommended augmented abstraction): {total_pure_rec}/{total_near} "
          f"near-region events pure by construction; mean box-count ratio vs. baseline = x{avg_ratio:.2f}")
    print(f"CSV: {RESULTS_CSV}")
    return rows


if __name__ == "__main__":
    run()
