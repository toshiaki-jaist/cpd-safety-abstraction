"""Section 13.5: scenario-style (EGO/NPC snapshot) box-sequence diagrams
for the two-phase analysis (Section 13.3), on one representative log.

Reuses `scenario_snapshot_diagram.plot_scenario_snapshot_sequence` (the
same "scenario visualization" style used throughout Sections 12.14-12.24
and `visualize_five_abstractions.py`) to render:

  1. Phase 1's baseline C&C predicate abstraction (13 boxes on log 0067).
  2. Phase 2's event-augmented, FAR-frozen abstraction (24 boxes on log
     0067) -- the same abstraction, refined at every Section 13.1 event.

Log 0067 is used as the representative log: it has been this project's
consistent pilot log since Section 12.19, and is the log all of Sections
13.1-13.3's worked examples and numbers already refer to.

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.visualize_two_phase_scenario
"""

import os

from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame, cutin_onset_frame,
)
from logverify.reference_model_comparison import (
    ego_speed_series, actual_accel_series, compute_ttc,
)
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series
from logverify.auto_grid import auto_near_range_from_risk_frame
from logverify.safety_predicate_abstraction import cc_predicate_label_fn, compress_by_label
from logverify.event_metric_annotations import (
    real_brake_onset_frame, real_deceleration_change_frames,
    ttc_zone_transition_frames, distance_milestone_frames, speed_milestone_frames,
)
from logverify.event_augmented_predicate_abstraction import (
    real_behavior_phase_fn, ttc_ratchet_zone_fn, distance_ratchet_zone_fn,
    speed_ratchet_zone_fn, event_augmented_label_fn,
)
from logverify.visualize_five_abstractions import Run, make_snapshots, _format_label
from logverify.scenario_snapshot_diagram import plot_scenario_snapshot_sequence

from logverify.paths import LOG_0067 as LOG_PATH  # see logverify/paths.py
OUT_DIR = "out_gif/two_phase_scenario"


def _format_augmented_label(label) -> str:
    """(('RISK', -3), 2, 'danger', 3, 1) -> 'RISK k=-3 | ev2 ttc=danger d=3 v=1'
    (('FAR',),) -> 'FAR'
    """
    cc_part = _format_label(label[0])
    if len(label) == 1:
        return cc_part
    _, real_phase, ttc_zone, dist_zone, speed_zone = label
    return f"{cc_part} | ev{real_phase} ttc={ttc_zone} d={dist_zone} v={speed_zone}"


def run():
    os.makedirs(OUT_DIR, exist_ok=True)
    data = _load(LOG_PATH)
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

    closest_frame, _ = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    cutin_frame = cutin_onset_frame(rys, closest_frame)
    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    gy, near_rx = 0.364, 40.0
    near_ry_cc = auto_near_range_from_risk_frame(rys, cc_risk_frame, margin_factor=1.2, default=10.0)
    cc_label_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx, gy, near_ry=near_ry_cc)

    # Phase 1: baseline
    cc_runs_raw, cc_boxes = compress_by_label(n, valid, cc_label_fn)
    runs_phase1 = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in cc_runs_raw]

    # Phase 2: event-augmented (FAR-frozen)
    brake_frame = real_brake_onset_frame(accel)
    decel_change_frames = real_deceleration_change_frames(accel, timestamps)
    ttc_transitions = ttc_zone_transition_frames(ttcs)
    distance_events = distance_milestone_frames(rxs, eh_l + nh_l)
    speed_events = speed_milestone_frames(ego_speed, cutin_frame)
    real_phase_fn = real_behavior_phase_fn(brake_frame, decel_change_frames)
    ttc_zone_fn = ttc_ratchet_zone_fn(ttc_transitions)
    dist_zone_fn = distance_ratchet_zone_fn(distance_events)
    speed_zone_fn = speed_ratchet_zone_fn(speed_events, cutin_frame)
    rec_fn = event_augmented_label_fn(cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn, speed_zone_fn, freeze_far=True)
    rec_runs_raw, rec_boxes = compress_by_label(n, valid, rec_fn)
    runs_phase2 = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in rec_runs_raw]

    for key, label_fmt, runs, total_boxes, out_name in (
        ("phase1", _format_label, runs_phase1, len(cc_boxes), "phase1_baseline_scenario.png"),
        ("phase2", _format_augmented_label, runs_phase2, len(rec_boxes), "phase2_augmented_scenario.png"),
    ):
        snapshots = make_snapshots(runs, rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w)
        # make_snapshots always formats with _format_label; override box_index
        # for phase 2 so the augmented label components are visible too.
        for snap, r in zip(snapshots, runs):
            snap.box_index = label_fmt(r.box_id)
        out_path = f"{OUT_DIR}/{out_name}"
        title = (f"{'フェーズ1: C&C述語抽象化のみ' if key == 'phase1' else 'フェーズ2: C&C述語+実イベント/criticality metrics'}"
                 f"（ログ0067、全箱数={total_boxes}）")
        plot_scenario_snapshot_sequence(
            snapshots, out_path,
            ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
            title=title, show_time=True, transition_arrow_style="panel",
            panel_w_in=1.9, panel_h_in=2.3, t_ref=timestamps[0],
        )
        print(f"{title} -> {out_path}")


if __name__ == "__main__":
    run()
