"""Section 13.2 (prototype): fold the Section 13.1 event/criticality-metric
annotations directly into the predicate label itself, so that each of
those transitions becomes a genuine box boundary rather than something
merely annotated after the fact.

## Motivation (user, 2026-09-05)

"If an important transition is buried, shouldn't we split the BOX at that
transition -- i.e., abstract using the C&C model *plus* this event/metric
data mapping together?" This is structurally correct: purity in this
project's predicate abstraction (Section 12.25) holds *by construction*
because the label is defined to change exactly at the safety model's own
onset frame. The same construction works for any other frame we choose to
make part of the label -- including the Section 13.1 events -- at the cost
of (a) more boxes and (b) reintroducing exactly the kind of
externally-chosen numeric threshold (TTC 1.5s/3.0s, distance 20/10/5m,
brake-onset -0.15 m/s^2) that Sections 12.9-12.25 deliberately moved away
from when rejecting arbitrary metric grids.

This is NOT the same situation as the Section 12.28 C&C+RSS combined
label: both C&C and RSS are independently, formally established safety
standards, not thresholds chosen by this project. Combining C&C with the
Section 13.1 events combines a safety model with a set of ad hoc
criticality-metric thresholds. That may still be a reasonable choice for a
research question that specifically cares about those thresholds, but it
is a different kind of choice, with a different justification, and this
module exists to make its quantitative cost (box count vs. the 13-box
baseline) visible before deciding whether to adopt it -- exactly as
Section 12.28 first measured the C&C+RSS combination's cost before judging
whether it was worth it.

## Design

Each Section 13.1 signal is turned into a small monotonic "phase" index
(never decreasing over the log, so a signal that would otherwise oscillate
near a threshold -- e.g. raw per-frame TTC zone -- does not multiply boxes
the way it multiplied *events* in Section 13.1's first, un-filtered
attempt):

  - `real_behavior_phase`: 0, then +1 at the real brake-onset frame, then
    +1 again at each real deceleration-change frame (Section 13.1
    definitions, unchanged).
  - `ttc_ratchet_zone`: "safe" -> "caution" -> "danger", using the same
    persistence-filtered first-arrival frames as Section 13.1 (never
    reverts once reached, so TTC's raw oscillation near a threshold does
    not re-trigger it).
  - `distance_ratchet_zone`: 0 -> 1 -> 2 -> 3 as the closing distance
    first drops below 20m / 10m / 5m.
  - `speed_ratchet_zone`: 0 -> 1 -> 2 as Ego's speed first drops below
    half its pre-cut-in cruising speed, then below 1.0 m/s (only tracked
    from the cut-in onset frame onward, "n/a" before it -- see Section
    13.1's note on why frame 0 is the wrong start point).

The augmented label is the tuple
`(cc_label, real_behavior_phase, ttc_ratchet_zone, distance_ratchet_zone,
speed_ratchet_zone)`, where `cc_label` is exactly
`cc_predicate_label_fn`'s existing label (unchanged). Because every
component is monotonic, the augmented abstraction is pure at every
Section 13.1 event *by construction*, mirroring Section 12.28's argument
for the C&C+RSS combined label.

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.event_augmented_predicate_abstraction
"""

from typing import Callable, Hashable, List, Sequence

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
    cc_predicate_label_fn, compress_by_label, LabelRun, _purity_of_predicate_abstraction,
)
from logverify.event_metric_annotations import (
    real_brake_onset_frame, real_deceleration_change_frames,
    ttc_zone_transition_frames, distance_milestone_frames, speed_milestone_frames,
    Event,
)

from logverify.paths import LOG_0067 as LOG_PATH  # see logverify/paths.py


def _phase_from_breakpoints(frame: int, breakpoints: Sequence[int]) -> int:
    """`breakpoints`（昇順）のうち、frame以下のものの個数を返す
    （単調に増加するフェーズ番号）。

    ---
    English: returns how many of the (ascending) `breakpoints` are <=
    frame -- a monotonically non-decreasing phase index.
    """
    phase = 0
    for bp in breakpoints:
        if frame >= bp:
            phase += 1
    return phase


def real_behavior_phase_fn(brake_frame, decel_change_frames) -> Callable[[int], int]:
    breakpoints = sorted([f for f in [brake_frame] if f is not None] + list(decel_change_frames))
    def fn(frame):
        return _phase_from_breakpoints(frame, breakpoints)
    return fn


def ttc_ratchet_zone_fn(ttc_transitions) -> Callable[[int], str]:
    """ttc_transitionsは[(frame, "safe", "caution"), (frame, "safe", "danger")]
    のような形式(event_metric_annotations.ttc_zone_transition_framesの出力)。
    """
    breakpoints = sorted(f for f, _, _ in ttc_transitions)
    zones = ["safe", "caution", "danger"]
    def fn(frame):
        phase = _phase_from_breakpoints(frame, breakpoints)
        return zones[min(phase, len(zones) - 1)]
    return fn


def distance_ratchet_zone_fn(distance_events) -> Callable[[int], int]:
    breakpoints = sorted(f for f, _ in distance_events)
    def fn(frame):
        return _phase_from_breakpoints(frame, breakpoints)
    return fn


def speed_ratchet_zone_fn(speed_events, start_frame) -> Callable[[int], int]:
    breakpoints = sorted(f for f, _ in speed_events)
    def fn(frame):
        if frame < start_frame:
            return -1  # n/a: before the cut-in onset, not yet meaningful
        return _phase_from_breakpoints(frame, breakpoints)
    return fn


def event_augmented_label_fn(cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn,
                              speed_zone_fn, freeze_far=True) -> Callable[[int], Hashable]:
    """`freeze_far=True`(既定, 推奨): cc_label_fnが`("FAR",)`を返すフレーム
    では補助フェーズを無視し、`(("FAR",),)`という単一のラベルに潰す。
    これにより「遠方(安全性モデルが注意を払わない範囲)は単一の箱」という
    12.25節以来の述語抽象化の原則を、拡張後も保つ。

    `freeze_far=False`は素朴な版(常に全フェーズをラベルに含める)で、
    比較のために残してある — この場合、FAR領域内でも補助フェーズが
    変化し続けるフレーム(例: カットイン前の無関係な減速ブリップ、衝突後の
    無関係な減速変化)がそのままFAR領域を複数の箱に分割してしまう
    (ログ0067で実測: 27箱中4箱がFAR領域の分割によるもの)。

    ---
    English: `freeze_far=True` (default, recommended): whenever
    `cc_label_fn` returns `("FAR",)`, ignore the auxiliary phases and
    collapse to the single label `(("FAR",),)`, preserving the Section
    12.25 principle that the far region (where the safety model does not
    attend) is exactly one box, even after this augmentation.

    `freeze_far=False` is the naive version (always includes every phase
    in the label), kept for comparison -- it lets far-region frames where
    an auxiliary phase happens to change (an irrelevant pre-scenario decel
    blip, an irrelevant post-collision decel change) split the FAR region
    into multiple boxes (measured on log 0067: 4 of 27 boxes were such
    FAR-region splits).
    """
    def label_fn(frame):
        cc_label = cc_label_fn(frame)
        if freeze_far and cc_label == ("FAR",):
            return (cc_label,)
        return (cc_label, real_phase_fn(frame), ttc_zone_fn(frame),
                dist_zone_fn(frame), speed_zone_fn(frame))
    return label_fn


def run():
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

    # --- Baseline: C&C predicate abstraction alone (Section 12.25-12.28, unchanged) ---
    baseline_runs, baseline_boxes = compress_by_label(n, valid, cc_label_fn)

    # --- Section 13.1 events, reused verbatim ---
    brake_frame = real_brake_onset_frame(accel)
    decel_change_frames = real_deceleration_change_frames(accel, timestamps)
    ttc_transitions = ttc_zone_transition_frames(ttcs)
    distance_events = distance_milestone_frames(rxs, eh_l + nh_l)
    speed_events = speed_milestone_frames(ego_speed, cutin_frame)

    real_phase_fn = real_behavior_phase_fn(brake_frame, decel_change_frames)
    ttc_zone_fn = ttc_ratchet_zone_fn(ttc_transitions)
    dist_zone_fn = distance_ratchet_zone_fn(distance_events)
    speed_zone_fn = speed_ratchet_zone_fn(speed_events, cutin_frame)

    naive_label_fn = event_augmented_label_fn(
        cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn, speed_zone_fn, freeze_far=False)
    naive_runs, naive_boxes = compress_by_label(n, valid, naive_label_fn)

    augmented_label_fn = event_augmented_label_fn(
        cc_label_fn, real_phase_fn, ttc_zone_fn, dist_zone_fn, speed_zone_fn, freeze_far=True)
    aug_runs, aug_boxes = compress_by_label(n, valid, augmented_label_fn)

    print(f"=== ベースライン: C&C述語抽象化のみ (12.25-12.28節、変更なし) ===")
    print(f"  真の箱数 = {len(baseline_boxes)}")
    print()
    print(f"=== 拡張版(素朴; FAR領域も含めて全フェーズをラベル化) ===")
    print(f"  真の箱数 = {len(naive_boxes)}  (ベースライン比 x{len(naive_boxes)/len(baseline_boxes):.1f})")
    n_far_split = sum(1 for k in naive_boxes if k[0] == ("FAR",))
    print(f"  うちFAR領域の分割によるもの: {n_far_split}箱"
          f"（「遠方は単一箱」という原則が一部崩れている）")
    print()
    print(f"=== 拡張版(推奨; FAR領域は従来通り単一箱に凍結) ===")
    print(f"  真の箱数 = {len(aug_boxes)}  (ベースライン比 x{len(aug_boxes)/len(baseline_boxes):.1f})")
    print()

    all_events: List[Event] = []
    if brake_frame is not None:
        all_events.append(Event("実ブレーキ開始", brake_frame, "real_behavior"))
    for f in decel_change_frames:
        all_events.append(Event("実減速度変化", f, "real_behavior"))
    for f, frm, to in ttc_transitions:
        all_events.append(Event(f"TTC {frm}->{to}", f, "criticality_metric"))
    for f, m in distance_events:
        all_events.append(Event(f"距離<{m:.0f}m", f, "criticality_metric"))
    for f, label in speed_events:
        all_events.append(Event(f"速度節目 {label}", f, "criticality_metric"))
    all_events.sort(key=lambda e: e.frame)

    print("=== 拡張版(推奨=FAR凍結)での各イベントのpurity ===")
    n_pure = 0
    for ev in all_events:
        p = _purity_of_predicate_abstraction(aug_runs, ev.frame)
        pure = p.get("applicable") and p.get("pure")
        n_pure += int(bool(pure))
        in_far = cc_label_fn(ev.frame) == ("FAR",)
        note = " (FAR領域内のイベント: 意図的に無視される)" if in_far else ""
        print(f"  frame={ev.frame:5d}  {ev.name:20s} {'PURE' if pure else 'IMPURE'}{note}")
    print(f"  -> {n_pure}/{len(all_events)} イベントでBOX境界と一致"
          f"(FAR領域内のイベントを除き、残りは構造上必然的にPURE)")
    print()

    p_cc = _purity_of_predicate_abstraction(aug_runs, cc_risk_frame)
    print(f"C&C自身のonset(frame {cc_risk_frame})でのpurity: "
          f"{'PURE' if p_cc.get('pure') else 'IMPURE'} (これも構造上必然)")

    return dict(baseline_boxes=len(baseline_boxes), aug_boxes=len(aug_boxes),
                n_events=len(all_events), n_pure=n_pure)


if __name__ == "__main__":
    run()
