"""Section 13.1 (prototype): annotating the safety-model-guided predicate
abstraction (Section 12.25-12.28, `safety_predicate_abstraction.py`) with
independently-derived event/metric timestamps, so that after abstraction we
can check whether the resulting BOX sequence actually preserves the
distinctions we would want to inspect.

## Motivation (from user discussion, 2026-09-05)

The existing predicate abstraction (`cc_predicate_label_fn`,
`rss_predicate_label_fn`) draws its box boundaries purely from each safety
model's OWN state variables (JAMA C&C's risk-perceived frame / RSS's
violation-onset frame, plus contact). That is deliberate -- see
`docs/method.md` -- but it leaves open a separate question: does the
resulting box sequence still let us *locate*, after the fact,

  1. the timing of REAL driver-behavior events -- when Ego actually started
     braking, and when its actual deceleration profile changed -- as
     observed in the log itself (NOT the safety model's own simulated/
     counterfactual deceleration or speed, which is a different quantity);
     and
  2. the timing of standard criticality-metric milestones (TTC zone
     crossings, closing-distance milestones, Ego speed milestones)?

These are not proposed as new box-boundary criteria (that would reintroduce
the metric-grid problem Section 12.25 moved away from). Instead they are
computed independently and then checked against the *existing* predicate
abstraction's run boundaries, frame by frame, to see whether each event
lands cleanly on a box boundary (the abstraction "sees" it) or is buried
inside a larger box (the abstraction is blind to it at the current
granularity).

## What counts as "real" vs. "safety-model" deceleration

`jama_cc_model.cc_deceleration_at` and `rss_model.simulate_rss_reference`
both define a *counterfactual* deceleration profile the safety model
prescribes from its own risk frame onward -- this is a hypothetical, not
something Ego actually did. This module instead reads Ego's actually
achieved longitudinal acceleration directly from the log
(`groundtruth_ego.acceleration.linear.x`, the same field
`reference_model_comparison.actual_accel_series` already extracts) and
detects events purely from that real signal. The two must not be
conflated: "brake onset" here means the frame at which Ego's real
acceleration first drops persistently below a small negative threshold,
not the frame the C&C/RSS model *would have* started braking.

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.event_metric_annotations
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame, cutin_onset_frame,
)
from logverify.reference_model_comparison import (
    ego_speed_series, actual_accel_series, compute_ttc, ttc_zone,
    TTC_DANGER, TTC_CAUTION,
)
from logverify.jama_cc_model import find_risk_perceived_frame, _first_persistent_trigger
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_near_range_from_risk_frame
from logverify.safety_predicate_abstraction import (
    cc_predicate_label_fn, rss_predicate_label_fn, compress_by_label, LabelRun,
)

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from logverify.paths import LOG_0067 as LOG_PATH  # see logverify/paths.py
OUT_PATH = "out_gif/event_metric_annotations.png"

# Real-braking-onset threshold: a small negative longitudinal acceleration,
# well below normal cruise-speed noise but well above the C&C model's
# eventual 0.774G (7.6 m/s^2) so it fires on the *start* of braking, not
# only once braking is already severe.
#
# Calibration note (log 0067): Ego's actual deceleration during the
# critical window (risk-perceived frame 588 to the collision) never
# exceeds about -0.5 m/s^2 -- far short of the JAMA C&C model's 0.774G
# (~7.6 m/s^2) reference response. A -0.5 threshold therefore misses
# Ego's real (weak) braking attempt entirely and instead only catches an
# unrelated -0.9 m/s^2 blip at frame 53 (an initial speed-settling
# maneuver at the very start of the log, long before the cut-in). -0.15
# is low enough to catch Ego's real, if inadequate, response while still
# well above sensor/control noise.
REAL_BRAKE_ONSET_THRESHOLD = -0.15  # m/s^2

# A "deceleration-change" event: a jump in the (finite-difference) jerk of
# Ego's real acceleration exceeding this magnitude, persisting briefly.
# This flags moments where the driver noticeably changes how hard it is
# braking/releasing -- independent of any safety model's own profile.
JERK_CHANGE_THRESHOLD = 3.0  # m/s^3
JERK_PERSIST_FRAMES = 2
JERK_EVENT_MERGE_WINDOW = 10  # frames; merge change-points closer than this

# Closing-distance milestones (m), evaluated on rx - contact boundary.
DISTANCE_MILESTONES = (20.0, 10.0, 5.0)


@dataclass
class Event:
    name: str
    frame: int
    category: str  # "real_behavior" | "criticality_metric"


def real_brake_onset_frame(accel: Sequence[float], threshold=REAL_BRAKE_ONSET_THRESHOLD,
                            persist_frames=3) -> Optional[int]:
    """Egoの実際の縦方向加速度が持続的にthreshold未満になった最初のフレーム
    （実際のブレーキ開始）。安全性モデルが処方する反実仮想の減速とは別物。

    ---
    English: first frame at which Ego's real longitudinal acceleration
    persistently drops below `threshold` -- the actual braking onset, as
    distinct from any safety model's prescribed/counterfactual deceleration.
    """
    def below(i):
        return accel[i] is not None and accel[i] < threshold
    return _first_persistent_trigger(len(accel), below, persist_frames)


def real_deceleration_change_frames(accel: Sequence[float], timestamps: Sequence[float],
                                     jerk_threshold=JERK_CHANGE_THRESHOLD,
                                     persist_frames=JERK_PERSIST_FRAMES,
                                     merge_window=JERK_EVENT_MERGE_WINDOW) -> List[int]:
    """Egoの実加速度の時間微分(jerk)が閾値を持続的に超える箇所を、
    「減速度が明確に変化した」イベントとして検出する。近接するイベントは
    1つにまとめる。

    ---
    English: detects frames where the jerk (time derivative) of Ego's real
    acceleration persistently exceeds `jerk_threshold` in magnitude --
    flagged as a "deceleration changed noticeably" event. Nearby detections
    are merged into one.
    """
    n = len(accel)
    jerks = [None] * n
    for i in range(n - 1):
        dt = timestamps[i + 1] - timestamps[i]
        if dt <= 0 or accel[i] is None or accel[i + 1] is None:
            continue
        jerks[i] = (accel[i + 1] - accel[i]) / dt

    def big_jerk(i):
        return jerks[i] is not None and abs(jerks[i]) > jerk_threshold

    events = []
    run = 0
    for i in range(n):
        if big_jerk(i):
            run += 1
            if run == persist_frames:
                events.append(i - persist_frames + 1)
        else:
            run = 0

    merged = []
    for e in events:
        if merged and e - merged[-1] <= merge_window:
            continue
        merged.append(e)
    return merged


def ttc_zone_transition_frames(ttcs: Sequence[Optional[float]],
                                persist_frames: int = 3) -> List[Tuple[int, str, str]]:
    """TTCが初めて各ゾーン(caution/danger)に持続的に到達したフレームの
    リスト(frame, "safe", to_zone)。

    12.24節で明らかになったのと同じ理由(TTCは相対速度の有限差分から
    計算されるため単一フレームのノイズに弱い)で、素朴な「1フレーム前と
    ゾーンが違えば遷移」という判定は同じ危険度への出入りを何十回も検出
    してしまう(実際、この関数の前バージョンではログ0067で90件以上の
    「悪化」を検出した——ほとんどがTTCがしきい値付近で振動しているだけの
    ノイズだった)。`find_risk_perceived_frame`と同じ
    `_first_persistent_trigger`のパターンを使い、「danger/cautionゾーンに
    最初に持続的に到達した」1フレームだけを、ゾーンごとに1回だけ報告する。

    ---
    English: for each of the "caution"/"danger" zones, the first frame at
    which TTC persistently reaches that zone (frame, "safe", to_zone) --
    reported at most once per zone.

    For the same reason surfaced in Section 12.24 (TTC, from a finite
    difference of relative velocity, is fragile to single-frame noise), a
    naive "zone differs from the previous frame" check fires dozens of
    times as TTC oscillates near a threshold (the earlier version of this
    function found 90+ "worsening" events on log 0067, almost all noise).
    This reuses the same `_first_persistent_trigger` pattern as
    `find_risk_perceived_frame` to report each zone's first persistent
    arrival exactly once.
    """
    def reaches(zone_name):
        order = {"safe": 0, "caution": 1, "danger": 2}
        def pred(i):
            v = ttcs[i]
            return v is not None and order[ttc_zone(v)] >= order[zone_name]
        return pred

    out = []
    for zone_name in ("caution", "danger"):
        f = _first_persistent_trigger(len(ttcs), reaches(zone_name), persist_frames)
        if f is not None:
            out.append((f, "safe", zone_name))
    return out


def distance_milestone_frames(rxs: Sequence[Optional[float]], contact_len: float,
                               milestones=DISTANCE_MILESTONES) -> List[Tuple[int, float]]:
    """縦方向の接触境界までの距離(rx - contact_len)が各milestoneを最初に
    下回ったフレーム。近づく方向のみを対象とする。

    ---
    English: for each milestone distance, the first frame at which the
    longitudinal gap to the contact boundary drops below it (closing only).
    """
    out = []
    for m in milestones:
        below = False
        for i, rx in enumerate(rxs):
            if rx is None:
                continue
            gap = rx - contact_len
            if not below and gap < m:
                out.append((i, m))
                below = True
    return out


def speed_milestone_frames(ego_speed: Sequence[float], onset_frame: int) -> List[Tuple[int, str]]:
    """Ego速度の節目: カットイン開始前の巡航速度(v0)を基準に、半減・
    ほぼ停止(<1m/s)に最初に達したフレーム。

    走査は`onset_frame`以降に限る。ログの冒頭は多くの場合まだ巡航速度に
    達する前の加速区間であり、そこから素朴に0フレーム目から走査すると、
    巡航速度に達する「前」の低速フェーズがそのまま「半減速度」「ほぼ停止」
    として誤検出される（実際にログ0067で発生した）。

    ---
    English: Ego speed milestones relative to its pre-cut-in cruising speed
    v0: first frame (at or after `onset_frame`) reaching half of v0, and
    first frame reaching near-stop (<1 m/s).

    The scan starts at `onset_frame`, not frame 0: the start of a log is
    often still in the initial acceleration-to-cruise phase, and naively
    scanning from frame 0 misidentifies that low-speed-before-cruise phase
    as "half speed" / "near stop" (observed in practice on log 0067).
    """
    lookback = ego_speed[max(0, onset_frame - 50):onset_frame + 1]
    v0 = sum(lookback) / len(lookback) if lookback else ego_speed[onset_frame]
    out = []
    half_hit, stop_hit = False, False
    for i in range(onset_frame, len(ego_speed)):
        v = ego_speed[i]
        if not half_hit and v < 0.5 * v0:
            out.append((i, f"半減速度(<{0.5*v0:.1f}m/s, v0={v0:.1f}m/s)"))
            half_hit = True
        if not stop_hit and v < 1.0:
            out.append((i, "ほぼ停止(<1.0m/s)"))
            stop_hit = True
    return out


def locate_event_in_runs(frame: int, runs: List[LabelRun]) -> Optional[LabelRun]:
    for r in runs:
        if r.start_frame <= frame <= r.end_frame:
            return r
    return None


def annotate(events: List[Event], runs: List[LabelRun], model_name: str) -> List[dict]:
    rows = []
    for ev in events:
        run = locate_event_in_runs(ev.frame, runs)
        if run is None:
            rows.append(dict(event=ev.name, frame=ev.frame, category=ev.category,
                              model=model_name, box_label=None, at_boundary=None,
                              run_span=None))
            continue
        at_boundary = (ev.frame == run.start_frame)
        rows.append(dict(
            event=ev.name, frame=ev.frame, category=ev.category, model=model_name,
            box_label=run.label, at_boundary=at_boundary,
            run_span=(run.start_frame, run.end_frame),
        ))
    return rows


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
    closest_frame, _ = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    cutin_frame = cutin_onset_frame(rys, closest_frame)

    # --- Existing safety-model onsets & predicate abstractions (unchanged) ---
    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, rys, timestamps, ego_speed, npc_speed)
    gy = 0.364
    near_rx = 40.0
    near_ry_cc = auto_near_range_from_risk_frame(rys, cc_risk_frame, margin_factor=1.2, default=10.0)
    near_ry_rss = auto_near_range_from_risk_frame(rys, rss_risk_frame, margin_factor=1.2, default=10.0)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]
    cc_label_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx, gy, near_ry=near_ry_cc)
    rss_label_fn = rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, rss_risk_frame, near_rx, gy, near_ry=near_ry_rss)
    cc_runs, cc_boxes = compress_by_label(n, valid, cc_label_fn)
    rss_runs, rss_boxes = compress_by_label(n, valid, rss_label_fn)

    # --- New: independently-derived events ---
    events: List[Event] = []
    brake_frame = real_brake_onset_frame(accel)
    if brake_frame is not None:
        events.append(Event("実ブレーキ開始 (real brake onset)", brake_frame, "real_behavior"))
    for f in real_deceleration_change_frames(accel, timestamps):
        events.append(Event(f"実減速度変化 (real decel change)", f, "real_behavior"))
    for f, frm, to in ttc_zone_transition_frames(ttcs):
        events.append(Event(f"TTCゾーン悪化 {frm}->{to}", f, "criticality_metric"))
    for f, m in distance_milestone_frames(rxs, eh_l + nh_l):
        events.append(Event(f"距離節目 <{m:.0f}m", f, "criticality_metric"))
    for f, label in speed_milestone_frames(ego_speed, cutin_frame):
        events.append(Event(f"速度節目 {label}", f, "criticality_metric"))
    events.sort(key=lambda e: e.frame)

    print(f"検出されたイベント数: {len(events)}")
    print(f"JAMA C&C risk-perceived frame: {cc_risk_frame}, RSS violation-onset frame: {rss_risk_frame}")
    print()

    all_rows = []
    for model_name, runs in (("C&C", cc_runs), ("RSS", rss_runs)):
        print(f"=== {model_name}述語抽象化に対するイベント対応 ===")
        rows = annotate(events, runs, model_name)
        all_rows.extend(rows)
        n_boundary = sum(1 for r in rows if r["at_boundary"])
        n_buried = sum(1 for r in rows if r["at_boundary"] is False)
        n_outside = sum(1 for r in rows if r["at_boundary"] is None)
        for r in rows:
            if r["at_boundary"] is None:
                status = "対象範囲外(FARの外側/無効フレーム)"
            elif r["at_boundary"]:
                status = f"BOX境界と一致 -> label={r['box_label']}"
            else:
                status = (f"BOX内部に埋没(box run {r['run_span']}) -> "
                          f"label={r['box_label']}")
            print(f"  frame={r['frame']:5d}  {r['event']:38s} {status}")
        print(f"  -> 境界一致:{n_boundary}件 / 内部埋没:{n_buried}件 / 対象外:{n_outside}件")
        print()

    path = plot_annotations(gk, rxs, accel, ttcs, ego_speed, events, cc_runs, rss_runs,
                             cc_risk_frame, rss_risk_frame)
    print(f"図を書き出しました: {path}")
    return all_rows


def plot_annotations(gk, rxs, accel, ttcs, ego_speed, events, cc_runs, rss_runs,
                      cc_risk_frame, rss_risk_frame, output_path=OUT_PATH):
    ts = [rec["timestamp"] - gk[0]["timestamp"] for rec in gk]
    win_start = max(0, min(e.frame for e in events) - 30) if events else 0
    win_end = min(len(gk), max(e.frame for e in events) + 30) if events else len(gk)
    plot_ts = ts

    fig, (ax_box, ax_accel, ax_ttc) = plt.subplots(
        3, 1, figsize=(12, 8), sharex=True, gridspec_kw={"height_ratios": [1.2, 1.5, 1.5]})

    def shade_runs(ax, runs, y0, y1, alpha=0.12):
        palette = ["#1565c0", "#e53935", "#43a047", "#fb8c00", "#8e24aa", "#00897b", "#6d4c41"]
        color_of = {}
        for r in runs:
            key = str(r.label)
            if key not in color_of:
                color_of[key] = palette[len(color_of) % len(palette)]
            ax.axvspan(plot_ts[r.start_frame], plot_ts[min(r.end_frame + 1, len(plot_ts) - 1)],
                       color=color_of[key], alpha=alpha, linewidth=0)
            ax.axvline(plot_ts[r.start_frame], color=color_of[key], linewidth=0.8, alpha=0.6)

    shade_runs(ax_box, cc_runs, 0, 1)
    ax_box.set_yticks([])
    ax_box.set_ylabel("C&C BOX\n(境界=縦線)")
    ax_box.set_title("述語抽象化のBOX境界 vs. 実イベント/criticality metrics節目（ログ0067）", fontsize=11)

    ax_accel.plot(plot_ts[win_start:win_end], accel[win_start:win_end], color="#e53935", linewidth=1.4,
                  label="実Ego縦加速度 (achieved)")
    ax_accel.axhline(REAL_BRAKE_ONSET_THRESHOLD, color="#9e9e9e", linestyle=":", linewidth=0.8)
    ax_accel.set_ylabel("加速度 (m/s^2)")
    ax_accel.legend(loc="lower left", fontsize=8)

    finite = [(t, v) for t, v in zip(plot_ts[win_start:win_end], ttcs[win_start:win_end]) if v is not None and v < 30]
    if finite:
        xs, ys = zip(*finite)
        ax_ttc.plot(xs, ys, color="#1565c0", linewidth=1.4)
    ax_ttc.axhspan(0, TTC_DANGER, color="#e53935", alpha=0.1)
    ax_ttc.axhspan(TTC_DANGER, TTC_CAUTION, color="#fb8c00", alpha=0.1)
    ax_ttc.set_ylim(0, 10)
    ax_ttc.set_ylabel("TTC (s)")
    ax_ttc.set_xlabel("経過時間 (s)")

    marker_style = {"real_behavior": ("o", "#c62828"), "criticality_metric": ("^", "#1565c0")}
    for ax in (ax_box, ax_accel, ax_ttc):
        for ev in events:
            if not (win_start <= ev.frame < win_end):
                continue
            marker, color = marker_style[ev.category]
            ax.axvline(plot_ts[ev.frame], color=color, linestyle="--", linewidth=0.6, alpha=0.5)

    for ev in events:
        if win_start <= ev.frame < win_end:
            marker, color = marker_style[ev.category]
            ax_accel.scatter([plot_ts[ev.frame]], [accel[ev.frame]], marker=marker, color=color,
                              s=50, zorder=5, edgecolor="black", linewidth=0.5)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    run()
