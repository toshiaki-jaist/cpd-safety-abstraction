"""ログ0071（衝突ログ）: 述語抽象化(C&C)が266箱まで膨らんだ、
docs/multi_log_results.md 5節で言及した「箱が膨らんだ場合」の具体例を
1つ可視化する。箱数が多いため、visualize_five_abstractions.pyと同じ方針
(MAX_BOXES=25、最接近フレーム付近に絞る)で表示する。

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.visualize_blown_up_case
"""

import os

from logverify.paths import DATA_DIR
from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy, closest_approach_frame
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.safety_predicate_abstraction import compress_by_label, cc_predicate_label_fn
from logverify.visualize_five_abstractions import Run, make_snapshots, truncate_near_closest, MAX_BOXES
from logverify.scenario_snapshot_diagram import plot_scenario_snapshot_sequence

LOG_NAME = "TD-NI-AR-SD-N04-CI-0071.json"
OUT_PATH = "out_gif/blown_up_case_0071_cc_predicate.png"


def run():
    os.makedirs("out_gif", exist_ok=True)
    path = str(DATA_DIR / LOG_NAME)
    data = _load(path)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    closest_frame, _ = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    geometry_grid = auto_grid_params_from_ajisai(path)
    cc_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=geometry_grid.gy)
    cc_runs_raw, cc_boxes = compress_by_label(n, valid, cc_fn)
    all_runs = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in cc_runs_raw]
    print(f"全箱数(distinct)={len(cc_boxes)}  全run数={len(all_runs)}")

    # このログでは、箱の急増が「衝突前の長い接近・複雑な横移動」ではなく、
    # 衝突(最接近)後にrsが激しく振動すること(衝突後の物理挙動)によって
    # lane_kが暴れる(box #RISK k=60, k=312 のような値まで出る)ことが原因
    # だと判明した。それが見えるように、最接近前を少し・最接近後を多めに
    # 表示する(通常のtruncate_near_closestは最接近後を3箱までしか見せない
    # ため、ここでは専用の切り出しを行う)。
    before = [r for r in all_runs if r.start_frame <= closest_frame]
    after = [r for r in all_runs if r.start_frame > closest_frame]
    n_before = min(6, len(before))
    n_after = MAX_BOXES - n_before
    shown_runs = before[-n_before:] + after[:n_after]
    snapshots = make_snapshots(shown_runs, rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w)

    title = (f"[箱が膨らんだ例] ログ0071 JAMA C&C述語抽象化　"
             f"全箱数={len(cc_boxes)}(全run={len(all_runs)})　表示={len(snapshots)}箱"
             f"（最接近前{n_before}箱+最接近後{len(shown_runs)-n_before}箱）")
    plot_scenario_snapshot_sequence(
        snapshots, OUT_PATH,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title=title, show_time=True, transition_arrow_style="panel",
        panel_w_in=1.6, panel_h_in=2.1, t_ref=timestamps[0],
    )
    print(f"-> {OUT_PATH}")
    return OUT_PATH


if __name__ == "__main__":
    run()
