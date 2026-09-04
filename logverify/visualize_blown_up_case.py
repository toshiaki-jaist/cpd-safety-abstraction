"""ログ0071（衝突ログ）: 述語抽象化(C&C)が266箱まで膨らんだ、
docs/multi_log_results.md 5節で言及した「箱が膨らんだ場合」の具体例と、
12.26節でry方向にもnear_ryの境界を導入した修正版（12箱まで縮小）を、
before/afterとして並べて可視化する。

原因は、`RISK`ラベルのlane_k(ry方向の格子index)にrx方向のnear_rxと
対称な上限がなく、|rx|は近傍にとどまったまま|ry|だけが大きく振れる
(最接近後の物理的な挙動、または計測上のアーティファクト)とlane_kが
際限なく分岐し続けることだった。near_ryを導入し、それを超えるryを
位置によらず単一のFAR箱に潰すことで解消する。

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.visualize_blown_up_case
"""

import os

from logverify.paths import DATA_DIR
from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy, closest_approach_frame
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_grid_params_from_ajisai, auto_near_range_from_risk_frame
from logverify.safety_predicate_abstraction import compress_by_label, cc_predicate_label_fn
from logverify.visualize_five_abstractions import Run, make_snapshots, truncate_near_closest, MAX_BOXES
from logverify.scenario_snapshot_diagram import plot_scenario_snapshot_sequence

LOG_NAME = "TD-NI-AR-SD-N04-CI-0071.json"
OUT_PATH_BEFORE = "out_gif/blown_up_case_0071_cc_predicate.png"
OUT_PATH_AFTER = "out_gif/blown_up_case_0071_cc_predicate_fixed.png"


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

    # --- Before: near_ryなし(元の実装、12.25節時点) ---
    cc_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=geometry_grid.gy)
    cc_runs_raw, cc_boxes = compress_by_label(n, valid, cc_fn)
    all_runs = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in cc_runs_raw]
    print(f"[before] 全箱数(distinct)={len(cc_boxes)}  全run数={len(all_runs)}")

    # 最接近前を少し・最接近後を多めに表示する(通常のtruncate_near_closest
    # は最接近後を3箱までしか見せないため、ここでは専用の切り出しを行う。
    # 箱の急増が最接近後に起きることを見せるため)。
    before = [r for r in all_runs if r.start_frame <= closest_frame]
    after = [r for r in all_runs if r.start_frame > closest_frame]
    n_before = min(6, len(before))
    n_after = MAX_BOXES - n_before
    shown_runs = before[-n_before:] + after[:n_after]
    snapshots = make_snapshots(shown_runs, rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w)
    title = (f"[Before: near_ryなし] ログ0071 JAMA C&C述語抽象化　"
             f"全箱数={len(cc_boxes)}(全run={len(all_runs)})　表示={len(snapshots)}箱"
             f"（最接近前{n_before}箱+最接近後{len(shown_runs)-n_before}箱）")
    plot_scenario_snapshot_sequence(
        snapshots, OUT_PATH_BEFORE,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title=title, show_time=True, transition_arrow_style="panel",
        panel_w_in=1.6, panel_h_in=2.1, t_ref=timestamps[0],
    )
    print(f"-> {OUT_PATH_BEFORE}")

    # --- After: near_ryあり(12.26節の修正版) ---
    near_ry = auto_near_range_from_risk_frame(rys, cc_risk_frame, margin_factor=1.2, default=10.0)
    cc_fn_fixed = cc_predicate_label_fn(
        rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=geometry_grid.gy, near_ry=near_ry)
    cc_runs_fixed, cc_boxes_fixed = compress_by_label(n, valid, cc_fn_fixed)
    all_runs_fixed = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in cc_runs_fixed]
    print(f"[after]  全箱数(distinct)={len(cc_boxes_fixed)}  全run数={len(all_runs_fixed)}  (near_ry={near_ry:.2f}m)")

    snapshots_fixed = make_snapshots(all_runs_fixed, rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w)
    title_fixed = (f"[After: near_ry={near_ry:.2f}m を導入] ログ0071 JAMA C&C述語抽象化　"
                   f"全箱数={len(cc_boxes_fixed)}(全run={len(all_runs_fixed)})　全箱を表示")
    plot_scenario_snapshot_sequence(
        snapshots_fixed, OUT_PATH_AFTER,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title=title_fixed, show_time=True, transition_arrow_style="panel",
        panel_w_in=1.6, panel_h_in=2.1, t_ref=timestamps[0],
    )
    print(f"-> {OUT_PATH_AFTER}")

    return OUT_PATH_BEFORE, OUT_PATH_AFTER


if __name__ == "__main__":
    run()
