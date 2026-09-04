"""10ログ規模での5variant比較（1ログでの予備実験 docs/method.md の一般化）。

ログの選定は、元のsgcpdリポジトリの12.24節「AJISAIカットインの衝突ログ
5本・非衝突ログ5本」の選定をそのまま踏襲する（衝突ログ0067は12.19節以降
一貫して使ってきたパイロットログでもある）。

各ログについて、(1)〜(5)の5つのvariantそれぞれの:
  - 真の箱数 (distinct box数)
  - purity（自分自身が対象とする安全性モデルのonsetに対して。
    (1)(2)は汎用格子なのでC&C onset・RSS onsetの両方を評価対象とする。
    (3)はC&C onsetのみ、(4)はRSS onsetのみ、(5)はC&C onsetのみ。）
  - Z3のmembership check時間（`multi_log_model._model_from_sequences`と
    `verify_logs_included`を使って、5variantすべてに同じ方法でモデルを
    構築し、同じmembership checkを1回走らせて計測する。述語抽象化(3)(4)
    は箱の同一性をラベルで管理するため、ラベル->連番のマッピングを
    `(0, box_id)`という2要素のBoxKeyに変換してから同じ関数に渡す）

を計測し、`out_gif/multi_log_five_abstractions/results.csv`（ログ×variant
の生データ）と集計グラフを出力する。

How to run / 実行方法:
    cd cpd-safety-abstraction && python3 -m logverify.multi_log_five_abstractions
"""

import csv
import json
import os
import subprocess
import sys
import time
import types
from dataclasses import dataclass
from typing import Optional

# Z3のmembership check (`verify_logs_included`) は、箱数(=モデルのstep数)が
# 大きい計量格子 variant では制約構築が非常に重くなることが分かっている
# (12.24節・docs/method.mdの既知の限界)。10ログ x 5variant を現実的な時間で
# 終えるため、1回のmembership checkを別プロセスで実行し、OSレベルの
# タイムアウト(プロセスkill)をかける。シグナルベースのタイムアウトは、
# 割り込みがz3オブジェクトの後始末(__del__)中に発生すると内部状態が壊れ、
# プロセス全体がハングしうることが分かったため、プロセス分離の方式に
# 切り替えた。タイムアウトした場合はエラーにせず「時間内に完了しなかった」
# という、それ自体が意味のあるスケーラビリティのデータ点として記録する。
Z3_TIMEOUT_S = 10

from logverify.paths import DATA_DIR
from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import (
    auto_grid_params_from_ajisai,
    auto_grid_params_naive_uniform,
    auto_near_range_from_risk_frame,
    AutoGridParams,
)
from logverify.grid_bridge import compress_to_grid_states_variable_hysteresis
from logverify.safety_predicate_abstraction import (
    compress_by_label, cc_predicate_label_fn, rss_predicate_label_fn,
)
from logverify.compare_safety_model_abstractions import _purity_for_onset

OUT_DIR = "out_gif/multi_log_five_abstractions"
RESULTS_CSV = f"{OUT_DIR}/results.csv"

# sgcpdリポジトリ12.24節と同じログ選定。
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


@dataclass
class VariantResult:
    log_id: str
    is_collision: bool
    variant: str
    n_boxes: int
    cc_applicable: bool
    cc_pure: Optional[bool]
    cc_smear_m: Optional[float]
    rss_applicable: bool
    rss_pure: Optional[bool]
    rss_smear_m: Optional[float]
    z3_membership_time_s: Optional[float]
    z3_all_sat: Optional[bool]
    z3_timed_out: bool


def _smear_fields(purity: dict):
    if not purity.get("applicable"):
        return False, None, None
    pure = purity["pure"]
    smear = 0.0 if pure else purity["box_rx_span_m"]
    return True, pure, smear


def _time_membership_check(sequence, car="NPC"):
    """`sequence`（(lane_like, position_like)の列。generic BoxKey）を
    別プロセス(`logverify._z3_timing_worker`)に渡し、そのプロセス内で
    gcpd.Modelを構築してmembership checkにかかった時間(秒)とSATだったかを
    計測する。`Z3_TIMEOUT_S`秒以内にプロセスが終了しなければ、
    (None, None, True)（timed_out=True）を返す。

    5variantすべてに同一の`_model_from_sequences`(方法Cの共通モデル構築
    ロジック。格子の切り方には依存しない)を使うことで、"箱数がZ3の
    コストにどう効くか"を、抽象化方式に依存しない条件で比較できる。
    """
    payload = json.dumps(sequence)
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "logverify._z3_timing_worker"],
            input=payload, capture_output=True, text=True, timeout=Z3_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        return None, None, True
    if proc.returncode != 0:
        raise RuntimeError(f"_z3_timing_worker failed: {proc.stderr}")
    out = json.loads(proc.stdout.strip().splitlines()[-1])
    return out["elapsed_s"], out["all_sat"], False


def analyze_log(json_path: str, log_id: str, is_collision: bool):
    data = _load(json_path)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, ego_speed, npc_speed)

    geometry_grid = auto_grid_params_from_ajisai(json_path)
    rx_extent = max(abs(v) for v in rxs if v is not None)
    gy = geometry_grid.gy
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    results = []

    def add_metric_grid_variant(name, grid: AutoGridParams):
        states = compress_to_grid_states_variable_hysteresis(
            rxs, rys, grid.rx_near_cell, grid.rx_far_cell, grid.rx_near_range, gy, margin_ratio=0.3)
        n_boxes = len(states)
        cc_p = _purity_for_onset(rxs, states, cc_risk_frame)
        rss_p = _purity_for_onset(rxs, states, rss_risk_frame)
        cc_app, cc_pure, cc_smear = _smear_fields(cc_p)
        rss_app, rss_pure, rss_smear = _smear_fields(rss_p)
        sequence = [(s.k, s.i) for s in states]
        z3_t, z3_sat, z3_timeout = _time_membership_check(sequence)
        results.append(VariantResult(
            log_id, is_collision, name, n_boxes,
            cc_app, cc_pure, cc_smear, rss_app, rss_pure, rss_smear, z3_t, z3_sat, z3_timeout,
        ))

    def add_predicate_variant(name, label_fn, onset_side: str):
        runs, label_of_box = compress_by_label(n, valid, label_fn)
        n_boxes = len(label_of_box)
        # 述語抽象化はpurity計算を、汎用格子(GridState: .index/.start_frame/
        # .end_frame)と同じインターフェイスに変換してから使う。LabelRunは
        # フィールド名が異なる(box_id)ため、.index を持つ薄いラッパーに
        # 詰め替える(_box_span_containing_frame が s.index を参照するため)。
        purity_states = [
            types.SimpleNamespace(index=r.box_id, start_frame=r.start_frame, end_frame=r.end_frame)
            for r in runs
        ]
        cc_p = _purity_for_onset(rxs, purity_states, cc_risk_frame if onset_side == "cc" else None)
        rss_p = _purity_for_onset(rxs, purity_states, rss_risk_frame if onset_side == "rss" else None)
        cc_app, cc_pure, cc_smear = _smear_fields(cc_p)
        rss_app, rss_pure, rss_smear = _smear_fields(rss_p)
        # 述語abstraction: 箱の同一性はラベル。汎用のBoxKey形式(2要素タプル)
        # に変換してから、汎用格子と全く同じmembership check経路に渡す。
        label_to_id = {}
        sequence = []
        for r in runs:
            if r.label not in label_to_id:
                label_to_id[r.label] = len(label_to_id)
            sequence.append((0, label_to_id[r.label]))
        z3_t, z3_sat, z3_timeout = _time_membership_check(sequence)
        results.append(VariantResult(
            log_id, is_collision, name, n_boxes,
            cc_app, cc_pure, cc_smear, rss_app, rss_pure, rss_smear, z3_t, z3_sat, z3_timeout,
        ))

    # (1) 車両物理サイズ基準(一様格子)
    g1 = auto_grid_params_naive_uniform(rx_extent, cell_width=geometry_grid.rx_near_cell, gy=gy)
    add_metric_grid_variant("(1) 車両物理サイズ基準", g1)

    # (2) 一様格子ベースライン
    g2 = auto_grid_params_naive_uniform(rx_extent, cell_width=2.0, gy=gy)
    add_metric_grid_variant("(2) 一様格子ベースライン", g2)

    # (3) JAMA C&C述語抽象化
    cc_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=gy)
    add_predicate_variant("(3) JAMA C&C述語抽象化", cc_fn, onset_side="cc")

    # (4) RSS述語抽象化
    rss_fn = rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, rss_risk_frame, near_rx=40.0, gy=gy)
    add_predicate_variant("(4) RSS述語抽象化", rss_fn, onset_side="rss")

    # (5) 参考: C&C基準near/far計量格子 (near_rangeのみC&C onsetに連動)
    near_range5 = auto_near_range_from_risk_frame(rxs, cc_risk_frame, margin_factor=1.2, default=15.0)
    g5 = AutoGridParams(gy=gy, rx_near_cell=geometry_grid.rx_near_cell,
                         rx_near_range=near_range5, rx_far_cell=geometry_grid.rx_far_cell)
    add_metric_grid_variant("(5) 参考:C&C基準near/far格子", g5)

    return results


def run():
    os.makedirs(OUT_DIR, exist_ok=True)
    all_results = []
    for name in COLLISION_LOGS:
        path = str(DATA_DIR / name)
        print(f"=== {name} (collision) ===")
        r = analyze_log(path, name, is_collision=True)
        for vr in r:
            z3_str = f"{vr.z3_membership_time_s:.3f}s sat={vr.z3_all_sat}" if not vr.z3_timed_out else f">{Z3_TIMEOUT_S}s (timeout)"
            print(f"  {vr.variant}: boxes={vr.n_boxes} cc_pure={vr.cc_pure} rss_pure={vr.rss_pure} z3={z3_str}")
        all_results.extend(r)
    for name in NON_COLLISION_LOGS:
        path = str(DATA_DIR / name)
        print(f"=== {name} (non-collision) ===")
        r = analyze_log(path, name, is_collision=False)
        for vr in r:
            z3_str = f"{vr.z3_membership_time_s:.3f}s sat={vr.z3_all_sat}" if not vr.z3_timed_out else f">{Z3_TIMEOUT_S}s (timeout)"
            print(f"  {vr.variant}: boxes={vr.n_boxes} cc_pure={vr.cc_pure} rss_pure={vr.rss_pure} z3={z3_str}")
        all_results.extend(r)

    with open(RESULTS_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(vars(all_results[0]).keys()))
        writer.writeheader()
        for vr in all_results:
            writer.writerow(vars(vr))
    print(f"\n結果を書き出しました: {RESULTS_CSV}")
    return all_results


if __name__ == "__main__":
    run()
