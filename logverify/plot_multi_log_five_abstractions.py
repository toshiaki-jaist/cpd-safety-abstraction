"""`multi_log_five_abstractions.py`が書き出した10ログ x 5variantの結果
(`out_gif/multi_log_five_abstractions/results.csv`)を集計し、以下の図表を
作る:

  1. box_counts_per_log.png   -- ログごと・variantごとの真の箱数(対数軸)
  2. box_counts_boxplot.png   -- variantごとの箱数分布(10ログ, 箱ひげ図)
  3. purity_summary.png       -- 「大事な境界(自分自身の安全性モデルの
                                  onset)をつぶしていないか」の10ログ集計
                                  (pure率 + impureな場合の平均smear)
  4. z3_cost_vs_boxes.png     -- 追加の分析観点: 真の箱数とZ3 membership
                                  checkのコスト(計測できた時間、または
                                  タイムアウト)の関係。10秒以内に完了した
                                  割合をvariantごとに示す。
  5. summary_table.md         -- 上記の数値サマリをMarkdown表として出力

How to run / 実行方法:
    cd cpd-safety-abstraction
    python3 -m logverify.multi_log_five_abstractions   # 先にこちらを実行してresults.csvを作る
    python3 -m logverify.plot_multi_log_five_abstractions
"""

import csv
import statistics
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

RESULTS_CSV = "out_gif/multi_log_five_abstractions/results.csv"
OUT_DIR = "out_gif/multi_log_five_abstractions"

VARIANTS = [
    "(1) 車両物理サイズ基準",
    "(2) 一様格子ベースライン",
    "(3) JAMA C&C述語抽象化",
    "(4) RSS述語抽象化",
    "(5) 参考:C&C基準near/far格子",
    "(3b) JAMA C&C述語抽象化+ry境界",
    "(4b) RSS述語抽象化+ry境界",
    "(6) C&C+RSS同時述語抽象化",
]
VARIANT_COLORS = {
    "(1) 車両物理サイズ基準": "#78909c",
    "(2) 一様格子ベースライン": "#757575",
    "(3) JAMA C&C述語抽象化": "#1565c0",
    "(4) RSS述語抽象化": "#ef6c00",
    "(5) 参考:C&C基準near/far格子": "#9e9e9e",
    "(3b) JAMA C&C述語抽象化+ry境界": "#0d47a1",
    "(4b) RSS述語抽象化+ry境界": "#e65100",
    "(6) C&C+RSS同時述語抽象化": "#6a1b9a",
}
# 「自分自身が対象とする安全性モデル」のonsetだけを見る、という
# docs/method.mdの方針(cross-model purityは評価対象にしない)。
# (6)は12.28節: C&C・RSS両方のonsetを対象とする("both")。
OWN_ONSET_SIDE = {
    "(1) 車両物理サイズ基準": "both",
    "(2) 一様格子ベースライン": "both",
    "(3) JAMA C&C述語抽象化": "cc",
    "(4) RSS述語抽象化": "rss",
    "(5) 参考:C&C基準near/far格子": "cc",
    "(3b) JAMA C&C述語抽象化+ry境界": "cc",
    "(4b) RSS述語抽象化+ry境界": "rss",
    "(6) C&C+RSS同時述語抽象化": "both",
}


def _parse_bool(s):
    if s in ("", None):
        return None
    return s == "True"


def _parse_float(s):
    if s in ("", None):
        return None
    return float(s)


def load_results(path=RESULTS_CSV):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["is_collision"] = _parse_bool(row["is_collision"])
            row["n_boxes"] = int(row["n_boxes"])
            row["cc_applicable"] = _parse_bool(row["cc_applicable"])
            row["cc_pure"] = _parse_bool(row["cc_pure"])
            row["cc_smear_m"] = _parse_float(row["cc_smear_m"])
            row["rss_applicable"] = _parse_bool(row["rss_applicable"])
            row["rss_pure"] = _parse_bool(row["rss_pure"])
            row["rss_smear_m"] = _parse_float(row["rss_smear_m"])
            row["z3_membership_time_s"] = _parse_float(row["z3_membership_time_s"])
            row["z3_all_sat"] = _parse_bool(row["z3_all_sat"])
            row["z3_timed_out"] = _parse_bool(row["z3_timed_out"])
            rows.append(row)
    return rows


def plot_box_counts_per_log(rows, out_path=f"{OUT_DIR}/box_counts_per_log.png"):
    logs = sorted({r["log_id"] for r in rows}, key=lambda l: (
        0 if any(r["log_id"] == l and r["is_collision"] for r in rows) else 1, l))
    by_log_variant = {(r["log_id"], r["variant"]): r["n_boxes"] for r in rows}

    x = np.arange(len(logs))
    n_variants = len(VARIANTS)
    w = 0.8 / n_variants
    fig, ax = plt.subplots(figsize=(16, 6.5))
    center = (n_variants - 1) / 2
    for i, variant in enumerate(VARIANTS):
        vals = [by_log_variant[(log, variant)] for log in logs]
        ax.bar(x + (i - center) * w, vals, width=w, label=variant, color=VARIANT_COLORS[variant])
    ax.set_yscale("log")
    ax.set_xticks(x)
    short_labels = [l.replace("TD-NI-AR-SD-N04-CI-", "").replace(".json", "") +
                    ("\n(衝突)" if any(r["log_id"] == l and r["is_collision"] for r in rows) else "\n(非衝突)")
                    for l in logs]
    ax.set_xticklabels(short_labels, fontsize=8.5)
    ax.set_ylabel("真の箱数 (distinct boxes, 対数軸)")
    ax.set_title("10ログでの真の箱数比較 (8variant, (3b)(4b)はry境界導入版, (6)はC&C+RSS同時抽象化)")
    ax.legend(fontsize=7.5, ncol=2, loc="upper left")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def plot_box_counts_boxplot(rows, out_path=f"{OUT_DIR}/box_counts_boxplot.png"):
    data = [[r["n_boxes"] for r in rows if r["variant"] == v] for v in VARIANTS]
    fig, ax = plt.subplots(figsize=(12, 5.5))
    bp = ax.boxplot(data, tick_labels=[v.replace("参考:", "参考:\n") for v in VARIANTS], patch_artist=True, showmeans=True)
    for patch, v in zip(bp["boxes"], VARIANTS):
        patch.set_facecolor(VARIANT_COLORS[v])
        patch.set_alpha(0.6)
    for i, v in enumerate(VARIANTS):
        vals = data[i]
        ax.scatter([i + 1] * len(vals), vals, color="black", s=12, alpha=0.6, zorder=3)
    ax.set_yscale("log")
    ax.set_ylabel("真の箱数 (distinct boxes, 対数軸)")
    ax.set_title("10ログにわたる箱数の分布 (中央値・四分位範囲・各ログの値)")
    ax.tick_params(axis="x", labelsize=7.5, rotation=10)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _own_smear_and_purity(row, variant):
    side = OWN_ONSET_SIDE[variant]
    smears = []
    pures = []
    if side in ("cc", "both") and row["cc_applicable"]:
        smears.append(row["cc_smear_m"])
        pures.append(row["cc_pure"])
    if side in ("rss", "both") and row["rss_applicable"]:
        smears.append(row["rss_smear_m"])
        pures.append(row["rss_pure"])
    return pures, smears


def plot_purity_summary(rows, out_path=f"{OUT_DIR}/purity_summary.png"):
    pure_rate = []
    avg_smear_impure = []
    n_applicable = []
    for v in VARIANTS:
        all_pures = []
        all_smears_impure = []
        for r in rows:
            if r["variant"] != v:
                continue
            pures, smears = _own_smear_and_purity(r, v)
            for p, s in zip(pures, smears):
                all_pures.append(p)
                if not p:
                    all_smears_impure.append(s)
        n_applicable.append(len(all_pures))
        pure_rate.append(100.0 * sum(all_pures) / len(all_pures) if all_pures else 0.0)
        avg_smear_impure.append(statistics.mean(all_smears_impure) if all_smears_impure else 0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5.5))

    bars = ax1.bar(range(len(VARIANTS)), pure_rate, color=[VARIANT_COLORS[v] for v in VARIANTS])
    for i, (r, n) in enumerate(zip(pure_rate, n_applicable)):
        ax1.text(i, r + 2, f"{r:.0f}%\n(n={n})", ha="center", fontsize=9)
    ax1.set_ylim(0, 115)
    ax1.set_ylabel("自分自身のonsetでpureだった割合 (%)")
    ax1.set_title("purity率 (10ログ集計、自モデルのonsetのみ)")
    ax1.set_xticks(range(len(VARIANTS)))
    ax1.set_xticklabels([v.replace("参考:", "参考:\n") for v in VARIANTS], fontsize=8, rotation=15)

    bars2 = ax2.bar(range(len(VARIANTS)), avg_smear_impure, color=[VARIANT_COLORS[v] for v in VARIANTS])
    for i, s in enumerate(avg_smear_impure):
        label = "PURE(該当なし)" if s == 0 else f"{s:.2f}m"
        ax2.text(i, s + max(avg_smear_impure) * 0.02 if max(avg_smear_impure) > 0 else 0.02, label, ha="center", fontsize=9)
    ax2.set_ylabel("impureだった場合の平均smear量 (m)")
    ax2.set_title("impure時のsmear量の平均 (10ログ集計)")
    ax2.set_xticks(range(len(VARIANTS)))
    ax2.set_xticklabels([v.replace("参考:", "参考:\n") for v in VARIANTS], fontsize=8, rotation=15)

    fig.suptitle("「大事な境界(onset)をつぶしていないか」の10ログ集計", fontsize=12.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path, list(zip(VARIANTS, pure_rate, avg_smear_impure, n_applicable))


def plot_z3_cost(rows, out_path=f"{OUT_DIR}/z3_cost_vs_boxes.png"):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5.5))

    for v in VARIANTS:
        vr = [r for r in rows if r["variant"] == v]
        finished = [r for r in vr if not r["z3_timed_out"]]
        ax1.scatter([r["n_boxes"] for r in finished], [r["z3_membership_time_s"] for r in finished],
                    label=v, color=VARIANT_COLORS[v], s=45)
    ax1.set_xscale("log")
    ax1.set_xlabel("真の箱数 (対数軸)")
    ax1.set_ylabel("Z3 membership check 時間 (秒。10秒以内に完了したもののみ)")
    ax1.set_title("箱数 vs 実測Z3計算時間")
    ax1.legend(fontsize=7.5)
    ax1.axhline(10, color="red", linestyle="--", linewidth=0.8)

    timeout_rate = []
    for v in VARIANTS:
        vr = [r for r in rows if r["variant"] == v]
        rate = 100.0 * sum(r["z3_timed_out"] for r in vr) / len(vr)
        timeout_rate.append(rate)
    ax2.bar(range(len(VARIANTS)), timeout_rate, color=[VARIANT_COLORS[v] for v in VARIANTS])
    for i, r in enumerate(timeout_rate):
        ax2.text(i, r + 2, f"{r:.0f}%", ha="center", fontsize=9)
    ax2.set_ylim(0, 115)
    ax2.set_ylabel("10秒以内に完了しなかった割合 (%, 10ログ中)")
    ax2.set_title("Z3 membership checkのタイムアウト率")
    ax2.set_xticks(range(len(VARIANTS)))
    ax2.set_xticklabels([v.replace("参考:", "参考:\n") for v in VARIANTS], fontsize=8, rotation=15)

    fig.suptitle("追加の分析観点: 抽象化のスケーラビリティ (Z3 membership checkのコスト)", fontsize=12.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path, list(zip(VARIANTS, timeout_rate))


def write_summary_table(rows, purity_stats, timeout_stats, out_path=f"{OUT_DIR}/summary_table.md"):
    lines = ["# 10ログ集計サマリ\n"]
    lines.append("## 箱数 (10ログ)\n")
    lines.append("| variant | 最小 | 中央値 | 最大 | 平均 |")
    lines.append("|---|---|---|---|---|")
    for v in VARIANTS:
        vals = [r["n_boxes"] for r in rows if r["variant"] == v]
        lines.append(f"| {v} | {min(vals)} | {statistics.median(vals):.0f} | {max(vals)} | {statistics.mean(vals):.1f} |")

    lines.append("\n## purity (自分自身のonsetのみ、10ログ集計)\n")
    lines.append("| variant | pure率 | 該当ログ数 | impure時の平均smear |")
    lines.append("|---|---|---|---|")
    for v, rate, smear, n in purity_stats:
        lines.append(f"| {v} | {rate:.0f}% | {n} | {smear:.2f}m |")

    lines.append("\n## Z3 membership check タイムアウト率 (10秒上限, 10ログ x 各variant)\n")
    lines.append("| variant | タイムアウト率 |")
    lines.append("|---|---|")
    for v, rate in timeout_stats:
        lines.append(f"| {v} | {rate:.0f}% |")

    lines.append("\n## 衝突ログ vs 非衝突ログでの箱数 (中央値)\n")
    lines.append("| variant | 衝突ログ(5本) | 非衝突ログ(5本) |")
    lines.append("|---|---|---|")
    for v in VARIANTS:
        coll = [r["n_boxes"] for r in rows if r["variant"] == v and r["is_collision"]]
        non = [r["n_boxes"] for r in rows if r["variant"] == v and not r["is_collision"]]
        lines.append(f"| {v} | {statistics.median(coll):.0f} | {statistics.median(non):.0f} |")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return out_path


def run():
    rows = load_results()
    p1 = plot_box_counts_per_log(rows)
    print(f"箱数(ログ別)図: {p1}")
    p2 = plot_box_counts_boxplot(rows)
    print(f"箱数(分布)図: {p2}")
    p3, purity_stats = plot_purity_summary(rows)
    print(f"purity集計図: {p3}")
    p4, timeout_stats = plot_z3_cost(rows)
    print(f"Z3コスト図: {p4}")
    p5 = write_summary_table(rows, purity_stats, timeout_stats)
    print(f"サマリ表: {p5}")


if __name__ == "__main__":
    run()
