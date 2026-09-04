"""12.25節: JAMA C&C(Competent and Careful)ドライバモデルの代わりに、
RSS (Responsibility-Sensitive Safety, Shalev-Shwartz, Shammah, Shashua,
2017, arXiv:1708.06374) の縦方向最小安全車間公式を「safety model」として
使った場合の、jama_cc_model.py に相当するプロトタイプ実装。

## 位置づけ

本プロジェクトの検証パイプラインは、次の2つの異なる役割を持つ:

  (A) 抽象化の粒度 (grid): Egoに近い「重要な」領域を細かく、遠方を粗く
      量子化する near/far 格子。これまでは `auto_grid.py` が、
      両車両の物理サイズ（接触境界）から機械的に導出していた
      （JAMA C&Cの閾値を直接には使っていない——12.19〜12.24節の実装は
      「C&Cモデルによる安全性判定」と「格子の粒度決定」が実は別々の
      仕組みだった、という点は本節で明確化しておく）。

  (B) 安全性判定モデル (safety model): 実ログが「有能で慎重な人間なら
      回避できたはずの状況」を実際には回避できなかったかどうかを判定する
      基準。これまでは JAMA C&C モデル（`jama_cc_model.py`）がこの役割を
      担っていた。

ユーザーからの提案は、(B)の安全性判定モデルをJAMA C&CからRSSに差し替えると
どうなるか、というものである。さらに、(A)の格子粒度を安全性判定モデルの
「どこからリスクを気にし始めるか」という判断（risk-perceived frame /
RSS-violation frame）に連動させることで、初めて文字通りの
"safety-model-guided abstraction and refinement" になる——これが
`auto_grid.auto_near_range_from_risk_frame` の役割である。

## RSS 縦方向最小安全車間公式

RSS論文 (Shalev-Shwartz et al. 2017) Definition 3 (Longitudinal Safe
Distance) より:

    d_min = [v_r*rho + (1/2)*a_max_accel*rho^2
             + (v_r + rho*a_max_accel)^2 / (2*b_min)]
            - v_f^2 / (2*b_max)

  - v_r: 後続車（rear car、これから安全性が問われる側）の速度
  - v_f: 先行車（front car）の速度
  - rho: 応答時間 (response time)
  - a_max_accel: 応答時間中に後続車が加速しうる最大加速度
  - b_min: 後続車が「必ずこれだけは出せる」と約束する最小制動力
  - b_max: 先行車が出しうる最大制動力（最悪ケース）

d_min は「両者がこの公式通りに振る舞えば、先行車がどれだけ急ブレーキを
踏んでも後続車が追突しない」ことを保証する最小車間距離である。
d_min未満は「RSS基準では安全とは言えない(not RSS-safe)」ことを意味する
——JAMA C&Cの「リスク知覚境界(0.72m/TTC2.0秒)」に相当する、RSS独自の
「注意すべき距離」の定義である。

デフォルトのパラメータ値(rho=1.0s, a_max_accel=2.0, b_min=4.0, b_max=8.0)
は、RSS論文および後続の解説記事で最も頻繁に例示される値を採用した
(論文自体はこれらを「典型的な乗用車のパラメータ例」として挙げており、
JAMA C&Cモデルの0.774Gのような公式な規制値ではないことに注意)。

このモジュールは「1つのログで試してみる」という12.25節の依頼に応える
ための第一歩であり、以下を簡略化している(将来の課題):
  - ~~横方向のRSS安全距離公式(カットイン・合流シナリオ用)は未実装。~~
    **12.27節で追加実装済み**(下記参照)。
  - b_min/b_maxの値はRSS論文の例示値をそのまま採用しており、AJISAI
    車両の実測制動性能から較正してはいない(JAMA C&Cの0.774Gは公的な
    統計に基づく値である点と対照的)。

---
English:
Section 12.25: a prototype analogous to `jama_cc_model.py`, but using the
RSS (Responsibility-Sensitive Safety, Shalev-Shwartz, Shammah, Shashua,
2017, arXiv:1708.06374) longitudinal minimum safe distance formula as the
"safety model" instead of the JAMA C&C (Competent and Careful) driver
model.

## Where this fits

The verification pipeline has two distinct roles:

  (A) Abstraction granularity (the grid): the near/far grid that keeps the
      "important" region close to Ego fine and the far region coarse.
      Until now this was derived mechanically from the two vehicles'
      physical size (contact boundary) by `auto_grid.py` -- it did NOT
      directly use the JAMA C&C thresholds. This section makes explicit a
      point that was previously implicit: "safety judgment via the C&C
      model" and "choice of grid granularity" were actually two separate
      mechanisms in Sections 12.19-12.24.

  (B) The safety-judgment model: the standard against which we ask
      whether the actual log failed to avoid a situation that a
      competent-and-careful human, or an RSS-compliant agent, could have
      avoided. Until now the JAMA C&C model (`jama_cc_model.py`) played
      this role.

The user's proposal is to swap (B) from JAMA C&C to RSS, and -- by tying
(A)'s grid extent to (B)'s own notion of "where does this safety model
start to care" (the risk-perceived / RSS-violation frame) -- to turn this
into a genuine instance of "safety-model-guided abstraction and
refinement". That tying is what `auto_grid.auto_near_range_from_risk_frame`
does.

## RSS longitudinal minimum safe distance

From the RSS paper (Shalev-Shwartz et al. 2017), Definition 3
(Longitudinal Safe Distance):

    d_min = [v_r*rho + (1/2)*a_max_accel*rho^2
             + (v_r + rho*a_max_accel)^2 / (2*b_min)]
            - v_f^2 / (2*b_max)

  - v_r: speed of the rear car (the one whose safety is being judged)
  - v_f: speed of the front car
  - rho: response time
  - a_max_accel: the rear car's maximum acceleration during the response
    time
  - b_min: the minimum braking the rear car commits to being able to
    apply
  - b_max: the front car's worst-case (maximum) braking capability

d_min is the minimum gap such that, as long as both cars behave according
to this formula, the rear car will not rear-end the front car no matter
how hard the front car brakes. A gap below d_min means the configuration
is "not RSS-safe" -- RSS's own analog of JAMA C&C's "risk-perceived
boundary" (0.72m lateral / TTC 2.0s longitudinal).

The default parameter values (rho=1.0s, a_max_accel=2.0, b_min=4.0,
b_max=8.0) are the values most commonly used as illustrative examples in
the RSS paper and follow-up explainer articles (the paper itself presents
these as "typical passenger car" example values, not an officially
regulated figure the way JAMA C&C's 0.774G is).

This module is a first step toward the Section 12.25 request to "try it
on one log first", and simplifies the following (future work):
  - b_min/b_max are taken as-is from the RSS paper's example values, not
    calibrated against the AJISAI vehicles' actual measured braking
    performance (unlike JAMA C&C's 0.774G, which is based on public
    statistics).

## 12.27節: RSS 横方向最小安全距離公式（追加）

ユーザーからの指摘「RSSの横方向は考慮されていなかったのですね。横方向も
考慮してください」を受けて、RSS論文 Definition 6 / Lemma 4
(Lateral Safe Distance) を追加実装した。

    d_min,lat = mu + [ (v1+v1rho)/2*rho + v1rho^2/(2*a_lat_min_brake)
                        - ( (v2+v2rho)/2*rho - v2rho^2/(2*a_lat_min_brake) ) ]_+

  - v1, v2: 車1・車2の横方向速度（車1は「より小さい座標」側、車2は
    「より大きい座標」側にいるものとし、共通の1次元横方向軸上で正の値は
    互いに近づく向き）
  - v1rho = v1 + rho * a_lat_max_accel, v2rho = v2 - rho * a_lat_max_accel
    （応答時間rhoの間、両者が互いに向かって最大横加速度で近づいた場合の
    速度）
  - a_lat_max_accel: 応答時間中に出しうる最大横加速度
  - a_lat_min_brake: 各車が「必ずこれだけは出せる」と約束する最小横制動力
  - mu: 最終的に確保すべき横方向の余裕(fluctuation margin)
  - [x]_+ = max(x, 0)

これは縦方向公式(Def. 3)と同じ発想——「応答時間中は最悪の加速、その後は
約束した制動力で減速(停止)した場合でも、最終的に余裕muを残せる距離」——を
横方向に、かつ両車に対称に適用したものである(縦方向は「後続車の応答」対
「先行車の最大制動」という非対称な関係だが、横方向はどちらの車が先に
反応すべきかが対称なため、両車それぞれの応答分を差し引く形になる)。

このプロジェクトでは、egoを基準とした相対座標系(`rys`、ego自身の左方向を
正とする)をそのまま使う簡略化を取る: egoは常にこの座標系の原点(ry=0)・
横方向速度0として扱い(egoが自ら大きく横移動するケースは対象外)、NPCの
横方向速度はrysの時間差分（`demo_scenario_snapshot.lateral_speed_at`と
同じ、前後half_winフレームの中心差分）から求める。`ry`の符号から
「NPCがegoの左右どちら側にいるか」を判定し、座標が小さい側を車1・大きい
側を車2として上記の公式に当てはめる（縦方向公式が`rx`の符号で「どちらが
後続車か」を判定するのと同じパターン）。

判定は、縦方向違反(|rx| < d_min,lon)と横方向違反(|ry| < d_min,lat)の
どちらか一方でも成立すればRSS違反とする——JAMA C&Cモデルが横方向境界
(0.72m)とTTC境界(2.0秒)のうち早い方でリスクを知覚するのと同じ構造
(`find_risk_perceived_frame`参照)。

デフォルトのパラメータ値(rho=1.0s、a_lat_max_accel=0.2 m/s^2、
a_lat_min_brake=0.8 m/s^2、mu=0.1m)は、RSS論文自体には具体的な数値
例示がないため、RSSの公開実装であるIntel ad-rss-libの
"Parameter Discussion"ドキュメントが例示している値を採用した
（縦方向のrho/a_max_accel/b_min/b_maxと同様、公式な規制値ではなく
広く引用される例示値という位置づけ）。

---
English: Section 12.27 addition -- the RSS lateral (sideways) minimum
safe distance formula (Definition 6 / Lemma 4 of the RSS paper), added
in response to the user's request to also account for the lateral
direction. See the docstrings of `rss_lateral_min_distance` and
`rss_lateral_min_distance_at` below for the formula and its adaptation to
this project's ego-centered coordinate frame. An RSS violation is now
"longitudinal violation OR lateral violation" (mirroring how JAMA C&C
combines its lateral boundary and TTC boundary via whichever triggers
first). Default lateral parameter values (rho=1.0s,
a_lat_max_accel=0.2 m/s^2, a_lat_min_brake=0.8 m/s^2, mu=0.1m) follow the
Intel ad-rss-lib "Parameter Discussion" documentation's example values,
since the RSS paper itself gives no concrete numeric example for the
lateral parameters.

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.rss_model
"""

import math

from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame,
)
from logverify.reference_model_comparison import ego_speed_series
from logverify.jama_cc_model import _first_persistent_trigger

# RSS (Shalev-Shwartz, Shammah, Shashua 2017, arXiv:1708.06374), commonly
# cited example parameter values -- NOT an official regulatory figure.
RSS_RESPONSE_TIME = 1.0       # s (rho)
RSS_MAX_ACCEL = 2.0           # m/s^2 (a_max,accel)
RSS_MIN_BRAKE = 4.0           # m/s^2 (b_min): what the rear car commits to
RSS_MAX_BRAKE = 8.0           # m/s^2 (b_max): front car's worst-case braking

# RSS lateral (Definition 6 / Lemma 4), Section 12.27. The paper itself
# gives no concrete numeric example for these; these are the illustrative
# example values from Intel's public ad-rss-lib "Parameter Discussion"
# documentation -- NOT an official regulatory figure, same caveat as above.
RSS_LAT_RESPONSE_TIME = 1.0   # s (rho), same response time as longitudinal
RSS_LAT_MAX_ACCEL = 0.2       # m/s^2 (a_lat,max,accel)
RSS_LAT_MIN_BRAKE = 0.8       # m/s^2 (a_lat,min,brake): committed minimum lateral braking
RSS_LAT_MU = 0.1              # m (mu): lateral fluctuation margin

from logverify.paths import LOG_0067 as LOG_PATH  # see logverify/paths.py


def npc_speed_series(data, npc_name="npc1"):
    """NPCの速度(スカラー)の時系列。ego_speed_seriesのNPC版。

    ---
    English: NPC's scalar speed time series -- the NPC counterpart of
    ego_speed_series.
    """
    gk = data["groundtruth_kinematic"]
    out = []
    for rec in gk:
        npc = next((v for v in rec.get("groundtruth_vehicles", []) if v["name"] == npc_name), None)
        if npc is None:
            out.append(None)
            continue
        out.append(math.hypot(npc["twist"]["linear"]["x"], npc["twist"]["linear"]["y"]))
    return out


def rss_longitudinal_min_distance(
    v_r, v_f,
    rho=RSS_RESPONSE_TIME, a_max_accel=RSS_MAX_ACCEL,
    b_min=RSS_MIN_BRAKE, b_max=RSS_MAX_BRAKE,
):
    """RSS縦方向最小安全車間 d_min (Def. 3)。0未満にはクリップする。"""
    d = (v_r * rho + 0.5 * a_max_accel * rho ** 2
         + (v_r + rho * a_max_accel) ** 2 / (2 * b_min)
         - v_f ** 2 / (2 * b_max))
    return max(0.0, d)


def rss_min_distance_at(rx, ego_v, npc_v, **kwargs):
    """そのフレームでのrxの符号から「どちらが後続車か」を判定し、
    RSS縦方向最小安全車間を返す。rxがNoneの場合はNoneを返す。

    ---
    English: decides which car is the rear car from the sign of rx at
    this frame, and returns the RSS longitudinal minimum safe distance.
    Returns None if rx is None.
    """
    if rx is None:
        return None
    if rx >= 0:
        # NPC ahead of ego -> ego is the rear car.
        return rss_longitudinal_min_distance(v_r=ego_v, v_f=npc_v, **kwargs)
    # NPC behind ego -> NPC is the rear car.
    return rss_longitudinal_min_distance(v_r=npc_v, v_f=ego_v, **kwargs)


def _lateral_velocity_series(rys, timestamps, half_win=5):
    """NPCの横方向速度(d(ry)/dt)の時系列。egoの`ego_speed_series`に相当する
    横方向版。`demo_scenario_snapshot.lateral_speed_at`と同じ、前後
    half_winフレームの中心差分。ryがNoneの区間(NPC未検出)はNoneを返す。

    ---
    English: NPC's lateral velocity (d(ry)/dt) time series -- the lateral
    counterpart of ego_speed_series, computed the same way as
    demo_scenario_snapshot.lateral_speed_at (centered finite difference
    over +-half_win frames). Returns None where ry is unavailable.
    """
    n = len(rys)
    out = []
    for i in range(n):
        i0, i1 = max(0, i - half_win), min(n - 1, i + half_win)
        if rys[i0] is None or rys[i1] is None:
            out.append(None)
            continue
        dt = timestamps[i1] - timestamps[i0]
        out.append((rys[i1] - rys[i0]) / dt if dt > 0 else 0.0)
    return out


def rss_lateral_min_distance(
    v1, v2,
    rho=RSS_LAT_RESPONSE_TIME, a_lat_max_accel=RSS_LAT_MAX_ACCEL,
    a_lat_min_brake=RSS_LAT_MIN_BRAKE, mu=RSS_LAT_MU,
):
    """RSS横方向最小安全距離 d_min,lat (Def. 6 / Lemma 4)。

    v1は共通の横方向軸上で座標が小さい側の車の速度、v2は座標が大きい側の
    車の速度(軸に沿った符号付き値、正方向がその軸の正方向)。

    ---
    English: RSS lateral minimum safe distance (Def. 6 / Lemma 4). v1 is
    the signed velocity (along the shared lateral axis) of the car at the
    smaller coordinate, v2 of the car at the larger coordinate.
    """
    v1rho = v1 + rho * a_lat_max_accel
    v2rho = v2 - rho * a_lat_max_accel
    d = mu + max(
        0.0,
        (v1 + v1rho) / 2 * rho + v1rho ** 2 / (2 * a_lat_min_brake)
        - ((v2 + v2rho) / 2 * rho - v2rho ** 2 / (2 * a_lat_min_brake)),
    )
    return d


def rss_lateral_min_distance_at(ry, ry_velocity, **kwargs):
    """そのフレームでのryの符号から「NPCがego(ry=0, v=0)に対して軸のどちら
    側にいるか」を判定し、車1・車2をその座標順に割り当ててRSS横方向最小
    安全距離を返す(縦方向のrss_min_distance_atがrxの符号で後続車/先行車を
    判定するのと同じパターン)。ry/ry_velocityがNoneの場合はNoneを返す。

    ---
    English: decides, from the sign of ry at this frame, which side of ego
    (fixed at ry=0, v=0 in this project's ego-centered frame) the NPC is
    on, assigns car1/car2 by coordinate order, and returns the RSS lateral
    minimum safe distance (mirrors how rss_min_distance_at uses the sign
    of rx to decide rear/front car). Returns None if ry or ry_velocity is
    None.
    """
    if ry is None or ry_velocity is None:
        return None
    if ry >= 0:
        # NPC at the larger coordinate (car2); ego (0, v=0) is car1.
        v1, v2 = 0.0, ry_velocity
    else:
        # NPC at the smaller coordinate (car1); ego (0, v=0) is car2.
        v1, v2 = ry_velocity, 0.0
    return rss_lateral_min_distance(v1, v2, **kwargs)


def rss_verdicts(rxs, rys, timestamps, ego_speed, npc_speed, near_rx=40.0,
                  lat_half_win=5, lon_kwargs=None, lat_kwargs=None):
    """フレームごとの (d_min_lon, d_min_lat, is_violation) のリスト。
    |rx|<near_rxの範囲でのみ判定する(縦方向遠方は無関係なため;
    jama_cc_modelのnear_rxと同じ考え方)。is_violationは縦方向違反
    (|rx|<d_min_lon)と横方向違反(|ry|<d_min_lat)のOR(12.27節)。

    ---
    English: per-frame list of (d_min_lon, d_min_lat, is_violation). Only
    evaluated for |rx| < near_rx (irrelevant far away -- same idea as
    jama_cc_model's near_rx). is_violation is the OR of the longitudinal
    violation (|rx| < d_min_lon) and the lateral violation
    (|ry| < d_min_lat), Section 12.27.
    """
    lon_kwargs = lon_kwargs or {}
    lat_kwargs = lat_kwargs or {}
    lat_velocity = _lateral_velocity_series(rys, timestamps, half_win=lat_half_win)
    out = []
    for rx, ry, ev, nv, ry_v in zip(rxs, rys, ego_speed, npc_speed, lat_velocity):
        if rx is None or nv is None or abs(rx) > near_rx:
            out.append((None, None, False))
            continue
        d_min_lon = rss_min_distance_at(rx, ev, nv, **lon_kwargs)
        lon_violation = abs(rx) < d_min_lon

        d_min_lat = rss_lateral_min_distance_at(ry, ry_v, **lat_kwargs)
        lat_violation = d_min_lat is not None and abs(ry) < d_min_lat

        out.append((d_min_lon, d_min_lat, lon_violation or lat_violation))
    return out


def find_rss_risk_frame(rxs, rys, timestamps, ego_speed, npc_speed, near_rx=40.0,
                         persist_frames=3, lat_half_win=5, lon_kwargs=None, lat_kwargs=None):
    """最初にRSS違反(縦方向 または 横方向)が持続的に発生するフレームを返す。
    jama_cc_model.find_risk_perceived_frameのRSS版。

    ---
    English: returns the first frame at which an RSS violation
    (longitudinal OR lateral) persists. The RSS counterpart of
    jama_cc_model.find_risk_perceived_frame.
    """
    verdicts = rss_verdicts(rxs, rys, timestamps, ego_speed, npc_speed, near_rx=near_rx,
                             lat_half_win=lat_half_win, lon_kwargs=lon_kwargs, lat_kwargs=lat_kwargs)

    def violated(i):
        return verdicts[i][2]

    frame = _first_persistent_trigger(len(verdicts), violated, persist_frames)
    return frame, verdicts


def simulate_rss_reference(gk, rxs, ego_speed, risk_frame,
                            rho=RSS_RESPONSE_TIME, b_min=RSS_MIN_BRAKE):
    """RSSのリスクフレームから、「応答時間rho経過後、b_minで一定減速する」
    というRSS自身が要求する最低限の反応をEgoが取った場合の反実仮想軌道。

    jama_cc_model.simulate_cc_referenceと同じ構造だが、減速プロファイルは
    C&Cの「0.6秒かけて線形立ち上がり」ではなく、RSSの定義どおり
    「応答時間の間は現状維持、その後はb_minで一定」というステップ型。

    ---
    English: counterfactual trajectory for Ego taking the minimal response
    RSS itself demands from the RSS risk frame: hold speed during the
    response time rho, then brake at a constant b_min. Same structure as
    jama_cc_model.simulate_cc_reference, but the deceleration profile is a
    step (RSS's own definition) rather than C&C's 0.6s linear ramp.
    """
    ts = [rec["timestamp"] for rec in gk]
    n = len(gk)
    rx_ref = list(rxs)

    v_ref = ego_speed[risk_frame]
    traveled_actual = 0.0
    traveled_ref = 0.0
    t_risk = ts[risk_frame]
    for i in range(risk_frame, n - 1):
        dt = ts[i + 1] - ts[i]
        if dt <= 0:
            continue
        a_ref = b_min if (ts[i] - t_risk) >= rho else 0.0
        v_ref = max(0.0, v_ref - a_ref * dt)
        traveled_ref += v_ref * dt
        traveled_actual += ego_speed[i] * dt
        if rxs[i + 1] is not None:
            rx_ref[i + 1] = rxs[i + 1] + (traveled_actual - traveled_ref)
    return rx_ref


def run():
    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)

    closest_frame, risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    timestamps = [rec["timestamp"] for rec in gk]
    risk_frame, verdicts = find_rss_risk_frame(rxs, rys, timestamps, ego_speed, npc_speed)

    print(f"RSS違反(縦方向|rx|<d_min,lon または横方向|ry|<d_min,lat)が"
          f"最初に持続的に発生するフレーム: {risk_frame}")
    if risk_frame is not None:
        d_min_lon, d_min_lat, _ = verdicts[risk_frame]
        d_min_lon_s = f"{d_min_lon:.2f}m" if d_min_lon is not None else "n/a"
        d_min_lat_s = f"{d_min_lat:.2f}m" if d_min_lat is not None else "n/a"
        print(f"  (t={gk[risk_frame]['timestamp']-gk[0]['timestamp']:.3f}s, "
              f"rx={rxs[risk_frame]:.2f}m, d_min_lon={d_min_lon_s}, "
              f"ry={rys[risk_frame]:.2f}m, d_min_lat={d_min_lat_s}, "
              f"最近接フレーム{closest_frame}の"
              f"{gk[closest_frame]['timestamp']-gk[risk_frame]['timestamp']:.3f}秒前)")

        rx_ref = simulate_rss_reference(gk, rxs, ego_speed, risk_frame)
        min_risk_ref, min_risk_frame = None, None
        for i in range(risk_frame, min(len(rxs), closest_frame + 30)):
            if rx_ref[i] is None or rys[i] is None:
                continue
            r = max(abs(rx_ref[i]) / (eh_l + nh_l), abs(rys[i]) / (eh_w + nh_w))
            if min_risk_ref is None or r < min_risk_ref:
                min_risk_ref, min_risk_frame = r, i
        preventable = min_risk_ref is not None and min_risk_ref >= 1.0
        print()
        print("RSSモデルによる反実仮想シミュレーション結果:")
        print(f"  最小2Dリスク値(反実仮想) = {min_risk_ref:.4f} (frame {min_risk_frame})"
              if min_risk_ref is not None else "  最小2Dリスク値: 計算不能")
        print(f"  実際の最小2Dリスク値 = {risk:.4f} (衝突: risk<1)")
        print(f"  判定: {'予防可能 (preventable)' if preventable else '予防不可能 (unpreventable)'}")
    else:
        print("  この近距離範囲(near_rx)内でRSS違反は検出されませんでした。")


if __name__ == "__main__":
    run()
