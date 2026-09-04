"""`multi_log_five_abstractions.py`のZ3 membership-check計測を、別プロセスで
実行するための小さなワーカー。標準入力からsequence(JSON配列の[lane,pos]の列)
を受け取り、gcpd.Modelを構築してmembership checkを行い、経過時間とSAT結果を
標準出力にJSONで書き出す。

サブプロセスとして実行することで、大きな箱数のvariantでZ3の計算が長時間化
した場合でも、呼び出し側から`subprocess.run(..., timeout=...)`でOSレベルの
確実なタイムアウト(プロセスkill)をかけられる(シグナルベースのタイムアウトは
z3オブジェクトの後始末中に割り込むと内部状態が壊れうるため、プロセス分離の
方が安全)。

How to run / 実行方法（単体では通常呼ばない。multi_log_five_abstractions.py
から subprocess 経由で呼ばれる想定）:
    echo '[[0,0],[0,1],[0,1]]' | python3 -m logverify._z3_timing_worker
"""

import json
import sys
import time


def main():
    sequence = [tuple(pair) for pair in json.loads(sys.stdin.read())]

    from logverify.multi_log_model import _model_from_sequences, verify_logs_included, MultiLogModel
    from logverify.membership import reset_solver

    reset_solver()
    t0 = time.perf_counter()
    m, box_id_of = _model_from_sequences([sequence], "NPC")
    mlm = MultiLogModel(model=m, box_id_of=box_id_of, sequences=[sequence], gx=0.0, gy=0.0)
    results = verify_logs_included(mlm)
    elapsed = time.perf_counter() - t0
    all_sat = all(r.is_member for r in results)
    print(json.dumps({"elapsed_s": elapsed, "all_sat": all_sat}))


if __name__ == "__main__":
    main()
