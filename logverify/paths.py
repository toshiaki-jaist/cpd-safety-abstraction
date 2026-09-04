"""AJISAIログJSONファイルの場所を1箇所にまとめる。

このリポジトリはAJISAIデータセット（JAMA-Traceable ADS Runtime Log
Dataset）そのものは含まない（配布元から別途取得する必要がある）。
各スクリプトはデフォルトで`<リポジトリルート>/data/`以下を見るが、
環境変数`SGCPD_DATA_DIR`でディレクトリを上書きできる。

---
English: Centralizes the location of AJISAI log JSON files.

This repository does not bundle the AJISAI dataset itself (JAMA-
Traceable ADS Runtime Log Dataset) -- it must be obtained separately from
its distributor. Scripts default to looking under `<repo root>/data/`,
overridable via the `SGCPD_DATA_DIR` environment variable.
"""

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = Path(os.environ.get("SGCPD_DATA_DIR", REPO_ROOT / "data"))

# 12.19節以降の分析で一貫して使ってきた単一ログでの実験対象。
# (English) The single-log experiment subject used consistently since
# Section 12.19's analysis.
LOG_0067 = str(DATA_DIR / "TD-NI-AR-SD-N04-CI-0067.json")


def log_path(name: str) -> str:
    """ログのファイル名（拡張子なし可）からフルパスを返す。

    ---
    English: Returns the full path for a log given its filename (the
    .json extension is optional).
    """
    if not name.endswith(".json"):
        name += ".json"
    return str(DATA_DIR / name)
