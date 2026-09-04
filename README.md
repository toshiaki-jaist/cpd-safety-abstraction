# CPD Safety-Model-Guided Abstraction

This repository implements and evaluates **safety-model-guided abstraction
and refinement** for Car Position Diagram (CPD) verification: instead of
partitioning a driving log into boxes using an arbitrary metric grid, the
abstraction boundaries are derived from the state of a formal safety
model (JAMA's Competent-and-Careful driver model, or RSS) itself.

It is a focused extract, prepared for a paper on this method, from a
larger research codebase (`sgcpd`) that also explores other, unrelated
CPD-construction approaches — this repository intentionally contains
**only** the code for the safety-model-guided abstraction method: the
two safety models, the predicate-abstraction construction, the metric-grid
comparison baselines, and the visualization/evaluation scripts used to
compare them.

See [`docs/method.md`](docs/method.md) for the full method description,
the comparison methodology (purity/smear as an "did the abstraction erase
something important" metric), and the single-log pilot results, and
[`docs/multi_log_results.md`](docs/multi_log_results.md) for the 10-log
(5 collision + 5 non-collision) reproduction and its updated conclusions.

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Data

This repository does not bundle the AJISAI dataset (JAMA-Traceable ADS
Runtime Log Dataset) used in the pilot evaluation. Obtain log JSON files
separately and place them under `data/` (see `data/README.md`), or point
the `SGCPD_DATA_DIR` environment variable at wherever they live.

## Usage

Run from the repository root (`python3 -m logverify.<script>`):

```bash
# Predicate abstraction for both safety models on the pilot log
python3 -m logverify.safety_predicate_abstraction

# Metric-grid comparison baselines
python3 -m logverify.compare_safety_model_abstractions

# The 5-variant comparison used in docs/method.md:
python3 -m logverify.visualize_five_abstractions       # box-sequence figures
python3 -m logverify.plot_five_abstractions_summary    # box-count chart
python3 -m logverify.plot_five_abstractions_purity     # purity/smear chart

# 10-log (5 collision + 5 non-collision) reproduction:
python3 -m logverify.multi_log_five_abstractions       # collects out_gif/multi_log_five_abstractions/results.csv
python3 -m logverify.plot_multi_log_five_abstractions  # aggregate charts + summary table
```

Figures are written to `out_gif/` (git-ignored; regenerate as needed).

## Repository layout

```
gcpd.py                    Core CPD (Car Position Diagram) model
logverify/
  paths.py                 AJISAI log file path resolution (SGCPD_DATA_DIR)
  jama_cc_model.py          JAMA C&C driver/safety model
  rss_model.py               RSS (longitudinal) safety model
  safety_predicate_abstraction.py   Predicate abstraction (the core method)
  auto_grid.py                Metric-grid parameter derivation / grid search
  grid_bridge.py               Frame-sequence -> grid box-state compression
  multi_log_model.py, membership.py   CPD model / Z3 membership-check infra
  compare_safety_model_abstractions.py   Metric-grid variant comparison
  visualize_five_abstractions.py         5-variant box-sequence figures (single-log pilot)
  plot_five_abstractions_summary.py      Box-count comparison chart (single-log pilot)
  plot_five_abstractions_purity.py       Purity/smear comparison chart (single-log pilot)
  multi_log_five_abstractions.py         10-log 5-variant comparison (box count, purity, Z3 cost)
  plot_multi_log_five_abstractions.py    Aggregate charts + summary table for the 10-log run
  _z3_timing_worker.py                   Subprocess worker used to time/timeout each Z3 membership check
  scenario_snapshot_diagram.py, model_diagram.py   Plotting primitives
  reference_model_comparison.py, synth_thresholds_multilog.py   Supporting analysis
  batch_jama_cc_analysis.py, demo_jama_cc_snapshot.py, demo_scenario_snapshot.py   Demos/batch scripts
  abstract_cause.py, abstract_cause_diagram.py, compute_ratios_standalone.py   Supporting analysis
docs/
  method.md                 Full method description and pilot results
data/                        AJISAI log files go here (not bundled)
```

## Status

Single-log pilot evaluation (AJISAI `TD-NI-AR-SD-N04-CI-0067`); see
`docs/method.md` section 6 for current limitations and next steps
(multi-log reproduction, RSS lateral formula, scalability measurement).
