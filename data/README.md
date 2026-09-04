# Data directory

This repository does not bundle the AJISAI dataset (JAMA-Traceable ADS
Runtime Log Dataset). Obtain the log JSON files separately from their
distributor and place them here, e.g.:

```
data/TD-NI-AR-SD-N04-CI-0067.json
```

The default data directory is `<repo root>/data/`; override it with the
`SGCPD_DATA_DIR` environment variable if your logs live elsewhere. See
`logverify/paths.py`.
