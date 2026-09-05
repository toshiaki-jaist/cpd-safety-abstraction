# Section 13.3: the two-phase analysis framework

Sections 13.1 and 13.2 are not "first attempt, then replacement" -- per
user direction (2026-09-05), they are two phases of one analysis, meant
to be used together, each answering a different question with a
different abstraction:

## Phase 1: coarse C&C/RSS predicate abstraction + event annotation

**Abstraction**: the plain safety-model-guided predicate abstraction
(Sections 12.25-12.28) -- box boundaries driven only by the safety
model's own onset/contact state. On log 0067: 13 boxes.

**Analysis**: attach the Section 13.1 events (real brake onset, real
deceleration changes, TTC/distance/speed criticality milestones) to this
abstraction as annotations, and ask *where inside which box* each event
falls. This phase's question is diagnostic: does the minimal,
safety-model-only abstraction already expose the timing of independently
important events, or are they buried? (Answer, Section 13.1: on log 0067,
all 12 are buried -- none land on a box boundary.)

**Implementation**: `logverify/event_metric_annotations.py`.

## Phase 2: event-augmented (refined) abstraction

**Abstraction**: the Section 13.2 label, folding the same events into the
label itself as monotonic phase indices (with the far region frozen back
to a single box). On log 0067: 24 boxes (vs. 13 in Phase 1) -- a
deliberate refinement of Phase 1's abstraction, not a different pipeline.

**Analysis**: now that every near-region event is a genuine box boundary
by construction, box-level analysis (purity, box-sequence visualization,
Z3 membership checking, cross-log comparison) can be done directly in
terms of these events, without needing a separate annotation pass.

**Implementation**: `logverify/event_augmented_predicate_abstraction.py`.

## Why keep both phases rather than only using Phase 2

- **Phase 1 is the diagnostic step.** It is what tells you, for a given
  log (or log set), whether Phase 2's extra granularity is actually
  needed -- if Phase 1 already showed every event landing on a boundary,
  refining further would be pointless. Phase 1's "N events buried, M at
  boundaries" count is itself a result worth tracking as the analysis
  scales to more logs.
- **Phase 2 has a real cost** (Section 13.2): more boxes, and a
  reintroduction of project-chosen numeric thresholds (TTC zones,
  distance milestones, brake-onset threshold) that the original
  predicate-abstraction redesign (Section 12.25) specifically avoided.
  Phase 1's minimal abstraction remains the right default for questions
  that only care about the safety model's own verdict (e.g. the
  C&C/RSS-vs-Z3-membership-check pipeline in `docs/multi_log_results.md`);
  Phase 2 is the right tool when the research question specifically needs
  to locate these auxiliary events at box granularity.
- **Comparing the two phases is itself informative**: the box-count ratio
  (13 -> 24 on log 0067, x1.8) quantifies how much of the "real world"
  complexity the minimal safety-model abstraction was hiding, per log.
  Tracking this ratio across a wider log set is a natural next step (see
  Limitations below).

## Status / next steps

Both phases have been run on a single pilot log (0067) only. Extending
both to the same 10-log set used in `docs/multi_log_results.md` (5
collision + 5 non-collision) is the natural next step: it would show
(a) whether Phase 1's "everything buried" finding generalizes, (b)
whether Phase 2's box-count growth ratio (~x1.8 here) is stable or
log-dependent, and (c) whether the enlarged Phase 2 abstraction stays
cheap enough for Z3 membership checking.
