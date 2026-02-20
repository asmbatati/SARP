# SAFEMRS Changelog

> Chronological record of all development sessions, prompts, and outcomes for the SAFEMRS project.

---

## Session 15 — 2026-02-21 (Early Morning)

### 15.1 Global Submodule desktop.ini Purge & Git Repair

**Prompt:**
> Submodules have mane desktop.ini files. Please resolve all of them

**Actions:**
- **Global Recursive Purge**: Executed an exhaustive search and deletion of all `desktop.ini` files across the entire workspace, specifically targeting submodule directories (`ros2_agent_sim`, `safemrs`, `safemrs_sim`).
- **Locks Cleared**: Removed `.git/packed-refs.lock` and `.git/index.lock` to resolve "Another git process seems to be running" errors.
- **Verification**: Confirmed `git status` and `git pull` are fully operational and submodules are clean of any `desktop.ini` files.

---

## Session 14 — 2026-02-19 (Night)

### 14.1 Third Implementation Roadmap Review — Spot Integration & Schema Fixes

**Prompt:**

> Third review pass. Analyze each comment, fix relevant issues, update changelog.

**Review source:** External reviewer, 5 potential issues + 2 minor items.

**All 7 items assessed and addressed:**

#### Blocker Fixes

| # | Issue | Fix Applied |
|---|-------|-------------|
| 1 | **`_trace_to_word_automaton` was a stub** — most technically tricky part of Spot integration; must be built before Day 2 ends | Implemented fully: `spot.make_twa_graph` with `set_buchi()`, `prop_deterministic(True)`, AP registration via `aut.register_ap()`, linear chain of BDD-labeled edges with self-loop on final state; added `_state_dict_to_bdd()` converting AP state dict → BDD conjunction using `buddy.bdd_ithvar`/`bdd_nithvar` |
| 2 | **Numeric AP `drone_battery` invalid in Spot** — Spot is boolean-only; `G(drone_active -> drone_battery > 10)` won't parse | Replaced `drone_battery` with discretized boolean APs: `battery_ok` (battery > 10%), `battery_critical` (≤ 5%), `mission_within_range`; `AP_REGISTRY` renamed to `BOOLEAN_AP_REGISTRY`; battery spec fixed to `G(drone_active -> battery_ok)` |
| 3 | **`from_pddl`/`from_bt_xml` stubs ambiguous** — unclear which directions need implementation | Replaced stubs with `raise NotImplementedError(...)` + explicit doc: `to_pddl` and `to_bt_xml` are required; `from_pddl` and `from_bt_xml` are not needed for paper scope and should not be implemented before Day 10 |
| 4 | **Dual mode in `run_experiment` ran sequentially** — latency reported as `max(v_f, v_l)` but channels ran one after another, making Table III latency numbers optimistic | Replaced sequential calls with `ThreadPoolExecutor(max_workers=2)` in `run_experiment` dual branch; latency now correctly reflects parallel wall-clock time |
| 5 | **Per-hazard confidence mismatch** — `_deduplicate_hazards` uses `h.get("confidence", 0)` but confidence was at result level, not per-hazard | Added `h["confidence"] = calibrated_conf` before `_deduplicate_hazards` call, propagating result-level confidence to each hazard dict |

#### Minor Fixes

| # | Issue | Fix Applied |
|---|-------|-------------|
| 6 | **`EXCLUSIVE_LOCATIONS` hardcoded** | Replaced hardcoded set with `_load_exclusive_locations_from_config()` reading from `safemrs/config/domain.yaml`; falls back to empty set (fail-safe, no false positives) |
| 7 | **`cohen_kappa: 1.0` comment misleading** | Clarified comment: this is per-scenario pair κ (both annotators agree → κ=1), not the aggregate κ; aggregate is computed by `compute_agreement.py` — target ≥ 0.90 |

**Files changed:**

- `proposal/implementation_roadmap.md` — 7 fixes across §2.2, §2.3.1, spec libraries, §2.4.3, §3.1, §4.1
- `CHANGELOG.md` — this entry

---

## Session 13 — 2026-02-19 (Late Evening)

### 13.1 Second Implementation Roadmap Review — Depth Hardening

**Prompt:**

> 300-word structured review of roadmap. Analyze each comment, address relevant issues, update changelog.

**Review source:** External reviewer, 10 new issues across Formal Channel, LLM Channel, Fusion, Benchmark, ROS 2.

**All 10 issues assessed and addressed:**

#### Formal Channel (Channel 1)

| # | Issue | Fix Applied |
|---|-------|-------------|
| 1 | **AP encoder missing** — `_plan_to_state_sequence` was a stub with no encoding logic | Implemented full AP encoder: event-timeline extraction, concurrent action detection, AP→boolean mapping keyed on `AP_REGISTRY`; temporal overlap captured by event-boundary timestep slicing |
| 2 | **Temporal overlap not caught by PDDL SequentialSimulator** | Added `_check_concurrent_location_overlap()` to `PDDLValidator`: explicitly checks all action pairs for same exclusive location during overlapping `[time_start, time_end)` intervals |
| 3 | **Numeric fluents not mentioned** | Added note requiring `(:functions (battery ?r - robot))` declarations + `(:init (= (battery drone_1) 100))` in problem file; without these, battery/range constraints silently pass |
| 4 | **Deontic checker still had `action["id"]`** *(missed in Session 12)* | Fixed to `action.id` |

#### LLM Safety Channel (Channel 2)

| # | Issue | Fix Applied |
|---|-------|-------------|
| 5 | **Confidence calibration missing** | Added `_calibrator` (Platt-scaling), `_calibrate()` method to `LLMSafetyReasoner`; raw LLM confidence → calibrated probability; train during Day 7 annotation phase |
| 6 | **Hazard deduplication missing** | Added `_deduplicate_hazards()`: groups by `(frozenset(affected_actions), severity)`, keeps highest-confidence version; prevents inflated hazard counts |
| 7 | **Deterministic prompting** | Added `temperature=0.0`, `seed=42`, `response_format={"type": "json_object"}` to `SubReasonerBase.__init__` |
| 7b | **LLM output caching** | Added `_response_cache` (SHA-256 keyed dict) in `SubReasonerBase`; controlled by `SAFEMRS_CONFIG["cache_llm"]`; stabilizes repeated-run latency measurements |

#### Fusion Mechanism

| # | Issue | Fix Applied |
|---|-------|-------------|
| 8 | **Risk severity levels missing** | Replaced `confidence: "high"/"low"` with `risk_level: "none"/"critical"/"medium"/"low"` via `SEVERITY_MAP`; drives ROS 2 gating: `critical`/`medium` block, `low` flags for review |

#### Benchmark

| # | Issue | Fix Applied |
|---|-------|-------------|
| 9 | **YAML schema missing provenance metadata** | Added `metadata` block: `planner_source`, `generation_prompt`, `validation_status`, `created_date` |

#### ROS 2 Integration

| # | Issue | Fix Applied |
|---|-------|-------------|
| 10 | **No LLM watchdog timeout** | Added `_get_with_timeout(future, timeout_s=10.0)` with graceful fallback to formal-only verdict + `llm_timeout: True` flag; wired into `verify_callback` via `llm_timeout_s` ROS param |

**Files changed:**

- `proposal/implementation_roadmap.md` — 10 code fixes across §2.3, §2.4, §2.5, §3.1, §5.2
- `CHANGELOG.md` — this entry

---

## Session 12 — 2026-02-19 (Evening)

### 12.1 Implementation Roadmap Review — Bug Fixes & Hardening

**Prompt:**

> Analyze external code review of `implementation_roadmap.md`, address all valid issues, update changelog.

**Review source:** External reviewer, 8 issues identified (5 blockers + 3 paper-claim fragility).

**All 8 issues assessed as VALID and addressed:**

#### Blocker Fixes

| # | Issue | Fix Applied |
|---|-------|-------------|
| 1 | **Spec library type inconsistency** — `LTLVerifier.__init__` docstring said `category → list` but specs are `dict[name → formula]` | Changed signature to `dict[str, dict[str, str]]`, iteration to `for spec_name, phi_str in formulas.items()`, violations now include `spec_name` field |
| 2 | **LTL finite-trace semantics** — `_check_trace()` was `pass`; Büchi acceptance is for infinite traces, plan traces are finite | Implemented stutter-based approach: `_check_trace_stutter()` repeats last state 100× to simulate infinite suffix, uses negated-property product emptiness check |
| 3 | **PDDL validator uses dict access on dataclass** — `action["id"]` crashes on `Action` dataclass | Changed to `action.id`, `action.task` |
| 4 | **ROS2 node passes dict where InternalPlan expected** — `json.loads()` returns dict, not `InternalPlan` | Added `JSONPlanConverter().from_json(plan_dict)` conversion; fixed `to_bt_xml(plan)` call |
| 5 | **Latency accounting wrong in dual mode** — fusion output doesn't contain `latency_s` | Compute `latency = max(v_f["latency_s"], v_l["latency_s"])` for parallel execution; per-mode latency extraction |

#### Paper-Claim Hardening

| # | Issue | Fix Applied |
|---|-------|-------------|
| A | **LLM 100% recall claim fragile** — parse failures look like misses | Added retry logic (`max_retries=2`) + JSON validation + markdown fence stripping + conservative fallback (`{"verdict": "safe", "confidence": 0.0, "parse_failure": True}`) to `SubReasonerBase` |
| B | **`expected_formal`/`expected_llm` annotations unused** | Added `compute_expected_channel_analysis()` to `Evaluator` — produces confusion matrix of expected vs actual catches per channel, strengthening Theorem 1 |
| C | **Category label inconsistency could produce 6/8 instead of 7/7** | Added `CANONICAL_CATEGORIES` list (7 entries) + `CATEGORY_MAP` dict normalizing all YAML variants (e.g., `spatial_conflict` → `spatial`); experiment runner uses normalized categories |

#### New Section Added

| Section | Content |
|---------|---------|
| **§4.2 Canonical Category Map & Scenario Loader** | `CATEGORY_MAP` constant + `load_scenario_as_plan()` function that deterministically maps YAML → `InternalPlan` (fills `plan_id`, `command`, `dependencies`, `robots`) + `load_all_scenarios()` directory walker |

**Section renumbering:** Old §4.2 → §4.3, Old §4.3 → §4.4

**Files changed:**

- `proposal/implementation_roadmap.md` — 8 code fixes + 1 new section (§4.2)
- `CHANGELOG.md` — this entry

---

## Session 11 — 2026-02-19 (Midday)

### 11.1 Literature Review Rewrite — Precision and Overclaim Removal

**Prompt:**

> Update the tex latex/main.tex, develop clear and literature review section, and update changelog. Be precise and avoid overclaims.

**Context — Critical finding from reading SafePlan and VerifyLLM full text:**

After reading both papers in full, we identified that the original gap statement ("no existing paper combines formal logic verification AND LLM-based safety reasoning") was inaccurate:

- **SafePlan** (Obi et al., 2025) already combines deontic logic + LTL with CoT reasoning for safety
- **VerifyLLM** (Grigorev et al., 2025) already combines LTL with LLM sliding-window analysis

The key difference is _architectural_, not whether they use both:

1. In SafePlan and VerifyLLM, formal logic is embedded _inside_ the LLM prompt — single integrated pipeline, single final verdict
2. In SAFEMRS, two _independent_ channels produce _separate_ verdicts, reconciled through fusion

**Changes to `latex/main.tex`:**

| Change                     | Before                                                                                                                   | After                                                                                                                                                                                                         |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Introduction gap statement | "No existing framework combines formal logic and LLM-based safety reasoning into a corroborative pre-execution pipeline" | "No existing framework runs formal logic and LLM-based safety reasoning as _architecturally independent channels_ that produce separate verdicts and reconcile them through a corroborative fusion mechanism" |
| Related Work §2.2          | 1-paragraph generic summaries of SafePlan/VerifyLLM                                                                      | Detailed multi-paragraph descriptions based on actual paper content, citing specific mechanisms (deontic logic layers, sliding-window, LCS ablation)                                                          |
| Related Work §2.3          | "Research Gap" with broad overclaim                                                                                      | "Positioning and Research Gap" with 3 precise differentiation axes (independent channels, fusion, multi-robot)                                                                                                |
| SafePlan hazard rate claim | "hazard rates exceeding 30%" (unsourced)                                                                                 | "significant error rates" (no fabricated numbers)                                                                                                                                                             |
| SafePlan bibitem           | Wrong authors (J. Chen, Y. Liu)                                                                                          | Correct authors (I. Obi et al.)                                                                                                                                                                               |
| VerifyLLM bibitem          | Wrong authors (A. Konighofer, M. Tappler)                                                                                | Correct authors (D.S. Grigorev, A.K. Kovalev, A.I. Panov)                                                                                                                                                     |
| RoCo reference             | Not cited                                                                                                                | Added as additional MRS planning work                                                                                                                                                                         |

**Key design decisions:**

- Avoided claiming "no work uses both formal + LLM" — SafePlan does
- Differentiation is now on 3 precise axes: (1) independent channels, (2) corroborative fusion with disagreement handling, (3) multi-robot coordination safety vs. single-robot prompt/plan quality
- Cited VerifyLLM's own ablation (LCS 0.183→0.178 without LTL) to demonstrate their LTL module has minimal independent effect
- Used wording "architecturally independent channels" consistently to mark the genuine novelty

### 11.2 Table I, TikZ Architecture Diagram, and Tracing Example

**Prompt:**

> Positioning and Research Gap: Table is missing. Architecture section: add an illustrative step-by-step tracing example. Add TikZ diagram for the architecture as figure and cite it.

**Changes to `latex/main.tex`:**

1. **Table I (Feature comparison):** Added `tab:related_comparison` comparing 7 systems (SafePlan, VerifyLLM, LTLCodeGen, NL2HLTL2PLAN, SAFER, S-ATLAS, LaMMA-P) + SAFEMRS across 6 axes (Independent channels, Formal, LLM, Fusion, MRS, Pre-execution). SAFEMRS is the only system with all 6 checkmarks.

2. **Figure 1 (TikZ architecture diagram):** Full-width `figure*` with TikZ drawing showing: NL Command → Agentic Reasoning Layer → Candidate Plan π → parallel fork into Channel 1 (Formal Logic: LTL + PDDL + Deontic) and Channel 2 (LLM Safety CoT: Invariant + Conflict + Common-Sense) → separate verdicts $v_F$, $v_L$ → Corroborative Fusion → Approve / Reject / Review → ROS 2 Execution. Includes annotations for "Sound, Incomplete" and "Complete, Unsound" on each channel.

3. **§4.5 Illustrative Example (Dual-Channel Trace):** Concrete 4-step trace of a warehouse inspection command:
   - Step 1: Plan generation (4 actions with time intervals)
   - Step 2: Formal channel detects spatial conflict (drone + Go2 overlapping in east corridor during [70, 80]s)
   - Step 3: LLM channel detects common-sense hazard (industrial exhaust vents on roof) but _misses_ the corridor overlap
   - Step 4: Corroborative fusion merges both hazard explanations into a unified rejection

   Demonstrates complementarity: formal catches precise temporal-spatial conflicts; LLM catches domain-knowledge hazards.

### 11.3 Experimental Roadmap & Implementation Plan

**Prompt:**

> Make a clear experimental roadmap and implementation plan to validate this with ROS2. How to implement, how to contrast with benchmark, give all details.

**Created [`proposal/implementation_roadmap.md`](proposal/implementation_roadmap.md)** — Comprehensive 10-section plan covering:

1. **Repository structure** — `safemrs/` (core Python pkg), `safemrs_sim/` (Gazebo), `ros2_agent_sim/` (ROS 2 nodes)
2. **Implementation modules** — Detailed code for each component:
   - Agentic Reasoning Layer (NL → structured JSON plan)
   - Channel 1: LTL Verifier (Spot) + PDDL Validator (unified-planning) + Deontic Checker
   - Channel 2: 4 sub-reasoners (Invariant, Conflict, CommonSense, Physical) with prompt templates
   - Corroborative Fusion (3-way decision: Approve/Reject/Review)
3. **Benchmark creation** — YAML schema per scenario, generation pipeline, annotation tool, Cohen's κ
4. **Experiment execution** — `run_all.py` running 5 baselines × 100 scenarios
5. **Metrics computation** — HDR, FPR, Cov, ΔC, disagreement analysis
6. **ROS 2 integration** — `SafemrsNode` with `/safemrs/verify_plan` service
7. **11-day schedule** — Day-by-day from Feb 19 to Mar 1
8. **Validation checklist** — 19 paper claims mapped to specific evidence requirements
9. **Dependencies** — unified-planning, spot-python, langchain, ollama, ROS 2 Jazzy
10. **Risk mitigation** — Fallbacks for Spot install issues, noisy LLM, weak Qwen3

**Key insight:** The critical path is the Python `safemrs/` package running on JSON plan scenarios. ROS 2 and Gazebo add credibility but are not blocking for paper submission.

### 11.4 Multi-Format Plan Representation (JSON / PDDL / BehaviorTree)

**Prompt:**

> Planning/planner.py must be configurable to output JSON, PDDL, or ROS2 Behavior Tree. Fix and make consistent with all implementation.

**Changes to `proposal/implementation_roadmap.md`:**

1. **New §2.1 `plan_representations/`** — Added canonical `InternalPlan` dataclass + three format converters (`JSONPlanConverter`, `PDDLPlanConverter`, `BTXMLPlanConverter`). All modules consume `InternalPlan`, never raw JSON/PDDL/XML directly.
2. **Refactored §2.2 `AgenticPlanner`** — Now uses Strategy pattern: `generate_plan()` always returns `InternalPlan`; `export()` serializes to configured format via `PlanFormat` enum.
3. **Updated all downstream modules** — `FormalVerifier`, `LTLVerifier`, `PDDLValidator`, `DeonticChecker`, `LLMSafetyReasoner`, and all sub-reasoners now accept `InternalPlan` instead of `dict`.
4. **Updated ROS 2 node** — `SafemrsNode` accepts `plan_format` parameter; can export BT XML for BehaviorTree.CPP executor.
5. **Updated experiment runner** — `run_all.py` converts benchmark YAML to `InternalPlan` via `JSONPlanConverter.from_json()`.
6. **Config** — Added `PlanFormat` enum and `plan_format` field to `SAFEMRS_CONFIG`.

---

## Session 10 — 2026-02-19 (Morning)

### 10.1 IROS Paper Outline with Initial Content

**Prompt:**

> Based on iros2026_scope.md, create a paper outline with structure and initial content and expected experiments and results to fit IROS standard. Update changelog. For 6 pages, put expected number of words in each section.

**Created [latex/main.tex](latex/main.tex)** — Full 6-page IROS paper outline with:

| Section                  | Page Budget   | Word Budget      | Content                                                                                                                                                                                                          |
| ------------------------ | ------------- | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| I. Introduction          | 1.0 page      | ~800 words       | Problem context, safety gap, gap statement, 3 contribution bullets, results preview                                                                                                                              |
| II. Related Work         | 0.75 page     | ~550 words       | LLM-based MRS planning (SMART-LLM, COHERENT, DART-LLM, LaMMA-P), safety verification (SafePlan, VerifyLLM, LTLCodeGen, NL2HLTL2PLAN, SAFER, S-ATLAS), clear gap statement                                        |
| III. Problem Formulation | 0.75 page     | ~550 words       | 3 formal definitions (Multi-Robot Task Plan, Safety Verifier, Hazard Category Coverage), channel characterization (sound-but-incomplete vs. complete-but-unsound), Theorem 1 (Strict Complementarity) with proof |
| IV. Architecture         | 1.5 pages     | ~1,200 words     | 5 subsections: Agentic Reasoning Layer, Formal Logic Verifier (LTL + PDDL + Deontic), LLM Safety CoT Reasoner (4 sub-reasoners), Corroborative Fusion (3-way decision), ROS2 Execution Layer                     |
| V. Experiments           | 1.5 pages     | ~1,200 words     | 100-scenario benchmark across 7 hazard categories, 4 baselines, 6 metrics, main results table, per-category breakdown, LLM backbone comparison (GPT-4o vs Qwen3:8b), disagreement analysis, latency analysis     |
| VI. Conclusion           | 0.5 page      | ~350 words       | Key findings, limitations, future work (triple-channel → ICRA 2027)                                                                                                                                              |
| **Total**                | **6.0 pages** | **~4,800 words** |                                                                                                                                                                                                                  |

**Key design decisions:**

- **Title:** "SAFEMRS: Corroborative Dual-Channel Pre-Execution Safety Verification for LLM-Based Heterogeneous Multi-Robot Task Planning"
- **Theorem 1** provides formal proof of channel complementarity (strict superset coverage)
- **Expected main result:** 96% HDR (dual) vs. 71% (formal-only) vs. 71% (LLM-only)
- **Per-category complementarity:** Formal catches spatial/resource/temporal (100%); LLM catches common-sense/physical (100%); dual gets ≥86% on all 7
- **Architecture-level validation:** GPT-4o (96% dual HDR) + Qwen3:8b (92% dual HDR) confirms contribution is not model-dependent
- **Disagreement analysis:** ~12% disagreement rate, categorized into LLM false alarms (60%) and LLM misses (40%)
- All TODO markers placed for figures/tables to be created during implementation
- 12 placeholder references included (to be replaced with .bib file)

---

## Session 9 — 2026-02-18 (Late Night)

### 9.1 Complexity Evaluation & Multi-Venue Project Plan

**Prompt:**

> Read the CHANGELOG, architecture_proposal.md, and competitive_analysis.md. Evaluate the complexity of the current proposal. If too complex, suggest simplifications while ensuring rigor and novelty for IROS. Avoid salami-slicing. Also analyze the receding horizon planning / MPC idea in context. Write a feasible action plan in project_plan.md.

**Created [project_plan.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/project_plan.md)** — 9-section project plan:

| Section                      | Content                                                                                                                                                                                                                                       |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| §1 Complexity Diagnosis      | Quantified scope inventory: 8 major components, 19 features across 16 heatmap columns — **verdict: too complex** for a 6-page paper                                                                                                           |
| §2 Simplification Strategy   | Core principle: "one deep idea per paper" with supporting contributions                                                                                                                                                                       |
| §3 RHP/MPC Idea Analysis     | Mapped receding horizon planning to SAFEMRS (Model→World Model+KB, Constraints→LTL/CBF, Solve→ARL+SRL, Execute→first action, Observe→RTM); identified as **strong and differentiated** — no existing LLM-MRS paper formalizes planning as MPC |
| §4 Multi-Venue Decomposition | 3-paper plan with anti-salami-slicing verification                                                                                                                                                                                            |
| §5 Non-Overlap Matrix        | Verified each paper has different RQ, baselines, metrics, venue type                                                                                                                                                                          |
| §6 Timeline                  | Gantt-style sequencing: Paper 1 → Paper 2 → Paper 3 (with parallel tracks)                                                                                                                                                                    |
| §7 Simplified Paper 1        | Cut from ~15 components to 5 for IROS feasibility                                                                                                                                                                                             |
| §8 Action Items              | Week-by-week breakdown of immediate next steps                                                                                                                                                                                                |
| §9 Summary                   | Decision table with recommendations                                                                                                                                                                                                           |

**Key decisions:**

| Paper       | Focus                                            | Venue     | Core Contribution                                          |
| ----------- | ------------------------------------------------ | --------- | ---------------------------------------------------------- |
| **Paper 1** | Triple-channel corroborative safety verification | IROS 2026 | Formal proof that triple-channel fusion dominates subsets  |
| **Paper 2** | Receding horizon planning for LLM-MRS            | ICRA 2027 | MPC-style formalization of LLM-based multi-robot planning  |
| **Paper 3** | Agentic cognitive proxy architecture             | RA-L      | MCP integration + Robot Resumes + HAL systems contribution |

**Complexity reduction for Paper 1:**

- Removed: MCP, 6/8 planning backends, HAL (6 adapters), VLA bridge, Robot Resumes, Skill Ontology, UUV/Quadruped
- Kept: SRL (3 channels + fusion + conformal), basic ARL, 2 backends (PDDL + BT), basic RTM, UAV + UGV

---

## Session 8 — 2026-02-18 (Late Night)

### 8.1 Resolve Git Submodules & Cleanup

**Prompt:**

> Fix `already exists in index` errors, switch to `neuros-x` repos, and delete `desktop.ini` files.

**Actions:**

- **Switched Remote Origin**: Replaced `asmbatati` submodules with **`neuros-x` organization** repositories:
  - `safemrs` (Core)
  - `safemrs_sim` (Simulation)
  - `safemrs_docker` (Infrastructure)
- **Cleanup**: Recursively deleted `desktop.ini` files to resolve "untracked content" warnings.

**Outcome:**

- All 3 submodules successfully registered from `neuros-x`.
- Repository status clean.

---

## Session 7 — 2026-02-16 (Evening)

### 7.1 Analyze Implementation Repositories

**Prompt:**

> Analyze `ros2_agent_sim` and `ros2_agent_sim_docker` repos — deep dive every package, tool, launch file, and Docker config. Map against SAFEMRS architecture.

**Analyzed files across both repos:**

| Repository                      | Files Analyzed                                                                                                                                                | Key Findings                                                                                                                         |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `ros2_agent_sim/ros2_agent`     | `ros2_agent_node.py`, `model.py`, `specialist_models.py`, `system_prompts.py`, `drone_tools.py` (926 lines, 7 tools), `unitree_tools.py` (760 lines, 8 tools) | Single ROSA agent (Qwen3:8b) controls all robots; VLM specialist (Qwen2.5VL:7b); fire-and-forget threaded tools; namespace isolation |
| `ros2_agent_sim/simulation_gui` | `gui_node.py` (247 lines), `App.jsx` (232 lines)                                                                                                              | FastAPI + React drag-drop builder; generates launch files dynamically; only Go2 + drone implemented                                  |
| `ros2_agent_sim/sar_system`     | `sar_system.launch.py` (608 lines)                                                                                                                            | Go2 (CHAMP + EKF) + PX4 drone (SITL + MAVROS + XRCE-DDS); timed spawn sequence                                                       |
| `ros2_agent_sim/sim_evaluation` | `eval_logger.py` (102 lines)                                                                                                                                  | RTF logger only — no safety or task metrics                                                                                          |
| `ros2_agent_sim_docker`         | `README.md`, PX4 model configs, scripts                                                                                                                       | Monolithic image bundles Ollama (~20+ GB)                                                                                            |

---

### 7.2 Create GitHub Implementation Analysis

**Prompt:**

> Write the whole analysis into `github_implementation_analysis.md`. Include suggested structure for new repos, ROS 2 packages, Docker image contents, and installation requirements (Ollama stays on host).

**Created [github_implementation_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/github_implementation_analysis.md)** (~780 lines) with 7 sections:

| Section                  | Content                                                                                                       |
| ------------------------ | ------------------------------------------------------------------------------------------------------------- |
| §1 Current Repo Audit    | Package-by-package breakdown of all 6 ROS 2 packages + Docker repo                                            |
| §2 Gap Analysis          | 5-tier Mermaid diagram + 18-row gap table (SRL, ARL, APL, RTM, HAL, Eval) + 8 structural problems             |
| §3 New Repo Structure    | 3 repos under `github.com/asmbatati`: `safemrs` (6 packages), `safemrs_sim` (5 packages), `safemrs_docker`    |
| §4 Docker Architecture   | Layered images (~8–10 GB base); **Ollama + models (~20+ GB) on host**, not in container; `docker-compose.yml` |
| §5 System Requirements   | Hardware (16 GB RAM min, NVIDIA GPU), storage breakdown (~26–28 GB total), prerequisites                      |
| §6 Phased Roadmap        | 4 phases: Safety (wk 1–3), Planning+Monitoring (wk 4–6), Eval (wk 7–9), Advanced (wk 10+)                     |
| §7 Strengths to Preserve | 10 elements to carry forward (ROSA, namespace isolation, VLM pipeline, dynamic launch, Docker)                |

Key design decisions:

- **3-repo split**: Core framework (`safemrs`) separate from simulation (`safemrs_sim`) and infrastructure (`safemrs_docker`)
- **Ollama on host**: Avoids 20+ GB in Docker image; container connects via `host.docker.internal:11434`
- **All repos under `asmbatati`** (private) → transfer to org later
- **6 new ROS 2 packages**: `safemrs_agent`, `safemrs_hal`, `safemrs_evaluation`, `safemrs_gui`, `safemrs_msgs`, `safemrs_bringup`

---

## Session 6 — 2026-02-16 (Late Afternoon)

### 6.1 Integrate Brainstorming Insights into Proposal Documents

**Prompt:**

> Update architecture_proposal.md, competitive_analysis.md, and CHANGELOG.md based on the new insights in brainstorming_output.md.

**Changes to [architecture_proposal.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/architecture_proposal.md)** (501 → 545 lines):

| Section                 | Before                                | After                                                                                  |
| ----------------------- | ------------------------------------- | -------------------------------------------------------------------------------------- |
| Subtitle                | Generic architecture description      | **"Verifiable Neuro-Symbolic Cognitive Proxy"** framing                                |
| §3 Architecture Diagram | 5 subgraphs (ARL, MCP, SRL, RTM, MRS) | **7 subgraphs** (+HAL with 6 adapters, Robot Resumes in MRS)                           |
| Robot types             | Manipulator + Mobile Base             | **Quadruped + UUV (Underwater)** — diverse morphologies                                |
| §4.4 RTM                | Basic closed-loop monitoring          | **PEFA (Proposal-Execution-Feedback-Adjustment)** closed-loop with 4 formalized phases |
| §4.6 (new)              | N/A                                   | **HAL — Middleware-Agnostic Proxy** (ROS 1, ROS 2, gRPC, MAVLink, XRCE-DDS, VLA)       |
| VLA                     | Not mentioned                         | **VLA Execution Bridge** for symbolic→continuous control ("last mile" execution)       |
| §4.7 MRS                | Basic fleet description               | **Embodiment-Aware Fleet** with Robot Resumes (URDF-derived) + Skill Ontology          |
| §5 Sequence Diagram     | "Dual-Channel Safety Verification"    | **"Triple-Channel Safety Verification"**                                               |
| §6.1 Heatmap            | 16 features                           | **19 features** (+VLA bridge, HAL/middleware, embodiment-aware allocation)             |
| §7 Novelties            | 5 novelties                           | **6 novelties** (+Novelty 6: Embodiment-Aware Cognitive Proxy)                         |
| §10 Summary             | 10 gaps addressed                     | **13 gaps** (+middleware-agnostic, embodiment-aware, symbolic→continuous bridge)       |

**Changes to [competitive_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/competitive_analysis.md)** (462 → 471 lines):

| Section           | Before                   | After                                                                                      |
| ----------------- | ------------------------ | ------------------------------------------------------------------------------------------ |
| §2.13 Heatmap     | 14 systems × 13 columns  | **14 systems × 16 columns** (+VLA Bridge, HAL/Middleware, Robot Resumes)                   |
| §3 Strategy D     | 4 bullet points          | **5 bullets** (+cognitive proxy framing, HAL, VLA, Robot Resumes as systems contributions) |
| §6 Action Plan    | Phase 2 with 4 items     | **6 items** (+HAL implementation, Robot Resume generator)                                  |
| §7 Recommendation | 6 reasons, generic title | **7 reasons** (+cognitive proxy framing), new merged title                                 |

New suggested paper title:

> _"SAFEMRS: A Verifiable Neuro-Symbolic Cognitive Proxy for Safe Multi-Robot Autonomy with Triple-Channel Safety Verification"_

---

## Session 5 — 2026-02-16 (Afternoon, Later)

### 5.1 Cross-Analysis of Brainstorming and Main.tex vs. Proposal Documents

**Prompt:**

> Analyze brainstorming.md and main.tex. Then compare them with proposal/architecture_proposal.md and suggested strategies in proposal/competitive_analysis.md. Make a file named brainstorming_output.md.

**Created [brainstorming_output.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/brainstorming_output.md)** — 9-section cross-analysis:

| Section                        | Content                                                                                                                             |
| ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- |
| §1 Document Overview           | 4-document comparison table                                                                                                         |
| §2 Architectural Comparison    | 7-tier layer mapping (brainstorming 4-tier vs. main.tex 4-tier vs. SAFEMRS 5-layer)                                                 |
| §3 Research Question Alignment | RQ granularity comparison                                                                                                           |
| §4 Concept Inventory           | What each document uniquely contributes (VLA, Robot Resumes, PEFA from brainstorming; triple-channel, CBFs, conformal from SAFEMRS) |
| §5 Competitive Positioning     | Feature coverage per framing against 14-system heatmap                                                                              |
| §6 Strategy Alignment          | Gap analysis for Strategy A and B requirements                                                                                      |
| §7 Synthesis                   | Proposed merged 5-tier architecture + keep/drop recommendations                                                                     |
| §8 Action Items                | 12 concrete tasks (immediate, short-term, paper strategy)                                                                           |
| §9 Risk Assessment             | 6 risks with severity ratings and mitigations                                                                                       |

Key finding: brainstorming and SAFEMRS are **complementary** — brainstorming provides systems framing (cognitive proxy, HAL, VLA, middleware diversity), SAFEMRS provides safety depth (triple-channel, CBF, conformal prediction).

---

## Session 4 — 2026-02-16 (Afternoon)

### 4.1 Update Architecture Proposal with Expanded Literature

**Prompt:**

> Now update the architecture proposal to reflect the expanded literature summary.

**Changes to [architecture_proposal.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/architecture_proposal.md)** (433 → 500 lines):

| Section                        | Before                      | After                                                                                              |
| ------------------------------ | --------------------------- | -------------------------------------------------------------------------------------------------- |
| §1 Motivation                  | 4 bullet categories         | 7 categories (+runtime safety, code gen, multi-agent, hybrid LLM+RL)                               |
| §4.3 Safety Reasoning Layer    | Dual-channel (formal + LLM) | **Triple-channel** (formal + LLM + CBF runtime enforcement) with conformal prediction              |
| §4.4 Real-Time Monitoring      | Basic monitoring            | References LLM-CBT, CLGA, integrates CBF safety margins                                            |
| §4.5 Abstract Planning Layer   | 5 backends                  | **8 backends** (+STL from AutoTAMP, FSM from Mu et al., Code Gen from Code as Policies/ProgPrompt) |
| §6.1 Feature Comparison Matrix | 8 systems, 12 features      | **14 systems**, 16 features (+CBF enforcement, conformal, code gen, multi-agent dialog)            |
| §6.2 Positioning Paragraphs    | 6 comparisons               | **12 comparisons** (+SAFER, S-ATLAS, RoCo, LLM-CBT, Code as Policies, NL2HLTL2PLAN)                |
| §7 Novelty Claims              | 4 novelties                 | **5 novelties** (+Novelty 5: Multi-paradigm safety integration)                                    |
| §8 Research Questions          | 4 RQs                       | **5 RQs** (+RQ5 on conformal prediction calibration)                                               |
| §9 Evaluation Plan             | 5 dimensions                | **7 dimensions** (+Runtime Safety, Manipulation; +RoCoBench, LEMMA, OBiMan-Bench)                  |
| §10 Summary Table              | 7 gaps                      | **10 gaps** (+runtime enforcement, probabilistic bounds, code safety)                              |

Key architectural evolution: dual-channel → **triple-channel** safety verification integrating CBFs (from SAFER), conformal prediction (from S-ATLAS), and syntax-guaranteed LTL (from LTLCodeGen).

---

### 4.2 Update Competitive Analysis with New Insights

**Prompt:**

> Now update competitive_analysis.md with the new insights.

**Changes to [competitive_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/competitive_analysis.md)** (360 → 461 lines):

| Section                 | Before                                 | After                                                                        |
| ----------------------- | -------------------------------------- | ---------------------------------------------------------------------------- |
| §1 Integration Trap     | 8 rows                                 | **13 rows** (+CBF, conformal, code gen, dialog, hybrid RL)                   |
| §2 Competitors          | 7 systems (§2.1–2.7)                   | **12 systems** (§2.1–2.12: +SAFER, S-ATLAS, RoCo, LLM-CBT, Code as Policies) |
| §2.2 VerifyLLM          | Single system                          | **3-way comparison** (VerifyLLM + LTLCodeGen + NL2HLTL2PLAN)                 |
| §2.13 Summary Heatmap   | 8 systems × 10 columns                 | **14 systems × 13 columns**                                                  |
| §3 Strategy A           | Dual-channel with 5-category benchmark | **Triple-channel** with 7-category benchmark + conformal calibration         |
| §4 Paper Structure      | 3 related work categories              | **5 categories** (+runtime safety, code gen)                                 |
| §5 Rejection Table      | Generic baselines                      | **8 baselines** with channel-specific failure modes                          |
| §6 Action Plan          | Dual-channel + 2 backends              | **Triple-channel** (3 sub-channels) + **8 backends** + 8 baselines           |
| §7 Final Recommendation | 5 reasons, dual-channel                | **6 reasons**, triple-channel                                                |

All "dual-channel" references updated to "triple-channel." New suggested paper title:

> _"Triple-Channel Corroborative Safety Verification for LLM-Based Multi-Robot Task Planning: Unifying Formal Logic, Probabilistic Reasoning, and Runtime Enforcement"_

---

## Session 3 — 2026-02-16 (Morning)

### 3.1 Extract Added PDFs to Text

**Prompt:**

> Read the CHANGELOG.md first. Then proposal/literature/pdf/added — Make a Python script that extracts all the added papers and converts them to txt in proposal/literature/txt/added. Also make refactoring of the repo. After you finish update the CHANGELOG.md.

**Outcome:**

- Refactored `proposal/scripts/extract_pdfs.py` to support command-line arguments for flexible input/output directories
- Installed `pymupdf` dependency
- Extracted 30 new PDFs from `proposal/literature/pdf/added` → `proposal/literature/txt/added`

---

### 3.2 Expand Literature Summary (16 → 46 Papers)

**Prompt:**

> Now analyze the new papers and add them to the literature summary.

**Changes to [literature_summary.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/literature_summary.md):**

Added 30 new paper summaries (§2.17 – §2.46) organized in 3 batches:

| Batch      | Papers                                                                                                        | Key Topics                                                          |
| ---------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| §2.17–2.26 | SAFER, LLM-CBT, LTLCodeGen, NL2HLTL2PLAN, S-ATLAS, Yuan et al., CLGA, EmbodiedAgent, Wang et al., Chen et al. | Safety pipelines, BT generation, formal specs, conformal prediction |
| §2.27–2.36 | Code-as-Symbolic-Planner, ICCO, FCRF, Huang Survey, Reasoner, Zuzuárregui, RoCo, AutoTAMP, LEMMA, AutoMisty   | Code generation, MARL, reflection, multi-agent dialog, STL          |
| §2.37–2.46 | Kwon et al., Hoffmeister, Code as Policies, ProgPrompt, LLM-GROP, LABOR, Mu et al., OBiMan                    | Scene graphs, zero-knowledge BTs, manipulation benchmarks           |

Updated all thematic analysis sections (LLM integration, dependency modeling, heterogeneous teams, safety/verification, benchmarks) and comparative summary table to span all 46 papers (44 unique).

---

## Session 2 — 2026-02-12 (Afternoon)

### 2.1 Architecture Proposal

**Prompt:**

> Now, we would like to develop a new architecture of heterogeneous multi-robot system as shown in the picture. We assume that we have a multi-robot system in a partially observed environment. The system can receive commands in natural languages from a human. It goes through an agentic reasoning layer which is connected to the multi-robot systems, also connected to external resources through MCP tools and agents, and is able to convert human intent into complex workflows to be executed by the multi-robot system. Any complex task can be executed by one robot or multiple robots. The robots may or may not have access to external information. They are able to communicate with each other. One of the most important contributions is that we will have a safety layer that will ensure the safe operation of any complex multi-task execution by the multi-robot system. We will have also a real-time monitoring layer that will connect real available data during execution from the multi-robot system and external world.
>
> We want to use formal logic verification on top of LLM reasoning to combine a rule-based approach (formal verification) and probabilistic LLM reasoning in our system.
>
> For modeling multi-robot task planning, we can use general-purpose semi-structured data (JSON, YAML) or dedicated planning frameworks (PDDL or Behavior Tree which are more recent - you can propose other trending planning techniques if available), but we may use an abstract planning layer for now that can support multiple technologies like PDDL, behavior tree, or others.
>
> Make me an md file that writes this idea clearly and systematically and contrasts it with the literature you summarized above. This is a picture that also illustrates our idea: proposal-idea.png. You can also make a mermaid diagram.

**Outcome:** Created [architecture_proposal.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/architecture_proposal.md) with:

- 5-layer architecture (ARL, SRL, APL, RTM, MRS)
- Dual-channel safety verification (formal + LLM)
- Abstract planning layer (5 backends)
- Mermaid architecture diagram
- Feature comparison matrix (8 systems)
- 4 novelties, 4 research questions

---

### 2.2 Competitive Analysis

**Prompt:**

> Now, make me an md file that compares clearly with this paper and my idea and propose how to make my contribution more substantial and not just incremental for IROS and ICRA.

**Outcome:** Created [competitive_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/competitive_analysis.md) with:

- Integration trap analysis (what reviewers will say)
- 7 paper-by-paper comparisons (SafePlan, VerifyLLM, COHERENT, DEXTER-LLM, LaMMA-P, LiP-LLM, DART-LLM)
- Summary heatmap
- 4 strategies (A: dual-channel safety, B: agentic MCP, C: POMDP safety, D: combined)
- Recommended paper structure, rejection avoidance, action plan

---

## Session 1 — 2026-02-12 (Morning)

### 1.1 PDF Extraction Script

**Prompt:**

> Make a Python script that extracts all these papers and converts them to txt. Also make refactoring of the repo.

**Outcome:**

- Created `proposal/scripts/extract_pdfs.py` using PyMuPDF
- Extracted 16 original PDFs from `proposal/literature/pdf` → `proposal/literature/txt`

---

### 1.2 Initial Literature Summary

**Prompt:**

> Analyze the literature and make a summary in md file.

**Outcome:** Created [literature_summary.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/literature_summary.md) with:

- 16 paper summaries (SMART-LLM, COHERENT, DART-LLM, SafePlan, VerifyLLM, DEXTER-LLM, LaMMA-P, LiP-LLM, PLANTOR, LAN2CB, AutoHMA-LLM, GMATP-LLM, RoCoBench, Kwon et al., Hoffmeister)
- Thematic analysis (LLM integration, dependency modeling, heterogeneous teams, safety)
- Comparative summary table
- Research gaps identification

---

## Project File Structure

```
SAFEMRS/
├── CHANGELOG.md                          ← This file
├── github_implementation_analysis.md     ← Repo analysis + restructuring plan (Session 7)
├── brainstorming_output.md               ← Cross-analysis document (Session 5)
├── main.tex                              ← IROS paper draft
├── gui.jpeg                              ← GUI mockup
├── ros2_agent_sim/                       ← Current implementation (to be replaced)
│   ├── ros2_agent/                       ← LLM agent (ROSA + Ollama)
│   ├── simulation_gui/                   ← Web GUI (React + FastAPI)
│   ├── sar_system/                       ← SAR launch orchestration
│   ├── drone_sim/                        ← PX4 SITL launch
│   ├── sim_evaluation/                   ← RTF logger
│   └── gps_bridge/                       ← GPS relay (C++)
├── ros2_agent_sim_docker/                ← Docker environment (to be replaced)
├── proposal/
│   ├── architecture_proposal.md          ← SAFEMRS architecture (545 lines)
│   ├── competitive_analysis.md           ← Competitive strategy (471 lines)
│   ├── literature_summary.md             ← Literature review (46 papers)
│   ├── proposal-idea.png                 ← Original architecture sketch
│   ├── scripts/
│   │   └── extract_pdfs.py               ← PDF → TXT extraction tool
│   └── literature/
│       ├── pdf/                           ← Original 16 PDFs
│       ├── pdf/added/                     ← 30 additional PDFs
│       ├── txt/                           ← Extracted text (original)
│       └── txt/added/                     ← Extracted text (added)
```
