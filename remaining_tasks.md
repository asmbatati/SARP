# SAFEMRS — Remaining Tasks for Paper Finalization
## IROS 2026 Submission Deadline: March 2, 2026

> **Current status:** Experiments complete (102 scenarios, Qwen3:8b + GPT-4o). LaTeX tables filled. 51/51 tests passing. Zero `\unvalidated{}` remaining.
>
> **Remaining work:** Simulation figures, additional LLM comparisons, baseline comparisons, Gazebo world, paper polish, and submission.

---

## Author Roster

| # | Author | Role | Affiliation |
|---|--------|------|-------------|
| **A1** | Abdulrahman S. Al-Batati | Lead — system + experiments | PSU Robotics & IoT Lab |
| **A2** | Anis Koubaa | paper narrative + review | Alfaisal University |
| **A3** | Khaled Gabr | Simulation + Gazebo | PSU |
| **A4** | Serry Sibaee | Evaluation + figures | PSU |
| **A5** | Mohamed Abdelkader | LLM backends + experiments | PSU |
| **A6** | Yasser Alhabashi | Benchmark + testing | PSU |
| **A7** | Fatimah Alahmed | Literature + related work | PSU |
| **A8** | Imen Jarraya | LaTeX + references | PSU |
| **A9** | Wadii Boulila | Safety analysis + writing | PSU |

---

## Task Priority Legend

| Symbol | Meaning |
|--------|---------|
| 🔴 **P0** | Paper-blocking — cannot submit without this |
| 🟠 **P1** | Critical for paper quality / reviewer expectations |
| 🟡 **P2** | Important for completeness |
| 🟢 **P3** | Nice-to-have / future work |

---

## Section 1 — Additional LLM Backend Comparisons

### T1.1 — Claude 3.5 Haiku Comparison 🟠 P1
**Assigned to:** A1 (system), A5 (execution)
**Effort:** ~4 hours (experiment) + 2 hours (analysis)

**Why:** IROS reviewers will ask "is this model-dependent?" Two backends (Qwen3:8b + GPT-4o) may not be sufficient. Claude adds a third data point from Anthropic's model family, demonstrating architecture-level generalization.

**Steps:**
```bash
# Set API key
export ANTHROPIC_API_KEY=sk-ant-...

# Add Claude backend support to get_llm() in base_reasoner.py
# (add elif backend.startswith("claude"): from langchain_anthropic import ChatAnthropic)

# Install dependency
pip install langchain-anthropic

# Run experiment
cd safemrs/
PYTHONUNBUFFERED=1 PYTHONPATH=. nohup python3 experiments/run_llm_background.py \
    --backend claude-3-5-haiku-20241022 \
    --results-dir results/final > results/claude_experiment.log 2>&1 &

# After completion, copy formal results (deterministic, identical to qwen/gpt)
cp results/final/formal_only_qwen3:8b.csv results/final/formal_only_claude-3-5-haiku.csv

# Analyze
PYTHONPATH=. python3 experiments/analyze_results.py \
    --results-dir results/final --backend claude-3-5-haiku-20241022
```

**Expected deliverable:** Fill row 3 of Table V in `latex/main.tex`:
```
Claude-3.5-Haiku (cloud) & ??\% & ??\% & ?\% & ??s \\
```

**Files to update:** `latex/main.tex` (Table V + narrative), `results/final/README.md`, `CHANGELOG.md`

---

### T1.2 — Llama 3.1:8b Local Comparison 🟡 P2
**Assigned to:** A5
**Effort:** ~3 hours

**Why:** Demonstrates that SAFEMRS works with open-source models beyond Qwen — important for reproducibility and accessibility claims.

```bash
ollama pull llama3.1:8b
PYTHONPATH=. python3 experiments/run_llm_background.py \
    --backend llama3.1:8b --results-dir results/final > results/llama_experiment.log 2>&1 &
```

**Note:** Optional — include only if results are interesting (e.g., significantly different HDR/FPR pattern).

---

## Section 2 — Comparison with Baselines

### T2.1 — VerifyLLM Baseline Reproduction 🔴 P0
**Assigned to:** A1, A6
**Effort:** ~6 hours

**Why:** The paper positions SAFEMRS against VerifyLLM (`\cite{2025_VerifyLLMLLMBased_grigorev}`) and SafePlan. IROS reviewers will expect at least one quantitative comparison on the same benchmark. Currently Table III only shows our own three modes (formal-only, LLM-only, dual). We need a "No Verification" row with real numbers and ideally a faithful VerifyLLM-style reimplementation.

**Steps:**
1. Implement a `VerifyLLM-style` baseline: single LTL-check pipeline (LLM generates LTL → syntax check only, no independent formal verification)
2. Implement a `SafePlan-style` baseline: single LLM safety reasoner (same CoT prompts, but LLM output is the final verdict — no formal channel)
3. Both are already partially captured as "Formal-only" and "LLM-only" rows in Table III — verify their naming matches the paper narrative

**Current Table III rows and what they map to:**
| Row | Baseline | Status |
|-----|----------|--------|
| No Verification | COHERENT-style | ❌ Missing — needs HDR=0%, FPR=0% row (trivially: approve all) |
| Formal-Only | VerifyLLM-style | ✅ Done — 77.4% HDR, 10.2% FPR |
| LLM-Only | SafePlan-style | ✅ Done — 83.0% HDR, 4.1% FPR |
| PDDL-Only | LaMMA-P-style | ❌ Missing — formal channel PDDL-only (no LTL) |
| **SAFEMRS (Dual)** | **Ours** | ✅ Done — 64.2% HDR, 0.0% FPR |

**Action:** Add "No Verification" row (trivial: approve all → HDR=0%, FPR=0%) and "PDDL-Only" row to Table III. Run PDDL-only mode experiment.

```bash
# PDDL-only: run formal channel with only PDDL validator active (disable LTL checker)
PYTHONPATH=. python3 experiments/run_all.py \
    --modes pddl_only --results-dir results/final
```

**Files:** `safemrs/experiments/run_all.py` (add pddl_only mode), `latex/main.tex` (add rows)

---

### T2.2 — Ablation Study Table 🟠 P1
**Assigned to:** A1, A4
**Effort:** ~3 hours

**Why:** Reviewers will ask "what happens if you remove one channel?" The per-category HDR data (Table IV) already shows this — but an explicit ablation summary in the text would strengthen the contribution.

**Deliverable:** Add a paragraph to §5.4 (Disagreement Analysis) explicitly stating:
- "Removing the formal channel (LLM-only): FPR rises to 4.1%, commonsense coverage maintained"
- "Removing the LLM channel (formal-only): commonsense HDR drops to 0%, FPR=10.2%"
- "Dual channel: FPR=0.0% (Qwen3:8b) / 2.0% (GPT-4o), EffCov=87.7% / 96.1%"

---

## Section 3 — Gazebo Simulation

### T3.1 — Inspection World SDF 🔴 P0
**Assigned to:** A3
**Effort:** ~8 hours

**Why:** The paper describes a "UAV + UGV building inspection" scenario but the current simulation uses the generic PX4 default world. For Figure 1 (system overview) and the video, a dedicated inspection world is needed.

**Requirements:**
- A multi-floor building structure with:
  - Narrow corridors (triggers spatial conflict hazard)
  - Rooms of different sizes
  - Roof access area (drone inspection zone)
  - Ground floor (Go2 patrol zone)
- Outdoor area for drone approach/departure
- World file: `neuros-demos/sar/worlds/sar_inspection.sdf` ✅ Created

**Reference worlds to adapt:**
- PX4 `warehouse.sdf` (has indoor structure)
- Gazebo Fuel: `construction_cone`, `person_standing` models for realism

**Launch integration:**
```bash
# Launch with inspection world
ros2 launch neuros_demos sar_system.launch.py \
  world:=sar_inspection.sdf
```

---

### T3.2 — Simulation Verification End-to-End 🔴 P0
**Assigned to:** A1, A3
**Effort:** ~4 hours

**Why:** Section IV of the paper describes the ROS 2 integration but does not include empirical data from a live simulation run. The safety gate must be demonstrated to work in the real simulation.

**Verification checklist:**
- [ ] `ros2 launch neuros_demos sar_system.launch.py` — both robots spawn correctly
- [ ] `ros2 launch neuros_demos sar_system.launch.py world:=sar_inspection.sdf` — inspection world variant
- [ ] `ros2 run neuros_agent neuros_agent_node --ros-args -p llm_model:=qwen3:8b -p safety_mode:=dual` — SAFEMRS gate initializes
- [ ] Safe command approved: `"Drone takeoff to 5 meters"` → Approve + drone lifts off
- [ ] Unsafe command blocked: `"Move drone to position 0 -2 1"` → Review/Reject (spatial conflict with Go2)
- [ ] Passthrough mode: same unsafe command executes without blocking
- [ ] Measure actual `check_action()` latency in the ROS 2 environment (log timing)
- [ ] Safety gate latency: formal-only ≤ 50ms, dual ≤ 100ms + LLM time

**Deliverable:** Latency measurements added to §5.7 (Latency Analysis) as ROS 2 integration numbers.

---

### T3.3 — Gazebo Screenshots for Paper 🟠 P1
**Assigned to:** A3, A4
**Effort:** ~2 hours (after T3.1)

**Required screenshots:**
1. **Figure 1 candidate:** Gazebo view of Go2 + drone in inspection world with SAFEMRS safety gate blocking an unsafe command (show terminal with rejection message)
2. **RViz view:** TF tree / robot poses showing both robots active simultaneously
3. **Camera feeds:** Drone gimbal camera + Go2 ground camera side-by-side

**Specifications:**
- Resolution: ≥ 1920×1080
- Format: PNG (for LaTeX inclusion)
- File location: `latex/figures/`

---

## Section 4 — Experimental Figures

### T4.1 — Figure: Per-Category HDR Bar Chart 🔴 P0
**Assigned to:** A4
**Effort:** ~3 hours

**Why:** Table IV (per-category HDR) contains critical information about channel complementarity, but a bar chart makes this visually immediate for reviewers. This is the paper's "killer figure."

**Required plot:**
- Grouped bar chart: 7 categories × 3 systems (Formal-Only, LLM-Only, SAFEMRS Dual)
- Y-axis: HDR (0–100%)
- Highlight: Formal=100% on structural categories, LLM=100% on physical/battery, Dual=0% on commonsense (annotation: "→ Review")
- Style: IEEE-compatible, single-column width (3.5 inches)

```python
# Script location: experiments/generate_figures.py
# Use: matplotlib + seaborn, IEEE color palette
# Output: latex/figures/fig_per_category_hdr.pdf
```

---

### T4.2 — Figure: System Architecture Diagram 🔴 P0
**Assigned to:** A4, A8
**Effort:** ~4 hours

**Why:** The paper currently has Figure 1 as a placeholder. A clean system architecture figure is mandatory for IROS.

**Required diagram:**
- Show: NL command → SAFEMRS dual-channel → Formal Channel (LTL+PDDL) + LLM Channel (4 sub-reasoners) → Corroborative Fusion → Approve/Reject/Review → ROS 2 robot execution
- Style: IEEE TIE style, two columns wide if needed
- Tools: draw.io (export PDF) or matplotlib patches
- File: `latex/figures/fig_architecture.pdf`

**Key elements to show:**
- Channel independence (no shared state between channels)
- Fusion logic (AND-approve, AND-reject, XOR-review)
- ROSA agent integration
- Human-in-the-loop for Review decisions

---

### T4.3 — Figure: Review Rate vs. Hazard Category 🟠 P1
**Assigned to:** A4
**Effort:** ~2 hours

**Why:** The disagreement analysis (§5.6) discusses per-category review rates but currently only shows text numbers. A visualization showing which categories trigger the most human review is insightful.

**Required plot:**
- Horizontal bar chart: 7 categories sorted by review rate
- Annotation: "commonsense=43% (formal always Safe)" + "ordering=36%"
- Secondary annotation: EffCov contribution (how many of these reviews are unsafe)

---

### T4.4 — Figure: Effective Coverage vs. FPR Tradeoff 🟡 P2
**Assigned to:** A4
**Effort:** ~2 hours

**Why:** Shows the Pareto improvement of dual-channel over single-channel: lower FPR AND higher effective coverage.

**Required plot:**
- Scatter plot: FPR (x-axis) vs. EffCov (y-axis)
- Points: Formal-Only, LLM-Only (Qwen), Dual (Qwen), LLM-Only (GPT-4o), Dual (GPT-4o)
- Ideal point: top-left corner (0% FPR, 100% EffCov)
- Arrow showing movement from single-channel → dual-channel

---

## Section 5 — Paper Polish

### T5.1 — References .bib File Completion 🔴 P0
**Assigned to:** A7, A8
**Effort:** ~4 hours

**Context:** The user has switched `main.tex` from inline `\begin{thebibliography}` to `\bibliography{references.bib}`. The `references.bib` file must be populated with all cited works using the new citation keys (format: `YEAR_TitleWords_firstauthor`).

**Required entries (cite keys already used in main.tex):**

| Cite Key | Paper | Action |
|----------|-------|--------|
| `2019_CooperativeHeterogeneous_rizk` | Rizk et al., ACM Computing Surveys 2019 | Add to .bib |
| `2024_SMARTLLMSmart_kannan` | SMART-LLM, IROS 2024 | Add to .bib |
| `2025_COHERENTCollaboration_liu` | COHERENT, ICRA 2024/2025 | Add to .bib |
| `2025_DARTLLMDependencyAware_wang` | DART-LLM, IROS 2024 | Add to .bib |
| `2025_SafePlanLeveraging_obi` | SafePlan, arXiv:2503.06892 | Add to .bib |
| `2025_VerifyLLMLLMBased_grigorev` | VerifyLLM, arXiv:2507.05118 | Add to .bib |
| `2025_LTLCodeGenCode_rabiei` | LTLCodeGen | Add to .bib |
| `2025_Nl2Hltl2PlanScaling_xu` | NL2HLTL2PLAN | Add to .bib |
| `2025_SafetyAware_khan` | SAFER (CBF) | Add to .bib |
| `2025_ProbabilisticallyCorrect_wanga` | S-ATLAS (conformal) | Add to .bib |
| `2025_LaMMAPGeneralizable_zhang` | LaMMA-P | Add to .bib |
| `2024_RoCoDialectic_mandia` | RoCo, ICRA 2024 | Add to .bib |
| `2016_Spot20_duret-lutz` | Spot 2.0, ATVA 2016 | Add to .bib |

**Verification:**
```bash
cd latex/
pdflatex main.tex && bibtex main && pdflatex main.tex && pdflatex main.tex
# Check: no "undefined reference" warnings
```

---

### T5.2 — LaTeX Compilation & Page Count Check 🔴 P0
**Assigned to:** A8
**Effort:** ~2 hours

**Requirements:**
- Paper must be **exactly 6 pages** (IROS 2026 limit)
- No LaTeX compilation errors or warnings
- All figures properly placed (no overflow)
- All tables fit within column width (resizebox applied ✅)
- References section does not overflow page 6

**Checklist:**
- [ ] `pdflatex main.tex` — zero errors
- [ ] `bibtex main` — zero undefined references
- [ ] Count pages: `pdfinfo main.pdf | grep Pages`
- [ ] Check figure quality at 300 DPI (IEEE requirement)
- [ ] Verify author names match — updated to `Abdulrahman S. Al-Batati` (not Abdullah)
- [ ] `\balance` macro active for equal-length final columns ✅

---

### T5.3 — Abstract & Conclusion Review 🟠 P1
**Assigned to:** A2
**Effort:** ~2 hours

**Check the following:**
- Abstract states the most important numbers: 0.0% hard FPR (Qwen3:8b), 87.7% EffCov, 23.5% review rate, 102-scenario benchmark
- Conclusion bullet 4 now correctly says GPT-4o results are confirmed ✅
- Limitations section acknowledges simulation-only evaluation (no real robot)
- Future work mentions triple-channel (CBF) extension for ICRA 2027

---

### T5.4 — Section V Narrative Consistency Pass 🟠 P1
**Assigned to:** A2, A9
**Effort:** ~3 hours

**Items to verify:**
- [ ] §5.1: Formal channel narrative says "sound but incomplete" — matches results (100% spatial HDR, 0% commonsense)
- [ ] §5.2: LLM channel narrative says "broadly complete but unsound" — matches (71.4% commonsense, 4.1% FPR)
- [ ] §5.3: Fusion section explains why dual hard-HDR < both single channels (AND-semantics require agreement)
- [ ] §5.4: Table III description includes EffCov=87.7% explicitly
- [ ] §5.5: Table V narrative correctly states GPT-4o EffCov=96.1%, Review=20.6% ✅
- [ ] §5.6: Effective ΔC=32% statistic is cited correctly
- [ ] §5.7: Latency narrative covers both Qwen3:8b (69.3s) and GPT-4o (5.2s) ✅

---

### T5.5 — Figure Placement & Captions 🟡 P2
**Assigned to:** A8
**Effort:** ~2 hours (after T4.1–T4.3 deliver figures)

- Place `fig_architecture.pdf` as Figure 1 (top of paper, two-column)
- Place `fig_per_category_hdr.pdf` as Figure 2 (§5.4, single-column)
- Place Gazebo screenshot as Figure 3 (§4, demonstrating ROS 2 integration)
- Write IEEE-style captions for all figures
- Ensure figures do not push text onto a 7th page

---

## Section 6 — Simulation Video (IROS Requirement)

### T6.1 — Demo Video Recording 🟠 P1
**Assigned to:** A1, A3
**Effort:** ~4 hours (after T3.1 and T3.2)

**IROS video requirements:**
- Duration: ≤ 3 minutes
- Format: MP4, ≥ 720p
- Must include narration or on-screen text

**Video storyboard:**
1. (0:00–0:20) Title card: SAFEMRS architecture diagram
2. (0:20–0:50) Gazebo simulation launches — Go2 + drone visible in inspection world
3. (0:50–1:20) Demo 1 — safe command approved: "Drone takeoff to 5 meters" → drone lifts off, terminal shows "✅ Approved"
4. (1:20–2:00) Demo 2 — unsafe command blocked: "Move drone to Go2 position" → terminal shows "❌ Rejected: Spatial conflict — drone altitude 1.0m < minimum separation from Go2"
5. (2:00–2:30) Demo 3 — commonsense hazard reviewed: "Fly through narrow corridor while Go2 is inside" → terminal shows "⚠️ Review: LLM flagged commonsense hazard (formal channel cannot express room-sharing conflict)"
6. (2:30–3:00) Results summary: Table III + Table V numbers on screen

**Tools:** OBS Studio (screen capture) + ffmpeg (compression)

---

## Section 7 — Code & Repository

### T7.1 — Add Claude Backend to `base_reasoner.py` 🟠 P1
**Assigned to:** A1
**Effort:** ~1 hour

```python
# In safemrs/safemrs/channel_llm/base_reasoner.py
# Add elif block in get_llm():
elif backend.startswith("claude"):
    try:
        from langchain_anthropic import ChatAnthropic
        return ChatAnthropic(
            model=backend,
            temperature=kwargs.get("temperature", 0.0),
        )
    except (ImportError, Exception) as e:
        logger.warning(f"Failed to initialize Anthropic backend: {e}")
```

**Also update `pyproject.toml`:**
```toml
[project.optional-dependencies]
anthropic = ["langchain-anthropic>=0.3.0"]
```

---

### T7.2 — Update `generate_scenarios.py` Documentation 🟡 P2
**Assigned to:** A6
**Effort:** ~1 hour

Verify that `generate_scenarios.py` has accurate comments describing the 102-scenario structure (7 categories, exact per-category counts). Cross-check with Table II in the paper.

---

### T7.3 — Repository Cleanup for Public Release 🟡 P2
**Assigned to:** A1
**Effort:** ~2 hours (after submission)

- Remove `OPENAI_API_KEY` from any scripts or logs that may have captured it
- Ensure `results/gpt4o_experiment.log` does not contain the API key
- Add `.gitignore` entries for `*.log`, `results/gpt4o_experiment.log`
- Verify `README.md` has correct public repo URL: `https://github.com/asmbatati/SAFEMRS`
- Tag release: `git tag v1.0.0-iros2026`

---

## Summary Table — All Tasks

| ID | Task | Owner | Priority | Effort | Status |
|----|------|-------|----------|--------|--------|
| T1.1 | Claude 3.5 Haiku comparison | A1, A5 | 🟠 P1 | 6h | ⏳ Pending |
| T1.2 | Llama 3.1:8b comparison | A5 | 🟡 P2 | 3h | ⏳ Optional |
| T2.1 | VerifyLLM / No-Verification baselines | A1, A6 | 🔴 P0 | 6h | ⏳ Pending |
| T2.2 | Ablation study write-up | A1, A4 | 🟠 P1 | 3h | ⏳ Pending |
| T3.1 | Inspection world SDF | A3 | 🔴 P0 | 8h | ⏳ Pending |
| T3.2 | Simulation end-to-end verification | A1, A3 | 🔴 P0 | 4h | ⏳ Pending |
| T3.3 | Gazebo screenshots | A3, A4 | 🟠 P1 | 2h | ⏳ After T3.1 |
| T4.1 | Per-category HDR bar chart | A4 | 🔴 P0 | 3h | ⏳ Pending |
| T4.2 | System architecture figure | A4, A8 | 🔴 P0 | 4h | ⏳ Pending |
| T4.3 | Review rate bar chart | A4 | 🟠 P1 | 2h | ⏳ Pending |
| T4.4 | EffCov vs FPR scatter | A4 | 🟡 P2 | 2h | ⏳ Optional |
| T5.1 | references.bib completion | A7, A8 | 🔴 P0 | 4h | ⏳ Pending |
| T5.2 | LaTeX compile + page count | A8 | 🔴 P0 | 2h | ⏳ Pending |
| T5.3 | Abstract + conclusion review | A2 | 🟠 P1 | 2h | ⏳ Pending |
| T5.4 | Section V narrative pass | A2, A9 | 🟠 P1 | 3h | ⏳ Pending |
| T5.5 | Figure placement + captions | A8 | 🟡 P2 | 2h | ⏳ After T4 |
| T6.1 | Demo video recording | A1, A3 | 🟠 P1 | 4h | ⏳ After T3 |
| T7.1 | Claude backend in base_reasoner.py | A1 | 🟠 P1 | 1h | ⏳ Pending |
| T7.2 | generate_scenarios.py docs | A6 | 🟡 P2 | 1h | ⏳ Pending |
| T7.3 | Repo cleanup for public release | A1 | 🟡 P2 | 2h | ⏳ After submission |

---

## Critical Path to Submission

```
Week 1 (Feb 22–28):
  A3:  T3.1 — Build inspection world SDF         [8h]
  A7+A8: T5.1 — Complete references.bib          [4h]
  A1:  T2.1 — Add No-Verification + PDDL-Only rows [6h]
  A1:  T7.1 — Add Claude backend                  [1h]
  A1+A5: T1.1 — Run Claude comparison experiment  [6h]
  A4:  T4.2 — Architecture figure                 [4h]
  A4:  T4.1 — Per-category HDR bar chart          [3h]

Week 2 (Mar 1–2, DEADLINE):
  A1+A3: T3.2 — Simulation verification           [4h]
  A3+A4: T3.3 — Gazebo screenshots                [2h]
  A4:  T4.3 — Review rate chart                   [2h]
  A2:  T5.3 — Abstract + conclusion review        [2h]
  A2+A9: T5.4 — Section V narrative pass          [3h]
  A8:  T5.2 — LaTeX compile + 6-page check        [2h]
  A8:  T5.5 — Figure placement                    [2h]
  A1+A3: T6.1 — Record demo video                 [4h]
  ALL: Final read-through + submit                [2h]
```

**Total estimated effort remaining:** ~70–75 person-hours across 9 authors.

---

## Already Complete ✅

| Item | Status |
|------|--------|
| 102-scenario benchmark (7 categories) | ✅ Complete |
| Qwen3:8b full experiment (formal/LLM/dual) | ✅ Complete |
| GPT-4o full experiment (formal/LLM/dual) | ✅ Complete |
| All LaTeX tables (II, III, IV, V) | ✅ Filled |
| Zero `\unvalidated{}` in paper | ✅ Confirmed |
| 51/51 unit tests passing | ✅ Confirmed |
| EffCov + ΔC metrics implemented | ✅ Complete |
| Per-category review rate in `analyze_results.py` | ✅ Complete |
| `reproduce.sh` + `check_progress.py` with `--results-dir` | ✅ Complete |
| GitHub repo URL in Acknowledgment | ✅ Complete |
| Author list updated (9 authors, correct affiliations) | ✅ Complete (user edit) |
| Citation keys updated to new format | ✅ Complete (user edit) |
| Tables resized with `\resizebox` | ✅ Complete (user edit) |
| `\bibliography{references.bib}` (switched from inline) | ✅ Complete (user edit) |
| SAFEMRS simulation guide in `final_roadmap.md` | ✅ Complete |
| NEUROS-X vision documented in `NEUROS-X.md` | ✅ Complete |
