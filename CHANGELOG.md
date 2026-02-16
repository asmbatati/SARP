# SAFEMRS Changelog

> Chronological record of all development sessions, prompts, and outcomes for the SAFEMRS project.

---

## Session 7 — 2026-02-16 (Evening)

### 7.1 Analyze Implementation Repositories

**Prompt:**
> Analyze `ros2_agent_sim` and `ros2_agent_sim_docker` repos — deep dive every package, tool, launch file, and Docker config. Map against SAFEMRS architecture.

**Analyzed files across both repos:**

| Repository | Files Analyzed | Key Findings |
|---|---|---|
| `ros2_agent_sim/ros2_agent` | `ros2_agent_node.py`, `model.py`, `specialist_models.py`, `system_prompts.py`, `drone_tools.py` (926 lines, 7 tools), `unitree_tools.py` (760 lines, 8 tools) | Single ROSA agent (Qwen3:8b) controls all robots; VLM specialist (Qwen2.5VL:7b); fire-and-forget threaded tools; namespace isolation |
| `ros2_agent_sim/simulation_gui` | `gui_node.py` (247 lines), `App.jsx` (232 lines) | FastAPI + React drag-drop builder; generates launch files dynamically; only Go2 + drone implemented |
| `ros2_agent_sim/sar_system` | `sar_system.launch.py` (608 lines) | Go2 (CHAMP + EKF) + PX4 drone (SITL + MAVROS + XRCE-DDS); timed spawn sequence |
| `ros2_agent_sim/sim_evaluation` | `eval_logger.py` (102 lines) | RTF logger only — no safety or task metrics |
| `ros2_agent_sim_docker` | `README.md`, PX4 model configs, scripts | Monolithic image bundles Ollama (~20+ GB) |

---

### 7.2 Create GitHub Implementation Analysis

**Prompt:**
> Write the whole analysis into `github_implementation_analysis.md`. Include suggested structure for new repos, ROS 2 packages, Docker image contents, and installation requirements (Ollama stays on host).

**Created [github_implementation_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/github_implementation_analysis.md)** (~780 lines) with 7 sections:

| Section | Content |
|---------|---------|
| §1 Current Repo Audit | Package-by-package breakdown of all 6 ROS 2 packages + Docker repo |
| §2 Gap Analysis | 5-tier Mermaid diagram + 18-row gap table (SRL, ARL, APL, RTM, HAL, Eval) + 8 structural problems |
| §3 New Repo Structure | 3 repos under `github.com/asmbatati`: `safemrs` (6 packages), `safemrs_sim` (5 packages), `safemrs_docker` |
| §4 Docker Architecture | Layered images (~8–10 GB base); **Ollama + models (~20+ GB) on host**, not in container; `docker-compose.yml` |
| §5 System Requirements | Hardware (16 GB RAM min, NVIDIA GPU), storage breakdown (~26–28 GB total), prerequisites |
| §6 Phased Roadmap | 4 phases: Safety (wk 1–3), Planning+Monitoring (wk 4–6), Eval (wk 7–9), Advanced (wk 10+) |
| §7 Strengths to Preserve | 10 elements to carry forward (ROSA, namespace isolation, VLM pipeline, dynamic launch, Docker) |

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

| Section | Before | After |
|---------|--------|-------|
| Subtitle | Generic architecture description | **"Verifiable Neuro-Symbolic Cognitive Proxy"** framing |
| §3 Architecture Diagram | 5 subgraphs (ARL, MCP, SRL, RTM, MRS) | **7 subgraphs** (+HAL with 6 adapters, Robot Resumes in MRS) |
| Robot types | Manipulator + Mobile Base | **Quadruped + UUV (Underwater)** — diverse morphologies |
| §4.4 RTM | Basic closed-loop monitoring | **PEFA (Proposal-Execution-Feedback-Adjustment)** closed-loop with 4 formalized phases |
| §4.6 (new) | N/A | **HAL — Middleware-Agnostic Proxy** (ROS 1, ROS 2, gRPC, MAVLink, XRCE-DDS, VLA) |
| VLA | Not mentioned | **VLA Execution Bridge** for symbolic→continuous control ("last mile" execution) |
| §4.7 MRS | Basic fleet description | **Embodiment-Aware Fleet** with Robot Resumes (URDF-derived) + Skill Ontology |
| §5 Sequence Diagram | "Dual-Channel Safety Verification" | **"Triple-Channel Safety Verification"** |
| §6.1 Heatmap | 16 features | **19 features** (+VLA bridge, HAL/middleware, embodiment-aware allocation) |
| §7 Novelties | 5 novelties | **6 novelties** (+Novelty 6: Embodiment-Aware Cognitive Proxy) |
| §10 Summary | 10 gaps addressed | **13 gaps** (+middleware-agnostic, embodiment-aware, symbolic→continuous bridge) |

**Changes to [competitive_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/competitive_analysis.md)** (462 → 471 lines):

| Section | Before | After |
|---------|--------|-------|
| §2.13 Heatmap | 14 systems × 13 columns | **14 systems × 16 columns** (+VLA Bridge, HAL/Middleware, Robot Resumes) |
| §3 Strategy D | 4 bullet points | **5 bullets** (+cognitive proxy framing, HAL, VLA, Robot Resumes as systems contributions) |
| §6 Action Plan | Phase 2 with 4 items | **6 items** (+HAL implementation, Robot Resume generator) |
| §7 Recommendation | 6 reasons, generic title | **7 reasons** (+cognitive proxy framing), new merged title |

New suggested paper title:
> *"SAFEMRS: A Verifiable Neuro-Symbolic Cognitive Proxy for Safe Multi-Robot Autonomy with Triple-Channel Safety Verification"*

---

## Session 5 — 2026-02-16 (Afternoon, Later)

### 5.1 Cross-Analysis of Brainstorming and Main.tex vs. Proposal Documents

**Prompt:**
> Analyze brainstorming.md and main.tex. Then compare them with proposal/architecture_proposal.md and suggested strategies in proposal/competitive_analysis.md. Make a file named brainstorming_output.md.

**Created [brainstorming_output.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/brainstorming_output.md)** — 9-section cross-analysis:

| Section | Content |
|---------|---------|
| §1 Document Overview | 4-document comparison table |
| §2 Architectural Comparison | 7-tier layer mapping (brainstorming 4-tier vs. main.tex 4-tier vs. SAFEMRS 5-layer) |
| §3 Research Question Alignment | RQ granularity comparison |
| §4 Concept Inventory | What each document uniquely contributes (VLA, Robot Resumes, PEFA from brainstorming; triple-channel, CBFs, conformal from SAFEMRS) |
| §5 Competitive Positioning | Feature coverage per framing against 14-system heatmap |
| §6 Strategy Alignment | Gap analysis for Strategy A and B requirements |
| §7 Synthesis | Proposed merged 5-tier architecture + keep/drop recommendations |
| §8 Action Items | 12 concrete tasks (immediate, short-term, paper strategy) |
| §9 Risk Assessment | 6 risks with severity ratings and mitigations |

Key finding: brainstorming and SAFEMRS are **complementary** — brainstorming provides systems framing (cognitive proxy, HAL, VLA, middleware diversity), SAFEMRS provides safety depth (triple-channel, CBF, conformal prediction).

---

## Session 4 — 2026-02-16 (Afternoon)

### 4.1 Update Architecture Proposal with Expanded Literature

**Prompt:**
> Now update the architecture proposal to reflect the expanded literature summary.

**Changes to [architecture_proposal.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/architecture_proposal.md)** (433 → 500 lines):

| Section | Before | After |
|---------|--------|-------|
| §1 Motivation | 4 bullet categories | 7 categories (+runtime safety, code gen, multi-agent, hybrid LLM+RL) |
| §4.3 Safety Reasoning Layer | Dual-channel (formal + LLM) | **Triple-channel** (formal + LLM + CBF runtime enforcement) with conformal prediction |
| §4.4 Real-Time Monitoring | Basic monitoring | References LLM-CBT, CLGA, integrates CBF safety margins |
| §4.5 Abstract Planning Layer | 5 backends | **8 backends** (+STL from AutoTAMP, FSM from Mu et al., Code Gen from Code as Policies/ProgPrompt) |
| §6.1 Feature Comparison Matrix | 8 systems, 12 features | **14 systems**, 16 features (+CBF enforcement, conformal, code gen, multi-agent dialog) |
| §6.2 Positioning Paragraphs | 6 comparisons | **12 comparisons** (+SAFER, S-ATLAS, RoCo, LLM-CBT, Code as Policies, NL2HLTL2PLAN) |
| §7 Novelty Claims | 4 novelties | **5 novelties** (+Novelty 5: Multi-paradigm safety integration) |
| §8 Research Questions | 4 RQs | **5 RQs** (+RQ5 on conformal prediction calibration) |
| §9 Evaluation Plan | 5 dimensions | **7 dimensions** (+Runtime Safety, Manipulation; +RoCoBench, LEMMA, OBiMan-Bench) |
| §10 Summary Table | 7 gaps | **10 gaps** (+runtime enforcement, probabilistic bounds, code safety) |

Key architectural evolution: dual-channel → **triple-channel** safety verification integrating CBFs (from SAFER), conformal prediction (from S-ATLAS), and syntax-guaranteed LTL (from LTLCodeGen).

---

### 4.2 Update Competitive Analysis with New Insights

**Prompt:**
> Now update competitive_analysis.md with the new insights.

**Changes to [competitive_analysis.md](file:///g:/My%20Drive/02%20Areas/Career/PSU/Research/Active/MARS/IROS%20Paper/SAFEMRS/proposal/competitive_analysis.md)** (360 → 461 lines):

| Section | Before | After |
|---------|--------|-------|
| §1 Integration Trap | 8 rows | **13 rows** (+CBF, conformal, code gen, dialog, hybrid RL) |
| §2 Competitors | 7 systems (§2.1–2.7) | **12 systems** (§2.1–2.12: +SAFER, S-ATLAS, RoCo, LLM-CBT, Code as Policies) |
| §2.2 VerifyLLM | Single system | **3-way comparison** (VerifyLLM + LTLCodeGen + NL2HLTL2PLAN) |
| §2.13 Summary Heatmap | 8 systems × 10 columns | **14 systems × 13 columns** |
| §3 Strategy A | Dual-channel with 5-category benchmark | **Triple-channel** with 7-category benchmark + conformal calibration |
| §4 Paper Structure | 3 related work categories | **5 categories** (+runtime safety, code gen) |
| §5 Rejection Table | Generic baselines | **8 baselines** with channel-specific failure modes |
| §6 Action Plan | Dual-channel + 2 backends | **Triple-channel** (3 sub-channels) + **8 backends** + 8 baselines |
| §7 Final Recommendation | 5 reasons, dual-channel | **6 reasons**, triple-channel |

All "dual-channel" references updated to "triple-channel." New suggested paper title:
> *"Triple-Channel Corroborative Safety Verification for LLM-Based Multi-Robot Task Planning: Unifying Formal Logic, Probabilistic Reasoning, and Runtime Enforcement"*

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

| Batch | Papers | Key Topics |
|-------|--------|------------|
| §2.17–2.26 | SAFER, LLM-CBT, LTLCodeGen, NL2HLTL2PLAN, S-ATLAS, Yuan et al., CLGA, EmbodiedAgent, Wang et al., Chen et al. | Safety pipelines, BT generation, formal specs, conformal prediction |
| §2.27–2.36 | Code-as-Symbolic-Planner, ICCO, FCRF, Huang Survey, Reasoner, Zuzuárregui, RoCo, AutoTAMP, LEMMA, AutoMisty | Code generation, MARL, reflection, multi-agent dialog, STL |
| §2.37–2.46 | Kwon et al., Hoffmeister, Code as Policies, ProgPrompt, LLM-GROP, LABOR, Mu et al., OBiMan | Scene graphs, zero-knowledge BTs, manipulation benchmarks |

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
