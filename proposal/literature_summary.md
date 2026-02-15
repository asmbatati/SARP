# Literature Summary: LLM-Based Multi-Robot Task Planning

> **16 papers analyzed** | Spanning 2020–2025 | Focus: Large Language Models for Multi-Robot Coordination, Task Planning, and Verification

---

## 1. Overview

This document summarizes 16 research papers investigating the intersection of **Large Language Models (LLMs)** and **multi-robot systems (MRS)**. The literature reveals a rapidly maturing field where LLMs are combined with classical planning formalisms (PDDL, HTN, LTL), optimization methods (LP, MILP), and structured representations (DAGs, behavior trees, hierarchical trees) to achieve robust task decomposition, allocation, and execution for heterogeneous robot teams.

---

## 2. Paper-by-Paper Summaries

### 2.1 Webster et al. (2020) — Corroborative V&V of Human–Robot Teams

| Field          | Detail                                                                                                                                             |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | A Corroborative Approach to Verification and Validation of Human–Robot Teams                                                                       |
| **Venue**      | International Journal of Robotics Research                                                                                                         |
| **Key Idea**   | Combines **formal verification**, **simulation-based testing**, and **user validation** into a corroborative V&V methodology for human–robot teams |
| **Method**     | Uses model checking (PRISM), agent-based simulation (Gazebo), and user studies; evaluates trade-offs between realism and coverability              |
| **Case Study** | Handover task between a human and a robot arm                                                                                                      |
| **Relevance**  | Establishes the need for multi-faceted verification of robot plans — a theme echoed by later LLM-based verification works                          |

---

### 2.2 Pellier et al. (2023) — HDDL 2.1: Temporal HTN Planning

| Field         | Detail                                                                                                                                             |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**     | HDDL 2.1: Towards Defining a Formalism and a Semantics for Temporal HTN Planning                                                                   |
| **Venue**     | ICAPS Workshop                                                                                                                                     |
| **Key Idea**  | Extends PDDL/HDDL to support **temporal and numerical constraints** within Hierarchical Task Network (HTN) planning                                |
| **Method**    | Proposes syntax and semantics for durative actions, temporal ordering (before, after, between), and numeric fluents in hierarchical decompositions |
| **Relevance** | Provides the formal planning language foundation that several LLM-based frameworks (PLANTOR, GMATP-LLM, LaMMA-P) build upon or complement          |

---

### 2.3 Zhao et al. (2024) — MultiBotGPT

| Field        | Detail                                                                                                                                                      |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | Applying Large Language Model to a Control System for Multi-Robot Task Assignment                                                                           |
| **Venue**    | IEEE Access                                                                                                                                                 |
| **Key Idea** | Integrates **GPT-3.5** into a layered multi-robot control system (LLLM → LCI → LRC) for translating natural language commands into executable UAV/UGV tasks |
| **Results**  | Outperforms BERT-based baselines in task success rates; reduces human operator cognitive load                                                               |
| **Robots**   | UAVs and UGVs in simulated reconnaissance/delivery scenarios                                                                                                |

---

### 2.4 Kannan et al. (2024) — SMART-LLM

| Field          | Detail                                                                                                                                 |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | SMART-LLM: Smart Multi-Agent Robot Task Planning using Large Language Models                                                           |
| **Venue**      | IROS 2024                                                                                                                              |
| **Key Idea**   | Uses **few-shot prompting** with LLMs for three-stage multi-robot planning: task decomposition → coalition formation → task allocation |
| **Benchmark**  | Introduces a benchmark dataset for evaluating multi-robot LLM planners                                                                 |
| **Results**    | Demonstrates promising results in both simulation (AI2-THOR) and real-world scenarios                                                  |
| **Limitation** | Coarse task decomposition granularity; struggles with complex interdependencies                                                        |

---

### 2.5 Saccon et al. (2025) — PLANTOR

| Field                 | Detail                                                                                                                                        |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**             | A Temporal Planning Framework for Multi-Agent Systems via LLM-Aided Knowledge Base Management                                                 |
| **Venue**             | RAL / ICRA 2025                                                                                                                               |
| **Key Idea**          | Integrates LLMs with a **Prolog-based knowledge base** for temporal multi-robot planning                                                      |
| **Method**            | Two-phase KB generation (LLM draft → Prolog validation) + three-step planning (KB query → MILP scheduling → Behavior Tree execution via ROS2) |
| **Temporal Handling** | Explicit durative actions, resource constraints, and parallel execution via MILP optimization                                                 |
| **Results**           | LLMs produce accurate KBs with minimal human feedback in furniture assembly tasks                                                             |

---

### 2.6 Yang et al. (2025) — AutoHMA-LLM

| Field         | Detail                                                                                                                                     |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| **Title**     | AutoHMA-LLM: Efficient Task Coordination and Execution in Heterogeneous Multi-Agent Systems Using Hybrid Large Language Models             |
| **Venue**     | ICRA 2025                                                                                                                                  |
| **Key Idea**  | **Hybrid cloud/device LLM architecture** — a cloud LLM as central planner + device-specific LLMs and Generative Agents for local execution |
| **Scenarios** | Logistics, inspection, and search & rescue with heterogeneous robots                                                                       |
| **Results**   | Improved accuracy, communication efficiency, and reduced token usage compared to fully centralized approaches                              |

---

### 2.7 Liu et al. (2025) — COHERENT

| Field         | Detail                                                                                                                    |
| ------------- | ------------------------------------------------------------------------------------------------------------------------- |
| **Title**     | COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models                                    |
| **Venue**     | ICRA 2025                                                                                                                 |
| **Key Idea**  | Centralized LLM framework using a **Proposal-Execution-Feedback-Adjustment (PEFA)** mechanism for iterative task planning |
| **Benchmark** | 100 complex long-horizon tasks for heterogeneous multi-robot systems                                                      |
| **Results**   | **97.5% success rate**, surpassing previous methods in both success rate and efficiency                                   |

---

### 2.8 Huang et al. (2025) — LAN2CB

| Field        | Detail                                                                                                 |
| ------------ | ------------------------------------------------------------------------------------------------------ |
| **Title**    | Compositional Coordination for Multi-Robot Teams with Large Language Models                            |
| **Venue**    | ICRA 2025                                                                                              |
| **Key Idea** | Translates **natural language → behavior trees → executable Python code** for multi-robot coordination |
| **Method**   | Mission Analysis module (NL to BT) + Code Generation module (BT to Python)                             |
| **Results**  | Robust coordination demonstrated in both simulation and real-world environments                        |

---

### 2.9 Wang et al. (2025) — DART-LLM

| Field            | Detail                                                                                                         |
| ---------------- | -------------------------------------------------------------------------------------------------------------- |
| **Title**        | DART-LLM: Dependency-Aware Multi-Robot Task Decomposition and Execution using Large Language Models            |
| **Venue**        | ICRA 2025                                                                                                      |
| **Key Idea**     | Uses **Directed Acyclic Graphs (DAGs)** to model task dependencies explicitly                                  |
| **Architecture** | QA LLM (decomposition) → Breakdown Function (robot assignment) → Actuation module → VLM-based object detection |
| **Key Finding**  | Explicit dependency modeling significantly improves smaller LLMs' planning performance                         |
| **Results**      | State-of-the-art performance on multi-robot task benchmarks                                                    |

---

### 2.10 Zhu et al. (2025) — DEXTER-LLM

| Field        | Detail                                                                                                                                |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | DEXTER-LLM: Dynamic and Explainable Coordination of Multi-Robot Systems in Unknown Environments via Large Language Models             |
| **Venue**    | Preprint 2025                                                                                                                         |
| **Key Idea** | **Dynamic online planning** in unknown environments with multi-stage LLM reasoning and explainability                                 |
| **Method**   | Mission comprehension → online subtask generation → optimal assignment (branch-and-bound) → dynamic adaptation with human-in-the-loop |
| **Results**  | **100% success rate** in tested scenarios; superior plan quality with explainable outputs                                             |

---

### 2.11 Deng et al. (2025) — GMATP-LLM

| Field        | Detail                                                                                                                                                     |
| ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | GMATP-LLM: A General Multi-Agent Task Dynamic Planning Method using Large Language Models                                                                  |
| **Venue**    | Preprint 2025                                                                                                                                              |
| **Key Idea** | Combines **Chain-of-Thought (CoT) prompting** with PDDL planners for multi-agent task planning                                                             |
| **Method**   | LLM-based CoT task decomposition → PDDL goal generation → intelligent planner solver + 3D Spatio-Temporal Motion Corridor for parallel motion optimization |
| **Novelty**  | Integrates spatial-temporal motion planning with high-level task planning                                                                                  |

---

### 2.12 Gupta et al. (2025) — Hierarchical Trees for Multi-Robot Mission Planning

| Field         | Detail                                                                                                  |
| ------------- | ------------------------------------------------------------------------------------------------------- |
| **Title**     | Generalized Mission Planning for Heterogeneous Multi-Robot Teams via LLM-Constructed Hierarchical Trees |
| **Venue**     | ICRA 2025                                                                                               |
| **Key Idea**  | LLM-constructed **hierarchical trees** for mission decomposition with custom APIs and subtree routines  |
| **Method**    | Complex missions → hierarchical sub-task trees → optimized robot schedules via MRTA alternatives        |
| **Strengths** | Flexibility, scalability, and demonstrated generalization across mission types                          |

---

### 2.13 Zhang et al. (2025) — LaMMA-P

| Field            | Detail                                                                                                                                   |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**        | LaMMA-P: Generalizable Multi-Agent Long-Horizon Task Allocation and Planning with LM-Driven PDDL Planner                                 |
| **Venue**        | ICRA 2025                                                                                                                                |
| **Key Idea**     | Integrates LLM reasoning with **PDDL Fast Downward heuristic search** for long-horizon multi-agent planning                              |
| **Architecture** | 6 modules: Precondition Identifier → Task Allocator → Problem Generator → PDDL Validator → Fast Downward/LLM Planner → Sub-Plan Combiner |
| **Benchmark**    | MAT-THOR: 70 tasks across compound, complex, and vague command categories                                                                |
| **Results**      | **105% higher success rate** and **36% higher efficiency** than SMART-LLM baseline                                                       |

---

### 2.14 Obata et al. (2025) — LiP-LLM

| Field             | Detail                                                                                                                 |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------- |
| **Title**         | LiP-LLM: Integrating Linear Programming and Dependency Graph with Large Language Models for Multi-Robot Task Planning  |
| **Venue**         | IEEE RA-L, Feb 2025                                                                                                    |
| **Key Idea**      | Combines **dependency graph generation** (via LLMs) with **linear programming** (LP) for optimal task allocation       |
| **Method**        | 3 steps: skill list generation (SayCan-style likelihood) → DAG dependency graph → LP-based optimal assignment          |
| **Results**       | Maximum success rate difference of **0.82** over baselines; robust across robot constraints and environment complexity |
| **Key Advantage** | LP-based allocation scales better than LLM-based allocation and avoids hallucination in assignment                     |

---

### 2.15 Obi et al. (2025) — SafePlan

| Field          | Detail                                                                                                                                   |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | SafePlan: Leveraging Formal Logic and Chain-of-Thought Reasoning for Enhanced Safety in LLM-based Robotic Task Planning                  |
| **Venue**      | arXiv 2025                                                                                                                               |
| **Key Idea**   | Multi-component **safety verification framework** for LLM-based robotic task planning                                                    |
| **Components** | Prompt Sanity Check COT Reasoner (3 layers: societal → organizational → individual) + Invariant/Precondition/Postcondition COT Reasoners |
| **Benchmark**  | 621 expert-curated task prompts tested on AI2-THOR                                                                                       |
| **Results**    | **90.5% reduction** in harmful task prompt acceptance while maintaining reasonable safe task acceptance                                  |

---

### 2.16 Grigorev et al. (2025) — VerifyLLM

| Field        | Detail                                                                                                                                            |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | VerifyLLM: LLM-Based Pre-Execution Task Plan Verification for Robots                                                                              |
| **Venue**    | arXiv, Jul 2025                                                                                                                                   |
| **Key Idea** | **Pre-execution verification** of robot task plans by combining LTL formulas with LLM-based contextual reasoning                                  |
| **Method**   | Translation Module (NL → LTL) + Verification Module (sliding window analysis detecting position errors, missing prerequisites, redundant actions) |
| **Datasets** | ALFRED-LTL, VirtualHome-LTL                                                                                                                       |
| **Results**  | ~40% reduction in ordering errors with Claude-based implementation; LLM verification contributes far more than LTL alone                          |

---

## 3. Thematic Analysis

### 3.1 LLM Integration Patterns

The literature reveals **four dominant patterns** for integrating LLMs into multi-robot planning:

| Pattern                     | Description                                                        | Papers                                                         |
| --------------------------- | ------------------------------------------------------------------ | -------------------------------------------------------------- |
| **LLM as Decomposer**       | LLM decomposes NL instructions into sub-tasks                      | SMART-LLM, DART-LLM, COHERENT, LaMMA-P, LiP-LLM, GMATP-LLM     |
| **LLM + Classical Planner** | LLM generates formal specifications solved by traditional planners | LaMMA-P (PDDL), GMATP-LLM (PDDL), PLANTOR (MILP), LiP-LLM (LP) |
| **LLM as Code Generator**   | LLM directly produces executable code or behavior trees            | LAN2CB, MultiBotGPT, SafePlan                                  |
| **LLM as Verifier**         | LLM validates plans for correctness and safety                     | VerifyLLM, SafePlan, DEXTER-LLM (human-in-the-loop)            |

### 3.2 Dependency and Temporal Modeling

A critical recurring challenge is correctly modeling **task dependencies and temporal constraints**:

- **DAG-based**: DART-LLM, LiP-LLM explicitly use directed acyclic graphs
- **PDDL temporal extensions**: HDDL 2.1, PLANTOR, GMATP-LLM
- **LTL-based**: VerifyLLM, SafePlan use Linear Temporal Logic for formal specifications
- **Hierarchical decomposition**: Gupta et al., LaMMA-P, AutoHMA-LLM

### 3.3 Heterogeneous Robot Teams

Most frameworks explicitly target **heterogeneous multi-robot systems** where robots possess different skills:

- Skill-aware allocation: SMART-LLM, LaMMA-P, LiP-LLM, COHERENT
- Cloud/device hybrid architectures: AutoHMA-LLM
- Capability-conditioned planning: DART-LLM, Gupta et al.

### 3.4 Safety and Verification

A growing sub-field addresses plan **safety and correctness**:

| Approach                   | Mechanism                                  | Paper          |
| -------------------------- | ------------------------------------------ | -------------- |
| Pre-execution verification | LTL + LLM sliding window                   | VerifyLLM      |
| Prompt safety screening    | Formal logic + CoT multi-layer reasoning   | SafePlan       |
| Corroborative V&V          | Model checking + simulation + user studies | Webster et al. |
| Human-in-the-loop          | Dynamic adaptation with explainability     | DEXTER-LLM     |

### 3.5 Benchmarks and Evaluation

| Benchmark         | Source          | Tasks                             | Papers Using It                                        |
| ----------------- | --------------- | --------------------------------- | ------------------------------------------------------ |
| AI2-THOR          | Allen AI        | Household simulation              | SMART-LLM, LaMMA-P, COHERENT, SafePlan                 |
| MAT-THOR          | Zhang et al.    | 70 long-horizon multi-agent tasks | LaMMA-P                                                |
| VirtualHome       | Puig et al.     | Daily household activities        | VerifyLLM                                              |
| ALFRED            | Shridhar et al. | Vision-language navigation        | VerifyLLM                                              |
| Custom benchmarks | Various         | Domain-specific                   | DART-LLM, COHERENT (100 tasks), SafePlan (621 prompts) |

---

## 4. Comparative Summary Table

| Paper          | Year | LLM Role               | Planning Formalism   | Optimization       | Robots        | Key Metric                    |
| -------------- | ---- | ---------------------- | -------------------- | ------------------ | ------------- | ----------------------------- |
| Webster et al. | 2020 | —                      | Model checking       | —                  | HRI team      | V&V coverage                  |
| Pellier et al. | 2023 | —                      | HDDL 2.1 / HTN       | —                  | Multi-agent   | Formalism completeness        |
| MultiBotGPT    | 2024 | Controller             | Layered architecture | —                  | UAV + UGV     | Task success rate             |
| SMART-LLM      | 2024 | Decomposer             | Few-shot prompting   | —                  | Heterogeneous | SR, GCR, Exe                  |
| PLANTOR        | 2025 | KB manager             | Prolog + MILP        | MILP               | Multi-arm     | KB accuracy                   |
| AutoHMA-LLM    | 2025 | Coordinator            | Hybrid cloud/device  | —                  | Heterogeneous | Token efficiency              |
| COHERENT       | 2025 | Planner (PEFA)         | Centralized LLM      | —                  | Heterogeneous | **97.5% SR**                  |
| LAN2CB         | 2025 | Code generator         | Behavior trees       | —                  | Multi-robot   | Execution robustness          |
| DART-LLM       | 2025 | Decomposer             | DAG dependencies     | —                  | Multi-robot   | SOTA SR                       |
| DEXTER-LLM     | 2025 | Online planner         | Branch-and-bound     | B&B                | Multi-robot   | **100% SR**                   |
| GMATP-LLM      | 2025 | Decomposer (CoT)       | PDDL                 | Motion corridor    | Multi-agent   | Plan quality                  |
| Gupta et al.   | 2025 | Tree constructor       | Hierarchical trees   | MRTA               | Heterogeneous | Scalability                   |
| LaMMA-P        | 2025 | Decomposer + validator | PDDL + Fast Downward | Heuristic search   | Heterogeneous | **105% SR improvement**       |
| LiP-LLM        | 2025 | Graph generator        | DAG + LP             | Linear programming | Arm + mobile  | **0.82 SR diff.**             |
| SafePlan       | 2025 | Safety verifier        | Formal logic + CoT   | —                  | Multi-robot   | **90.5% harm reduction**      |
| VerifyLLM      | 2025 | Plan verifier          | LTL + LLM            | —                  | Single/multi  | **40% order error reduction** |

---

## 5. Key Research Gaps and Future Directions

1. **Real-world deployment**: Most frameworks are validated only in simulation (AI2-THOR, VirtualHome). Real-world transfer with sensor noise and dynamic environments remains underexplored.

2. **Scalability**: Few papers demonstrate performance beyond 3–5 robots. LiP-LLM and AutoHMA-LLM discuss scalability but lack large-scale validation.

3. **Safety guarantees**: SafePlan and VerifyLLM are pioneering, but formal safety guarantees for LLM-generated plans remain an open problem.

4. **Temporal reasoning**: HDDL 2.1 and PLANTOR address temporal planning, but most LLM-based frameworks treat time simplistically or ignore it.

5. **Dynamic re-planning**: Only DEXTER-LLM explicitly handles unknown environments with online re-planning; most frameworks assume static, fully observable environments.

6. **Unified benchmarks**: The field lacks a standardized benchmark for comparing multi-robot LLM planners across diverse scenarios, robot types, and complexity levels.

7. **Explainability**: DEXTER-LLM emphasizes explainability, but most frameworks provide limited insight into why specific task allocations or plans were chosen.

---

## 6. References

1. Webster, M. et al. (2020). "A corroborative approach to verification and validation of human–robot teams." _IJRR_.
2. Pellier, D. et al. (2023). "HDDL 2.1: Towards Defining a Formalism and a Semantics for Temporal HTN Planning." _ICAPS Workshop_.
3. Zhao, J. et al. (2024). "Applying Large Language Model to a Control System for Multi-Robot Task Assignment." _IEEE Access_.
4. Kannan, S. et al. (2024). "SMART-LLM: Smart Multi-Agent Robot Task Planning using Large Language Models." _IROS 2024_.
5. Saccon, E. et al. (2025). "A Temporal Planning Framework for Multi-Agent Systems via LLM-Aided Knowledge Base Management." _RAL / ICRA 2025_.
6. Yang, Z. et al. (2025). "AutoHMA-LLM: Efficient Task Coordination and Execution in Heterogeneous Multi-Agent Systems Using Hybrid Large Language Models." _ICRA 2025_.
7. Liu, K. et al. (2025). "COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models." _ICRA 2025_.
8. Huang, Y. et al. (2025). "Compositional Coordination for Multi-Robot Teams with Large Language Models." _ICRA 2025_.
9. Wang, Y. et al. (2025). "DART-LLM: Dependency-Aware Multi-Robot Task Decomposition and Execution using Large Language Models." _ICRA 2025_.
10. Zhu, W. et al. (2025). "DEXTER-LLM: Dynamic and Explainable Coordination of Multi-Robot Systems in Unknown Environments via Large Language Models." _Preprint_.
11. Deng, H. et al. (2025). "GMATP-LLM: A General Multi-Agent Task Dynamic Planning Method using Large Language Models." _Preprint_.
12. Gupta, A. et al. (2025). "Generalized Mission Planning for Heterogeneous Multi-Robot Teams via LLM-Constructed Hierarchical Trees." _ICRA 2025_.
13. Zhang, X. et al. (2025). "LaMMA-P: Generalizable Multi-Agent Long-Horizon Task Allocation and Planning with LM-Driven PDDL Planner." _ICRA 2025_.
14. Obata, K. et al. (2025). "LiP-LLM: Integrating Linear Programming and Dependency Graph with Large Language Models for Multi-Robot Task Planning." _IEEE RA-L_.
15. Obi, I. et al. (2025). "SafePlan: Leveraging Formal Logic and Chain-of-Thought Reasoning for Enhanced Safety in LLM-based Robotic Task Planning." _arXiv_.
16. Grigorev, D. et al. (2025). "VerifyLLM: LLM-Based Pre-Execution Task Plan Verification for Robots." _arXiv_.
