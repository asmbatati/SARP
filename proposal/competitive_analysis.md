# Competitive Analysis & Strategy for a Substantial Contribution to IROS/ICRA

> How does SAFEMRS compare to the state-of-the-art, and what must change to make it a **non-incremental**, high-impact contribution?

---

## 1. Honest Assessment: What Reviewers Will See

Before proposing solutions, we must confront the risk that IROS/ICRA reviewers will perceive the current SAFEMRS proposal as **incremental integration** — combining pieces from existing works without a sufficiently deep standalone contribution.

### The "Integration Trap"

| SAFEMRS Component                | Reviewer Will Say: "This Already Exists in…" |
| -------------------------------- | -------------------------------------------- |
| Agentic LLM task decomposition   | SMART-LLM, COHERENT, DART-LLM, LaMMA-P       |
| DAG-based dependency modeling    | DART-LLM, LiP-LLM                            |
| Formal logic verification (LTL)  | VerifyLLM                                    |
| LLM-based safety reasoning (CoT) | SafePlan                                     |
| Dynamic re-planning              | DEXTER-LLM                                   |
| Heterogeneous robot coordination | COHERENT, AutoHMA-LLM                        |
| Behavior tree execution          | PLANTOR, LAN2CB                              |
| PDDL integration                 | LaMMA-P, GMATP-LLM                           |

> [!CAUTION]
> **If the paper reads as "we combine SafePlan + VerifyLLM + DEXTER-LLM + MCP into one system," it will be desk-rejected at top venues.** Integration alone is not a contribution in robotics — reviewers demand either (a) new theory, (b) a new capability that no prior work could achieve, or (c) dramatically better empirical results on challenging benchmarks.

---

## 2. Paper-by-Paper Competitive Comparison

### 2.1 vs. SafePlan (Obi et al., 2025) — _Closest competitor on safety_

| Dimension               | SafePlan                                                                                                               | SAFEMRS                                               |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| **Safety mechanism**    | LLM-based CoT with 3-layer screening (societal → organizational → individual) + invariant/pre/post-condition reasoners | Dual-channel: formal logic (LTL/CTL) + LLM CoT        |
| **Formal verification** | ❌ No formal logic — purely LLM reasoning                                                                              | ✅ LTL/CTL model checking + deontic logic             |
| **When it verifies**    | Pre-execution (prompt screening only)                                                                                  | Pre-execution + runtime monitoring                    |
| **Multi-robot**         | Single-robot tasks in AI2-THOR                                                                                         | Heterogeneous multi-robot teams                       |
| **Hallucination risk**  | High — safety verdict is itself an LLM output                                                                          | Lower — formal channel provides ground truth          |
| **Conflict detection**  | Implicit in CoT reasoning                                                                                              | Explicit resource/spatial/temporal conflict detection |
| **Benchmark**           | 621 expert-curated prompts                                                                                             | TBD                                                   |

**Gap SAFEMRS fills**: SafePlan's fundamental weakness is that its safety verdict is **another LLM output** — it can hallucinate safety just as it can hallucinate plans. SAFEMRS's formal logic channel provides a **verifiable, non-probabilistic** safety baseline that the LLM channel complements but cannot override.

**But is that enough?** No — combining two existing ideas (SafePlan's CoT + VerifyLLM's LTL) is still integration. See §3 for how to deepen this.

---

### 2.2 vs. VerifyLLM (Grigorev et al., 2025) — _Closest competitor on formal verification_

| Dimension                | VerifyLLM                                            | SAFEMRS                                                                            |
| ------------------------ | ---------------------------------------------------- | ---------------------------------------------------------------------------------- |
| **Logic formalism**      | LTL only                                             | LTL + CTL + Deontic logic                                                          |
| **Verification method**  | Translation Module (NL → LTL) + LLM sliding window   | Model checking (state space exploration) + LLM CoT                                 |
| **Scope**                | Post-planning, pre-execution                         | Pre-execution + runtime                                                            |
| **Multi-robot**          | Single robot (ALFRED, VirtualHome)                   | Multi-robot with inter-robot dependencies                                          |
| **Error types detected** | Position errors, missing prerequisites, redundancies | + resource conflicts, spatial conflicts, temporal violations, common-sense hazards |
| **LLM role**             | Verifier only                                        | Planner + verifier + re-planner                                                    |
| **Results**              | ~40% ordering error reduction                        | TBD                                                                                |

**Gap SAFEMRS fills**: VerifyLLM verifies single-robot plans in fully observable environments. It does not handle multi-robot coordination constraints, resource conflicts, or partial observability. SAFEMRS extends formal verification to **multi-agent temporal properties** — a fundamentally harder problem.

---

### 2.3 vs. COHERENT (Liu et al., 2025) — _Closest competitor on performance_

| Dimension              | COHERENT                       | SAFEMRS                                     |
| ---------------------- | ------------------------------ | ------------------------------------------- |
| **Planning**           | Centralized LLM with PEFA loop | Agentic LLM with MCP augmentation           |
| **Success rate**       | 97.5%                          | TBD                                         |
| **Safety**             | ❌ None                        | ✅ Dual-channel                             |
| **External knowledge** | ❌ Closed-loop                 | ✅ MCP tools + external agents              |
| **Re-planning**        | PEFA feedback loop (internal)  | RTM-triggered (external observation-driven) |
| **Environment**        | Fully observable simulation    | Partially observable                        |

**Gap SAFEMRS fills**: COHERENT achieves impressive results but in **safe, fully observable** environments. It has no safety layer — a dangerous plan will execute if the LLM generates it. It also cannot access external information to ground its reasoning.

---

### 2.4 vs. DEXTER-LLM (Zhu et al., 2025) — _Closest competitor on dynamic adaptation_

| Dimension                    | DEXTER-LLM             | SAFEMRS                                  |
| ---------------------------- | ---------------------- | ---------------------------------------- |
| **Online re-planning**       | ✅ Yes                 | ✅ Yes                                   |
| **Unknown environments**     | ✅ Yes                 | ✅ Yes (partially observable)            |
| **Safety verification**      | ❌ None                | ✅ Dual-channel                          |
| **External resource access** | ❌ No                  | ✅ MCP                                   |
| **Explainability**           | ✅ Verbal explanations | Partial (through CoT traces)             |
| **Optimization**             | Branch-and-bound       | Abstract (LP, MILP, or heuristic search) |

**Gap SAFEMRS fills**: DEXTER-LLM re-plans dynamically but never verifies whether the new plan is safe. In high-stakes environments (warehouses, hospitals), this is unacceptable.

---

### 2.5 vs. LaMMA-P (Zhang et al., 2025) — _Closest competitor on LLM + formal planning_

| Dimension           | LaMMA-P                  | SAFEMRS                             |
| ------------------- | ------------------------ | ----------------------------------- |
| **Planner**         | PDDL + Fast Downward     | Abstract (PDDL, BT, DAG, HTN)       |
| **Validation**      | PDDL syntactic validator | Formal logic (LTL/CTL) + LLM safety |
| **Task allocation** | LLM-based                | LLM-based + formal optimization     |
| **Benchmark**       | MAT-THOR (70 tasks)      | TBD                                 |
| **SR improvement**  | 105% over SMART-LLM      | TBD                                 |

**Gap SAFEMRS fills**: LaMMA-P validates PDDL syntax but not semantic safety. It cannot detect a syntactically valid plan that is semantically dangerous.

---

### 2.6 vs. LiP-LLM (Obata et al., 2025) — _Closest competitor on optimization_

| Dimension               | LiP-LLM                      | SAFEMRS                            |
| ----------------------- | ---------------------------- | ---------------------------------- |
| **Dependency modeling** | DAG (LLM-generated)          | DAG (within abstract layer)        |
| **Task allocation**     | Linear Programming (optimal) | Configurable (LP, MILP, heuristic) |
| **Safety**              | ❌ None                      | ✅ Dual-channel                    |
| **External knowledge**  | ❌ No                        | ✅ MCP                             |
| **Re-planning**         | ❌ Static                    | ✅ Dynamic                         |

---

### 2.7 vs. DART-LLM (Wang et al., 2025)

| Dimension        | DART-LLM                   | SAFEMRS                         |
| ---------------- | -------------------------- | ------------------------------- |
| **Dependencies** | DAG-based, QA LLM          | DAG-based within abstract layer |
| **Verification** | VLM-based object detection | Formal logic + LLM CoT          |
| **Execution**    | Open-loop                  | Closed-loop with RTM            |
| **Safety**       | ❌                         | ✅                              |

---

### 2.8 Summary Heatmap

|             | Task Decomp | Dep. Modeling | Formal Verif. | LLM Safety | Dual Verif. | MCP/External | RT Monitor | Re-plan | Partial Obs. | Multi-Formal |
| ----------- | :---------: | :-----------: | :-----------: | :--------: | :---------: | :----------: | :--------: | :-----: | :----------: | :----------: |
| SMART-LLM   |     🟢      |      🔴       |      🔴       |     🔴     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| COHERENT    |     🟢      |      🔴       |      🔴       |     🔴     |     🔴      |      🔴      |     🔴     |   🟡    |      🔴      |      🔴      |
| DART-LLM    |     🟢      |      🟢       |      🔴       |     🔴     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| LaMMA-P     |     🟢      |      🔴       |      🟡       |     🔴     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| LiP-LLM     |     🟢      |      🟢       |      🔴       |     🔴     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| DEXTER-LLM  |     🟢      |      🔴       |      🔴       |     🔴     |     🔴      |      🔴      |     🟡     |   🟢    |      🟢      |      🔴      |
| SafePlan    |     🔴      |      🔴       |      🔴       |     🟢     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| VerifyLLM   |     🔴      |      🔴       |      🟢       |     🔴     |     🔴      |      🔴      |     🔴     |   🔴    |      🔴      |      🔴      |
| **SAFEMRS** |     🟢      |      🟢       |      🟢       |     🟢     |     🟢      |      🟢      |     🟢     |   🟢    |      🟢      |      🟢      |

🟢 = Full support | 🟡 = Partial | 🔴 = Not supported

---

## 3. How to Make the Contribution Substantial (Not Incremental)

> [!IMPORTANT]
> The difference between an incremental and a substantial paper is **depth, not breadth**. A paper that goes deep on one novel idea beats a paper that shallowly combines ten existing ideas. Below are concrete strategies.

---

### Strategy A: Deep-Dive on Corroborative Dual-Channel Verification (Recommended)

**Core idea**: Don't just _combine_ formal verification and LLM CoT — **prove theoretically and demonstrate empirically that neither alone is sufficient, and that their combination provides provably better safety properties**.

#### What to do:

1. **Formalize the Corroborative Fusion** as a decision-theoretic framework:
   - Define a **safety lattice** where the formal channel provides a _sound but incomplete_ safety verdict (it catches violations of specified properties but misses unspecified common-sense hazards)
   - Define the LLM channel as _complete but unsound_ (it can reason about novel hazards but may hallucinate safety)
   - Prove that the corroborative fusion is **strictly safer** than either channel alone under reasonable assumptions

2. **Introduce a formal safety metric**, e.g.:
   - **Safety Coverage** = % of hazard types detectable
   - **Safety Soundness** = % of "safe" verdicts that are actually safe
   - **Safety Completeness** = % of actually safe plans that are approved
   - Show theoretically and empirically that dual-channel dominates single-channel on the coverage-soundness Pareto frontier

3. **Create a new safety-focused benchmark** — the **Multi-Robot Safety Challenge (MRSC)**:
   - 500+ scenarios across 5 categories: (i) resource conflicts, (ii) spatial collisions, (iii) temporal deadline violations, (iv) common-sense hazards (LLM-detectable only), (v) constraint violations (formal-logic-detectable only)
   - Show that **no single-channel system** scores above 70% across all categories, while dual-channel scores 90%+
   - This benchmark would be a standalone contribution that the community adopts

4. **Ablation study** showing the failure modes of each channel in isolation:
   - Cases where formal verification says "safe" but LLM catches a common-sense hazard (e.g., "Place the knife on the edge of the table near the child")
   - Cases where LLM says "safe" but formal logic catches a temporal/resource violation (e.g., two robots scheduled for the same tool simultaneously)

> [!TIP]
> **This strategy makes the contribution non-incremental because**: (a) the theoretical framework is new, (b) the benchmark is a standalone contribution, (c) the empirical results demonstrate a capability gap that no prior work can fill.

---

### Strategy B: Agentic Planning with MCP as a New Paradigm

**Core idea**: Position the paper around a **new paradigm for grounded multi-robot planning** — the LLM doesn't just decompose tasks from its parametric knowledge; it autonomously retrieves real-world context, calls specialized tools, and refines plans through an agentic loop.

#### What to do:

1. **Formally define "Agentic Multi-Robot Planning"** as a new problem formulation:
   - Standard MRS planning assumes a fixed world model M
   - Agentic MRS planning assumes an **incomplete world model** M₀ that the planner augments through tool calls to produce M₁, M₂, …, Mₖ before planning
   - This is a **different problem class** from all prior work

2. **Design experiments that demonstrate grounding reduces hallucination**:
   - Compare: (a) LLM plans from parametric knowledge only vs. (b) LLM plans with MCP-augmented context
   - Use realistic scenarios: "Deploy robots to inspect the building" where the LLM needs to query a floor plan API, check robot battery levels via a database, and verify weather conditions via a web service
   - Measure: plan correctness, hallucination rate, task feasibility

3. **Show that MCP integration enables qualitatively new mission types**:
   - Missions that are **impossible** without external knowledge (e.g., "Evacuate the building following the fire safety plan" requires querying the building's safety protocol database)
   - Missions requiring real-time external data (e.g., "Navigate around current construction zones" requires querying a city API)

> [!TIP]
> **This strategy makes the contribution non-incremental because**: it defines a new problem class (agentic planning ≠ standard planning) and shows qualitative capabilities impossible for prior systems.

---

### Strategy C: Safety-Aware Re-Planning under Partial Observability (New Problem)

**Core idea**: Define and solve a **new problem** — how to guarantee safety when re-planning in partially observable environments where the world state is uncertain.

#### What to do:

1. **Formulate the problem mathematically**:
   - Given a POMDP (Partially Observable MDP), a set of LTL safety constraints Φ, and a set of LLM-generated sub-tasks T, find a policy π that maximizes mission completion while satisfying Φ with probability ≥ 1 − ε
   - This is a **new problem formulation** that no prior work addresses

2. **Propose a solution that alternates between**:
   - LLM-based belief state estimation (what the robots collectively believe about the world)
   - Formal verification of the plan under the current belief state
   - Re-planning when the belief state update invalidates safety constraints

3. **Benchmark on scenarios with injected partial observability**:
   - Occluded obstacles, uncertain object locations, robot sensor failures
   - Show that SAFEMRS maintains safety guarantees while DEXTER-LLM (closest competitor) does not

> [!TIP]
> **This strategy makes the contribution non-incremental because**: it identifies and solves a genuinely new problem that combines safety, re-planning, and partial observability.

---

### Strategy D: Combine A + B (Most Ambitious — Potential RA-L + IROS)

If targeting a journal paper (RA-L with IROS presentation), combine Strategies A and B:

1. **Theoretical contribution**: Corroborative safety framework (Strategy A)
2. **Systems contribution**: Agentic MCP-augmented planning pipeline (Strategy B)
3. **Benchmark contribution**: Multi-Robot Safety Challenge (from Strategy A)
4. **Empirical contribution**: Comprehensive experiments showing both safety and grounding improvements

---

## 4. Recommended Paper Structure for IROS/ICRA

Based on what gets accepted at these venues, here is a recommended structure:

```
I.    Introduction (1 page)
      - Problem: LLMs are powerful planners but unsafe + ungrounded
      - Gap: No framework combines formal safety + LLM reasoning + external grounding
      - Contribution: 3 bullet points (theory + system + benchmark)

II.   Related Work (0.75 pages)
      - LLM-based multi-robot planning (SMART-LLM, COHERENT, DART-LLM, LaMMA-P)
      - Safety in LLM planning (SafePlan, VerifyLLM)
      - Dynamic re-planning (DEXTER-LLM)
      - Clear gap statement

III.  Problem Formulation (0.75 pages)
      - Formal definition of the safety-aware agentic planning problem
      - Definitions of safety properties (LTL + deontic)
      - Complexity analysis

IV.   SAFEMRS Architecture (1.5 pages)
      - System overview (one main figure)
      - Agentic Reasoning Layer + MCP
      - Abstract Planning Layer
      - Dual-Channel Safety Verification (theoretical framework)
      - Real-Time Monitoring

V.    Experiments (1.5 pages)
      - Benchmark description (MRSC or adaptation of existing)
      - Baselines: SafePlan, VerifyLLM, COHERENT, DEXTER-LLM, LaMMA-P
      - Metrics: SR, Safety Coverage, Safety Soundness, Re-planning latency
      - Results tables + ablation study

VI.   Discussion & Conclusion (0.5 pages)
      - Key findings
      - Limitations (be honest — reviewers respect it)
      - Future work
```

**Total**: 6 pages (IROS/ICRA limit)

---

## 5. What NOT To Do (Common Rejection Reasons)

> [!WARNING]
> These are the most common reasons multi-robot LLM papers get rejected at IROS/ICRA:

| Mistake                             | Why It Kills the Paper                               | How SAFEMRS Avoids It                                                                |
| ----------------------------------- | ---------------------------------------------------- | ------------------------------------------------------------------------------------ |
| **"We combine X + Y + Z"**          | Integration ≠ contribution; reviewers want depth     | Focus on ONE deep contribution (dual safety verification) with supporting components |
| **Simulation only, no real robots** | IROS/ICRA reviewers distrust simulation-only results | Include at least a small real-robot experiment (even 2 robots doing 3 tasks)         |
| **Only GPT-4 tested**               | "What if it only works because GPT-4 is good?"       | Test on 2–3 LLMs (GPT-4, Claude, Llama-3) to show architecture-level contribution    |
| **No baselines**                    | Can't judge significance without comparison          | Compare against SafePlan, VerifyLLM, COHERENT, LaMMA-P on same benchmark             |
| **Vague safety claims**             | "Our system is safer" without quantification         | Define formal safety metrics and measure them rigorously                             |
| **Too many components, no depth**   | Breadth without depth feels like a tech report       | Cut to 4 key components; give math for the novel one                                 |
| **No failure analysis**             | Reviewers want to see where the system fails         | Include a "Limitations and Failure Cases" section                                    |

---

## 6. Concrete Action Plan

### Phase 1: Sharpen the Contribution (1–2 weeks)

- [ ] Choose Strategy A, B, or C as the **primary** contribution
- [ ] Write the formal problem definition (Section III of the paper)
- [ ] Design the corroborative safety framework with mathematical rigor
- [ ] Design the benchmark (500+ scenarios with ground-truth safety labels)

### Phase 2: Implementation (3–4 weeks)

- [ ] Implement the Agentic Reasoning Layer with MCP tool integration
- [ ] Implement the Dual-Channel Safety Verification (LTL model checking + LLM CoT)
- [ ] Implement the Abstract Planning Layer (start with PDDL + BT backends)
- [ ] Build the Real-Time Monitoring Layer
- [ ] Integrate with a multi-robot simulator (AI2-THOR or Gazebo)

### Phase 3: Experiments (2–3 weeks)

- [ ] Run benchmark experiments with 4+ baselines
- [ ] Full ablation study (remove each component independently)
- [ ] Test on 2–3 LLM backends
- [ ] Small real-robot demonstration (if feasible)
- [ ] Prepare results tables and figures

### Phase 4: Writing (1–2 weeks)

- [ ] Draft following the recommended structure in §4
- [ ] Internal review and iteration
- [ ] Final polish and submission

---

## 7. Final Recommendation

> [!IMPORTANT]
> **Go with Strategy A (Dual-Channel Corroborative Safety Verification) as the lead contribution.**
>
> **Reasons**:
>
> 1. **Safety is the hottest topic** in LLM robotics right now — both SafePlan and VerifyLLM appeared in 2025, signaling strong community interest.
> 2. **No one has combined formal + LLM safety** — this is genuinely novel, not integration of existing systems (SafePlan has no formal logic; VerifyLLM has no LLM safety reasoning).
> 3. **It's provable** — you can provide theoretical guarantees about the corroborative framework, which IROS/ICRA reviewers value.
> 4. **The benchmark is a standalone contribution** — even if the architecture is questioned, a high-quality multi-robot safety benchmark will be valued.
> 5. **MCP integration (Strategy B) can be the supporting novelty** — mention it, demonstrate it, but don't make it the headline.

The paper title could be:

> **"Corroborative Safety Verification for LLM-Based Multi-Robot Task Planning: Combining Formal Logic and Probabilistic Reasoning"**

This title signals: (1) a new verification approach, (2) it's for multi-robot systems, (3) it combines two distinct paradigms — exactly what a reviewer wants to see.
