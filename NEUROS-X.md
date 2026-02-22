# NEUROS-X
## Neuro-Symbolic Nexus — Robotics AI Research Organization

> **Mission:** Design and build the universal cognitive middleware layer between humans and heterogeneous robotic systems — abstracting robot middleware complexity so that any user can command, coordinate, and optimize any robot fleet through natural language and multi-modal intent alone.

---

## 1. Vision

### The Core Idea

NEUROS-X builds a **two-sided installable middleware stack**:

- **User Side** — A cognitive HRI interface (web app / CLI / voice) where the user describes missions in natural language, images, or structured constraints
- **Robot Side** — A lightweight agent node installed on each robot, advertising its capabilities, receiving allocated tasks, and executing them safely

Once configured, the user **only provides intent**. The system handles:
- Mission parsing and semantic grounding
- Task decomposition and multi-robot allocation
- Safety verification (pre-execution + runtime)
- Middleware translation (ROS 2, gRPC, MAVLink, micro-ROS)
- Closed-loop monitoring and failure recovery

### Why It Matters

Modern robotic systems require deep expertise in ROS, DDS, hardware drivers, safety protocols, and multi-robot coordination. NEUROS-X eliminates this barrier, enabling domain experts (firefighters, surgeons, facility managers) to operate complex heterogeneous fleets without robotics engineering knowledge.

### Design Philosophy: Neuro-Symbolic Nexus

The name and identity encode the two complementary paradigms at the system's core:

| Symbol | Meaning |
|--------|---------|
| **Neuro** | Neural / LLM reasoning — flexible, commonsense-capable, multi-modal |
| **Symbolic** | Formal logic — sound, verifiable, constraint-enforcing |
| **Nexus** | The fusion point — where both paradigms produce a unified, safe decision |

The logo concept: **organic neural network complexity enclosed within a geometric shield / logic gate** — representing the union of adaptive intelligence and formal rigor.

---

## 2. Five-Tier Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  TIER 1: Semantic Intent Layer                                    │
│  Multi-modal input (text, voice, image, .md mission brief)       │
│  VLM grounding → formal mission specification (JSON/PDDL)        │
│  3D Scene Graph construction · Robot Resume registry             │
├──────────────────────────────────────────────────────────────────┤
│  TIER 2: Cognitive Orchestration Layer                           │
│  LLM reasoning · Hierarchical Task Network (HTN) decomposition   │
│  8 planning backends: PDDL, BT, DAG, HTN, STL, FSM, Code, YAML  │
│  Task allocation via Robot Resume matching (URDF-derived)        │
├──────────────────────────────────────────────────────────────────┤
│  TIER 3: Triple-Channel Safety Verification  ← PRIMARY CONTRIB   │
│  Channel 1 (Formal): LTL model checking + PDDL preconditions     │
│  Channel 2 (LLM CoT): 4 sub-reasoners, independent verdict       │
│  Channel 3 (Runtime): CBF barrier monitoring + conformal bounds  │
│  Corroborative fusion: Approve / Reject / Review (human-in-loop) │
├──────────────────────────────────────────────────────────────────┤
│  TIER 4: Tool Mediation & Skill Ontology Layer                   │
│  MCP tool invocation · Robot Skill Ontology                      │
│  VLA execution bridge (symbolic plan → motor tokens)             │
│  Trigger-based expert agent loading (60-80% resource savings)    │
├──────────────────────────────────────────────────────────────────┤
│  TIER 5: Heterogeneous HAL + Real-Time Monitoring               │
│  Universal middleware bridge: ROS 2, ROS 1, gRPC, MAVLink        │
│  PEFA loop: Proposal → Execution → Feedback → Adjust            │
│  Execution tracker · Failure detection · Re-planning trigger     │
└──────────────────────────────────────────────────────────────────┘
```

**Research Papers by Tier:**

| Tier | NEUROS-X Component | First Paper | Venue Target |
|------|-------------------|-------------|-------------|
| Tier 3 | Triple-Channel Safety Verification | **SAFEMRS** (dual-channel) | IROS 2026 |
| Tier 2 | Cognitive Orchestration + Multi-Robot Planning | Future paper | ICRA 2027 |
| Tier 1 | Semantic Intent Grounding (VLM + Scene Graph) | Future paper | RA-L |
| Tier 4 | MCP Tool Mediation + VLA Bridge | Future paper | IROS 2027 |
| Tier 5 | HAL + PEFA Closed-Loop Monitoring | Future paper | ICRA 2028 |

---

## 3. GitHub Organization Structure

### Organization: `github.com/NEUROS-X`

All repositories migrated from `github.com/asmbatati` to the NEUROS-X organization upon public release.

---

### Repository Map

```
NEUROS-X/
│
├── neuros-core/              ← Full 5-tier cognitive middleware (long-term goal)
│
├── neuros-safemrs/           ← PAPER 1: Tier 3 dual-channel safety verification
├── neuros-sim/               ← Simulation worlds + multi-robot descriptions
├── neuros-docker/            ← Docker infrastructure + CI/CD
│
├── neuros-agent/             ← ROS 2 LLM agent node (ROSA + LangChain + tools)
├── neuros-gui/               ← Web simulation interface (React + FastAPI)
├── neuros-demos/             ← Mission demonstrations (SAR, inspection, patrol)
├── neuros-bridge/            ← Middleware HAL (ROS 2, MAVLink, gRPC, micro-ROS)
├── neuros-benchmarking/      ← Evaluation & metrics framework
│
├── neuros-planning/          ← Cognitive orchestration layer (future ICRA 2027)
├── neuros-grounding/         ← Semantic intent layer (future RA-L)
│
└── neuros-x.github.io/       ← Organization website
```

---

## 4. Repository Specifications

### 4.1 `neuros-safemrs` — Core Safety Verification Framework

> **Status:** Active — IROS 2026 paper underway
> **Maps to:** NEUROS-X Tier 3

The primary research contribution: corroborative dual-channel (→ triple-channel) pre-execution safety verification for LLM-based heterogeneous multi-robot task planning.

**Current state:** Python package (`safemrs/`) with 102-scenario benchmark, 51 passing unit tests, and confirmed results for Qwen3:8b and GPT-4o backends.

**Package structure:**

```
neuros-safemrs/                   (repo root)
├── safemrs/                      (Python package, pip-installable)
│   ├── channel_formal/           Channel 1: LTL + PDDL + Deontic
│   │   ├── ltl_checker.py
│   │   ├── pddl_validator.py
│   │   └── deontic_checker.py
│   ├── channel_llm/              Channel 2: 4 LLM sub-reasoners
│   │   ├── base_reasoner.py
│   │   ├── safety_reasoner.py
│   │   └── sub_reasoners/
│   ├── fusion/                   Corroborative fusion mechanism
│   │   └── fusion.py             Approve/Reject/Review logic
│   ├── benchmark/                102 YAML scenarios + evaluator
│   │   ├── scenarios/
│   │   └── evaluator.py          HDR, FPR, EffCov, ΔC metrics
│   └── ros2_integration/         ROS 2 node + SafetyGate
│       └── safety_gate.py
├── experiments/                  Experiment runners + analysis scripts
│   ├── reproduce.sh
│   ├── run_llm_background.py
│   ├── check_progress.py
│   └── analyze_results.py
├── tests/                        51 unit tests (pytest)
├── results/final/                Archived experiment CSVs
│   ├── formal_only_qwen3:8b.csv
│   ├── llm_only_qwen3:8b.csv
│   ├── dual_qwen3:8b.csv
│   ├── formal_only_gpt-4o.csv
│   ├── llm_only_gpt-4o.csv
│   └── dual_gpt-4o.csv
├── latex/                        IROS 2026 paper (main.tex)
└── pyproject.toml
```

**Key metrics (confirmed):**

| Backend | HDR (Dual) | FPR (Dual) | EffCov | Latency |
|---------|:---:|:---:|:---:|:---:|
| Qwen3:8b (local) | 64.2% | **0.0%** | 87.7% | 69.3s |
| GPT-4o (cloud) | 75.5% | 2.0% | 96.1% | 5.2s |

---

### 4.2 `neuros-sim` — Multi-Robot Simulation Environments

> **Status:** Partially active (extracted from SAFEMRS)
> **Maps to:** NEUROS-X Tier 5 (execution platform)

Gazebo Harmonic worlds, robot URDF/SDF descriptions, and ROS 2 launch orchestration for all NEUROS-X robot types. Kept simulation-agnostic from `neuros-safemrs` core.

**Supported robot types:**

| Robot | Type | Middleware | Status |
|-------|------|-----------|--------|
| Unitree Go2 | Quadruped | CHAMP + cmd_vel + EKF | ✅ Active |
| PX4 x500 | UAV (multirotor) | MAVROS + XRCE-DDS | ✅ Active |
| Clearpath Husky | Wheeled UGV | cmd_vel + Nav2 | 🟡 Planned |
| TurtleBot 4 | Wheeled UGV | cmd_vel + Nav2 | 🟡 Planned |
| Unitree H1 | Humanoid | whole-body controller | 🟠 Future |
| UR5e | Arm manipulator | MoveIt 2 + ros2_control | 🟠 Future |
| Franka Panda | Arm manipulator | MoveIt 2 + ros2_control | 🟠 Future |
| BlueROV2 | Marine AUV | MAVROS + ArduSub | 🔵 Concept |

**Package structure:**

```
neuros-sim/
├── bringup/                      ROS 2 launch orchestration
│   ├── launch/
│   │   ├── sar_full.launch.py    Go2 + PX4 drone, shared world
│   │   ├── inspection.launch.py  Drone (exterior) + Go2 (interior)
│   │   ├── patrol.launch.py      Multi-UGV facility patrol
│   │   └── evaluation.launch.py  Benchmarking setup
│   ├── worlds/
│   │   ├── sar_default.sdf       Default outdoor SAR world (current)
│   │   ├── sar_inspection.sdf    Building inspection world (needed)
│   │   ├── sar_wilderness.sdf    Outdoor wilderness (future)
│   │   └── warehouse.sdf         Indoor manipulation (future)
│   └── config/
│       ├── rviz/neuros.rviz
│       └── bridge_topics.yaml
├── robots/                       Per-robot ROS 2 packages
│   ├── quadruped/
│   │   ├── unitree_go2_description/   Go2 URDF + meshes
│   │   └── unitree_go2_sim/           CHAMP controller config
│   ├── uav/
│   │   ├── px4_x500_description/      PX4 x500 SDF + configs
│   │   └── px4_models/                PX4 aircraft configs (4020, 4021, 4022)
│   ├── ugv/
│   │   ├── clearpath_husky_description/   Husky URDF + meshes (planned)
│   │   └── turtlebot4_description/        TurtleBot 4 URDF (planned)
│   ├── humanoid/
│   │   └── unitree_h1_description/        H1 URDF + meshes (future)
│   └── manipulator/
│       ├── ur5e_description/              UR5e URDF (future)
│       └── franka_description/            Franka Panda URDF (future)
└── drone_sim/                    PX4 SITL + MAVROS launch
    ├── launch/
    │   ├── drone.launch.py
    │   ├── gz_sim.launch.py
    │   └── mavros.launch.py
    └── mavros/
```

---

### 4.3 `neuros-docker` — Docker & Infrastructure

> **Status:** Active (submodule under SAFEMRS)

Reproducible development environment: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL + XRCE-DDS + MAVROS.

**Design principles:**
- Ollama runs **on the host** (not in Docker) — avoids 20+ GB in image
- Model weights never baked into image — pulled at runtime
- Shared volume for workspace source + PX4

```
neuros-docker/
├── docker/
│   ├── Dockerfile.base     ROS 2 + Gazebo + PX4 + MAVROS + Python deps (~8-10 GB)
│   └── Dockerfile.dev      Extends base + VS Code + RQt + debug tools
├── docker-compose.yml      Multi-service: neuros + ollama (optional)
├── scripts/
│   ├── docker_run.sh
│   ├── entrypoint.sh
│   ├── install.sh
│   └── install_ollama.sh   Separate: model weights download
├── requirements/
│   ├── requirements.txt
│   └── system_packages.txt
└── ci/
    ├── build.yml
    └── test.yml
```

---

### 4.4 `neuros-agent` — ROS 2 LLM Agent Node

> **Status:** Active (extracted from `ros2_agent_sim/ros2_agent`)
> **Maps to:** NEUROS-X Tier 2 + Tier 4

Standalone ROS 2 node powering natural language robot interaction. ROSA (NASA JPL) + LangChain dispatch tool calls to any connected robot, with the SAFEMRS safety gate as pre-execution middleware.

```
neuros-agent/
├── neuros_agent/
│   ├── agent_node.py         Main node: LangChain + ROSA + safety gate
│   ├── safety/
│   │   └── safety_gate.py    SAFEMRS integration (dual/formal/passthrough)
│   ├── tools/
│   │   ├── drone_tools.py    7 UAV tools via MAVROS
│   │   ├── go2_tools.py      8 Go2 tools via cmd_vel/odom
│   │   ├── manipulation_tools.py  MoveIt 2 arm tools (future)
│   │   └── vlm_tools.py      VLM scene analysis (Qwen2.5VL)
│   ├── prompts/
│   │   └── system_prompt.py  Robot-aware system prompt
│   └── robot_resume/
│       └── resume_loader.py  Capability profile loader
├── config/agent_params.yaml
├── launch/agent.launch.py
└── setup.py
```

**Key ROS 2 parameters:**

| Parameter | Default | Options |
|-----------|---------|---------|
| `llm_model` | `qwen3:8b` | `gpt-4o`, `claude-3-5-haiku`, `llama3.1:8b` |
| `safety_mode` | `dual` | `dual`, `formal_only`, `passthrough` |
| `ollama_base_url` | `http://host.docker.internal:11434` | Any Ollama endpoint |

---

### 4.5 `neuros-gui` — Web Simulation Interface

> **Status:** Active (extracted from `ros2_agent_sim/simulation_gui`)
> **Maps to:** NEUROS-X Tier 1 (intent input) + Tier 5 (monitoring)

React + FastAPI web application for scenario building, robot spawning, mission dispatch, and live safety monitoring.

```
neuros-gui/
├── frontend/                 React + Vite + Tailwind + Framer Motion
│   └── src/components/
│       ├── ScenarioBuilder.jsx   Drag-drop robot + asset placement
│       ├── MissionPanel.jsx      NL command input → agent dispatch
│       ├── SafetyDashboard.jsx   Live Approve/Reject/Review log
│       └── RobotMonitor.jsx      Per-robot pose + battery + status
├── backend/                  FastAPI
│   ├── main.py               REST endpoints + ROS 2 bridge
│   ├── scenario_generator.py Auto-generate launch files from GUI state
│   └── ros2_bridge.py        Subscribe to safety gate + robot topics
└── docker-compose.gui.yml
```

---

### 4.6 `neuros-demos` — Mission Demonstrations

> **Status:** Active (extracted from `ros2_agent_sim/sar_system`)
> **Maps to:** NEUROS-X Tier 5 (execution platform)

Self-contained preconfigured multi-robot mission scenarios. Each demo ships with its own launch file, Gazebo world config, documented ROSA prompts, and expected safety gate behavior.

```
neuros-demos/
├── sar/                      Search & Rescue (Go2 + PX4 drone)
│   ├── launch/sar.launch.py
│   └── README.md             Prompts + expected safety gate responses
├── inspection/               Building inspection (drone + Go2)
│   ├── launch/inspection.launch.py
│   └── README.md
├── patrol/                   Multi-UGV facility patrol
│   ├── launch/patrol.launch.py
│   └── README.md
├── manipulation/             Table-top + mobile base (future)
└── shared/
    ├── evaluation.launch.py  RTF + performance logger
    └── gps_bridge/           C++ GPS relay (shared dependency)
```

---

### 4.7 `neuros-bridge` — Middleware Abstraction Layer

> **Status:** Planned (after SAFEMRS submission)
> **Maps to:** NEUROS-X Tier 5

Universal robot action interface: one API, any middleware. Converts `move_to()`, `get_pose()`, `stop()` to the appropriate protocol. Also generates Robot Resumes (capability profiles) from URDF/SDF for task allocation.

```
neuros-bridge/
├── neuros_bridge/
│   ├── interface.py          Abstract base: move_to(), get_pose(), stop(), status()
│   ├── resume.py             URDF/SDF → Robot Resume (capability profile)
│   └── adapters/
│       ├── ros2_adapter.py   cmd_vel + action servers (UGV / quadruped)
│       ├── mavros_adapter.py MAVROS service calls (UAV)
│       ├── mavlink_adapter.py Direct MAVLink (companion-less UAV)
│       ├── grpc_adapter.py   gRPC robot API (future)
│       └── microros_adapter.py micro-ROS embedded (future)
├── tests/
└── setup.py
```

---

### 4.8 `neuros-benchmarking` — Evaluation Framework

> **Status:** Planned (metrics generalized from `neuros-safemrs`)
> **Maps to:** Cross-cutting — supports all NEUROS-X papers

Standalone evaluation library for safety, planning, and execution quality across all NEUROS-X experiments. Generalizes the SAFEMRS benchmark runner into a reusable framework.

```
neuros-benchmarking/
├── neuros_benchmarking/
│   ├── metrics/
│   │   ├── safety_metrics.py    HDR, FPR, EffCov, ΔC (from SAFEMRS)
│   │   ├── planning_metrics.py  Task success, makespan, resource util
│   │   └── execution_metrics.py RTF, latency, failure rate
│   ├── scenarios/
│   │   ├── safety/              102 SAFEMRS YAML scenarios (migrated)
│   │   ├── planning/            Multi-robot allocation scenarios (future)
│   │   └── execution/           End-to-end mission scenarios (future)
│   ├── runner.py                Batch experiment runner (CLI)
│   └── reporter.py              CSV → matplotlib figures + LaTeX tables
├── experiments/
└── setup.py
```

---

### 4.9 `neuros-planning` — Cognitive Orchestration (Future)

> **Status:** Planned — ICRA 2027 paper

HTN/PDDL planning with multi-robot task allocation, multi-formalism support (PDDL, BT, DAG, HTN, STL, FSM), and coalition formation.

---

### 4.10 `neuros-grounding` — Semantic Intent Layer (Future)

> **Status:** Planned — RA-L paper

VLM-based multi-modal intent parsing, 3D scene graph construction, and formal mission specification generation from natural language + images.

---

## 5. Technology Stack

| Layer | Technology |
|-------|-----------|
| **LLM Inference (local)** | Ollama + Qwen3:8b (planning), Qwen2.5VL:7b (vision) |
| **LLM Inference (cloud)** | OpenAI GPT-4o, Anthropic Claude (comparison) |
| **LLM Framework** | LangChain + ROSA (NASA JPL) |
| **Formal Verification** | Python LTL/PDDL (production), Spot library (optional) |
| **Robot Middleware** | ROS 2 Jazzy (primary), MAVROS, XRCE-DDS |
| **Simulation** | Gazebo Harmonic, PX4 SITL |
| **UAV Stack** | PX4 → XRCE-DDS → MAVROS → ROS 2 |
| **UGV Stack** | CHAMP → ros2_control → EKF → ROS 2 |
| **Planning** | unified-planning (PDDL), py-trees (BT), custom HTN |
| **Safety Runtime** | Control Barrier Functions (CBF), conformal prediction |
| **GUI** | React + Vite + Tailwind + Framer Motion + FastAPI |
| **CI/CD** | GitHub Actions + pytest (51 tests) |
| **Containerization** | Docker + docker-compose (no model weights baked in) |
| **Evaluation** | Custom benchmark YAML + CSV pipeline + matplotlib |

---

## 6. Research Roadmap

### Active: SAFEMRS (IROS 2026 — deadline March 2, 2026)

**Paper:** SAFEMRS: Corroborative Dual-Channel Pre-Execution Safety Verification for LLM-Based Heterogeneous Multi-Robot Task Planning

**Status:** ✅ Experiments complete · ✅ LaTeX filled · ⏳ Simulation figures needed · ⏳ Submission pending

See [`remaining_tasks.md`](remaining_tasks.md) for detailed author task distribution.

---

### Next: SAFEMRS+ / Triple-Channel (IROS 2027)

Extend dual-channel to full triple-channel by adding:
- **Channel 3:** Runtime CBF enforcement (continuous, not just pre-execution)
- Conformal prediction for calibrated uncertainty bounds on safety decisions
- PEFA closed-loop re-planning on runtime violation detection

---

### Future: Cognitive Orchestration Paper (ICRA 2027)

Full Tier 2 implementation:
- LLM-driven HTN decomposition with hardware-aware allocation
- Robot Resume generation from URDF
- Multi-formalism planning (PDDL, BT, DAG)
- Multi-agent coordination (per-robot sub-agents + meta-coordinator)

---

## 7. Current Repository Mapping

| Current Repo / Path | Target NEUROS-X Repo | Transfer When |
|---------------------|---------------------|---------------|
| `asmbatati/SAFEMRS` → `safemrs/` Python pkg | `NEUROS-X/neuros-safemrs` | After IROS submission |
| `asmbatati/SAFEMRS` → `latex/` | Archived inside `NEUROS-X/neuros-safemrs` | After IROS submission |
| `asmbatati/SAFEMRS` → `safemrs_sim/` | `NEUROS-X/neuros-sim` | After IROS submission |
| `asmbatati/ros2_agent_sim_docker` | `NEUROS-X/neuros-docker` | After IROS submission |
| `asmbatati/ros2_agent_sim` → `ros2_agent/` | `NEUROS-X/neuros-agent` | After IROS submission |
| `asmbatati/ros2_agent_sim` → `simulation_gui/` | `NEUROS-X/neuros-gui` | After IROS submission |
| `asmbatati/ros2_agent_sim` → `sar_system/` | `NEUROS-X/neuros-demos` | After IROS submission |
| *(new)* | `NEUROS-X/neuros-bridge` | Phase 2 |
| *(new)* | `NEUROS-X/neuros-benchmarking` | Phase 2 |

---

## 8. Installation (Future Unified Setup)

```bash
# 1. Pull Docker environment
git clone https://github.com/NEUROS-X/neuros-docker.git

# 2. Clone workspace repos into shared volume
mkdir -p ~/neuros_ws/src && cd ~/neuros_ws/src
git clone https://github.com/NEUROS-X/neuros-safemrs.git
git clone https://github.com/NEUROS-X/neuros-sim.git
git clone https://github.com/NEUROS-X/neuros-agent.git
git clone https://github.com/NEUROS-X/neuros-demos.git

# 3. Install Ollama on HOST (never inside Docker)
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull qwen3:8b
ollama pull qwen2.5vl:7b

# 4. Launch Docker environment
cd ~/neuros-docker && docker compose up -d

# 5. Enter container, build workspace
docker exec -it neuros bash
cd ~/neuros_ws && colcon build --symlink-install && source install/setup.bash

# 6. Install neuros-safemrs Python package
pip install -e src/neuros-safemrs/safemrs/

# 7. Launch a demo (e.g. SAR)
ros2 launch neuros_demos sar.launch.py

# 8. Launch neuros-agent with dual-channel safety gate
ros2 launch neuros_agent agent.launch.py \
  llm_model:=qwen3:8b safety_mode:=dual

# 9. (Optional) Launch web GUI
cd ~/neuros_ws/src/neuros-gui && docker compose -f docker-compose.gui.yml up
```

---

*NEUROS-X — Neuro-Symbolic Nexus | Prince Sultan University Robotics & IoT Lab*
*Organization GitHub: https://github.com/NEUROS-X*
*First paper: https://github.com/asmbatati/SAFEMRS (→ NEUROS-X/neuros-safemrs)*
