# SAFEMRS: Safe Agentic Framework for Externally-augmented Multi-Robot Systems

> A Novel Architecture for LLM-Based Heterogeneous Multi-Robot Task Planning with Formal Safety Verification, Agentic Reasoning, and Real-Time Monitoring

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-Research-orange.svg)]()

---

## 📋 Project Overview

**SAFEMRS** is a research project aimed at publication at IROS/ICRA conferences, focusing on LLM-based heterogeneous multi-robot task planning with formal safety verification.

### Key Features

- 🧠 **Agentic Reasoning Layer**: LLM-driven autonomous task decomposition with Chain-of-Thought (CoT)
- 🔌 **MCP Integration**: External tool and knowledge base access via Model Context Protocol
- 🛡️ **Dual-Channel Safety Verification**: Combines formal logic (LTL/CTL/Deontic) with probabilistic LLM reasoning
- 📡 **Real-Time Monitoring**: Continuous state aggregation with anomaly detection and dynamic re-planning
- 🤖 **Multi-Formalism Planning**: Abstract layer supporting PDDL, Behavior Trees, DAG, HTN, and JSON/YAML

---

## 📁 Repository Structure

```
SAFEMRS IROS Paper/
├── README.md                    # This file
├── .gitignore                   # Git ignore patterns
├── update_repo.sh               # Automated update script
└── proposal/
    ├── README.md                # Proposal overview
    ├── architecture_proposal.md # Complete architecture design (433 lines)
    ├── competitive_analysis.md  # Gap analysis & strategy (360 lines)
    ├── literature_summary.md    # 16 papers analyzed (327 lines)
    ├── images/                  # Diagrams and figures
    │   ├── idea.drawio
    │   └── improved.drawio
    ├── literature/
    │   ├── pdf/                 # Source papers (16 PDFs)
    │   └── txt/                 # Extracted plain-text versions
    └── scripts/
        └── extract_pdfs.py      # PDF → TXT extraction script
```

---

## 🚀 Getting Started

### Prerequisites

- Python 3.8+
- Git
- PyMuPDF for PDF extraction: `pip install pymupdf`

### Clone the Repository

```bash
git clone https://github.com/asmbatati/SAFEMRS.git
cd SAFEMRS
```

### Authentication Setup

Since this repository requires authentication, you have two options:

#### Option 1: Using Personal Access Token (Recommended)

1. Go to GitHub Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate new token with `repo` scope
3. Update the remote URL:
   ```bash
   git remote set-url origin https://YOUR_TOKEN@github.com/asmbatati/SAFEMRS.git
   ```

#### Option 2: Using SSH

1. Set up SSH keys for GitHub ([GitHub SSH Setup](https://docs.github.com/en/authentication/connecting-to-github-with-ssh))
2. Update the remote URL:
   ```bash
   git remote set-url origin git@github.com:asmbatati/SAFEMRS.git
   ```

---

## 🔄 Using the Update Script

The `update_repo.sh` script automates the process of adding, committing, and pushing changes:

### Basic Usage

```bash
# With custom commit message
./update_repo.sh "Your commit message here"

# Without message (uses timestamp)
./update_repo.sh
```

### First-Time Setup

After setting up authentication, run:
```bash
chmod +x update_repo.sh
./update_repo.sh "Initial setup complete"
```

---

## 📊 Project Status

**Status**: Planning/Design Phase Complete - Ready for Implementation

### Completed ✅

- [x] Comprehensive literature review (16 papers, 2020-2025)
- [x] Complete architecture design (4-layer framework)
- [x] Competitive analysis with 8 closest competitors
- [x] Research questions and evaluation plan defined
- [x] Paper structure proposed (6-page IROS/ICRA format)

### In Progress 🔄

- [ ] Formalize corroborative safety framework mathematically
- [ ] Design Multi-Robot Safety Challenge (MRSC) benchmark
- [ ] Implement Agentic Reasoning Layer with MCP integration
- [ ] Implement Dual-Channel Safety Verification
- [ ] Build Abstract Planning Layer
- [ ] Create Real-Time Monitoring Layer

### Planned 📋

- [ ] Simulation integration (AI2-THOR or Gazebo)
- [ ] Benchmark experiments with baselines
- [ ] Ablation studies
- [ ] Real-robot demonstration
- [ ] Paper writing and submission

---

## 🎯 Core Novelties

### 1. Dual-Channel Safety Verification
First framework to combine **formal logic verification** (LTL/CTL model checking) with **probabilistic LLM safety reasoning** (CoT invariant checking) via corroborative fusion.

### 2. Agentic Reasoning with MCP
Novel paradigm where the LLM autonomously retrieves real-world context, calls specialized tools, and refines plans through external augmentation - reducing hallucination.

### 3. Real-Time Monitoring Loop
Structured monitoring layer that continuously fuses robot telemetry with environmental data and triggers verified re-planning.

### 4. Abstract Multi-Formalism Planning
Technology-agnostic interface supporting multiple planning formalisms (PDDL, BT, DAG, HTN, YAML) - enabling optimal formalism selection per task.

---

## 📚 Related Work

Our framework builds upon and extends:

- **SMART-LLM** (Kannan et al., 2024) - Task decomposition
- **COHERENT** (Liu et al., 2025) - Multi-robot coordination
- **DART-LLM** (Wang et al., 2025) - Dependency modeling
- **LaMMA-P** (Zhang et al., 2025) - LLM + PDDL integration
- **SafePlan** (Obi et al., 2025) - LLM-based safety screening
- **VerifyLLM** (Grigorev et al., 2025) - Formal verification
- **DEXTER-LLM** (Zhu et al., 2025) - Dynamic re-planning

See [literature_summary.md](proposal/literature_summary.md) for detailed analysis.

---

## 📖 Documentation

- **[Architecture Proposal](proposal/architecture_proposal.md)**: Complete system architecture with Mermaid diagrams
- **[Competitive Analysis](proposal/competitive_analysis.md)**: Gap analysis and contribution strategy
- **[Literature Summary](proposal/literature_summary.md)**: Thematic analysis of 16 papers (2020-2025)

---

## 🤝 Contributing

This is an active research project. For questions or collaboration inquiries, please contact the research team.

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 📧 Contact

- **Research Supervisor**: Prof. Anis Koubaa
- **Repository Owner**: asmbatati
- **Collaborator**: Anis Koubaa (aniskoubaa)
- **Project Affiliation**: ScaleX Research Lab

---

## 🔗 Links

- [Architecture Documentation](proposal/architecture_proposal.md)
- [Literature Review](proposal/literature_summary.md)
- [Competitive Analysis](proposal/competitive_analysis.md)

---

**Last Updated**: February 15, 2026
