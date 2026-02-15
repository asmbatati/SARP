# IROS Paper

Research project on LLM-driven multi-robot task planning.

## Repository Structure

```
IROS Paper/
├── README.md
├── images/              # Diagrams and figures
├── literature/
│   ├── pdf/             # Source papers (16 PDFs)
│   └── txt/             # Extracted plain-text versions
└── scripts/
    └── extract_pdfs.py  # PDF → TXT extraction script
```

## Extracting Literature to Text

```bash
# 1. Install dependency
pip install pymupdf

# 2. Run extraction (from the repo root)
python scripts/extract_pdfs.py
```

The script reads every PDF in `literature/pdf/` and writes a corresponding `.txt` file into `literature/txt/`.
