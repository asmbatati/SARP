#!/usr/bin/env python3
"""
extract_pdfs.py
---------------
Extract text from all PDF files in literature/pdf/ and save
each as a plain-text file in literature/txt/.

Requirements:
    pip install pymupdf

Usage:
    python scripts/extract_pdfs.py
"""

import os
import sys
from pathlib import Path

try:
    import fitz  # PyMuPDF
except ImportError:
    sys.exit(
        "ERROR: PyMuPDF is not installed.\n"
        "Install it with:  pip install pymupdf"
    )

# ── paths (relative to repo root) ──────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
PDF_DIR = REPO_ROOT / "literature" / "pdf"
TXT_DIR = REPO_ROOT / "literature" / "txt"


def extract_text(pdf_path: Path) -> str:
    """Return the full text of a PDF, concatenated page-by-page."""
    doc = fitz.open(pdf_path)
    pages = []
    for page_num, page in enumerate(doc, start=1):
        text = page.get_text()
        pages.append(f"--- Page {page_num} ---\n{text}")
    doc.close()
    return "\n\n".join(pages)


def main() -> None:
    if not PDF_DIR.exists():
        sys.exit(f"ERROR: PDF directory not found: {PDF_DIR}")

    TXT_DIR.mkdir(parents=True, exist_ok=True)

    pdf_files = sorted(PDF_DIR.glob("*.pdf"))
    if not pdf_files:
        sys.exit(f"No PDF files found in {PDF_DIR}")

    print(f"Found {len(pdf_files)} PDF(s) in {PDF_DIR}\n")

    success, failed = 0, 0

    for pdf_path in pdf_files:
        txt_name = pdf_path.stem + ".txt"
        txt_path = TXT_DIR / txt_name

        try:
            text = extract_text(pdf_path)
            txt_path.write_text(text, encoding="utf-8")
            size_kb = txt_path.stat().st_size / 1024
            print(f"  ✓  {pdf_path.name}  →  {txt_name}  ({size_kb:.1f} KB)")
            success += 1
        except Exception as exc:
            print(f"  ✗  {pdf_path.name}  — {exc}")
            failed += 1

    print(f"\nDone.  {success} succeeded, {failed} failed.")


if __name__ == "__main__":
    main()
