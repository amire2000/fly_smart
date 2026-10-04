"""Command-line entry point for the offline flight analyzer."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tkinter as tk

from .loader import AnalysisLoadError
from .viewer import AnalyzerApplication


def main(argv: list[str] | None = None) -> int:
    """Start the analyzer, optionally using a supplied run folder."""
    parser = argparse.ArgumentParser(description="Review a recorded Fly Smart flight attempt")
    parser.add_argument("run_folder", nargs="?", type=Path, help="run folder or direct attempt folder")
    args = parser.parse_args(argv)
    run_folder = args.run_folder
    root = tk.Tk()
    if run_folder is None:
        from tkinter.filedialog import askdirectory

        selected = askdirectory(initialdir=str(Path("outputs/ttc_runs")), title="Select Fly Smart run folder")
        if not selected:
            root.destroy()
            return 0
        run_folder = Path(selected)
    try:
        AnalyzerApplication(root, run_folder)
    except AnalysisLoadError as exc:
        root.destroy()
        print(f"offline analyzer error: {exc}", file=sys.stderr)
        return 1
    root.mainloop()
    return 0
