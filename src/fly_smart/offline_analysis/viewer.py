"""Tkinter desktop composition for offline flight analysis."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk
from pathlib import Path

import cv2
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from .loader import AnalysisLoadError, OfflineAttempt, discover_attempts, load_attempt
from .plots import GraphPanelRegistry
from .playback import PlaybackController, TimelineMapper
from .state import FlightStatePresenter
from .video import NoVideoSource, OpenCvVideoSource, VideoSource


class AnalyzerApplication:
    """Compose the offline analyzer components and bind their UI events."""

    def __init__(self, root: tk.Tk, run_folder: Path) -> None:
        self.root = root
        self.run_folder = run_folder
        self.attempts = discover_attempts(run_folder)
        self.attempt: OfflineAttempt | None = None
        self.video: VideoSource = NoVideoSource()
        self.playback: PlaybackController | None = None
        self.presenter = FlightStatePresenter()
        self.graphs: GraphPanelRegistry | None = None
        self._image: ImageTk.PhotoImage | None = None
        self._updating_timeline = False

        self.root.title("Fly Smart offline flight analyzer")
        self.root.geometry("1500x950")
        self._build_layout()
        self._populate_attempts()
        self._load_attempt(0)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(30, self._tick)

    def _build_layout(self) -> None:
        """Create controls, video/state area, graph toggles, and canvas."""
        toolbar = ttk.Frame(self.root, padding=6)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="Choose output folder", command=self.choose_run_folder).pack(side=tk.LEFT)
        ttk.Label(toolbar, text="Attempt:").pack(side=tk.LEFT, padx=(12, 4))
        self.attempt_var = tk.StringVar()
        self.attempt_menu = ttk.Combobox(toolbar, textvariable=self.attempt_var, state="readonly", width=18)
        self.attempt_menu.pack(side=tk.LEFT)
        self.attempt_menu.bind("<<ComboboxSelected>>", lambda _event: self._load_attempt(self.attempt_menu.current()))
        self.play_button = ttk.Button(toolbar, text="Play", command=self.toggle_play)
        self.play_button.pack(side=tk.LEFT, padx=(16, 2))
        ttk.Button(toolbar, text="◀ Step", command=lambda: self.step(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Step ▶", command=lambda: self.step(1)).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Speed:").pack(side=tk.LEFT, padx=(12, 4))
        self.speed_var = tk.StringVar(value="1.0×")
        speed_menu = ttk.Combobox(toolbar, textvariable=self.speed_var, state="readonly", values=("0.25×", "0.5×", "1.0×", "2.0×"), width=7)
        speed_menu.pack(side=tk.LEFT)
        speed_menu.bind("<<ComboboxSelected>>", lambda _event: self._set_speed())
        self.time_var = tk.StringVar(value="0.000 s")
        ttk.Label(toolbar, textvariable=self.time_var).pack(side=tk.RIGHT)

        self.timeline = ttk.Scale(self.root, from_=0, to=1, command=self._seek_from_slider)
        self.timeline.pack(fill=tk.X, padx=8)

        body = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=3)

        self.video_label = ttk.Label(left, text="No movie loaded", anchor=tk.CENTER)
        self.video_label.pack(fill=tk.BOTH, expand=True)
        self.state_text = tk.StringVar(value="No attempt loaded")
        ttk.Label(left, textvariable=self.state_text, justify=tk.LEFT, anchor=tk.NW, padding=8).pack(fill=tk.X)
        self.graph_controls = ttk.LabelFrame(left, text="Graphs", padding=6)
        self.graph_controls.pack(fill=tk.X, pady=(8, 0))

        self.graph_frame = ttk.Frame(right)
        self.graph_frame.pack(fill=tk.BOTH, expand=True)

    def _populate_attempts(self) -> None:
        self.attempt_menu["values"] = [path.name for path in self.attempts]
        self.attempt_menu.current(0)

    def choose_run_folder(self) -> None:
        """Choose a new run folder and reload its first attempt."""
        from tkinter.filedialog import askdirectory

        selected = askdirectory(initialdir=str(self.run_folder), title="Select Fly Smart run folder")
        if not selected:
            return
        try:
            self.run_folder = Path(selected)
            self.attempts = discover_attempts(self.run_folder)
            self._populate_attempts()
            self._load_attempt(0)
        except AnalysisLoadError as exc:
            self._fatal(str(exc))

    def _load_attempt(self, index: int) -> None:
        """Load an attempt and rebuild the graph/video bindings."""
        try:
            self.attempt = load_attempt(self.attempts[index])
            self.video.close()
            self.video = NoVideoSource()
            if self.attempt.video_path:
                try:
                    self.video = OpenCvVideoSource(self.attempt.video_path)
                except OSError:
                    self.video = NoVideoSource()
            self.playback = PlaybackController(self.attempt.log.time_s)
            self.graphs = GraphPanelRegistry()
            self._replace_graph_controls()
            self._replace_canvas()
            self.timeline.configure(to=max(0, len(self.attempt.log.time_s) - 1), value=0)
            self._render()
        except (AnalysisLoadError, IndexError) as exc:
            self._fatal(str(exc))

    def _replace_graph_controls(self) -> None:
        for child in self.graph_controls.winfo_children():
            child.destroy()
        assert self.graphs is not None
        for panel in self.graphs.panels:
            value = tk.BooleanVar(value=panel.default_visible)
            ttk.Checkbutton(self.graph_controls, text=panel.name, variable=value, command=lambda name=panel.name, var=value: self._toggle_panel(name, var.get())).pack(anchor=tk.W)

    def _replace_canvas(self) -> None:
        for child in self.graph_frame.winfo_children():
            child.destroy()
        assert self.graphs is not None
        canvas = FigureCanvasTkAgg(self.graphs.plot.figure, master=self.graph_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _toggle_panel(self, name: str, visible: bool) -> None:
        if self.graphs:
            self.graphs.set_visible(name, visible)
            self.graphs.plot.figure.canvas.draw_idle()

    def toggle_play(self) -> None:
        if not self.playback:
            return
        state = self.playback.pause() if self.playback.state.playing else self.playback.play()
        self.play_button.configure(text="Pause" if state.playing else "Play")

    def step(self, delta: int) -> None:
        if self.playback:
            self.playback.step(delta)
            self._render()

    def _set_speed(self) -> None:
        if self.playback:
            self.playback.set_speed(float(self.speed_var.get().replace("×", "")))

    def _seek_from_slider(self, value: str) -> None:
        if self._updating_timeline:
            return
        if self.playback and not self.playback.state.playing:
            self.playback.seek(int(float(value)))
            self._render()

    def _tick(self) -> None:
        if self.playback and self.playback.state.playing:
            state = self.playback.tick()
            self.play_button.configure(text="Pause" if state.playing else "Play")
            self._render()
        self.root.after(30, self._tick)

    def _render(self) -> None:
        if not self.attempt or not self.playback or not self.graphs:
            return
        index = self.playback.state.index
        state = self.presenter.at(self.attempt.log, index)
        self.time_var.set(f"{state.time_s:.3f} s")
        self._updating_timeline = True
        try:
            self.timeline.set(index)
        finally:
            self._updating_timeline = False
        self.state_text.set(self.presenter.text(state) + ("\nmovie: unavailable (graphs only)" if not self.video.available else ""))
        self.graphs.update(self.attempt.log, index)
        if self.video.available:
            frame_index = TimelineMapper.frame_index(state.time_s, self.video.fps, self.video.frame_count)
            frame = self.video.frame_at(frame_index)
            if frame is not None:
                frame = cv2.resize(frame, (640, 360))
                cv2.putText(frame, f"ANALYSIS  t={state.time_s:.2f}s  {state.phase}/{state.vertical_mode}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                self._image = ImageTk.PhotoImage(Image.fromarray(frame))
                self.video_label.configure(image=self._image, text="")

    def _fatal(self, message: str) -> None:
        messagebox.showerror("Offline analyzer error", message)
        self.root.destroy()
        raise SystemExit(1)

    def close(self) -> None:
        """Release video and close the Tk application."""
        self.video.close()
        self.root.destroy()
