# -*- coding: utf-8 -*-
"""
GNSS 热启动（断天线重接）多设备汇总分析 HMI
=====================================================================
用法：python gnss_warmstart_hmi.py

功能：
  * 顶部产品下拉选择设备类型（北云 BY / 华测 HUACE，可在 gnss_parsers.py 注册扩展）
  * 添加文件：按当前下拉选中的类型加入下方列表（左列文件名，右列设备类型）
  * 可混合添加多台设备的同轮热启动日志，一次性做统一时序对齐 + 取早 + 汇总报告
  * 放弃第 1 次断线循环（周秒未对齐），自第 2 次起用绝对 GPS 周秒对齐，
    每次循环取所有设备中最早的 locktime 作为统一 t0
  * 点击"开始分析"调用 warmstart_report.py 输出 HTML / MD / 图表 / JSON
"""
from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import gnss_parsers as G

APP_TITLE = "GNSS 热启动分析"
BASE_DIR = Path(__file__).resolve().parent
REPORT_SCRIPT = "warmstart_report.py"


def open_path(path: str) -> None:
    if not path:
        return
    if os.name == "nt":
        os.startfile(path)  # type: ignore[attr-defined]
    elif sys.platform == "darwin":
        subprocess.Popen(["open", path])
    else:
        subprocess.Popen(["xdg-open", path])


class WarmstartHMI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title(APP_TITLE)
        self.root.geometry("1160x780")
        self.root.minsize(980, 660)

        # entries: list of dict(path=..., product=...)
        self.entries: list[dict] = []
        self.running = False
        self.worker: threading.Thread | None = None
        self.ui_queue: queue.Queue[tuple] = queue.Queue()
        self.last_report: str = ""

        self._build_ui()
        self.root.after(100, self._poll_ui_queue)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        self.root.configure(bg="#f5f6fa")

        tk.Label(self.root, text=APP_TITLE, font=("微软雅黑", 18, "bold"),
                 bg="#f5f6fa", fg="#2c3e50").pack(pady=(16, 4))
        tk.Label(self.root,
                 text="先用下拉选择设备类型再添加文件；可混合多台同轮热启动日志，统一时序对齐、取最早 locktime 作为统一 t0，输出一份汇总报告",
                 font=("微软雅黑", 10), bg="#f5f6fa", fg="#7f8c8d").pack(pady=(0, 12))

        # 设备类型 + 添加
        top = ttk.LabelFrame(self.root, text="添加设备日志")
        top.pack(fill=tk.X, padx=20, pady=(0, 10))
        tk.Label(top, text="设备类型:", font=("微软雅黑", 10)).pack(side=tk.LEFT, padx=(12, 4))
        self.product_var = tk.StringVar()
        self._products = list(G.PARSERS.values())
        names = [p.display for p in self._products]
        self.product_var.set(names[0])
        self.product_combo = ttk.Combobox(top, textvariable=self.product_var,
                                          state="readonly", values=names, width=44)
        self.product_combo.pack(side=tk.LEFT, padx=4, pady=10)
        self.browse_button = ttk.Button(top, text="添加该类型文件...", command=self.browse_files)
        self.browse_button.pack(side=tk.LEFT, padx=8)
        ttk.Label(top, text="（同一设备加多个文件时，仅第一份参与分析）",
                  foreground="#7f8c8d").pack(side=tk.LEFT, padx=10)

        # 文件列表（左文件名 右设备类型）
        list_frame = ttk.LabelFrame(self.root, text="分析组（多台设备统一对齐取早）")
        list_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 10))
        cols = ("file", "product")
        self.tree = ttk.Treeview(list_frame, columns=cols, show="headings", selectmode="extended")
        self.tree.heading("file", text="文件名")
        self.tree.heading("product", text="设备类型")
        self.tree.column("file", width=720, anchor=tk.W)
        self.tree.column("product", width=260, anchor=tk.W)
        yscroll = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=yscroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=8)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=8)

        # 控制按钮
        ctrl = ttk.Frame(self.root)
        ctrl.pack(fill=tk.X, padx=20, pady=(0, 8))
        self.start_button = ttk.Button(ctrl, text="开始分析", command=self.start_analysis)
        self.start_button.pack(side=tk.LEFT, padx=(0, 8))
        self.remove_button = ttk.Button(ctrl, text="移除选中", command=self.remove_selected)
        self.remove_button.pack(side=tk.LEFT, padx=8)
        self.clear_button = ttk.Button(ctrl, text="清空", command=self.clear_files)
        self.clear_button.pack(side=tk.LEFT, padx=8)
        self.open_report_button = ttk.Button(ctrl, text="打开报告", command=self.open_report, state=tk.DISABLED)
        self.open_report_button.pack(side=tk.LEFT, padx=8)
        self.open_dir_button = ttk.Button(ctrl, text="打开输出目录", command=self.open_dir, state=tk.DISABLED)
        self.open_dir_button.pack(side=tk.LEFT, padx=8)
        self.auto_open_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl, text="完成后自动打开报告", variable=self.auto_open_var).pack(side=tk.LEFT, padx=12)
        self.count_var = tk.StringVar(value="已选 0 个文件 / 0 台设备")
        ttk.Label(ctrl, textvariable=self.count_var).pack(side=tk.RIGHT)

        # 日志
        log_frame = ttk.LabelFrame(self.root, text="运行日志")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 8))
        self.log_text = tk.Text(log_frame, height=9, font=("Consolas", 9), wrap=tk.WORD)
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=8)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=8)
        self.log_text.tag_config("info", foreground="#2c3e50")
        self.log_text.tag_config("success", foreground="#1e8449")
        self.log_text.tag_config("error", foreground="#c0392b")

        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status_var, anchor=tk.W,
                  foreground="#7f8c8d").pack(fill=tk.X, padx=22, pady=(0, 10))

    # ------------------------------------------------------------------ helpers
    def _current_product_display(self) -> str:
        return self.product_var.get()

    def browse_files(self) -> None:
        disp = self._current_product_display()
        paths = filedialog.askopenfilenames(
            title=f"选择【{disp}】热启动日志",
            filetypes=[("GNSS 日志", "*.log *.dat"), ("所有文件", "*.*")])
        added = 0
        for p in paths:
            if any(e['path'] == p for e in self.entries):
                continue
            self.entries.append(dict(path=p, product=disp))
            self.tree.insert("", tk.END, values=(os.path.basename(p), disp))
            added += 1
        self._refresh_count()
        if added:
            self._log(f"添加 {added} 个文件（{disp}）\n", "info")

    def remove_selected(self) -> None:
        sel = self.tree.selection()
        # 从后往前删
        idxs = sorted((self.tree.index(i) for i in sel), reverse=True)
        for i in idxs:
            self.tree.delete(self.tree.get_children()[i])
            del self.entries[i]
        self._refresh_count()

    def clear_files(self) -> None:
        for i in self.tree.get_children():
            self.tree.delete(i)
        self.entries.clear()
        self._refresh_count()

    def _refresh_count(self) -> None:
        n_dev = len(set(e['product'] for e in self.entries))
        self.count_var.set(f"已选 {len(self.entries)} 个文件 / {n_dev} 台设备")

    def _log(self, msg: str, tag: str = "info") -> None:
        self.log_text.insert(tk.END, msg, tag)
        self.log_text.see(tk.END)
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 4000:
            self.log_text.delete("1.0", f"{line_count - 4000}.0")

    # ------------------------------------------------------------------ run
    def start_analysis(self) -> None:
        if self.running:
            messagebox.showwarning(APP_TITLE, "分析正在进行中")
            return
        if not self.entries:
            messagebox.showwarning(APP_TITLE, "请先添加数据文件")
            return

        out_dir = filedialog.askdirectory(
            title="选择输出目录（将生成 report.html / report.md / 图片 / summary.json）",
            initialdir=str(BASE_DIR / "reports"))
        if not out_dir:
            return

        auto_open = self.auto_open_var.get()
        files = [e['path'] for e in self.entries]
        n_dev = len(set(e['product'] for e in self.entries))

        self.running = True
        self.start_button.configure(state=tk.DISABLED)
        self.browse_button.configure(state=tk.DISABLED)
        self.open_report_button.configure(state=tk.DISABLED)
        self.open_dir_button.configure(state=tk.DISABLED)
        self.status_var.set("分析中...")
        self._log(f"\n===== 开始分析 {len(files)} 个文件 / {n_dev} 台设备 → {out_dir} =====\n", "info")

        self.worker = threading.Thread(target=self._run, args=(files, out_dir, auto_open), daemon=True)
        self.worker.start()

    def _run(self, files: list[str], out_dir: str, auto_open: bool) -> None:
        try:
            script = BASE_DIR / REPORT_SCRIPT
            cmd = [sys.executable, "-X", "utf8", str(script), out_dir, *files]
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["PYTHONIOENCODING"] = "utf-8"
            creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            try:
                process = subprocess.Popen(cmd, cwd=str(BASE_DIR), stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                           errors="replace", env=env, creationflags=creationflags, bufsize=1)
            except Exception as exc:
                self.ui_queue.put(("log", f"启动分析进程失败：{exc}\n", "error"))
                self.ui_queue.put(("done", ""))
                return
            assert process.stdout is not None
            for line in process.stdout:
                self.ui_queue.put(("log", line, "info"))
            rc = process.wait()
            report = os.path.join(out_dir, "report.html")
            if rc == 0 and os.path.isfile(report):
                self.ui_queue.put(("log", f"分析完成：{report}\n", "success"))
                self.ui_queue.put(("done", report))
                if auto_open:
                    open_path(report)
            else:
                self.ui_queue.put(("log", f"分析失败，返回码 {rc}\n", "error"))
                self.ui_queue.put(("done", ""))
        finally:
            pass

    def _poll_ui_queue(self) -> None:
        try:
            while True:
                event = self.ui_queue.get_nowait()
                if event[0] == "log":
                    _, line, tag = event
                    self._log(line, tag)
                elif event[0] == "done":
                    _, report = event
                    self.running = False
                    self.start_button.configure(state=tk.NORMAL)
                    self.browse_button.configure(state=tk.NORMAL)
                    self.status_var.set("分析完成" if report else "就绪")
                    if report:
                        self.last_report = report
                        self.open_report_button.configure(state=tk.NORMAL)
                        self.open_dir_button.configure(state=tk.NORMAL)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_ui_queue)

    def open_report(self) -> None:
        open_path(self.last_report)

    def open_dir(self) -> None:
        open_path(os.path.dirname(self.last_report))

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    WarmstartHMI().run()