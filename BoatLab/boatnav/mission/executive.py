"""Mission executive: runs task generators in order, with pause / skip / goto / abort and
per-task time budgets. Operator commands arrive from the dashboard via the runner."""
from __future__ import annotations

import copy

from .tasks import REGISTRY


class TaskRun:
    def __init__(self, spec: dict, index: int):
        self.spec = dict(spec)
        self.type = spec["type"]
        self.index = index
        self.challenge = REGISTRY[self.type][1]
        self.status = "pending"
        self.result: dict = {}
        self.t_start = None
        self.t_end = None

    def as_dict(self):
        return {"index": self.index, "type": self.type, "challenge": self.challenge, "status": self.status,
                "label": self.spec.get("label") or self.type, "result": self.result,
                "duration_s": round(self.t_end - self.t_start, 1) if self.t_start is not None and self.t_end is not None else None}


class Executive:
    def __init__(self, ctx, mission: dict):
        self.ctx = ctx
        self.mission = copy.deepcopy(mission)
        self.tasks = [TaskRun(t, i) for i, t in enumerate(mission["tasks"])]
        self.budget = mission.get("time_budget_s", {})
        self.index = 0
        self.gen = None
        self.paused = mission.get("start_paused", False)
        self.aborted = False
        self.finished = False

    # ------------------------------------------------------------------ control
    def step(self, state):
        ctx = self.ctx
        ctx.state = state
        ctx.t = state.t
        if self.aborted or self.finished or self.paused:
            ctx.phase = "aborted" if self.aborted else ("finished" if self.finished else "paused")
            return 0.0, 0.0
        if self.gen is None:
            if self.index >= len(self.tasks):
                self.finished = True
                ctx.log("Mission complete")
                return 0.0, 0.0
            task = self.tasks[self.index]
            fn = REGISTRY[task.type][0]
            params = {k: v for k, v in task.spec.items() if k not in ("type", "label")}
            task.status, task.t_start = "running", ctx.t
            ctx.log(f"Task {task.index + 1}/{len(self.tasks)}: {task.spec.get('label', task.type)}")
            self.gen = fn(ctx, **params)
        task = self.tasks[self.index]
        limit = task.spec.get("time_budget_s", self.budget.get(task.type, self.budget.get("default", 300)))
        if ctx.t - task.t_start > limit:
            ctx.log(f"Task {task.type}: time budget {limit}s exceeded - moving on", "warn")
            self._finish(task, {"status": "timeout"})
            return 0.0, 0.0
        try:
            return next(self.gen)
        except StopIteration as e:
            self._finish(task, e.value or {"status": "done"})
            return 0.0, 0.0

    def _finish(self, task, result):
        task.result = result
        st = result.get("status", "done")
        task.status = "done" if st == "done" else ("failed" if st in ("failed", "blocked") else st)
        task.t_end = self.ctx.t
        self.ctx.log(f"Task {task.type} -> {task.status}", "info" if task.status == "done" else "warn")
        self.gen = None
        self.index += 1

    # ------------------------------------------------------------------ operator commands
    def command(self, cmd: str, **kw) -> dict:
        ctx = self.ctx
        if cmd == "pause":
            self.paused = True
        elif cmd in ("resume", "start"):
            self.paused = False
            self.aborted = False
        elif cmd in ("abort", "estop"):
            self.aborted = True
            if self.gen is not None:
                self._finish(self.tasks[self.index], {"status": "aborted"})
        elif cmd == "skip":
            if self.gen is not None:
                self._finish(self.tasks[self.index], {"status": "skipped"})
            elif self.index < len(self.tasks):
                self.tasks[self.index].status = "skipped"
                self.index += 1
        elif cmd == "goto":
            i = int(kw.get("index", 0))
            if 0 <= i < len(self.tasks):
                if self.gen is not None:
                    self._finish(self.tasks[self.index], {"status": "interrupted"})
                self.index, self.finished, self.aborted = i, False, False
                for t in self.tasks[i:]:
                    t.status = "pending"
        elif cmd == "set_color":
            ctx.identify_color = kw.get("color", ctx.identify_color)
            for t in self.tasks:
                if t.type == "identify" and t.status == "pending":
                    t.spec["color"] = ctx.identify_color
        elif cmd == "waypoints":
            spec = {"type": "waypoints", "label": "operator waypoints", "points": kw.get("points"),
                    "latlon": kw.get("latlon"), "stop": True}
            if self.gen is not None:
                self._finish(self.tasks[self.index], {"status": "interrupted"})
            self.tasks.insert(self.index, TaskRun(spec, self.index))
            for i, t in enumerate(self.tasks):
                t.index = i
            self.finished = self.aborted = False
            self.paused = False
        else:
            return {"ok": False, "error": f"unknown command {cmd}"}
        ctx.log(f"Operator: {cmd} {kw if kw else ''}".strip())
        return {"ok": True}

    def snapshot(self) -> dict:
        return {"name": self.mission.get("name", ""), "index": self.index, "paused": self.paused,
                "aborted": self.aborted, "finished": self.finished, "tasks": [t.as_dict() for t in self.tasks]}
