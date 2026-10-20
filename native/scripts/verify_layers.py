"""Check that model, problem and dynamics have no forbidden include dependencies."""
import argparse
import json
import re
from pathlib import Path


def verify(native: Path):
    core = native / "src/g1_controller"
    roots = [native / "include", core]
    header_suffixes = {".h", ".hh", ".hpp", ".hxx", ".ipp", ".inl", ".tpp"}
    for layer in ("model", "problem", "dynamics"):
        roots.extend(sorted((core / layer).glob("*/include")))
    headers = {}
    for root in roots:
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in header_suffixes:
                headers.setdefault(path.relative_to(root).as_posix(), set()).add(path)

    def owner(path):
        if path in (core / "InverseDynamics.cpp",
                    native / "include/g1_controller/InverseDynamics.h"):
            return "dynamics"
        for layer in ("model", "problem", "dynamics"):
            if path.is_relative_to(core / layer):
                return layer
        if path == native / "include/g1_controller/G1Controller.h":
            return "problem"
        return None

    include = re.compile(r'^\s*#include\s*["<]([^">]+)[">]', re.MULTILINE)
    suffixes = header_suffixes | {".cpp", ".cc", ".cxx"}
    files = {path for root in (core, native / "include") for path in root.rglob("*")
             if path.is_file() and path.suffix in suffixes}
    graph = {}
    for path in files:
        dependencies = set()
        for name in include.findall(path.read_text(encoding="utf-8")):
            local = path.parent / name
            if local.is_file():
                dependencies.add(local.resolve())
            dependencies.update(headers.get(name, ()))
        graph[path] = dependencies

    checked = {layer: 0 for layer in ("model", "problem", "dynamics")}
    violations = []
    for path in files:
        layer = owner(path)
        if layer is None:
            continue
        checked[layer] += 1
        forbidden = {"model": {"problem", "dynamics"},
                     "problem": {"dynamics"}, "dynamics": {"problem"}}[layer]
        pending = list(graph[path])
        seen = set()
        while pending:
            dependency = pending.pop()
            if dependency in seen:
                continue
            seen.add(dependency)
            if owner(dependency) in forbidden:
                violations.append(f"{path.relative_to(native)} -> {dependency.relative_to(native)}")
            pending.extend(graph.get(dependency, ()))
    if violations:
        raise RuntimeError("Forbidden layer dependencies:\n" + "\n".join(sorted(set(violations))))
    return {"passed": True, "checked_files": checked,
            "problem_dynamics_cross_dependencies": 0}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    print(json.dumps(verify(args.native.resolve()), indent=2))
