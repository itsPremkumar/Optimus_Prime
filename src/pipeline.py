#!/usr/bin/env python3
"""
Optimus Prime G1 Build Pipeline
================================
Orchestrates: run simulation -> capture screenshots -> validate outputs

Usage:
    python pipeline.py                  # uses config.json
    python pipeline.py --config my.json # custom config
"""
import json, os, sys, subprocess
from pathlib import Path

BASE_DIR = Path(__file__).parent
PROJECT_DIR = BASE_DIR.parent
CONFIG_FILE = BASE_DIR / "config.json"


def load_config(path):
    if not path.exists():
        config = {
            "module": "ALL",
            "export_stl": True,
            "export_step": False,
            "export_urdf": False,
            "capture_screenshots": True,
            "visual_audit": False,
            "production_report": True,
            "post_capture_views": False,
            "fusion": {
                "mcp_url": "http://127.0.0.1:27182/mcp",
                "auto_launch": True,
                "keep_docs": False,
            },
        }
        with open(path, "w") as f:
            json.dump(config, f, indent=2)
        print(f"[pipeline] Created default {path} -- edit it to configure the pipeline.")
        return config
    with open(path) as f:
        return json.load(f)


def run_simulation(config):
    print("=" * 60)
    print("  STEP 1: Run Optimus Prime v14 Simulation")
    print("=" * 60)
    args = [
        sys.executable,
        str(BASE_DIR / "run_simulation.py"),
        "--module",
        config["module"],
    ]
    if config["capture_screenshots"]:
        args.append("--capture")
    if config["export_stl"]:
        args.append("--export-stl")
    if config["export_step"]:
        args.append("--export-step")
    if config["export_urdf"]:
        args.append("--export-urdf")
    if config["visual_audit"]:
        args.append("--visual-audit")
    if not config["production_report"]:
        args.append("--no-production-report")
    if not config["fusion"]["auto_launch"]:
        args.append("--no-launch")
    if config["fusion"]["keep_docs"]:
        args.append("--keep-docs")
    if config["fusion"]["mcp_url"] != "http://127.0.0.1:27182/mcp":
        args.extend(["--mcp-url", config["fusion"]["mcp_url"]])
    result = subprocess.run(args, cwd=BASE_DIR)
    return result.returncode == 0


def capture_views():
    print("=" * 60)
    print("  STEP 2: Capture Comprehensive Screenshot Views")
    print("=" * 60)
    args = [sys.executable, str(BASE_DIR / "capture_optimus.py")]
    result = subprocess.run(args, cwd=BASE_DIR)
    return result.returncode == 0


def validate_outputs(config):
    print("=" * 60)
    print("  STEP 3: Validate Outputs")
    print("=" * 60)
    output_dir = PROJECT_DIR / "output"
    results = []
    all_ok = True

    export_dir = output_dir / "exports"
    if config["export_stl"] or config["export_step"] or config["export_urdf"]:
        if config["export_stl"]:
            stls = list(export_dir.rglob("*.stl")) if export_dir.exists() else []
            ok = len(stls) > 0
            results.append(("STL exports", f"{len(stls)} files", "PASS" if ok else "FAIL"))
            all_ok &= ok
        if config["export_step"]:
            steps = list(export_dir.rglob("*.step")) if export_dir.exists() else []
            ok = len(steps) > 0
            results.append(("STEP exports", f"{len(steps)} files", "PASS" if ok else "FAIL"))
            all_ok &= ok
        if config["export_urdf"]:
            urdfs = list(export_dir.rglob("*.urdf")) if export_dir.exists() else []
            ok = len(urdfs) > 0
            results.append(("URDF exports", f"{len(urdfs)} files", "PASS" if ok else "FAIL"))
            all_ok &= ok
    else:
        results.append(("Exports", "disabled in config", "SKIP"))

    log_dir = output_dir / "logs"
    logs = sorted(log_dir.glob("*.txt")) if log_dir.exists() else []
    if logs:
        has_errors = False
        for log_file in logs[-3:]:
            content = log_file.read_text(encoding="utf-8", errors="replace")
            for keyword in ("ERROR", "Traceback", "exception", "failed"):
                if keyword.lower() in content.lower():
                    has_errors = True
                    break
        results.append(
            ("Logs", f"{len(logs)} files, {'errors' if has_errors else 'OK'}", "FAIL" if has_errors else "PASS")
        )
        all_ok &= not has_errors
    else:
        results.append(("Logs", "no log files", "FAIL"))
        all_ok = False

    screenshot_dir = output_dir / "screenshots"
    if config["capture_screenshots"]:
        pngs = list(screenshot_dir.glob("*.png")) if screenshot_dir.exists() else []
        ok = len(pngs) > 0
        results.append(("Screenshots", f"{len(pngs)} files", "PASS" if ok else "FAIL"))
        all_ok &= ok

    boms = list(output_dir.glob("BOM_v14_*.csv")) if output_dir.exists() else []
    ok = len(boms) > 0
    results.append(("BOM", f"{len(boms)} files", "PASS" if ok else "FAIL"))
    all_ok &= ok

    guides = list(output_dir.glob("ASSEMBLY_GUIDE_v14_*.txt")) if output_dir.exists() else []
    ok = len(guides) > 0
    results.append(("Assembly Guide", f"{len(guides)} files", "PASS" if ok else "FAIL"))
    all_ok &= ok

    manifests = list(output_dir.glob("BUILD_MANIFEST_v14_*.json")) if output_dir.exists() else []
    ok = len(manifests) > 0
    results.append(("Build Manifest", f"{len(manifests)} files", "PASS" if ok else "FAIL"))
    all_ok &= ok

    print()
    print(f"{'Check':<25} {'Detail':<30} {'Status'}")
    print("-" * 75)
    for name, detail, status in results:
        print(f"{name:<25} {detail:<30} {status}")
    print("-" * 75)
    print(f"{'OVERALL':<25} {'':<30} {'PASS' if all_ok else 'FAIL'}")

    return all_ok


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Optimus Prime G1 Build Pipeline")
    parser.add_argument("--config", default=str(CONFIG_FILE), help="Path to config JSON")
    args = parser.parse_args()

    print("=" * 60)
    print("  OPTIMUS PRIME G1  --  BUILD PIPELINE")
    print("=" * 60)

    config_path = Path(args.config)
    config = load_config(config_path)
    print(f"Config: {config_path.resolve()}")
    print(f"Module: {config['module']}")
    print(f"STL: {config['export_stl']}, STEP: {config['export_step']}, URDF: {config['export_urdf']}")
    print(f"Screenshots: {config['capture_screenshots']}, Post-capture views: {config['post_capture_views']}")
    print()

    sim_ok = run_simulation(config)
    if not sim_ok:
        print("[pipeline] Simulation step FAILED -- aborting pipeline.")
        sys.exit(1)

    if config.get("post_capture_views"):
        capture_views()

    validate_ok = validate_outputs(config)

    print()
    if validate_ok:
        print("  Pipeline completed successfully.")
    else:
        print("  Pipeline completed with validation warnings.")
        sys.exit(1)


if __name__ == "__main__":
    main()
