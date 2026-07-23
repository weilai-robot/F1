#!/usr/bin/env python3
"""Create and verify the provenance manifest for one navigation evaluation."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from nav_eval_contract import SUCCESS_CONTRACT, SUCCESS_CONTRACT_VERSION


SCHEMA_VERSION = 1
MANIFEST_NAME = "evaluation_manifest.json"


class ManifestError(RuntimeError):
    """Raised when evidence is incomplete or does not match the requested run."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path, root: Path) -> dict[str, Any]:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def report_files(report_dir: Path) -> list[dict[str, Any]]:
    return [
        file_record(path, report_dir)
        for path in sorted(report_dir.rglob("*"))
        if path.is_file() and path.name != MANIFEST_NAME
    ]


def source_files(repo_root: Path, paths: list[str]) -> list[dict[str, Any]]:
    records = []
    for relative in paths:
        path = repo_root / relative
        if not path.is_file():
            raise ManifestError(f"source evidence file does not exist: {relative}")
        records.append(
            {
                "path": relative,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    return records


def create_manifest(args: argparse.Namespace) -> Path:
    report_dir = Path(args.report_dir).resolve()
    repo_root = Path(args.repo_root).resolve()
    report_dir.mkdir(parents=True, exist_ok=True)

    files = report_files(report_dir)
    metric_result_json = [
        item
        for item in files
        if item["path"].endswith(".json")
        and Path(item["path"]).name
        not in {
            "ci_diagnostic.json",
            "evaluation_manifest.json",
            "infrastructure_failure.json",
        }
    ]
    evidence_status = "complete" if metric_result_json else "infrastructure_failure"
    if not metric_result_json:
        failure_path = report_dir / "infrastructure_failure.json"
        if not failure_path.exists():
            failure_path.write_text(
                json.dumps(
                    {
                        "classification": "infrastructure_failure",
                        "reason": "navigation evaluation produced no metric result",
                        "test_outcome": args.test_outcome,
                    },
                    indent=2,
                    ensure_ascii=False,
                )
                + "\n",
                encoding="utf-8",
            )
        files = report_files(report_dir)

    manifest = {
        "schema": "f1.navigation-evaluation",
        "schema_version": SCHEMA_VERSION,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "run": {
            "repository": args.repository,
            "workflow": args.workflow,
            "event": args.event,
            "run_id": str(args.run_id),
            "run_attempt": int(args.run_attempt),
            "scenario": args.scenario,
            "test_outcome": args.test_outcome,
        },
        "source": {
            "f1_commit": args.f1_commit,
            "navigation_gitlink_commit": args.navigation_commit,
            "navigation_checkout_commit": args.navigation_commit,
        },
        "environment": {
            "entrypoint": "f1ros",
            "ros_distro": args.ros_distro or os.environ.get("ROS_DISTRO"),
            "conda_environment": (
                args.conda_environment or os.environ.get("CONDA_DEFAULT_ENV")
            ),
            "python": args.python_version or platform.python_version(),
        },
        "metric_contract": {
            "success": SUCCESS_CONTRACT,
            "contract_version": SUCCESS_CONTRACT_VERSION,
        },
        "evidence": {
            "status": evidence_status,
            "report_files": files,
            "source_files": source_files(
                repo_root,
                [
                    ".github/workflows/nav_eval.yml",
                    "scripts/ci_nav_test.sh",
                    "scripts/nav_eval_contract.py",
                    "scripts/nav_test_runner.py",
                    "scripts/nav_eval_manifest.py",
                ],
            ),
        },
    }

    output = report_dir / MANIFEST_NAME
    output.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(output)
    return output


def load_manifest(results_dir: Path) -> tuple[Path, dict[str, Any]]:
    matches = list(results_dir.rglob(MANIFEST_NAME))
    if len(matches) != 1:
        raise ManifestError(
            f"expected exactly one {MANIFEST_NAME}, found {len(matches)}"
        )
    path = matches[0]
    return path, json.loads(path.read_text(encoding="utf-8"))


def verify_manifest(
    results_dir: Path,
    expected_run_id: Optional[str] = None,
    expected_run_attempt: Optional[int] = None,
    expected_f1_commit: Optional[str] = None,
    expected_navigation_commit: Optional[str] = None,
) -> dict[str, Any]:
    results_dir = results_dir.resolve()
    manifest_path, manifest = load_manifest(results_dir)

    if manifest.get("schema") != "f1.navigation-evaluation":
        raise ManifestError("unexpected manifest schema")
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ManifestError("unsupported manifest schema version")

    run = manifest.get("run", {})
    source = manifest.get("source", {})
    if expected_run_id and str(run.get("run_id")) != str(expected_run_id):
        raise ManifestError(
            f"run id mismatch: expected {expected_run_id}, got {run.get('run_id')}"
        )
    if (
        expected_run_attempt is not None
        and run.get("run_attempt") != expected_run_attempt
    ):
        raise ManifestError(
            "run attempt mismatch: "
            f"expected {expected_run_attempt}, got {run.get('run_attempt')}"
        )
    if expected_f1_commit and source.get("f1_commit") != expected_f1_commit:
        raise ManifestError(
            "F1 commit mismatch: "
            f"expected {expected_f1_commit}, got {source.get('f1_commit')}"
        )
    if (
        expected_navigation_commit
        and source.get("navigation_gitlink_commit") != expected_navigation_commit
    ):
        raise ManifestError(
            "navigation commit mismatch: "
            f"expected {expected_navigation_commit}, "
            f"got {source.get('navigation_gitlink_commit')}"
        )

    evidence_root = manifest_path.parent
    records = manifest.get("evidence", {}).get("report_files", [])
    if not records:
        raise ManifestError("manifest contains no report evidence")
    for record in records:
        relative = Path(record["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ManifestError(f"unsafe evidence path: {relative}")
        path = evidence_root / relative
        if not path.is_file():
            raise ManifestError(f"missing evidence file: {relative}")
        if path.stat().st_size != record.get("size_bytes"):
            raise ManifestError(f"evidence size mismatch: {relative}")
        if sha256_file(path) != record.get("sha256"):
            raise ManifestError(f"evidence checksum mismatch: {relative}")

    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    create = subparsers.add_parser("create", help="write a run manifest")
    create.add_argument("--report-dir", required=True)
    create.add_argument("--repo-root", required=True)
    create.add_argument("--repository", required=True)
    create.add_argument("--workflow", required=True)
    create.add_argument("--event", required=True)
    create.add_argument("--run-id", required=True)
    create.add_argument("--run-attempt", required=True, type=int)
    create.add_argument("--scenario", required=True)
    create.add_argument("--test-outcome", required=True)
    create.add_argument("--f1-commit", required=True)
    create.add_argument("--navigation-commit", required=True)
    create.add_argument("--ros-distro")
    create.add_argument("--conda-environment")
    create.add_argument("--python-version")

    verify = subparsers.add_parser("verify", help="verify downloaded evidence")
    verify.add_argument("--results-dir", required=True)
    verify.add_argument("--run-id")
    verify.add_argument("--run-attempt", type=int)
    verify.add_argument("--f1-commit")
    verify.add_argument("--navigation-commit")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "create":
            create_manifest(args)
        else:
            manifest = verify_manifest(
                Path(args.results_dir),
                expected_run_id=args.run_id,
                expected_run_attempt=args.run_attempt,
                expected_f1_commit=args.f1_commit,
                expected_navigation_commit=args.navigation_commit,
            )
            print(
                "verified "
                f"run={manifest['run']['run_id']} "
                f"attempt={manifest['run']['run_attempt']} "
                f"f1={manifest['source']['f1_commit']} "
                f"navigation={manifest['source']['navigation_gitlink_commit']}"
            )
    except (ManifestError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
