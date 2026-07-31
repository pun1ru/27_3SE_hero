"""Apply guarded, exact text replacements from a JSON manifest."""

from __future__ import annotations

import argparse
import difflib
import json
import os
from pathlib import Path
from typing import Any


def load_utf8(path: Path) -> tuple[str, bool]:
    data = path.read_bytes()
    has_bom = data.startswith(b"\xef\xbb\xbf")
    return data.decode("utf-8-sig" if has_bom else "utf-8"), has_bom


def resolve_relative(root: Path, relative_path: str) -> Path:
    candidate = Path(relative_path)
    if candidate.is_absolute():
        raise ValueError(f"path must be relative: {relative_path}")
    resolved_root = root.resolve()
    resolved_path = (root / candidate).resolve()
    if os.path.commonpath((str(resolved_root), str(resolved_path))) != str(resolved_root):
        raise ValueError(f"path escapes project root: {relative_path}")
    return resolved_path


def validate_manifest(manifest: Any, root: Path) -> list[tuple[Path, str, str, int]]:
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), list):
        raise ValueError("manifest must contain a files array")

    operations: list[tuple[Path, str, str, int]] = []
    for file_entry in manifest["files"]:
        if not isinstance(file_entry, dict):
            raise ValueError("each files entry must be an object")
        target = resolve_relative(root, file_entry.get("path", ""))
        if not target.is_file():
            raise FileNotFoundError(target)
        replacements = file_entry.get("replacements")
        if not isinstance(replacements, list) or not replacements:
            raise ValueError(f"replacements must be a non-empty array: {target}")
        for replacement in replacements:
            if not isinstance(replacement, dict):
                raise ValueError(f"replacement must be an object: {target}")
            old = replacement.get("old")
            new = replacement.get("new")
            count = replacement.get("count", 1)
            if not isinstance(old, str) or not old:
                raise ValueError(f"old must be a non-empty string: {target}")
            if not isinstance(new, str):
                raise ValueError(f"new must be a string: {target}")
            if not isinstance(count, int) or count < 1:
                raise ValueError(f"count must be a positive integer: {target}")
            operations.append((target, old, new, count))
    return operations


def print_preview(root: Path, before: dict[Path, str], after: dict[Path, tuple[str, bool]]) -> None:
    for target, original_text in before.items():
        updated_text = after[target][0]
        relative_path = target.relative_to(root).as_posix()
        diff = list(
            difflib.unified_diff(
                original_text.splitlines(),
                updated_text.splitlines(),
                fromfile=f"a/{relative_path}",
                tofile=f"b/{relative_path}",
                lineterm="",
            )
        )
        if diff:
            print("\n".join(diff))
        else:
            print(f"NO CHANGE {relative_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="validate and show unified diff")
    mode.add_argument("--apply", action="store_true", help="validate and write changes")
    args = parser.parse_args()

    root = args.project_root.resolve()
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    operations = validate_manifest(manifest, root)

    original_contents: dict[Path, str] = {}
    contents: dict[Path, tuple[str, bool]] = {}
    for target, old, new, expected_count in operations:
        if target not in contents:
            text, has_bom = load_utf8(target)
            original_contents[target] = text
            contents[target] = (text, has_bom)
        text, has_bom = contents[target]
        actual_count = text.count(old)
        if actual_count != expected_count:
            raise ValueError(
                f"{target}: expected {expected_count} matches, found {actual_count}"
            )
        contents[target] = (text.replace(old, new, expected_count), has_bom)

    if args.check:
        print_preview(root, original_contents, contents)
        print("Preview only; no files were changed.")
        return 0

    for target, (text, has_bom) in contents.items():
        encoded = text.encode("utf-8")
        target.write_bytes((b"\xef\xbb\xbf" if has_bom else b"") + encoded)
        print(f"UPDATED {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
