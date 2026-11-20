#!/usr/bin/env python3
"""Check that every example that is built is documented.

An example lives in examples/<name>/ or examples/<section>/<name>/, with the
source <name>.cpp (or <name>.cu). It is built when the CMakeLists.txt of its
directory or of its section calls bitrl_add_example(<name> ...), and
examples/CMakeLists.txt reaches that CMakeLists.txt through ADD_SUBDIRECTORY. Its page id is bitrl_example_<id>, where
<id> is <name> without a leading "example_": value_iteration is
bitrl_example_value_iteration, and an example_14 would be bitrl_example_14. Every example that is built
- has a page <name>.md whose first line is
      \\page bitrl_example_<id> <title>
- is linked from examples/examples.md.

Examples that are not built are listed, but are not errors.

Usage: ci/check_example_docs.py [path-to-bitrl]
"""
import re
import sys
from pathlib import Path


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1])
    examples = root / "examples"
    index = (examples / "examples.md").read_text()
    linked = set(re.findall(r"\\ref bitrl_example_([a-z0-9_]+)\b", index))

    # Follow ADD_SUBDIRECTORY from examples/CMakeLists.txt, as CMake does. Lines that
    # are commented out do not count; a directory added under an IF does.
    built = set()
    pending = [examples]
    while pending:
        directory = pending.pop()
        cmake = directory / "CMakeLists.txt"
        if not cmake.exists():
            continue
        text = re.sub(r"#.*", "", cmake.read_text())
        # bitrl_add_example(<name>) builds <name>.cpp next to it or <name>/<name>.cpp
        # below it; the CUDA example has its own ADD_EXECUTABLE
        for name in re.findall(r"\b(?:bitrl_add_example|ADD_EXECUTABLE)\(\s*([A-Za-z0-9_]+)", text, re.I):
            built.add((directory, name))
            built.add((directory / name, name))
        for sub in re.findall(r"\bADD_SUBDIRECTORY\(\s*([A-Za-z0-9_./-]+)", text, re.I):
            pending.append(directory / sub)

    errors = []
    names = []
    not_built = []
    for source in sorted(list(examples.rglob("*.cpp")) + list(examples.rglob("*.cu"))):
        directory = source.parent
        if source.stem != directory.name or "build" in source.relative_to(examples).parts:
            continue
        name = directory.name
        if (directory, name) not in built:
            not_built.append(str(directory.relative_to(root)))
            continue
        page_id = name.removeprefix("example_")
        names.append(page_id)

        page = directory / f"{name}.md"
        if not page.exists():
            errors.append(f"{name} has no page: add {page.relative_to(root)}")
            continue
        text = page.read_text()
        first = text.splitlines()[0] if text else ""
        if not re.fullmatch(rf"\\page bitrl_example_{page_id} \S.*", first):
            errors.append(f"{page.relative_to(root)}: the first line must be "
                          f"'\\page bitrl_example_{page_id} <title>', not '{first}'")
        if page_id not in linked:
            errors.append(f"examples/examples.md does not link to \\ref bitrl_example_{page_id}")

    duplicates = sorted({n for n in names if names.count(n) > 1})
    for name in duplicates:
        errors.append(f"two examples are called {name}: the executables and pages would clash")

    for name in sorted(linked - set(names)):
        if not any(name.startswith(n + "_") for n in names):
            errors.append(f"examples/examples.md links to bitrl_example_{name}, which is not a built example")

    for error in errors:
        print(f"error: {error}")
    if not errors:
        print(f"{len(names)} examples are built, have a page and are linked from examples.md")
    if not_built:
        print(f"not built, so not checked: {', '.join(not_built)}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
