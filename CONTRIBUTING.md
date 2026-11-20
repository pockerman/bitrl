# Contributing to bitrl

## Commit messages

bitrl uses [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).
PRs are squash merged, so the **PR title** becomes the commit on `master`. A check
fails any PR whose title does not follow the format:

```
<type>(<optional scope>): <what changes>
```

| Type | Use it for | In the changelog | Version bump |
| ---- | ---------- | ---------------- | ------------ |
| `feat` | A new feature: an algorithm, a policy, an environment, an example | Features | minor |
| `fix` | A bug fix | Bug Fixes | patch |
| `perf` | Faster code that behaves the same | Performance | patch |
| `refactor` | Restructured code that behaves the same | Refactoring | none |
| `docs` | Documentation and example pages only | Documentation | none |
| `test` | Tests only | hidden | none |
| `build` | CMake, dependencies | hidden | none |
| `ci` | GitHub workflows, `ci/` scripts | hidden | none |
| `chore`, `style` | Anything else, formatting | hidden | none |
| `revert` | Reverting an earlier commit | Reverts | none |

The scope is optional and names the part of the library, for example `envs`,
`algorithms`, `policies`, `trainers` or `examples`.

A change that breaks existing code adds `!` after the type, `feat(envs)!: ...`, or
a `BREAKING CHANGE:` paragraph in the PR description. It bumps the major version.

Examples:

```
feat(algorithms): add expected SARSA
fix(envs): throw on an invalid grid world action
docs(examples): add the page for example 22
ci: cache libtorch
```

## Formatting

The code is formatted with clang-format 14 and the settings in `.clang-format`; CI fails
on code that is not. Format it before you push:

```
pip install clang-format==14.0.6   # once, if your clang-format is not version 14
./format.sh
```

`./format.sh --check` only reports the files that are not formatted.

## Releases

Releases are made by [release-please](https://github.com/googleapis/release-please).
After every merge to `master` it updates a release PR that bumps `version.txt` and
adds the new entries to `CHANGELOG.md`. Merging that PR tags the release, `vX.Y.Z`,
and creates the GitHub release.

`version.txt` is the only place the version is kept; CMake reads it. Do not change
it by hand.

## Example pages

Every example that is built has a page next to its source, `<name>.md`, whose first line
is `\page bitrl_example_<id> <title>`, where `<id>` is the name without a leading
`example_`. `examples/examples.md` links to it. The page shows the source with
`\include <name>/<name>.cpp` rather than a copy of it. `ci/check_docs.sh` checks all of
this and that Doxygen builds the pages without warnings; it needs Doxygen, and clones
bitsim unless `BITSIM_SOURCE_DIR` points to a checkout.
