#!/usr/bin/env bash
set -Eeuo pipefail

class_name="GitHubReleaseScript"

usage() {
  cat <<'EOF'
Usage:
  ./release.sh [tag] [options]

Examples:
  ./release.sh v0.1.1
  ./release.sh v0.1.1 --branch main
  ./release.sh v0.1.1 --branch main --title "v0.1.1" --notes generated
  ./release.sh

Options:
  --branch BRANCH       Branch to release from. Default: current branch
  --remote REMOTE       Git remote name. Default: origin
  --title TITLE         Release title. Default: tag
  --notes MODE          Notes mode: generated, manual, file, none. Default: generated
  --notes-file FILE     File used when --notes file
  --draft               Create release as draft
  --prerelease          Mark release as prerelease
  --target COMMIT       Tag this specific commit instead of branch HEAD
  --no-pull             Do not run git pull --ff-only
  --help                Show this help

Interactive mode:
  Run ./release.sh with no arguments.
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

ask() {
  local prompt="$1"
  local default="${2:-}"
  local value

  if [[ -n "$default" ]]; then
    read -r -p "$prompt [$default]: " value
    echo "${value:-$default}"
  else
    read -r -p "$prompt: " value
    echo "$value"
  fi
}

ask_yes_no() {
  local prompt="$1"
  local answer
  read -r -p "$prompt [y/N]: " answer
  [[ "$answer" == "y" || "$answer" == "Y" ]]
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

package_version_from_pyproject() {
  local ref="${1:-}"

  python3 - "$ref" <<'PY'
import re
import subprocess
import sys
from pathlib import Path

ref = sys.argv[1]

if ref:
    try:
        content = subprocess.check_output(["git", "show", f"{ref}:pyproject.toml"])
    except subprocess.CalledProcessError:
        sys.exit("pyproject.toml not found")
else:
    pyproject_path = Path("pyproject.toml")

    if not pyproject_path.is_file():
        sys.exit("pyproject.toml not found")

    content = pyproject_path.read_bytes()

try:
    import tomllib
except ModuleNotFoundError:
    text = content.decode("utf-8")
    in_project = False

    for line in text.splitlines():
        stripped = line.strip()

        if not stripped or stripped.startswith("#"):
            continue

        if stripped.startswith("[") and stripped.endswith("]"):
            in_project = stripped == "[project]"
            continue

        if in_project:
            match = re.match(r"""version\s*=\s*(['"])([^'"]+)\1\s*(?:#.*)?$""", stripped)

            if match:
                print(match.group(2))
                break
    else:
        sys.exit("project.version not found in pyproject.toml")
else:
    data = tomllib.loads(content.decode("utf-8"))

    try:
        print(data["project"]["version"])
    except KeyError:
        sys.exit("project.version not found in pyproject.toml")
PY
}

fallback_version_from_version_py() {
  local ref="${1:-}"

  python3 - "$ref" <<'PY'
import ast
import subprocess
import sys
from pathlib import Path

ref = sys.argv[1]

if ref:
    try:
        content = subprocess.check_output(
            ["git", "show", f"{ref}:mobipick_gui/version.py"],
            stderr=subprocess.DEVNULL,
        ).decode("utf-8")
    except subprocess.CalledProcessError:
        raise SystemExit(0)
else:
    version_path = Path("mobipick_gui/version.py")

    if not version_path.is_file():
        raise SystemExit(0)

    content = version_path.read_text(encoding="utf-8")

tree = ast.parse(content, filename="mobipick_gui/version.py")

for node in tree.body:
    if not isinstance(node, ast.Assign):
        continue

    for target in node.targets:
        if isinstance(target, ast.Name) and target.id == "_FALLBACK_VERSION":
            if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                print(node.value.value)
                raise SystemExit(0)

            sys.exit("_FALLBACK_VERSION is not a string literal")

sys.exit("_FALLBACK_VERSION not found in mobipick_gui/version.py")
PY
}

release_tag_candidates() {
  gh release list --limit 100 --json tagName --jq '.[].tagName' 2>/dev/null || true
  git tag --list 'v[0-9]*.[0-9]*.[0-9]*' 2>/dev/null || true
}

suggest_release_tag() {
  local package_version="$1"
  local release_tags

  release_tags="$(release_tag_candidates)"

  RELEASE_TAG_CANDIDATES="$release_tags" python3 - "$package_version" <<'PY'
import os
import re
import sys

package_version = sys.argv[1]
version_pattern = re.compile(r"^v([0-9]+)\.([0-9]+)\.([0-9]+)$")
package_pattern = re.compile(r"^([0-9]+)\.([0-9]+)\.([0-9]+)$")
released_versions = []

for tag in os.environ.get("RELEASE_TAG_CANDIDATES", "").splitlines():
    match = version_pattern.match(tag.strip())

    if match:
        released_versions.append(tuple(int(part) for part in match.groups()))

package_match = package_pattern.match(package_version)

if package_match:
    package_parts = tuple(int(part) for part in package_match.groups())
else:
    package_parts = None

if released_versions:
    major, minor, patch = max(released_versions)
    suggested_parts = (major, minor, patch + 1)

    if package_parts and package_parts > suggested_parts:
        suggested_parts = package_parts

    print("v{}.{}.{}".format(*suggested_parts))
elif package_parts:
    print(f"v{package_version}")
else:
    print(f"v{package_version}")
PY
}

default_release_tag() {
  local package_version
  package_version="$(package_version_from_pyproject)"
  suggest_release_tag "$package_version"
}

validate_package_version_matches_tag() {
  local tag="$1"
  local ref="${2:-}"
  local expected_version="${tag#v}"
  local package_version
  local fallback_version

  package_version="$(package_version_from_pyproject "$ref")"

  if [[ "$package_version" != "$expected_version" ]]; then
    die "Package version mismatch: tag is '$tag' but pyproject.toml has version '$package_version'. Expected '$expected_version'."
  fi

  fallback_version="$(fallback_version_from_version_py "$ref")"

  if [[ -n "$fallback_version" && "$fallback_version" != "$expected_version" ]]; then
    die "Package version mismatch: tag is '$tag' but mobipick_gui/version.py has _FALLBACK_VERSION = '$fallback_version'. Expected '$expected_version'."
  fi
}

current_branch() {
  git rev-parse --abbrev-ref HEAD
}

validate_tag() {
  local tag="$1"

  [[ -n "$tag" ]] || die "Tag is required"
  [[ "$tag" =~ ^v[0-9]+\.[0-9]+\.[0-9]+([.-][A-Za-z0-9._-]+)?$ ]] || {
    die "Tag '$tag' does not look like vMAJOR.MINOR.PATCH, for example v0.1.1"
  }
}

git_is_clean() {
  [[ -z "$(git status --porcelain)" ]]
}

main() {
  local tag=""
  local branch=""
  local remote="origin"
  local title=""
  local notes_mode="generated"
  local notes_file=""
  local draft=false
  local prerelease=false
  local target=""
  local do_pull=true

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --branch)
        branch="${2:-}"
        shift 2
        ;;
      --remote)
        remote="${2:-}"
        shift 2
        ;;
      --title)
        title="${2:-}"
        shift 2
        ;;
      --notes)
        notes_mode="${2:-}"
        shift 2
        ;;
      --notes-file)
        notes_file="${2:-}"
        shift 2
        ;;
      --draft)
        draft=true
        shift
        ;;
      --prerelease)
        prerelease=true
        shift
        ;;
      --target)
        target="${2:-}"
        shift 2
        ;;
      --no-pull)
        do_pull=false
        shift
        ;;
      --help)
        usage
        exit 0
        ;;
      -*)
        die "Unknown option: $1"
        ;;
      *)
        if [[ -z "$tag" ]]; then
          tag="$1"
        else
          die "Unexpected argument: $1"
        fi
        shift
        ;;
    esac
  done

  require_cmd git
  require_cmd gh
  require_cmd python3

  git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "Not inside a Git repository"
  gh auth status >/dev/null 2>&1 || die "GitHub CLI is not authenticated. Run: gh auth login"

  if [[ -z "$tag" ]]; then
    tag="$(ask "Release tag" "$(default_release_tag)")"
  fi

  validate_tag "$tag"

  if [[ -z "$branch" ]]; then
    branch="$(ask "Branch to release from" "$(current_branch)")"
  fi

  if [[ -z "$title" ]]; then
    title="$(ask "Release title" "$tag")"
  fi

  if [[ "$notes_mode" != "generated" && "$notes_mode" != "manual" && "$notes_mode" != "file" && "$notes_mode" != "none" ]]; then
    die "--notes must be one of: generated, manual, file, none"
  fi

  if [[ "$notes_mode" == "file" && -z "$notes_file" ]]; then
    notes_file="$(ask "Release notes file")"
  fi

  if [[ "$notes_mode" == "file" ]]; then
    [[ -f "$notes_file" ]] || die "Notes file not found: $notes_file"
  fi

  if ! git_is_clean; then
    git status --short
    die "Working tree is not clean. Commit, stash, or discard changes first."
  fi

  git fetch "$remote" --tags

  if git rev-parse "$tag" >/dev/null 2>&1; then
    die "Local tag already exists: $tag"
  fi

  if git ls-remote --tags "$remote" "refs/tags/$tag" | grep -q "$tag"; then
    die "Remote tag already exists: $tag"
  fi

  if gh release view "$tag" >/dev/null 2>&1; then
    die "GitHub Release already exists: $tag"
  fi

  git checkout "$branch"

  if [[ "$do_pull" == true ]]; then
    git pull --ff-only "$remote" "$branch"
  fi

  if [[ -z "$target" ]]; then
    target="$(git rev-parse HEAD)"
  else
    git cat-file -e "${target}^{commit}" 2>/dev/null || die "Target is not a valid commit: $target"
    target="$(git rev-parse "$target")"
  fi

  validate_package_version_matches_tag "$tag" "$target"

  local short_target
  short_target="$(git rev-parse --short "$target")"

  echo
  echo "Release plan"
  echo "Repository:  $(gh repo view --json nameWithOwner --jq .nameWithOwner)"
  echo "Remote:      $remote"
  echo "Branch:      $branch"
  echo "Tag:         $tag"
  echo "Title:       $title"
  echo "Target:      $short_target"
  echo "Notes mode:  $notes_mode"
  echo "Draft:       $draft"
  echo "Prerelease:  $prerelease"
  echo
  git log -1 --oneline "$target"
  echo

  ask_yes_no "Proceed and publish this release?" || {
    echo "Cancelled."
    exit 0
  }

  echo
  echo "Creating annotated tag..."
  git tag -a "$tag" "$target" -m "Release $tag"

  echo "Pushing tag..."
  git push "$remote" "$tag"

  local gh_args=(
    release create "$tag"
    --title "$title"
    --target "$target"
  )

  case "$notes_mode" in
    generated)
      gh_args+=(--generate-notes)
      ;;
    manual)
      echo
      echo "Enter release notes. Press Ctrl+D when done:"
      local notes
      notes="$(cat)"
      gh_args+=(--notes "$notes")
      ;;
    file)
      gh_args+=(--notes-file "$notes_file")
      ;;
    none)
      gh_args+=(--notes "")
      ;;
  esac

  if [[ "$draft" == true ]]; then
    gh_args+=(--draft)
  fi

  if [[ "$prerelease" == true ]]; then
    gh_args+=(--prerelease)
  fi

  echo "Creating GitHub Release..."
  gh "${gh_args[@]}"

  echo
  echo "Done."
  gh release view "$tag" --web
}

main "$@"
