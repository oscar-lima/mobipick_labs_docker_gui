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

  git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "Not inside a Git repository"
  gh auth status >/dev/null 2>&1 || die "GitHub CLI is not authenticated. Run: gh auth login"

  if [[ -z "$tag" ]]; then
    tag="$(ask "Release tag" "v0.1.1")"
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
