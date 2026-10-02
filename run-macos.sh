#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary=zeliboba_ui
if [[ "${1:-}" == --cli ]]; then
    binary=zeliboba
    shift
fi

if [[ ! -x "$repo_dir/build/bin/$binary" ]]; then
    printf 'Executable not found: %s/build/bin/%s\n' "$repo_dir" "$binary" >&2
    if [[ "$binary" == zeliboba_ui ]]; then
        printf 'Build with ./build-macos.sh (SDL3: brew install sdl3), or run ./run-macos.sh --cli.\n' >&2
    else
        printf 'Build first with ./build-macos.sh --headless.\n' >&2
    fi
    exit 1
fi

exec "$repo_dir/build/bin/$binary" "$@"
