#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
enable_sdl3=ON
configure_args=(-DCMAKE_BUILD_TYPE=Release)

while (($#)); do
    case "$1" in
        --headless) enable_sdl3=OFF ;;
        -h|--help)
            printf 'Usage: %s [--headless] [CMake configure options]\n' "$0"
            printf 'Builds the CLI, tools, tests, and SDL3 frontend in build/bin.\n'
            exit 0
            ;;
        *) configure_args+=("$1") ;;
    esac
    shift
done

if ! command -v cmake >/dev/null 2>&1; then
    printf 'CMake is required. Install it with: brew install cmake sdl3\n' >&2
    exit 1
fi
if ! xcrun --find clang++ >/dev/null 2>&1; then
    printf 'Apple command line tools are required. Run: xcode-select --install\n' >&2
    exit 1
fi

# Homebrew may be outside CMake's default search path, especially on Apple Silicon.
if [[ "$enable_sdl3" == ON ]] && command -v brew >/dev/null 2>&1; then
    if sdl3_prefix="$(brew --prefix sdl3 2>/dev/null)"; then
        configure_args=("-DCMAKE_PREFIX_PATH=$sdl3_prefix" "${configure_args[@]}")
    fi
fi

cmake -S "$repo_dir" -B "$repo_dir/build" \
    -DZLB_ENABLE_SDL3="$enable_sdl3" \
    "${configure_args[@]}"
build_jobs="$(sysctl -n hw.logicalcpu 2>/dev/null || printf '2')"
cmake --build "$repo_dir/build" --parallel "$build_jobs"
printf 'Built executables: %s/build/bin\n' "$repo_dir"
