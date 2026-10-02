#!/usr/bin/env bash
# zeliboba - run a Windows build from WSL with the ZLB_* diagnostics forwarded.
#
#   ./run-wsl.sh --cli -ex "boot" -ex "runm 300000" -ex "quit"
#   ZLB_ARM_PC_LOG=scratch/pc.log ./run-wsl.sh --cli -ex "loadstate ..." ...
#   ZLB_BIN=zeliboba_ui ./run-wsl.sh --screenshot out.bmp --screenshot-tab panel
#
# Why this exists: the emulator is a Windows executable, and WSL does **not** pass
# the Linux environment to Windows processes by default - a `ZLB_*` variable set in
# a WSL shell is invisible to zeliboba.exe unless its name is listed in WSLENV.
# Every diagnostic in docs/DEBUGGER.md silently does nothing without it (the run
# looks like the feature is broken).  This wrapper collects the ZLB_* variables,
# publishes them through WSLENV and execs the requested binary out of build/bin.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
binary="${ZLB_BIN:-zeliboba}"

forward=""
for name in $(compgen -v | grep '^ZLB_' | grep -v '^ZLB_BIN$' | sort); do
    forward="${forward:+$forward:}$name"
done
if [ -n "$forward" ]; then
    export WSLENV="${WSLENV:+$WSLENV:}$forward"
    # Path-valued diagnostics keep working because the binary's CWD is this
    # directory, so relative paths resolve next to build/.
    printf '[run-wsl] forwarding %s\n' "$forward" >&2
fi

exe="$root/build/bin/$binary.exe"
if [ ! -x "$exe" ]; then
    printf 'run-wsl: %s does not exist - build it first:\n' "$exe" >&2
    printf '  msbuild zeliboba.slnx -p:Configuration=Release -p:Platform=x64 -m\n' >&2
    exit 1
fi

cd "$root"
exec "$exe" "$@"
