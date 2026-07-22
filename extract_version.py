# SPDX-License-Identifier: Apache-2.0
#
# PlatformIO pre-build script (see platformio.ini's extra_scripts). Injects
# the real git version/lineage into the firmware as a compile-time define -
# MK1_FW_VERSION (main/sketch.cpp) resolves to this if present, so the
# on-screen "FW:" string is always traceable to an actual commit/tag instead
# of a hand-maintained constant someone forgets to bump (see the "board
# shows FW: v0.11.0 on v1.2.1 hardware" bench finding this was written to
# fix - the old hardcoded string was last touched in WO11 and never updated
# again across five subsequent releases).
#
# `git describe --tags --always --dirty` gives exactly what we want:
#   - "v1.2.1"                 - built exactly at that tag, clean tree
#   - "v1.2.1-3-g95708da"      - 3 commits past v1.2.1, clean tree
#   - "v1.2.1-3-g95708da-dirty" - same, but with uncommitted local changes
#   - "95708da"                 - no tags reachable at all (--always fallback)
# The "-N-g<hash>[-dirty]" suffix is not a bug to hide - it's the whole
# point: it makes drift from the last tagged release visible on the
# device itself, which a bare "v1.2.1" string could never do.
#
# Runs via subprocess (not the PlatformIO "!" inline-shell-command syntax)
# specifically for Windows portability - "!" build_flags entries are
# executed through the OS default shell (cmd.exe here), which does not
# understand POSIX command substitution: os-independent subprocess.
# check_output() sidesteps that entirely.

Import("env")

import subprocess

try:
    version = subprocess.check_output(
        ["git", "describe", "--tags", "--always", "--dirty"],
        cwd=env["PROJECT_DIR"],
        stderr=subprocess.DEVNULL,
    ).decode().strip()
except Exception:
    # No git available, not a git repo, or some other failure - fall back
    # to a value that's obviously not a real version rather than silently
    # lying with a stale one.
    version = "unknown"

env.Append(CPPDEFINES=[("MK1_GIT_VERSION", '\\"%s\\"' % version)])
