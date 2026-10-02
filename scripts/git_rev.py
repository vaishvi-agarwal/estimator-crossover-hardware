"""Print a compiler flag that stamps the git commit into the firmware.

PlatformIO runs this script during every build (see `build_flags` in
platformio.ini) and adds whatever it prints to the compiler command line.
The firmware prints the commit in its `#` metadata lines, and logger.py saves
it in every session's JSON sidecar (test card field 7: device identity).

If the working tree has uncommitted changes, "-dirty" is appended, so you can
tell when data came from code that was never committed.

(Copied from the sel-sefi-sensor-guard repository.)
"""
import subprocess


def git(*args):
    try:
        return subprocess.check_output(["git", *args], stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return ""


commit = git("rev-parse", "--short=10", "HEAD") or "unknown"
if git("status", "--porcelain", "--untracked-files=no"):
    commit += "-dirty"

print("-DFW_GIT_COMMIT='\"%s\"'" % commit)
