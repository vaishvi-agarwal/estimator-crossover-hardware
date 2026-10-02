# Lets the tests import the analysis scripts (analysis/*.py) as modules.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "analysis"))
