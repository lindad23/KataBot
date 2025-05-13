from pathlib import Path

PATH_ROOT = Path(__file__).parent
PATH_LOGS = PATH_ROOT.parent / "logs"
PATH_LOGS.mkdir(exist_ok=True)
