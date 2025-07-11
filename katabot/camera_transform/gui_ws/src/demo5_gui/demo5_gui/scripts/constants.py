from pathlib import Path
PATH_FILE_DIR = Path(__file__).parent
PATH_MODELS_DIR = PATH_FILE_DIR / 'models'
PATH_ROOT = Path(__file__).parents[2]

SURFACE_MODEL_PATH = PATH_MODELS_DIR / 'segment_best.onnx'
PART_MODEL_PATH = PATH_MODELS_DIR / 'detection_best.onnx'
SURFACE_CLASSES = ['back', 'front', 'left', 'right', 'top', 'bottom']
PART_CLASSES = ['screw1', 'screw2', 'screw3', 'screw4'] # 添加你所有的零件类别
