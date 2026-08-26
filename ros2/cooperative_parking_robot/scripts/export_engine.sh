#!/usr/bin/env bash
# .pt -> TensorRT .engine 변환 (젯슨에서만 실행)
#
# engine 파일은 GPU 아키텍처와 TensorRT 버전에 묶인다. 맥이나 PC에서 만든
# engine은 젯슨에서 안 돌고, JetPack을 올리면 다시 만들어야 한다.
# 그래서 반드시 이 젯슨에서 실행한다.
#
#   bash scripts/export_engine.sh ~/parkingbot/parking_vehicle_yolo11n_seg.pt
#   bash scripts/export_engine.sh <model.pt> [imgsz] [fp16|fp32]

set -e
MODEL="${1:?사용법: $0 <model.pt> [imgsz=640] [fp16|fp32]}"
IMGSZ="${2:-640}"
PRECISION="${3:-fp16}"

if [ ! -f "$MODEL" ]; then
  echo "모델 파일이 없습니다: $MODEL" >&2
  exit 1
fi

echo "=============================================================="
echo " TensorRT engine 변환"
echo "   모델      : $MODEL"
echo "   입력 크기 : ${IMGSZ}x${IMGSZ}   <- 런타임 imgsz 와 반드시 같아야 함"
echo "   정밀도    : $PRECISION"
echo "=============================================================="
echo

echo "--- 0. 전제 조건 확인 ---"
python3 - "$MODEL" <<'PY'
import sys
ok = True

# numpy 부터 본다. ROS Humble 의 cv_bridge 는 numpy 1.x 로 컴파일돼 있어서
# numpy 2 가 올라오면 cv2/torch/cv_bridge 가 한꺼번에 깨진다. pip 설치를
# 할 때마다 딸려 올라오기 쉬우므로 매번 확인한다.
try:
    import numpy
    major = int(numpy.__version__.split('.')[0])
    if major >= 2:
        ok = False
        print(f"  numpy        {numpy.__version__}  ** 2 이상 **")
        print('               pip3 install "numpy<2"')
        print('               재발 방지: echo "numpy<2" > ~/pip-constraints.txt')
        print('                          export PIP_CONSTRAINT=$HOME/pip-constraints.txt')
    else:
        print(f"  numpy        {numpy.__version__}")
except ImportError:
    ok = False
    print("  numpy        ** 없음 **")

try:
    import torch
    print(f"  torch        {torch.__version__}")
    if torch.cuda.is_available():
        print(f"  CUDA         사용 가능 · {torch.cuda.get_device_name(0)}")
    else:
        ok = False
        print("  CUDA         ** 사용 불가 **")
        print("               engine 변환은 CUDA 가 있어야 합니다.")
        print("               pip 로 설치한 PyTorch 는 젯슨 CUDA 와 맞지 않습니다.")
        print("               JetPack 용 PyTorch 휠로 교체해야 합니다.")
except ImportError:
    ok = False
    print("  torch        ** 없음 **")

try:
    import tensorrt
    print(f"  tensorrt     {tensorrt.__version__}")
except ImportError:
    ok = False
    print("  tensorrt     ** 없음 **  (JetPack 에 포함되어야 정상)")
    print("               sudo apt install python3-libnvinfer-dev")

try:
    import ultralytics
    print(f"  ultralytics  {ultralytics.__version__}")
except ImportError:
    ok = False
    print("  ultralytics  ** 없음 **  ->  pip3 install ultralytics")

# 변환 경로가 .pt -> ONNX -> TensorRT 라서 ONNX 쪽도 필요하다.
# onnxruntime-gpu 는 일반 PyPI 에 aarch64 빌드가 없어 ultralytics 의
# 자동 설치가 실패한다. 젯슨 인덱스에서 따로 받아야 한다.
for module, hint in (
        ('onnx', 'pip3 install "onnx>=1.12.0,<2.0.0"'),
        ('onnxslim', 'pip3 install onnxslim'),
        ('onnxruntime', 'pip3 install onnxruntime-gpu '
                        '--extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126/')):
    try:
        loaded = __import__(module)
        print(f"  {module:12} {getattr(loaded, '__version__', '?')}")
    except ImportError:
        ok = False
        print(f"  {module:12} ** 없음 **  ->  {hint}")

# 모델 종류 확인 (torch 없이도 zip 메타데이터로 확인 가능)
import zipfile, re
path = sys.argv[1]
if zipfile.is_zipfile(path):
    blob = b''
    with zipfile.ZipFile(path) as z:
        for name in z.namelist():
            if name.endswith('.pkl'):
                blob += z.read(name)
    text = blob.decode('latin-1')
    kind = ('segment' if 'SegmentationModel' in text
            else 'detect' if 'DetectionModel' in text else '?')
    print(f"  모델 종류    {kind}")
    if kind == 'segment':
        print("               -> 런타임에 model_mode:=vehicle_seg 를 쓰세요")

sys.exit(0 if ok else 2)
PY

status=$?
if [ $status -ne 0 ]; then
  echo
  echo "전제 조건이 갖춰지지 않아 중단합니다. 위 항목을 먼저 해결하세요." >&2
  exit $status
fi

echo
echo "--- 1. 변환 (몇 분 걸립니다. 진행 중 젯슨이 느려질 수 있습니다) ---"
HALF_FLAG="half=True"
[ "$PRECISION" = "fp32" ] && HALF_FLAG="half=False"

yolo export model="$MODEL" format=engine imgsz="$IMGSZ" "$HALF_FLAG" device=0

ENGINE="${MODEL%.*}.engine"
if [ ! -f "$ENGINE" ]; then
  echo "변환 실패: $ENGINE 가 생성되지 않았습니다" >&2
  exit 1
fi

echo
echo "--- 2. 결과 확인 ---"
ls -lh "$ENGINE"
python3 - "$ENGINE" "$IMGSZ" <<'PY'
import sys
from ultralytics import YOLO
engine, imgsz = sys.argv[1], int(sys.argv[2])
# .engine 은 task 정보가 없어 명시해야 한다. 이걸 빠뜨리면 segmentation
# 모델이 detect 로 열려 mask 가 사라진다.
model = YOLO(engine, task='segment')
print('  클래스 :', model.names)
import numpy as np
dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
result = model(dummy, imgsz=imgsz, verbose=False)[0]
has_mask = getattr(result, 'masks', None) is not None
print(f'  추론    OK · masks 속성 {"있음" if has_mask else "없음(빈 입력이라 정상)"}')
print()
print('  런타임에 이렇게 쓰세요:')
print(f'    model_path:={engine}')
print( '    model_mode:=vehicle_seg')
print(f'    inference_imgsz:={imgsz}      <- 변환 크기와 반드시 동일')
PY

echo
echo "=============================================================="
echo " 완료: $ENGINE"
echo "=============================================================="
