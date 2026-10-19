#!/usr/bin/env bash
# Downloads ONNX Runtime and the pretrained segmentation model(s) for the segmentation colour mode.
# Usage: scripts/download_models.sh [b0|b1|b2|b4 ...]   (default: b0)
#   ONNX Runtime -> thirdparty/onnxruntime (skipped when present)
#   SegFormer ADE20K (150 classes, ONNX export of Xenova on Hugging Face) -> models/segformer-<size>-ade
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORT_VERSION=1.20.1
SIZES=("$@")
[ ${#SIZES[@]} -eq 0 ] && SIZES=(b0)

if [ ! -f "$ROOT/thirdparty/onnxruntime/include/onnxruntime_cxx_api.h" ]; then
    echo "Downloading ONNX Runtime $ORT_VERSION"
    mkdir -p "$ROOT/thirdparty/onnxruntime"
    curl -fSL "https://github.com/microsoft/onnxruntime/releases/download/v$ORT_VERSION/onnxruntime-linux-x64-$ORT_VERSION.tgz" \
        | tar xz -C "$ROOT/thirdparty/onnxruntime" --strip-components=1
fi

for size in "${SIZES[@]}"; do
    dir="$ROOT/models/segformer-$size-ade"
    url="https://huggingface.co/Xenova/segformer-$size-finetuned-ade-512-512/resolve/main"
    echo "Downloading SegFormer-${size^^} ADE20K to $dir"
    mkdir -p "$dir"
    curl -fSL -o "$dir/model.onnx" "$url/onnx/model.onnx"
    curl -fsSL -o "$dir/config.json" "$url/config.json"
    curl -fsSL -o "$dir/preprocessor_config.json" "$url/preprocessor_config.json"
done

echo "Done. Re-run cmake in build/ when ONNX Runtime was just downloaded."
