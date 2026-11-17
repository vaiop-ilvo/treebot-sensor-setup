#!/usr/bin/env bash
# Downloads ONNX Runtime and the pretrained segmentation model(s) for the segmentation colour mode.
# Usage: scripts/download_models.sh [--gpu] [b0|b1|b2|b4 ...]   (default: b0)
#   ONNX Runtime -> thirdparty/onnxruntime (skipped when present; --gpu replaces a CPU build by the GPU build)
#   --gpu: also the CUDA 12 / cuDNN 9 libraries the GPU build needs -> thirdparty/cuda/lib (NVIDIA's pip wheels,
#          no root needed; the application loads them from there). Needs the NVIDIA driver (nvidia-smi must work).
#   SegFormer ADE20K (150 classes, ONNX export of Xenova on Hugging Face) -> models/segformer-<size>-ade
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORT_VERSION=1.20.1
ORT_DIR="$ROOT/thirdparty/onnxruntime"
CUDA_DIR="$ROOT/thirdparty/cuda"
# CUDA 12 / cuDNN 9 runtime libraries for the GPU build of ONNX Runtime 1.20
CUDA_WHEELS=(
    nvidia-cuda-runtime-cu12==12.6.77
    nvidia-cuda-nvrtc-cu12==12.6.85
    nvidia-cublas-cu12==12.6.4.1
    nvidia-cufft-cu12==11.3.0.4
    nvidia-curand-cu12==10.3.7.77
    nvidia-cudnn-cu12==9.5.0.50
)

GPU=0
SIZES=()
for arg in "$@"; do
    case "$arg" in
        --gpu) GPU=1 ;;
        *) SIZES+=("$arg") ;;
    esac
done
[ ${#SIZES[@]} -eq 0 ] && SIZES=(b0)

ort_present() { [ -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ]; }
ort_is_gpu() { [ -f "$ORT_DIR/lib/libonnxruntime_providers_cuda.so" ]; }

if ! ort_present || { [ $GPU -eq 1 ] && ! ort_is_gpu; }; then
    package="onnxruntime-linux-x64-$ORT_VERSION"
    [ $GPU -eq 1 ] && package="onnxruntime-linux-x64-gpu-$ORT_VERSION"
    echo "Downloading $package"
    rm -rf "$ORT_DIR"
    mkdir -p "$ORT_DIR"
    curl -fSL "https://github.com/microsoft/onnxruntime/releases/download/v$ORT_VERSION/$package.tgz" \
        | tar xz -C "$ORT_DIR" --strip-components=1
fi

if [ $GPU -eq 1 ] && [ ! -f "$CUDA_DIR/lib/libcudnn.so.9" ]; then
    echo "Downloading the CUDA 12 / cuDNN 9 libraries (about 1.8 GB) to $CUDA_DIR"
    wheels="$(mktemp -d)"
    trap 'rm -rf "$wheels"' EXIT
    python3 -m pip download --quiet --no-deps --only-binary=:all: --platform manylinux2014_x86_64 --platform manylinux_2_27_x86_64 \
        --python-version 3.12 -d "$wheels" "${CUDA_WHEELS[@]}"
    mkdir -p "$CUDA_DIR/lib"
    for wheel in "$wheels"/*.whl; do
        unzip -q -o "$wheel" 'nvidia/*/lib/*.so*' -d "$wheels/x"
    done
    find "$wheels/x" -name '*.so*' -exec mv -f {} "$CUDA_DIR/lib/" \;
    # Not used by ONNX Runtime (libnvblas would replace the BLAS of the whole program when loaded)
    rm -f "$CUDA_DIR"/lib/libnvblas.so* "$CUDA_DIR"/lib/libcufftw.so*
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
