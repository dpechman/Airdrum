#!/bin/bash
set -e

FQBN="esp32:esp32:adafruit_matrixportal_esp32s3"
SKETCH_DIR="/tmp/central"
SOURCE="$(dirname "$0")/central.ino"
PORT="${1:-/dev/ttyACM0}"

echo "=== AIR DRUM — Gravando CENTRAL (ESP32-S3) ==="
echo "Porta: $PORT"
echo ""

mkdir -p "$SKETCH_DIR"
cp "$SOURCE" "$SKETCH_DIR/central.ino"

echo "[1/2] Compilando..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

echo ""
echo "[2/2] Gravando em $PORT..."
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo ""
echo "Central gravada com sucesso!"
echo ""
echo "Abrindo monitor serial (CTRL+C para sair)..."
arduino-cli monitor --port "$PORT" --config baudrate=115200
