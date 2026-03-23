#!/bin/bash
set -e

FQBN="esp8266:esp8266:generic"
SKETCH_DIR="/tmp/stick"
SOURCE="$(dirname "$0")/stick.ino"
PORT="${1:-/dev/ttyUSB0}"

echo "=== AIR DRUM — Gravando STICK (ESP8266) ==="
echo "Porta: $PORT"
echo ""

mkdir -p "$SKETCH_DIR"
cp "$SOURCE" "$SKETCH_DIR/stick.ino"

echo "[1/2] Compilando..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

echo ""
echo "[2/2] Gravando em $PORT..."
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo ""
echo "Stick gravado com sucesso!"
echo ""
echo "Abrindo monitor serial (CTRL+C para sair)..."
arduino-cli monitor --port "$PORT" --config baudrate=115200
