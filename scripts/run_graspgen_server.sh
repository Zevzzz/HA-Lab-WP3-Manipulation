#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GRASPGEN_DIR="${ROOT_DIR}/deps/GraspGen"
MODELS_DIR="${GRASPGEN_DIR}/GraspGenModels"

if [[ ! -d "${GRASPGEN_DIR}" ]]; then
  echo "Error: GraspGen repo not found at ${GRASPGEN_DIR}"
  exit 1
fi

if [[ ! -d "${MODELS_DIR}" ]]; then
  echo "Error: GraspGenModels not found at ${MODELS_DIR}"
  exit 1
fi

cd "${GRASPGEN_DIR}"
bash docker/run.sh . --models "${MODELS_DIR}"
