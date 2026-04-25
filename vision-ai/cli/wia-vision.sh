#!/bin/bash
# WIA-AI-021 Vision AI — CLI helper
# wia-vision.sh — submit inference requests to a WIA-AI-021 v1 endpoint
#
# Usage:
#   wia-vision.sh detect <image-uri>
#   wia-vision.sh segment <image-uri>
#   wia-vision.sh ocr <image-uri> [--lang=en,ko]
#   wia-vision.sh stream <rtsp-uri>
#   wia-vision.sh discovery
#   wia-vision.sh help
#
# Environment:
#   WIA_VISION_ENDPOINT  — base URL, e.g. https://vision.example.org/wia-vision-ai/v1
#   WIA_VISION_TOKEN     — OAuth 2.1 bearer token (RFC 9700)

set -uo pipefail

VERB="${1:-help}"

require_endpoint() {
    if [ -z "${WIA_VISION_ENDPOINT:-}" ]; then
        echo "error: WIA_VISION_ENDPOINT is not set" >&2
        exit 2
    fi
    if [ -z "${WIA_VISION_TOKEN:-}" ]; then
        echo "error: WIA_VISION_TOKEN is not set" >&2
        exit 2
    fi
}

case "$VERB" in
    discovery)
        require_endpoint
        curl -fsSL \
            -H "Accept: application/json" \
            "${WIA_VISION_ENDPOINT}/.well-known/wia-vision-ai"
        ;;
    detect|segment|ocr)
        require_endpoint
        URI="${2:?missing input image URI}"
        BODY=$(printf '{"modelId":"default","input":{"kind":"image","uri":"%s"},"options":{"confidenceThreshold":0.5}}' "$URI")
        curl -fsSL \
            -H "Authorization: Bearer ${WIA_VISION_TOKEN}" \
            -H "Content-Type: application/json" \
            -X POST \
            --data "$BODY" \
            "${WIA_VISION_ENDPOINT}/${VERB}"
        ;;
    stream)
        require_endpoint
        URI="${2:?missing RTSP URI}"
        BODY=$(printf '{"modelId":"default","input":{"kind":"stream","uri":"%s"},"options":{"explainability":false}}' "$URI")
        curl -fsSL \
            -H "Authorization: Bearer ${WIA_VISION_TOKEN}" \
            -H "Content-Type: application/json" \
            -X POST \
            --data "$BODY" \
            "${WIA_VISION_ENDPOINT}/stream"
        ;;
    help|*)
        sed -n '1,16p' "$0"
        ;;
esac
