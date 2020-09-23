#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
FILE_ID="1pWX9lu46fGFOr3TgtWKgyDT9NgsbV-UN"
TARGET_FILE="${SCRIPT_DIR}/../data/kitchen_classifier.onnx"

wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id='${FILE_ID} -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" -O "${TARGET_FILE}" && rm -rf /tmp/cookies.txt
