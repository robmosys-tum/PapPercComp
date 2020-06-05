#!/bin/bash

FILE_ID="1--GlyeQ8ALd2rr6JDx3XFJt_67mooH78"
TARGET_FILE="../data/kitchen_classifier.onnx"

wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1--GlyeQ8ALd2rr6JDx3XFJt_67mooH78' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" -O ${TARGET_FILE} && rm -rf /tmp/cookies.txt
