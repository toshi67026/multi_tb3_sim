# !/bin/bash

CONFIG_FILE_PATH="$(git rev-parse --show-toplevel)/mypy.ini"
echo "config_file: "${CONFIG_FILE_PATH}
CONFIG_FILE_OPTION="--config-file=${CONFIG_FILE_PATH}"

find . -name "*.py" \
    -not -path "*/.*/*" \
    -not -path "setup.py" \
    | xargs python3 -m mypy ${CONFIG_FILE_OPTION} $* \
    | uniq
