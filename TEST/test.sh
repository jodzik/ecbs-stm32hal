#!/bin/bash

set -e

# Получаем абсолютный путь к директории, где находится скрипт
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Переходим в директорию скрипта
cd "$SCRIPT_DIR"

echo "=== Building tests ==="

mkdir -p build

gcc -std=gnu11 -g -Wall -Wextra \
    -I stub -I .. -I ../../ecbs-c -I ../../framer7b-c -I ../../safe-c -I ../../crc-c -I ../../byteorder-c \
    -DCONFIG_ECBS_MAX_PAYLOAD_SIZE=256 -DCONFIG_ECBS_MAX_ENDPOINTS=8 \
    -o build/test_ecbs_stm32hal \
    test.c ../ecbs_stm32hal.c ../../ecbs-c/ecbs.c ../../framer7b-c/framer7b.c \
    ../../crc-c/crc32.c ../../safe-c/safe_c.c ../../byteorder-c/byteorder.c

echo ""
echo "=== Running tests ==="
echo ""

set +e
./build/test_ecbs_stm32hal
RET="$?"
set -e

if [[ 0 = "$RET" ]]; then
    echo ""
    echo "=== Tests successfully completed ==="
else
    echo ""
    echo "=== Tests failed ==="
fi

exit "$RET"
