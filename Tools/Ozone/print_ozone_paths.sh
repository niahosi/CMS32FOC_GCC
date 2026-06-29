#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)

echo "Board watch ELF:"
wslpath -w "$ROOT_DIR/build/gcc-debug/cms32_board_watch_test"
echo
echo "Main firmware ELF:"
wslpath -w "$ROOT_DIR/build/gcc-debug/cms32foc"
echo
echo "Ozone project:"
wslpath -w "$ROOT_DIR/Tools/Ozone/cms32_board_watch_test.jdebug"
