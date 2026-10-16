#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root_dir/tests/output"
"${HOST_CC:-gcc}" -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root_dir/Control/Inc" "$root_dir/Control/Src/control.c" \
  "$root_dir/tests/test_control.c" -lm -o "$root_dir/tests/output/test_control"
"$root_dir/tests/output/test_control"
"${HOST_CC:-gcc}" -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root_dir/tests/mocks" -I"$root_dir/Control/Inc" \
  "$root_dir/Drivers/Board/Src/motor.c" "$root_dir/tests/test_motor.c" \
  -lm -o "$root_dir/tests/output/test_motor"
"$root_dir/tests/output/test_motor"
"${HOST_CC:-gcc}" -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root_dir/tests/mocks" -I"$root_dir/Control/Inc" -I"$root_dir/Drivers/Board/Inc" \
  "$root_dir/Drivers/Board/Src/mpu6050.c" "$root_dir/tests/test_mpu6050.c" \
  -lm -o "$root_dir/tests/output/test_mpu6050"
"$root_dir/tests/output/test_mpu6050"
"${HOST_CC:-gcc}" -std=gnu11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root_dir/tests/mocks" -I"$root_dir/Control/Inc" \
  "$root_dir/Core/Src/syscalls.c" "$root_dir/tests/test_libc_heap.c" \
  -o "$root_dir/tests/output/test_libc_heap"
"$root_dir/tests/output/test_libc_heap"
python3 "$root_dir/tests/check_project.py"
