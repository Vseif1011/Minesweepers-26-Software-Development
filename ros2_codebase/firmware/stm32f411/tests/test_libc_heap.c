#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
_Alignas(8) unsigned char test_heap[1024];
__asm__(".global __libc_heap_start\n.set __libc_heap_start,test_heap\n"
        ".global __libc_heap_end\n.set __libc_heap_end,test_heap+1024\n");
void *_sbrk(ptrdiff_t increment);
void board_panic(void) { __builtin_trap(); }
int main(void) {
  assert(_sbrk(0)==test_heap);
  assert(_sbrk(24)==test_heap); /* newlib srand internal state */
  assert(_sbrk(0)==test_heap+24);
  errno=0;assert(_sbrk(1001)==(void*)-1 && errno==ENOMEM);
  assert(_sbrk(0)==test_heap+24);
  assert(_sbrk(-24)==test_heap+24);
  assert(_sbrk(1024)==test_heap);
  assert(_sbrk(1)==(void*)-1);
  assert(_sbrk(-1025)==(void*)-1);
  assert(_sbrk(PTRDIFF_MIN)==(void*)-1);
  assert(_sbrk(-1024)==test_heap+1024);
  assert(_sbrk(-1)==(void*)-1);
  puts("newlib heap tests: srand allocation, exact bounds, exhaustion and negative-increment limits passed");
}
