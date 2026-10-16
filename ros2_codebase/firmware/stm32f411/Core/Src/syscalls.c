#include "board.h"
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
int _gettimeofday(struct timeval *tv,void *timezone) {
  (void)timezone;
  if (!tv) { errno=EINVAL;return -1; }
  errno=ENOSYS;return -1; /* No libc wall clock; serial task manages command time. */
}
extern char __libc_heap_start, __libc_heap_end;
void *_sbrk(ptrdiff_t increment) {
  /* Bounded optional libc allocation, separate from FreeRTOS task storage. */
  static char *current=&__libc_heap_start;
  if ((increment>0 && increment>(&__libc_heap_end-current)) ||
      (increment<0 && increment<(&__libc_heap_start-current))) {
    errno=ENOMEM;return (void*)-1;
  }
  char *previous=current;current+=increment;return previous;
}
int _write(int fd,char *buffer,int len) { (void)fd;(void)buffer;return len; }
int _read(int fd,char *buffer,int len) { (void)fd;(void)buffer;(void)len;return 0; }
int _close(int fd) { (void)fd;errno=EBADF;return -1; }
int _fstat(int fd,struct stat *s) { (void)fd;s->st_mode=S_IFCHR;return 0; }
int _isatty(int fd) { (void)fd;return 1; }
int _lseek(int fd,int offset,int whence) { (void)fd;(void)offset;(void)whence;return 0; }
int _getpid(void) { return 1; }
int _kill(int pid,int signal) { (void)pid;(void)signal;errno=EINVAL;return -1; }
void _exit(int code) { (void)code;board_panic(); }
