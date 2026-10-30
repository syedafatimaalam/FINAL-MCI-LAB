#ifndef HEAP_DRIVER_H
#define HEAP_DRIVER_H

#include <stdint.h>
#include <stddef.h>

#define HEAP_START_ADDR  0x20001000UL
#define HEAP_SIZE        4096U
#define BLOCK_SIZE       16U
#define BLOCK_COUNT      256U

void heap_init(void);
void *heap_alloc(size_t size);
void heap_free(void *ptr);

#endif