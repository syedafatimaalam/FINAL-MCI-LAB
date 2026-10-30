#include "heap_driver.h"

// 0 = Free block
// 1 = Occupied block
static uint8_t block_map[BLOCK_COUNT];

// Stores the number of allocated blocks
// Only the first block of an allocation
// contains its allocation length.
static uint16_t allocation_size[BLOCK_COUNT];


// ========================================
// FUNCTION 1: INITIALIZE HEAP
// ========================================

void heap_init(void)
{
    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        block_map[i] = 0;
        allocation_size[i] = 0;
    }
}


// ========================================
// FUNCTION 2: ALLOCATE MEMORY
// ========================================

void *heap_alloc(size_t size)
{
    if (size == 0 || size > HEAP_SIZE)
    {
        return NULL;
    }

    // Calculate number of required blocks
    size_t blocks_needed =
        (size / BLOCK_SIZE) +
        ((size % BLOCK_SIZE) != 0U);

    // Search for consecutive free blocks
    for (size_t i = 0;
         i + blocks_needed <= BLOCK_COUNT;
         i++)
    {
        int found = 1;

        for (size_t j = 0; j < blocks_needed; j++)
        {
            if (block_map[i + j] != 0)
            {
                found = 0;
                break;
            }
        }

        // If enough consecutive blocks are available
        if (found)
        {
            // Mark blocks as occupied
            for (size_t j = 0; j < blocks_needed; j++)
            {
                block_map[i + j] = 1;
            }

            // Store allocation size
            allocation_size[i] =
                (uint16_t)blocks_needed;

            // Calculate SRAM address
            uintptr_t address =
                HEAP_START_ADDR + i * BLOCK_SIZE;

            return (void *)address;
        }
    }

    // No sufficient contiguous memory
    return NULL;
}


// ========================================
// FUNCTION 3: FREE MEMORY
// ========================================

void heap_free(void *ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    uintptr_t address = (uintptr_t)ptr;

    // Check valid heap address
    if (address < HEAP_START_ADDR ||
        address >= HEAP_START_ADDR + HEAP_SIZE)
    {
        return;
    }

    // Calculate offset
    uintptr_t offset =
        address - HEAP_START_ADDR;

    // Check pointer alignment
    if ((offset % BLOCK_SIZE) != 0)
    {
        return;
    }

    // Find starting block
    size_t index = offset / BLOCK_SIZE;

    // Get number of allocated blocks
    size_t blocks = allocation_size[index];

    // Reject invalid or already-freed pointers
    if (blocks == 0 ||
        blocks > BLOCK_COUNT - index)
    {
        return;
    }

    // Release blocks
    for (size_t i = 0; i < blocks; i++)
    {
        block_map[index + i] = 0;
    }

    // Clear allocation record
    allocation_size[index] = 0;
}