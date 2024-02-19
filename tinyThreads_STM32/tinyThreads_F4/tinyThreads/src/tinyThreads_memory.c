#include "tinyThreads_memory.h"

// Memory pools for each size
uint8_t small_pool[CFG_MEM_POOLNUM_SMALL_BLOCKS][CFG_MEM_POOL_SMALL_SIZE_BYTES];
uint8_t medium_pool[CFG_MEM_POOLNUM_MEDIUM_BLOCKS][CFG_MEM_POOL_MEDIUM_SIZE_BYTES];
uint8_t large_pool[CFG_MEM_POOLNUM_LARGE_BLOCKS][CFG_MEM_POOL_LARGE_SIZE_BYTES];

// Allocation flags for each block
uint8_t small_alloc_flags[CFG_MEM_POOLNUM_SMALL_BLOCKS] = {0};
uint8_t medium_alloc_flags[CFG_MEM_POOLNUM_MEDIUM_BLOCKS] = {0};
uint8_t large_alloc_flags[CFG_MEM_POOLNUM_LARGE_BLOCKS] = {0};

// Function to allocate memory from the pool
void *allocate_memory(size_t size)
{
    if (size <= CFG_MEM_POOL_SMALL_SIZE_BYTES)
    {
        for (int i = 0; i < CFG_MEM_POOLNUM_SMALL_BLOCKS; i++)
        {
            if (!small_alloc_flags[i])
            {
                small_alloc_flags[i] = 1; // Mark as allocated
                return small_pool[i];
            }
        }
    }
    else if (size <= CFG_MEM_POOL_MEDIUM_SIZE_BYTES)
    {
        for (int i = 0; i < CFG_MEM_POOLNUM_MEDIUM_BLOCKS; i++)
        {
            if (!medium_alloc_flags[i])
            {
                medium_alloc_flags[i] = 1; // Mark as allocated
                return medium_pool[i];
            }
        }
    }
    else if (size <= CFG_MEM_POOL_LARGE_SIZE_BYTES)
    {
        for (int i = 0; i < CFG_MEM_POOLNUM_LARGE_BLOCKS; i++)
        {
            if (!large_alloc_flags[i])
            {
                large_alloc_flags[i] = 1; // Mark as allocated
                return large_pool[i];
            }
        }
    }

    // No available block
    return NULL;
}

// Function to free allocated memory back to the pool
void free_memory(void *ptr)
{
    // Check which pool the pointer belongs to and mark it as free
    for (int i = 0; i < CFG_MEM_POOLNUM_SMALL_BLOCKS; i++)
    {
        if (ptr == small_pool[i])
        {
            small_alloc_flags[i] = 0;
            return;
        }
    }

    for (int i = 0; i < CFG_MEM_POOLNUM_MEDIUM_BLOCKS; i++)
    {
        if (ptr == medium_pool[i])
        {
            medium_alloc_flags[i] = 0;
            return;
        }
    }

    for (int i = 0; i < CFG_MEM_POOLNUM_LARGE_BLOCKS; i++)
    {
        if (ptr == large_pool[i])
        {
            large_alloc_flags[i] = 0;
            return;
        }
    }
}