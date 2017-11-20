/*
* Flash-based Non-Volatile Memory (NVM)
* 
* This file supports storing and loading persistent configuration based on
* the STM32 builtin flash memory.
*
* The STM32F405xx has 12 flash sectors of heterogeneous size. We use the last
* two sectors for configuration data. These pages have a size of 128kB each.
* Setting any bit in these sectors to 0 is always possible, but setting them
* to 1 requires erasing the whole sector.
*
* We consider each sector as an array of 64-bit fields and use the beginning
* as an allocation block. The allocation block keeps track of the state of each
* field (erased, invalid, valid).
*
* One sector is always considered the valid sector and the other one is the
* victim for the next write access. On startup, if there is exactly one sector
* whose last non-erased value has the state "valid" that sector is considered
* the valid sector. In any other case the selection is undefined.
*
* When writing a block of data, instead of always erasing the whole victim sector the
* new data is appended in the erased area. This presumably increases flash life span.
* The victim sector is only erased if there is not enough space for the new data.
*
* To write a new block of data atomically we first set all associated allocation
* states to "invalid" then write the data and then set the allocation states
* to "valid" (in the direction of increasing address).
*/

#include "nvm.h"

#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <cmsis_os.h>

#if defined(STM32F405xx)
//__attribute__((__section__(".flash_sector_10")))
//const volatile uint8_t flash_sector_10[0x20000];
//__attribute__((__section__(".flash_sector_11")))
//const volatile uint8_t flash_sector_11[0x20000];
#define FLASH_SECTOR_10_BASE (const volatile uint8_t*)0x80C0000UL
#define FLASH_SECTOR_10_SIZE 0x20000UL
#define FLASH_SECTOR_11_BASE (const volatile uint8_t*)0x80E0000UL
#define FLASH_SECTOR_11_SIZE 0x20000UL

#define HAL_FLASH_ClearError() __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR)
#else
#error "unknown flash sector size"
#endif

typedef enum {
    VALID = 0,
    INVALID = 1,
    ERASED = 3
} field_state_t;

typedef struct {
    size_t index;               //!< next field to be written to (can be equal to n_data)
    const uint32_t sector_id;   //!< HAL ID of this sector
    const size_t n_data;        //!< number of 64-bit fields in this sector
    const size_t n_reserved;    //!< number of 64-bit fields in this sector that are reserved for the allocation table
    const volatile uint8_t* const alloc_table;
    const volatile uint64_t* const data;
} sector_t;

sector_t sectors[] = { {
    .sector_id = FLASH_SECTOR_10,
    .n_data = FLASH_SECTOR_10_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_10_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_10_BASE,
    .data = (uint64_t *)FLASH_SECTOR_10_BASE
}, {
    .sector_id = FLASH_SECTOR_11,
    .n_data = FLASH_SECTOR_11_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_11_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_11_BASE,
    .data = (uint64_t *)FLASH_SECTOR_11_BASE
}};

uint8_t valid_sector_;

// @brief Erases a flash sector. This sets all bits in the sector to 1.
// The sector's current index is reset to the minimum value (n_reserved).
// @returns 0 on success or a non-zero error code otherwise
int erase(sector_t *sector) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = 0, // only used for mass erase
        .Sector = sector->sector_id,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    uint32_t sector_error;
    if (HAL_FLASHEx_Erase(&erase_struct, &sector_error) != HAL_OK)
        goto fail;
    sector->index = sector->n_reserved;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    //printf("erase failed: %u \r\n", HAL_FLASH_GetError());
    return HAL_FLASH_GetError(); // non-zero
}


// @brief Writes states into the allocation table.
// The write operation goes in the direction of increasing indices.
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 on success or a non-zero error code otherwise
int set_allocation_state(sector_t *sector, size_t index, size_t count, field_state_t state) {
    if (index < sector->n_reserved)
        return -1;
    if (index + count >= sector->n_data)
        return -1;

    // expand state to state for 4 values
    const uint8_t states = (state << 0) | (state << 2) | (state << 4) | (state << 6);
    
    // handle unaligned start
    uint8_t mask = ~(0xff << ((index & 0x3) << 1));
    count += index & 0x3;
    index -= index & 0x3;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // write states
    for (; count >= 4; count -= 4, index += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
        mask = 0;
    }

    // handle unaligned end
    if (count) {
        mask |= ~(0xff >> ((4 - count) << 1));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
    }
    
    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Reads the allocation table from behind to determine how many fields match the
// reference state.
// @param sector: The sector on which to perform the search
// @param max_index: The maximum index that should be considered
// @param ref_state: The reference state
// @param state: Set to the first encountered state that is unequal to ref_state.
//               Set to ref_state if all encountered states are equal to ref_state.
// @returns The smallest index that points to a field with ref_state.
//          This value is at least sector->n_reserved and at most max_index.
size_t scan_allocation_table(sector_t *sector, size_t max_index, field_state_t ref_state, field_state_t *state) {
    const uint8_t ref_states = (ref_state << 0) | (ref_state << 2) | (ref_state << 4) | (ref_state << 6);
    size_t index = (((max_index + 3) >> 2) << 2); // start at the max index but round up to a multiple of 4
    size_t ignore = index - max_index;
    uint8_t states = ref_states;

    //printf("scan from %08x to %08x for %02x\r\n", index, sector->n_reserved, ref_states); osDelay(5);

    // read 4 states at a time
    for (; index >= (sector->n_reserved + 4); index -= 4) {
        states = sector->alloc_table[(index - 1) >> 2];
        if (ignore) { // ignore the upper 1, 2 or 3 states if max_index was unaligned
            uint8_t ignore_mask = ~(0xff >> (ignore << 1));
            states = (states & ~ignore_mask) | (ref_states & ignore_mask);
            ignore = 0;
        }
        if (states != ref_states)
            break;
    }

    // once we encounterd a byte with any state mismatch determine which of the 4 states it is
    for (; ((states >> 6) == (ref_states & 0x3)) && (index > sector->n_reserved); index--) {
        states <<= 2;
    }
    
    *state = states >> 6;
    //printf("(it's %02x)\r\n", index); osDelay(5);
    return index;
}

// Loads the head of the NVM data.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise
int NVM_init(void) {
    field_state_t sector0_state, sector1_state;
    sectors[0].index = scan_allocation_table(&sectors[0], sectors[0].n_data,
                ERASED, &sector0_state);
    sectors[1].index = scan_allocation_table(&sectors[1], sectors[1].n_data,
                ERASED, &sector1_state);
    //printf("sector states: %02x, %02x\r\n", sector0_state, sector1_state); osDelay(5);

    // Select valid sector on a best effort basis
    // (in unfortunate cases valid_sector might actually point
    // to an invalid or erased sector)
    valid_sector_ = 0;
    if (sector1_state == VALID)
        valid_sector_ = 1;
    
    int state = 0;
    /*// bring non-valid sectors into a known state
    this is not absolutely required
    if (sector0_state != VALID)
        state |= erase(&sectors[0]);
    if (sector1_state != VALID)
        state |= erase(&sectors[1]);
    */
    return state;
}

// @brief Erases all data in the NVM.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise
int NVM_erase(void) {
    valid_sector_ = 0;
    sectors[0].index = sectors[0].n_reserved;
    sectors[1].index = sectors[1].n_reserved;

    int state = 0;
    state |= erase(&sectors[0]);
    state |= erase(&sectors[1]);
    return state;
}

// @brief Reads the last valid 64-bit values of the non-volatile memory.
// @returns 0 on success or a non-zero error code otherwise
int NVM_read_tail(uint64_t *data, size_t *length) {
    sector_t *sector = &sectors[valid_sector_];

    // make sure we only read valid data
    uint8_t first_nonvalid_state;
    size_t min_index = scan_allocation_table(sector, sector->index,
        VALID, &first_nonvalid_state);
    if (*length > sector->index - min_index)
        *length = sector->index - min_index;

    const uint64_t *src_ptr = (const uint64_t *)&sector->data[sector->index - *length]; // cast away volatile
    memcpy(data, src_ptr, *length * sizeof(data[0]));
    return 0;
}

// @brief Atomically appends an array of 64-bit values to the non-volatile memory.
//
// The block size must be at most equal to the size indicated by max_append_size() (TODO).
// This operation is atomic even if power is lost. If the operation fails, the last
// successful append() operation remains untouched.
//
// @returns 0 on success or a non-zero error code otherwise
int NVM_append(uint64_t *data, size_t length) {
    int status = 0;
    sector_t *victim = &sectors[1 - valid_sector_];
    if (length > victim->n_data - victim->n_reserved)
        return -1;

    // make room for the new data
    if (length > victim->n_data - victim->index)
        if ((status = erase(victim)))
            return status;

    // invalidate the fields we're about to write
    status = set_allocation_state(victim, victim->index, length, INVALID);
    if (status)
        return status;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    for (size_t i = 0; i < length; ++i) {
        //printf("write %08x <= %08x\r\n", (uintptr_t)&victim->data[victim->index + i], data[i]); osDelay(5);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                (uintptr_t)&victim->data[victim->index + i], (uint32_t)data[i]) != HAL_OK)
            goto fail;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                (uintptr_t)&victim->data[victim->index + i] + 4, (uint32_t)(data[i] >> 32)) != HAL_OK)
            goto fail;
    }
    HAL_FLASH_Lock();

    // set the newly-written fields to valid
    status = set_allocation_state(victim, victim->index, length, VALID);
    victim->index += length;
    if (status)
        return status;

    // invalidate the other sector
    if (sectors[valid_sector_].index < sectors[valid_sector_].n_data)
        status = set_allocation_state(&sectors[valid_sector_], sectors[valid_sector_].index, 1, INVALID);
    else
        status = erase(&sectors[valid_sector_]);
    if (status)
        return status;

    valid_sector_ = 1 - valid_sector_;
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}


void NVM_demo(void) {
    const size_t test_length = 14;
    uint64_t data[test_length];
    size_t length = test_length;

    osDelay(100);
    printf("=== NVM TEST ===\r\n"); osDelay(5);
    //NVM_erase();
    if (NVM_init() != 0) {
        printf("init error\r\n"); osDelay(5);
    } else if (NVM_read_tail(data, &length) != 0) {
        printf("read error\r\n"); osDelay(5);
    } else {
        if (length < test_length) {
            printf("uninitialized (found %u fields) => init\r\n", length); osDelay(5);
            for (size_t i = 0; i < test_length; i++)
                data[i] = i * 3;
        } else {
            printf("have data: \r\n"); osDelay(5);
            for (size_t i = 0; i < test_length; i++)
                { printf(" %08x \r\n", (unsigned int)data[i]); osDelay(5); }
            printf("\r\n"); osDelay(5);

            printf(" => increment by 1\r\n"); osDelay(5);
            for (size_t i = 0; i < test_length; i++)
                data[i] += 1;
        }
            
        if (NVM_append(data, test_length) != 0)
            printf("write error\r\n");
        printf("done\r\n");
    }
}