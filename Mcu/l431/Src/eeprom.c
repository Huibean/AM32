/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"
#include <string.h>
#include <targets.h>

#define page_size 0x800                   // 2 kb for l431
uint32_t FLASH_FKEY1 =0x45670123;
uint32_t FLASH_FKEY2 =0xCDEF89AB;

/*
  memcpy version that runs from RAM
 */
__attribute__((section(".ramfunc")))
void memcpy_ram(void *dest, const void *src, uint32_t length)
{
    uint8_t *d = (uint8_t *)dest;
    const uint8_t *s = (const uint8_t *)src;
    while (length--) {
	*d++ = *s++;
    }
}

/*
  write to flash, done as a RAM function to allow for DroneCAN fw update
 */
__attribute__((section(".ramfunc")))
void save_flash_nolib(uint8_t *data, int length, uint32_t add){
    if ((add & 0x7) != 0 || (length & 0x7)) {
        // address and length must be on 8 byte boundary
	return;
    }
    // we need to flash on 32 bit boundaries
    uint32_t data_length = length / 4;
    volatile FLASH_TypeDef *flash = FLASH;

    // clear errors
    flash->SR |= FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
        FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR |
        FLASH_SR_RDERR | FLASH_SR_OPTVERR;

    // unlock flash
    while ((flash->SR & FLASH_SR_BSY) != 0) ;

    if ((flash->CR & FLASH_CR_LOCK) != 0) {
        flash->KEYR = FLASH_FKEY1;
        flash->KEYR = FLASH_FKEY2;
    }

    // erase page if address is divisable by page size
    if ((add % page_size) == 0){
        flash->CR = FLASH_CR_PER;
        flash->CR |= (add/page_size) << 3;
        flash->CR |= FLASH_CR_STRT;
        while ((flash->SR & FLASH_SR_BSY) != 0) ;
    }

    uint32_t index = 0;
    volatile uint32_t *fdata = (volatile uint32_t *)add;

    while (index < data_length) {
        // flash two words at a time
        uint32_t words[2];
	memcpy_ram((void*)&words[0], &data[index*4], sizeof(words));

        flash->CR = FLASH_CR_PG;

        fdata[index] = words[0];
        fdata[index+1] = words[1];

        while ((flash->SR & FLASH_SR_BSY) != 0) ;

        flash->SR |= FLASH_SR_EOP;
        flash->CR = 0;
        index += 2;
    }

    // lock flash again
    SET_BIT(flash->CR, FLASH_CR_LOCK);
}

void read_flash_bin(uint8_t*  data , uint32_t add, int out_buff_len) {
    memcpy_ram(data, (const void*)add, out_buff_len);
}

#ifdef DRONECAN_SUPPORT
/*
  pointer to the start of application flash
 */
extern uint32_t g_pfnVectors[2];

/*
  function to upgrade flash, running from RAM so flash instructions not needed
  on completion this resets the MCU, booting the new firmware
 */
__attribute__((section(".ramfunc")))
void flash_upgrade(const uint8_t *data, uint32_t length)
{
    /*
      disable interrupts while flashing. The interrupt handlers are in
      flash, so one being called after an erase can trigger a
      hardfault
     */
    __disable_irq();

    uint32_t dest_add = (uint32_t)&g_pfnVectors[0];

    // round up to multiple of 8
    length = (length+7U) & ~7U;
    while (length > 0) {
	// pat the watchdog in case flashing takes more than the
	// watchdog timeout
	LL_IWDG_ReloadCounter(IWDG);

	const uint32_t chunk = length>256?256:length;
	save_flash_nolib((uint8_t *)data, chunk, dest_add);
	length -= chunk;
	dest_add += chunk;
	data += chunk;
    }

    __ISB();
    // reboot to start new fw
    NVIC_SystemReset();
}
#endif // DRONECAN_SUPPORT
