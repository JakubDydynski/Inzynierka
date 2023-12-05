/*
	STM32_Blocks library
	F401 Flash routines
	gbm 04'2022
*/

#include "board.h"
#include "flash.h"

// return Flash page/sector size
static _Bool flash_pgsize(void *addr)
{
	uint32_t a = (uint32_t) addr - 0x08000000;
	if (a < 0x10000)
		return 0x4000;
	if (a < 0x20000)
		return 0x10000;
	return 0x20000;
}

// return Flash sector number
static uint8_t flash_sn(void *addr)
{
	uint32_t a = (uint32_t) addr - 0x08000000;
	if (a < 0x10000)
		return a / 0x4000;
	if (a < 0x20000)
		return 4;
	return 5 + (a - 0x20000) / 0x20000;

}

_Bool flash_isblank(const void *addr, uint32_t size)
{
	const uint32_t *fp = addr;
	size = (size + 3) / 4;
	while (size--)
	{
		if (*fp++ != 0xffffffff)
			return 0;
	}
	return 1;
}

void flash_erase_page(void *addr)
{
	if (!flash_isblank(addr, flash_pgsize(addr)))
	{
		// erase page
		while(FLASH->SR & FLASH_SR_BSY);
		FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR;
		FLASH->CR = flash_sn(addr) << FLASH_CR_SNB_Pos | FLASH_CR_PSIZE_1 | FLASH_CR_SER;
		FLASH->CR |= FLASH_CR_STRT;
		while(FLASH->SR & FLASH_SR_BSY);
	}
}

void flash_write16(uint16_t *addr, const uint16_t data)
{
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR;
	FLASH->CR = FLASH_CR_PSIZE_0 | FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_PG;
	*(volatile uint16_t *)addr = data;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->CR &= ~FLASH_CR_PG;
	FLASH->SR = FLASH_SR_EOP;
}

void flash_write32(uint32_t *addr, const uint32_t data)
{
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR;
	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_PG;
	*(volatile uint32_t *)addr = data;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->CR &= ~FLASH_CR_PG;
	FLASH->SR = FLASH_SR_EOP;
}

void flash_write64(uint32_t *addr, const uint32_t *data)
{
	flash_write32((uint32_t *)addr, data[0]);
	flash_write32((uint32_t *)addr + 1, data[1]);
}
#if 0
void flash_write_page(void *dst, const void * src)
{
	// ENABLE programming
	while (FLASH->SR & FLASH_SR_BSY);
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR;
	FLASH->CR = FLASH_CR_PG;
	
	uint16_t* fp = dst;
	const uint16_t* sp = src;
	
	for (uint32_t i = 0; i < FLASH_PAGE_SIZE / 2; i++)
	{
		// program in halfword units
		if (sp[0] != 0xffff)
		{
			// program
			*fp++ = *sp++;
			while(FLASH->SR & FLASH_SR_BSY);
			FLASH->SR = FLASH_SR_EOP;
		}
		else
		{
			// skip
			fp++;
			sp++;
		}
			
	}
	FLASH->CR &= ~FLASH_CR_PG;
}
#endif
