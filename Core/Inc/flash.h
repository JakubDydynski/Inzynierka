/*
	STM32_Blocks library
	F10x, F0 series Flash routines
	gbm 11'2018..02'2019
*/

_Bool flash_isblank(const void *addr, uint32_t size);
void flash_erase_page(void *addr);
void flash_write_page(void *dst, const void * src);
void flash_write64(uint32_t *dst, const uint32_t *data);
void flash_write32(uint32_t *dst, const uint32_t data);
void flash_write16(uint16_t *dst, const uint16_t data);
