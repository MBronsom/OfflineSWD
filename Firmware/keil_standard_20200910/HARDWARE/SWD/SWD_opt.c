/**
 * @file    SWD_opt.c
 * @brief   ͨ��SWDЭ���MCU��FLASH���
 */
#include "swd_host.h"
#include "SWD_opt.h"

extern const program_target_t flash_opt;

error_t target_opt_init(void)
{
	if (0 == swd_set_target_state_hw(RESET_PROGRAM))
	{
		return ERROR_RESET;
	}

	// ���ر���㷨��Ŀ��MCU��SRAM������ʼ��
	if (0 == swd_write_memory(flash_opt.algo_start, (uint8_t *)flash_opt.algo_blob, flash_opt.algo_size))
	{
		return ERROR_ALGO_DL;
	}

	if (0 == swd_flash_syscall_exec(&flash_opt.sys_call_s, flash_opt.init, 0, 0, 0, 0))
	{
		return ERROR_INIT;
	}

	return ERROR_SUCCESS;
}

error_t target_opt_uninit(void)
{
	if (0 == swd_flash_syscall_exec(&flash_opt.sys_call_s, flash_opt.uninit, 0, 0, 0, 0))
	{
		return ERROR_INIT;
	}
	return ERROR_SUCCESS;
}

error_t target_opt_program_page(uint32_t addr, const uint8_t *buf, uint32_t size)
{
	// Write page to buffer
	if (!swd_write_memory(flash_opt.program_buffer, (uint8_t *)buf, size))
	{
		return ERROR_ALGO_DATA_SEQ;
	}

	// Run flash programming
	if (!swd_flash_syscall_exec(&flash_opt.sys_call_s,
								flash_opt.program_page,
								addr,
								size,
								flash_opt.program_buffer,
								0))
	{
		return ERROR_WRITE;
	}
	return ERROR_SUCCESS;
}

error_t target_opt_erase_sector(uint32_t addr)
{
	if (0 == swd_flash_syscall_exec(&flash_opt.sys_call_s, flash_opt.erase_sector, addr, 0, 0, 0))
	{
		return ERROR_ERASE_SECTOR;
	}

	return ERROR_SUCCESS;
}

error_t target_opt_erase_chip(void)
{
	error_t status = ERROR_SUCCESS;

	if (0 == swd_flash_syscall_exec(&flash_opt.sys_call_s, flash_opt.erase_chip, 0, 0, 0, 0))
	{
		return ERROR_ERASE_ALL;
	}

	return status;
}
