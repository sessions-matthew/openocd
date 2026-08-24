// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2026 by Texas Instruments / OpenOCD contributors        *
 *   Target driver for TI PRU-ICSS (Programmable Real-Time Unit)           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ti_pru.h"
#include "target.h"
#include "target_type.h"
#include "arm_adi_v5.h"
#include "register.h"
#include "breakpoints.h"
#include "image.h"
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <helper/nvp.h>
#include <helper/jim-nvp.h>

static const char *const ti_pru_reg_names[TI_PRU_NUM_REGS] = {
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
	"r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
	"r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
	"r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
	"pc", "cycles", "status"
};

/* Forward declarations */
static int ti_pru_poll(struct target *target);
static int ti_pru_halt(struct target *target);
static int ti_pru_resume(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints, bool debug_execution);
static int ti_pru_step(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints);

/* Read/Write 32-bit register on MEM-AP */
static int ti_pru_read_u32(struct ti_pru_common *pru, target_addr_t addr, uint32_t *val)
{
	if (!pru || !pru->ap)
		return ERROR_FAIL;
	return mem_ap_read_u32(pru->ap, addr, val);
}

static int ti_pru_write_u32(struct ti_pru_common *pru, target_addr_t addr, uint32_t val)
{
	if (!pru || !pru->ap)
		return ERROR_FAIL;
	return mem_ap_write_u32(pru->ap, addr, val);
}

/* Register get / set handlers */
static int ti_pru_get_core_reg(struct reg *reg)
{
	struct ti_pru_reg *pru_reg = reg->arch_info;
	struct target *target = pru_reg->target;
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t val = 0;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "Target not halted, register read may be stale");
	}

	if (pru_reg->num < TI_PRU_NUM_GP_REGS) {
		/* General purpose registers R0-R31 use cached values on AM335x */
		val = pru_reg->value;
		retval = ERROR_OK;
	} else if (pru_reg->num == TI_PRU_PC) {
		/* Program Counter: STS register bits 15:0 = PCTR (word address) */
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_STS, &val);
		if (retval == ERROR_OK)
			val = (val & 0xFFFF) * 4;  /* Convert instruction word offset to byte address */
	} else if (pru_reg->num == TI_PRU_CYCLES) {
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CYCLE, &val);
	} else if (pru_reg->num == TI_PRU_STATUS) {
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &val);
	} else {
		return ERROR_FAIL;
	}

	if (retval != ERROR_OK)
		return retval;

	pru_reg->value = val;
	buf_set_u32(reg->value, 0, 32, val);
	reg->valid = true;
	reg->dirty = false;

	return ERROR_OK;
}

static int ti_pru_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct ti_pru_reg *pru_reg = reg->arch_info;
	struct target *target = pru_reg->target;
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t val = buf_get_u32(buf, 0, 32);
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Target must be halted to write registers");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (pru_reg->num < TI_PRU_NUM_GP_REGS) {
		target_addr_t reg_addr = pru->base_addr + PRU_DEBUG_GPREG_BASE + (pru_reg->num * 4);
		retval = ti_pru_write_u32(pru, reg_addr, val);
	} else if (pru_reg->num == TI_PRU_PC) {
		/* Set PCTR reset value in CTRL register (bits 31:16) */
		uint32_t ctrl_val = 0;
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl_val);
		if (retval == ERROR_OK) {
			uint32_t word_pc = (val / 4) & 0xFFFF;
			ctrl_val = (ctrl_val & ~PRU_CTRL_PCTR_RST_VAL_MASK) |
				(word_pc << PRU_CTRL_PCTR_RST_VAL_SHIFT);
			retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl_val);
		}
	} else if (pru_reg->num == TI_PRU_CYCLES) {
		retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CYCLE, val);
	} else if (pru_reg->num == TI_PRU_STATUS) {
		retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, val);
	} else {
		return ERROR_FAIL;
	}

	if (retval != ERROR_OK)
		return retval;

	pru_reg->value = val;
	buf_set_u32(reg->value, 0, 32, val);
	reg->valid = true;
	reg->dirty = false;

	return ERROR_OK;
}

static const struct reg_arch_type ti_pru_reg_arch_type = {
	.get = ti_pru_get_core_reg,
	.set = ti_pru_set_core_reg,
};

static struct reg_data_type ti_pru_type_uint32 = { .type = REG_TYPE_UINT32, .id = "uint32" };
static struct reg_data_type ti_pru_type_code_ptr = { .type = REG_TYPE_CODE_PTR, .id = "code_ptr" };

static struct reg_cache *ti_pru_build_reg_cache(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(TI_PRU_NUM_REGS, sizeof(struct reg));
	struct reg_feature *feature = malloc(sizeof(struct reg_feature));

	if (!cache || !reg_list || !feature) {
		free(cache);
		free(reg_list);
		free(feature);
		return NULL;
	}

	feature->name = "org.gnu.gdb.pru.core";
	cache->name = "TI PRU Registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = TI_PRU_NUM_REGS;

	for (size_t i = 0; i < TI_PRU_NUM_REGS; i++) {
		pru->core_regs[i].num = i;
		pru->core_regs[i].target = target;
		pru->core_regs[i].value = 0;

		reg_list[i].name = ti_pru_reg_names[i];
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].type = &ti_pru_reg_arch_type;
		reg_list[i].arch_info = &pru->core_regs[i];
		reg_list[i].feature = feature;
		reg_list[i].caller_save = false;

		if (i == TI_PRU_PC)
			reg_list[i].reg_data_type = &ti_pru_type_code_ptr;
		else
			reg_list[i].reg_data_type = &ti_pru_type_uint32;
	}

	pru->core_cache = cache;
	*register_get_last_cache_p(&target->reg_cache) = cache;
	return cache;
}

struct ti_pru_private_config {
	struct adiv5_private_config adiv5_config;
	target_addr_t base_addr;
	target_addr_t iram_addr;
	target_addr_t dram_addr;
};

enum ti_pru_cfg_param {
	CFG_BASE_ADDR,
	CFG_IRAM_ADDR,
	CFG_DRAM_ADDR,
};

static const struct nvp nvp_pru_config_opts[] = {
	{ .name = "-base-addr", .value = CFG_BASE_ADDR },
	{ .name = "-iram-addr", .value = CFG_IRAM_ADDR },
	{ .name = "-dram-addr", .value = CFG_DRAM_ADDR },
	{ .name = NULL, .value = -1 }
};

static int ti_pru_jim_configure(struct target *target, struct jim_getopt_info *goi)
{
	struct ti_pru_private_config *pc;
	const struct nvp *n;
	int e;

	pc = (struct ti_pru_private_config *)target->private_config;
	if (!pc) {
		pc = calloc(1, sizeof(struct ti_pru_private_config));
		if (!pc)
			return JIM_ERR;
		pc->adiv5_config.ap_num = DP_APSEL_INVALID;
		pc->base_addr = AM335X_PRU0_CTRL_BASE;
		pc->iram_addr = AM335X_PRU0_IRAM_BASE;
		pc->dram_addr = AM335X_PRU0_DRAM_BASE;
		target->private_config = pc;
	}

	e = adiv5_jim_configure_ext(target, goi, &pc->adiv5_config, ADI_CONFIGURE_DAP_COMPULSORY);
	if (e != JIM_CONTINUE)
		return e;

	if (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		e = jim_nvp_name2value_obj(goi->interp, nvp_pru_config_opts, goi->argv[0], &n);
		if (e != JIM_OK)
			return JIM_CONTINUE;

		e = jim_getopt_obj(goi, NULL);
		if (e != JIM_OK)
			return e;

		switch (n->value) {
		case CFG_BASE_ADDR: {
			jim_wide w;
			e = jim_getopt_wide(goi, &w);
			if (e != JIM_OK)
				return e;
			pc->base_addr = (target_addr_t)w;
			break;
		}
		case CFG_IRAM_ADDR: {
			jim_wide w;
			e = jim_getopt_wide(goi, &w);
			if (e != JIM_OK)
				return e;
			pc->iram_addr = (target_addr_t)w;
			break;
		}
		case CFG_DRAM_ADDR: {
			jim_wide w;
			e = jim_getopt_wide(goi, &w);
			if (e != JIM_OK)
				return e;
			pc->dram_addr = (target_addr_t)w;
			break;
		}
		}
		return JIM_OK;
	}

	return JIM_CONTINUE;
}

/* Target interface functions */
static int ti_pru_target_create(struct target *target)
{
	struct ti_pru_common *pru;
	struct ti_pru_private_config *pc;

	pc = (struct ti_pru_private_config *)target->private_config;
	if (!pc) {
		LOG_TARGET_ERROR(target, "No private configuration (DAP) provided");
		return ERROR_FAIL;
	}

	if (pc->adiv5_config.ap_num == DP_APSEL_INVALID) {
		LOG_TARGET_ERROR(target, "AP number not specified for PRU target");
		return ERROR_FAIL;
	}

	if (!target->gdb_port_override)
		target->gdb_port_override = strdup("disabled");

	pru = calloc(1, sizeof(struct ti_pru_common));
	if (!pru) {
		LOG_TARGET_ERROR(target, "Out of memory");
		return ERROR_FAIL;
	}

	pru->common_magic = TI_PRU_COMMON_MAGIC;
	pru->dap = pc->adiv5_config.dap;
	pru->ap_num = pc->adiv5_config.ap_num;

	/* Configure addresses */
	pru->base_addr = pc->base_addr ? pc->base_addr : AM335X_PRU0_CTRL_BASE;
	pru->iram_addr = pc->iram_addr ? pc->iram_addr : AM335X_PRU0_IRAM_BASE;
	pru->dram_addr = pc->dram_addr ? pc->dram_addr : AM335X_PRU0_DRAM_BASE;
	pru->iram_size = AM335X_PRU_IRAM_SIZE;

	target->arch_info = pru;
	return ERROR_OK;
}

static int ti_pru_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_TARGET_DEBUG(target, "%s", __func__);
	ti_pru_build_reg_cache(target);
	target->state = TARGET_UNKNOWN;
	target->debug_reason = DBG_REASON_UNDEFINED;
	return ERROR_OK;
}

static void ti_pru_deinit_target(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);

	LOG_TARGET_DEBUG(target, "%s", __func__);

	if (pru) {
		if (pru->ap)
			dap_put_ap(pru->ap);
		free(pru);
		target->arch_info = NULL;
	}

	free(target->private_config);
}

static void ti_pru_ensure_clocks(struct ti_pru_common *pru)
{
	if (!pru || !pru->ap)
		return;

	/* 1. De-assert PRU-ICSS local reset in PRM_PER (0x44E00C00) */
	uint32_t rstctrl = 0;
	if (mem_ap_read_u32(pru->ap, 0x44E00C00, &rstctrl) == ERROR_OK) {
		rstctrl &= ~(1 << 1); /* Clear PRU_ICSS_LRST */
		mem_ap_write_u32(pru->ap, 0x44E00C00, rstctrl);
	}

	/* 2. Select L3F 200 MHz clock in CM_DPLL (0x44E00530) */
	mem_ap_write_u32(pru->ap, 0x44E00530, 0x00);

	/* 3. Wake up PRU-ICSS clock domain (0x44E00140 = SW_WKUP) */
	mem_ap_write_u32(pru->ap, 0x44E00140, 0x02);

	/* 4. Enable PRU-ICSS module clock (0x44E000E8 = MODULEMODE_ENABLE) */
	mem_ap_write_u32(pru->ap, 0x44E000E8, 0x02);

	/* 5. Enable GPIO1 module clock (0x44E000AC = MODULEMODE_ENABLE) */
	mem_ap_write_u32(pru->ap, 0x44E000AC, 0x02);

	/* 6. Wait for PRU-ICSS module to transition out of disabled state */
	for (int i = 0; i < 100; i++) {
		uint32_t clkctrl = 0;
		if (mem_ap_read_u32(pru->ap, 0x44E000E8, &clkctrl) == ERROR_OK) {
			if ((clkctrl & 0x00030000) != 0x00030000 && (clkctrl & 0x03) == 0x02)
				break;
		}
		alive_sleep(1);
	}

	/* 7. Configure PRU-ICSS CFG SYSCFG (0x4A326004): NO-IDLE, NO-STANDBY, STANDBY_INIT=0 */
	mem_ap_write_u32(pru->ap, 0x4A326004, 0x05);
}

static int ti_pru_examine(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);

	if (!target_was_examined(target)) {
		if (!pru->ap) {
			pru->ap = dap_get_ap(pru->dap, pru->ap_num);
			if (!pru->ap) {
				LOG_TARGET_ERROR(target, "Cannot get AP %" PRIu64, pru->ap_num);
				return ERROR_FAIL;
			}
		}

		/* Ensure PRU-ICSS clocks and power are active before accessing registers */
		ti_pru_ensure_clocks(pru);

		uint32_t ctrl = 0;
		int retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
		if (retval != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read PRU CTRL register at 0x%08" TARGET_PRIxADDR, pru->base_addr);
			return retval;
		}

		if (ctrl & PRU_CTRL_RUNSTATE) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		} else {
			target->state = TARGET_HALTED;
			target->debug_reason = DBG_REASON_DBGRQ;
		}

		target_set_examined(target);
		LOG_TARGET_INFO(target, "PRU-ICSS examined successfully (CTRL=0x%08" PRIx32 ")", ctrl);
	}

	return ERROR_OK;
}

static int ti_pru_poll(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t ctrl = 0;
	int retval;

	if (!pru || !pru->ap)
		return ERROR_FAIL;

	retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	bool is_running = (ctrl & PRU_CTRL_RUNSTATE) != 0;

	if (is_running) {
		if (target->state != TARGET_RUNNING) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
			if (!target->gdb_port_override || strcmp(target->gdb_port_override, "disabled") != 0)
				target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		}
	} else {
		if (target->state != TARGET_HALTED) {
			target->state = TARGET_HALTED;
			target->debug_reason = DBG_REASON_DBGRQ;

			/* Refresh PC */
			uint32_t sts = 0;
			ti_pru_read_u32(pru, pru->base_addr + PRU_REG_STS, &sts);
			uint32_t pc = (sts & 0xFFFF) * 4;

			LOG_TARGET_DEBUG(target, "PRU halted at PC=0x%08" PRIx32 " (CTRL=0x%08" PRIx32 ")", pc, ctrl);
			if (!target->gdb_port_override || strcmp(target->gdb_port_override, "disabled") != 0)
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}

	return ERROR_OK;
}

static int ti_pru_arch_state(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t ctrl = 0, sts = 0, cycles = 0;

	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_STS, &sts);
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CYCLE, &cycles);

	uint32_t pc = (sts & 0xFFFF) * 4;
	LOG_TARGET_DEBUG(target, "state=%s, PC=0x%08" PRIx32 " (ins=0x%04" PRIx32 "), Cycles=%" PRIu32 ", CTRL=0x%08" PRIx32,
		target_state_name(target),
		pc, (sts & 0xFFFF), cycles, ctrl);

	return ERROR_OK;
}

static int ti_pru_halt(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t ctrl = 0;
	int retval;

	LOG_TARGET_DEBUG(target, "%s", __func__);

	if (target->state == TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "Target was already halted");
		return ERROR_OK;
	}

	retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Clear ENABLE bit (bit 1) */
	ctrl &= ~PRU_CTRL_ENABLE;
	retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for RUNSTATE to clear */
	int timeout = 1000;
	while (timeout > 0) {
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
		if (retval == ERROR_OK && !(ctrl & PRU_CTRL_RUNSTATE))
			break;
		alive_sleep(1);
		timeout--;
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	if (!target->gdb_port_override || strcmp(target->gdb_port_override, "disabled") != 0)
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static int ti_pru_resume(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints, bool debug_execution)
{
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t ctrl = 0;
	int retval;

	LOG_TARGET_DEBUG(target, "%s: current=%d, address=0x%08" TARGET_PRIxADDR, __func__, current, address);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current) {
		/* Set starting PC address */
		uint32_t word_pc = (address / 4) & 0xFFFF;
		ctrl = (word_pc << PRU_CTRL_PCTR_RST_VAL_SHIFT) | PRU_CTRL_SOFT_RST_N;
		ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl);
	}

	/* Start core: ENABLE | CTR_EN | SOFT_RST_N (0x0000000B) */
	ctrl = PRU_CTRL_CTR_EN | PRU_CTRL_ENABLE | PRU_CTRL_SOFT_RST_N;
	retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl);
	if (retval != ERROR_OK)
		return retval;

	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!target->gdb_port_override || strcmp(target->gdb_port_override, "disabled") != 0)
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	return ERROR_OK;
}

static int ti_pru_step(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints)
{
	struct ti_pru_common *pru = target_to_pru(target);
	uint32_t ctrl = 0;
	int retval;

	LOG_TARGET_DEBUG(target, "%s", __func__);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current) {
		uint32_t word_pc = (address / 4) & 0xFFFF;
		ctrl = (word_pc << PRU_CTRL_PCTR_RST_VAL_SHIFT) | PRU_CTRL_SOFT_RST_N;
		ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl);
	}

	/* Single step: SINGLE_STEP | CTR_EN | ENABLE | SOFT_RST_N (0x0000010B) */
	ctrl = PRU_CTRL_SINGLE_STEP | PRU_CTRL_CTR_EN | PRU_CTRL_ENABLE | PRU_CTRL_SOFT_RST_N;
	retval = ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Hardware auto-clears ENABLE after 1 instruction */
	int timeout = 500;
	while (timeout > 0) {
		retval = ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
		if (retval == ERROR_OK && !(ctrl & PRU_CTRL_RUNSTATE))
			break;
		alive_sleep(1);
		timeout--;
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_SINGLESTEP;

	/* Update PC and notify callbacks */
	uint32_t sts = 0;
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_STS, &sts);
	uint32_t pc = (sts & 0xFFFF) * 4;
	LOG_TARGET_DEBUG(target, "Single-step completed at PC=0x%08" PRIx32, pc);

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

static int ti_pru_assert_reset(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);

	LOG_TARGET_DEBUG(target, "%s", __func__);

	/* Assert reset: SOFT_RST_N = 0, ENABLE = 0 */
	ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, 0);

	target->state = TARGET_RESET;
	target->debug_reason = DBG_REASON_UNDEFINED;
	return ERROR_OK;
}

static int ti_pru_deassert_reset(struct target *target)
{
	struct ti_pru_common *pru = target_to_pru(target);

	LOG_TARGET_DEBUG(target, "%s: reset_halt=%d", __func__, target->reset_halt);

	if (target->reset_halt) {
		/* De-assert reset but keep core disabled: SOFT_RST_N = 1, ENABLE = 0 */
		ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, PRU_CTRL_SOFT_RST_N);
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_DBGRQ;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	} else {
		/* De-assert reset and run: SOFT_RST_N = 1, ENABLE = 1, CTR_EN = 1 */
		ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL,
			PRU_CTRL_CTR_EN | PRU_CTRL_ENABLE | PRU_CTRL_SOFT_RST_N);
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	}

	return ERROR_OK;
}

/* Memory Read/Write */
static int ti_pru_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct ti_pru_common *pru = target_to_pru(target);
	target_addr_t phys_addr = address;

	/* If reading from low addresses [0x0 .. 0x1FFF], route to PRU IRAM */
	if (address < pru->iram_size) {
		phys_addr = pru->iram_addr + address;
	}

	return mem_ap_read_buf(pru->ap, buffer, size, count, phys_addr);
}

static int ti_pru_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct ti_pru_common *pru = target_to_pru(target);
	target_addr_t phys_addr = address;

	/* If writing to low addresses [0x0 .. 0x1FFF], route to PRU IRAM */
	if (address < pru->iram_size) {
		phys_addr = pru->iram_addr + address;
	}

	return mem_ap_write_buf(pru->ap, buffer, size, count, phys_addr);
}

static int ti_pru_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct ti_pru_common *pru = target_to_pru(target);

	if (!pru->core_cache)
		return ERROR_FAIL;

	*reg_list_size = (reg_class == REG_CLASS_ALL) ? TI_PRU_NUM_REGS : TI_PRU_NUM_GDB_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));
	if (!*reg_list)
		return ERROR_FAIL;

	for (int i = 0; i < *reg_list_size; i++) {
		(*reg_list)[i] = &pru->core_cache->reg_list[i];
	}

	return ERROR_OK;
}

static const char *ti_pru_get_gdb_arch(const struct target *target)
{
	return "pru";
}

/* Custom TCL Commands */
COMMAND_HANDLER(ti_pru_handle_dump_regs_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct ti_pru_common *pru = target_to_pru(target);

	if (pru->common_magic != TI_PRU_COMMON_MAGIC) {
		command_print(CMD, "Current target is not a TI PRU core");
		return ERROR_FAIL;
	}

	uint32_t ctrl = 0, sts = 0, cycles = 0;
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CTRL, &ctrl);
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_STS, &sts);
	ti_pru_read_u32(pru, pru->base_addr + PRU_REG_CYCLE, &cycles);

	uint32_t dram0_val = 0;
	ti_pru_read_u32(pru, pru->dram_addr, &dram0_val);

	uint32_t pc = (sts & 0xFFFF) * 4;
	command_print(CMD, "=== %s State (Base: 0x%08" TARGET_PRIxADDR ") ===", target_name(target), pru->base_addr);
	command_print(CMD, "State: %s | PC: 0x%04" PRIx32 " (byte 0x%08" PRIx32 ") | Cycles: %" PRIu32 " | CTRL: 0x%08" PRIx32,
		(ctrl & PRU_CTRL_RUNSTATE) ? "RUNNING" : "HALTED",
		(sts & 0xFFFF), pc, cycles, ctrl);
	command_print(CMD, "Data RAM 0 Heartbeat (0x%08" TARGET_PRIxADDR "): 0x%08" PRIx32 " (%" PRIu32 " iterations)",
		pru->dram_addr, dram0_val, dram0_val);

	return ERROR_OK;
}

COMMAND_HANDLER(ti_pru_handle_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	struct ti_pru_common *pru = target_to_pru(target);
	struct image image;
	int retval;

	if (pru->common_magic != TI_PRU_COMMON_MAGIC) {
		command_print(CMD, "Current target is not a TI PRU core");
		return ERROR_FAIL;
	}

	image.base_address = 0;
	image.base_address_set = false;

	retval = image_open(&image, CMD_ARGV[0], (CMD_ARGC >= 2) ? CMD_ARGV[1] : NULL);
	if (retval != ERROR_OK)
		return retval;

	/* Ensure PRU clocks and power domain are active */
	ti_pru_ensure_clocks(pru);

	/* Halt PRU before loading firmware */
	ti_pru_halt(target);

	uint32_t total_written = 0;
	for (unsigned int i = 0; i < image.num_sections; i++) {
		uint8_t *buffer = malloc(image.sections[i].size);
		if (!buffer) {
			image_close(&image);
			return ERROR_FAIL;
		}

		size_t size_read;
		retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &size_read);
		if (retval == ERROR_OK) {
			target_addr_t dest = pru->iram_addr + image.sections[i].base_address;
			retval = mem_ap_write_buf(pru->ap, buffer, 4, (uint32_t)(size_read / 4), dest);
			if (retval == ERROR_OK)
				total_written += size_read;
		}
		free(buffer);
		if (retval != ERROR_OK)
			break;
	}

	image_close(&image);

	if (retval == ERROR_OK) {
		command_print(CMD, "Loaded %" PRIu32 " bytes of PRU firmware into IRAM (0x%08" TARGET_PRIxADDR ")",
			total_written, pru->iram_addr);
		/* Reset Program Counter to 0 */
		ti_pru_write_u32(pru, pru->base_addr + PRU_REG_CTRL, PRU_CTRL_SOFT_RST_N);
	}

	return retval;
}

static int ti_pru_handle_resume_command(struct command_invocation *cmd)
{
	struct target *target = get_current_target(cmd->ctx);
	if (strcmp(target_type_name(target), "ti_pru") != 0) {
		command_print(cmd, "Error: Target is not a ti_pru instance");
		return ERROR_FAIL;
	}
	return ti_pru_resume(target, 1, 0, 0, 0);
}

static int ti_pru_handle_halt_command(struct command_invocation *cmd)
{
	struct target *target = get_current_target(cmd->ctx);
	if (strcmp(target_type_name(target), "ti_pru") != 0) {
		command_print(cmd, "Error: Target is not a ti_pru instance");
		return ERROR_FAIL;
	}
	return ti_pru_halt(target);
}

static const struct command_registration ti_pru_exec_command_handlers[] = {
	{
		.name = "dump_regs",
		.handler = ti_pru_handle_dump_regs_command,
		.mode = COMMAND_EXEC,
		.help = "Dump all 32 PRU registers, PC, Cycle counter, and Status",
		.usage = "",
	},
	{
		.name = "load",
		.handler = ti_pru_handle_load_command,
		.mode = COMMAND_EXEC,
		.help = "Load binary firmware into PRU Instruction RAM (IRAM)",
		.usage = "<filename> [type]",
	},
	{
		.name = "resume",
		.handler = ti_pru_handle_resume_command,
		.mode = COMMAND_EXEC,
		.help = "Resume PRU execution directly via MEM-AP",
		.usage = "",
	},
	{
		.name = "halt",
		.handler = ti_pru_handle_halt_command,
		.mode = COMMAND_EXEC,
		.help = "Halt PRU execution directly via MEM-AP",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ti_pru_command_handlers[] = {
	{
		.name = "ti_pru",
		.mode = COMMAND_ANY,
		.help = "TI PRU target commands",
		.usage = "",
		.chain = ti_pru_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type ti_pru_target = {
	.name = "ti_pru",

	.target_create = ti_pru_target_create,
	.target_jim_configure = ti_pru_jim_configure,
	.init_target = ti_pru_init_target,
	.deinit_target = ti_pru_deinit_target,
	.examine = ti_pru_examine,

	.poll = ti_pru_poll,
	.arch_state = ti_pru_arch_state,

	.halt = ti_pru_halt,
	.resume = ti_pru_resume,
	.step = ti_pru_step,

	.assert_reset = ti_pru_assert_reset,
	.deassert_reset = ti_pru_deassert_reset,

	.get_gdb_arch = ti_pru_get_gdb_arch,
	.get_gdb_reg_list = ti_pru_get_gdb_reg_list,

	.read_memory = ti_pru_read_memory,
	.write_memory = ti_pru_write_memory,

	.commands = ti_pru_command_handlers,
};
