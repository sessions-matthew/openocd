/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2026 by Texas Instruments / OpenOCD contributors        *
 *   Target driver for TI PRU-ICSS (Programmable Real-Time Unit)           *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_TI_PRU_H
#define OPENOCD_TARGET_TI_PRU_H

#include "target.h"
#include "target_type.h"
#include "arm_adi_v5.h"
#include "register.h"

#define TI_PRU_COMMON_MAGIC         0x50525530  /* 'PRU0' */

/* Register offsets within PRU Control block (TRM 4.5.1) */
#define PRU_REG_CTRL                0x0000
#define PRU_REG_STS                 0x0004
#define PRU_REG_WAKEUP_EN           0x0008
#define PRU_REG_CYCLE               0x000C
#define PRU_REG_STALL               0x0010
#define PRU_REG_CONT_TBL_PRU0       0x0020
#define PRU_REG_CONT_TBL_PRU1       0x0024
#define PRU_REG_CONT_TBL_PROG_PTR0  0x0028
#define PRU_REG_CONT_TBL_PROG_PTR1  0x002C

/* Register offsets within PRU Debug block (TRM 4.5.2) */
#define PRU_DEBUG_GPREG_BASE        0x0400  /* GPREG0..31 at 0x400 - 0x47C */

/* PRU_CTRL Register bitfields */
#define PRU_CTRL_SOFT_RST_N         (1 << 0)
#define PRU_CTRL_ENABLE             (1 << 1)
#define PRU_CTRL_SLEEPING           (1 << 2)
#define PRU_CTRL_CTR_EN             (1 << 3)
#define PRU_CTRL_SINGLE_STEP        (1 << 8)
#define PRU_CTRL_RUNSTATE           (1 << 15)
#define PRU_CTRL_PCTR_RST_VAL_MASK  (0xFFFF0000)
#define PRU_CTRL_PCTR_RST_VAL_SHIFT (16)

/* Default physical memory addresses on AM335x */
#define AM335X_PRU0_CTRL_BASE       0x4A322000
#define AM335X_PRU0_DEBUG_BASE      0x4A322400
#define AM335X_PRU0_IRAM_BASE       0x4A334000
#define AM335X_PRU0_DRAM_BASE       0x4A300000

#define AM335X_PRU1_CTRL_BASE       0x4A324000
#define AM335X_PRU1_DEBUG_BASE      0x4A324400
#define AM335X_PRU1_IRAM_BASE       0x4A338000
#define AM335X_PRU1_DRAM_BASE       0x4A302000

#define AM335X_PRU_SHARED_RAM_BASE  0x4A310000
#define AM335X_PRU_IRAM_SIZE        0x2000  /* 8 KB (2048 32-bit instructions) */
#define AM335X_PRU_DRAM_SIZE        0x2000  /* 8 KB Data RAM */

/* PRU Opcode for Software Breakpoint (HALT instruction) */
#define PRU_OPCODE_HALT             0x2A000000

/* Register indexes for GDB */
enum ti_pru_reg_index {
	TI_PRU_R0 = 0,
	TI_PRU_R1,
	TI_PRU_R2,
	TI_PRU_R3,
	TI_PRU_R4,
	TI_PRU_R5,
	TI_PRU_R6,
	TI_PRU_R7,
	TI_PRU_R8,
	TI_PRU_R9,
	TI_PRU_R10,
	TI_PRU_R11,
	TI_PRU_R12,
	TI_PRU_R13,
	TI_PRU_R14,
	TI_PRU_R15,
	TI_PRU_R16,
	TI_PRU_R17,
	TI_PRU_R18,
	TI_PRU_R19,
	TI_PRU_R20,
	TI_PRU_R21,
	TI_PRU_R22,
	TI_PRU_R23,
	TI_PRU_R24,
	TI_PRU_R25,
	TI_PRU_R26,
	TI_PRU_R27,
	TI_PRU_R28,
	TI_PRU_R29,
	TI_PRU_R30,
	TI_PRU_R31,
	TI_PRU_PC,
	TI_PRU_CYCLES,
	TI_PRU_STATUS,
	TI_PRU_NUM_REGS
};

#define TI_PRU_NUM_GP_REGS          32
#define TI_PRU_NUM_GDB_REGS         33  /* R0-R31 + PC */

struct ti_pru_reg {
	uint32_t num;
	struct target *target;
	uint32_t value;
};

struct ti_pru_common {
	int common_magic;
	struct adiv5_dap *dap;
	struct adiv5_ap *ap;
	uint64_t ap_num;

	target_addr_t base_addr;   /* PRU Control register base (e.g. 0x4A322000) */
	target_addr_t iram_addr;   /* PRU IRAM base (e.g. 0x4A334000) */
	target_addr_t dram_addr;   /* PRU Data RAM base (e.g. 0x4A300000) */
	uint32_t iram_size;        /* IRAM size in bytes */

	struct reg_cache *core_cache;
	struct ti_pru_reg core_regs[TI_PRU_NUM_REGS];

	bool single_step_active;
};

static inline struct ti_pru_common *target_to_pru(struct target *target)
{
	return (struct ti_pru_common *)target->arch_info;
}

extern struct target_type ti_pru_target;

#endif /* OPENOCD_TARGET_TI_PRU_H */
