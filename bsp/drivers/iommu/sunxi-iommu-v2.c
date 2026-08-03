/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*******************************************************************************
 * Copyright (C) 2016-2018, Allwinner Technology CO., LTD.
 * Author: zhuxianbin <zhuxianbin@allwinnertech.com>
 *
 * This file is provided under a dual BSD/GPL license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 ******************************************************************************/
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/sizes.h>
#include <linux/device.h>
#include <asm/cacheflush.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/reset.h>
#include <linux/pm_domain.h>

#include "sunxi-iommu.h"
#include "sunxi-iommu-pgtable.h"

/*
 * by design iommu driver should be part of iommu
 * and get to it by ../../dma-iommu.h
 * sunxi bsp have seperate root, use different path
 * to reach dma-iommu.h
 */
#include "../../../drivers/iommu/dma-iommu.h"

#define _max(x, y) (((u64)(x) > (u64)(y)) ? (x) : (y))

/*
 * Register of IOMMU device
 */
#define IOMMU_VERSION_REG 0x0000
#define IOMMU_RESET_REG 0x0010
#define IOMMU_ENABLE_REG 0x0014
#define IOMMU_AUTO_GATING_REG 0x001c
#define IOMMU_AUTO_GATING_DEFAULT_VAL 0x7fff0003
#define IOMMU_TTB_REG 0x0020
#define IOMMU_TLB_ENABLE_REG 0x0028
#define IOMMU_TLB_PREFETCH_REG 0x002c
#define IOMMU_TLB_FLUSH_ENABLE_REG 0x0030
#define IOMMU_TLB_IVLD_MODE_SEL_REG 0x0040
#define IOMMU_TLB_IVLD_START_ADDR_REG 0x0044
#define IOMMU_TLB_IVLD_END_ADDR_REG 0x0048
#define IOMMU_TLB_IVLD_ADDR_REG 0x004c
#define IOMMU_TLB_IVLD_ADDR_MASK_REG 0x0050
#define IOMMU_TLB_IVLD_ENABLE_REG 0x0054
#define IOMMU_TLB_IVLD_ADDR_ALIGN_SHIFT (12)
#define IOMMU_TLB_IVLD_ADDR_OFFSET (0)
#define IOMMU_PC_IVLD_MODE_SEL_REG 0x0060
#define IOMMU_PC_IVLD_ADDR_REG 0x0064
#define IOMMU_PC_IVLD_START_ADDR_REG 0x0068
#define IOMMU_PC_IVLD_ENABLE_REG 0x006c
#define IOMMU_PC_IVLD_END_ADDR_REG 0x0070
#define IOMMU_PC_IVLD_ADDR_ALIGN_SHIFT (22)
#define IOMMU_PC_IVLD_ADDR_OFFSET (0)
#define IOMMU_INT_ENABLE_REG 0x080
#define IOMMU_INT_CLR_REG 0x0084
#define IOMMU_INT_STA_REG 0x0088

#define IOMMU_INT_ERR_LOW_ADDR7_REG 0x008C
#define IOMMU_INT_ERR_HIGH_ADDR7_REG 0x0090

#define IOMMU_INT_ERR_LOW_ADDR8_REG 0x0094
#define IOMMU_INT_ERR_HIGH_ADDR8_REG 0x0098

#define IOMMU_INT_ERR_DATA7_REG 0x009C
#define IOMMU_INT_ERR_DATA8_REG 0x00a0

#define IOMMU_L1PG_INT_REG 0x00b0
#define IOMMU_L2PG_INT_REG 0x00b4
#define IOMMU_LOW_VA_REG 0x00c0
#define IOMMU_HIGH_VA_REG 0x00c4
#define IOMMU_VA_DATA_REG 0x00c8
#define IOMMU_VA_CONFIG_REG 0x00cc
#define IOMMU_PMU_ENABLE_REG 0x00e0
#define IOMMU_PMU_CLR_REG 0x00e4
#define IOMMU_PVT_HANG_EN 0x00e8
#define IOMMU_PVT_HANG_PGTB 0x00ec
#define IOMMU_PMU_ACCESS_LOW7_REG 0x00f0
#define IOMMU_PMU_ACCESS_HIGH7_REG 0x00f4
#define IOMMU_PMU_HIT_LOW7_REG 0x00f8
#define IOMMU_PMU_HIT_HIGH7_REG 0x00fc
#define IOMMU_PMU_ACCESS_LOW8_REG 0x0100
#define IOMMU_PMU_ACCESS_HIGH8_REG 0x0104
#define IOMMU_PMU_HIT_LOW8_REG 0x0108
#define IOMMU_PMU_HIT_HIGH8_REG 0x010c

/* per-master micro TLB related */
#define IOMMU_MIC_BASE_OFFSET 0x400
#define IOMMU_MIC_MAX_MASTER 6
#define IOMMU_MIC_AUTO_GATING_REG(master_id) \
	(0x200 * master_id + 0x0000 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_BYP_REG(master_id) \
	(0x200 * master_id + 0x0004 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_WBUF_CTRL_REG(master_id) \
	(0x200 * master_id + 0x0008 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_OOO_REG(master_id) \
	(0x200 * master_id + 0x000c + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_4KB_BDY_PRT_REG(master_id) \
	(0x200 * master_id + 0x0010 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_TLB_ENABLE_REG(master_id) \
	(0x200 * master_id + 0x0014 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_TLB_FLUSH_ENABLE_REG(master_id) \
	(0x200 * master_id + 0x0018 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_DM_AUT_CTRL_REG(master_id) \
	(0x200 * master_id + 0x0020 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_DM_AUT_OVWT_REG(master_id) \
	(0x200 * master_id + 0x0024 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_ENABLE_REG(master_id) \
	(0x200 * master_id + 0x0030 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_CLR_REG(master_id) \
	(0x200 * master_id + 0x0034 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_STA_REG(master_id) \
	(0x200 * master_id + 0x0038 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_ERR_LOW_ADDR_REG(master_id) \
	(0x200 * master_id + 0x003c + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(master_id) \
	(0x200 * master_id + 0x0040 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_INT_ERR_DATA_REG(master_id) \
	(0x200 * master_id + 0x0044 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_ENABLE_REG(master_id) \
	(0x200 * master_id + 0x0050 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_CLR_REG(master_id) \
	(0x200 * master_id + 0x0054 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_ACCESS_LOW_REG(master_id) \
	(0x200 * master_id + 0x0060 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_ACCESS_HIGH_REG(master_id) \
	(0x200 * master_id + 0x0064 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_HIT_LOW_REG(master_id) \
	(0x200 * master_id + 0x0068 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_HIT_HIGH_REG(master_id) \
	(0x200 * master_id + 0x006c + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_TL_LOW_REG(master_id) \
	(0x200 * master_id + 0x0080 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_TL_HIGH_REG(master_id) \
	(0x200 * master_id + 0x0084 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PMU_ML_REG(master_id) \
	(0x200 * master_id + 0x0088 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PVT_HANG_EN_REG(master_id) \
	(0x200 * master_id + 0x0090 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PVT_HANG_ADDR_REG(master_id) \
	(0x200 * master_id + 0x0094 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_PVT_HANG_AUTH_REG(master_id) \
	(0x200 * master_id + 0x0098 + IOMMU_MIC_BASE_OFFSET)
#define IOMMU_MIC_GEN_WLAST_EN_REG(master_id) \
	(0x200 * master_id + 0x009c + IOMMU_MIC_BASE_OFFSET)

#define IOMMU_PER_SET_ADDR_SIZE 0x10000
#define IOMMU_HW_SET_COUNT 2
/*
 * IOMMU enable register field
 */
#define IOMMU_ENABLE 0x1

#define IOMMU_INT_L1PG_CLR_EN_MASK 0x1
#define IOMMU_INT_L2PG_CLR_EN_MASK 0x2
#define IOMMU_INT_L1PG_STA_MASK (1 << 16)
#define IOMMU_INT_L2PG_STA_MASK (1 << 17)

#define DEFAULT_BYPASS_VALUE 0x3ff
static const u32 master_id_bitmap[] = { 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40 };

#define sunxi_wait_when(COND, MS)                                             \
	({                                                                    \
		unsigned long timeout__ = jiffies + msecs_to_jiffies(MS) + 1; \
		int ret__ = 0;                                                \
		while ((COND)) {                                              \
			if (time_after(jiffies, timeout__)) {                 \
				ret__ = (!COND) ? 0 : -ETIMEDOUT;             \
				break;                                        \
			}                                                     \
			udelay(1);                                            \
		}                                                             \
		ret__;                                                        \
	})

/* for reg that should be same at every instance */
#define WRITE_EQUAL_REG(iommu, offset_in_instance, val)                      \
	do {                                                                 \
		sunxi_iommu_write(iommu, offset_in_instance, val);           \
		sunxi_iommu_write(                                           \
			iommu, offset_in_instance + IOMMU_PER_SET_ADDR_SIZE, \
			val);                                                \
	} while (0)

#define READ_EQUAL_REG(iommu, offset_in_instance)                             \
	({                                                                    \
		int ret[2] = { 0xA55A3CC3, 0xA55A3CC3 }; /*poison*/           \
		ret[0] = sunxi_iommu_read(iommu, offset_in_instance);         \
		ret[1] = sunxi_iommu_read(                                    \
			iommu, offset_in_instance + IOMMU_PER_SET_ADDR_SIZE); \
		/* TBD: equal assertion for rets */                           \
		ret[0];                                                       \
	})
/*
 * The format of device tree, and client device how to use it.
 *
 * /{
 *	....
 *	smmu: iommu@xxxxx {
 *		compatible = "allwinner,iommu";
 *		reg = <xxx xxx xxx xxx>;
 *		interrupts = <GIC_SPI xxx IRQ_TYPE_LEVEL_HIGH>;
 *		interrupt-names = "iommu-irq";
 *		clocks = <&iommu_clk>;
 *		clock-name = "iommu-clk";
 *		#iommu-cells = <1>;
 *		status = "enabled";
 *	};
 *
 *	de@xxxxx {
 *		.....
 *		iommus = <&smmu ID>;
 *	};
 *
 * }
 *
 * Here, ID number is 0 ~ 5, every client device have a unique id.
 * Every id represent a micro TLB, also represent a master device.
 *
 */

enum sunxi_iommu_version {
	IOMMU_VERSION_V10 = 0x10,
	IOMMU_VERSION_V11,
	IOMMU_VERSION_V12,
	IOMMU_VERSION_V13,
	IOMMU_VERSION_V14,
};

struct sunxi_iommu_plat_data {
	u32 version;
	u32 tlb_prefetch;
	u32 tlb_invalid_mode;
	u32 ptw_invalid_mode;
	char *master[IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT];
};

struct sunxi_iommu_domain {
	unsigned int *pgtable; /* first page directory, size is 16KB */
	u32 *sg_buffer;
	struct spinlock dt_lock; /* lock for modifying page table @ pgtable */
	struct dma_iommu_mapping *mapping;
	struct iommu_domain domain;
	//struct iova_domain iovad;
	/* list of master device, it represent a micro TLB */
	// struct list_head mdevs;
	spinlock_t lock;
};

struct sunxi_iommu_master_dev {
	struct device *dev;
	u32 id;
};

struct sunxi_iommu_dev {
	struct iommu_device iommu;
	struct device *dev;
	void __iomem *base;
	struct clk **clk;
	int irq[IOMMU_HW_SET_COUNT];
	u32 bypass;
	spinlock_t iommu_lock;
	struct iommu_domain *domain;
	struct list_head rsv_list;
	struct sunxi_iommu_plat_data *plat_data;
	struct reset_control **rst;
	u32 skip_mask;
	struct sunxi_iommu_master_dev *master;
	struct sunxi_iommu_data	*parent_data;
};
static struct class *distribute_master_cs;

struct sunxi_iommu_data {
	struct device *dev;
	struct sunxi_iommu_dev *sunxi_iommu;

	unsigned int tlbid;
	bool flag;
	struct dma_iommu_mapping *mapping;
};

static struct kmem_cache *iopte_cache;
static struct sunxi_iommu_dev *global_iommu_dev;
static bool iommu_hw_init_flag;
static struct device *dma_dev;

static struct sunxi_iommu_domain *to_sunxi_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct sunxi_iommu_domain, domain);
}

typedef void (*sunxi_iommu_fault_cb)(void);
static sunxi_iommu_fault_cb
	sunxi_iommu_fault_notify_cbs[IOMMU_HW_SET_COUNT * IOMMU_MIC_MAX_MASTER];

static inline u32 sunxi_iommu_read(struct sunxi_iommu_dev *iommu, u32 offset)
{
	if (unlikely((offset & (IOMMU_PER_SET_ADDR_SIZE - 1)) >
		     IOMMU_MIC_BASE_OFFSET)) {
		int master_id = 0;
		u32 read;
		u32 test_offset = offset;
		if (test_offset > IOMMU_PER_SET_ADDR_SIZE) {
			test_offset -= IOMMU_PER_SET_ADDR_SIZE;
			master_id += 6;
		}
		test_offset -= IOMMU_MIC_BASE_OFFSET;
		master_id += test_offset / 0x200;
		if (iommu->skip_mask & (1 << master_id))
			return 0;

		if (iommu->master[master_id].dev)
			WARN_ON(pm_runtime_get_if_in_use(
					iommu->master[master_id].dev) <= 0);
		read = readl(iommu->base + offset);
		if (iommu->master[master_id].dev)
			pm_runtime_put(iommu->master[master_id].dev);

		return read;
	}
	return readl(iommu->base + offset);
}

static inline void sunxi_iommu_write(struct sunxi_iommu_dev *iommu, u32 offset,
				     u32 value)
{
	if (unlikely((offset & (IOMMU_PER_SET_ADDR_SIZE - 1)) >
		     IOMMU_MIC_BASE_OFFSET)) {
		int master_id = 0;
		u32 test_offset = offset;
		if (test_offset > IOMMU_PER_SET_ADDR_SIZE) {
			test_offset -= IOMMU_PER_SET_ADDR_SIZE;
			master_id += 6;
		}
		test_offset -= IOMMU_MIC_BASE_OFFSET;
		master_id += test_offset / 0x200;
		if (iommu->skip_mask & (1 << master_id))
			return;

		if (iommu->master[master_id].dev)
			WARN_ON(pm_runtime_get_if_in_use(
					iommu->master[master_id].dev) <= 0);

		writel(value, iommu->base + offset);
		if (iommu->master[master_id].dev)
			pm_runtime_put(iommu->master[master_id].dev);

		return;
	}
	writel(value, iommu->base + offset);
}

static void sunxi_iommu_distribute_mater_get(int mask)
{
	int i;
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		if (!(mask & (1 << i)))
			continue;
		if (global_iommu_dev->master[i].dev)
			pm_runtime_get_sync(global_iommu_dev->master[i].dev);
	}
}
static void sunxi_iommu_distribute_mater_put(int mask)
{
	int i;
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		if (!(mask & (1 << i)))
			continue;
		if (global_iommu_dev->master[i].dev)
			pm_runtime_put(global_iommu_dev->master[i].dev);
	}
}

static void __sunxi_enable_device_iommu_unlocked(unsigned int master_id, bool flag)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int insatnce_offset;
	if (flag)
		iommu->bypass &= ~(1 << master_id);
	else
		iommu->bypass |= (1 << master_id);

	/* bypass is global, perform per instance fix up after bypass updated */
	/* we got serval instance at this iobase, get instance idx and in instance offset */
	insatnce_offset =
		(master_id / IOMMU_MIC_MAX_MASTER) * IOMMU_PER_SET_ADDR_SIZE;
	master_id = master_id % IOMMU_MIC_MAX_MASTER;

	sunxi_iommu_write(iommu, insatnce_offset + IOMMU_MIC_BYP_REG(master_id),
			  flag == 0);
}

void sunxi_enable_device_iommu(unsigned int master_id, bool flag)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long mflag;

	sunxi_iommu_distribute_mater_get(1 << master_id);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);

	__sunxi_enable_device_iommu_unlocked(master_id, flag);

	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	sunxi_iommu_distribute_mater_put(1 << master_id);
}
EXPORT_SYMBOL(sunxi_enable_device_iommu);

static int sunxi_tlb_flush(struct sunxi_iommu_dev *iommu)
{
	int ret = 0;
	int i, j;
	u32 target_reg_offset;
	/* flush all possible master */
	for (i = 0; i < IOMMU_HW_SET_COUNT; i++) {
		j = 0;
		target_reg_offset = IOMMU_PER_SET_ADDR_SIZE * i +
				    IOMMU_TLB_FLUSH_ENABLE_REG;
		sunxi_iommu_write(iommu, target_reg_offset, 0x00000003);

		for (; j < IOMMU_MIC_MAX_MASTER; j++) {
			target_reg_offset = IOMMU_PER_SET_ADDR_SIZE * i +
					    IOMMU_MIC_TLB_FLUSH_ENABLE_REG(j);
			sunxi_iommu_write(iommu, target_reg_offset, 0x00000001);
		}
	}
	udelay(10);//wait for hardware done
	for (i = 0; i < IOMMU_HW_SET_COUNT; i++) {
		j = 0;
		target_reg_offset = IOMMU_PER_SET_ADDR_SIZE * i +
				    IOMMU_TLB_FLUSH_ENABLE_REG;
		sunxi_iommu_write(iommu, target_reg_offset, 0x00000000);
	}

	return ret;
}

static void __sunxi_iommu_enable_interrupt_unlocked(int enable)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int i;

	if (enable)
		WRITE_EQUAL_REG(iommu, IOMMU_INT_ENABLE_REG,
				IOMMU_INT_L1PG_CLR_EN_MASK |
					IOMMU_INT_L2PG_CLR_EN_MASK);
	else
		WRITE_EQUAL_REG(iommu, IOMMU_INT_ENABLE_REG, 0x0);

	/* we write nothing else but 0/1 into register */
	enable = !!enable;
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		int masterid = i;
		unsigned int instance_offset =
			(masterid / IOMMU_MIC_MAX_MASTER) *
			IOMMU_PER_SET_ADDR_SIZE;
		masterid = masterid % IOMMU_MIC_MAX_MASTER;
		sunxi_iommu_write(iommu,
				  instance_offset +
					  IOMMU_MIC_INT_ENABLE_REG(masterid),
				  enable);
	}
}
void sunxi_iommu_enable_interrupt(int enable)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long mflag;

	sunxi_iommu_distribute_mater_get(0xffffffff);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);

	__sunxi_iommu_enable_interrupt_unlocked(enable);

	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	sunxi_iommu_distribute_mater_put(0xffffffff);
}
EXPORT_SYMBOL(sunxi_iommu_enable_interrupt);

static int sunxi_iommu_hw_init(struct iommu_domain *input_domain)
{
	int ret = 0;
	int iommu_enable = 0;
	phys_addr_t dte_addr;
	unsigned long mflag;
	int i;
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(input_domain);

	sunxi_iommu_distribute_mater_get(0xffffffff);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	dte_addr = __pa(sunxi_domain->pgtable);
	WRITE_EQUAL_REG(iommu, IOMMU_TTB_REG, dte_addr >> 14);

	/*
	 * set preftech functions, including:
	 * master prefetching and only prefetch valid page to TLB/PTW
	 */
	WRITE_EQUAL_REG(iommu, IOMMU_TLB_PREFETCH_REG, plat_data->tlb_prefetch);
	/* new TLB invalid function: use range(start, end) to invalid TLB, started at version V12 */
	if (plat_data->version >= IOMMU_VERSION_V12)
		WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG,
				plat_data->tlb_invalid_mode);
	/* new PTW invalid function: use range(start, end) to invalid PTW, started at version V14 */
	if (plat_data->version >= IOMMU_VERSION_V14)
		WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_MODE_SEL_REG,
				plat_data->ptw_invalid_mode);

	__sunxi_iommu_enable_interrupt_unlocked(1);

	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		if ((iommu->bypass >> i) & 0x1)
			__sunxi_enable_device_iommu_unlocked(i, 0);
		else
			__sunxi_enable_device_iommu_unlocked(i, 1);
	}

	ret = sunxi_tlb_flush(iommu);
	if (ret) {
		dev_err(iommu->dev, "Enable flush all request timed out\n");
		goto out;
	}
	WRITE_EQUAL_REG(iommu, IOMMU_AUTO_GATING_REG, IOMMU_AUTO_GATING_DEFAULT_VAL);
	WRITE_EQUAL_REG(iommu, IOMMU_ENABLE_REG, IOMMU_ENABLE);
	iommu_enable = READ_EQUAL_REG(iommu, IOMMU_ENABLE_REG);
	if (iommu_enable != 0x1) {
		iommu_enable = READ_EQUAL_REG(iommu, IOMMU_ENABLE_REG);
		if (iommu_enable != 0x1) {
			dev_err(iommu->dev,
				"iommu enable failed! No iommu in bitfile!\n");
			ret = -ENODEV;
			goto out;
		}
	}
	iommu_hw_init_flag = true;

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	sunxi_iommu_distribute_mater_put(0xffffffff);

	return ret;
}

static int sunxi_tlb_invalid_locked(dma_addr_t iova, dma_addr_t iova_mask)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	dma_addr_t iova_end = iova_mask;
	int ret = 0;

	/* new TLB invalid function: use range(start, end) to invalid TLB page */
	if (plat_data->version >= IOMMU_VERSION_V12) {
		pr_debug("iommu: TLB invalid:0x%pad-0x%pad\n", &iova,
			 &iova_end);
		WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_START_ADDR_REG,
				(iova >> IOMMU_TLB_IVLD_ADDR_ALIGN_SHIFT)
					<< IOMMU_TLB_IVLD_ADDR_OFFSET);
		WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_END_ADDR_REG,
				(iova_end >> IOMMU_TLB_IVLD_ADDR_ALIGN_SHIFT)
					<< IOMMU_TLB_IVLD_ADDR_OFFSET);
	} else {
		/* old TLB invalid function: only invalid 4K at one time */
		WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_ADDR_REG, iova);
		WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG, iova_mask);
	}
	WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);

	udelay(10); //wait for hardware done
	WRITE_EQUAL_REG(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x0);

	return ret;
}

static int sunxi_tlb_invalid(dma_addr_t iova, dma_addr_t iova_mask)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	ret = sunxi_tlb_invalid_locked(iova, iova_mask);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

static int sunxi_ptw_cache_invalid_locked(dma_addr_t iova_start,
					  dma_addr_t iova_end)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	int ret = 0;

	/* new PTW invalid function: use range(start, end) to invalid PTW page */
	if (plat_data->version >= IOMMU_VERSION_V14) {
		pr_debug("iommu: PTW invalid:0x%pad-0x%pad\n", &iova_start,
			 &iova_end);
		WARN_ON(iova_end == 0);
		WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_START_ADDR_REG,
				(iova_start >> IOMMU_PC_IVLD_ADDR_ALIGN_SHIFT)
					<< IOMMU_PC_IVLD_ADDR_OFFSET);
		WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_END_ADDR_REG,
				(iova_end >> IOMMU_PC_IVLD_ADDR_ALIGN_SHIFT)
					<< IOMMU_PC_IVLD_ADDR_OFFSET);
	} else {
		/* old ptw invalid function: only invalid 1M at one time */
		pr_debug("iommu: PTW invalid:0x%x\n", (unsigned int)iova_start);
		WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_ADDR_REG, iova_start);
	}
	WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);

	udelay(10); //wait for hardware done
	WRITE_EQUAL_REG(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x0);
	return ret;
}

static int sunxi_ptw_cache_invalid(dma_addr_t iova_start, dma_addr_t iova_end)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_ptw_cache_invalid_locked(iova_start, iova_end);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

static void sunxi_zap_tlb(unsigned long iova, size_t size)
{
	const struct sunxi_iommu_plat_data *plat_data =
		global_iommu_dev->plat_data;

	if (plat_data->version <= IOMMU_VERSION_V11) {
		sunxi_tlb_invalid(iova, (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid(iova + SPAGE_SIZE, (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid(iova + size, (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid(iova + size + SPAGE_SIZE, (u32)IOMMU_PT_MASK);
		sunxi_ptw_cache_invalid(iova, 0);
		sunxi_ptw_cache_invalid(iova + SPD_SIZE, 0);
		sunxi_ptw_cache_invalid(iova + size, 0);
		sunxi_ptw_cache_invalid(iova + size + SPD_SIZE, 0);
	} else if (plat_data->version <= IOMMU_VERSION_V13) {
		sunxi_tlb_invalid(iova, iova + 2 * SPAGE_SIZE);
		sunxi_tlb_invalid(iova + size - SPAGE_SIZE,
				  iova + size + 8 * SPAGE_SIZE);
		sunxi_ptw_cache_invalid(iova, 0);
		sunxi_ptw_cache_invalid(iova + size, 0);

		sunxi_ptw_cache_invalid(iova + SPD_SIZE, 0);
		sunxi_ptw_cache_invalid(iova + size + SPD_SIZE, 0);
		sunxi_ptw_cache_invalid(iova + size + 2 * SPD_SIZE, 0);
	} else {
		sunxi_tlb_invalid(iova, iova + 2 * SPAGE_SIZE);
		sunxi_tlb_invalid(iova + size - SPAGE_SIZE,
				  iova + size + 8 * SPAGE_SIZE);
		sunxi_ptw_cache_invalid(iova, iova + SPD_SIZE);
		sunxi_ptw_cache_invalid(iova + size - SPD_SIZE, iova + size);
	}

	return;
}

static int sunxi_iommu_map(struct iommu_domain *domain, unsigned long iova,
			   phys_addr_t paddr, size_t size, size_t count,
			    int prot, gfp_t gfp, size_t *mapped)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	size_t iova_start, iova_end, s_iova_start;
	int ret;
	unsigned long flags;

	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size);
	s_iova_start = iova_start;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	ret = sunxi_pgtable_prepare_l1_tables(sunxi_domain->pgtable, iova_start,
					      iova_end, prot);
	if (ret) {
		spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);
		return -ENOMEM;
	}

	iova_start = s_iova_start;
	sunxi_pgtable_prepare_l2_tables(sunxi_domain->pgtable,
					iova_start, iova_end, paddr, prot);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);
	*mapped = size;

	return 0;
}

static size_t sunxi_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
				size_t size, size_t count, struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	const struct sunxi_iommu_plat_data *plat_data;
	size_t iova_start, iova_end;
	u32 iova_tail_size;
	unsigned long flags;

	plat_data = global_iommu_dev->plat_data;
	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size);

	if (gather->start > iova_start)
		gather->start = iova_start;
	if (gather->end < iova_end)
		gather->end = iova_end;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	/* Invalid TLB and PTW */
	if (plat_data->version >= IOMMU_VERSION_V12)
		sunxi_tlb_invalid(iova_start, iova_end);
	if (plat_data->version >= IOMMU_VERSION_V14)
		sunxi_ptw_cache_invalid(iova_start, iova_end);

	for (; iova_start < iova_end;) {
		iova_tail_size = sunxi_pgtable_delete_l2_tables(
			 sunxi_domain->pgtable, iova_start, iova_end);

		if (plat_data->version < IOMMU_VERSION_V14)
			sunxi_ptw_cache_invalid(iova_start, 0);
		iova_start += iova_tail_size;
	}
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return size;
}

static int sunxi_iommu_iotlb_sync_map(struct iommu_domain *domain, unsigned long iova,
				size_t size)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	unsigned long flags;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	sunxi_zap_tlb(iova, size);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return 0;
}

void sunxi_iommu_iotlb_sync(struct iommu_domain *domain,
			    struct iommu_iotlb_gather *iotlb_gather)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	unsigned long flags;

	if (plat_data->version >= IOMMU_VERSION_V14)
		return;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	sunxi_zap_tlb(iotlb_gather->start,
		      iotlb_gather->end - iotlb_gather->start);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return;
}

static phys_addr_t sunxi_iommu_iova_to_phys(struct iommu_domain *domain,
					    dma_addr_t iova)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	phys_addr_t ret = 0;
	unsigned long flags;

	WARN_ON(sunxi_domain->pgtable == NULL);

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	ret = sunxi_pgtable_iova_to_phys(sunxi_domain->pgtable, iova);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return ret;
}

static struct iommu_domain *sunxi_iommu_domain_alloc_paging(struct device *dev)
{
	struct sunxi_iommu_domain *sunxi_domain;

	sunxi_domain = kzalloc(sizeof(*sunxi_domain), GFP_KERNEL);

	if (!sunxi_domain)
		return NULL;

	sunxi_domain->pgtable = sunxi_pgtable_alloc();
	if (!sunxi_domain->pgtable) {
		pr_err("sunxi domain get pgtable failed\n");
		goto err_page;
	}

	sunxi_domain->sg_buffer = (unsigned int *)__get_free_pages(
		GFP_KERNEL, get_order(MAX_SG_TABLE_SIZE));
	if (!sunxi_domain->sg_buffer) {
		pr_err("sunxi domain get sg_buffer failed\n");
		goto err_sg_buffer;
	}

	sunxi_domain->domain.pgsize_bitmap = SZ_4K | 
		SZ_16K | SZ_64K | SZ_256K | SZ_1M | SZ_4M | SZ_16M;

	sunxi_domain->domain.geometry.aperture_start = 0;
	sunxi_domain->domain.geometry.aperture_end = (1ULL << 34) - 1;
	sunxi_domain->domain.geometry.force_aperture = true;
	spin_lock_init(&sunxi_domain->dt_lock);

	if (!iommu_hw_init_flag) {
		if (sunxi_iommu_hw_init(&sunxi_domain->domain))
			pr_err("sunxi iommu hardware init failed\n");
	}

	return &sunxi_domain->domain;

err_sg_buffer:
	sunxi_pgtable_free(sunxi_domain->pgtable);
	sunxi_domain->pgtable = NULL;
err_page:
	kfree(sunxi_domain);

	return NULL;
}

static void sunxi_iommu_domain_free(struct iommu_domain *domain)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	unsigned long flags;

	sunxi_iommu_distribute_mater_get(0xffffffff);
	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);
	sunxi_pgtable_clear(sunxi_domain->pgtable);
	sunxi_tlb_flush(global_iommu_dev);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);
	sunxi_iommu_distribute_mater_put(0xffffffff);
	sunxi_pgtable_free(sunxi_domain->pgtable);
	sunxi_domain->pgtable = NULL;
	free_pages((unsigned long)sunxi_domain->sg_buffer,
		   get_order(MAX_SG_TABLE_SIZE));
	sunxi_domain->sg_buffer = NULL;
	kfree(sunxi_domain);
}

static int sunxi_iommu_attach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	return 0;
}

static void sunxi_iommu_probe_device_finalize(struct device *dev)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);

	sunxi_enable_device_iommu(data->tlbid, data->flag);
}

static struct iommu_device *sunxi_iommu_probe_device(struct device *dev)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);

	if (!data) /* Not a iommu client device */
		return ERR_PTR(-ENODEV);

	return &data->sunxi_iommu->iommu;
}

static void sunxi_iommu_release_device(struct device *dev)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);

	if (!data)
		return;

	sunxi_enable_device_iommu(data->tlbid, false);
	dev->iommu_group = NULL;
	dev->dma_parms = NULL;
	kfree(data);
	data = NULL;
	dev_iommu_priv_set(dev, NULL);
}

static int sunxi_iommu_of_xlate(struct device *dev,
				const struct of_phandle_args *args)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sunxi_iommu_dev *iommu;

	if (!sysmmu)
		return -ENODEV;

	iommu = platform_get_drvdata(sysmmu);
	if (iommu == NULL)
		return -ENODEV;

	if (!data) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		data->tlbid = args->args[0];
		data->flag = args->args[1];
		data->sunxi_iommu = iommu;
		data->dev = dev;
		data->sunxi_iommu->parent_data = data;
		dev_iommu_priv_set(dev, data);
	}

	return 0;
}

static void __dump_int_from_one_instance(struct sunxi_iommu_dev *iommu,
					 int index)
{
	u32 inter_status_reg = 0;
	dma_addr_t addr_reg = 0;
	u32 int_masterid_bitmap = 0;
	u32 data_reg = 0;
	u32 l1_pgint_reg = 0;
	u32 l2_pgint_reg = 0;
	u32 master_id = 0;
	unsigned long mflag;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	u32 offset = index * IOMMU_PER_SET_ADDR_SIZE;
	u32 int_clear_mask;
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(iommu->domain);

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	inter_status_reg = sunxi_iommu_read(iommu, offset + IOMMU_INT_STA_REG) &
			   0x3ffff;
	l1_pgint_reg = sunxi_iommu_read(iommu, offset + IOMMU_L1PG_INT_REG);
	l2_pgint_reg = sunxi_iommu_read(iommu, offset + IOMMU_L2PG_INT_REG);
	int_masterid_bitmap = inter_status_reg | l1_pgint_reg | l2_pgint_reg;

	if (inter_status_reg & (1 << 0)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 0]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(0));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(0))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(0));
	} else if (inter_status_reg & (1 << 1)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 1]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(1));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(1))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(1));
	} else if (inter_status_reg & (1 << 2)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 2]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(2));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(2))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(2));
	} else if (inter_status_reg & (1 << 3)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 3]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(3));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(3))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(3));
	} else if (inter_status_reg & (1 << 4)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 4]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(4));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(4))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(4));
	} else if (inter_status_reg & (1 << 5)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 5]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(5));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(5))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(5));
	} else if (inter_status_reg & (1 << 6)) {
		pr_err("%s Invalid Authority\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER + 6]);
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_LOW_ADDR_REG(6));
		addr_reg |= (u64)sunxi_iommu_read(
				    iommu,
				    offset + IOMMU_MIC_INT_ERR_HIGH_ADDR_REG(6))
			    << 32;
		data_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_MIC_INT_ERR_DATA_REG(6));
	} else if (inter_status_reg & IOMMU_INT_L1PG_STA_MASK) {
		/*It's OK to prefetch an invalid page, no need to print msg for debug.*/
		if (!(int_masterid_bitmap & (1U << 31)))
			pr_err("L1 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_INT_ERR_LOW_ADDR7_REG);
		addr_reg |=
			(u64)sunxi_iommu_read(
				iommu, offset + IOMMU_INT_ERR_HIGH_ADDR7_REG)
			<< 32;
		data_reg = sunxi_iommu_read(iommu,
					    offset + IOMMU_INT_ERR_DATA7_REG);
	} else if (inter_status_reg & IOMMU_INT_L2PG_STA_MASK) {
		if (!(int_masterid_bitmap & (1U << 31)))
			pr_err("L2 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(
			iommu, offset + IOMMU_INT_ERR_LOW_ADDR8_REG);
		addr_reg |=
			(u64)sunxi_iommu_read(
				iommu, offset + IOMMU_INT_ERR_HIGH_ADDR8_REG)
			<< 32;
		data_reg = sunxi_iommu_read(iommu,
					    offset + IOMMU_INT_ERR_DATA8_REG);
	} else
		pr_err("sunxi iommu int error!!!\n");

	if (!(int_masterid_bitmap & (1U << 31))) {
		sunxi_pgtable_invalid_helper(sunxi_domain->pgtable, addr_reg);
		int_masterid_bitmap &= 0xffff;
		master_id = __ffs(int_masterid_bitmap);
		pr_err("Bug is in %s module, invalid address: 0x%llx, data:0x%x, id:0x%x\n",
		       plat_data->master[index * IOMMU_MIC_MAX_MASTER +
					 master_id],
		       addr_reg, data_reg, int_masterid_bitmap);

		/* master debug callback */
		if (sunxi_iommu_fault_notify_cbs[index * IOMMU_MIC_MAX_MASTER +
						 master_id])
			sunxi_iommu_fault_notify_cbs
				[index * IOMMU_MIC_MAX_MASTER + master_id]();
	}

	/* invalid TLB */
	sunxi_tlb_invalid_locked(addr_reg, addr_reg + 4 * SPAGE_SIZE);

	/* invalid PTW */
	sunxi_ptw_cache_invalid_locked(addr_reg, addr_reg + 2 * SPD_SIZE);

	int_clear_mask = l1_pgint_reg ? IOMMU_INT_L1PG_CLR_EN_MASK : 0;
	int_clear_mask |= l2_pgint_reg ? IOMMU_INT_L2PG_CLR_EN_MASK : 0;
	sunxi_iommu_write(iommu, offset + IOMMU_INT_CLR_REG, int_clear_mask);
	if (!(int_masterid_bitmap & (1U << 31)))
		sunxi_iommu_write(iommu, offset + IOMMU_MIC_INT_CLR_REG(master_id), 1);
	inter_status_reg |= (l1_pgint_reg | l2_pgint_reg);
	inter_status_reg &= 0xffff;
	sunxi_iommu_write(iommu, offset + IOMMU_RESET_REG, ~inter_status_reg);
	sunxi_iommu_write(iommu, offset + IOMMU_RESET_REG, 0xffffffff);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
}

static irqreturn_t sunxi_iommu_irq(int irq, void *dev_id)
{
	struct sunxi_iommu_dev *iommu = dev_id;

	__dump_int_from_one_instance(iommu, irq == iommu->irq[1]);

	return IRQ_HANDLED;
}

static ssize_t sunxi_iommu_enable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct sunxi_iommu_data *iommu_data = dev_iommu_priv_get(dev);
	struct sunxi_iommu_dev *iommu = iommu_data->sunxi_iommu;
	u32 data;

	spin_lock(&iommu->iommu_lock);
	data = READ_EQUAL_REG(iommu, IOMMU_PMU_ENABLE_REG);
	spin_unlock(&iommu->iommu_lock);

	return scnprintf(buf, PAGE_SIZE,
		"enable = %d\n", data & 0x1 ? 1 : 0);
}

static ssize_t sunxi_iommu_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct sunxi_iommu_data *iommu_data = dev_iommu_priv_get(dev);
	struct sunxi_iommu_dev *iommu = iommu_data->sunxi_iommu;
	unsigned long val;
	u32 data;
	int retval;
	int i;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		spin_lock(&iommu->iommu_lock);
		data = READ_EQUAL_REG(iommu, IOMMU_PMU_ENABLE_REG);
		WRITE_EQUAL_REG(iommu, IOMMU_PMU_ENABLE_REG, data | 0x1);
		data = READ_EQUAL_REG(iommu, IOMMU_PMU_CLR_REG);
		WRITE_EQUAL_REG(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when(
			((sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG) & 0x1) ||
			 (sunxi_iommu_read(iommu, IOMMU_PER_SET_ADDR_SIZE +
							IOMMU_PMU_CLR_REG) &
			  0x1)),
			1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
		spin_unlock(&iommu->iommu_lock);
	} else {
		spin_lock(&iommu->iommu_lock);
		data = READ_EQUAL_REG(iommu, IOMMU_PMU_CLR_REG);
		WRITE_EQUAL_REG(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when(
			((sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG) & 0x1) ||
			 (sunxi_iommu_read(iommu, IOMMU_PER_SET_ADDR_SIZE +
							IOMMU_PMU_CLR_REG) &
			  0x1)),
			1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
		data = READ_EQUAL_REG(iommu, IOMMU_PMU_ENABLE_REG);
		WRITE_EQUAL_REG(iommu, IOMMU_PMU_ENABLE_REG, data & ~0x1);
		spin_unlock(&iommu->iommu_lock);
	}

	sunxi_iommu_distribute_mater_get(0xffffffff);
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		int masterid = i;
		unsigned int instance_offset =
			(masterid / IOMMU_MIC_MAX_MASTER) *
			IOMMU_PER_SET_ADDR_SIZE;
		masterid = masterid % IOMMU_MIC_MAX_MASTER;
		sunxi_iommu_write(iommu,
				  instance_offset +
					  IOMMU_MIC_PMU_ENABLE_REG(masterid),
				  !!val);
		sunxi_iommu_write(iommu,
				  instance_offset +
					  IOMMU_MIC_PMU_CLR_REG(masterid),
				  1);
	}
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT; i++) {
		int masterid = i;
		unsigned int instance_offset =
			(masterid / IOMMU_MIC_MAX_MASTER) *
			IOMMU_PER_SET_ADDR_SIZE;
		masterid = masterid % IOMMU_MIC_MAX_MASTER;
		sunxi_iommu_write(iommu,
				  instance_offset +
					  IOMMU_MIC_PMU_CLR_REG(masterid),
				  0);
	}
	sunxi_iommu_distribute_mater_put(0xffffffff);

	return count;
}

static ssize_t sunxi_iommu_profilling_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct sunxi_iommu_dev *iommu = data->sunxi_iommu;
	const struct sunxi_iommu_plat_data *plat_data = iommu->plat_data;
	struct {
		u64 macrotlb_access_count;
		u64 macrotlb_hit_count;
		u64 ptwcache_access_count;
		u64 ptwcache_hit_count;
		struct {
			u64 access_count;
			u64 hit_count;
			u64 latency;
			u32 max_latency;
		} micro_tlb[IOMMU_MIC_MAX_MASTER];
	} *iommu_profile;

	int i, j;
	int out_len = 0;

	iommu_profile = kmalloc(sizeof(*iommu_profile) * IOMMU_HW_SET_COUNT,
				GFP_KERNEL);
	if (!iommu_profile)
		goto err;

	sunxi_iommu_distribute_mater_get(0xffffffff);
	spin_lock(&iommu->iommu_lock);

	for (i = 0; i < IOMMU_HW_SET_COUNT; i++) {
		j = 0;
		iommu_profile[i].macrotlb_access_count =
			((u64)(sunxi_iommu_read(
				       iommu,
				       i * IOMMU_PER_SET_ADDR_SIZE +
					       IOMMU_PMU_ACCESS_HIGH7_REG) &
			       0x7ff)
			 << 32) |
			sunxi_iommu_read(iommu,
					 i * IOMMU_PER_SET_ADDR_SIZE +
						 IOMMU_PMU_ACCESS_LOW7_REG);
		iommu_profile[i].macrotlb_hit_count =
			((u64)(sunxi_iommu_read(
				       iommu, i * IOMMU_PER_SET_ADDR_SIZE +
						      IOMMU_PMU_HIT_HIGH7_REG) &
			       0x7ff)
			 << 32) |
			sunxi_iommu_read(iommu, i * IOMMU_PER_SET_ADDR_SIZE +
							IOMMU_PMU_HIT_LOW7_REG);

		iommu_profile[i].ptwcache_access_count =
			((u64)(sunxi_iommu_read(
				       iommu,
				       i * IOMMU_PER_SET_ADDR_SIZE +
					       IOMMU_PMU_ACCESS_HIGH8_REG) &
			       0x7ff)
			 << 32) |
			sunxi_iommu_read(iommu,
					 i * IOMMU_PER_SET_ADDR_SIZE +
						 IOMMU_PMU_ACCESS_LOW8_REG);
		iommu_profile[i].ptwcache_hit_count =
			((u64)(sunxi_iommu_read(
				       iommu, i * IOMMU_PER_SET_ADDR_SIZE +
						      IOMMU_PMU_HIT_HIGH8_REG) &
			       0x7ff)
			 << 32) |
			sunxi_iommu_read(iommu, i * IOMMU_PER_SET_ADDR_SIZE +
							IOMMU_PMU_HIT_LOW8_REG);
		for (; j < IOMMU_MIC_MAX_MASTER; j++) {
			iommu_profile[i].micro_tlb[j].access_count =
				((u64)(sunxi_iommu_read(
					       iommu,
					       i * IOMMU_PER_SET_ADDR_SIZE +
						       IOMMU_MIC_PMU_ACCESS_HIGH_REG(
							       0)) &
				       0x7ff)
				 << 32) |
				sunxi_iommu_read(
					iommu,
					i * IOMMU_PER_SET_ADDR_SIZE +
						IOMMU_MIC_PMU_ACCESS_LOW_REG(
							0));
			iommu_profile[i].micro_tlb[j].hit_count =
				((u64)(sunxi_iommu_read(
					       iommu,
					       i * IOMMU_PER_SET_ADDR_SIZE +
						       IOMMU_MIC_PMU_HIT_HIGH_REG(
							       0)) &
				       0x7ff)
				 << 32) |
				sunxi_iommu_read(
					iommu,
					i * IOMMU_PER_SET_ADDR_SIZE +
						IOMMU_MIC_PMU_HIT_LOW_REG(0));
		}
	}

	spin_unlock(&iommu->iommu_lock);
	sunxi_iommu_distribute_mater_put(0xffffffff);
	out_len = 0;
	for (i = 0; i < IOMMU_HW_SET_COUNT; i++) {
		j = 0;
		out_len += sysfs_emit_at(
			buf, out_len,
			"iommu%d macrotlb_access_count = 0x%llx\n", i,
			iommu_profile[i].macrotlb_access_count);
		out_len +=
			sysfs_emit_at(buf, out_len,
				      "iommu%d macrotlb_hit_count = 0x%llx\n",
				      i, iommu_profile[i].macrotlb_hit_count);
		out_len += sysfs_emit_at(
			buf, out_len,
			"iommu%d ptwcache_access_count = 0x%llx\n", i,
			iommu_profile[i].ptwcache_access_count);
		out_len +=
			sysfs_emit_at(buf, out_len,
				      "iommu%d ptwcache_hit_count = 0x%llx\n",
				      i, iommu_profile[i].ptwcache_hit_count);
		for (; j < IOMMU_MIC_MAX_MASTER; j++) {
			out_len += sysfs_emit_at(
				buf, out_len,
				"%s_access_count = 0x%llx\n",
				plat_data->master[i * IOMMU_MIC_MAX_MASTER + j],
				iommu_profile[i].micro_tlb[j].access_count);
			out_len += sysfs_emit_at(
				buf, out_len, "%s_hit_count = 0x%llx\n",
				plat_data->master[i * IOMMU_MIC_MAX_MASTER + j],
				iommu_profile[i].micro_tlb[j].hit_count);
			out_len += sysfs_emit_at(
				buf, out_len,
				"%s_total_latency = 0x%llx\n",
				plat_data->master[i * IOMMU_MIC_MAX_MASTER + j],
				iommu_profile[i].micro_tlb[j].latency);
			out_len += sysfs_emit_at(
				buf, out_len, "%s_max_latency = 0x%x\n",
				plat_data->master[i * IOMMU_MIC_MAX_MASTER + j],
				iommu_profile[i].micro_tlb[j].max_latency);
		}
	}

err:
	if (iommu_profile)
		kfree(iommu_profile);

	return out_len;
}

static ssize_t sunxi_iommu_map_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(data->sunxi_iommu->domain);
	ssize_t len = 0;

	len = sunxi_iommu_dump_rsv_list(&data->sunxi_iommu->rsv_list, len, buf,
					PAGE_SIZE, true);
	len = sunxi_pgtable_dump(sunxi_domain->pgtable, len, buf, PAGE_SIZE,
				 true);
	
	return len;
}

static struct device_attribute sunxi_iommu_enable_attr =
	__ATTR(enable, 0644, sunxi_iommu_enable_show, sunxi_iommu_enable_store);
static struct device_attribute sunxi_iommu_profilling_attr =
	__ATTR(profilling, 0444, sunxi_iommu_profilling_show, NULL);

static struct device_attribute sunxi_iommu_map_attr =
	__ATTR(page_debug, 0444, sunxi_iommu_map_show, NULL);

static void sunxi_iommu_sysfs_create(struct platform_device *_pdev,
				     struct sunxi_iommu_dev *iommu)
{
	device_create_file(&_pdev->dev, &sunxi_iommu_enable_attr);
	device_create_file(&_pdev->dev, &sunxi_iommu_profilling_attr);
	device_create_file(&_pdev->dev, &sunxi_iommu_map_attr);
}

static void sunxi_iommu_sysfs_remove(struct platform_device *_pdev)
{
	device_remove_file(&_pdev->dev, &sunxi_iommu_enable_attr);
	device_remove_file(&_pdev->dev, &sunxi_iommu_profilling_attr);
	device_remove_file(&_pdev->dev, &sunxi_iommu_map_attr);
}

static void sunxi_iommu_get_resv_regions(struct device *dev,
					 struct list_head *head)
{
	struct iommu_resv_region *entry;
	struct iommu_resv_region *region;
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct sunxi_iommu_dev *iommu = data->sunxi_iommu;

	if (list_empty(&iommu->rsv_list)) {
		bus_for_each_dev(&platform_bus_type, NULL, &iommu->rsv_list,
			sunxi_iommu_check_cmd);
		return;
	}

	list_for_each_entry(entry, &iommu->rsv_list, list) {
		dev_err(dev, "iommu_alloc_resv_region\n");
		region = iommu_alloc_resv_region(entry->start, entry->length,
						 entry->prot, entry->type, GFP_KERNEL);
		list_add_tail(&region->list, head);
	}
}

static const struct iommu_domain_ops sunxi_iommu_domain_ops = {
	.attach_dev		= sunxi_iommu_attach_dev,
	.iotlb_sync_map	= sunxi_iommu_iotlb_sync_map,
	.iotlb_sync		= sunxi_iommu_iotlb_sync,
	.iova_to_phys	= sunxi_iommu_iova_to_phys,
	.map_pages		= sunxi_iommu_map,
	.unmap_pages	= sunxi_iommu_unmap,
	.free			= sunxi_iommu_domain_free,
};
static const struct iommu_ops sunxi_iommu_ops = {
	.device_group = generic_single_device_group,
	.domain_alloc_paging = sunxi_iommu_domain_alloc_paging,
	.of_xlate = sunxi_iommu_of_xlate,
	.get_resv_regions = sunxi_iommu_get_resv_regions,
	.probe_device = sunxi_iommu_probe_device,
	.probe_finalize = sunxi_iommu_probe_device_finalize,
	.release_device = sunxi_iommu_release_device,
	.default_domain_ops = &sunxi_iommu_domain_ops,
};

static int __init_plat_data_from_ofnode(struct sunxi_iommu_plat_data *data,
					struct device_node *node)
{
	int count;
	const char *tmp;
	int i;

	memset(data, 0, sizeof(*data));
	of_property_read_u32(node, "version", &data->version);
	of_property_read_u32(node, "ptw_invalid_mode", &data->ptw_invalid_mode);
	of_property_read_u32(node, "tlb_invalid_mode", &data->tlb_invalid_mode);
	count = of_property_read_string_array(node, "masters", NULL,
					      ARRAY_SIZE(data->master));
	if (count <= 0)
		return -ENODATA;
	for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT && i < count;
	     i++) {
		data->master[i] = kmalloc(16, GFP_KERNEL);
		if (!data->master[i])
			return -ENOMEM;
		of_property_read_string_index(node, "masters", i, &tmp);
		strncpy(data->master[i], tmp, 16);
	}
	return 0;
}

static int
sunxi_iommu_prepare_master(struct sunxi_iommu_master_dev *master_dev,
			    struct device_node *child, int id,
			    struct device *dev)
{
	int ret;
	master_dev->dev = device_create(distribute_master_cs, dev,
					MKDEV(0, 0), NULL, "%s",
					child->name);
	if (IS_ERR(master_dev->dev)) {
		return PTR_ERR(master_dev->dev);
	}
	master_dev->dev->of_node = child;
	ret = dev_pm_domain_attach(master_dev->dev, 0);
	if (ret) {
		dev_err(master_dev->dev, "attach fail:%d\n", ret);
		goto err_dev;
	}
	ret = pm_runtime_set_active(master_dev->dev);
	if (ret) {
		dev_err(master_dev->dev, "active fail:%d\n", ret);
		goto err_att;
	}
	pm_runtime_use_autosuspend(master_dev->dev);
	pm_runtime_set_autosuspend_delay(master_dev->dev, 5000);
	pm_runtime_enable(master_dev->dev);
	return ret;
err_att:
	dev_pm_domain_detach(master_dev->dev, 0);
err_dev:
	device_destroy(distribute_master_cs, MKDEV(0, 0));
	return ret;
}

static int
sunxi_iommu_probe_distribute_masters(struct device *dev,
				     struct sunxi_iommu_dev *iommu)
{
	struct device_node *np = dev->of_node;
	struct device_node *child = NULL;
	int ret;
	static int32_t pd_configurated_mask = -1;

	iommu->bypass = DEFAULT_BYPASS_VALUE;

	iommu->master = devm_kcalloc(
		dev, IOMMU_HW_SET_COUNT * IOMMU_MIC_MAX_MASTER,
		sizeof(struct sunxi_iommu_master_dev), GFP_KERNEL | __GFP_ZERO);
	if (!iommu->master)
		return -ENOMEM;

	if (!distribute_master_cs) {
		distribute_master_cs = class_create("iommu_master");
		if (IS_ERR(distribute_master_cs)) {
			pr_err("device class file already in use\n");
			return -ENOMEM;
		}
	}

	for_each_available_child_of_node(np, child) {
		uint32_t id;
		struct sunxi_iommu_master_dev *master_dev;
		if (!of_property_present(child, "iommu-master"))
			continue;
		if (of_property_read_u32(child, "id", &id))
			BUG();
		if (of_property_present(child, "skip")) {
			iommu->skip_mask |= 1 << id;
			continue;
		}

		master_dev = &iommu->master[id];
		master_dev->id = id;

		if (of_property_present(child, "power-domains")) {
			struct of_phandle_args pd_args;
			// struct device_node *pd_np = NULL;
			

			ret = of_parse_phandle_with_args(child, "power-domains",
							 "#power-domain-cells",
							 0, &pd_args);
			BUG_ON(ret < 0);

			if (pd_configurated_mask == -1) {
				pd_configurated_mask = 0;

				pd_configurated_mask |= 1 << pd_args.args[0];
			}

			if (pd_configurated_mask & (1 << pd_args.args[0])) {
				ret = sunxi_iommu_prepare_master(
					master_dev, child, id, dev);
				if (ret)
					return ret;
			}
		}
		iommu->skip_mask &= ~(1 << id);
	}

	return 0;
}

static int sunxi_iommu_probe(struct platform_device *pdev)
{
	int ret, irq, irq_got;
	struct device *dev = &pdev->dev;
	struct sunxi_iommu_dev *iommu;
	struct resource *res;
	struct property *prop;
	struct clk **pclk;
	const char *name;
	int clk_count;
	int i;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	ret = sunxi_iommu_probe_distribute_masters(dev, iommu);
	if (ret) {
		dev_err(dev, "master probe failed with %d\n", ret);
		return ret;
	}

	iopte_cache = sunxi_pgtable_alloc_pte_cache();
	if (!iopte_cache) {
		pr_err("%s: Failed to create sunx-iopte-cache.\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "Unable to find resource region\n");
		ret = -ENOENT;
		goto err_res;
	}

	iommu->base = devm_ioremap_resource(&pdev->dev, res);
	if (!iommu->base) {
		dev_dbg(dev, "Unable to map IOMEM @ PA:%#x\n",
			(unsigned int)res->start);
		ret = -ENOENT;
		goto err_res;
	}

	for (irq_got = 0; irq_got < IOMMU_HW_SET_COUNT; irq_got++) {
		irq = platform_get_irq(pdev, irq_got);
		if (irq <= 0) {
			dev_dbg(dev, "Unable to find IRQ resource\n");
			ret = -ENOENT;
			goto err_irq;
		}
		pr_info("sunxi iommu: irq = %d\n", irq);

		ret = devm_request_irq(dev, irq, sunxi_iommu_irq, 0,
				       dev_name(dev), (void *)iommu);
		if (ret < 0) {
			dev_dbg(dev, "Unabled to register interrupt handler\n");
			goto err_irq;
		}

		iommu->irq[irq_got] = irq;
	}

	clk_count = of_count_phandle_with_args(dev->of_node, "resets", "#reset-cells");
	if (clk_count > 0) {
		iommu->rst = devm_kcalloc(dev, clk_count, sizeof(void *),
						GFP_KERNEL | __GFP_ZERO);
		if (!iommu->rst)
			goto err_clk;
		for (i = 0; i < clk_count; i++) {
			iommu->rst[i] =
				devm_reset_control_get_by_index(dev, i);
			if (IS_ERR_OR_NULL(iommu->rst[i])) {
				dev_err(dev, "unable to get reset[%d]", i);
				goto err_clk;
			}
			if (reset_control_deassert(iommu->rst[i])) {
				dev_err(dev, "Couldn't reset control deassert\n");
				goto err_clk;
			}
		}
	}

	clk_count = of_property_count_strings(dev->of_node, "clock-names");
	if (clk_count <= 0) {
		dev_err(dev, "no clocks found\n");
		goto err_clk;
	}
	iommu->clk = kzalloc(
		sizeof(void *) * (clk_count + 1 /*sentinel*/), GFP_KERNEL);
	pclk = iommu->clk;
	of_property_for_each_string(dev->of_node, "clock-names", prop, name) {
		*pclk = devm_clk_get(dev, name);
		if (IS_ERR(*pclk)) {
			iommu->clk = NULL;
			dev_dbg(dev, "Unable to find clock %s\n", name);
			ret = -ENOENT;
			goto err_clk;
		}
		pclk++;
	}
	pclk = iommu->clk;
	while (*pclk) {
		clk_prepare_enable(*pclk);
		pclk++;
	}

	platform_set_drvdata(pdev, iommu);
	iommu->dev = dev;
	spin_lock_init(&iommu->iommu_lock);
	global_iommu_dev = iommu;
	iommu->plat_data =
		kmalloc(sizeof(*iommu->plat_data), GFP_KERNEL);

	if (!iommu->plat_data) {
		dev_dbg(dev, "no mem for plat data\n");
		ret = -ENOMEM;
		goto err_plat;
	}

	ret = __init_plat_data_from_ofnode(iommu->plat_data,
					   dev->of_node);
	if (ret) {
		goto err_plat;
	}

	if (iommu->plat_data->version !=
	    sunxi_iommu_read(iommu, IOMMU_VERSION_REG)) {
		dev_err(dev,
			"iommu version mismatch, please check and reconfigure\n");
		goto err_plat;
	}

	sunxi_iommu_sysfs_create(pdev, iommu);
	ret = iommu_device_sysfs_add(&iommu->iommu, dev, NULL,
				     dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to register iommu in sysfs\n");
		goto err_plat;
	}

	INIT_LIST_HEAD(&iommu->rsv_list);

	ret = iommu_device_register(&iommu->iommu, &sunxi_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		goto err_plat;
	}

	if (!dma_dev) {
		dma_dev = &pdev->dev;
		sunxi_pgtable_set_dma_dev(dma_dev);
	}

	return 0;

err_plat:
	if (iommu->plat_data) {
		for (i = 0; i < IOMMU_MIC_MAX_MASTER * IOMMU_HW_SET_COUNT;
		     i++) {
			if (iommu->plat_data->master[i])
				kfree(iommu->plat_data->master[i]);
		}
		kfree(iommu->plat_data);
	}
err_clk:
	for (; irq_got > 0; irq_got--) {
		devm_free_irq(dev, iommu->irq[irq_got - 1], iommu);
	}
err_irq:
	devm_iounmap(dev, iommu->base);
err_res:
	sunxi_pgtable_free_pte_cache(iopte_cache);
	dev_err(dev, "Failed to initialize\n");

	return ret;
}

static void sunxi_iommu_remove(struct platform_device *pdev)
{
	struct sunxi_iommu_dev *iommu = platform_get_drvdata(pdev);
	struct iommu_resv_region *entry, *next;
	int i;

	sunxi_pgtable_free_pte_cache(iopte_cache);
	if (!list_empty(&iommu->rsv_list)) {
		list_for_each_entry_safe(entry, next, &iommu->rsv_list,
					  list)
			kfree(entry);
	}

	for (i = 0; i < IOMMU_HW_SET_COUNT; i++) {
		devm_free_irq(iommu->dev, iommu->irq[i], iommu);
	}
	devm_iounmap(iommu->dev, iommu->base);
	sunxi_iommu_sysfs_remove(pdev);
	iommu_device_sysfs_remove(&iommu->iommu);
	iommu_device_unregister(&iommu->iommu);
	global_iommu_dev = NULL;
}

static int sunxi_iommu_suspend(struct device *dev)
{
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct clk **pclk = data->sunxi_iommu->clk;
	
	while (*pclk) {
		clk_disable_unprepare(*pclk);
		pclk++;
	}

	return 0;
}

static int sunxi_iommu_resume(struct device *dev)
{
	int err;
	struct sunxi_iommu_data *data = dev_iommu_priv_get(dev);
	struct clk **pclk = data->sunxi_iommu->clk;
	
	while (*pclk) {
		clk_prepare_enable(*pclk);
		pclk++;
	}

	if (unlikely(!data->sunxi_iommu->domain))
		return 0;

	err = sunxi_iommu_hw_init(data->sunxi_iommu->domain);

	return err;
}

const struct dev_pm_ops sunxi_iommu_pm_ops = {
	.suspend = sunxi_iommu_suspend,
	.resume = sunxi_iommu_resume,
};

static const struct of_device_id sunxi_iommu_dt_ids[] = {
	{ .compatible = "allwinner,iommu-v20" },
	{ /* sentinel */ },
};

static struct platform_driver
	sunxi_iommu_driver = { .probe = sunxi_iommu_probe,
			       .remove = sunxi_iommu_remove,
			       .driver = {
				       .name = "sunxi-iommu-v2",
				       .pm = &sunxi_iommu_pm_ops,
				       .of_match_table = sunxi_iommu_dt_ids,
			       } };

static int __init sunxi_iommu_init(void)
{
	return platform_driver_register(&sunxi_iommu_driver);
}

static void __exit sunxi_iommu_exit(void)
{
	return platform_driver_unregister(&sunxi_iommu_driver);
}

subsys_initcall(sunxi_iommu_init);
module_exit(sunxi_iommu_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.3.4");
MODULE_AUTHOR("ouayngkun<ouyangkun@allwinnertech.com>");
