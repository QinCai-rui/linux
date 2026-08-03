/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * sunxi iommu: main structures
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include "sunxi-iommu-pgtable.h"

#include <linux/iommu.h>
/*
 * by design iommu driver should be part of iommu
 * and get to it by ../../dma-iommu.h
 * sunxi bsp have seperate root, use different path
 * to reach dma-iommu.h
 */
#include <../drivers/iommu/dma-iommu.h>

#define MAX_SG_SIZE (128 << 20)
#define MAX_SG_TABLE_SIZE ((MAX_SG_SIZE / SPAGE_SIZE) * sizeof(u32))
#define DUMP_REGION_MAP 0
#define DUMP_REGION_RESERVE 1
struct dump_region {
	u32 access_mask;
	size_t size;
	u32 type;
	dma_addr_t phys, iova;
};
struct sunxi_iommu_dev;


int sunxi_iommu_check_cmd(struct device *dev, void *data);
u32 sunxi_iommu_dump_rsv_list(struct list_head *rsv_list, ssize_t len,
			      char *buf, size_t buf_len, bool for_sysfs_show);
int iova_show_on_irq(void);
void sunxi_iommu_register_vendorhook(void);
void sunxi_iommu_init_debugfs(struct sunxi_iommu_dev *sunxi_iommu);
void sunxi_iommu_release_debugfs(void);
