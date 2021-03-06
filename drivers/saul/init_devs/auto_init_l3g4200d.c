/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of L3G4200D pressure sensors
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "assert.h"
#include "log.h"
#include "saul_reg.h"
#include "l3g4200d.h"
#include "l3g4200d_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define L3G4200D_NUM    ARRAY_SIZE(l3g4200d_params)

/**
 * @brief   Allocate memory for the device descriptors
 */
static l3g4200d_t l3g4200d_devs[L3G4200D_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[L3G4200D_NUM];

/**
 * @brief   Define the number of saul info
 */
#define L3G4200D_INFO_NUM    ARRAY_SIZE(l3g4200d_saul_info)

/**
 * @brief   Reference the driver struct
 */
extern saul_driver_t l3g4200d_saul_driver;

void auto_init_l3g4200d(void)
{
    assert(L3G4200D_NUM == L3G4200D_INFO_NUM);

    for (unsigned int i = 0; i < L3G4200D_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing l3g4200d #%u\n", i);

        if (l3g4200d_init(&l3g4200d_devs[i], &l3g4200d_params[i]) < 0) {
            LOG_ERROR("[auto_init_saul] error initializing l3g4200d #%u\n", i);
            continue;
        }

        saul_entries[i].dev = &(l3g4200d_devs[i]);
        saul_entries[i].name = l3g4200d_saul_info[i].name;
        saul_entries[i].driver = &l3g4200d_saul_driver;
        saul_reg_add(&(saul_entries[i]));
    }
}
