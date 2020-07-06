/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;

    if (FMC_Read_User(64 * 1024, &uData) < 0)
    {
        g_apromSize = 64 * 1024;
    }
    else
    {
        g_apromSize = 128 * 1024;
    }

    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0)   //DFEN enable
    {
        FMC_Read_User(Config1, &uData);

        // filter the reserved bits in CONFIG1
        uData &= 0x000FFFFF;

        if ((uData > g_apromSize) || (uData & FMC_FLASH_PAGE_SIZE - 1))   //avoid config1 value from error
        {
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    }
    else
    {
        *addr = g_apromSize;
        *size = 0;
    }
}
