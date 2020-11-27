/***************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU( Device Firmware Upgrade) class.
 *           A Windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "fmc_user.h"
#include "dfu_transfer.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HIRC
#define PLL_CLOCK           72000000

#define V6M_AIRCR_VECTKEY_DATA    0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ     0x00000004UL

#define DetectPin   PD0

uint32_t g_apromSize;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCTL = (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    CLK->PWRCTL |= CLK_PWRCTL_HIRC48MEN_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBCKSEL_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t uData;

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-funcition I/O */
    SYS_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_ISPFF_Msk;

    /* Get APROM Size */
    if (FMC_Read_User(64 * 1024, &uData) < 0)
    {
        g_apromSize = 64 * 1024;
    }
    else
    {
        g_apromSize = 128 * 1024;
    }

    /* Open USB controller */
    USBD_Open(&gsInfo, DFU_ClassRequest, NULL);

    /*Init Endpoint configuration for DFU */
    DFU_Init();

    /* Start USB device */
    USBD_Start();

    /* polling USBD interrupt flag */
    while (DetectPin == 0)
    {
        USBD_IRQHandler();
    }

    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);//clear bit
    FMC->ISPCTL &=  ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
