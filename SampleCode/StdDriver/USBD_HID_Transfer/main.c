/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_transfer.h"

#define CRYSTAL_LESS        1

#define TRIM_INIT           (GCR_BASE+0x12C)
/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#if (!CRYSTAL_LESS)
    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));
#else
    /* Set core clock */
    CLK_SetCoreClock(72000000);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBCKSEL_Msk;

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPD multi-function pins for UART0 RXD and TXD, and Clock Output */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /*
        This sample code demonstrate how to use HID interface to transfer data
        between PC and USB device.
        A demo window tool is also included in "WindowsTool" directory with this
        sample code. User can use it to test data transfer with this sample code.
    */

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();

    /* Init UART for debug message */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Transfer Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /*Init Endpoint configuration for HID */
    HID_Init();

    /* Start USB device */
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
#endif

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    while(SYS->PDID)
    {
#if CRYSTAL_LESS
        /* Re-start crystal-less when any error found */
        if (SYS->IRC48MTISTS & (SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk))
        {
            SYS->IRC48MTISTS = SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk;

            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Waiting for USB bus stable */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
            while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

            /* Re-enable crystal-less */
            SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
            //printf("USB trim fail. Just retry. SYS->HIRCTSTS = 0x%x, SYS->HIRCTCTL = 0x%x\n", SYS->HIRCTSTS, SYS->HIRCTCTL);
        }
#endif
    }
}





