/******************************************************************************
 * @file     main.c
 * @brief    Show how to read/program embedded flash by ISP function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


#if defined(__ARMCC_VERSION)
extern uint32_t Image$$RO$$Base;
#endif


void SYS_Init(void)
{
    int32_t i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for clock ready */
    i = 22000000; // For timeout
    while(i-- > 0)
    {
        if((CLK->STATUS & (CLK_STATUS_PLLSTB_Msk | CLK_STATUS_HXTSTB_Msk)) ==
                (CLK_STATUS_PLLSTB_Msk | CLK_STATUS_HXTSTB_Msk))
            break;
    }

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    uint8_t u8Ch;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART_Init();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0
            FMC_Boot0, RO=0x2000
            FMC_Boot1, RO=0x4000
            FMC_Boot2, RO=0x6000
            FMC_Boot3, RO=0x8000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\n\n");
    printf("+---------------------------------------------+\n");
    printf("|     Multi-Boot Sample Code(0x%08X)      |\n", FMC_GetVECMAP());
    printf("+---------------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Check if boot mode is "APROM with IAP" */
    if ((FMC_Read(FMC_CONFIG_BASE) & 0xC0) != 0x80)
    {
        printf("\n\nPlease set boot mode as [APROM with IAP]!\n");
        while (1);
    }

#if defined(__BASE__)
    printf("Boot from 0\n");
#endif
#if defined(__BOOT0__)
    printf("Boot from 0x2000\n");
#endif
#if defined(__BOOT1__)
    printf("Boot from 0x4000\n");
#endif
#if defined(__BOOT2__)
    printf("Boot from 0x6000\n");
#endif
#if defined(__BOOT3__)
    printf("Boot from 0x8000\n");
#endif
#if defined(__LDROM__)
    printf("Boot from 0x100000\n");
#endif

#if !defined(__ARMCC_VERSION)
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#else
    printf("Current RO Base = 0x%x, VECMAP = 0x%x\n", (uint32_t)&Image$$RO$$Base, FMC_GetVECMAP());
#endif

    printf("Select one boot image: \n");
    printf("[0] Boot 0, base = 0x2000\n");
    printf("[1] Boot 1, base = 0x4000\n");
    printf("[2] Boot 2, base = 0x6000\n");
    printf("[3] Boot 3, base = 0x8000\n");
#if defined(__ICCARM__) || defined(__ARMCC_VERSION)
    printf("[4] Boot 4, base = 0x100000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    u8Ch = getchar();

    switch (u8Ch)
    {
    case '0':
        FMC_SetVectorPageAddr(0x2000);
        break;

    case '1':
        FMC_SetVectorPageAddr(0x4000);
        break;

    case '2':
        FMC_SetVectorPageAddr(0x6000);
        break;

    case '3':
        FMC_SetVectorPageAddr(0x8000);
        break;

#if defined(__ICCARM__) || defined(__ARMCC_VERSION)
    case '4':
        FMC_SetVectorPageAddr(0x100000);
        break;
#endif

    default:
        FMC_SetVectorPageAddr(0x0);
        break;
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDone\n");

    while (SYS->PDID) __WFI();
}
