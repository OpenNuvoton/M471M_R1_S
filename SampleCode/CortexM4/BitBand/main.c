/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate the usage of Cortex-M4 BitBand.
 *
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/* Memory Address */
#define MEM_ADDR(address)       *((volatile unsigned long *) (address))
/* BitBand Address */
#define BITBAND(address,bitnum) ((address & 0xF0000000)+0x02000000+((address & 0xFFFFF)<<5)+(bitnum<<2))

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}

void UART0_Init()
{
    UART_Open(UART0, 115200);
}

void BitBand_Test(void)
{
    uint32_t u32Temp;
    uint32_t u32TestAddress;

    u32TestAddress = 0x20003F00;

    //Read SRAM Address(0x20003F00)
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at test address (0x20003F00) is: 0x%x", u32Temp);

    //Use BitBand function to read SRAM bit value
    u32Temp = MEM_ADDR(BITBAND(u32TestAddress, 3));
    printf("\n Use Bit-Band function to get value at bit 3: %x \n", u32Temp);

    //Use BitBand function set bit
    printf("\n Use Bit-Band function set test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 1;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);

    //Use BitBand function clear bit
    printf("\n Use Bit-Band function clear test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 0;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);
}

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n Start Bit-Band test: \n");

    BitBand_Test();

    while(1);
}

