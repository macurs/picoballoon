/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v11.0
processor: K32L2B31xxxxA
package_id: K32L2B31VLH0A
mcu_data: ksdk2_0
processor_version: 12.0.0
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: f82b228f-a9a9-4506-a76e-962c6d170db1
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * SF_SPI initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SF_SPI'
- type: 'spi'
- mode: 'SPI_Polling'
- custom_name_enabled: 'true'
- type_id: 'spi_672b694426b0a10a1d774659ee8f8435'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'SPI1'
- config_sets:
  - fsl_spi:
    - spi_mode: 'kSPI_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - spi_master_config:
      - enableMaster: 'true'
      - enableStopInWaitMode: 'false'
      - polarity: 'kSPI_ClockPolarityActiveHigh'
      - phase: 'kSPI_ClockPhaseSecondEdge'
      - direction: 'kSPI_MsbFirst'
      - dataMode: 'kSPI_8BitMode'
      - txWatermark: 'kSPI_TxFifoOneHalfEmpty'
      - rxWatermark: 'kSPI_RxFifoOneHalfFull'
      - outputMode: 'kSPI_SlaveSelectAutomaticOutput'
      - pinMode: 'kSPI_PinModeNormal'
      - baudRate_Bps: '500000'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const spi_master_config_t SF_SPI_config = {
  .enableMaster = true,
  .enableStopInWaitMode = false,
  .polarity = kSPI_ClockPolarityActiveHigh,
  .phase = kSPI_ClockPhaseSecondEdge,
  .direction = kSPI_MsbFirst,
  .dataMode = kSPI_8BitMode,
  .txWatermark = kSPI_TxFifoOneHalfEmpty,
  .rxWatermark = kSPI_RxFifoOneHalfFull,
  .outputMode = kSPI_SlaveSelectAutomaticOutput,
  .pinMode = kSPI_PinModeNormal,
  .baudRate_Bps = 500000UL
};

static void SF_SPI_init(void) {
  /* Initialization function */
  SPI_MasterInit(SF_SPI_PERIPHERAL, &SF_SPI_config, SF_SPI_CLK_FREQ);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  SF_SPI_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
