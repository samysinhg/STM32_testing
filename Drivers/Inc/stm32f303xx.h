/*
 * stm32f303xx.h
 *
 *  Created on: Jun 8, 2022
 *      Author: shammik
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include<stdint.h>
#define _vo  volatile

//BASE ADDRESSES OF MEMORIES


#define FLASH_BASEADDR    0X08000000U
#define RAM_BASEADDR     0X20000000U
#define CCMSRAM_BASEADDR     0X10000000U
#define ROM_BASEADDR      0X1FFFD800U


//BASES ADDRESSES OF PERIPHERAL BASE ADDRESSES
#define AHB4PERIPH_BASEADDR     0x60000000U
#define AHB3PERIPH_BASEADDR     0X50000000U
#define AHB2PERIPH_BASEADDR     0X48000000U
#define AHB1PERIPH_BASEADDR     0X40020000U
#define APB2PERIPH_BASEADDR     0X40010000U
#define APB1PERIPH_BASEADDR     0X40000000U

//AHB3 PERIPHERAL ADDRESSES
#define ADC1_BASEADDR         (AHB3PERIPH_BASEADDR + 0x000)
#define ADC2_BASEADDR         (AHB3PERIPH_BASEADDR + 0x100)
#define ADC3_BASEADDR         (AHB3PERIPH_BASEADDR + 0x04C)
#define ADC4_BASEADDR         (AHB3PERIPH_BASEADDR + 0x14C)

//AHB2 PERIPHERAl ADRESSES
#define GPIOA_BASEADDR         (AHB2PERIPH_BASEADDR)
#define GPIOB_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x0400)
#define GPIOC_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x0800)
#define GPIOD_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x0C00)
#define GPIOE_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x1000)
#define GPIOF_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x1400)
#define GPIOG_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x1800)
#define GPIOH_BASEADDR         (AHB2PERIPH_BASEADDR  + 0x1C00)

//AHB1 PERIPHERAL ADDRESS
#define DMA1_BASEADDR           (AHB1PERIPH_BASEADDR)
#define DMA2_BASEADDR           (AHB1PERIPH_BASEADDR +  0x0400)
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR +  0x1000)
#define FLASHINTERFCE_BASEADDR  (AHB1PERIPH_BASEADDR + 0x2000)
#define CRC_BASEADDR            (AHB1PERIPH_BASEADDR +  0x3000)
#define TSC_BASEADDR            (AHB1PERIPH_BASEADDR +  0x4000)

//APB2PERIPHERAL ADDRESS
#define SYSCFG_COMP_OPAMP_BASEADDR       (APB2PERIPH_BASEADDR)
#define EXTI_BASEADDR                    (APB2PERIPH_BASEADDR + 0x0400)
#define TIM1_BASEADDR                    (APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR                    (APB2PERIPH_BASEADDR + 0x3000)
#define TIM8_BASEADDR                    (APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR                  (APB2PERIPH_BASEADDR + 0x3800)
#define SPI4_BASEADDR                    (APB2PERIPH_BASEADDR + 0x3C00)
#define TIM15_BASEADDR                   (APB2PERIPH_BASEADDR + 0x4000)
#define TIM16_BASEADDR                   (APB2PERIPH_BASEADDR + 0x4400)
#define TIM17_BASEADDR                   (APB2PERIPH_BASEADDR + 0x4800)
#define TIM20_BASEADDR                   (APB2PERIPH_BASEADDR + 0x5000)

//APB1PERIPHERAL BASE ADDRESS

#define  TIM2_BASEADDR                   (APB1PERIPH_BASEADDR)
#define  TIM3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0400)
#define  TIM4_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0800)
#define  TIM6_BASEADDR                   (APB1PERIPH_BASEADDR + 0x1000)
#define  TIM7_BASEADDR                   (APB1PERIPH_BASEADDR + 0x1400)
#define  RTC_BASEADDR                    (APB1PERIPH_BASEADDR + 0x2800)
#define  WWDG_BASEADDR                   (APB1PERIPH_BASEADDR + 0x2C00)
#define  IWDG_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3000)
#define  I2S2EXT_BASEADDR                (APB1PERIPH_BASEADDR + 0x3400)
#define  SPI2_I2S2_BASEADDR              (APB1PERIPH_BASEADDR + 0x3800)
#define  SPI3_I2S3_BASEADDR              (APB1PERIPH_BASEADDR + 0x3C00)
#define  I2S3EXT_BASEADDR                (APB1PERIPH_BASEADDR + 0x4000)
#define  USART2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400)
#define  USART3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800)
#define  UART4_BASEADDR                  (APB1PERIPH_BASEADDR + 0x4C00)
#define  UART5_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5000)
#define  I2C1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5400)
#define  I2C2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5800)
#define  USB_DEVICE_FS_BASEADDR          (APB1PERIPH_BASEADDR + 0x5C00)
#define  USB_CAN_SRAM                    (APB1PERIPH_BASEADDR + 0x6000)
#define  BXCAN_BASEADDR                  (APB1PERIPH_BASEADDR + 0x6400)
#define  PWR_BASEADDR                    (APB1PERIPH_BASEADDR + 0x7000)
#define  DAC1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x7400)
#define  I2C3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x7800)

/**********************************register definition*****************************************************/



typedef struct {
        _vo	uint32_t   MODER;
		_vo	uint32_t   OTYPER;
		_vo	uint32_t   OSPEEDR;
		_vo	uint32_t   PUPDR;
		_vo	uint32_t   IDR;
		_vo	uint32_t   ODR;
		_vo	uint32_t   BSRR;
		_vo	uint32_t   LCKR;
		_vo	uint32_t   AFRL;
		_vo	uint32_t   AFRH;
		_vo uint32_t  BRR;

}GPIO_RegDef_t;

typedef struct{
	_vo uint32_t CR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t  APB2RSTR;
	_vo uint32_t  APB1RSTR;
	_vo uint32_t AHBENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t APB1ENR;
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	_vo uint32_t AHBRSTR;
	_vo uint32_t CFGR2;
	_vo uint32_t CFGR3;
}RCC_RegDef_t;

#define  GPIOA     ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define  GPIOB     ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define  GPIOC     ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define  GPIOD     ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define  GPIOE     ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define  GPIOF     ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define  GPIOG     ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define  GPIOH     ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define   RCC      ((GPIO_RegDef_t *)RCC_BASEADDR)


//Clock Enable Macros for GPIOx Peripherals

#define GPIOA_PCLK_EN()     (RCC->AHBENR  |= (1<<17))
#define GPIOB_PCLK_EN()     (RCC->AHBENR  |= (1<<18))
#define GPIOC_PCLK_EN()     (RCC->AHBENR  |= (1<<19))
#define GPIOD_PCLK_EN()     (RCC->AHBENR  |= (1<<20))
#define GPIOE_PCLK_EN()     (RCC->AHBENR  |= (1<<21))
#define GPIOF_PCLK_EN()     (RCC->AHBENR  |= (1<<22))
#define GPIOG_PCLK_EN()     (RCC->AHBENR  |= (1<<23))
#define GPIOH_PCLK_EN()     (RCC->AHBENR  |= (1<<16))


//Clock Enable Macros for I2Cx Peripherals

#define I2C1_PCLK_EN()      ( RCC->APB1ENR |=(1 << 21))
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |=(1 << 22))
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |=(1 << 30))

//CLOCK ENABLE MACROS for SPIx Peripherals

#define SPI1_PCLK_EN()      ( RCC->APB2ENR |=(1 << 12))
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |=(1 << 14))
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |=(1 << 15))
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |=(1 << 15))


//some macros
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET

//Macros to disable GPIOx

#define GPIOA_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<17));   (RCC->AHBRSTR  &= ~(1<<17));} while(0)
#define GPIOB_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<18));   (RCC->AHBRSTR  &= ~(1<<18));} while(0)
#define GPIOC_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<19));   (RCC->AHBRSTR  &= ~(1<<19));} while(0)
#define GPIOD_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<20));   (RCC->AHBRSTR  &= ~(1<<20));} while(0)
#define GPIOE_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<21));   (RCC->AHBRSTR  &= ~(1<<21));} while(0)
#define GPIOF_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<22));   (RCC->AHBRSTR  &= ~(1<<22));} while(0)
#define GPIOG_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<23));   (RCC->AHBRSTR  &= ~(1<<23));} while(0)
#define GPIOH_REG_RESET()           do{(RCC->AHBRSTR  |= (1<<16));   (RCC->AHBRSTR  &= ~(1<<16));} while(0)

#endif /* INC_STM32F303XX_H_ */
