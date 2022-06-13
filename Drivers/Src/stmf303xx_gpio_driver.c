

#include "stm32f303xx_gpio_driver.h"


/*Peripheral Clock Setup
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx  == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx  == GPIOB){
			GPIOB_PCLK_EN();
	    }
		else if(pGPIOx  == GPIOC){
			GPIOC_PCLK_EN();
	    }
		else if(pGPIOx  == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx  == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx  == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx  == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx  == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx  == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx  == GPIOB){
			GPIOB_PCLK_DI();
	    }
		else if(pGPIOx  == GPIOC){
			GPIOC_PCLK_DI();
	    }
		else if(pGPIOx  == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx  == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx  == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx  == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx  == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

/* Init and Deinit
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	//Configure the mode of GPIO PIN
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=   GPIO_MODE_ANALOG){
    //the non-interrupt mode
    	temp= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    	pGPIOHandle->pGPIOx->MODER  &= ~( 0x3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    	pGPIOHandle->pGPIOx->MODER |=temp;
    	temp=0;
    }
    else{
    	//interrupt mode
    }

	//Configure the Speed
     temp=0;
     temp= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OSPEEDR |=temp;
     temp=0;
	//Configure the pupd settings

     temp= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->PUPDR |=temp;

     temp=0;

	//configure the optype
     temp= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OTYPER |=temp;

     //configure the alternate functionality
     if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==   GPIO_MODE_AF){

    	 uint8_t temp1=  ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8);
    	 uint8_t temp2=   ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8);
    	 pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 *temp2));
    	 pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
     }
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	     if(pGPIOx  == GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx  == GPIOB){
				GPIOB_REG_RESET();
		    }
			else if(pGPIOx  == GPIOC){
				GPIOC_REG_RESET();
		    }
			else if(pGPIOx  == GPIOD){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx  == GPIOE){
				GPIOE_REG_RESET();
			}
			else if(pGPIOx  == GPIOF){
				GPIOF_REG_RESET();
			}
			else if(pGPIOx  == GPIOG){
				GPIOG_REG_RESET();
			}
			else if(pGPIOx  == GPIOH){
				GPIOH_REG_RESET();
			}


}







/*Data Read and Write
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber );
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and ISR handling
 *
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
