/*
 * phal_generic_stm32f4x.h
 *
 *  Created on: Jan 15, 2024
 *      Author: monurparlak@gmail.com
 *  
 *  Purpose:
 *
 */

/*****************************************************************************/
/**< Includes */
/*****************************************************************************/

#include "phal_gpio_stm32f4x.h"

/*****************************************************************************/
/**< Function Definitions */
/*****************************************************************************/

void phal_gpio_clock_control(gpio_reg_def_t *p_gpiox, bool activite)
{
    /**< Do something */
}

void phal_gpio_init(gpio_handle_t *p_gpio_handle)
{
    /**< Do something */
}

void phal_gpio_deinit(gpio_reg_def_t *p_gpiox)
{
    /**< Do something */
}

uint8_t phal_gpio_read_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no)
{
    /**< Do something */
}

uint16_t phal_gpio_read_port(gpio_reg_def_t *p_gpiox)
{
    /**< Do something */
}

void phal_gpio_write_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no, uint8_t value)
{
    /**< Do something */
}

void phal_gpio_toggle_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no)
{
    /**< Do something */
}

void phal_gpio_write_port(gpio_reg_def_t *p_gpiox, uint8_t value)
{
    /**< Do something */
}

void phal_gpio_irq(uint8_t irq_no, uint8_t irq_priority, uint8_t activite)
{
    /**< Do something */
}

void phal_gpio_irq_handle(uint8_t pin_no);
{
    /**< Do something */
}
