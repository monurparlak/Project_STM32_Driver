/*
 * phal_gpio_stm32f4x.h
 *
 *  Created on: Jan 15, 2024
 *      Author: monurparlak@gmail.com
 *
 *  Purpose:
 *
 */

#ifndef INC_PHAL_GPIO_STM32F4X_H_
#define INC_PHAL_GPIO_STM32F4X_H_

/*****************************************************************************/
/**< Includes */
/*****************************************************************************/

#include "phal_generic_stm32f4x.h"

/*****************************************************************************/
/**< Definitions */
/*****************************************************************************/

/**
 * @brief GPIO pin configuration for individual GPIO pins
 */
#define GPIO_PIN_0       0 
#define GPIO_PIN_1       1 
#define GPIO_PIN_2       2 
#define GPIO_PIN_3       3 
#define GPIO_PIN_4       4 
#define GPIO_PIN_5       5 
#define GPIO_PIN_6       6 
#define GPIO_PIN_7       7 
#define GPIO_PIN_8       8 
#define GPIO_PIN_9       9 
#define GPIO_PIN_11      11
#define GPIO_PIN_12      12
#define GPIO_PIN_13      13
#define GPIO_PIN_14      14
#define GPIO_PIN_15      15

/**
 * @brief GPIO modes configuration for individual GPIO pins
 */
#define GPIO_MODE_IN	        0
#define GPIO_MODE_OUT	        1
#define	GPIO_MODE_AF	        2
#define GPIO_MODE_AN	        3
#define GPIO_MODE_IT_FALLING    4
#define GPIO_MODE_IT_RISING     5
#define GPIO_MODE_IT_BOTH       6

/**
 * @brief GPIO output types configuration for individual GPIO pins
 */
#define GPIO_OPTYPE_PP  0
#define GPIO_OPTYPE_OD  1

/**
 * @brief GPIO speed configuration for individual GPIO pins
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_VERY_HIGH 3

/**
 * @brief GPIO pull-up/pull-down configuration for individual GPIO pins
 */
#define GPIO_NOPULL     0
#define GPIO_PULLUP     1
#define GPIO_PULLDOWN   2

/*****************************************************************************/
/**< Typedef Structures */
/*****************************************************************************/

/**
 * @brief Configuration structure for individual GPIO pins
 */
typedef struct {
    uint8_t gpio_pin_no;        /**< GPIO Pin Number */
    uint8_t gpio_pin_mode;      /**< GPIO Pin Mode (e.g., Input, Output, Alternate Function, etc.) */
    uint8_t gpio_pin_speed;     /**< GPIO Pin Speed (e.g., Low, Medium, High) */
    uint8_t gpio_pin_pupd;      /**< GPIO Pin Pull-up/Pull-down Configuration */
    uint8_t gpio_pin_optype;    /**< GPIO Pin Output Type (e.g., Push-Pull, Open-Drain) */
    uint8_t gpio_pin_alt_func;  /**< GPIO Alternate Function for the pin (if used) */
    
} gpio_pin_config_t;

/**
 * @brief GPIO handle structure
 */
typedef struct {
    gpio_reg_def_t *p_gpio_portx;       /**< Pointer to the GPIO port register definition */
    gpio_pin_config_t gpio_pin_config;  /**< Configuration for the specific GPIO pin */

} gpio_handle_t;

/*****************************************************************************/
/**< Macros */
/*****************************************************************************/


/*****************************************************************************/
/**< Function Prototypes */
/*****************************************************************************/

/** 
 * @brief Controls GPIO clock
 * Activates or deactivates clock for a specific GPIO module.
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module.
 *                (For example: GPIOA, GPIOB, etc.)
 *
 * @param activate: If true, enables the clock for the GPIO module;
 *                  If false, disables it.
 *
 * @return: None
 *
 */
void phal_gpio_clock_control(gpio_reg_def_t *p_gpiox, bool activite);

/** 
 * @brief Initializes the GPIO
 * Configures the GPIO pins based on the provided handle.
 *
 * @param p_gpio_handle: Pointer to the GPIO handle structure containing
 *                       configuration parameters for GPIO initialization.
 *
 * @return: None
 *
 */
void phal_gpio_init(gpio_handle_t *p_gpio_handle);

/** 
 * @brief Deinitializes the GPIO
 * Resets the GPIO configuration for the specified GPIO module.
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 to be deinitialized (e.g., GPIOA, GPIOB, etc.).
 *
 * @return: None
 *
 */
void phal_gpio_deinit(gpio_reg_def_t *p_gpiox);

/** 
 * @brief Reads the state of a specific GPIO pin
 * Retrieves and returns the current logical state (high or low) of the
 * specified GPIO pin in the given GPIO module.
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 containing the target pin (e.g., GPIOA, GPIOB, etc.).
 *
 * @param pin_no:  The pin number of the GPIO module to be read.
 *
 * @return: The logical state of the specified GPIO pin (1 for high, 0 for low).
 *
 */
uint8_t phal_gpio_read_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no);

/** 
 * @brief Reads the state of all GPIO pins in a GPIO port
 * Retrieves and returns the current logical states (high or low) of all
 * pins in the specified GPIO module as a 16-bit value.
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 containing the target port (e.g., GPIOA, GPIOB, etc.).
 *
 * @return: The 16-bit value representing the logical states of all GPIO pins
 *          in the specified GPIO port.
 *
 */
uint16_t phal_gpio_read_port(gpio_reg_def_t *p_gpiox);

/** 
 * @brief Writes a specific value to a GPIO pin
 * Sets or clears the specified GPIO pin in the given GPIO module based on
 * the provided value (1 for high, 0 for low).
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 containing the target pin (e.g., GPIOA, GPIOB, etc.).
 *
 * @param pin_no:  The pin number of the GPIO module to be written.
 *
 * @param value:   The value to be written to the specified GPIO pin
 *                 (1 for high, 0 for low).
 *
 * @return: None
 *
 */
void phal_gpio_write_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no, uint8_t value);

/** 
 * @brief Toggles the state of a specific GPIO pin
 * Inverts the current logical state (high to low or low to high) of
 * the specified GPIO pin in the given GPIO module.
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 containing the target pin (e.g., GPIOA, GPIOB, etc.).
 *
 * @param pin_no:  The pin number of the GPIO module to be toggled.
 *
 * @return: None
 *
 */
void phal_gpio_toggle_pin(gpio_reg_def_t *p_gpiox, uint8_t pin_no);

/** 
 * @brief Writes a specific value to all GPIO pins in a GPIO port
 * Sets or clears all pins in the specified GPIO module based on the provided value
 * (1 for high, 0 for low).
 *
 * @param p_gpiox: Pointer to the register structures of the GPIO module
 *                 containing the target port (e.g., GPIOA, GPIOB, etc.).
 *
 * @param value:   The value to be written to all GPIO pins in the specified port
 *                 (1 for high, 0 for low).
 *
 * @return: None
 *
 */
void phal_gpio_write_port(gpio_reg_def_t *p_gpiox, uint8_t value);

/** 
 * @brief Configures GPIO interrupt
 * Configures the specified GPIO interrupt with the given parameters.
 *
 * @param irq_no:       The IRQ number associated with the GPIO interrupt.
 *
 * @param irq_priority: The priority level of the GPIO interrupt.
 *
 * @param activite:     The trigger edge of the GPIO interrupt (rising, falling, or both).
 *
 * @return: None
 *
 */
void phal_gpio_irq(uint8_t irq_no, uint8_t irq_priority, uint8_t activite);

/** 
 * @brief Handles GPIO interrupt
 * Handles the GPIO interrupt associated with the specified pin.
 *
 * @param pin_no: The pin number of the GPIO module associated with the interrupt.
 *
 * @return: None
 *
 */
void phal_gpio_irq_handle(uint8_t pin_no);

#endif /* INC_PHAL_GPIO_STM32F4X_H_ */
