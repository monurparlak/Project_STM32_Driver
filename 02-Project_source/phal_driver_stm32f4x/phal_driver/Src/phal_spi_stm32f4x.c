/*
 * phal_spi_stm32f4x.c
 *
 *  Created on: Jan 20, 2024
 *      Author: monurparlak@gmail.com
 *
 *  Purpose:
 *
 */

#include "phal_spi_stm32f4x.h"

void phal_spi_clock_control(spi_reg_def_t *p_spix, bool activite)
{
	/* TODO: Do Something */
}

void phal_spi_init(spi_handle_t *p_spi_handle)
{
	/* TODO: Do Something */
}

void phal_spi_deinit(spi_reg_def_t *p_spix)
{
	/* TODO: Do Something */
}

void phal_spi_send_data(spi_reg_def_t *p_spix, uint8_t *p_tx_buffer, uint32_t len)
{
	/* TODO: Do Something */
}

void phal_spi_receive(spi_reg_def_t *p_spix, uint8_t *p_rx_buffer, uint32_t len)
{
	/* TODO: Do Something */
}

void phal_spi_irq_interrupt(uint8_t irq_no, uint8_t activite)
{
	/* TODO: Do Something */
}

void phal_spi_irq_priority(uint8_t irq_no, uint32_t irq_priority)
{
	/* TODO: Do Something */
}

void phal_spi_irq_handle(uint8_t pin_no)
{
	/* TODO: Do Something */
}


