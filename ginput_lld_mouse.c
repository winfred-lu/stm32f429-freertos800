/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

/**
 * @file    drivers/ginput/touch/MCU/ginput_lld_mouse.c
 * @brief   GINPUT Touch low level driver source for the MCU.
 *
 * @defgroup Mouse Mouse
 * @ingroup GINPUT
 *
 * @{
 */

#include "gfx.h"

#if (GFX_USE_GINPUT && GINPUT_NEED_MOUSE) /*|| defined(__DOXYGEN__)*/

#include "src/ginput/driver_mouse.h"

#include "stm32f429i_discovery_ioe.h"

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static inline void init_board(void) {
	IOE_Config();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initialise the mouse/touch.
 *
 * @notapi
 */
void ginput_lld_mouse_init(void) {
	init_board();
}

/**
 * @brief   Read the mouse/touch position.
 *
 * @param[in] pt	A pointer to the structure to fill
 *
 * @notapi
 */
void ginput_lld_mouse_get_reading(MouseReading *pt) {
	TP_STATE* tps;

	tps = IOE_TP_GetState();
	if (tps->TouchDetected) {
		pt->buttons = GINPUT_TOUCH_PRESSED;
		pt->x = tps->X;
		pt->y = tps->Y;
		pt->z = tps->Z;
	}
	else {
		pt->buttons = 0;
	}
}

#endif /* GFX_USE_GINPUT && GINPUT_NEED_MOUSE */
/** @} */
