/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#include "gfx.h"

#if GFX_USE_GDISP

#if defined(GDISP_SCREEN_HEIGHT)
  #warning "GDISP: This low level driver does not support setting a screen size. It is being ignored."
  #undef GISP_SCREEN_HEIGHT
#endif
#if defined(GDISP_SCREEN_WIDTH)
  #warning "GDISP: This low level driver does not support setting a screen size. It is being ignored."
  #undef GDISP_SCREEN_WIDTH
#endif

#define GDISP_DRIVER_VMT      GDISPVMT_SSD1327
#include "gdisp_lld_config.h"
#include "src/gdisp/gdisp_driver.h"

#include "board_SSD1327.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#ifndef GDISP_SCREEN_HEIGHT
	#define GDISP_SCREEN_HEIGHT		96
#endif
#ifndef GDISP_SCREEN_WIDTH
	#define GDISP_SCREEN_WIDTH		128
#endif
#ifndef GDISP_INITIAL_CONTRAST
	#define GDISP_INITIAL_CONTRAST	100
#endif
#ifndef GDISP_INITIAL_BACKLIGHT
	#define GDISP_INITIAL_BACKLIGHT	10
#endif

#include "SSD1327.h"

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Some common routines and macros
#define dummy_read(g)				{ volatile uint16_t dummy; dummy = read_data(g); (void) dummy; }
#define write_reg(g, reg, data)		{ write_cmd(g, reg); write_data(g, data); }

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

LLDSPEC bool_t gdisp_lld_init(GDisplay *g) {
	// No private area for this controller
	g->priv = 0;

	// Initialise the board interface
	init_board(g);

	// Hardware reset
	setpin_reset(g, TRUE);
	gfxSleepMilliseconds(20);
	setpin_reset(g, FALSE);
	gfxSleepMilliseconds(20);

	// Get the bus for the following initialisation commands
	acquire_bus(g);

  write_cmd(g, SSD1327_MAST_CODE); //Master code
  write_data(g, 0x00);
  write_cmd(g, SSD1327_SET_SLEEP_ON);  // set display off

  write_cmd(g, SSD1327_SET_REMAP);
  write_data(g, 0x52); 
  write_cmd(g, SSD1327_SET_DISPLAY_START);
  write_data(g, 0x00);
  write_cmd(g, SSD1327_SET_DISPLAY_OFFSET);
  write_data(g, 0x20);
  write_cmd(g, SSD1327_SET_DISPLAY_MODE_RESET);

  write_cmd(g, SSD1327_SET_MUX_RATIO);
  write_data(g, 0x5F);
  write_cmd(g, SSD1327_SET_FUNCTION_SELECT);
  write_data(g, 0x01);
  write_cmd(g, SSD1327_SET_CONTRAST);
  write_data(g, 0x5F);
  write_cmd(g, SSD1327_USE_LINEAR_GREY);

  write_cmd(g, SSD1327_SET_PHASE_LENGTH);
  write_data(g, 0x31);
  write_cmd(g, SSD1327_CLOCKDIV_OSCFREQ);
  write_data(g, 0x41);
  write_cmd(g, SSD1327_DISPLAY_ENHANCEMENT);
  write_data(g, 0xB5);
  write_cmd(g, SSD1327_SET_SECOND_PRECHARGE);
  write_data(g, 0x04);
  write_cmd(g, SSD1327_SET_PRECHARGE_VOLTAGE);
  write_data(g, 0x05);
  write_cmd(g, SSD1327_SET_VCOMH);
  write_data(g, 0x07);
  write_cmd(g, SSD1327_SET_FUNCTION_SELECT_B);
  write_data(g, 0x02);

//i2c restart
  write_cmd(g, SSD1327_MAST_CODE); //Master code
  write_data(g, 0x00);
  write_cmd(g, SSD1327_SET_COLUMN_ADDRESS); //Master code
  write_data(g, 0x00); //COLUMN START
  write_data(g, 0x3F); //COLUMN END
  write_cmd(g, SSD1327_SET_ROW_ADDRESS); //Master code
  write_data(g, 0x00); //ROW START
  write_data(g, 0x7F); //ROW END

  write_cmd(g, SSD1327_MAST_CODE); //Master code
  write_data(g, 0x40);

	uint16_t i = 0;
	for (i = 0; i < 128*128; i++)
  {
    write_data(g, 0);
  }

	release_bus(g);

    // Finish Init
    post_init_board(g);

 	// Release the bus
//	release_bus(g);

	/* Turn on the back-light */
	set_backlight(g, GDISP_INITIAL_BACKLIGHT);

	/* Initialise the GDISP structure */
	g->g.Width = GDISP_SCREEN_WIDTH;
	g->g.Height = GDISP_SCREEN_HEIGHT;
	g->g.Orientation = GDISP_ROTATE_0;
	g->g.Powermode = powerOn;
	g->g.Backlight = GDISP_INITIAL_BACKLIGHT;
	g->g.Contrast = GDISP_INITIAL_CONTRAST;
	return TRUE;
}

#if GDISP_HARDWARE_STREAM_WRITE
	LLDSPEC	void gdisp_lld_write_start(GDisplay *g) {
		acquire_bus(g);
		write_cmd(g, SSD1327_SET_COLUMN_ADDRESS);
		write_data(g, g->p.x);
		write_data(g, g->p.x + g->p.cx - 1);
		write_cmd(g, SSD1327_SET_ROW_ADDRESS);
		write_data(g, g->p.y);
		write_data(g, g->p.y + g->p.cy - 1);
		write_cmd(g, SSD1327_WRITE_RAM);
	}
	LLDSPEC	void gdisp_lld_write_color(GDisplay *g) {
		LLDCOLOR_TYPE	c;

		c = gdispColor2Native(g->p.color);
		write_data(g, c >> 8);
		write_data(g, c & 0xFF);
	}
	LLDSPEC	void gdisp_lld_write_stop(GDisplay *g) {
		release_bus(g);
	}
#endif

#if GDISP_HARDWARE_CLEARS
  LLDSPEC void gdisp_lld_clear(GDisplay *g) {
    acquire_bus(g);
    write_cmd(g, SSD1327_SET_COLUMN_ADDRESS);
    write_data(g, 0);
    write_data(g, GDISP_SCREEN_WIDTH - 1);
    write_cmd(g, SSD1327_SET_ROW_ADDRESS);
    write_data(g, 0);
    write_data(g, GDISP_SCREEN_HEIGHT - 1);
    write_cmd(g, SSD1327_WRITE_RAM);
    LLDCOLOR_TYPE c;
    c = gdispColor2Native(g->p.color);
    uint16_t i = 0;
    for (i = 0; i < GDISP_SCREEN_WIDTH*GDISP_SCREEN_HEIGHT; i++)
    {
      write_data(g, c >> 8);
      write_data(g, c & 0xFF);
    }
    release_bus(g);
  }
#endif // GDISP_HARDWARE_CLEARS

#if GDISP_HARDWARE_FILLS
  LLDSPEC void gdisp_lld_fill_area(GDisplay *g) {
    coord_t x = g->p.x;
    coord_t y = g->p.y;
    coord_t w = g->p.cx;
    coord_t h = g->p.cy;
    LLDCOLOR_TYPE c = g->p.color;

    acquire_bus(g);
    write_cmd(g, SSD1327_SET_COLUMN_ADDRESS);
    write_data(g, x);
    write_data(g, x + w -1);
    write_cmd(g, SSD1327_SET_ROW_ADDRESS);
    write_data(g, y);
    write_data(g, y + h - 1);
    write_cmd(g, SSD1327_WRITE_RAM);
    uint16_t i = 0;
    for (i = 0; i < w*h; i++)
    {
      write_data(g, c >> 8);
      write_data(g, c & 0xFF);
    }
    release_bus(g);
  }
#endif // GDISP_HARDWARE_FILLS

#if GDISP_HARDWARE_STREAM_READ
	#error "SSD1327 - Stream Read is not supported yet"
	LLDSPEC	void gdisp_lld_read_start(GDisplay *g) {
		acquire_bus(g);
		//set_viewport(g);
		//write_index(g, 0x2E);
		setreadmode(g);
		//dummy_read(g);
	}
	LLDSPEC	color_t gdisp_lld_read_color(GDisplay *g) {
		uint16_t	data;

		data = read_data(g);
		return gdispNative2Color(data);
	}
	LLDSPEC	void gdisp_lld_read_stop(GDisplay *g) {
		setwritemode(g);
		release_bus(g);
	}
#endif

#if GDISP_NEED_CONTROL && GDISP_HARDWARE_CONTROL
//	#error "SSD1327 - Hardware control is not supported yet"
LLDSPEC void gdisp_lld_control(GDisplay *g) {
  switch(g->p.x) {
  case GDISP_CONTROL_POWER:
    if (g->g.Powermode == (powermode_t)g->p.ptr)
      return;
    switch((powermode_t)g->p.ptr) {
    case powerOff:
      acquire_bus(g);
      write_cmd(g, SSD1327_SET_DISPLAY_MODE_ALL_OFF);
      release_bus();
      break;
    case powerSleep:
    case powerDeepSleep:
      acquire_bus(g);
      write_cmd(g, SSD1327_SET_SLEEP_ON);
      release_bus(g);
      break;
    case powerOn:
      acquire_bus(g);
      write_cmd(g, SSD1327_SET_SLEEP_OFF);
      write_cmd(g, SSD1327_SET_DISPLAY_MODE_ALL_ON);
      release_bus(g);
      break;
    default:
      return;
    }
    g->g.Powermode = (powermode_t)g->p.ptr;
    return;

  case GDISP_CONTROL_ORIENTATION:
    if (g->g.Orientation == (orientation_t)g->p.ptr)
      return;
    switch((orientation_t)g->p.ptr) {
    case GDISP_ROTATE_0: // correct
      acquire_bus(g);
      write_reg(g, SSD1327_SET_REMAP, (0b01<<6) | (1<<5) | (1<<4) | (1<<2) | 0b00);
      // bits:
      // [0] : address increment (0: horizontal, 1: vertical, reset 0)
      // [1] : column remap (0: 0..127, 1: 127..0, reset 0)
      // [2] : color remap (0: A->B->C, 1: C->B->A, reset 0)
      // [3] : reserved
      // [4] : column scan direction (0: top->down, 1: bottom->up, reset 0)
      // [5] : odd/even split COM (0: disable, 1: enable, reset 1)
      // [6..7] : color depth (00,01: 65k, 10: 262k, 11: 262k format 2)
      write_cmd(g, SSD1327_WRITE_RAM);
      g->g.Height = GDISP_SCREEN_HEIGHT;
      g->g.Width = GDISP_SCREEN_WIDTH;
      release_bus(g);
      break;
    case GDISP_ROTATE_90:
      acquire_bus(g);
      write_reg(g, SSD1327_SET_REMAP, (0b01<<6) | (1<<5) | (1<<4) | (1<<2) | 0b10);
      write_cmd(g, SSD1327_WRITE_RAM);
      g->g.Height = GDISP_SCREEN_WIDTH;
      g->g.Width = GDISP_SCREEN_HEIGHT;
      release_bus(g);
      break;
    case GDISP_ROTATE_180:
      acquire_bus(g);
      write_reg(g, SSD1327_SET_REMAP, (0b01<<6) | (1<<5) | (0<<4) | (1<<2) | 0b00);
      write_reg(g, SSD1327_SET_REMAP, 0b01100110);
      write_cmd(g, SSD1327_WRITE_RAM);
      g->g.Height = GDISP_SCREEN_HEIGHT;
      g->g.Width = GDISP_SCREEN_WIDTH;
      release_bus(g);
      break;
    case GDISP_ROTATE_270:
      acquire_bus(g);
      write_reg(g, SSD1327_SET_REMAP, (0b01<<6) | (1<<5) | (0<<4) | (1<<2) | 0b01);
      write_cmd(g, SSD1327_WRITE_RAM);
      g->g.Height = GDISP_SCREEN_WIDTH;
      g->g.Width = GDISP_SCREEN_HEIGHT;
      release_bus(g);
      break;
    default:
      return;
    }
    g->g.Orientation = (orientation_t)g->p.ptr;
    return;

  case GDISP_CONTROL_BACKLIGHT:
      if ((unsigned)g->p.ptr > 100)
        g->p.ptr = (void *)100;
      set_backlight(g, (unsigned)g->p.ptr);
      g->g.Backlight = (unsigned)g->p.ptr;
      return;

  //case GDISP_CONTROL_CONTRAST:
      default:
          return;
  }
}
#endif

#endif /* GFX_USE_GDISP */
