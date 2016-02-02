#ifndef SSD1327_H
#define SSD1327_H

#define SSD1327_SET_COLUMN_ADDRESS              0x15
#define SSD1327_SET_ROW_ADDRESS                 0x75
//#define SSD1327_WRITE_RAM                       	0x5C
//#define SSD1327_READ_RAM                        	0x5D
#define SSD1327_SET_REMAP                       0xA0
	
#define SSD1327_SET_DISPLAY_START               0xA1
#define SSD1327_SET_DISPLAY_OFFSET              0xA2
#define SSD1327_SET_DISPLAY_MODE_ALL_OFF        0xA6
#define SSD1327_SET_DISPLAY_MODE_ALL_ON         0xA5
#define SSD1327_SET_DISPLAY_MODE_RESET          0xA4
#define SSD1327_SET_DISPLAY_MODE_INVERT         0xA7
#define SSD1327_SET_FUNCTION_SELECT             0xAB
#define SSD1327_SET_SLEEP_ON                    0xAE
#define SSD1327_SET_SLEEP_OFF                   0xAF
#define SSD1327_SET_PRECHARGE_VOLTAGE           0xBC
	
#define SSD1327_DISPLAY_ENHANCEMENT             0xB4
#define SSD1327_CLOCKDIV_OSCFREQ                0xB3
//#define SSD1327_SET_VSL                         	0xB4
#define SSD1327_SET_GPIO                        0xB5
#define SSD1327_SET_SECOND_PRECHARGE            0xB6
	
#define SSD1327_LUT_GRAYSCALE                   0xB8
#define SSD1327_USE_LINEAR_GREY                 0xB9
#define SSD1327_SET_PRECHARGE                 	0xBB
#define SSD1327_SET_VCOMH                       0xBE
	
//#define SSD1327_MASTER_CONTRAST_CURRENT_CONTROL 	0xC7
#define SSD1327_SET_MUX_RATIO                   0xA8
#define SSD1327_SET_COMMAND_LOCK                0xFD

#define SSD1327_SET_FUNCTION_SELECT_B           0xD5

#define SSD1327_MAST_CODE						0x78
#define SSD1327_SET_CONTRAST                    0x81
#define SSD1327_SET_PHASE_LENGTH                0xB1

#endif // SSD1327_H
