/**
 * @file conf_promatix.h
 * @author Agustin Allegui
 * @brief Configuraciones de aplicacion de promatix
 * @version 0.1
 * @date 2020-10-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CONF_PROMATIX_H
#define CONF_PROMATIX_H

#define PROMATIX_DEV_EUI {0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}
#define PROMATIX_APP_EUI {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}
#define PROMATIX_APP_KEY {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}
#define PROMATIX_SUB_BAND 2



#define PROMATIX_START_HOUR 6
#define PROMATIX_TIMES_PER_DAY 1

#define PROMATIX_INITIAL_PERIOD (PROMATIX_START_HOUR<<8)+PROMATIX_TIMES_PER_DAY

#endif