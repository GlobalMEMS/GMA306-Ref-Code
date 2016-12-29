/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Usage: GMA306 Sensor Driver Test
 *
 ****************************************************************************
 * @section License
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/
 
/*! @file sensor_driver_test.c
 *  @brief  GMA306 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include "gma306.h"
#include "gSensor_autoNil.h"

#define DELAY_MS(ms)	//.....     /* Add your time delay function here */

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  u8 c;
  bus_support_t gma306_bus;
  raw_data_xyzt_t rawData;
  raw_data_xyzt_t offsetData;

  /* Add your HW initialization code here
    ...
    ...
    ...
    ...
  */
	
  /* GMA306 I2C bus setup */
  bus_init_I2C(&gma306_bus, GMA306_7BIT_I2C_ADDR);
  gma306_bus_init(&gma306_bus);

  /* GMA306 soft reset */
  gma306_soft_reset();

  /* GMA306 initialization */
  gma306_initialization();
	
  /* GMA306 Offset AutoNil */
  printf("Place and hold g-sensor in level for offset AutoNil.\n"); 
  printf("Press y when ready.\n");
  do{
    c = getchar();
  }while(c != 'y' && c != 'Y');

  //Conduct g-sensor AutoNil, g is along the Z-axis
  gSensorAutoNil(gma306_read_data_xyz, AUTONIL_AUTO + AUTONIL_Z, GMA306_RAW_DATA_SENSITIVITY, &offsetData);

  printf("Offset_XYZ=%d,%d%d\n", offsetData.u.x, offsetData.u.y, offsetData.u.z);
	
  for(;;)
    {

      /* Read XYZT data */
      gma306_read_data_xyzt(&rawData);

      printf("Raw_XYZT=%d,%d,%d,%d\n", rawData.u.x, rawData.u.y, rawData.u.z, rawData.u.t);
			
      printf("Calib_XYZ=%d,%d,%d\n", rawData.u.x - offsetData.u.x, rawData.u.y - offsetData.u.y, rawData.u.z - offsetData.u.z);

      /* Delay 1 sec */
      DELAY_MS(1000);
		
    }
}
