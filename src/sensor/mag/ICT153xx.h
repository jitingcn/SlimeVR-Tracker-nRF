#ifndef ICT153XX_H
#define ICT153XX_H

#include "sensor/sensor.h"

/* ICT-15312 / ICT-15318, DS-000548 revision 1.0, sections 5.6 and 7. */
#define ICT153XX_I2C_ADDR 0x1E
#define ICT153XX_MANU_ID 0x00
#define ICT153XX_CHIP_ID 0x01
#define ICT153XX_MANU_ID_VALUE 0xE7
#define ICT153XX_CHIP_ID_VALUE 0x45

#define ICT153XX_MODE_CTRL 0x04
#define ICT153XX_MODE_STATUS 0x05
#define ICT153XX_STATUS 0x06
#define ICT153XX_TEMP_DATA_LSB 0x08
#define ICT153XX_SEQUENCER_CTRL 0x7C
#define ICT153XX_GLOBAL_LOCK 0x7F

extern const sensor_mag_t sensor_mag_ict153xx;

#endif
