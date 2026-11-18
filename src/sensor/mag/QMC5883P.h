/* QMC5883P three-axis magnetometer driver.
 *
 * Written against the QST datasheet QMC5883P (QST-PD-B002-22, Rev E). The
 * register plumbing follows this repository's QMC6309.c because both parts
 * share one interface style (little-endian data, CHIP_ID at 0x00, STATUS at
 * 0x09, CTRL1/CTRL2, four modes plus suspend, SET/RESET bit). That is a
 * skeleton reuse, not a product lineage.
 *
 * Differences from QMC6309 that every change below follows:
 *   1. I2C only: probe 0x2C directly or through an IMU's I2C master,
 *      including when the IMU itself uses SPI. Register 0x00 reads 0x80;
 *      register 0x01 is sample data, not a second identity register.
 *   2. ODR bits live in CTRL1 (0x0A) [3:2] (QMC6309 uses CTRL2 [6:4]), two
 *      bits: 00 = 10, 01 = 50, 10 = 100, 11 = 200 Hz.
 *   3. 8 G sensitivity is 3750 LSB/G (QMC6309: 4000); RNG_8G is still 0b10.
 *   4. CTRL2 (0x0B) init value = RNG_8G<<2 | SET_RESET_ON = 0x08. Power-on
 *      sequence: 0x29 = 0x06 (sign register) -> 0x0B = 0x08 -> 0x0A config.
 *   5. A mode change must transit through suspend: write MODE=00 before
 *      entering another running mode. Single mode returns to suspend by
 *      itself once the measurement is done.
 *   6. Self-test is CTRL2 bit 6; setting it and reading the data registers
 *      5 ms later is a factory check. The driver never runs it during init
 *      because it perturbs the measurement; the bit stays defined for debug.
 *
 * Do not copy register values from the QMC5883L driver: L and P are
 * incompatible. In particular CTRL_2 = 0x41 would set P's self-test bit and
 * select the 30 G range, and the correct P value is 0x08, not 0x01. L and P
 * can share one bus because each answers on its own address (0x0D vs 0x2C).
 */
#ifndef QMC5883P_h
#define QMC5883P_h

#include "sensor/sensor.h"

int qmc5883p_init(float period_s, float *actual_period_s);
void qmc5883p_shutdown(void);

int qmc5883p_update_odr(float period_s, float *actual_period_s);

void qmc5883p_mag_oneshot(void);
bool qmc5883p_mag_read(float m[3]);

void qmc5883p_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc5883p;

#endif
