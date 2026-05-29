/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>

#include "retained.h"

// Adafruit UF2 bootloader GPREGRET magic values.
// See: Adafruit_nRF52_Bootloader/src/main.c (DFU_MAGIC_*).
#define ADAFRUIT_DFU_MAGIC_UF2_RESET 0x57
#define ADAFRUIT_DFU_MAGIC_OTA_RESET 0xA8

#define USER_SHUTDOWN_ENABLED CONFIG_USER_SHUTDOWN // Allow user to use reset or sw0 to shutdown
#ifdef CONFIG_IGNORE_CHARGE_WAKE_ON_VBUS
#define IGNORE_CHARGE_WAKE_ON_VBUS 1
#else
#define IGNORE_CHARGE_WAKE_ON_VBUS 0
#endif
#define IGNORE_RESET CONFIG_IGNORE_RESET // If sw0 available, don't change any reset behavior
#define WOM_USE_DCDC CONFIG_WOM_USE_DCDC // Use DCDC instead of LDO for WOM if it is more efficient

/* Sensor gyroscope, accelerometer, and magnetometer axes should align to the IMU body axes.
 * SENSOR_QUATERNION_CORRECTION right-multiplies the fused sensor quaternion to align sensor axes
 * to the device/body axes following Android convention: Qdevice = Qfused * Qcorr.
 * SENSOR_QUATERNION_OUTPUT_BIAS left-multiplies only the reported quaternion to add an optional
 * preview/world-frame neutral-pose bias after device alignment: Qout = Qbias * Qdevice.
 * Sensor-frame vectors that are reported in local device coordinates (for example linear
 * acceleration and packet-4 magnetometer) should use the inverse/conjugate of Qcorr, because
 * they are active vector rotations rather than right-multiplied orientation composition.
 * This lets the preview/model pose be adjusted without changing the device-frame linear
 * acceleration basis, which continues to use only SENSOR_QUATERNION_CORRECTION.
 * The two transforms intentionally stay separate because left and right quaternion multiplies do
 * not generally commute.
 * On flat surface / face up:
 * Left from the perspective of the device / right from your perspective is +X
 * Front side (facing up) is +Z
 * Mounted on body / standing up:
 * Top side of the device is +Y
 * Front side (facing out) is +Z
 */

// TODO: not matching anymore
#if defined(CONFIG_BOARD_SLIMEVRMINI_P1_UF2) || defined(CONFIG_BOARD_SLIMEVRMINI_P2_UF2)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -mx, mz, -my
#define SENSOR_QUATERNION_CORRECTION 0.7071f, 0.7071f, 0.0f, 0.0f
#endif
#if defined(CONFIG_BOARD_SLIMEVRMINI_P4_UF2)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, -mx, -mz
#define SENSOR_QUATERNION_CORRECTION 0.7071f, 0.0f, 0.0f, 0.7071f
#endif

#if defined(CONFIG_BOARD_SLIMENRF_R1) || defined(CONFIG_BOARD_SLIMENRF_R2) || defined(CONFIG_BOARD_SLIMENRF_R3)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, -mx, -mz
#define SENSOR_QUATERNION_CORRECTION 0.0f, 0.7071f, 0.7071f, 0.0f
#endif

#if defined(CONFIG_BOARD_FOXSMOL40_UF2)
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, mx, mz
#endif

#if defined(CONFIG_BOARD_FOXSNACKLITEV2_UF2)
#define SENSOR_QUATERNION_CORRECTION 0.0f, 0.7071f, 0.7071f, 0.0f
#endif

#if defined(CONFIG_BOARD_PROMICRO_UF2) || defined(CONFIG_BOARD_STYRIA_MINI_UF2)
#define SENSOR_QUATERNION_CORRECTION 0.7071f, 0.0f, 0.0f, -0.7071f
// Output bias disabled: the server's own reset/calibration compensates for yaw offset,
// so the extra left-multiply (+90° Z to make neutral pose report identity) is unnecessary.
// #define SENSOR_QUATERNION_OUTPUT_BIAS 0.7071f, 0.0f, 0.0f, 0.7071f
#endif

// default orientation for most boards with the sensor mounted flat on the PCB
// with the top side as +X and front side as +Z and left side as +Y from your perspective
// on stacked promicro with common breakout board
#ifndef SENSOR_MAGNETOMETER_AXES_ALIGNMENT
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT mx, my, mz // mag axes alignment to sensor body
#endif
// not sure if this is needed or correct, it still seems weird in server without full reset, but leaving it for now
#ifndef SENSOR_QUATERNION_CORRECTION
#define SENSOR_QUATERNION_CORRECTION 1.0f, 0.0f, 0.0f, 0.0f
#endif
#ifndef SENSOR_QUATERNION_OUTPUT_BIAS
#define SENSOR_QUATERNION_OUTPUT_BIAS 1.0f, 0.0f, 0.0f, 0.0f
// Default identity bias lets the compiler elide the extra output-bias multiply entirely.
#define SENSOR_QUATERNION_OUTPUT_BIAS_IS_IDENTITY 1
#endif
#ifndef SENSOR_QUATERNION_OUTPUT_BIAS_IS_IDENTITY
// If a board explicitly defines SENSOR_QUATERNION_OUTPUT_BIAS as identity, set this to 1 too.
#define SENSOR_QUATERNION_OUTPUT_BIAS_IS_IDENTITY 0
#endif

#endif
