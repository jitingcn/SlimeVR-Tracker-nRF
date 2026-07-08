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
#include "parse_args.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>

size_t parse_args(char *str, char *argv[], size_t size)
{
	size_t argc = 0;

	if (size == 0) {
		return 0;
	}

	argv[0] = NULL;

	if (str == NULL) {
		return 0;
	}

	while (isspace((unsigned char)*str)) {
		str++;
	}

	while (*str != '\0') {
		if (argc + 1 >= size) {
			printk("Too many parameters (max %u)\n", (unsigned int)(size - 1));
			argv[0] = NULL;
			return 0;
		}

		argv[argc++] = str;

		while (*str != '\0' && !isspace((unsigned char)*str)) {
			str++;
		}

		if (*str == '\0') {
			break;
		}

		*str++ = '\0';
		while (isspace((unsigned char)*str)) {
			str++;
		}
	}

	argv[argc] = NULL;
	return argc;
}

int32_t parse_i32(const char *str, uint8_t base)
{
	long long res = strtoll(str, NULL, base);

	if (res < INT32_MIN) {
		res = INT32_MIN;
		errno = ERANGE;
	}
	if (res > INT32_MAX) {
		res = INT32_MAX;
		errno = ERANGE;
	}

	return (int32_t)res;
}

uint32_t parse_u32(const char *str, uint8_t base)
{
	unsigned long long res = strtoull(str, NULL, base);

	if (res > UINT32_MAX) {
		res = UINT32_MAX;
		errno = ERANGE;
	}

	return (uint32_t)res;
}

uint64_t parse_u64(const char *str, uint8_t base)
{
	return strtoull(str, NULL, base);
}
