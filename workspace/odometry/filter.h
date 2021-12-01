/*
 * filter.h
 *
 *  Linear filter related utilities.
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#ifndef FILTER_H_
#define FILTER_H_
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"

#include "F2837xD/f28379dSerial.h"

typedef struct {
    uint16_t pos;
    uint16_t size;
    float data[0];
} SlidingWindow;


/**
 * Initialize a sliding window.
 * Only works in toplevel.
 * @param name : Name of the window.
 * @param size : Constant size of the window.
 */
#define sliding_window_init(name, size) \
typedef struct { SlidingWindow w; float _data1[size]; float _data2[size]; } __ ## name ## _ ## T; \
__ ## name ## _ ## T name = { {0, size} };

#define sliding_window_setup(window) \
(window).w.size = (sizeof(window)._data1 / sizeof(float)); \

#define sliding_window_decl(name, size) \
typedef struct { SlidingWindow w; float _data1[size]; float _data2[size]; } name; \

#define windowsize(name) (sizeof(name._data1) / sizeof(float))

/**
 * Push a new number to the end of the sliding window.
 * @param w : pointer to sliding window.
 * @param num : Number to push.
 */
void sliding_window_push(void* window, float num);

/**
 * Read out of a sliding window.
 * @param w : pointer to sliding window.
 * @param idx: index to read
 */
float sliding_window_get(void* window, uint16_t idx);

/**
 * Print a sliding window. Doesn't work well for large window sizes.
 * @param serial : Pointer to serial object.
 * @param w : pointer to sliding window.
 */
void sliding_window_print(serial_t* serial, void* window);

/**
 * Apply a linear filter to a sliding window.
 * @param coeffs : Filter coefficients.
 * @param size : Size of the filter array. Window size should be at least this big.
 * @param window : Pointer to sliding window object.
 * @param window_size : Size of the sliding window
 * @return result of applying the filter.
 */
float filter(float* coeffs, int16_t size, void* window, int16_t window_size);

/**
 * Apply a linear filter to a sliding window.
 * @param coeffs : Filter coefficients.
 * @param size : Size of the filter array. Window size should be at least this big.
 * @param window : Pointer to sliding window object.
 * @param window_size : Size of the sliding window
 * @return result of applying the filter.
 */

float filter2(float* b, float* a_rev, int16_t filter_size, void* window, void* history, int16_t window_size);

#define arraysize(arr) (sizeof(arr)/sizeof((arr)[0]))

#endif /* FILTER_H_ */
