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

#define MACRO_WINDOW

#ifndef MACRO_WINDOW
#include "F2837xD/f28379dSerial.h"
/*
 * Function based sliding window implementation;
 */
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

#define windowsize(name) (sizeof(name._data1) / sizeof(float))

/**
 * Push a new number to the end of the sliding window.
 * @param w : Sliding window OBJECT!! NOT POINTER!
 * @param num : Number to push.
 */
#define sliding_window_push(w, num) _sliding_window_push(&(w), num)
void _sliding_window_push(void* window, float num);

/**
 * Print a sliding window. Doesn't work well for large window sizes.
 * @param serial : Pointer to serial object.
 * @param w : Sliding window OBJECT!! NOT POINTER!
 */
#define sliding_window_print(serial, w) _sliding_window_print(serial, &(w))
void _sliding_window_print(serial_t* serial, void* window);

#else

/*
 * Fully macro based sliding window. Maybe faster since u can inline the integers but more janky feeling.
 */
typedef struct {
    uint16_t pos;
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
__ ## name ## _ ## T name = { { 0 } }; \
const uint16_t __ ## name ## _size = (size);

#define windowsize(name) __ ## name ## _size

/**
 * Push a new number to the end of the sliding window.
 * @param w : Sliding window OBJECT!! NOT POINTER!
 * @param num : Number to push.
 */
#define sliding_window_push(w, num) {\
    SlidingWindow* window = (SlidingWindow*)(&(w));\
    window->data[ __ ## w ## _ ## size + window->pos] = (num);\
    ++window->pos;\
    if (window->pos == __ ## w ## _size ) {\
        memcpy(window->data, window->data + __ ## w ## _ ## size, __ ## w ## _ ## size * sizeof(float));\
        window->pos = 0;\
    }\
}

/**
 * Print a sliding window. Doesn't work well for large window sizes.
 * @param serial : Pointer to serial object.
 * @param w : Sliding window OBJECT!! NOT POINTER!
 */
#define sliding_window_print(serial, w) {\
    SlidingWindow* window = (SlidingWindow*)(&(w));\
    serial_printf((serial), "<window> [ ");\
    int i;\
    for (i = 0; i < __ ## w ## _ ## size; ++i) {\
        serial_printf((serial), "%.2f ", window->data[window->pos + i]);\
    }\
    serial_printf((serial), "]\r\n");\
}

#endif /* MACRO_WINDOW */


/**
 * Apply a linear filter to a sliding window.
 * @param coeffs : Filter coefficients.
 * @param size : Size of the filter array. Window size should be at least this big.
 * @param window : Pointer to sliding window object.
 * @param window_size : Size of the sliding window
 * @return result of applying the filter.
 */
float filter(float* coeffs, int16_t size, void* window, int16_t window_size);

#define arraysize(arr) (sizeof(arr)/sizeof((arr)[0]))

#endif /* FILTER_H_ */
