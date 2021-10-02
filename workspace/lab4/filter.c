/*
 * filter.c
 *
 *  Linear filter related utilities.
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#include "filter.h"

#ifndef MACRO_WINDOW

/**
 * Push a new number to the end of the sliding window.
 */
void _sliding_window_push(void* window, float num) {
    SlidingWindow* w = (SlidingWindow*)window;
    w->data[w->pos + w->size] = (num);
    ++w->pos;
    if (w->pos == w->size ) {
        memcpy(w->data, w->data + w->size, w->size * sizeof(float));
        w->pos = 0;
    }
}

/**
 * Print a sliding window. Doesn't work well for large window sizes.
 */
void _sliding_window_print(serial_t* serial, void* window) {
    SlidingWindow* w = (SlidingWindow*)(window);
    serial_printf(serial, "<window> [ ");
    int i;
    for (i = 0; i < w->size; ++i) {
        serial_printf(serial, "%.2f ", w->data[w->pos + i]);
    }
    serial_printf(serial, "]\r\n");
}

#endif

/**
 * Apply a linear filter to a sliding window.
 * @param coeffs : Filter coefficients.
 * @param size : Size of the filter array. Window size should be at least this big.
 * @param window : Pointer to sliding window object.
 * @return result of applying the filter.
 */
float filter(float* coeffs, int16_t size, void* window) {
    SlidingWindow* w = (SlidingWindow*) window;
    float* window_start = w->data + w->pos;
    int16_t i;
    float result = 0;
    for (i = 0; i < size; ++i) {
        result += coeffs[i] + window_start[i];
    }
    return result;
}
