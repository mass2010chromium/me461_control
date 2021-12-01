/*
 * filter.c
 *
 *  Linear filter related utilities.
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#include "filter.h"

/**
 * Push a new number to the end of the sliding window.
 */
void sliding_window_push(void* window, float num) {
    SlidingWindow* w = (SlidingWindow*)window;
    w->data[w->pos + w->size] = (num);
    ++w->pos;
    if (w->pos == w->size ) {
        memcpy(w->data, w->data + w->size, w->size * sizeof(float));
        w->pos = 0;
    }
}

float sliding_window_get(void* window, uint16_t idx) {
    SlidingWindow* w = (SlidingWindow*)window;
    return w->data[w->pos + w->size - idx - 1];
}

/**
 * Print a sliding window. Doesn't work well for large window sizes.
 */
void sliding_window_print(serial_t* serial, void* window) {
    SlidingWindow* w = (SlidingWindow*)(window);
    serial_printf(serial, "<window> [ ");
    int i;
    for (i = 0; i < w->size; ++i) {
        serial_printf(serial, "%.2f ", w->data[w->pos + i]);
    }
    serial_printf(serial, "]\r\n");
}

/**
 * Apply a linear filter to a sliding window.
 * @param coeffs : Filter coefficients.
 * @param size : Size of the filter array. Window size should be at least this big.
 * @param window : Pointer to sliding window object.
 * @return result of applying the filter.
 */
float filter(float* coeffs, int16_t size, void* window, int16_t window_size) {
    SlidingWindow* w = (SlidingWindow*) window;
    float* window_start = w->data + w->pos + (window_size - size);
    int16_t i;
    float result = 0;
    for (i = 0; i < size; ++i) {
        result += coeffs[i] * window_start[i];
    }
    return result;
}

float filter2(float* b, float* a_rev, int16_t size, void* window, void* history, int16_t window_size) {
    SlidingWindow* w = (SlidingWindow*) window;
    SlidingWindow* w2 = (SlidingWindow*) history;
    float* window_start = w->data + w->pos + (window_size - size);
    float* window_start2 = w2->data + w2->pos + (window_size - size);
    int16_t i;
    float result = b[0] * window_start[0];
    for (i = 1; i < size; ++i) {
        result += b[i] * window_start[i];
        result -= a_rev[i] * window_start2[i];
    }
    result /= a_rev[0];
    sliding_window_push(history, result);
    return result;
}
