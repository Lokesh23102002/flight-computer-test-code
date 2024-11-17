/*
 * utility.c
 *
 *  Created on: Nov 5, 2024
 *      Author: LokeshTanwar
 */

#include "utility.h"
void floatToString(float value, char* buffer, int decimalPlaces) {
    // Extract integer and decimal parts
    int intPart = (int)value;
    int decimalPart = (int)((value - intPart) * 100);  // Get two decimal places

    // Handle negative numbers
    if (value < 0) {
        intPart = -intPart;
        decimalPart = -decimalPart;
        buffer[0] = '-';
        buffer++;
    }

    // Convert integer and decimal parts to string
    int i = 0;
    do {
        buffer[i++] = (intPart % 10) + '0';
        intPart /= 10;
    } while (intPart != 0);

    // Reverse integer part in buffer
    int start = 0, end = i - 1;
    while (start < end) {
        char temp = buffer[start];
        buffer[start++] = buffer[end];
        buffer[end--] = temp;
    }

    buffer[i++] = '.';  // Add decimal point

    // Add decimal part
    if (decimalPart < 10) buffer[i++] = '0';  // Leading zero if necessary
    if (decimalPart == 0) buffer[i++] = '0';  // If decimal part is zero
    else {
        do {
            buffer[i++] = (decimalPart % 10) + '0';
            decimalPart /= 10;
        } while (decimalPart != 0);
    }

    buffer[i] = '\0';  // Null-terminate the string
}
