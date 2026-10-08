#include "amr/amr_cobs.h"

bool amr_cobs_encode(const uint8_t *input,
                     size_t input_length,
                     uint8_t *output,
                     size_t output_capacity,
                     size_t *output_length)
{
    if ((input == NULL && input_length != 0U) ||
        output == NULL ||
        output_length == NULL ||
        output_capacity == 0U) {
        return false;
    }

    size_t read_index = 0U;
    size_t write_index = 1U;
    size_t code_index = 0U;
    uint8_t code = 1U;

    while (read_index < input_length) {
        if (input[read_index] == 0U) {
            if (code_index >= output_capacity) {
                return false;
            }
            output[code_index] = code;
            code = 1U;
            code_index = write_index;
            ++write_index;
        } else {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index] = input[read_index];
            ++write_index;
            ++code;

            if (code == 0xFFU) {
                if (code_index >= output_capacity) {
                    return false;
                }
                output[code_index] = code;
                code = 1U;
                code_index = write_index;
                ++write_index;
            }
        }
        ++read_index;
    }

    if (code_index >= output_capacity) {
        return false;
    }
    output[code_index] = code;
    *output_length = write_index;
    return true;
}

bool amr_cobs_decode(const uint8_t *input,
                     size_t input_length,
                     uint8_t *output,
                     size_t output_capacity,
                     size_t *output_length)
{
    if (input == NULL ||
        input_length == 0U ||
        output == NULL ||
        output_length == NULL) {
        return false;
    }

    size_t read_index = 0U;
    size_t write_index = 0U;

    while (read_index < input_length) {
        const uint8_t code = input[read_index];
        if (code == 0U) {
            return false;
        }
        ++read_index;

        const size_t copy_length = (size_t)code - 1U;
        if (read_index + copy_length > input_length ||
            write_index + copy_length > output_capacity) {
            return false;
        }

        for (size_t index = 0U; index < copy_length; ++index) {
            output[write_index] = input[read_index];
            ++write_index;
            ++read_index;
        }

        if (code != 0xFFU && read_index < input_length) {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index] = 0U;
            ++write_index;
        }
    }

    *output_length = write_index;
    return true;
}

