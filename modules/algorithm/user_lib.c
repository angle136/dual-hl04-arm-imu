#include "user_lib.h"

void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value    = max;
    ramp_source_type->min_value    = min;
    ramp_source_type->input        = 0.0f;
    ramp_source_type->out          = 0.0f;
}

void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period,
                             const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0]       = num[0];
    first_order_filter_type->input        = 0.0f;
    first_order_filter_type->out          = 0.0f;
}

void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] /
            (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
            first_order_filter_type->out +
        first_order_filter_type->frame_period /
            (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
            first_order_filter_type->input;
}

float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
int16_t currentToInteger(float I_min, float I_max, int16_t V_min, int16_t V_max, float current)
{
    int16_t V = (int)((current - I_min) / (I_max - I_min) * (V_max - V_min) + V_min);
    if (V > V_max)
    {
        V = V_max;
    }
    if (V < V_min)
    {
        V = V_min;
    }
    return V;
}

float IntegerToCurrent(float I_min, float I_max, int16_t V_min, int16_t V_max, int16_t V)
{
    float I = (float)(V - V_min) / (float)(V_max - V_min) * (I_max - I_min) + I_min;
    if (I > I_max)
    {
        I = I_max;
    }
    if (I < I_min)
    {
        I = I_min;
    }
    return I;
}

float rad_to_deg(float rad)
{
    return rad * (180.0f / PI);
}

float deg_to_rad(float deg)
{
    return deg * (PI / 180.0f);
}
