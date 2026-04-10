//
// Created by Forest on 2026-04-09.
//

#include "calibration.h"
#include "motor_interface.h"
#include "foc.h"
#include <math.h>
#include <stdio.h>

#define M_PI 3.14159265358979323846f

float calibrate_counts_to_angle(uint16_t counts)
{
    const float scale = (2.0f * M_PI) / (float)ENCODER_COUNTS;
    return (float)counts * scale;
}

uint16_t calibrate_single_electrical_rev(float Ud, float Vbus, int steps)
{
    int32_t min_off_counts =  1000000;
    int32_t max_off_counts = -1000000;
    int64_t sum_off_counts =  0;

    for (int i = 0; i < steps; i++)
    {
        // Commanded electrical angle
        float theta_cmd = (2.0f * M_PI * (float)i) / (float)steps;

        // Apply open-loop d-axis voltage at theta_cmd
        DutyABC_t d = open_loop_step(Ud, 0.0f, theta_cmd, Vbus);
        write_duty(d);

        HAL_Delay(500); // let rotor settle

        // Read encoder raw counts (0..8191)
        uint16_t raw = motor_interface_get_position();

        // Convert commanded angle -> expected encoder count
        float counts_per_rad = (float)ENCODER_COUNTS / (2.0f * M_PI);
        uint16_t cmd_counts = (uint16_t)lroundf(theta_cmd * counts_per_rad);

        // Signed offset in counts (raw difference, NO normalization)
        int32_t off_counts = (int32_t)raw - (int32_t)cmd_counts;

        // Wrap-aware normalization for 13-bit encoder
        if (off_counts >  (ENCODER_COUNTS/2))  off_counts -= ENCODER_COUNTS;
        if (off_counts < -(ENCODER_COUNTS/2))  off_counts += ENCODER_COUNTS;

        // Track stats exactly as measured
        if (off_counts < min_off_counts) min_off_counts = off_counts;
        if (off_counts > max_off_counts) max_off_counts = off_counts;

        sum_off_counts += off_counts;

        printf("Step %d: Cmd angle %f rad, Cmd counts %u, Raw counts %u, Offset %ld counts\r\n",
               i, theta_cmd, cmd_counts, raw, (long)off_counts);
    }

    // Average offset in counts (signed)
    int32_t avg_off_counts = (int32_t)(sum_off_counts / steps);

    // Convert to unsigned 0..8191 for storage
    uint16_t avg_off_counts_u =
        (uint16_t)((avg_off_counts + ENCODER_COUNTS) & ENCODER_MASK);

    // Convert to radians for display only
    float rad_per_count = (2.0f * M_PI) / (float)ENCODER_COUNTS;
    float min_off_rad = min_off_counts * rad_per_count;
    float max_off_rad = max_off_counts * rad_per_count;
    float avg_off_rad = avg_off_counts * rad_per_count;

    printf("Encoder calibration (1 electrical rev):\r\n");
    printf("  Min offset: %ld counts (%f rad)\r\n", (long)min_off_counts, min_off_rad);
    printf("  Max offset: %ld counts (%f rad)\r\n", (long)max_off_counts, max_off_rad);
    printf("  Avg offset: %ld counts (%f rad)\r\n", (long)avg_off_counts, avg_off_rad);
    printf("  Peak-to-peak error: %ld counts (%f rad)\r\n",
           (long)(max_off_counts - min_off_counts),
           (max_off_rad - min_off_rad));

    return avg_off_counts_u;
}
