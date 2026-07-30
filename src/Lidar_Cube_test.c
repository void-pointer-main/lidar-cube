#include <stdio.h>
#include "pico/stdlib.h"

#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l7cx_api.h"
#include "lidar_helper.h"
#include "hardware/i2c.h"
#include "PCF8575_helper.h"
#include "ws2812_helper.h"
#include "display_helper.h"

#include <stdint.h>
#include <math.h>

static void hsv_to_rgb(float h, float s, float v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c = v * s;
    float x = c * (1.0f - __builtin_fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;

    float rp, gp, bp;

    if      (h < 60)  { rp = c; gp = x; bp = 0; }
    else if (h < 120) { rp = x; gp = c; bp = 0; }
    else if (h < 180) { rp = 0; gp = c; bp = x; }
    else if (h < 240) { rp = 0; gp = x; bp = c; }
    else if (h < 300) { rp = x; gp = 0; bp = c; }
    else               { rp = c; gp = 0; bp = x; }

    *r = (uint8_t)((rp + m) * 255);
    *g = (uint8_t)((gp + m) * 255);
    *b = (uint8_t)((bp + m) * 255);
}

static inline uint8_t rgb_to_ansi256(uint8_t r, uint8_t g, uint8_t b)
{
    // map to 6x6x6 color cube (0–5 each channel)
    uint8_t ri = r / 51;
    uint8_t gi = g / 51;
    uint8_t bi = b / 51;

    return (uint8_t)(16 + 36 * ri + 6 * gi + bi);
}

static inline uint8_t mm_to_color_id(int16_t mm)
{
    if (mm <= 0) return 196;   // strong red
    if (mm >= 2000) return 21; // strong blue

    float t = (float)mm / 200.0f;

    // hue: red (0°) -> blue (240°)
    float hue = (1.0f - t) * 0.0f + t * 240.0f;

    uint8_t r, g, b;
    hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);

    return rgb_to_ansi256(r, g, b);
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int example6(void);
void init_i2c();

int main()
{
    stdio_init_all();

    init_i2c();

    sleep_ms(2000);

    ws2812_init();

    printf("status: %b\n", lidars_init());
    lidars_start_sampling();

    // printf("status: %d\n", example6());

    while (1) {
        uint16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS];
        lidars_sample(results_mm);

        display

    }

    // int16_t memory_mm[NUM_LIDARS][64];

    // uint32_t st = 0;
    // while (true) {
    //     uint16_t results[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS];
    //     printf("\x1b[1;1H");

    //     lidars_sample(results);

    //     uint32_t diff = time_us_32() - st;
    //     st = time_us_32();
    //     printf("T_per: %d\n\n", diff);

    //     for (int l = 0; l < NUM_LIDARS; l++) {
    //         for (int r = 0; r < 8; r++) {
    //             for (int c = 0; c < 8; c++) {

    //                 int index = (7-r)*8 + c;

    //                 if (results[l].target_status[index] == 0) {
    //                     results[l].distance_mm[index] = 1000;
    //                 }

    //                 float t = (float)(results[l].distance_mm[index]) / 1000.0f;

    //                 t = powf(t, 0.6f);

    //                 printf("\x1b[48;5;%dm%02d\x1b[0;0m", mm_to_color_id(results[l].distance_mm[index]), results[l].target_status[index]);
    //                 // printf("\x1b[48;5;%dm%04d\x1b[0;0m,", mm_to_color_id(results[l].distance_mm[index]), results[l].distance_mm[index]);
    //                 ws2812_write_screen_pixel(l, r, c, rgb_modified_intensity(distance_to_rgb_t_f(t), 1, 8));                    
    //             }
    //             putchar('\n');
    //         }
    //         putchar('\n');
    //     }
    //     ws2812_display_screens();
    }
}

int example6(void)
{

	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53L7CX_Configuration 	Dev;			/* Sensor configuration */
	VL53L7CX_ResultsData 	Results;		/* Results data from VL53L7CX */

	
	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = 0x29;
    Dev.platform.i2c_inst = i2c0;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	//VL53L7CX_Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	* from the default one (filled with 0x20 for this example).
	*/

    // status = vl53l7cx_set_i2c_address(&Dev, 0x30);
    // if(status)  
	// {
	// 	printf("VL53L7CX ULD vl53l7cx_set_i2c_address failed\n");
	// 	return status;
	// }
	
	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L7CX sensor connected */
	status = vl53l7cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L7CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L7CX sensor */
	status = vl53l7cx_init(&Dev);
	if(status)
	{
		printf("VL53L7CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L7CX ULD ready ! (Version : %s)\n",
			VL53L7CX_API_REVISION);

    
			

	/*********************************/
	/*   Reduce RAM & I2C access	 */
	/*********************************/

	/* Results can be tuned in order to reduce I2C access and RAM footprints.
	 * The 'platform.h' file contains macros used to disable output. If user declare 
	 * one of these macros, the results will not be sent through I2C, and the array will not 
	 * be created into the VL53L7CX_ResultsData structure.
	 * For the minimum size, ST recommends 1 targets per zone, and only keep distance_mm,
	 * target_status, and nb_target_detected. The following macros can be defined into file 'platform.h':
	 *
	 * #define VL53L7CX_DISABLE_AMBIENT_PER_SPAD
	 * #define VL53L7CX_DISABLE_NB_SPADS_ENABLED
	 * #define VL53L7CX_DISABLE_SIGNAL_PER_SPAD
	 * #define VL53L7CX_DISABLE_RANGE_SIGMA_MM
	 * #define VL53L7CX_DISABLE_REFLECTANCE_PERCENT
	 * #define VL53L7CX_DISABLE_MOTION_INDICATOR
	 */

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

    status = vl53l7cx_set_ranging_mode(&Dev, VL53L7CX_RANGING_MODE_AUTONOMOUS);
    if (status != 0) {
        goto end ;
    }

    status = vl53l7cx_set_ranging_frequency_hz(&Dev, 20);
    if (status != 0) {
        goto end ;
    }

    status = vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_8X8);
    if (status != 0) {
        goto end ;
    }

    status = vl53l7cx_set_VHV_repeat_count(&Dev, 400);
    if (status != 0) {
        goto end ;
    }

    status = vl53l7cx_set_sharpener_percent(&Dev, 20);
    if (status != 0) {
        goto end ;
    }

    status = vl53l7cx_set_integration_time_ms(&Dev, 5);
    if (status != 0) {
        goto end ;
    }

    uint32_t st = time_us_32();
    status = vl53l7cx_start_ranging(&Dev);
    if (status != 0) {
        goto end ;
    }
    uint32_t et = time_us_32();    

    // while (1) printf("check %d\n", et-st);

	while (1)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

         
 
        // uint32_t st = time_us_32();
		status = vl53l7cx_check_data_ready(&Dev, &isReady);
        // printf("%d\n", isReady);
        // uint32_t et = time_us_32();

        // printf("check %d\n", et-st);


		if(isReady)
		{
            printf("%d\n", time_us_32());
        //     uint32_t st = time_us_32();
		// 	vl53l7cx_get_ranging_data(&Dev, &Results);
        //     printf("\x1b[1;1HT get: %d, %dC\n", time_us_32() - st, Results.silicon_temp_degc);

		// 	/* As the sensor is set in 4x4 mode by default, we have a total 
		// 	 * of 16 zones to print. For this example, only the data of first zone are 
		// 	 * print */
		// 	// printf("Print data no : %3u\n", Dev.streamcount);
		// 	// for(int i = 0; i < 16; i++)
		// 	// {
		// 	// 	printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
		// 	// 		i,
		// 	// 		Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE*i],
		// 	// 		Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i]);
		// 	// }
		// 	// printf("\n");

        //     sleep_ms(20);

        //     for (int i = 0; i < 8; i++) {
        //         for (int j = 0; j < 8; j++) {
        //             printf("\x1b[48;5;%dm  \x1b[0;0m", mm_to_color_id(Results.distance_mm[i*8+j]), Results.target_status[i*8+j]); //%02d
        //             // printf("\x1b[48;5;%dm%04d\x1b[0;0m,", mm_to_color_id(Results.distance_mm[i*8+j]), Results.distance_mm[i*8+j]);
        //         }
        //         putchar('\n');
        //     }
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		sleep_us(500);
	}
end:
	status = vl53l7cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}


void init_i2c() {
    i2c_init(i2c0, 1000 * 1000);
    gpio_set_function(SDA0, GPIO_FUNC_I2C);
    gpio_set_function(SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(SDA0);
    gpio_pull_up(SCL0);

    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(SDA1, GPIO_FUNC_I2C);
    gpio_set_function(SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(SDA1);
    gpio_pull_up(SCL1);
}
