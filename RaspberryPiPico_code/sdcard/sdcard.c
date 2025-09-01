#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "ff.h"
#include "sd_card.h"

FATFS fs;
FIL file;

static void log_line(FIL *f, uint32_t t_ms, float v1, float v2) {
    char line[96];
    int len = snprintf(line, sizeof line, "%lu,%.2f,%.2f\r\n",
                       (unsigned long)t_ms, v1, v2);
    UINT bw;
    f_write(f, line, (UINT)len, &bw);
}

int main(void) {
    stdio_init_all();
    sleep_ms(200);

    if (!sd_init_driver()) { while (1) tight_loop_contents(); }
    if (f_mount(&fs, "0:", 1) != FR_OK) { while (1) tight_loop_contents(); }

    FRESULT fr = f_open(&file, "log.csv", FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) { while (1) tight_loop_contents(); }

    if (f_size(&file) == 0) {
        const char *hdr = "timestamp_ms,value1,value2\r\n";
        UINT bw; f_write(&file, hdr, (UINT)strlen(hdr), &bw);
    }

    // Append one line per second for 10 seconds
    uint32_t end_ms = to_ms_since_boot(get_absolute_time()) + 10000;
    while (to_ms_since_boot(get_absolute_time()) < end_ms) {
        uint32_t t_ms = to_ms_since_boot(get_absolute_time());
        float v1 = 69.69f; //jew data
        float v2 = -67.67f;
        log_line(&file, t_ms, v1, v2);
        f_sync(&file);          
        sleep_ms(1000);
    }

    f_close(&file);
    f_mount(NULL, "0:", 0);
    while (1) tight_loop_contents();
}
