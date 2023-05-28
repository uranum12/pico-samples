#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define SPI_ID spi0
#define SPI_BAUD 100000

#define SPI_SCK 2
#define SPI_TX 3
#define SPI_RX 4
#define SPI_CS 5

enum osr {
    osr_skip = 0x00,
    osr_x1 = 0x01,
    osr_x2 = 0x02,
    osr_x4 = 0x03,
    osr_x8 = 0x04,
    osr_x16 = 0x05,
};

enum filter {
    filter_off = 0x00,
    filter_x2 = 0x01,
    filter_x4 = 0x02,
    filter_x8 = 0x03,
    filter_x16 = 0x04,
};

enum standby_time {
    standby_time_0p5ms = 0x00,
    standby_time_62p5ms = 0x01,
    standby_time_125ms = 0x02,
    standby_time_250ms = 0x03,
    standby_time_500ms = 0x04,
    standby_time_1000ms = 0x05,
    standby_time_10ms = 0x06,
    standby_time_20ms = 0x07,
};

enum mode {
    mode_sleep = 0x00,
    mode_forced = 0x01,
    mode_normal = 0x03,
};

struct settings {
    uint8_t osr_t;         // temperature oversampling
    uint8_t osr_p;         // pressure oversampling
    uint8_t osr_h;         // humidity oversampling
    uint8_t filter;        // filter coefficient
    uint8_t standby_time;  // standby time
};

struct calib_data {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
};

struct raw_data {
    uint32_t temperature;
    uint32_t pressure;
    uint16_t humidity;
};

struct data {
    double temperature;
    double pressure;
    double humidity;
};

struct data_int {
    int32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
};

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI_CS, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t addr, uint8_t data) {
    uint8_t buf[2];
    buf[0] = addr & 0x7F;
    buf[1] = data;
    cs_select();
    spi_write_blocking(SPI_ID, buf, 2);
    cs_deselect();
    sleep_us(10);
}

static void read_registers(uint8_t addr, uint8_t* buf, uint16_t len) {
    addr |= 0x80;
    cs_select();
    spi_write_blocking(SPI_ID, &addr, 1);
    sleep_us(10);
    spi_read_blocking(SPI_ID, 0, buf, len);
    cs_deselect();
    sleep_us(10);
}

void reset() {
    write_register(0xE0, 0xB6);
}

uint8_t get_chip_id() {
    uint8_t chip_id = 0;
    read_registers(0xD0, &chip_id, 1);
    return chip_id;
}

bool chip_id_valid(uint8_t chip_id) {
    return chip_id == 0x60;
}

uint8_t get_status() {
    uint8_t status = 0;
    read_registers(0xF3, &status, 1);
    return status;
}

bool status_measuring(uint8_t status) {
    return status & 0x08;
}

bool status_im_update(uint8_t status) {
    return status & 0x01;
}

void get_calib_data(struct calib_data* calib_data) {
    uint8_t buf[26];
    read_registers(0x88, buf, 26);
    calib_data->dig_t1 = (uint16_t)buf[1] << 8 | buf[0];
    calib_data->dig_t2 = (int16_t)(buf[3] << 8 | buf[2]);
    calib_data->dig_t3 = (int16_t)(buf[5] << 8 | buf[4]);
    calib_data->dig_p1 = (uint16_t)buf[7] << 8 | buf[6];
    calib_data->dig_p2 = (int16_t)(buf[9] << 8 | buf[8]);
    calib_data->dig_p3 = (int16_t)(buf[11] << 8 | buf[10]);
    calib_data->dig_p4 = (int16_t)(buf[13] << 8 | buf[12]);
    calib_data->dig_p5 = (int16_t)(buf[15] << 8 | buf[14]);
    calib_data->dig_p6 = (int16_t)(buf[17] << 8 | buf[16]);
    calib_data->dig_p7 = (int16_t)(buf[19] << 8 | buf[18]);
    calib_data->dig_p8 = (int16_t)(buf[21] << 8 | buf[20]);
    calib_data->dig_p9 = (int16_t)(buf[23] << 8 | buf[22]);
    calib_data->dig_h1 = buf[25];
    read_registers(0xE1, buf, 7);
    calib_data->dig_h2 = (int16_t)(buf[1] << 8 | buf[0]);
    calib_data->dig_h3 = buf[2];
    calib_data->dig_h4 = (int16_t)(buf[3] << 4 | (buf[4] & 0x0F));
    calib_data->dig_h5 = (int16_t)(buf[5] << 4 | buf[4] >> 4);
    calib_data->dig_h6 = (int8_t)buf[6];
}

uint8_t get_mode() {
    uint8_t buf = 0;
    read_registers(0xF4, &buf, 1);
    return buf & 0x03;
}

void set_mode(uint8_t mode) {
    uint8_t buf = 0;
    read_registers(0xF4, &buf, 1);
    buf = (buf & 0xFC) | mode;
    write_register(0xF4, buf);
}

void get_settings(struct settings* settings) {
    uint8_t buf[3];
    read_registers(0xF2, buf, 1);
    read_registers(0xF4, buf + 1, 2);
    settings->osr_h = buf[0] & 0x07;
    settings->osr_p = (buf[1] & 0x1C) >> 2;
    settings->osr_t = (buf[1] & 0xE0) >> 5;
    settings->filter = (buf[2] & 0x1C) >> 2;
    settings->standby_time = (buf[2] & 0xE0) >> 5;
}

void set_settings(struct settings* settings) {
    uint8_t mode = get_mode();
    write_register(0xF2, settings->osr_h);
    write_register(0xF4, settings->osr_t << 5 | settings->osr_p << 2 | mode);
    write_register(0xF5, settings->standby_time << 5 | settings->filter << 2);
}

void get_raw_data(struct raw_data* raw_data) {
    uint8_t buf[8];
    read_registers(0xF7, buf, 8);
    raw_data->pressure = buf[0] << 12 | buf[1] << 4 | buf[2] >> 4;
    raw_data->temperature = buf[3] << 12 | buf[4] << 4 | buf[5] >> 4;
    raw_data->humidity = buf[6] << 8 | buf[7];
}

double compensate_temperature(uint32_t raw_data, struct calib_data* calib_data,
                              int32_t* t_fine) {
    double var1 =
        ((double)raw_data) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0;
    var1 = var1 * ((double)calib_data->dig_t2);
    double var2 =
        (((double)raw_data) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    *t_fine = (int32_t)(var1 + var2);
    double temperature = (var1 + var2) / 5120.0;
    return temperature < -40.0  ? -40.0
           : 85.0 < temperature ? 85.0
                                : temperature;
}

double compensate_pressure(uint32_t raw_data, struct calib_data* calib_data,
                           int32_t t_fine) {
    double var1 = ((double)t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var1 = ((((double)calib_data->dig_p3) * var1 * var1 / 524288.0) +
            ((double)calib_data->dig_p2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);
    if (var1 > 0.0) {
        double var3 = 1048576.0 - (double)raw_data;
        var3 = (var3 - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * var3 * var3 / 2147483648.0;
        var2 = var3 * ((double)calib_data->dig_p8) / 32768.0;
        double pressure =
            (var3 + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0) / 100;
        return pressure < 300.0 ? 300.0 : 1100.0 < pressure ? 1100.0 : pressure;
    }
    return 300.0;
}

double compensate_humidity(uint16_t raw_data, struct calib_data* calib_data,
                           int32_t t_fine) {
    double var1 = ((double)t_fine) - 76800.0;
    double var2 = (((double)calib_data->dig_h4) * 64.0 +
                   (((double)calib_data->dig_h5) / 16384.0) * var1);
    double var3 = raw_data - var2;
    double var4 = ((double)calib_data->dig_h2) / 65536.0;
    double var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    double var6 =
        1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    double humidity =
        var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);
    humidity = humidity < 0.0 ? 0.0 : 100.0 < humidity ? 100.0 : humidity;
    return humidity;
}

void compensate_data(struct raw_data* raw_data, struct calib_data* calib_data,
                     struct data* data) {
    int32_t t_fine = 0;
    data->temperature =
        compensate_temperature(raw_data->temperature, calib_data, &t_fine);
    data->pressure =
        compensate_pressure(raw_data->pressure, calib_data, t_fine);
    data->humidity =
        compensate_humidity(raw_data->humidity, calib_data, t_fine);
}

int32_t compensate_temperature_int(uint32_t raw_data,
                                   struct calib_data* calib_data,
                                   int32_t* t_fine) {
    int32_t var1 =
        ((int32_t)(raw_data / 8)) - ((int32_t)calib_data->dig_t1 * 2);
    var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
    int32_t var2 = (int32_t)((raw_data / 16) - ((int32_t)calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
    *t_fine = var1 + var2;
    int32_t temperature = (*t_fine * 5 + 128) / 256;
    return temperature < -4000  ? -4000
           : 8500 < temperature ? 8500
                                : temperature;
}

uint32_t compensate_pressure_int(uint32_t raw_data,
                                 struct calib_data* calib_data,
                                 int32_t t_fine) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
    var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) +
           ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
    var1 = ((((int64_t)1) * 140737488355328) + var1) *
           ((int64_t)calib_data->dig_p1) / 8589934592;
    if (var1 != 0) {
        int64_t var3 = 1048576 - raw_data;
        var3 = (((var3 * 2147483648) - var2) * 3125) / var1;
        var1 = (((int64_t)calib_data->dig_p9) * (var3 / 8192) * (var3 / 8192)) /
               33554432;
        var2 = (((int64_t)calib_data->dig_p8) * var3) / 524288;
        var3 =
            ((var3 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
        uint32_t pressure = (uint32_t)(((var3 / 2) * 100) / 128);
        return pressure < 3000000    ? 3000000
               : 11000000 < pressure ? 11000000
                                     : pressure;
    }
    return 3000000;
}

uint32_t compensate_humidity_int(uint16_t raw_data,
                                 struct calib_data* calib_data,
                                 int32_t t_fine) {
    int32_t var1 = t_fine - ((int32_t)76800);
    int32_t var2 = (int32_t)(raw_data * 16384);
    int32_t var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    int32_t var4 = ((int32_t)calib_data->dig_h5) * var1;
    int32_t var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    uint32_t humidity = (uint32_t)(var5 / 4096);
    return 102400 < humidity ? 102400 : humidity;
}

void compensate_data_int(struct raw_data* raw_data,
                         struct calib_data* calib_data,
                         struct data_int* data_int) {
    int32_t t_fine = 0;
    data_int->temperature =
        compensate_temperature_int(raw_data->temperature, calib_data, &t_fine);
    data_int->pressure =
        compensate_pressure_int(raw_data->pressure, calib_data, t_fine);
    data_int->humidity =
        compensate_humidity_int(raw_data->humidity, calib_data, t_fine);
}

int main() {
    stdio_init_all();

    spi_init(SPI_ID, SPI_BAUD);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(SPI_SCK, SPI_TX, SPI_RX, GPIO_FUNC_SPI));

    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_OUT);
    gpio_put(SPI_CS, 1);
    bi_decl(bi_1pin_with_name(SPI_CS, "SPI CS"));

    reset();

    do {
        sleep_ms(1);
    } while (status_im_update(get_status()));

    uint8_t chip_id = get_chip_id();
    if (chip_id_valid(chip_id)) {
        printf("chip id, %#x is valid.\n", chip_id);
    }

    struct calib_data calib_data;
    get_calib_data(&calib_data);

    struct settings settings = {
        .osr_t = osr_x16,
        .osr_p = osr_x16,
        .osr_h = osr_x16,
        .filter = filter_x16,
        .standby_time = standby_time_1000ms,
    };
    set_settings(&settings);

    while (1) {
        set_mode(mode_forced);

        do {
            sleep_ms(10);
        } while (status_measuring(get_status()));

        struct raw_data raw_data;
        get_raw_data(&raw_data);

        struct data data;
        compensate_data(&raw_data, &calib_data, &data);
        printf("temperature : %lf\n", data.temperature);
        printf("pressure    : %lf\n", data.pressure);
        printf("humidity    : %lf\n", data.humidity);

        struct data_int data_int;
        compensate_data_int(&raw_data, &calib_data, &data_int);
        printf("temperature : %d\n", data_int.temperature);
        printf("pressure    : %d\n", data_int.pressure);
        printf("humidity    : %d\n", data_int.humidity);

        sleep_ms(1000);
    }
}
