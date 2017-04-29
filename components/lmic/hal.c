#include "hal.h"

int x_irq_level = 0;

spi_device_handle_t spi_handle;

// -----------------------------------------------------------------------------
// I/O

#define NSS_PIN            22
#define NSS_PIN_POS        (1<<NSS_PIN)

#define RST_PIN            19
#define RST_PIN_POS        (1<<RST_PIN)

#define DIO0_PIN           23
#define DIO0_PIN_POS       (1<<DIO0_PIN)

#define DIO1_PIN           18
#define DIO1_PIN_POS       (1<<DIO1_PIN)

#define DIO2_PIN           5
#define DIO2_PIN_POS       (1<<DIO2_PIN)

#define MOSI_PIN           2
#define MISO_PIN           4
#define SCK_PIN            17

#define IO_PIN_POS_OUT         (NSS_PIN_POS | RST_PIN_POS);
#define IO_PIN_POS_IN          (DIO0_PIN_POS | DIO1_PIN_POS | DIO2_PIN_POS)

// SPI
spi_device_handle_t spi;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;

static char tag[] = "hal.c";
static const char* TAG = "LMIC_HAL";

//static const u1_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN  };
//static const u1_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

// HAL state
static struct {
    int irqlevel;
    u4_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
    printf("hal io init\n");

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = IO_PIN_POS_OUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = IO_PIN_POS_IN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val) {

}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
  gpio_set_level(NSS_PIN, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
      gpio_config_t io_conf;
      io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_OUTPUT;
      io_conf.pin_bit_mask = (1<<RST_PIN);
      io_conf.pull_down_en = 0;
      io_conf.pull_up_en = 0;
      gpio_config(&io_conf);

      gpio_set_level(RST_PIN, val);
    } else { // keep pin floating
      gpio_config_t io_conf;
      io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_INPUT;
      io_conf.pin_bit_mask = (1<<RST_PIN);
      io_conf.pull_down_en = 0;
      io_conf.pull_up_en = 0;
      gpio_config(&io_conf);
    }
}

static int NUM_DIO = 3;
bool dio_states[3];

void hal_io_check() {
    if (dio_states[0] != gpio_get_level(DIO0_PIN)) {
        dio_states[0] = !dio_states[0];
        if (dio_states[0])
            printf("Fired IRQ0\n");
            radio_irq_handler(0);
    }

    if (dio_states[1] != gpio_get_level(DIO1_PIN)) {
        dio_states[1] = !dio_states[1];
        if (dio_states[1])
            printf("Fired IRQ1\n");
            radio_irq_handler(1);
    }

    if (dio_states[2] != gpio_get_level(DIO2_PIN)) {
        dio_states[2] = !dio_states[2];
        if (dio_states[2])
            printf("Fired IRQ2\n");
            radio_irq_handler(2);
    }
}

// generic EXTI IRQ handler for all channels
void EXTI_IRQHandler () {

}

#if CFG_lmic_clib
void EXTI0_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
    EXTI_IRQHandler();
}
#endif // CFG_lmic_clib

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
    ESP_LOGI(TAG, "Starting initialisation of SPI");
    esp_err_t ret;

    // init master
    spi_bus_config_t buscfg={
        .miso_io_num = MISO_PIN,
        .mosi_io_num = MOSI_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // init device
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 10000000,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 7,
        //.flags = SPI_DEVICE_HALFDUPLEX,
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    assert(ret==ESP_OK);

    ESP_LOGI(TAG, "Finished initialisation of SPI");
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t data) {
    uint8_t rxData = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &data;
    t.rx_buffer = &rxData;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    assert(ret == ESP_OK);

    return (u1_t) rxData;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    ESP_LOGI(TAG, "Starting initialisation of timer");
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;
    timer_config_t config;
    config.alarm_en = 0;
    config.auto_reload = 0;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = 120;
    config.intr_type = 0;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x0);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);

    ESP_LOGI(TAG, "Finished initalisation of timer");
}

u4_t hal_ticks () {
    uint64_t val;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &val);
    ESP_LOGD(TAG, "Getting time ticks");
    uint32_t t = (uint32_t) val;
    //u4_t result = (u4_t) us2osticks(t);
    return t;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
//static u2_t deltaticks (u4_t time) {
//    u4_t t = hal_ticks();
//    s4_t d = time - t;
//    if( d<=0 ) return 0;    // in the past
//    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
//    return (u2_t)d;
//}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {

    ESP_LOGI(TAG, "Wait until");
    s4_t delta = delta_time(time);

    while( delta > 2000){
        vTaskDelay(1);
        delta -= 1000;
    } if(delta > 0){
        vTaskDelay(delta / 1000);
    }
    ESP_LOGI(TAG, "Done waiting until");
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    return 1;
}

void TIM9_IRQHandler () {

}

// -----------------------------------------------------------------------------
// IRQ

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    //ESP_LOGD(TAG, "Disabling interrupts");
    if(x_irq_level < 1){
        //taskDISABLE_INTERRUPTS();
    }
    x_irq_level++;
}

void hal_enableIRQs () {
    //ESP_LOGD(TAG, "Enable interrupts");
    if(--x_irq_level == 0){
        //taskENABLE_INTERRUPTS();
        hal_io_check();
    }
}

void hal_sleep () {

}

// -----------------------------------------------------------------------------

void hal_init () {
    //memset(&HAL, 0x00, sizeof(HAL));
    //hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    //hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

