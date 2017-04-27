#include "lmic.h"
//#include "esp_log.h"
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

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
    ASSERT(val == 1 || val == 0);
#ifndef CFG_sx1276mb1_board

#endif

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
    esp_err_t ret;

    buscfg.miso_io_num=MISO_PIN;
    buscfg.mosi_io_num=MOSI_PIN;
    buscfg.sclk_io_num=SCK_PIN;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;

    devcfg.clock_speed_hz=1000000;               //Clock out at 1 MHz
    devcfg.mode=0;                                //SPI mode 0
    devcfg.queue_size=7;                          //We want to be able to queue 7 transactions at a time
    devcfg.spics_io_num=-1;

    printf("Init SPI Bus\n");
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    printf("Add device to Bus\n");
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t mydata = out;
    u1_t mydata1 = 0;
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=1*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=&mydata;               //Data
    t.rx_buffer=&mydata1;

    ret=spi_device_transmit(spi, &t); //Transmit!
    assert(ret==ESP_OK);

    return mydata1;
}

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
  // Nothing to do
}

#define US_PER_OSTICK_EXPONENT 4
#define US_PER_OSTICK (1 << US_PER_OSTICK_EXPONENT)
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)
portMUX_TYPE microsMux = portMUX_INITIALIZER_UNLOCKED;

u4_t micros() {
  static uint32_t lccount = 0;
  static uint32_t overflow = 0;
  uint32_t ccount;
  portENTER_CRITICAL_ISR(&microsMux);
  __asm__ __volatile__ ( "rsr     %0, ccount" : "=a" (ccount) );
  if(ccount < lccount){
      overflow += UINT32_MAX / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
  }
  lccount = ccount;
  portEXIT_CRITICAL_ISR(&microsMux);
  return overflow + (ccount / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
}

u4_t hal_ticks () {
    static uint8_t overflow = 0;
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    uint8_t msb = scaled >> 24;
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    overflow += (msb ^ overflow) & mask;

    //printf("%d\n", scaled | ((uint32_t)overflow << 24));

    return scaled | ((uint32_t)overflow << 24);

    //static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
  // No need to schedule wakeup, since we're not sleeping
  return delta_time(time) <= 0;
}

void TIM9_IRQHandler () {

}

// -----------------------------------------------------------------------------
// IRQ

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
  //irqlevel++;
  hal_io_check();
}

void hal_enableIRQs () {
  //if(irqlevel-- == 0) {
    
  //}
}

void hal_sleep () {

}

// -----------------------------------------------------------------------------

void hal_init () {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

#endif // CFG_lmic_clib
