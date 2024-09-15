#include <stdlib.h>
#include <stdint.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void target_get_serial_number(char* dest, size_t max_chars) __attribute__((weak));
void target_log(const char* str) __attribute__((weak));
void target_pre_main(void) __attribute__((weak));
void target_pre_detach(bool manifested) __attribute__((weak));
void target_post_setup(void) __attribute__((weak));
size_t target_get_timeout(void) __attribute__((weak));
void target_configure_hardware(void) __attribute__((weak));

static void write_byte(uint8_t addr, uint8_t data);
static uint8_t read_byte(uint8_t addr);
static void init_led_driver(void);

void target_get_serial_number(char* dest, size_t max_chars) {
    (void)max_chars;
    if (dest) {
        dest[0] = '\0';
    }
}

void target_log(const char* str) {
    (void)str;
}

void target_pre_main(void)
{

}

void target_post_setup(void)
{
	/* This runs just before starting to listen to USB */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
	gpio_set(GPIOB, GPIO5);
}

void target_configure_hardware(void)
{
	init_led_driver();
}


void target_pre_detach(bool manifested) {
    /* This runs just before executing a reboot in response to a USB bus reset
       or a detach request.
       If new firmware was successfully downloaded, manifested is set to true.
       This can be used to set flags or blink LEDs before rebooting.
     */
    (void)manifested;
}

size_t target_get_timeout(void)
{
	return 100;
}

static void write_byte(uint8_t addr, uint8_t data) {
    volatile uint32_t reg32;
	i2c_send_start(I2C1);
	while (!((I2C1_SR1 & I2C_SR1_SB)
		& (I2C1_SR2 & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(I2C1, (0x80>>1), I2C_WRITE);
	while (!(I2C1_SR1 & I2C_SR1_ADDR));
	reg32 = I2C1_SR2;
	i2c_send_data(I2C1, addr);
	while (!(I2C1_SR1 & I2C_SR1_BTF)); /* Await ByteTransferedFlag. */
	i2c_send_data(I2C1, data);
	while (!(I2C1_SR1 & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(I2C1);
}

static uint8_t read_byte(uint8_t addr) {
	volatile uint32_t reg32;
	uint8_t data;
	i2c_send_start(I2C1);
	while (!((I2C1_SR1 & I2C_SR1_SB)
		& (I2C1_SR2 & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(I2C1, (0x80>>1), I2C_WRITE);
	while (!(I2C1_SR1 & I2C_SR1_ADDR));
	reg32 = I2C1_SR2;
	i2c_send_data(I2C1, addr);
	while (!(I2C1_SR1 & I2C_SR1_BTF));

	i2c_send_start(I2C1);
	while (!((I2C1_SR1 & I2C_SR1_SB)
		& (I2C1_SR2 & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(I2C1, (0x80>>1), I2C_READ);
	while (!(I2C1_SR1 & I2C_SR1_ADDR));
	reg32 = I2C1_SR2;
	I2C1_CR1 &= ~I2C_CR1_ACK;
	while (!(I2C1_SR1 & I2C_SR1_BTF));
	data = (uint8_t)I2C1_DR;
	I2C1_CR1 &= ~I2C_CR1_POS;
	return data;
}


static void init_led_driver(void) {
    /* initialize PCA9685 */
    rcc_periph_clock_enable(RCC_GPIOA);

    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_reset_pulse(RST_I2C1);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
			  GPIO_I2C1_RE_SCL | GPIO_I2C1_RE_SDA);

    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_I2C1_REMAP);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO15);
	gpio_set(GPIOA, GPIO15);

//    uint8_t freq = 36;
	i2c_peripheral_disable(I2C1);
	i2c_set_clock_frequency(I2C1, 36);
	i2c_set_fast_mode(I2C1);
	i2c_set_ccr(I2C1, 0xb4);
	i2c_set_trise(I2C1, 0x25);
//	i2c_set_own_7bit_slave_address(I2C1, 0x32);
	i2c_peripheral_enable(I2C1);

	write_byte(0x00, 0b00100000u);  // set mode1
	write_byte(0x01, 0b00010101u);  // set mode2
	volatile uint8_t mode1 = read_byte(0x00);
	uint8_t restart = mode1 & 0b1000000;
	write_byte(0x00, 0b01101111u);  // set mode1
    for (uint32_t i = 0; i < 72000; i++) {
        __asm__("nop");
    }
	if(restart) { // restart required
		mode1 |= 0b10000000;
		write_byte(0x00, mode1);
	}
}
