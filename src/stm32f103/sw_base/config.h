#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#ifndef APP_BASE_ADDRESS
#define APP_BASE_ADDRESS (0x08000000 + BOOTLOADER_OFFSET)
#endif
#ifndef FLASH_SIZE_OVERRIDE
#define FLASH_SIZE_OVERRIDE 0x20000
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE  1024
#endif
#ifndef DFU_UPLOAD_AVAILABLE
#define DFU_UPLOAD_AVAILABLE 1
#endif
#ifndef DFU_DOWNLOAD_AVAILABLE
#define DFU_DOWNLOAD_AVAILABLE 1
#endif

#ifndef HAVE_LED
#define HAVE_LED 0
#endif
// #ifndef LED_OPEN_DRAIN
// #define LED_OPEN_DRAIN 0
// #endif
// #ifndef LED_GPIO_PORT
// #define LED_GPIO_PORT GPIOA
// #endif
// #ifndef LED_GPIO_PIN
// #define LED_GPIO_PIN GPIO0
// #endif

#ifndef HAVE_BUTTON
#define HAVE_BUTTON 0
#endif
// #ifndef BUTTON_ACTIVE_HIGH
// #define BUTTON_ACTIVE_HIGH 0
// #endif
// #ifndef BUTTON_GPIO_PORT
// #define BUTTON_GPIO_PORT GPIOA
// #endif
// #ifndef BUTTON_GPIO_PIN
// #define BUTTON_GPIO_PIN GPIO0
// #endif

#ifndef BUTTON_SAMPLE_DELAY_CYCLES
#define BUTTON_SAMPLE_DELAY_CYCLES 1440000
#endif

#ifndef HAVE_USB_PULLUP_CONTROL
#define HAVE_USB_PULLUP_CONTROL 0
#endif

#endif
