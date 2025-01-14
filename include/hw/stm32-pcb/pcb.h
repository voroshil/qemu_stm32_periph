#ifndef HW_STM32_PCB_BUS_H
#define HW_STM32_PCB_BUS_H

/* PCB bus */

#include "exec/ioport.h"
#include "exec/memory.h"
#include "hw/qdev.h"
#include "hw/arm/stm32.h"

typedef struct PCBBus PCBBus;
typedef struct PCBDevice PCBDevice;

#define TYPE_PCB_DEVICE "pcb-device"
#define PCB_DEVICE(obj) \
     OBJECT_CHECK(PCBDevice, (obj), TYPE_PCB_DEVICE)
#define PCB_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(PCBDeviceClass, (klass), TYPE_PCB_DEVICE)
#define PCB_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(PCBDeviceClass, (obj), TYPE_PCB_DEVICE)

#define TYPE_PCB_BUS "PCB"
#define PCB_BUS(obj) OBJECT_CHECK(PCBBus, (obj), TYPE_PCB_BUS)


typedef struct PCBDeviceClass {
    DeviceClass parent_class;
    DeviceRealize parent_realize;
} PCBDeviceClass;

struct PCBBus {
    /*< private >*/
    BusState parent_obj;
    /*< public >*/

    MemoryRegion *address_space_io;
    qemu_irq *irqs;
    DeviceState* devices[256];
    uint8_t next_addr;

    int nirqs;
    void (*gpio_connect)(PCBBus* bus, uint16_t n, qemu_irq irq);
    void (*gpio_set_value)(PCBBus* bus, uint16_t n, int value);
};

struct PCBDevice {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    void (*set_state)(PCBDevice* dev, const char *unit, int64_t state, Error **errp);
    int (*get_state)(PCBDevice* dev, const char *unit, Error **errp);

    uint8_t addr;
};


#define STM32_GPIO_INDEX(port,pin) ((uint8_t)(((port & 0xf)<<4) | (pin & 0xf)))
#define STM32_PORT_INDEX(gpio) ((gpio>>4) & 0xf)
#define STM32_PIN_INDEX(gpio) (gpio & 0xf)


#define DEBUG_STM32_PCB 1

#ifdef DEBUG_STM32_PCB
#define PCB_DPRINTF(fmt, ...)                         \
    do {                                                \
    qemu_timeval tv;                                    \
    qemu_gettimeofday(&tv);                             \
    int64_t v = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); \
    int32 n = v % 1000000000;                              \
    n /= 1000;         \
    v /= 1000000000;                                         \
    fprintf(stderr, "[%ld.%06ld,%ld.%06d]:" fmt , tv.tv_sec, tv.tv_usec, v, n, ## __VA_ARGS__);  \
    } while (0)
#else
#define PCB_DPRINTF(fmt, ...)
#endif

#endif
