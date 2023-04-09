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

    uint8_t addr;
};


#define STM32_GPIO_INDEX(port,pin) ((uint8_t)(((port & 0xf)<<4) | (pin & 0xf)))
#define STM32_PORT_INDEX(gpio) ((gpio>>4) & 0xf)
#define STM32_PIN_INDEX(gpio) (gpio & 0xf)

#endif
