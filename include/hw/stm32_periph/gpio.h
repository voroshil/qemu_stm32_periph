#ifndef HW_STM32_GPIO_BUS_H
#define HW_STM32_GPIO_BUS_H

/* GPIO bus */

#include "exec/ioport.h"
#include "exec/memory.h"
#include "hw/qdev.h"

#define GPIO_NUM_IRQS 16

#define TYPE_GPIO_DEVICE "gpio-device"
#define GPIO_DEVICE(obj) \
     OBJECT_CHECK(GPIODevice, (obj), TYPE_GPIO_DEVICE)
#define GPIO_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(GPIODeviceClass, (klass), TYPE_GPIO_DEVICE)
#define GPIO_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(GPIODeviceClass, (obj), TYPE_GPIO_DEVICE)

#define TYPE_GPIO_BUS "GPIO"
#define GPIO_BUS(obj) OBJECT_CHECK(GPIOBus, (obj), TYPE_GPIO_BUS)


typedef struct GPIODeviceClass {
    DeviceClass parent_class;
} GPIODeviceClass;

struct GPIOBus {
    /*< private >*/
    BusState parent_obj;
    /*< public >*/

    MemoryRegion *address_space_io;
    qemu_irq *irqs;
};

struct GPIODevice {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    uint32_t gpioirq[2];
    int nirqs;
    int ioport_id;
};

#endif
