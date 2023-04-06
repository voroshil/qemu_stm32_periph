#ifndef HW_STM32_PCB_BUS_H
#define HW_STM32_PCB_BUS_H

/* PCB bus */

#include "exec/ioport.h"
#include "exec/memory.h"
#include "hw/qdev.h"
#include "hw/arm/stm32.h"

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
} PCBDeviceClass;

struct PCBBus {
    /*< private >*/
    BusState parent_obj;
    /*< public >*/

    MemoryRegion *address_space_io;
    qemu_irq *irqs;
    stm32_periph_t  periph;
};

struct PCBDevice {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    uint32_t pcbirq[2];
    int nirqs;
    int ioport_id;
};

#endif
