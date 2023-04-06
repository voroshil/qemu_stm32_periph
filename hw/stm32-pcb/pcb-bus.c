#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "monitor/monitor.h"

static char *pcbbus_get_fw_dev_path(DeviceState *dev)
{
    PCBDevice *d = PCB_DEVICE(dev);
    char path[40];
    int off;

    off = snprintf(path, sizeof(path), "%s", qdev_fw_name(dev));
    if (d->ioport_id) {
        snprintf(path + off, sizeof(path) - off, "@%04x", d->ioport_id);
    }

    return g_strdup(path);
}
static void pcbbus_dev_print(Monitor *mon, DeviceState *dev, int indent)
{
    PCBDevice *d = PCB_DEVICE(dev);

    if (d->pcbirq[1] != -1) {
        monitor_printf(mon, "%*sgpio irqs %d,%d\n", indent, "",
                       d->pcbirq[0], d->pcbirq[1]);
    } else if (d->pcbirq[0] != -1) {
        monitor_printf(mon, "%*sgpio irq %d\n", indent, "",
                       d->pcbirq[0]);
    }
}
static void pcb_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->print_dev = pcbbus_dev_print;
    k->get_fw_dev_path = pcbbus_get_fw_dev_path;
}

static const TypeInfo pcb_bus_info = {
    .name = TYPE_PCB_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(PCBBus),
    .class_init = pcb_bus_class_init,
};

static void gpio_device_init(Object *obj)
{
    PCBDevice *dev = PCB_DEVICE(obj);

    dev->pcbirq[0] = -1;
    dev->pcbirq[1] = -1;
}

static Property pcb_bus_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", PCBBus, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};

static void pcbbus_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->fw_name = "gpio";
    dc->props = pcb_bus_properties;
}
static const TypeInfo pcbbus_bridge_info = {
    .name          = "pcbbus-bridge",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusDevice),
    .class_init    = pcbbus_bridge_class_init,
};
static void gpio_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->bus_type = TYPE_PCB_BUS;
}
static const TypeInfo gpio_device_type_info = {
    .name = TYPE_PCB_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(PCBDevice),
    .instance_init = gpio_device_init,
    .abstract = true,
    .class_size = sizeof(PCBDeviceClass),
    .class_init = gpio_device_class_init,
};
static void pcbbus_register_types(void)
{
    type_register_static(&pcb_bus_info);
    type_register_static(&pcbbus_bridge_info);
    type_register_static(&gpio_device_type_info);
}
type_init(pcbbus_register_types)
