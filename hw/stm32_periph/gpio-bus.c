#include "hw/sysbus.h"
#include "hw/stm32_periph/gpio.h"

static char *gpiobus_get_fw_dev_path(DeviceState *dev)
{
    GPIODevice *d = GPIO_DEVICE(dev);
    char path[40];
    int off;

    off = snprintf(path, sizeof(path), "%s", qdev_fw_name(dev));
    if (d->ioport_id) {
        snprintf(path + off, sizeof(path) - off, "@%04x", d->ioport_id);
    }

    return g_strdup(path);
}
static void gpiobus_dev_print(Monitor *mon, DeviceState *dev, int indent)
{
    GPIODevice *d = GPIO_DEVICE(dev);

    if (d->gpioirq[1] != -1) {
        monitor_printf(mon, "%*sgpio irqs %d,%d\n", indent, "",
                       d->gpioirq[0], d->gpioirq[1]);
    } else if (d->gpioirq[0] != -1) {
        monitor_printf(mon, "%*sgpio irq %d\n", indent, "",
                       d->gpioirq[0]);
    }
}
static void gpio_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->print_dev = gpiobus_dev_print;
    k->get_fw_dev_path = gpiobus_get_fw_dev_path;
}

static const TypeInfo gpio_bus_info = {
    .name = TYPE_GPIO_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(GPIOBus),
    .class_init = gpio_bus_class_init,
};

static void gpio_device_init(Object *obj)
{
    GPIODevice *dev = GPIO_DEVICE(obj);

    dev->gpioirq[0] = -1;
    dev->gpioirq[1] = -1;
}

static void gpiobus_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->fw_name = "gpio";
}
static const TypeInfo gpiobus_bridge_info = {
    .name          = "gpiobus-bridge",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusDevice),
    .class_init    = gpiobus_bridge_class_init,
};
static void gpio_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->bus_type = TYPE_GPIO_BUS;
}
static const TypeInfo gpio_device_type_info = {
    .name = TYPE_GPIO_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(GPIODevice),
    .instance_init = gpio_device_init,
    .abstract = true,
    .class_size = sizeof(GPIODeviceClass),
    .class_init = gpio_device_class_init,
};
static void gpiobus_register_types(void)
{
    type_register_static(&gpio_bus_info);
    type_register_static(&gpiobus_bridge_info);
    type_register_static(&gpio_device_type_info);
}
type_init(gpiobus_register_types)
