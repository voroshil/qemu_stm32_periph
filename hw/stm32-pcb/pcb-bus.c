#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "monitor/monitor.h"
#include "hw/irq.h"

#define TYPE_PCB_BRIDGE "stm32-pcb-bridge"


typedef struct {
  SysBusDevice parent_obj;

  PCBBus pcb_bus;
  qemu_irq out[STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT];
} PCBBridgeDevice;

#define PCB_BRIDGE(obj) OBJECT_CHECK(PCBBridgeDevice, (obj), TYPE_PCB_BRIDGE)

static char *stm32_pcb_get_fw_dev_path(DeviceState *dev)
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
static void stm32_pcb_dev_print(Monitor *mon, DeviceState *dev, int indent)
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

    k->print_dev = stm32_pcb_dev_print;
    k->get_fw_dev_path = stm32_pcb_get_fw_dev_path;
    k->max_dev = 256;
}

static const TypeInfo pcb_bus_info = {
    .name = TYPE_PCB_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(PCBBus),
    .class_init = pcb_bus_class_init,
};

static void pcb_device_init(Object *obj)
{
    PCBDevice *dev = PCB_DEVICE(obj);

    dev->pcbirq[0] = -1;
    dev->pcbirq[1] = -1;
}

//static Property pcb_bus_properties[] = {
//    DEFINE_PROP_PERIPH_T("periph", PCBBus, periph, STM32_PERIPH_UNDEFINED),
//    DEFINE_PROP_END_OF_LIST()
//};

static void stm32_pcb_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "STM32 GPIO to PCB Bridge";
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->fw_name = "pcb";
//    dc->props = pcb_bus_properties;
    dc->cannot_instantiate_with_device_add_yet = false;
}

static void stm32_pcb_bridge_irq_handler(void* opaque, int n, int level){
    PCBBridgeDevice *s = (PCBBridgeDevice*)opaque;
    if (s->out[n]) {
        qemu_set_irq(s->out[n], level);
    }
}
static void stm32_pcb_gpio_connect(PCBBus* bus, uint16_t gpio, qemu_irq irq){
    if (gpio >= STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT)
      return;

    PCBBridgeDevice *dev = PCB_BRIDGE(BUS(bus)->parent);

    if (dev->out[gpio]){
        dev->out[gpio] = qemu_irq_split(dev->out[gpio], irq);
    }else{
        dev->out[gpio] = irq;
    }
}
static void stm32_pcb_bridge_init(Object *obj)
{
    int i;
    PCBBridgeDevice *s = PCB_BRIDGE(obj);

    qbus_create_inplace(&s->pcb_bus, sizeof(s->pcb_bus), TYPE_PCB_BUS,
                        DEVICE(s), NULL);

    s->pcb_bus.gpio_connect = stm32_pcb_gpio_connect;

    qdev_init_gpio_out(DEVICE(s), s->out, STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT);


    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_b = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));

    qemu_irq* irqs = qemu_allocate_irqs(stm32_pcb_bridge_irq_handler, (void *)s, STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT);

    for(i=0; i<STM32_GPIO_PIN_COUNT; i++){
      if (gpio_a){ 
          qdev_connect_gpio_out(gpio_a, i, irqs[i]);
      }
      if (gpio_b){
          qdev_connect_gpio_out(gpio_b, i, irqs[STM32_GPIO_PIN_COUNT+i]);
      }
      if (gpio_c){
          qdev_connect_gpio_out(gpio_c, i, irqs[2*STM32_GPIO_PIN_COUNT+i]);
      }
    }
    for(i=0; i<STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT; i++){
      s->out[i] = 0;
    }
    s->pcb_bus.irqs = irqs;
    s->pcb_bus.nirqs = STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT;

}
static const TypeInfo stm32_pcb_bridge_info = {
    .name          = TYPE_PCB_BRIDGE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PCBBridgeDevice),
    .instance_init = stm32_pcb_bridge_init,
    .class_init    = stm32_pcb_bridge_class_init,
};
static void pcb_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->bus_type = TYPE_PCB_BUS;
}
static const TypeInfo pcb_device_type_info = {
    .name = TYPE_PCB_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(PCBDevice),
    .instance_init = pcb_device_init,
    .abstract = true,
    .class_size = sizeof(PCBDeviceClass),
    .class_init = pcb_device_class_init,
};
static void stm32_pcb_register_types(void)
{
    type_register_static(&pcb_bus_info);
    type_register_static(&stm32_pcb_bridge_info);
    type_register_static(&pcb_device_type_info);
}
type_init(stm32_pcb_register_types)
