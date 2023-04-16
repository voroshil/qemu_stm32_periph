#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "monitor/monitor.h"
#include "hw/irq.h"
#include "qmp-commands.h"
#include "hw/qdev-core.h"
#include "qapi-event.h"
#include "qapi-event.h"

#define TYPE_PCB_BRIDGE "stm32-pcb-bridge"


typedef struct {
  SysBusDevice parent_obj;

  PCBBus pcb_bus;
  qemu_irq out[STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT];
  DeviceState* gpio[STM32_GPIO_COUNT];

} PCBBridgeDevice;

#define PCB_BRIDGE(obj) OBJECT_CHECK(PCBBridgeDevice, (obj), TYPE_PCB_BRIDGE)


void qmp_x_pcb_set_state(int64_t device, const char *unit, int64_t state, Error **errp){
    bool ambig;
    PCBBridgeDevice *bridge = PCB_BRIDGE(object_resolve_path_type("", TYPE_PCB_BRIDGE, &ambig));
    if (!bridge){
        error_setg(errp, "No PCB bridge detected");
        return;
    }
    PCBBus* bus = &bridge->pcb_bus;
    if (!bus->devices[device]){
        error_setg(errp, "No device detected with addr=0x%02x", (uint8_t)device);
        return;
    }
    PCBDevice* dev = PCB_DEVICE(bus->devices[device]);
    if (!dev->set_state){
        error_setg(errp, "Device does not support set_state command");
        return;
    }
    dev->set_state(dev, unit, state, errp);
}

void qmp_x_pcb_report_state(int64_t device, const char *unit, Error **errp){
    bool ambig;
    PCBBridgeDevice *bridge = PCB_BRIDGE(object_resolve_path_type("", TYPE_PCB_BRIDGE, &ambig));
    if (!bridge){
        error_setg(errp, "No PCB bridge detected");
        return;
    }
    PCBBus* bus = &bridge->pcb_bus;
    if (!bus->devices[device]){
        error_setg(errp, "No device detected with addr=0x%02x", (uint8_t)device);
        return;
    }
    PCBDevice* dev = PCB_DEVICE(bus->devices[device]);
    if (!dev->get_state){
        error_setg(errp, "Device does not support get_state command");
        return;
    }

    int res = dev->get_state(dev, unit, errp);
    qapi_event_send_x_pcb(device, unit, res, &error_abort);
}

static void pcb_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->max_dev = 255;
}

static const TypeInfo pcb_bus_info = {
    .name = TYPE_PCB_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(PCBBus),
    .class_init = pcb_bus_class_init,
};

static void pcb_device_realize(DeviceState *dev, Error **errp)
{
    PCBDevice *pd = PCB_DEVICE(dev);

    PCBBus *bus = PCB_BUS(dev->parent_bus);

    if (pd->addr && bus->devices[pd->addr]){
        error_setg(errp, "Address already in use in PCB bus: 0x%02x", pd->addr);
        return;
    }else if(pd->addr){
      bus->devices[pd->addr] = dev;
    }else{
      // find next free addr
      for(; bus->next_addr && bus->devices[bus->next_addr]; bus->next_addr++);
      if (bus->next_addr == 0){
        error_setg(errp, "No free address in PCB bus");
        return;
      }
      qdev_prop_set_uint8(dev, "addr", bus->next_addr);

      bus->devices[pd->addr] = dev;
      bus->next_addr++;
    }
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
static void stm32_pcb_gpio_set_value(PCBBus* bus, uint16_t gpio, int value){
    int port = STM32_PORT_INDEX(gpio);
    if (port >= STM32_GPIO_COUNT)
        return;
    int pin = STM32_PIN_INDEX(gpio);
    if (pin >= STM32_GPIO_PIN_COUNT)
        return;

    PCBBridgeDevice *dev = PCB_BRIDGE(BUS(bus)->parent);
    DeviceState* gpio_dev = dev->gpio[port];
    if (gpio_dev){
if (gpio == 0x10) printf("PCB:XXX:%x:%d\n",gpio,value);
        qemu_irq irq = qdev_get_gpio_in(gpio_dev, pin);
        if (irq){
            uint8_t config = stm32_gpio_get_config_bits(STM32_GPIO(gpio_dev), pin);
            if(config == STM32_GPIO_IN_ANALOG){
                qemu_set_irq(irq, value);
            }else if (value >= V_IH_MV){
                qemu_irq_raise(irq);
            }else if (value <= V_IL_MV){
                qemu_irq_lower(irq);
            }
        }
    }
}
static void stm32_pcb_bridge_init(Object *obj)
{
    int i;
    PCBBridgeDevice *s = PCB_BRIDGE(obj);

    qbus_create_inplace(&s->pcb_bus, sizeof(s->pcb_bus), TYPE_PCB_BUS,
                        DEVICE(s), NULL);

    s->pcb_bus.gpio_connect = stm32_pcb_gpio_connect;
    s->pcb_bus.gpio_set_value = stm32_pcb_gpio_set_value;
    s->pcb_bus.next_addr=1;

    qdev_init_gpio_out(DEVICE(s), s->out, STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT);


    s->gpio[0] = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    s->gpio[1] = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
    s->gpio[2] = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));

    qemu_irq* irqs = qemu_allocate_irqs(stm32_pcb_bridge_irq_handler, (void *)s, STM32_GPIO_COUNT * STM32_GPIO_PIN_COUNT);

    for(i=0; i<STM32_GPIO_PIN_COUNT; i++){
      if (s->gpio[0]){ 
          qdev_connect_gpio_out(s->gpio[0], i, irqs[i]);
      }
      if (s->gpio[1]){
          qdev_connect_gpio_out(s->gpio[1], i, irqs[STM32_GPIO_PIN_COUNT+i]);
      }
      if (s->gpio[2]){
          qdev_connect_gpio_out(s->gpio[2], i, irqs[2*STM32_GPIO_PIN_COUNT+i]);
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

static Property pcb_device_properties[] = {
    DEFINE_PROP_UINT8("addr", PCBDevice, addr, 0),
    DEFINE_PROP_END_OF_LIST()
};
static void pcb_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->bus_type = TYPE_PCB_BUS;
    k->realize = pcb_device_realize;
    k->props = pcb_device_properties;
}
static const TypeInfo pcb_device_type_info = {
    .name = TYPE_PCB_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(PCBDevice),
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
