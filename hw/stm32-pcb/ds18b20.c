#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "qapi-event.h"

#include <inttypes.h>
#define STM32_PC13 STM32_GPIO_INDEX(STM32_GPIOC_INDEX, 13)

typedef struct  {
    /* Inherited */
    PCBDevice busdev;

    /* Properties */
    uint16_t data_gpio;

    /* Private */

    uint8_t gpio_value;
} Ds18b20State;


#define TYPE_STM32_DS18B20 "stm32-periph-ds18b20"
#define STM32_DS18B20(obj) OBJECT_CHECK(Ds18b20State, (obj), TYPE_STM32_DS18B20)

static void stm32_ds18b20_irq_handler(void *opaque, int n, int level)
{
    Ds18b20State *s = (Ds18b20State *)opaque;
    PCBDevice *pd = PCB_DEVICE(s);

    uint8_t new_value = s->gpio_value;

    assert(n == 0);

    if(level){
      new_value |= (1<<n);
    }else{
      new_value &= ~(1<<n);
    }

    uint8_t changed = s->gpio_value ^ new_value;
    uint8_t active = new_value;
    if (changed && active){
      qapi_event_send_x_pcb(pd->addr, "DS18B20", 1, &error_abort);
      printf("DS18B20[%s]: on\n", ((DeviceState*)s)->id);
    }else if (changed){
      qapi_event_send_x_pcb(pd->addr, "DS18B20", 0, &error_abort);
      printf("DS18B20[%s]: off\n", ((DeviceState*)s)->id);
    }

    s->gpio_value = new_value;
}

static int ds18b20_get_state(PCBDevice* dev, const char* unit, Error **errp){
  Ds18b20State *s = STM32_DS18B20(dev);

  if (strncmp("DS18B20", unit, 3) || unit[3] != 0){
    error_setg(errp, "Unsupported unit: %s", unit);
    return 0;
  }
  return s->gpio_value ? 0 : 1;
}
static void stm32_ds18b20_reset(DeviceState *dev)
{
    Ds18b20State *s = STM32_DS18B20(dev);

    s->gpio_value = 0;
}

static void stm32_ds18b20_realize(DeviceState *dev, Error **errp)
{
    Ds18b20State *s = STM32_DS18B20(dev);
    PCBBus* bus = PCB_BUS(dev->parent_bus);

    PCBDeviceClass *pc = PCB_DEVICE_GET_CLASS(dev);
    if (pc->parent_realize){
      pc->parent_realize(dev, errp);
    }

    s->busdev.get_state = ds18b20_get_state;

    if (STM32_PORT_INDEX(s->data_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for DATA: 0x%02x", s->data_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->data_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for DATA: 0x%02x", s->data_gpio);
      return;
    }

    qemu_irq* gpio_irq = qemu_allocate_irqs(stm32_ds18b20_irq_handler, (void *)s, 1);

    bus->gpio_connect(bus, s->data_gpio, gpio_irq[0]);

    stm32_ds18b20_reset((DeviceState *)s);
}
static Property stm32_ds18b20_properties[] = {
    DEFINE_PROP_UINT16("data", Ds18b20State, data_gpio, STM32_PC13),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_ds18b20_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    PCBDeviceClass * pc = PCB_DEVICE_CLASS(klass);
    pc->parent_realize = dc->realize;
    dc->realize = stm32_ds18b20_realize;
    dc->reset = stm32_ds18b20_reset;
    dc->props = stm32_ds18b20_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}
static TypeInfo stm32_ds18b20_info = {
    .name  = TYPE_STM32_DS18B20,
    .parent = TYPE_PCB_DEVICE,
    .instance_size  = sizeof(Ds18b20State),
    .class_init = stm32_ds18b20_class_init
};

static void stm32_ds18b20_register_types(void)
{
    type_register_static(&stm32_ds18b20_info);
}

type_init(stm32_ds18b20_register_types)
