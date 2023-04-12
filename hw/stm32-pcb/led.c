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
    bool active_low;

    /* Private */

    uint8_t gpio_value;
} LedState;


#define TYPE_STM32_LED "stm32-periph-led"
#define STM32_LED(obj) OBJECT_CHECK(LedState, (obj), TYPE_STM32_LED)

static void stm32_led_irq_handler(void *opaque, int n, int level)
{
    LedState *s = (LedState *)opaque;
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
    if (s->active_low){
      active = !active;
    }
    if (changed && active){
      qapi_event_send_x_pcb(pd->addr, "LED", 1, &error_abort);
      printf("LED[%s]: on\n", ((DeviceState*)s)->id);
    }else if (changed){
      qapi_event_send_x_pcb(pd->addr, "LED", 0, &error_abort);
      printf("LED[%s]: off\n", ((DeviceState*)s)->id);
    }

    s->gpio_value = new_value;
}

static void stm32_led_reset(DeviceState *dev)
{
    LedState *s = STM32_LED(dev);

    s->gpio_value = 0;
}

static void stm32_led_realize(DeviceState *dev, Error **errp)
{
    LedState *s = STM32_LED(dev);
    PCBBus* bus = PCB_BUS(dev->parent_bus);

    PCBDeviceClass *pc = PCB_DEVICE_GET_CLASS(dev);
    if (pc->parent_realize){
      pc->parent_realize(dev, errp);
    }
    if (STM32_PORT_INDEX(s->data_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for DATA: 0x%02x", s->data_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->data_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for DATA: 0x%02x", s->data_gpio);
      return;
    }

    qemu_irq* gpio_irq = qemu_allocate_irqs(stm32_led_irq_handler, (void *)s, 1);

    bus->gpio_connect(bus, s->data_gpio, gpio_irq[0]);

    stm32_led_reset((DeviceState *)s);
}
static Property stm32_led_properties[] = {
    DEFINE_PROP_UINT16("data-in", LedState, data_gpio, STM32_PC13),
    DEFINE_PROP_BOOL("active_low", LedState, active_low, true),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_led_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    PCBDeviceClass * pc = PCB_DEVICE_CLASS(klass);
    pc->parent_realize = dc->realize;
    dc->realize = stm32_led_realize;
    dc->reset = stm32_led_reset;
    dc->props = stm32_led_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}
static TypeInfo stm32_led_info = {
    .name  = TYPE_STM32_LED,
    .parent = TYPE_PCB_DEVICE,
    .instance_size  = sizeof(LedState),
    .class_init = stm32_led_class_init
};

static void stm32_led_register_types(void)
{
    type_register_static(&stm32_led_info);
}

type_init(stm32_led_register_types)
