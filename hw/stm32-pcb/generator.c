#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/arm/stm32_clktree.h"
#include <math.h>
#include <inttypes.h>

#define STM32_PA0 STM32_GPIO_INDEX(STM32_GPIOA_INDEX, 0)

typedef struct  {
    /* Inherited */
    PCBDevice busdev;

    /* Properties */
    uint16_t data_gpio;
    uint8_t mode;
    uint32_t freq;

    /* Private */
    Clk clk;

} GeneratorState;


#define TYPE_STM32_GENERATOR "stm32-periph-generator"
#define STM32_GENERATOR(obj) OBJECT_CHECK(GeneratorState, (obj), TYPE_STM32_GENERATOR)

static void stm32_generator_timer_trigger(void* opaque, int n, int level){
    if (!level)
        return ;

    assert(n == 0);

    GeneratorState *s = STM32_GENERATOR(opaque);
    DeviceState* dev = DEVICE(opaque);
    PCBBus* bus = PCB_BUS(dev->parent_bus);

    // Sine
    int value;
    if (s->mode == 0){ // Sine
        value = ((int)(3300.*(sin(2*M_PI*0.01*s->freq*qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9)+1.))&0xfff);
    }else if (s->mode == 1){ // Square
        value = ((int)(3300.*(sin(2*M_PI*0.01*s->freq*qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9)+1.))&0xfff);
        value = value > 1650 ? 3300 : 0;
    }else if (s->mode == 2){ // Saw
        value = ((int)(3300.*((0.01*s->freq*qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9))) % 3300);
    }else if (s->mode == 3){ // Triangle
        value = ((int)(3300.*((0.01*s->freq*qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9))) % 3300);
        value = value * 2;
        if (value > 3300){
          value = 6600 - value;
        }
    }else{
        value = 0;
    }
    bus->gpio_set_value(bus, s->data_gpio, value);
}

static void stm32_generator_realize(DeviceState *dev, Error **errp)
{
    GeneratorState *s = STM32_GENERATOR(dev);
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
    if (s->freq > 100000000){
      error_setg(errp, "Freq must be lower than 100000000: %d", s->freq);
      return;
    }
    s->clk = clktree_create_src_clk("GEN", s->freq, true);

    qemu_irq* irqs = qemu_allocate_irqs(stm32_generator_timer_trigger, (void *)s, 1);
    clktree_adduser(s->clk, irqs[0]);
}
static Property stm32_generator_properties[] = {
    DEFINE_PROP_UINT16("data-out", GeneratorState, data_gpio, STM32_PA0),
    DEFINE_PROP_UINT8("mode", GeneratorState, mode, 0),
    DEFINE_PROP_UINT32("freq", GeneratorState, freq, true),

    DEFINE_PROP_END_OF_LIST()
};

static void stm32_generator_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    PCBDeviceClass * pc = PCB_DEVICE_CLASS(klass);
    pc->parent_realize = dc->realize;
    dc->realize = stm32_generator_realize;
    dc->props = stm32_generator_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}
static TypeInfo stm32_generator_info = {
    .name  = TYPE_STM32_GENERATOR,
    .parent = TYPE_PCB_DEVICE,
    .instance_size  = sizeof(GeneratorState),
    .class_init = stm32_generator_class_init
};

static void stm32_generator_register_types(void)
{
    type_register_static(&stm32_generator_info);
}

type_init(stm32_generator_register_types)
