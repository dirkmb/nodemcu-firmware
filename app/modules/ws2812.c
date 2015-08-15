#include <malloc.h>
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "auxmods.h"
#include "lrotable.h"
#include "c_stdlib.h"
#include "c_string.h"
#include "user_interface.h"

static uint8_t *led_buffer = NULL;
static size_t led_buffer_length = 0;

static inline uint32_t _getCycleCount(void) {
  uint32_t cycles;
  __asm__ __volatile__("rsr %0,ccount":"=a" (cycles));
  return cycles;
}

// This algorithm reads the cpu clock cycles to calculate the correct
// pulse widths. It works in both 80 and 160 MHz mode.
// The values for t0h, t1h, ttot have been tweaked and it doesn't get faster than this.
// The datasheet is confusing and one might think that a shorter pulse time can be achieved.
// The period has to be at least 1.25us, even if the datasheet says:
//   T0H: 0.35 (+- 0.15) + T0L: 0.8 (+- 0.15), which is 0.85<->1.45 us.
//   T1H: 0.70 (+- 0.15) + T1L: 0.6 (+- 0.15), which is 1.00<->1.60 us.
// Anything lower than 1.25us will glitch in the long run.
static void ICACHE_RAM_ATTR ws2812_write(uint8_t pin, uint8_t *pixels, uint32_t length) {
  uint8_t *p, *end, pixel, mask;
  uint32_t t, t0h, t1h, ttot, c, start_time, pin_mask;

  WRITE_PERI_REG(0x60000914, 0x73); //reset watchdog

  pin_mask = 1 << pin;
  p =  pixels;
  end =  p + length;
  pixel = *p++;
  mask = 0x80;
  start_time = 0;
  t0h  = (1000 * system_get_cpu_freq()) / 3333;  // 0.30us (spec=0.35 +- 0.15)
  t1h  = (1000 * system_get_cpu_freq()) / 1666;  // 0.60us (spec=0.70 +- 0.15)
  ttot = (1000 * system_get_cpu_freq()) /  800;  // 1.25us (MUST be >= 1.25)

  while (true) {
    if (pixel & mask) {
        t = t1h;
    } else {
        t = t0h;
    }
    while (((c = _getCycleCount()) - start_time) < ttot); // Wait for the previous bit to finish
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pin_mask);      // Set pin high
    start_time = c;                                       // Save the start time
    while (((c = _getCycleCount()) - start_time) < t);    // Wait for high time to finish
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pin_mask);      // Set pin low
    if (!(mask >>= 1)) {                                  // Next bit/byte
      if (p >= end) {
        break;
      }
      pixel= *p++;
      mask = 0x80;
    }
  }
  WRITE_PERI_REG(0x60000914, 0x73); //reset watchdog
}

// Lua: ws2812.writergb(pin, "string")
// Byte triples in the string are interpreted as R G B values and sent to the hardware as G R B.
// WARNING: this function scrambles the input buffer :
//    a = string.char(255,0,128)
//    ws212.writergb(3,a)
//    =a.byte()
//    (0,255,128)

// ws2812.writergb(4, string.char(255, 0, 0)) uses GPIO2 and sets the first LED red.
// ws2812.writergb(3, string.char(0, 0, 255):rep(10)) uses GPIO0 and sets ten LEDs blue.
// ws2812.writergb(4, string.char(0, 255, 0, 255, 255, 255)) first LED green, second LED white.
static int ICACHE_FLASH_ATTR ws2812_writergb(lua_State* L)
{
  const uint8_t pin = luaL_checkinteger(L, 1);
  size_t length;
  const char *rgb = luaL_checklstring(L, 2, &length);

  // dont modify lua-internal lstring - make a copy instead
  char *buffer = (char *)c_malloc(length);
  c_memcpy(buffer, rgb, length);

  // Ignore incomplete Byte triples at the end of buffer:
  length -= length % 3;

  // Rearrange R G B values to G R B order needed by WS2812 LEDs:
  size_t i;
  for (i = 0; i < length; i += 3) {
    const char r = buffer[i];
    const char g = buffer[i + 1];
    buffer[i] = g;
    buffer[i + 1] = r;
  }

  // Initialize the output pin and wait a bit
  platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
  platform_gpio_write(pin, 0);

  // Send the buffer
  os_intr_lock();
  ws2812_write(pin_num[pin], (uint8_t*) buffer, length);
  os_intr_unlock();

  c_free(buffer);

  return 0;
}

// Lua: ws2812.write(pin, "string")
// Byte triples in the string are interpreted as G R B values.
// This function does not corrupt your buffer.
//
// ws2812.write(4, string.char(0, 255, 0)) uses GPIO2 and sets the first LED red.
// ws2812.write(3, string.char(0, 0, 255):rep(10)) uses GPIO0 and sets ten LEDs blue.
// ws2812.write(4, string.char(255, 0, 0, 255, 255, 255)) first LED green, second LED white.
static int ICACHE_FLASH_ATTR ws2812_writegrb(lua_State* L) {
  const uint8_t pin = luaL_checkinteger(L, 1);
  size_t length;
  const char *buffer = luaL_checklstring(L, 2, &length);

  // Initialize the output pin
  platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
  platform_gpio_write(pin, 0);

  // Send the buffer
  os_intr_lock();
  ws2812_write(pin_num[pin], (uint8_t*) buffer, length);
  os_intr_unlock();

  return 0;
}

// ws2812_move_right
//  move the whole data x leds to the right
static int ICACHE_FLASH_ATTR ws2812_move_right(lua_State *L)
{
  const size_t led_amount = luaL_checkinteger(L, 1);

  if(led_amount < 0)
  {
      return -1;
  }

  memmove(led_buffer + led_amount*3, led_buffer, led_buffer_length - led_amount*3);
  memset(led_buffer, 0, led_amount*3);

  return 0;
}


// ws2812_move_left
//  move the whole data x leds to the left
static int ICACHE_FLASH_ATTR ws2812_move_left(lua_State *L)
{
  const size_t led_amount = luaL_checkinteger(L, 1);

  if(led_amount < 0)
  {
      return -1;
  }

  memmove(led_buffer, led_buffer + led_amount*3, led_buffer_length - led_amount*3);
  memset(led_buffer + led_buffer_length - led_amount*3, 0, led_amount*3);

  return 0;
}


// ws2812_set_leds
//  updates the internal led_buffer at the given position
//  the parameter at 2nd position is a string which is copyed into the internal led_buffer
//  usefull to update more than one led
static int ICACHE_FLASH_ATTR ws2812_set_leds(lua_State *L)
{
  const size_t led_pos = luaL_checkinteger(L, 1);
  size_t length;
  const char *rgb = luaL_checklstring(L, 2, &length);

  if(led_pos >= led_buffer_length)
  {
      return -1;
  }
  if(led_pos < 0)
  {
      return -1;
  }

  // cutof the string if we reach the end of the buffer
  if(length + led_pos > led_buffer_length)
  {
      length = led_buffer_length - led_pos;
  }

  memcpy(led_buffer + led_pos*3, rgb, length);

  return 0;
}

// ws2812_set_led
//  updates the internal led_buffer at the given position
static int ICACHE_FLASH_ATTR ws2812_set_led(lua_State *L)
{
  const size_t led = luaL_checkinteger(L, 1);

  if(led >= led_buffer_length)
  {
      return -1;
  }
  if(led < 0)
  {
      return -1;
  }

  const uint8_t red = luaL_checkinteger(L, 2);
  const uint8_t green = luaL_checkinteger(L, 3);
  const uint8_t blue = luaL_checkinteger(L, 4);

  led_buffer[led*3+0] = green;
  led_buffer[led*3+1] = red;
  led_buffer[led*3+2] = blue;

  return 0;
}

// ws2812_write_buffer
//  writes out the internal led_buffer to the pin parsed as first parameter
static int ICACHE_FLASH_ATTR ws2812_write_buffer(lua_State *L)
{
  const uint8_t pin = luaL_checkinteger(L, 1);

  // Initialize the output pin
  platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
  platform_gpio_write(pin, 0);

  // Send the buffer
  os_intr_lock();
  ws2812_write(pin_num[pin], led_buffer, led_buffer_length);
  os_intr_unlock();

  return 0;
}

// ws2812_init_buffer
//  writes the parameter string into the internal led_buffer
//  realocs memory for new string
static int ICACHE_FLASH_ATTR ws2812_init_buffer(lua_State *L)
{
    const char *buffer = luaL_checklstring(L, 1, &led_buffer_length);

    if(led_buffer)
    {
        led_buffer = (uint8_t*)os_realloc(led_buffer, led_buffer_length);
    }
    else
    {
        led_buffer = (uint8_t*)os_malloc(led_buffer_length * sizeof(uint8_t));
    }

    memcpy(led_buffer, buffer, led_buffer_length);
    return 0;
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"
const LUA_REG_TYPE ws2812_map[] =
{
  { LSTRKEY( "writergb" ), LFUNCVAL( ws2812_writergb )},
  { LSTRKEY( "write" ), LFUNCVAL( ws2812_writegrb )},
  // new
  { LSTRKEY( "write_buffer" ), LFUNCVAL( ws2812_write_buffer )},
  { LSTRKEY( "set_led" ), LFUNCVAL( ws2812_set_led )},
  { LSTRKEY( "set_leds" ), LFUNCVAL( ws2812_set_leds )},
  { LSTRKEY( "move_left" ), LFUNCVAL( ws2812_move_left )},
  { LSTRKEY( "move_right" ), LFUNCVAL( ws2812_move_right )},
  { LSTRKEY( "init_buffer" ), LFUNCVAL( ws2812_init_buffer )},

  // end
  { LNILKEY, LNILVAL}
};

LUALIB_API int luaopen_ws2812(lua_State *L) {
  // TODO: Make sure that the GPIO system is initialized
  LREGISTER(L, "ws2812", ws2812_map);
  return 1;
}
