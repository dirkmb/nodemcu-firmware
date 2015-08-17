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

  WRITE_PERI_REG(0x60000914, 0x73); //reset watchdog, just to be sure the device does not reboot if we have long strips

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
    // Wait for the previous bit to finish, if we need to wait long enough to reset the watchdog timer, reset it. WDT causes restarts with long led strips.
    while (((c = _getCycleCount()) - start_time) < ttot) if(c - start_time > 10){ WRITE_PERI_REG(0x60000914, 0x73); } ;
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pin_mask);      // Set pin high
    start_time = c;                                       // Save the start time
    //  Wait for high time to finish; if we need to wait long enough to reset the watchdog timer, reset it. WDT causes restarts with long led strips.
    while (((c = _getCycleCount()) - start_time) < t) if(c - start_time > 10){ WRITE_PERI_REG(0x60000914, 0x73); } ;
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pin_mask);      // Set pin low
    if (!(mask >>= 1)) {                                  // Next bit/byte
      if (p >= end) {
        break;
      }
      pixel= *p++;
      mask = 0x80;
    }
  }
  WRITE_PERI_REG(0x60000914, 0x73); //reset watchdog, just to be sure the device does not reboot if we have long strips
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

  // Save the buffer
  if(led_buffer)
  {
    led_buffer = (uint8_t*)os_realloc(led_buffer, led_buffer_length);
  }
  else
  {
    led_buffer = (uint8_t*)os_malloc(led_buffer_length * sizeof(uint8_t));
  }
  memcpy(led_buffer, buffer, led_buffer_length);

  // Send the buffer
  os_intr_lock();
  ws2812_write(pin_num[pin], (uint8_t*) led_buffer, length);
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

  // Save the buffer
  if(led_buffer)
  {
    led_buffer = (uint8_t*)os_realloc(led_buffer, led_buffer_length);
  }
  else
  {
    led_buffer = (uint8_t*)os_malloc(led_buffer_length * sizeof(uint8_t));
  }
  memcpy(led_buffer, buffer, led_buffer_length);


  // Send the buffer
  os_intr_lock();
  ws2812_write(pin_num[pin], (uint8_t*) led_buffer, length);
  os_intr_unlock();

  return 0;
}


// ws2812_move_right
//  move the whole data x leds to the right
// ws2812.move_right(5) updates the internal buffer and moves all leds 5 positions to the right
static int ICACHE_FLASH_ATTR ws2812_move_right(lua_State *L)
{
  const size_t led_amount = luaL_checkinteger(L, 1);

  if(led_amount < 0)
  {
      return 0;
  }

  memmove(led_buffer + led_amount*3, led_buffer, led_buffer_length - led_amount*3);
  memset(led_buffer, 0, led_amount*3);

  return 0;
}


// ws2812_move_left
//  move the whole data x leds to the left
// ws2812.move_left(5) updates the internal buffer and moves all leds 5 positions to the left
static int ICACHE_FLASH_ATTR ws2812_move_left(lua_State *L)
{
  const size_t led_amount = luaL_checkinteger(L, 1);

  if(led_amount < 0)
  {
      return 0;
  }

  memmove(led_buffer, led_buffer + led_amount*3, led_buffer_length - led_amount*3);
  memset(led_buffer + led_buffer_length - led_amount*3, 0, led_amount*3);

  return 0;
}


// ws2812_get_led
//  updates the internal led_buffer at the given position, adds the values to the current values in the buffer
// ws2812.get_led(0) returns red,green,blue of the first led
static int ICACHE_FLASH_ATTR ws2812_get_led(lua_State *L)
{
  const size_t led = luaL_checkinteger(L, 1);

  if(led*3+3 > led_buffer_length)
  {
      return 0;
  }
  if(led < 0)
  {
      return 0;
  }

  const uint8_t blue = led_buffer[led*3+0];
  const uint8_t red = led_buffer[led*3+1];
  const uint8_t green = led_buffer[led*3+2];

  lua_pushinteger( L, red );
  lua_pushinteger( L, green );
  lua_pushinteger( L, blue );
  return 3;
}


// ws2812_add_led
//  updates the internal led_buffer at the given position, adds the values to the current values in the buffer
// ws2812.add_led(0, 128,0,0) binary or the value of the first led with the given rgb values. in this case the first led would become more red (if not already red)
static int ICACHE_FLASH_ATTR ws2812_add_led(lua_State *L)
{
  const size_t led = luaL_checkinteger(L, 1);

  if(led*3+3 > led_buffer_length)
  {
      return 0;
  }
  if(led < 0)
  {
      return 0;
  }

  const uint8_t red = luaL_checkinteger(L, 2);
  const uint8_t green = luaL_checkinteger(L, 3);
  const uint8_t blue = luaL_checkinteger(L, 4);

  led_buffer[led*3+0] |= blue;
  led_buffer[led*3+1] |= red;
  led_buffer[led*3+2] |= green;

  return 0;
}

// ws2812_add_leds
//  updates the internal led_buffer at the given position
//  the parameter at 2nd position is a string which is added to the current values in the internal led_buffer
//  usefull to update more than one led
// ws2812.add_leds(0, string.char(128,0,0)) same als add_led but takes a lua string with multiple values for more than one led (it is not rgb its grb like always)
static int ICACHE_FLASH_ATTR ws2812_add_leds(lua_State *L)
{
  const size_t led_pos = luaL_checkinteger(L, 1);
  size_t length;
  const char *brg = luaL_checklstring(L, 2, &length);
  int i;

  if(led_pos*3 > led_buffer_length)
  {
      return 0;
  }
  if(led_pos < 0)
  {
      return 0;
  }

  // cutof the string if we reach the end of the buffer
  if(length + led_pos > led_buffer_length)
  {
      length = led_buffer_length - led_pos;
  }

  for(i = 0; i < length ; i++)
  {
      led_buffer[led_pos*3 + i] |= brg[i];
  }

  return 0;
}

// ws2812_set_led
//  updates the internal led_buffer at the given position
// ws2812.set_led(0, 128,0,0) updates the value of the first led to 128 red
static int ICACHE_FLASH_ATTR ws2812_set_ledrgb(lua_State *L)
{
  const size_t led = luaL_checkinteger(L, 1);

  if(led*3+3 > led_buffer_length)
  {
      return 0;
  }
  if(led < 0)
  {
      return 0;
  }

  const uint8_t red = luaL_checkinteger(L, 2);
  const uint8_t green = luaL_checkinteger(L, 3);
  const uint8_t blue = luaL_checkinteger(L, 4);

  led_buffer[led*3+0] = blue;
  led_buffer[led*3+1] = red;
  led_buffer[led*3+2] = green;

  return 0;
}

// ws2812_set_leds
//  updates the internal led_buffer at the given position
//  the parameter at 2nd position is a string which is copyed into the internal led_buffer
//  usefull to update more than one led
//  (blue, red, green)
// ws2812.set_leds(0, string.char(128,0,0)) like set_led but takes a lua string for multiple led values. and it is not rgb its is grb
static int ICACHE_FLASH_ATTR ws2812_set_ledsbrg(lua_State *L)
{
  const size_t led_pos = luaL_checkinteger(L, 1);
  size_t length;
  const char *brg = luaL_checklstring(L, 2, &length);

  if(led_pos*3 > led_buffer_length)
  {
      return 0;
  }
  if(led_pos < 0)
  {
      return 0;
  }

  // cutof the string if we reach the end of the buffer
  if(length + led_pos > led_buffer_length)
  {
      length = led_buffer_length - led_pos;
  }

  memcpy(led_buffer + led_pos*3, brg, length);

  return 0;
}


// ws2812_write_buffer
//  writes out the internal led_buffer to the pin parsed as first parameter
// ws2812.write_buffer(4) writes the values of the internal buffer to the led strip connected at pin 4
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


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"
const LUA_REG_TYPE ws2812_map[] =
{
  { LSTRKEY( "writergb" ), LFUNCVAL( ws2812_writergb )},
  { LSTRKEY( "write" ), LFUNCVAL( ws2812_writegrb )},
  // new
  { LSTRKEY( "write_buffer" ), LFUNCVAL( ws2812_write_buffer )},
  { LSTRKEY( "get_led" ), LFUNCVAL( ws2812_get_led )},
  { LSTRKEY( "add_led" ), LFUNCVAL( ws2812_add_led )},
  { LSTRKEY( "add_leds" ), LFUNCVAL( ws2812_add_leds )},
  { LSTRKEY( "set_led" ), LFUNCVAL( ws2812_set_ledrgb )},
  { LSTRKEY( "set_leds" ), LFUNCVAL( ws2812_set_ledsbrg )},
  { LSTRKEY( "move_left" ), LFUNCVAL( ws2812_move_left )},
  { LSTRKEY( "move_right" ), LFUNCVAL( ws2812_move_right )},
#ifdef UART_TEST
  // uart
  { LSTRKEY( "uart_append_color" ), LFUNCVAL( ws28xx_color )},
  { LSTRKEY( "uart_fill_fifo" ), LFUNCVAL( fill_fifo )},
  { LSTRKEY( "uart_init" ), LFUNCVAL( ws28xx_init )},
#endif

  // end
  { LNILKEY, LNILVAL}
};

LUALIB_API int luaopen_ws2812(lua_State *L) {
  // TODO: Make sure that the GPIO system is initialized
  LREGISTER(L, "ws2812", ws2812_map);
  return 1;
}
