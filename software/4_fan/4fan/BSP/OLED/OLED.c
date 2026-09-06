#include "OLED.h"
#include "i2c.h"
#include <string.h>
#include <stdlib.h>

#define OLED_ADDRESS       0x78U
#define OLED_I2C_TIMEOUT   100U
#define OLED_I2C_HANDLE    hi2c2
#define OLED_PAGE          8U
#define OLED_ROW           (OLED_PAGE * 8U)
#define OLED_COLUMN        128U

uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

void OLED_Send(uint8_t *data, uint8_t length)
{
  (void)HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_ADDRESS, data,
                                length, OLED_I2C_TIMEOUT);
}

void OLED_SendCmd(uint8_t command)
{
  static uint8_t buffer[2];

  buffer[1] = command;
  OLED_Send(buffer, sizeof(buffer));
}

void OLED_NewFrame(void)
{
  memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

void OLED_ShowFrame(void)
{
  static uint8_t buffer[OLED_COLUMN + 1U];

  buffer[0] = 0x40U;
  for (uint8_t page = 0U; page < OLED_PAGE; page++)
  {
    OLED_SendCmd((uint8_t)(0xB0U + page));
    OLED_SendCmd(0x00U);
    OLED_SendCmd(0x10U);
    memcpy(&buffer[1], OLED_GRAM[page], OLED_COLUMN);
    OLED_Send(buffer, sizeof(buffer));
  }
}

void OLED_Init(void)
{
  static const uint8_t initCommands[] =
  {
    0xAEU, 0x20U, 0x02U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
    0x81U, 0xDFU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
    0x00U, 0xD5U, 0xF0U, 0xD9U, 0x22U, 0xDAU, 0x12U, 0xDBU,
    0x20U, 0x8DU, 0x14U
  };

  for (uint8_t i = 0U; i < sizeof(initCommands); i++)
  {
    OLED_SendCmd(initCommands[i]);
  }

  OLED_NewFrame();
  OLED_ShowFrame();
  OLED_SendCmd(0xAFU);
}

void OLED_DisPlay_On(void)
{
  OLED_SendCmd(0x8DU);
  OLED_SendCmd(0x14U);
  OLED_SendCmd(0xAFU);
}

void OLED_DisPlay_Off(void)
{
  OLED_SendCmd(0x8DU);
  OLED_SendCmd(0x10U);
  OLED_SendCmd(0xAEU);
}

void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color)
{
  if (x >= OLED_COLUMN || y >= OLED_ROW)
  {
    return;
  }

  if (color == OLED_COLOR_NORMAL)
  {
    OLED_GRAM[y / 8U][x] |= (uint8_t)(1U << (y % 8U));
  }
  else
  {
    OLED_GRAM[y / 8U][x] &= (uint8_t)~(1U << (y % 8U));
  }
}

void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data,
                       uint8_t start, uint8_t end, OLED_ColorMode color)
{
  uint8_t mask;

  if (page >= OLED_PAGE || column >= OLED_COLUMN)
  {
    return;
  }

  mask = (uint8_t)(((0xFFU >> (7U - (end - start))) << start) & 0xFFU);
  if (color != OLED_COLOR_NORMAL)
  {
    data = (uint8_t)~data;
  }
  OLED_GRAM[page][column] = (uint8_t)((OLED_GRAM[page][column] & ~mask) |
                                      (data & mask));
}

void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data,
                  OLED_ColorMode color)
{
  if (page >= OLED_PAGE || column >= OLED_COLUMN)
  {
    return;
  }

  OLED_GRAM[page][column] = color == OLED_COLOR_NORMAL ? data : (uint8_t)~data;
}

void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t length,
                       OLED_ColorMode color)
{
  uint8_t page = y / 8U;
  uint8_t bit = y % 8U;

  if (bit + length > 8U)
  {
    OLED_SetByte_Fine(page, x, (uint8_t)(data << bit), bit, 7U, color);
    OLED_SetByte_Fine(page + 1U, x, (uint8_t)(data >> (8U - bit)), 0U,
                      (uint8_t)(length + bit - 9U), color);
  }
  else
  {
    OLED_SetByte_Fine(page, x, (uint8_t)(data << bit), bit,
                      (uint8_t)(bit + length - 1U), color);
  }

  // 保留旧接口在反色模式下的逐像素覆盖行为。
  for (uint8_t i = 0U; i < length; i++)
  {
    OLED_SetPixel(x, (uint8_t)(y + i), !((data >> i) & 0x01U));
  }
}

void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color)
{
  uint8_t page = y / 8U;
  uint8_t bit = y % 8U;

  OLED_SetByte_Fine(page, x, (uint8_t)(data << bit), bit, 7U, color);
  if (bit != 0U)
  {
    OLED_SetByte_Fine(page + 1U, x, (uint8_t)(data >> (8U - bit)), 0U,
                      (uint8_t)(bit - 1U), color);
  }
}

void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data,
                   uint8_t width, uint8_t height, OLED_ColorMode color)
{
  uint8_t fullRows = height / 8U;
  uint8_t remainingBits = height % 8U;

  for (uint8_t column = 0U; column < width; column++)
  {
    for (uint8_t row = 0U; row < fullRows; row++)
    {
      OLED_SetBits(x + column, (uint8_t)(y + row * 8U),
                   data[column + row * width], color);
    }
  }

  if (remainingBits != 0U)
  {
    uint16_t fullBytes = (uint16_t)width * fullRows;
    for (uint8_t column = 0U; column < width; column++)
    {
      OLED_SetBits_Fine(x + column, (uint8_t)(y + fullRows * 8U),
                        data[fullBytes + column], remainingBits, color);
    }
  }
}

void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char character,
                         const ASCIIFont *font, OLED_ColorMode color)
{
  uint16_t charSize = (uint16_t)((font->h + 7U) / 8U) * font->w;

  OLED_SetBlock(x, y, font->chars + (character - ' ') * charSize,
                font->w, font->h, color);
}

void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *string,
                           const ASCIIFont *font, OLED_ColorMode color)
{
  while (*string != '\0')
  {
    OLED_PrintASCIIChar(x, y, *string++, font, color);
    x = (uint8_t)(x + font->w);
  }
}

uint8_t _OLED_GetUTF8Len(char *string)
{
  if ((string[0] & 0x80) == 0x00)
  {
    return 1U;
  }
  if ((string[0] & 0xE0) == 0xC0)
  {
    return 2U;
  }
  if ((string[0] & 0xF0) == 0xE0)
  {
    return 3U;
  }
  if ((string[0] & 0xF8) == 0xF0)
  {
    return 4U;
  }
  return 0U;
}

void OLED_PrintString(uint8_t x, uint8_t y, char *string,
                      const Font *font, OLED_ColorMode color)
{
  uint16_t index = 0U;
  uint8_t glyphSize = (uint8_t)(((font->h + 7U) / 8U) * font->w + 4U);

  while (string[index] != '\0')
  {
    uint8_t utf8Length = _OLED_GetUTF8Len(&string[index]);
    uint8_t found = 0U;

    if (utf8Length == 0U)
    {
      break;
    }

    for (uint8_t glyph = 0U; glyph < font->len; glyph++)
    {
      const uint8_t *head = font->chars + glyph * glyphSize;
      if (memcmp(&string[index], head, utf8Length) == 0)
      {
        OLED_SetBlock(x, y, head + 4U, font->w, font->h, color);
        x = (uint8_t)(x + font->w);
        index = (uint16_t)(index + utf8Length);
        found = 1U;
        break;
      }
    }

    if (found == 0U)
    {
      OLED_PrintASCIIChar(x, y,
                          utf8Length == 1U ? string[index] : ' ',
                          font->ascii, color);
      x = (uint8_t)(x + font->ascii->w);
      index = (uint16_t)(index + utf8Length);
    }
  }
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                   OLED_ColorMode color)
{
  static uint8_t temp;

  if (x1 == x2)
  {
    if (y1 > y2)
    {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++)
    {
      OLED_SetPixel(x1, y, color);
    }
  }
  else if (y1 == y2)
  {
    if (x1 > x2)
    {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++)
    {
      OLED_SetPixel(x, y1, color);
    }
  }
  else
  {
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1;
    int16_t y = y1;
    int16_t error = 0;

    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy)
    {
      for (x = x1; x != x2; x += ux)
      {
        OLED_SetPixel(x, y, color);
        error += dy;
        if ((error << 1) >= dx)
        {
          y += uy;
          error -= dx;
        }
      }
    }
    else
    {
      for (y = y1; y != y2; y += uy)
      {
        OLED_SetPixel(x, y, color);
        error += dx;
        if ((error << 1) >= dy)
        {
          x += ux;
          error -= dy;
        }
      }
    }
  }
}

void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        OLED_ColorMode color)
{
  OLED_DrawLine(x, y, x + width, y, color);
  OLED_DrawLine(x, y + height, x + width, y + height, color);
  OLED_DrawLine(x, y, x, y + height, color);
  OLED_DrawLine(x + width, y, x + width, y + height, color);
}

void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t width,
                              uint8_t height, OLED_ColorMode color)
{
  for (uint8_t row = 0U; row < height; row++)
  {
    OLED_DrawLine(x, y + row, x + width, y + row, color);
  }
}

void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                       uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
  OLED_DrawLine(x1, y1, x2, y2, color);
  OLED_DrawLine(x2, y2, x3, y3, color);
  OLED_DrawLine(x3, y3, x1, y1, color);
}

void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                             uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
  uint8_t a;
  uint8_t b;
  uint8_t y;
  uint8_t last = 0U;

  if (y1 > y2)
  {
    a = y2;
    b = y1;
  }
  else
  {
    a = y1;
    b = y2;
  }

  y = a;
  for (; y <= b; y++)
  {
    if (y <= y3)
    {
      OLED_DrawLine(x1 + (y - y1) * (x2 - x1) / (y2 - y1), y,
                    x1 + (y - y1) * (x3 - x1) / (y3 - y1), y, color);
    }
    else
    {
      last = y - 1U;
      break;
    }
  }
  for (; y <= b; y++)
  {
    OLED_DrawLine(x2 + (y - y2) * (x3 - x2) / (y3 - y2), y,
                  x1 + (y - last) * (x3 - x1) / (y3 - last), y, color);
  }
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t radius,
                     OLED_ColorMode color)
{
  int16_t a = 0;
  int16_t b = radius;
  int16_t delta = 3 - (radius << 1);

  while (a <= b)
  {
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x + b, y - a, color);
    OLED_SetPixel(x - a, y + b, color);
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x - a, y - b, color);
    OLED_SetPixel(x + b, y + a, color);
    OLED_SetPixel(x + a, y - b, color);
    OLED_SetPixel(x + a, y + b, color);
    OLED_SetPixel(x - b, y + a, color);
    a++;
    if (delta < 0)
    {
      delta += 4 * a + 6;
    }
    else
    {
      delta += 10 + 4 * (a - b);
      b--;
    }
    OLED_SetPixel(x + a, y + b, color);
  }
}

void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t radius,
                           OLED_ColorMode color)
{
  int16_t a = 0;
  int16_t b = radius;
  int16_t delta = 3 - (radius << 1);

  while (a <= b)
  {
    for (int16_t i = x - b; i <= x + b; i++)
    {
      OLED_SetPixel(i, y + a, color);
      OLED_SetPixel(i, y - a, color);
    }
    for (int16_t i = x - a; i <= x + a; i++)
    {
      OLED_SetPixel(i, y + b, color);
      OLED_SetPixel(i, y - b, color);
    }
    a++;
    if (delta < 0)
    {
      delta += 4 * a + 6;
    }
    else
    {
      delta += 10 + 4 * (a - b);
      b--;
    }
  }
}

void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b,
                      OLED_ColorMode color)
{
  int xpos = 0;
  int ypos = b;
  int a2 = a * a;
  int b2 = b * b;
  int delta = b2 + a2 * (0.25 - b);

  while (a2 * ypos > b2 * xpos)
  {
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    if (delta < 0)
    {
      delta += b2 * ((xpos << 1) + 3);
      xpos++;
    }
    else
    {
      delta += b2 * ((xpos << 1) + 3) + a2 * (-(ypos << 1) + 2);
      xpos++;
      ypos--;
    }
  }

  delta = b2 * (xpos + 0.5) * (xpos + 0.5) +
          a2 * (ypos - 1) * (ypos - 1) - a2 * b2;
  while (ypos > 0)
  {
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    if (delta < 0)
    {
      delta += b2 * ((xpos << 1) + 2) + a2 * (-(ypos << 1) + 3);
      xpos++;
      ypos--;
    }
    else
    {
      delta += a2 * (-(ypos << 1) + 3);
      ypos--;
    }
  }
}

void OLED_DrawImage(uint8_t x, uint8_t y, const Image *image,
                    OLED_ColorMode color)
{
  OLED_SetBlock(x, y, image->data, image->w, image->h, color);
}

void OLED_Test(void)
{
  uint8_t txMessage[] = {
    0x40U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU
  };

  OLED_SendCmd(0xB0U);
  OLED_SendCmd(0x10U);
  OLED_SendCmd(0x00U);
  (void)HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_ADDRESS, txMessage,
                                sizeof(txMessage), 10000U);
}
