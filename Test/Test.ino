
#include "mcufriend_shield.h"
#include "glcdfont.c"

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); }
#define WriteData(x) { CD_DATA; write16(x); }

#define wait_ms(ms)  delay(ms)
#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)


#define  BLACK  0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define swap(a, b) { int16_t t = a; a = b; b = t; }

typedef struct Adafruit_GFX_Variables {

    int16_t _width;
    int16_t _height; 
    int16_t cursor_x;
    int16_t cursor_y;
    uint16_t textcolor;
    uint16_t textbgcolor;
    uint8_t textsize;
    uint8_t rotation;
    boolean wrap; 
    uint8_t font_width;
    uint8_t font_height;
    uint8_t font_offset;
    unsigned char *font_addr;

 uint16_t _lcd_ID;
 uint16_t _lcd_rev;
 uint16_t _lcd_madctl;
 uint16_t _lcd_drivOut;
 uint16_t _MC;
 uint16_t _MP;
 uint16_t _MW;
 uint16_t _SC;
 uint16_t _EC;
 uint16_t _SP;
 uint16_t _EP;
 uint16_t  _lcd_xor;
 uint16_t _lcd_capable;
 
}Adafruit_GFX;

 void Adafruit_GFX_Init(Adafruit_GFX* point,int16_t w, int16_t h); 
 void Print(Adafruit_GFX* point,const char *str);
 void drawLine(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
 void drawFastVLine(Adafruit_GFX* point,int16_t x, int16_t y, int16_t h, uint16_t color);
 void drawFastHLine(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, uint16_t color);
 void drawRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
 
 void fillRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
 void fillScreen(Adafruit_GFX* point,uint16_t color);
 void invertDisplay(Adafruit_GFX* point,boolean i);
 void drawCircle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint16_t color);
 void drawCircleHelper(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
 void fillCircle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint16_t color);
 void fillCircleHelper(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);

 void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
 void fillTriangle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
 void drawRoundRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
 void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
 void drawBitmap(Adafruit_GFX* point,int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
 void drawXBitmap(Adafruit_GFX* point,int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
 void drawChar(Adafruit_GFX* point,int16_t x, int16_t y, unsigned char c, uint16_t color,uint16_t bg, uint8_t size);

 void setCursor(Adafruit_GFX* point,int16_t x, int16_t y);
 void setTextColor(Adafruit_GFX* point,uint16_t c);
 void setTextSize(Adafruit_GFX* point,uint8_t s);
 void setTextWrap(Adafruit_GFX* point,boolean w);
 void resetFont(Adafruit_GFX* point);
 
 void setFont(Adafruit_GFX* point,const unsigned char *addr, uint8_t width, uint8_t height, uint8_t offset);
 int16_t height(Adafruit_GFX* point) ;
 int16_t width(Adafruit_GFX* point) ;
 uint8_t getRotation(Adafruit_GFX* point);
 
/****************Prototype of adafruit is ended***************

_________________Prototype of MCUFRIEND__________________*/


  void     reset(Adafruit_GFX* point);      
  void     begin(Adafruit_GFX* point,uint16_t ID); 
  void     drawPixel(Adafruit_GFX* point,int16_t x, int16_t y, uint16_t color); 
 
  void LCD_WR_REG(uint8_t cmd);
  void LCD_WR_DATA(uint8_t dat);
  void     pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
  uint16_t readID(Adafruit_GFX* point);

  uint16_t readReg(Adafruit_GFX* point,uint16_t reg);
  uint32_t readReg32(uint16_t reg);
  int16_t  readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
  
  void     WriteCmdData(Adafruit_GFX* point,uint16_t cmd, uint16_t dat);       
  void     setRotation(Adafruit_GFX* point,uint8_t r);  
  void     setAddrWindow(Adafruit_GFX* point,int16_t x, int16_t y, int16_t x1, int16_t y1);
  void     pushColors(Adafruit_GFX* point,uint16_t *block, int16_t n, bool first);
  void     vertScroll(Adafruit_GFX* point,int16_t top, int16_t scrollines, int16_t offset);

/*______________________________________________________________________________
               ********Definition of Adafruit functions***********/

void Adafruit_GFX_Init(Adafruit_GFX* point,int16_t w, int16_t h)
{
    point-> _width = w;
    point-> _height = h;
    point-> rotation = 0; 
    point-> cursor_x = 0;
    point-> cursor_y = 0;
    point-> textcolor = 0xFFFF;
    point-> textbgcolor = 0xFFFF;
    point-> textsize = 1;
    point-> wrap = true; 
    point-> font_width = 5;
    point-> font_height = 8;
    point-> font_offset = 0;
    point-> font_addr = (unsigned char *)font;     
}


 void write(Adafruit_GFX* point,uint8_t c) {

  uint8_t real_width;
  real_width = point->font_width%2 == 1 ? point->font_width+1 : point->font_width;

  if (c == '\n') {
    point->cursor_y += point->textsize * point->font_height;
    point->cursor_x  = 0;
  }
  else if (c == '\r') {
    // skip em
  }
  else {
    drawChar(point,point->cursor_x, point->cursor_y, c, point->textcolor, point->textbgcolor, point->textsize);
    point->cursor_x += point->textsize * real_width;
    if (point->wrap && (point->cursor_x > (point->_width - point->textsize * real_width))) {
     point-> cursor_y += point->textsize * point->font_height;
      point->cursor_x = 0;
    }
  }
}


void Print(Adafruit_GFX* point, const char *str)
 {
    while (*str)
    {
     write(point,*str++);
    }
 }

void drawLine(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(point,y0, x0, color);
    } else {
      drawPixel(point,x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void drawCircle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint16_t color) 
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(point,x0  , y0+r, color);
  drawPixel(point,x0  , y0-r, color);
  drawPixel(point,x0+r, y0  , color);
  drawPixel(point,x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawPixel(point,x0 + x, y0 + y, color);
    drawPixel(point,x0 - x, y0 + y, color);
    drawPixel(point,x0 + x, y0 - y, color);
    drawPixel(point,x0 - x, y0 - y, color);
    drawPixel(point,x0 + y, y0 + x, color);
    drawPixel(point,x0 - y, y0 + x, color);
    drawPixel(point,x0 + y, y0 - x, color);
    drawPixel(point,x0 - y, y0 - x, color);
  }
}


void drawCircleHelper(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(point,x0 + x, y0 + y, color);
      drawPixel(point,x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      drawPixel(point,x0 + x, y0 - y, color);
      drawPixel(point,x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(point,x0 - y, y0 + x, color);
      drawPixel(point,x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(point,x0 - y, y0 - x, color);
      drawPixel(point,x0 - x, y0 - y, color);
    }
  }
}

void fillCircle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint16_t color)
 {
  drawFastVLine(point,x0, y0-r, 2*r+1, color);
  fillCircleHelper(point,x0, y0, r, 3, 0, color);
 }

void fillCircleHelper(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(point,x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(point,x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(point,x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(point,x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

void drawRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  drawFastHLine(point,x, y, w, color);
  drawFastHLine(point,x, y+h-1, w, color);
  drawFastVLine(point,x, y, h, color);
  drawFastVLine(point,x+w-1, y, h, color);
}

void drawFastVLine(Adafruit_GFX* point,int16_t x, int16_t y, int16_t h, uint16_t color)
{
  drawLine(point,x, y, x, y+h-1, color);
}

void drawFastHLine(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, uint16_t color)
{
  drawLine(point,x, y, x+w-1, y, color);
}

void fillRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  int16_t end;
  
  if(w < 0) 
  {
    w = -w;
    x -= w;
  }
  
  end = x + w;
  
  if(x < 0)
    x = 0;
  if(end > width(point))
    end = width(point);
    
    w = end - x;
    
  if(h < 0) 
  {
    h = -h;
    y -= h;
  }
  
  end = y + h;
  
  if(y < 0)
    y = 0;
  if(end > height(point))
    end = height(point);
    
  h = end - y;

  setAddrWindow(point,x, y, x + w - 1, y + h - 1);
  CS_ACTIVE;
  WriteCmd(point->_MW);
  
  if (h > w) 
  {
    end = h;
    h = w;
    w = end;
  }
  
  uint8_t hi = color >> 8, lo = color & 0xFF;
  
  CD_DATA;
  
  while(h-- > 0) 
  {
    end = w;
    
    #if USING_16BIT_BUS
      #if defined(__SAM3X8E__)
        #define STROBE_16BIT {WR_ACTIVE;WR_ACTIVE;WR_ACTIVE;WR_IDLE;WR_IDLE;}
      #else
        #define STROBE_16BIT {WR_ACTIVE; WR_IDLE;}
      #endif
    
    write_16(color);        //we could just do the strobe
    lo = end & 7;
    hi = end >> 3;
    
    if(hi)
      do
      {
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
      }
      
      while(--hi > 0);
      
      while(lo-- > 0) 
      {
        STROBE_16BIT;
      }
#else
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
#endif
    }
    CS_IDLE;
    if (!(point->_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(point,0, 0, width(point) - 1, height(point) - 1);
}


void fillScreen(Adafruit_GFX* point,uint16_t color)
 {
   fillRect(point,0, 0, point->_width, point->_height, color);
 }

void drawRoundRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  drawFastHLine(point,x+r  , y    , w-2*r, color); // Top
  drawFastHLine(point,x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(point,x    , y+r  , h-2*r, color); // Left
  drawFastVLine(point,x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(point,x+r    , y+r    , r, 1, color);
  drawCircleHelper(point,x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(point,x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(point,x+r    , y+h-r-1, r, 8, color);
}

void fillRoundRect(Adafruit_GFX* point,int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  fillRect(point,x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(point,x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(point,x+r    , y+r, r, 2, h-2*r-1, color);
}

void drawTriangle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  
  drawLine(point,x0, y0, x1, y1, color);
  drawLine(point,x1, y1, x2, y2, color);
  drawLine(point,x2, y2, x0, y0, color);
}

void fillTriangle(Adafruit_GFX* point,int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(point,a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
  int32_t
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(point,a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(point,a, y, b-a+1, color);
  }
}


void drawBitmap(Adafruit_GFX* point,int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
 {
    int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        drawPixel(point,x+i, y+j, color);
      }
    }
  }
 }

void drawXBitmap(Adafruit_GFX* point,int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
  int16_t i, j, byteWidth = (w + 7) / 8;
  
  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i % 8))) {
        drawPixel(point,x+i, y+j, color);
      }
    }
  }
}

void drawChar(Adafruit_GFX* point,int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
  uint8_t real_width;
  real_width = point->font_width%2 == 1 ? point->font_width+1 : point->font_width;

  if((x >= point->_width)            || // Clip right
     (y >= point->_height)           || // Clip bottom
     ((x + real_width * size - 1) < 0) || // Clip left
     ((y + point->font_height * size - 1) < 0))   // Clip top
    return;

  
    for (int8_t k=0; k < point->font_height/8; k++ ) {
      for (int8_t i=0; i<real_width; i++ ) {
        uint8_t line;
        if (i == point->font_width) 
            line = 0x0;
        else 
            line = pgm_read_byte(point->font_addr + (c-(point->font_offset))*(point->font_width)*(point->font_height)/8 + k*(point->font_width) + i);
        for (int8_t j = 0; j<8; j++) {
            if (line & 0x1) {
              if (size == 1) {// default size
                  drawPixel(point,x+i, y + k*8 + j, color);
              }
              else {  // big size
                  fillRect(point,x + (i*size), y + (k*8+j)*size, size, size, color);
              } 
            }
          else if (bg != color) {
            if (size == 1) {// default size
                  drawPixel(point,x+i, y + k*8 + j, bg);
            }
              else {  // big size
                  fillRect(point,x+i*size, y + (k*8+j)*size, size, size, bg);
              }
            }
            line >>= 1;
        }
      }
    }
}

void setCursor(Adafruit_GFX* point,int16_t x, int16_t y)
{
    point->cursor_x = x;
    point->cursor_y = y;
}

void setTextColor(Adafruit_GFX* point,uint16_t c)
{
  point->textcolor = point->textbgcolor = c;
}

void setTextSize(Adafruit_GFX* point,uint8_t s)
{
  point->textsize = (s > 0) ? s : 1;
}

void setTextWrap(Adafruit_GFX* point,boolean w)
{
   point->wrap = w;
}

uint8_t getRotation(Adafruit_GFX* point)
 {
    return point->rotation;
 }


int16_t height(Adafruit_GFX* point)
{
  return point->_height;
}

int16_t width(Adafruit_GFX* point)
{
  return point->_width;
}

void invertDisplay(Adafruit_GFX* point,boolean i) {
  // Do nothing, must be subclassed if supported
}

 void resetFont(Adafruit_GFX* point)
 {
    point->font_width = 5;
    point->font_height = 8;
    point->font_addr = (unsigned char *)font;
    point->font_offset = 0;
 }
 
 void setFont(Adafruit_GFX* point,const unsigned char *addr, uint8_t width, uint8_t height, uint8_t offset)
{
  point->font_width = width;
  point->font_height = height;
  point->font_addr = (unsigned char *)addr;
  point->font_offset = offset;
}
/*********************Function Definition of Adafruit ends here***************/

static uint8_t done_reset, is8347;
int16_t HEIGHT;

void  reset(Adafruit_GFX* point)
{
  done_reset = 1;
  setWriteDir();
  CTL_INIT();
  CS_IDLE;
  RD_IDLE;
  WR_IDLE;
  RESET_IDLE;
  wait_ms(50);
  RESET_ACTIVE;
  wait_ms(100);
  RESET_IDLE;
  wait_ms(100);

  WriteCmdData(point,0xB0, 0x0000);
}

void LCD_WR_REG(uint8_t cmd)
{
  CS_ACTIVE;
  CD_COMMAND;
  write8(cmd);
  CS_IDLE;
}


void LCD_WR_DATA(uint8_t dat)
{
  CS_ACTIVE;
  CD_DATA; 
  write8(dat);
  CS_IDLE;
}

int16_t readGRAM(Adafruit_GFX* point,int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
  uint16_t ret, dummy, _MR = point->_MW;
  int16_t n = w * h, row = 0, col = 0;
  uint8_t r, g, b, tmp;
  
  if(point->_lcd_capable & MIPI_DCS_REV1)
    _MR = 0x2E;
    setAddrWindow(point,x, y, x + w - 1, y + h - 1);
    
    while (n > 0) 
    {
      if(!(point->_lcd_capable & MIPI_DCS_REV1)) 
      {
        WriteCmdData(point,point->_MC, x + col);
        WriteCmdData(point,point->_MP, y + row);
      }
      
      CS_ACTIVE;
      WriteCmd(_MR);
      setReadDir();
      CD_DATA;
      
      if(point->_lcd_capable & READ_NODUMMY) 
      {
        ;
      } 
      else if((point->_lcd_capable & MIPI_DCS_REV1) || point->_lcd_ID == 0x1289) 
      {
        READ_8(r);
      } 
      else 
      {
        READ_16(dummy);
      }
      
      if(point->_lcd_ID == 0x1511)
        READ_8(r);   //extra dummy for R61511
        
      while (n) 
      {
        if(point->_lcd_capable & READ_24BITS)
        {
          READ_8(r);
          READ_8(g);
          READ_8(b);
          
          if(point->_lcd_capable & READ_BGR)
            ret = color565(b, g, r);
           else
            ret = color565(r, g, b);
        } 
        else 
        {
          READ_16(ret);
          
          if(point->_lcd_capable & READ_LOWHIGH)
            ret = (ret >> 8) | (ret << 8);            
          if(point->_lcd_capable & READ_BGR)
            ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
        }
        
        *block++ = ret;
        n--;
        
        if(!(point->_lcd_capable & AUTO_READINC))
          break;
      }
      
      if(++col >= w)
      {
        col = 0;
        
        if(++row >= h)
          row = 0;
      }
      RD_IDLE;
      CS_IDLE;
      setWriteDir();
    }
    if(!(point->_lcd_capable & MIPI_DCS_REV1))
      setAddrWindow(point,0, 0, width(point) - 1, height(point) - 1);
  
  return 0;
}


void WriteCmdData(Adafruit_GFX* point,uint16_t cmd, uint16_t dat)
{
  CS_ACTIVE;
  WriteCmd(cmd);
  WriteData(dat);
  CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
  CS_ACTIVE;
  WriteCmd(cmd);
  
  while(N-- > 0) 
  {
    uint8_t u8 = *block++;
    CD_DATA;
    write8(u8);
    if(N && is8347) 
    {
      cmd++;
      WriteCmd(cmd);
    }
  }
    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
  uint8_t d[4];
  d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
  WriteCmdParamN(cmd, 4, d);
}

void pushCommand(uint16_t cmd, uint8_t * block, int8_t N) 
{
  WriteCmdParamN(cmd, N, block);
}

static uint16_t read16bits(void)
{
  uint16_t ret;
  uint8_t lo;
  
  #if USING_16BIT_BUS
    READ_16(ret);               //single strobe to read whole bus
    
    if(ret > 255)              //ID might say 0x00D3
      return ret;
  #else
    READ_8(ret);
  #endif
  
  READ_8(lo);
  return (ret << 8) | lo;
}

uint16_t readID(Adafruit_GFX* point)
{
   uint16_t ret, ret2;
  uint8_t msb;
  
  ret = readReg(point,0);           //forces a reset() if called before begin()
  
  if(ret == 0x5408)          //the SPFD5408 fails the 0xD3D3 test.
    return 0x5408;
  if (ret == 0x5420)          //the SPFD5420 fails the 0xD3D3 test.
    return 0x5420;
  if (ret == 0x8989)          //SSD1289 is always 8989
    return 0x1289;
    
  ret = readReg(point,0x67);        //HX8347-A
  
  if(ret == 0x4747)
    return 0x8347;
    
  #if defined(SUPPORT_1963) && USING_16BIT_BUS 
    ret = readReg32(0xA1);      //SSD1963: [01 57 61 01]
    
    if(ret == 0x6101)
      return 0x1963;
  #endif
    
  ret = readReg40(0xBF);                                //HX8357B: [xx 01 62 83 57 FF] unsupported
  
  if(ret == 0x9481)          //ILI9481: [xx 02 04 94 81 FF]
    return 0x9481;
  if(ret == 0x1511)          //?R61511: [xx 02 04 15 11] not tested yet
    return 0x1511;
  if(ret == 0x1520)          //?R61520: [xx 01 22 15 20]
    return 0x1520;
  if(ret == 0x1581)          //R61581:  [xx 01 22 15 81]
    return 0x1581;
  if(ret == 0x1400)          //?RM68140:[xx FF 68 14 00] not tested yet
    return 0x6814;
    
  ret = readReg40(0xEF);      //ILI9327: [xx 02 04 93 27 FF] 
    
  if(ret == 0x9327)
    return 0x9327;
    
  ret = readReg32(0x04);      //ST7789V: [85 85 52] 
  
  if(ret == 0x8000)          //HX8357-D
    return 0x8357;
  if(ret == 0x8552)
    return 0x7789;
  
  ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    
  msb = ret >> 8;
  
  if(msb == 0x93 || msb == 0x94)
    return ret;             //0x9488, 9486, 9340, 9341
  if(ret == 0x00D3 || ret == 0xD3D3)
    return ret;             //16-bit write-only bus
    
  return readReg(point,0);  
}


uint16_t readReg(Adafruit_GFX* point,uint16_t reg)
{
  uint16_t ret;
  uint8_t lo;
  
  if(!done_reset)
    reset(point);
    
  CS_ACTIVE;
  WriteCmd(reg);
  setReadDir();
  CD_DATA;
  //READ_16(ret);
  ret = read16bits();
  RD_IDLE;
  CS_IDLE;
  setWriteDir();
  return ret;
}


uint32_t readReg40(uint16_t reg)
{
  uint16_t h, m, l;
  CS_ACTIVE;
  WriteCmd(reg);
  setReadDir();
  CD_DATA;
  h = read16bits();
  m = read16bits();
  l = read16bits();
  RD_IDLE;
  CS_IDLE;
  setWriteDir();
  return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}


uint32_t readReg32(uint16_t reg)
{
  uint16_t h, l;
  
  CS_ACTIVE;
  WriteCmd(reg);
  setReadDir();
  CD_DATA;
  h = read16bits();
  l = read16bits();
  RD_IDLE;
  CS_IDLE;
  setWriteDir();
  return ((uint32_t) h << 16) | (l);
}

void setRotation(Adafruit_GFX* point,uint8_t r)
{
  uint16_t GS, SS, ORG, REV = point->_lcd_rev;
  uint8_t val, d[3];
  
 
  switch(point->rotation) 
  {
    case 0:                    //PORTRAIT:
      val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
      break;
    case 1:                    //LANDSCAPE: 90 degrees
      val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
      break;
    case 2:                    //PORTRAIT_REV: 180 degrees
      val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
      break;
    case 3:                    //LANDSCAPE_REV: 270 degrees
      val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
      break;
  }
  
  if(point->_lcd_capable & INVERT_GS)
    val ^= 0x80;
  if(point->_lcd_capable & INVERT_SS)
    val ^= 0x40;
  if(point->_lcd_capable & INVERT_RGB)
    val ^= 0x08;
  if(point->_lcd_capable & MIPI_DCS_REV1) 
  {
    if(point->_lcd_ID == 0x6814) 
    {
      GS = (val & 0x80) ? (1 << 6) : 0;   //MY
      SS = (val & 0x40) ? (1 << 5) : 0;   //MX
      val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
      d[0] = 0;
      d[1] = GS | SS | 0x02;      //MY, MX
      d[2] = 0x3B;
      WriteCmdParamN(0xB6, 3, d);
      goto common_MC;
    }
    else if(point->_lcd_ID == 0x1963 || point->_lcd_ID == 0x9481 || point->_lcd_ID == 0x1511 || point->_lcd_ID == 0x1581) 
    {
      if(val & 0x80)
        val |= 0x01;    //GS
      if((val & 0x40))
        val |= 0x02;    //SS
      if(point->_lcd_ID == 0x1963)
        val &= ~0xC0;
      if(point->_lcd_ID == 0x9481 || point->_lcd_ID == 0x1581) 
        val &= ~0xD0;
      if(point->_lcd_ID == 0x1511) 
      {
        val &= ~0x10;   //remove ML
        val |= 0xC0;    //force penguin 180 rotation
      }
      
      goto common_MC;
    }
    else if (is8347) 
    {
      point->_MC = 0x02; point->_MP = 0x06; point->_MW = 0x22; point->_SC = 0x02; point->_EC = 0x04; point->_SP = 0x06; point->_EP = 0x08;
      
      if(point->_lcd_ID == 0x5252) 
      {
        val |= 0x02;   //VERT_SCROLLON
        
        if(val & 0x10) val |= 0x04;   //if (ML) SS=1 kludge mirror in XXX_REV modes
      }
      goto common_BGR;
    }
    
    common_MC:
    point->_MC = 0x2A; point->_MP = 0x2B; point->_MW = 0x2C; point->_SC = 0x2A; point->_EC = 0x2A; point->_SP = 0x2B; point->_EP = 0x2B;
    common_BGR:
    WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
   point-> _lcd_madctl = val;
  }
  else 
  {
    switch(point->_lcd_ID) 
    {
      #ifdef SUPPORT_0139
        case 0x0139:
          point->_SC = 0x46; point->_EC = 0x46; point->_SP = 0x48; point->_EP = 0x47;
          goto common_S6D;
      #endif
        case 0x0154:
         point-> _SC = 0x37; point->_EC = 0x36; point-> _SP = 0x39; point->_EP = 0x38;
          common_S6D:
            point->_MC = 0x20; point->_MP = 0x21; point->_MW = 0x22;
            GS = (val & 0x80) ? (1 << 9) : 0;
            SS = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(point,0x01, GS | SS | 0x0028);       // set Driver Output Control
            goto common_ORG;
        case 0x5420:
        case 0x7793:
        case 0x9326:
        case 0xB509:
          point->_MC = 0x200; point->_MP = 0x201; point->_MW = 0x202; point->_SC = 0x210; point->_EC = 0x211; point->_SP = 0x212; point->_EP = 0x213;
          GS = (val & 0x80) ? (1 << 15) : 0;
          uint16_t NL;
          NL = ((432 / 8) - 1) << 9;
          if(point->_lcd_ID == 0x9326 || point->_lcd_ID == 0x5420) 
            NL >>= 1;
          
          WriteCmdData(point,0x400, GS | NL);
          goto common_SS;
        default:
          point->_MC = 0x20; point->_MP = 0x21; point->_MW = 0x22; point->_SC = 0x50; point->_EC = 0x51; point->_SP = 0x52; point->_EP = 0x53;
          GS = (val & 0x80) ? (1 << 15) : 0;
          WriteCmdData(point,0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
          common_SS:
            SS = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(point,0x01, SS);     // set Driver Output Control
          common_ORG:
            ORG = (val & 0x20) ? (1 << 3) : 0;
            if(val & 0x08)
              ORG |= 0x1000;  //BGR
              
            point->_lcd_madctl = ORG | 0x0030;
            WriteCmdData(point,0x03, point->_lcd_madctl);    // set GRAM write direction and BGR=1.
            break;
      #ifdef SUPPORT_1289
        case 0x1289:
          point->_MC = 0x4E; point->_MP = 0x4F; point->_MW = 0x22; point->_SC = 0x44; point->_EC = 0x44; point->_SP = 0x45; point->_EP = 0x46;
          
          if(point->rotation & 1)
            val ^= 0xD0;    // exchange Landscape modes
          
          GS = (val & 0x80) ? (1 << 14) | (1 << 12) : 0;      //called TB (top-bottom)
          SS = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
          ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
          point->_lcd_drivOut = GS | SS | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
          
          if(val & 0x08)
            point->_lcd_drivOut |= 0x0800; //BGR
            WriteCmdData(point,0x01, point->_lcd_drivOut);   // set Driver Output Control
            WriteCmdData(point,0x11, ORG | 0x6070);   // set GRAM write direction.
            break;
      #endif
    }
  }
  
  if((point->rotation & 1) && ((point->_lcd_capable & MV_AXIS) == 0)) 
  {
    uint16_t x;
    x = point->_MC, point->_MC = point->_MP, point->_MP = x;
    x = point->_SC, point->_SC = point->_SP, point->_SP = x;    //.kbv check 0139
    x = point->_EC, point->_EC = point->_EP, point->_EP = x;    //.kbv check 0139
  }
  
  setAddrWindow(point,0, 0, width(point) - 1, height(point) - 1);
 
}


void drawPixel(Adafruit_GFX* point,int16_t x, int16_t y, uint16_t color)
{
  if(x < 0 || y < 0 || x >= width(point) || y >= height(point))
    return;
  
  #if defined(OFFSET_9327)
    if(point->_lcd_ID == 0x9327) 
    {
      if(rotation == 2) y += OFFSET_9327;
      if(rotation == 3) x += OFFSET_9327;
    }
  #endif
    if(point->_lcd_capable & MIPI_DCS_REV1) 
    {
      WriteCmdParam4(point->_MC, x >> 8, x, x >> 8, x);
      WriteCmdParam4(point->_MP, y >> 8, y, y >> 8, y);
    } 
    else 
    {
      WriteCmdData(point,point->_MC, x);
      WriteCmdData(point,point->_MP, y);
    }
    
    WriteCmdData(point,point->_MW, color);
}


void setAddrWindow(Adafruit_GFX* point,int16_t x, int16_t y, int16_t x1, int16_t y1)
{
  #if defined(OFFSET_9327)
    if(point->_lcd_ID == 0x9327) 
    {
      if (rotation == 2) y += OFFSET_9327, y1 += OFFSET_9327;
      if (rotation == 3) x += OFFSET_9327, x1 += OFFSET_9327;
    }
  #endif
    if(point->_lcd_capable & MIPI_DCS_REV1) 
    {
      WriteCmdParam4(point->_MC, x >> 8, x, x1 >> 8, x1);
      WriteCmdParam4(point->_MP, y >> 8, y, y1 >> 8, y1);
    }
    else
    {
      WriteCmdData(point,point->_MC, x);
      WriteCmdData(point,point->_MP, y);
      
      if(point->_lcd_capable & XSA_XEA_16BIT) 
      {
        if(point->rotation & 1)
          y1 = y = (y1 << 8) | y;
        else
          x1 = x = (x1 << 8) | x;
      }
        WriteCmdData(point,point->_SC, x);
        WriteCmdData(point,point->_SP, y);
        WriteCmdData(point,point->_EC, x1);
        WriteCmdData(point,point->_EP, y1);
    }
}


void pushColors(Adafruit_GFX* point,uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(point->_MW);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        write16(color);
    }
    CS_IDLE;
}


void vertScroll(Adafruit_GFX* point,int16_t top, int16_t scrollines, int16_t offset)
{
#if defined(OFFSET_9327)
  if (point->_lcd_ID == 0x9327) {
      if (rotation == 2 || rotation == 3) top += OFFSET_9327;
    }
#endif
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
  if (point->_lcd_ID == 0x9327) bfa += 32;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
  vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    if (point->_lcd_capable & MIPI_DCS_REV1) {
        uint8_t d[6];           // for multi-byte parameters
/*
        if (_lcd_ID == 0x9327) {        //panel is wired for 240x432 
            if (rotation == 2 || rotation == 3) { //180 or 270 degrees
                if (scrollines == HEIGHT) {
                    scrollines = 432;   // we get a glitch but hey-ho
                    vsp -= 432 - HEIGHT;
                }
                if (vsp < 0)
                    vsp += 432;
            }
            bfa = 432 - top - scrollines;
        }
*/
        d[0] = top >> 8;        //TFA
        d[1] = top;
        d[2] = scrollines >> 8; //VSA
        d[3] = scrollines;
        d[4] = bfa >> 8;        //BFA
        d[5] = bfa;
        WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
//        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
    d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
    if (is8347) { 
        d[0] = (offset != 0) ? (point->_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
      WriteCmdParamN(point->_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
    } else if (offset == 0 && (point->_lcd_capable & MIPI_DCS_REV1)) {
      WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
    }
    return;
    }
    // cope with 9320 style variants:
    switch (point->_lcd_ID) {
    case 0x7783:
        WriteCmdData(point,0x61, point->_lcd_rev);   //!NDL, !VLE, REV
        WriteCmdData(point,0x6A, vsp);        //VL#
        break;
    case 0x8230:  
        WriteCmdData(point,0x61, point->_lcd_rev);   //!NDL, !VLE, REV
        WriteCmdData(point,0x6A, vsp);        //VL#
        break;
#ifdef SUPPORT_0139
    case 0x0139:
        WriteCmdData(point,0x41, sea);        //SEA
        WriteCmdData(point,0x42, top);        //SSA
        WriteCmdData(point,0x43, vsp - top);  //SST
        break;
#endif
    case 0x0154:
        WriteCmdData(point,0x31, sea);        //SEA
        WriteCmdData(point,0x32, top);        //SSA
        WriteCmdData(point,0x33, vsp - top);  //SST
        break;
#ifdef SUPPORT_1289
    case 0x1289:
        WriteCmdData(point,0x41, vsp);        //VL#
        break;
#endif
  case 0x5420:
    case 0x7793:
  case 0x9326:
  case 0xB509:
        WriteCmdData(point,0x401, (1 << 1) | point->_lcd_rev);       //VLE, REV 
        WriteCmdData(point,0x404, vsp);       //VL# 
        break;
    default:
        // 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
        WriteCmdData(point,0x61, (1 << 1) | point->_lcd_rev);        //!NDL, VLE, REV
        WriteCmdData(point,0x6A, vsp);        //VL#
        break;
    }
}


#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0xFF

static void init_table(const void *table, int16_t size)
{
    uint8_t *p = (uint8_t *) table, dat[16];
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) {
            delay(len);
            len = 0;
        } else {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}

void begin(Adafruit_GFX* point,uint16_t ID)
{
    int16_t *p16;               //so we can "write" to a const protected variable.
    reset(point);
   point->_lcd_xor = 0;

  point->_lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
   
        static const uint8_t ILI9341_regValues_2_4[] PROGMEM = {        // BOE 2.4"
            0x01, 0,            // software reset
            TFTLCD_DELAY8, 50,  // .kbv will power up with ONLY reset, sleep out, display on
            0x28, 0,            //Display Off
            0xF6, 3, 0x01, 0x01, 0x00,  //Interface Control needs EXTC=1 MV_EOR=0, TM=0, RIM=0
            0xCF, 3, 0x00, 0x81, 0x30,  //Power Control B [00 81 30]
            0xED, 4, 0x64, 0x03, 0x12, 0x81,    //Power On Seq [55 01 23 01]
            0xE8, 3, 0x85, 0x10, 0x78,  //Driver Timing A [04 11 7A]
            0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,      //Power Control A [39 2C 00 34 02]
            0xF7, 1, 0x20,      //Pump Ratio [10]
            0xEA, 2, 0x00, 0x00,        //Driver Timing B [66 00]
            0xB0, 1, 0x00,      //RGB Signal [00] 
            0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
            //            0xB6, 2, 0x0A, 0xA2, 0x27, //Display Function [0A 82 27 XX]    .kbv SS=1  
            0xB4, 1, 0x00,      //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
            0xC0, 1, 0x21,      //Power Control 1 [26]
            0xC1, 1, 0x11,      //Power Control 2 [00]
            0xC5, 2, 0x3F, 0x3C,        //VCOM 1 [31 3C]
            0xC7, 1, 0xB5,      //VCOM 2 [C0]
            0x36, 1, 0x48,      //Memory Access [00]
            0xF2, 1, 0x00,      //Enable 3G [02]
            0x26, 1, 0x01,      //Gamma Set [01]
            0xE0, 15, 0x0f, 0x26, 0x24, 0x0b, 0x0e, 0x09, 0x54, 0xa8, 0x46, 0x0c, 0x17, 0x09, 0x0f, 0x07, 0x00,
            0xE1, 15, 0x00, 0x19, 0x1b, 0x04, 0x10, 0x07, 0x2a, 0x47, 0x39, 0x03, 0x06, 0x06, 0x30, 0x38, 0x0f,
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 150,
            0x29, 0,            //Display On
            0x3A, 1, 0x55,      //Pixel Format [66]
        };
      
#if !defined(USE_SERIAL)
        if (readReg32(0xD3) == 0x0000) {        //weird DealExtreme EXTC=0 shield
            
            point->_lcd_capable |= REV_SCREEN | READ_BGR;
        } else
#endif
        {
            init_table(ILI9341_regValues_2_4, sizeof(ILI9341_regValues_2_4));   //
        }
        
    point->_lcd_rev = ((point->_lcd_capable & REV_SCREEN) != 0);
    setRotation(point,0);            
}


Adafruit_GFX  obj;

void setup(void) 
{
  Adafruit_GFX_Init(&obj,240,320);
  reset(&obj);
  begin(&obj,9600);
}

void loop(void) 
{
  fillScreen(&obj,BLACK);
  unsigned long start = micros();
  setCursor(&obj,0, 20);

  setTextColor(&obj,RED);  
  setTextSize(&obj,1);
  Print(&obj,"HELLO\n");

  setTextColor(&obj,GREEN);  
  setTextSize(&obj,2);
  Print(&obj,"Hello Word\n");

  setTextColor(&obj,BLUE);  
  setTextSize(&obj,3);
  Print(&obj,"Successfull\n\n\n");

  setTextColor(&obj,WHITE);  
  setTextSize(&obj,4);
  Print(&obj,"Escape\n\n");

  setTextColor(&obj,CYAN);  
  setTextSize(&obj,5);
  Print(&obj,"Go On\n\n");
 
  setTextColor(&obj,YELLOW);  
  setTextSize(&obj,2);
  Print(&obj,"ABC c\n");
  
  delay(1000);
}
