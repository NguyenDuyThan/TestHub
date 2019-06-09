#include "TFT_22_ILI9225.h"
#include <spi.h>
#include "string.h"
#include "stdlib.h"
#include "stm32f10x.h"


// Constructor when using software SPI.  All output pins are configurable.
//us
void delay(__IO uint32_t nCount)
{
	while (nCount--)
	{
	}
}

uint8_t bitRead(uint8_t databit, uint8_t pos){
	return (databit>> pos & 1);
}
/*void TFT_22_ILI9225(uint8_t rst, uint8_t rs, uint8_t cs, uint8_t sdi, uint8_t clk, uint8_t led) {
	_rst  = rst;
	_rs   = rs;
	_cs   = cs;
	_sdi  = sdi;
	_clk  = clk;
	_led  = led;
	hwSPI = FALSE;
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
void TFT_22_ILI9225(uint8_t rst, uint8_t rs, uint8_t cs, uint8_t led) {
	_rst  = rst;
	_rs   = rs;
	_cs   = cs;
	_sdi  = _clk = 0;
	_led  = led;
	hwSPI = TRUE;
}*/


void orientCoordinates(uint16_t x1, uint16_t y1) {

	switch (_orientation) {
	case 0:  // ok
		break;
	case 1: // ok
		y1 = _maxY - y1 - 1;
		swap(x1, y1);
		break;
	case 2: // ok
		x1 = _maxX - x1 - 1;
		y1 = _maxY - y1 - 1;
		break;
	case 3: // ok
		x1 = _maxX - x1 - 1;
		swap(x1, y1);
		break;
	}
}


void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	orientCoordinates(x0, y0);
	orientCoordinates(x1, y1);

	if (x1<x0) swap(x0, x1);
	if (y1<y0) swap(y0, y1);

	writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1,x1);
	writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2,x0);

	writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1,y1);
	writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2,y0);

	writeRegister(ILI9225_RAM_ADDR_SET1,x0);
	writeRegister(ILI9225_RAM_ADDR_SET2,y0);

	writeCommand(0x00, 0x22);
}


void initTFT() {
	LCD_LED_SET;
	// Initialization Code
	TFT_RST_SET;//digitalWrite(_rst, 1); // Pull the reset pin high to release the ILI9225C from the reset status
	delay(1000);
	TFT_RST_RESET;//digitalWrite(_rst, 0); // Pull the reset pin low to reset ILI9225
	delay(10000);
	TFT_RST_SET;//digitalWrite(_rst, 1); // Pull the reset pin high to release the ILI9225C from the reset status
	delay(50000);

	/* Start Initial Sequence */
	/* Set SS bit and direction output from S528 to S1 */
	writeRegister(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB

	writeRegister(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	writeRegister(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
	writeRegister(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
	writeRegister(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
	delay(40000);

	// Power-on sequence
	writeRegister(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	writeRegister(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
	writeRegister(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
	writeRegister(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
	writeRegister(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
	delay(10000);
	writeRegister(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
	delay(50000);
	writeRegister(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
	writeRegister(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
	writeRegister(ILI9225_ENTRY_MODE, 0x1030); // set GRAM write direction and BGR=1.
	writeRegister(ILI9225_DISP_CTRL1, 0x0000); // Display off
	writeRegister(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
	writeRegister(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
	writeRegister(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
	writeRegister(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
	writeRegister(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
	writeRegister(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
	writeRegister(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

	/* Set GRAM area */
	writeRegister(ILI9225_GATE_SCAN_CTRL, 0x0000);
	writeRegister(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB);
	writeRegister(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000);
	writeRegister(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000);
	writeRegister(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB);
	writeRegister(ILI9225_PARTIAL_DRIVING_POS2, 0x0000);
	writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);
	writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000);
	writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB);
	writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

	/* Set GAMMA curve */
	writeRegister(ILI9225_GAMMA_CTRL1, 0x0000);
	writeRegister(ILI9225_GAMMA_CTRL2, 0x0808);
	writeRegister(ILI9225_GAMMA_CTRL3, 0x080A);
	writeRegister(ILI9225_GAMMA_CTRL4, 0x000A);
	writeRegister(ILI9225_GAMMA_CTRL5, 0x0A08);
	writeRegister(ILI9225_GAMMA_CTRL6, 0x0808);
	writeRegister(ILI9225_GAMMA_CTRL7, 0x0000);
	writeRegister(ILI9225_GAMMA_CTRL8, 0x0A00);
	writeRegister(ILI9225_GAMMA_CTRL9, 0x0710);
	writeRegister(ILI9225_GAMMA_CTRL10, 0x0710);

	writeRegister(ILI9225_DISP_CTRL1, 0x0012);
	delay(50000);
	writeRegister(ILI9225_DISP_CTRL1, 0x1017);

	setBacklight(TRUE);
	setOrientation(0);

	// Initialize variables
	setBackgroundColor( COLOR_BLACK );
	clear();
}

void clear() {
	uint8_t old = _orientation;
	setOrientation(0);
	fillRectangle(0, 0, _maxX - 1, _maxY - 1, COLOR_BLACK);
	setOrientation(old);
	delay(10000);
}

void clearxy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	uint8_t old = _orientation;
	setOrientation(0);
	fillRectangle(x1, y1, x2, y2, COLOR_BLACK);
	setOrientation(old);
	delay(10000);
}


void invert(BOOLEAN flag) {
	writeCommand(0x00, flag ? ILI9225C_INVON : ILI9225C_INVOFF);
}


void setBacklight(BOOLEAN flag) {
	if(flag == TRUE) LCD_LED_SET;//if (_led) digitalWrite(_led, flag);
	else LCD_LED_RESET;
}


void setDisplay(BOOLEAN flag) {
	if (flag == TRUE) {
		writeRegister(0x00ff, 0x0000);
		writeRegister(ILI9225_POWER_CTRL1, 0x0000);
		delay(50000);
		writeRegister(ILI9225_DISP_CTRL1, 0x1017);
		delay(200000);
	} else {
		writeRegister(0x00ff, 0x0000);
		writeRegister(ILI9225_DISP_CTRL1, 0x0000);
		delay(50000);
		writeRegister(ILI9225_POWER_CTRL1, 0x0003);
		delay(200000);
	}
}


void setOrientation(uint8_t orientation) {

	_orientation = orientation % 4;

	switch (_orientation) {
	case 0:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;
		break;
	case 1:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	case 2:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;
		break;
	case 3:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	}
}


uint8_t getOrientation() {
	return _orientation;
}


void drawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	drawLine(x1, y1, x1, y2, color);
	drawLine(x1, y1, x2, y1, color);
	drawLine(x1, y2, x2, y2, color);
	drawLine(x2, y1, x2, y2, color);
}


void fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	setWindow(x1, y1, x2, y2);

	for(uint16_t t=(y2 - y1 + 1) * (x2 - x1 + 1); t > 0; t--)
		writeData(color >> 8, color);
}


void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {

	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0, y0 + r, color);
	drawPixel(x0, y0-  r, color);
	drawPixel(x0 + r, y0, color);
	drawPixel(x0 - r, y0, color);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		drawPixel(x0 + x, y0 + y, color);
		drawPixel(x0 - x, y0 + y, color);
		drawPixel(x0 + x, y0 - y, color);
		drawPixel(x0 - x, y0 - y, color);
		drawPixel(x0 + y, y0 + x, color);
		drawPixel(x0 - y, y0 + x, color);
		drawPixel(x0 + y, y0 - x, color);
		drawPixel(x0 - y, y0 - x, color);
	}
}


void fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color) {

//	int16_t f = 1 - radius;
//	int16_t ddF_x = 1;
//	int16_t ddF_y = -2 * radius;
//	int16_t x = 0;
//	int16_t y = radius;
	int16_t r = 0;

	for(uint8_t i = 0; i<radius; i++){
		drawCircle(x0, y0, r+i, color);
	}

//	while (x<y) {
//		if (f >= 0) {
//			y--;
//			ddF_y += 2;
//			f += ddF_y;
//		}
//		x++;
//		ddF_x += 2;
//		f += ddF_x;
//
//		drawLine(x0 + x, y0 + y, x0 - x, y0 + y, color); // bottom   drawLine
//		drawLine(x0 + x, y0 - y, x0 - x, y0 - y, color); // top
//		drawLine(x0 + y, y0 - x, x0 + y, y0 + x, color); // right
//		drawLine(x0 - y, y0 - x, x0 - y, y0 + x, color); // left
//	}
//	fillRectangle(x0-x, y0-y, x0+x, y0+y, color);

//
//	int16_t f = 1 - radius;
//	int16_t ddF_x = 1;
//	int16_t ddF_y = -2 * radius;
//	int16_t x = 0;
//	int16_t y = radius;
//
//	drawPixel(x0, y0 + radius, color);
//	drawPixel(x0, y0-  radius, color);
//	drawPixel(x0 + radius, y0, color);
//	drawPixel(x0 - radius, y0, color);
//
//	while (x<y) {
//		if (f >= 0) {
//			y--;
//			ddF_y += 2;
//			f += ddF_y;
//		}
//		x++;
//		ddF_x += 2;
//		f += ddF_x;
////		drawLine(x0 - x,  y0 - y, x0 + x, y0 + y, color);
//		drawPixel(x0 + x, y0 + y, color);
//		drawPixel(x0 - x, y0 + y, color);
////		drawLine(x0 - x,  y0 - y, x0 + x, y0 + y, color);
//		drawPixel(x0 + x, y0 - y, color);
//		drawPixel(x0 - x, y0 - y, color);
//		drawLine(x0 + y,  y0 + x, x0 - y, y0 - x, color);
//		drawPixel(x0 + y, y0 + x, color);
//		drawPixel(x0 - y, y0 + x, color);
//		drawLine(x0 - y,  y0 - x, x0 + y, y0 + x, color);
//		drawPixel(x0 + y, y0 - x, color);
//		drawPixel(x0 - y, y0 - x, color);
//	}
}


void drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	// Classic Bresenham algorithm
	int16_t steep = abs(y2 - y1) > abs(x2 - x1);
	int16_t dx, dy;

	if (steep) {
		swap(x1, y1);
		swap(x2, y2);
	}

	if (x1 > x2) {
		swap(x1, x2);
		swap(y1, y2);
	}

	dx = x2 - x1;
	dy = abs(y2 - y1);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y1 < y2) ystep = 1;
	else ystep = -1;


	for (; x1<=x2; x1++) {
		if (steep) drawPixel(y1, x1, color);
		else       drawPixel(x1, y1, color);

		err -= dy;
		if (err < 0) {
			y1 += ystep;
			err += dx;
		}
	}
}


void drawPixel(uint16_t x1, uint16_t y1, uint16_t color) {

	if((x1 < 0) || (x1 >= _maxX) || (y1 < 0) || (y1 >= _maxY)) return;

	setWindow(x1, y1, x1+1, y1+1);
	orientCoordinates(x1, y1);
	writeData(color >> 8, color);
}


uint16_t maxX() {
	return _maxX;
}


uint16_t maxY() {
	return _maxY;
}


uint16_t setColor(uint8_t red8, uint8_t green8, uint8_t blue8) {
	// rgb16 = red5 green6 blue5
	return (red8 >> 3) << 11 | (green8 >> 2) << 5 | (blue8 >> 3);
}


void splitColor(uint16_t rgb, uint8_t red, uint8_t green, uint8_t blue) {
	// rgb16 = red5 green6 blue5
	red   = (rgb & 0b1111100000000000) >> 11 << 3;
	green = (rgb & 0b0000011111100000) >>  5 << 2;
	blue  = (rgb & 0b0000000000011111)       << 3;
}


void swap(uint16_t a, uint16_t b) {
	uint16_t w = a;
	a = b;
	b = w;
}

// Utilities
void writeCommand(uint8_t HI, uint8_t LO) {
	TFT_RS_RESET;//digitalWrite(_rs, LOW);
	TFT_CS_RESET;//digitalWrite(_cs, LOW);
	transfer(HI);
	transfer(LO);
	TFT_CS_SET;//digitalWrite(_cs, HIGH);
}


void writeData(uint8_t HI, uint8_t LO) {
	TFT_RS_SET;//digitalWrite(_rs, HIGH);
	TFT_CS_RESET;//digitalWrite(_cs, LOW);
	transfer(HI);
	transfer(LO);
	TFT_CS_SET;//digitalWrite(_cs, HIGH);
}


void writeRegister(uint16_t reg, uint16_t data) {
	writeCommand(reg >> 8, reg & 255);
	writeData(data >> 8, data & 255);
}


void drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color) {
	drawLine(x1, y1, x2, y2, color);
	drawLine(x2, y2, x3, y3, color);
	drawLine(x3, y3, x1, y1, color);
}


void fillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color) {

	uint16_t a, b, y, last;

	// Sort coordinates by Y order (y3 >= y2 >= y1)
	if (y1 > y2) {
		swap(y1, y2); swap(x1, x2);
	}
	if (y2 > y3) {
		swap(y3, y2); swap(x3, x2);
	}
	if (y1 > y2) {
		swap(y1, y2); swap(x1, x2);
	}

	if (y1 == y3) { // Handle awkward all-on-same-line case as its own thing
		a = b = x1;
		if (x2 < a)      a = x2;
		else if (x2 > b) b = x2;
		if (x3 < a)      a = x3;
		else if (x3 > b) b = x3;
			drawLine(a, y1, b, y1, color);
		return;
	}

	uint16_t	dx11 = x2 - x1,
				dy11 = y2 - y1,
				dx12 = x3 - x1,
				dy12 = y3 - y1,
				dx22 = x3 - x2,
				dy22 = y3 - y2,
				sa   = 0,
				sb   = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y2=y3 (flat-bottomed triangle), the scanline y2
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y2 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y1=y2
	// (flat-topped triangle).
	if (y2 == y3) last = y2;   // Include y2 scanline
	else          last = y2 - 1; // Skip it

	for (y = y1; y <= last; y++) {
	a   = x1 + sa / dy11;
	b   = x1 + sb / dy12;
	sa += dx11;
	sb += dx12;
	/* longhand:
	a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
	b = x1 + (x3 - x1) * (y - y1) / (y3 - y1);
	*/
	if (a > b) swap(a,b);
		drawLine(a, y, b, y, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y2=y3.
	sa = dx22 * (y - y2);
	sb = dx12 * (y - y1);
	for (; y<=y3; y++) {
		a   = x2 + sa / dy22;
		b   = x1 + sb / dy12;
		sa += dx22;
		sb += dx12;
		/* longhand:
		a = x2 + (x3 - x2) * (y - y2) / (y3 - y2);
		b = x1 + (x3 - x1) * (y - y1) / (y3 - y1);
		*/
		if (a > b) swap(a,b);
			drawLine(a, y, b, y, color);
	}
}


void setBackgroundColor(uint16_t color) {
	_bgColor = color;
}


void setFont(uint8_t* font) {

	cfont.font 	   = font;
	cfont.width    = readFontByte(0);
	cfont.height   = readFontByte(1);
	cfont.offset   = readFontByte(2);
	cfont.numchars = readFontByte(3);
	cfont.nbrows   = cfont.height / 8;

	if (cfont.height % 8) cfont.nbrows++;  // Set number of bytes used by height of font in multiples of 8
}


void drawText(uint16_t x, uint16_t y, char *s, uint16_t color) {

	uint16_t currx = x;
	uint8_t len = 0;
	if(strlen(s)){
		len = strlen(s);
		for (uint8_t k = 0; k < len; k++) {//for (uint8_t k = 0; k < strlen(s); k++) {
			currx += drawChar(currx, y, s[k], color) + 1;
		}
	}
	// Print every character in string
}


uint16_t drawChar(uint16_t x, uint16_t y, uint16_t ch, uint16_t color) {

	uint8_t charData, charWidth;
	uint8_t h, i, j;
	uint16_t charOffset;

	charOffset = (cfont.width * cfont.nbrows) + 1;  // bytes used by each character
	charOffset = (charOffset * (ch - cfont.offset)) + FONT_HEADER_SIZE;  // char offset (add 4 for font header)
	charWidth  = readFontByte(charOffset);  // get font width from 1st byte
	charOffset++;  // increment pointer to first character data byte

	for (i = 0; i <= charWidth; i++) {  // each font "column" (+1 blank column for spacing)
		h = 0;  // keep track of char height
		for (j = 0; j < cfont.nbrows; j++) 	{  // each column byte
			if (i == charWidth) charData = (uint8_t)0x0; // Insert blank column
			else                charData = readFontByte(charOffset);
			charOffset++;
			
			// Process every row in font character
			for (uint8_t k = 0; k < 8; k++) {
				if (h >= cfont.height ) break;  // No need to process excess bits
				if (bitRead(charData, k)) drawPixel(x + i, y + (j * 8) + k, color);
				else                      drawPixel(x + i, y + (j * 8) + k, _bgColor);
				h++;
			};
		};
	};
	return charWidth;
}

void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

	int16_t i, j, byteWidth = (w + 7) / 8;
	uint8_t byte;

	for(j = 0; j < h; j++) {
		for(i = 0; i < w; i++) {
			if (i & 7) byte <<= 1;
			else      byte   = bitmap[j * byteWidth + i / 8];
			if (byte & 0x80) drawPixel(x + i, y + j, color);
		}
	}
}
