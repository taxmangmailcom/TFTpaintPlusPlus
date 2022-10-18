// (c) ladyada / Elegoo / taxman@gmail.com
// Code under MIT License
// Adaptation of Elegoo TFTLCD code adapted from Adafruit TFTLCD code
// specifically for "ELEGOO UNO R3 2.8 Inches TFT Touch Screen with SD Card Socket w/All Technical Data in CD for Arduino UNO R3"
// operating as a shield on top of ELEGOO UNO R3 Board (Arduino compatible)
// to extend example touch/paint app for my daughter
// and to see if drag and drop was possible
// requires install of Elegoo libraries https://www.elegoo.com/pages/arduino-kits-support-files


#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

//edited X,Y to be more accurate for my display
#define TS_MINX 80
#define TS_MAXX 940

#define TS_MINY 60
#define TS_MAXY 920

#define NUMSAMPLES 2

//derive TouchScreen to fix getPoint to provide better invalid and no-pressure states
class TouchScreenFixed : public TouchScreen {
  private:
    uint8_t _yp, _ym, _xm, _xp;
    uint16_t _rxplate;
  
  public:
    TouchScreenFixed::TouchScreenFixed(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t rxplate) : TouchScreen(xp, yp, xm, ym, rxplate){
      _yp = yp;
      _xm = xm;
      _ym = ym;
      _xp = xp;
      _rxplate = rxplate;
    }

    TSPoint TouchScreenFixed::getPoint(void) {
      int x, y, z;
      int samples[NUMSAMPLES];
      uint8_t i, valid;
      

      uint8_t xp_port = digitalPinToPort(_xp);
      uint8_t yp_port = digitalPinToPort(_yp);
      uint8_t xm_port = digitalPinToPort(_xm);
      uint8_t ym_port = digitalPinToPort(_ym);

      uint8_t xp_pin = digitalPinToBitMask(_xp);
      uint8_t yp_pin = digitalPinToBitMask(_yp);
      uint8_t xm_pin = digitalPinToBitMask(_xm);
      uint8_t ym_pin = digitalPinToBitMask(_ym);

      valid = 1;

      pinMode(_yp, INPUT);
      pinMode(_ym, INPUT);
      
      *portOutputRegister(yp_port) &= ~yp_pin;
      *portOutputRegister(ym_port) &= ~ym_pin;
      
      pinMode(_xp, OUTPUT);
      pinMode(_xm, OUTPUT);

      *portOutputRegister(xp_port) |= xp_pin;
      *portOutputRegister(xm_port) &= ~xm_pin;
      
      for (i=0; i<NUMSAMPLES; i++) {
        samples[i] = analogRead(_yp);
      }
    #if NUMSAMPLES > 2
      insert_sort(samples, NUMSAMPLES);
    #endif
    #if NUMSAMPLES == 2
      if (samples[0] != samples[1]) { valid = 0; }
    #endif
      x = (1023-samples[NUMSAMPLES/2]);

      pinMode(_xp, INPUT);
      pinMode(_xm, INPUT);
      *portOutputRegister(xp_port) &= ~xp_pin;
      
      pinMode(_yp, OUTPUT);
      *portOutputRegister(yp_port) |= yp_pin;
      pinMode(_ym, OUTPUT);
      
      for (i=0; i<NUMSAMPLES; i++) {
        samples[i] = analogRead(_xm);
      }

    #if NUMSAMPLES > 2
      insert_sort(samples, NUMSAMPLES);
    #endif
    #if NUMSAMPLES == 2
      if (samples[0] != samples[1]) { valid = 0; }
    #endif

      y = (1023-samples[NUMSAMPLES/2]);

      pinMode(_xp, OUTPUT);
      *portOutputRegister(xp_port) &= ~xp_pin;
      *portOutputRegister(ym_port) |= ym_pin;
      *portOutputRegister(yp_port) &= ~yp_pin;
      pinMode(_yp, INPUT);
      
      int z1 = analogRead(_xm); 
      int z2 = analogRead(_yp);

      if (_rxplate != 0) {
        float rtouch;
        rtouch = z2;
        rtouch /= z1;
        rtouch -= 1;
        rtouch *= x;
        rtouch *= _rxplate;
        rtouch /= 1024;
        
        z = rtouch;
      } else {
        z = (1023-(z2-z1));
      }

//if invalid then return all -1 x,y,z
      if (! valid) {
        return TSPoint(-1,-1,-1);
      }

//if 0 pressure then get rid of x,y false values
      if (z == 0){
        return TSPoint(0, -1, -1);
      }       

//maybe actually real pressure
      return TSPoint(x, y, z);
    }

};

TouchScreenFixed ts = TouchScreenFixed(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

//RGB565 Color Picker
#define	BLACK   0x0000
#define	BLUE    0x001F
#define LBLUE   0xCE1F
#define DBLUE   0x080C
#define	RED     0xF800
#define	GREEN   0x07E0
#define DGREEN  0x33A8
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define PINK    0xFD78
#define ORANGE  0xFD00

#define BGCOLOR 0xDEFB

#define DOT         -1
#define LINE        -2
#define CIRCLE      -3
#define RECTANGLE   -4
#define TRIANGLE    -5

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define BOXSIZE 40
#define PENRADIUS 3
#define ERASERRADIUS 6

long menus[3][6] =  { 
                    {1, YELLOW, GREEN, CYAN, RED, MAGENTA} , 
                    {2, PINK, BGCOLOR, ORANGE, WHITE, BLACK} , 
                    {0, DOT, LINE, CIRCLE, RECTANGLE, TRIANGLE}
                    };

long oldcolor = BGCOLOR;
long currentcolor = menus[0][1];

int currentMenu = 0;

int selectedMenu = 0;
int selectedItem = 1;
int selectedPen = 1;

bool penDown = false;
bool penPrev = false;
int penPrevX = 0;
int penPrevY = 0;

int dragStartX = 0;
int dragStartY = 0;
int dragEndX = 0;
int dragEndY = 0;

long pen = DOT;

void setup(void) {
  Serial.begin(9600);
  Serial.println(F("Paint!"));

  setupLCD();

  tft.fillScreen(BGCOLOR);

  drawMenu();

  pinMode(13, OUTPUT);
}

//seems backwards for my LCD where 500 is min pressure, but 0 is still no pressure
#define MINPRESSURE 10
#define MAXPRESSURE 475

void loop()
{

  digitalWrite(13, HIGH);
  int tsp = ts.pressure();
  TSPoint p = ts.getPoint();

  //eliminate garbage
  //fake pressure
  if((tsp<0 || tsp>500) 
    //invalid x
    || (tsp>0 && p.x==-1) 
    //invalid y
    || (tsp>0 && p.y==-1) 
    //fake pressure
    || (tsp==0 && p.x!=-1) 
    //fake pressure
    || (tsp==0 && p.y!=-1)) {
    digitalWrite(13, LOW);
    return;
  }
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  //Serial.println(String(tsp) + "/" + String(p.z) + " " + String(p.x) + "," + String(p.y));

  if (tsp > -1 && tsp < MAXPRESSURE) {
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = (tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
  } else {
    //one last chance to ignore bad inputs
    return;
  }

 // Serial.println(String(tsp) + "/" + String(p.z) + " " + String(p.x) + "," + String(p.y));


  //legit no pressure.  leaving as 0-9 as maybe necessary for other touchscreens
  if(tsp>-1 && tsp<10) {
    //Serial.println("up");
    penDown = false;
  //touching menu box is same as pen up for ending drags
  } else if (tsp>10 && p.y < BOXSIZE){
    //Serial.println("up");
    penDown = false;
  } 
  //legit pressure
  else if (tsp>10){
    //Serial.println("down");
   // Serial.println(String(tsp) + " " + String(p.x) + "," + String(p.y));
    penDown = true;
  //why?
  } else {
    //Serial.println("why are we here?");
    //future non-menu items?
  }

//start of a drag
  if(penDown == true && penPrev == false){
    dragStartX = p.x;
    dragStartY = p.y;
    //Serial.println("Pen Down " + String(dragStartX) + " " + String(dragStartY));
  } else if (penDown == false && penPrev == true){
//end of a drag
    dragEndX = penPrevX;
    dragEndY = penPrevY;
    //Serial.println("Pen Up " + String(dragEndX) + " " + String(dragEndY));
  }  //else no change event else pen is already up or already down

  if(tsp>10 && p.y<BOXSIZE){
    //change menu item
    oldcolor = currentcolor;
    if (p.x < BOXSIZE) { 
      currentMenu = menus[currentMenu][0];
      //Serial.println("Menu Change");
      drawMenu();
    } else {
      //select the right color or brush
      int loc = p.x / BOXSIZE;
      long item = menus[currentMenu][loc];
      if(item<0){ 
        //brush type menu items are all negative
        pen = item;
        selectedPen = loc;      
      } else {
        //colors are all positive
        currentcolor = menus[currentMenu][loc];
        selectedMenu = currentMenu;
        selectedItem = loc;
      }
      drawMenu();
    }
  }


  if(pen != DOT){
    if(penPrev == 1 && penDown == 0) {
      //non brush type
      // Serial.println(String(dragStartX) + "," + String(dragStartY) + " " + String(dragEndX) + "," + String(dragEndY) + " " + String(int(currentcolor)));
      if(pen == LINE){
        //Serial.println("line time");
        //from start drag to end drag location
        tft.drawLine(dragStartX, dragStartY, dragEndX, dragEndY, int(currentcolor));
        drawMenu();
      } else if (pen == CIRCLE){
        //Serial.println("circle time");
        //start drag is center of circle, and end drag is distance of radius        
        unsigned long a1 = abs(dragEndX-dragStartX);
        unsigned long a2 = (a1*a1);
        unsigned long b1 = abs(dragEndY-dragStartY);
        unsigned long b2 = (b1*b1);
        long r = sqrt(a2+b2);
        //Serial.println(String(a1) + " " + String(a2)+ " " + String(b1) + " " + String(b2) + " " + String(r));
        tft.drawCircle(dragStartX, dragStartY, r, int(currentcolor));
        drawMenu();
      } else if (pen==RECTANGLE){
        //Serial.println("rect time");
        //start drag is first corner, and end drag is furthest corner
        long L=0;
        long W=0;
        long T=0;
        //moving start/end to work with the built in functions
        if(dragEndX<dragStartX){
          T = dragStartX;
          dragStartX = dragEndX;
          dragEndX=T;
        } 
        if(dragEndY<dragStartY){
          T = dragStartY;
          dragStartY = dragEndY;
          dragEndY=T;
        } 
        L = abs(dragEndX-dragStartX);            
        W = abs(dragEndY-dragStartY);
        // Serial.println(String(dragStartX) + " " + String(dragStartY)+ " " + String(L) + " " + String(W));
        tft.drawRect(dragStartX, dragStartY, L, W, int(currentcolor));
        drawMenu();
      } else if (pen==TRIANGLE){
        //Serial.println("tri time");     
        //start is a corner and end drag is the pinacle.
        //does not cover all scenarios, but would be difficult without refreshing the screen       
        long W = abs(dragEndY-dragStartY);
        //Serial.println(String(dragStartX) + " " + String(dragStartY)+ " " + String(dragEndX) + " " + String(dragEndY) + " " + String(W));
        // appropriately flip the triangle start/end for the built in functions
        if((dragStartX<dragEndX && dragStartY<dragEndY) || (dragStartX>dragEndX && dragStartY<dragEndY)){
        } else {
          W *= -1;
        }
        tft.drawTriangle(dragStartX, dragStartY, dragEndX, dragEndY, dragStartX, dragEndY+W, int(currentcolor));
        drawMenu();
      }
    } 
  } else {
    //regular brush
    if(int(currentcolor)!=BGCOLOR){
      //regular brush
      if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
        tft.fillCircle(p.x, p.y, PENRADIUS, int(currentcolor));
      }
    } else {
        //special brush of the same color as the background color
          if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
            tft.fillCircle(p.x, p.y, ERASERRADIUS, int(currentcolor));         
          }
    }         
  }

  //remember last pen state and location for dragging
  penPrev = penDown;
  penPrevX = p.x;
  penPrevY = p.y;

}

void drawMenu(){

  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, BGCOLOR);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, menus[currentMenu][1]);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, menus[currentMenu][2]);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, menus[currentMenu][3]);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, menus[currentMenu][4]);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, menus[currentMenu][5]);

//highlight selected item if on right menuscreen
  if(currentMenu == selectedMenu){
    //"highlight" selected color
    tft.drawRect(BOXSIZE*selectedItem, 0, BOXSIZE, BOXSIZE, BLACK);
  }

  if(currentMenu == 2){
    //"highlight brush icon"
    tft.drawRect(BOXSIZE*selectedPen, 0, BOXSIZE, BOXSIZE, BLACK);
  }

  if(currentMenu == 2){
    //drag brush icons
    //will this break on diff dimmension screens?  maybe 
    tft.fillCircle(60, 20, 5, BLACK);
    tft.drawLine(90,10,110,30, BLACK);
    tft.drawCircle(135, 20, 10, BLACK);
    tft.drawRect(170, 10, 20, 20, BLACK);
    tft.drawTriangle(210, 10, 210, 30, 230, 20, BLACK);
  }

//label bottom box as menu
  tft.setCursor(5,5);
  tft.setTextColor(BLACK);  tft.setTextSize(1);
  tft.println("Menu");
  delay(100);
}

void setupLCD() {
  tft.reset();
  
  uint16_t identifier = tft.readID();
  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101) {     
      identifier=0x9341;
      Serial.println(F("Found 0x9341 LCD driver"));
  }else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9341;
  }

  tft.begin(identifier);
  tft.setRotation(2);
}