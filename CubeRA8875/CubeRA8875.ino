//== Libraries ==
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"

//== Classes ==
#include "Point3D.h"
#include "Camera.h"

//== RA8875 Variables ==
// Library only supports hardware SPI at this time
// Connect SCLK to UNO Digital #13 (Hardware SPI clock)
// Connect MISO to UNO Digital #12 (Hardware SPI MISO)
// Connect MOSI to UNO Digital #11 (Hardware SPI MOSI)
#define RA8875_INT 3
#define RA8875_CS 10
#define RA8875_RESET 9

Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

//== Drawing Variables ==
uint32_t bgColor = RA8875_BLACK;
uint32_t foreColor = RA8875_YELLOW;
float xMax = 799; //479;
float yMax = 479; //319;
float xOrigin = xMax / 2;
float yOrigin = yMax / 2;
float xRot, xRotOld;
float yRot, yRotOld;

//== 3D Variables ==
Camera cam = Camera(Point3D(0, 0, -5), Point3D(0, 0, 0));

Point3D vertexes[] = {
  Point3D(-1, -1, -1), Point3D(+1, -1, -1), Point3D(+1, +1, -1), Point3D(-1, +1, -1),
  Point3D(-1, -1, +1), Point3D(+1, -1, +1), Point3D(+1, +1, +1), Point3D(-1, +1, +1),
};
int nVertices = sizeof(vertexes) / sizeof(Point3D);

uint16_t edges[][2] = {
  {0, 1}, {1, 2}, {2, 3}, {3, 0},
  {4, 5}, {5, 6}, {6, 7}, {7, 4},
  {0, 4}, {1, 5}, {2, 6}, {3, 7}
};
int nEdges = sizeof(edges) / (2 * sizeof(uint16_t));

//== Setup Routines ==
void SetupRA8875()
{
    /* Initialise the display using 'RA8875_480x272' or 'RA8875_800x480' */
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
    while (1);
  }

  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255);
  tft.fillScreen(RA8875_BLACK);

//  tft.drawPixel(0,0, RA8875_RED);
//  tft.drawPixel(799,0, RA8875_GREEN);
//  tft.drawPixel(799,479, RA8875_BLUE);
//  tft.drawPixel(0,479, RA8875_WHITE);
}

void SetupButtons()
{
  for (byte i=2; i<9; i++)
    pinMode(i,INPUT_PULLUP);
}

void setup()
{
  Serial.begin(9600);
  SetupButtons();
  SetupRA8875();
  DrawCube(foreColor); 
}

//== Drawing Routines ==
void Clear(uint32_t colour)
{
  //nex.clear(colour);
  tft.fillScreen(colour);
}

void DrawCircle(uint16_t x1, uint16_t y1, uint16_t r, uint32_t colour)
{
  //nex.drawCircle(x1, y1, r, colour);
  tft.drawCircle(x1, y1, r, colour);
}

//uint16_t ClipX(float y)
//{
//  return (max(min(y,xMax),0));
//}
//
//uint16_t ClipY(float y)
//{
//  return (max(min(y,yMax),0));
//}

// Originally from: https://stackoverflow.com/questions/11194876/clip-line-to-screen-coordinates
// Liang-Barsky function by Daniel White @ http://www.skytopia.com/project/articles/compsci/clipping.html
// This function inputs 8 numbers, and outputs 4 new numbers (plus a boolean value to say whether the clipped line is drawn at all).
//
bool LiangBarsky (float edgeLeft, float edgeRight, float edgeBottom, float edgeTop,   // Define the x/y clipping values for the border.
                  float x0src, float y0src, float x1src, float y1src,                 // Define the start and end points of the line.
                  float &x0clip, float &y0clip, float &x1clip, float &y1clip)         // The output values, so declare these outside.
{

    float t0 = 0.0;    float t1 = 1.0;
    float xdelta = x1src-x0src;
    float ydelta = y1src-y0src;
    float p,q,r;

    for(int edge=0; edge<4; edge++) {   // Traverse through left, right, bottom, top edges.
        if (edge==0) {  p = -xdelta;    q = -(edgeLeft-x0src);  }
        if (edge==1) {  p = xdelta;     q =  (edgeRight-x0src); }
        if (edge==2) {  p = -ydelta;    q = -(edgeBottom-y0src);}
        if (edge==3) {  p = ydelta;     q =  (edgeTop-y0src);   }   
        r = q/p;
        if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)

        if(p<0) {
            if(r>t1) return false;         // Don't draw line at all.
            else if(r>t0) t0=r;            // Line is clipped!
        } else if(p>0) {
            if(r<t0) return false;      // Don't draw line at all.
            else if(r<t1) t1=r;         // Line is clipped!
        }
    }

    x0clip = x0src + t0*xdelta;
    y0clip = y0src + t0*ydelta;
    x1clip = x0src + t1*xdelta;
    y1clip = y0src + t1*ydelta;

    return true;        // (clipped) line is drawn
}

void DrawLine(Point3D p1, Point3D p2, uint32_t colour)
{
  float x1c=0;
  float y1c=0;
  float x2c=0;
  float y2c=0;

  bool isVisible = LiangBarsky (0, xMax, 0, yMax,   // Define the x/y clipping values for the border.
                   p1.x, p1.y,  p2.x, p2.y,                 // Define the start and end points of the line.
                   x1c,  y1c,  x2c, y2c);         // The output values, so declare these outside.

  if (isVisible) tft.drawLine(x1c,y1c, x2c,y2c, colour);
}

Point3D rotate2D(Point3D p, float rot)
{
  float s = sin(rot);
  float c = cos(rot);
  return Point3D(c * p.x  - s * p.y , c * p.y + s * p.x , 0);
}

void DrawCube(uint32_t kolor)
{
  //  //-- Draw vertices --
  //  for (int i = 0; i < nVertices; i++)
  //  {
  //    int f = yOrigin / vertexes[i].z;
  //    int x = xOrigin + vertexes[i].x * f;
  //    int y = yOrigin + vertexes[i].y * f;
  //
  //    DrawCircle(x, y, 5, foreColor);
  //  }

  //-- Draw Edges --
  for (int e = 0; e < nEdges; e++)
  {
    // an edge is just two points.  We'll transform those two points in the loop below and save the transformed versions in this two element array.
    Point3D points[2] {Point3D(0, 0, 0), Point3D(0, 0, 0)};

    for (int v = 0; v < 2; v++) { // for the two points of an edge...
      Point3D vertex = vertexes[ edges[e][v] ]; // Translate edge index into actual vertex 3D coordinate.

      // Adjust vertex based on camera position
      vertex.x -= cam.pos.x;
      vertex.y -= cam.pos.y;
      vertex.z -= cam.pos.z;

      // Rotate around Y Axis first
      Point3D t = rotate2D( Point3D(vertex.x, vertex.z, 0), yRot); // Rotate2D only does not use the Z component of the first parameter
      vertex.x = t.x;
      vertex.z = t.y;

      // Then rotate around X Axis second
      t = rotate2D( Point3D(vertex.y, vertex.z, 0), xRot); // Rotate2D only does not use the Z component of the first parameter
      vertex.y = t.x;
      vertex.z = t.y;

      float f = (vertex.z==0) ? yOrigin : (yOrigin / vertex.z); // Further (larger z) will result in smaller f, which makes the point closer to origin making it look smaller
      int x = xOrigin + vertex.x * f;
      int y = yOrigin + vertex.y * f;
      points[v] = Point3D(x, y, 0);
    }

    DrawLine(points[0],points[1],foreColor);
  }
}

void Refresh()
{
  Clear(bgColor);
  DrawCube(foreColor);
}

void loop()
{
  int stp = 2;
  if (digitalRead(4) == 0) {
    cam.Update(stp, 'a');
    Refresh();
  }
  if (digitalRead(5) == 0) {
    cam.Update(stp, 'd');
    Refresh();
  }
  if (digitalRead(6) == 0) {
    cam.Update(stp, 'w');
    Refresh();
  }
  if (digitalRead(7) == 0) {
    cam.Update(stp, 's');
    Refresh();
  }
  if (digitalRead(8) == 0) {
    cam.Update(stp, 'q');
    Refresh();
  }
  if (digitalRead(2) == 0) {
    cam.Update(stp, 'e');
    Refresh();
  }

  xRot =  -PI/2 + (PI * analogRead(A0))/1023;
  yRot =  -PI/4 + (PI/2 * analogRead(A1))/1023;
  
  if ( fabs(xRot - xRotOld) > 0.1 || fabs(yRot - yRotOld)> 0.1) {
    Refresh();
    xRotOld = xRot;
    yRotOld = yRot;
  }

  delay(100);
  //nex.poll();
}
