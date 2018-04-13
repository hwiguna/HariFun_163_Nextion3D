//== Libraries ==
#include <Nextion.h>
#include <INextionColourable.h>
#include <SoftwareSerial.h>

//== Classes ==
#include "Point.h"
#include "Camera.h"
#include "NexGpio.h"

//== Nextion Variables ==
SoftwareSerial nextionSerial(10, 11); // RX, TX
Nextion nex(nextionSerial);
NexGpio gpio;

//== Drawing Variables ==
uint32_t bgColor = NEX_COL_BLACK;
uint32_t foreColor = NEX_COL_YELLOW;
int xMax = 479;
int yMax = 319;
int x0 = xMax / 2;
int y0 = yMax / 2;
float xRot, xRotOld;
float yRot, yRotOld;

//== 3D Variables ==
Camera cam = Camera(Point(0, 0, -5), Point(0, 0, 0));

Point vertexes[] = {
  Point(-1, -1, -1), Point(+1, -1, -1), Point(+1, +1, -1), Point(-1, +1, -1),
  Point(-1, -1, +1), Point(+1, -1, +1), Point(+1, +1, +1), Point(-1, +1, +1),
};
int nVertices = sizeof(vertexes) / sizeof(Point);

uint16_t edges[][2] = {
  {0, 1}, {1, 2}, {2, 3}, {3, 0},
  {4, 5}, {5, 6}, {6, 7}, {7, 4},
  {0, 4}, {1, 5}, {2, 6}, {3, 7}
};
int nEdges = sizeof(edges) / (2 * sizeof(uint16_t));

//== Setup Routines ==
// Does not seem to work, or maybe software serial can only go up to 9600
void SetBaud19200()
{
  String cmd = "set bauds=19200"; // 38400 is too fast for software serial
  nex.sendCommand(cmd.c_str());
}

void MoveCubeAwayFromOrigin(int howFarInZ)
{
  for (int i = 0; i < nVertices; i++)
  {
    vertexes[i].z += howFarInZ;
  }
}

void setup()
{
  Serial.begin(9600);

  nextionSerial.begin(9600);
  //SetBaud19200();

  nex.init();

  gpio.pin_mode(nex, 0, 0, 0);
  gpio.pin_mode(nex, 1, 0, 0);
  gpio.pin_mode(nex, 2, 0, 0);
  gpio.pin_mode(nex, 3, 0, 0);
  gpio.pin_mode(nex, 4, 0, 0);
  gpio.pin_mode(nex, 5, 0, 0);

  //Serial.println(nex.clear(bgColor));
  nex.clear(bgColor);

  //MoveCubeAwayFromOrigin(dist);
  DrawCube(foreColor); 
}

//== Drawing Routines ==
void DrawCircle(uint16_t x1, uint16_t y1, uint16_t r, uint32_t colour)
{
  //Serial.println(nex.drawCircle(x1, y1, r, colour));
  nex.drawCircle(x1, y1, r, colour);
}

uint16_t ClipX(float y)
{
  return (max(min(y,xMax),0));
}

uint16_t ClipY(float y)
{
  return (max(min(y,yMax),0));
}

void DrawLine(Point p1, Point p2, uint32_t colour)
{
  nex.drawLine(ClipX(p1.x), ClipY(p1.y), ClipX(p2.x), ClipY(p2.y), colour);
}

Point rotate2D(Point p, float rot)
{
  float s = sin(rot);
  float c = cos(rot);
  return Point(c * p.x  - s * p.y , c * p.y + s * p.x , 0);
}

void DrawCube(uint32_t kolor)
{
  //  //-- Draw vertices --
  //  for (int i = 0; i < nVertices; i++)
  //  {
  //    int f = y0 / vertexes[i].z;
  //    int x = x0 + vertexes[i].x * f;
  //    int y = y0 + vertexes[i].y * f;
  //
  //    DrawCircle(x, y, 5, foreColor);
  //  }

  //-- Draw Edges --
  for (int e = 0; e < nEdges; e++)
  {
    // an edge is just two points.  We'll transform those two points in the loop below and save the transformed versions in this two element array.
    Point points[2] {Point(0, 0, 0), Point(0, 0, 0)};

    for (int v = 0; v < 2; v++) { // for the two points of an edge...
      Point vertex = vertexes[ edges[e][v] ]; // Translate edge index into actual vertex 3D coordinate.

      // Adjust vertex based on camera position
      vertex.x -= cam.pos.x;
      vertex.y -= cam.pos.y;
      vertex.z -= cam.pos.z;

      // Rotate around Y Axis first
      Point t = rotate2D( Point(vertex.x, vertex.z, 0), yRot); // Rotate2D only does not use the Z component of the first parameter
      vertex.x = t.x;
      vertex.z = t.y;

      // Then rotate around X Axis second
      t = rotate2D( Point(vertex.y, vertex.z, 0), xRot); // Rotate2D only does not use the Z component of the first parameter
      vertex.y = t.x;
      vertex.z = t.y;

      float f = (vertex.z==0) ? y0 : (y0 / vertex.z); // Further (larger z) will result in smaller f, which makes the point closer to origin making it look smaller
      int x = x0 + vertex.x * f;
      int y = y0 + vertex.y * f;
      points[v] = Point(x, y, 0);
    }

    DrawLine(points[0],points[1],foreColor);
  }
}

void Refresh()
{
  nex.clear(bgColor);
  DrawCube(foreColor);
}

void loop()
{
  int stp = 2;
  if (gpio.getGpio(nex,5) == 0) {
    cam.Update(stp, 'a');
    Refresh();
  }
  if (gpio.getGpio(nex,2) == 0) {
    cam.Update(stp, 'd');
    Refresh();
  }
  if (gpio.getGpio(nex,4) == 0) {
    cam.Update(stp, 'w');
    Refresh();
  }
  if (gpio.getGpio(nex,3) == 0) {
    cam.Update(stp, 's');
    Refresh();
  }
  if (gpio.getGpio(nex,0) == 0) {
    cam.Update(stp, 'q');
    Refresh();
  }
  if (gpio.getGpio(nex,1) == 0) {
    cam.Update(stp, 'e');
    Refresh();
  }

  xRot =  map(analogRead(A0), 0, 1023, -PI/2, PI/ 2);
  //xRot =  -PI/2 + (PI * analogRead(A0))/1023;
  yRot =  -PI/2 + (PI * analogRead(A1))/1023;
  
  if ( fabs(xRot - xRotOld) > 0.1 || fabs(yRot - yRotOld)> 0.1) {
    Refresh();
    xRotOld = xRot;
    yRotOld = yRot;
  }

  delay(100);
  //nex.poll();
}
