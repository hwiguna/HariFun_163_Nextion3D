#include <Nextion.h>
#include <INextionColourable.h>
#include <SoftwareSerial.h>
#include "Point.h"
#include "Camera.h"

SoftwareSerial nextionSerial(10, 11); // RX, TX

Nextion nex(nextionSerial);
int dist = 3;
//NexGpio gpio; This does not exists in NeoNextion, so we'll read gpio manually.

uint32_t bgColor = NEX_COL_BLACK;
uint32_t foreColor = NEX_COL_YELLOW;
float xRot, xRotOld;
float yRot, yRotOld;

Camera cam = Camera(Point(0, 0, -5), Point(0, 0, 0));

// Does not seem to work, or maybe software serial can only go up to 9600
void SetBaud19200()
{
  String cmd = "set bauds=19200"; // 38400 is too fast for software serial
  nex.sendCommand(cmd.c_str());
}

void pin_mode(uint32_t port, uint32_t mode, uint32_t control_id)
{
  char buf;
  String cmd;

  cmd += "cfgpio ";
  buf = port + '0';
  cmd += buf;
  cmd += ',';
  buf = mode + '0';
  cmd += buf;
  cmd += ',';
  buf = control_id = '0';
  cmd += buf;

  nex.sendCommand(cmd.c_str());
}

uint16_t getGpio(int port)
{
  String cmd = "get pio";
  char buf = port + '0';
  cmd += buf;

  nex.sendCommand(cmd.c_str());
  uint32_t val;
  if (nex.receiveNumber(&val))
    return val;
  else
    return 0;
}

int xMax = 479;
int yMax = 319;
int x0 = xMax / 2;
int y0 = yMax / 2;

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

void MoveCubeAwayFromOrigin(int howFarInZ)
{
  for (int i = 0; i < nVertices; i++)
  {
    vertexes[i].z += howFarInZ;
  }
}

void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t colour)
{
  //Serial.println(nex.drawLine(x1, y1, x2, y2, colour));
  nex.drawLine(x1, y1, x2, y2, colour);
}

void DrawCircle(uint16_t x1, uint16_t y1, uint16_t r, uint32_t colour)
{
  //Serial.println(nex.drawCircle(x1, y1, r, colour));
  nex.drawCircle(x1, y1, r, colour);
}

void DrawLine(Point p1, Point p2, uint32_t colour)
{
  //Serial.println(nex.drawLine(p1.x, p1.y, p2.x, p2.y, colour));
  nex.drawLine(p1.x, p1.y, p2.x, p2.y, colour);
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

      int f = y0 / vertex.z; // 
      int x = x0 + vertex.x * f;
      int y = y0 + vertex.y * f;
      points[v] = Point(x, y, 0);
    }

    DrawLine(points[0], points[1], kolor); // Draw the transformed edge
  }
}

void setup()
{
  Serial.begin(9600);

  nextionSerial.begin(9600);
  //SetBaud19200();

  nex.init();

  pin_mode(0, 0, 0);
  pin_mode(1, 0, 0);
  pin_mode(2, 0, 0);
  pin_mode(3, 0, 0);
  pin_mode(4, 0, 0);
  pin_mode(5, 0, 0);

  //Serial.println(nex.clear(bgColor));
  nex.clear(bgColor);

  MoveCubeAwayFromOrigin(dist);
  DrawCube(foreColor);

  
}

void Refresh()
{
  nex.clear(bgColor);
  DrawCube(foreColor);
}

void loop()
{
  int stp = 1;
  if (getGpio(5) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 'a');
    Refresh();
  }
  if (getGpio(2) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 'd');
    Refresh();
  }
  if (getGpio(4) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 'w');
    Refresh();
  }
  if (getGpio(3) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 's');
    Refresh();
  }
  if (getGpio(0) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 'q');
    Refresh();
  }
  if (getGpio(1) == 0) {
    //DrawCube(NEX_COL_BLACK);
    cam.Update(stp, 'e');
    Refresh();
  }

  //xRot =  map(analogRead(A0), 0, 1023, 0, PI*100 / 2)/100;
  xRot =  -PI/2 + (PI * analogRead(A0))/1023;
  yRot =  -PI/2 + (PI * analogRead(A1))/1023;
  //if (abs(xRot - xRotOld) > 0.1) {
    Refresh();
    xRotOld = xRot;
  //}

  delay(100);
  //nex.poll();
}
