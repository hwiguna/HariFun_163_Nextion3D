#include <Nextion.h>
#include <INextionColourable.h>
#include <SoftwareSerial.h>

SoftwareSerial nextionSerial(10, 11); // RX, TX

Nextion nex(nextionSerial);

int xMax = 479;
int yMax = 319;

const uint16_t n = 21;
uint16_t x[n];
uint16_t y[n];
uint16_t x0 = xMax/2;
uint16_t y0 = yMax/2;
uint16_t r = yMax/2;

void PreComputeNodes()
{
  for (uint16_t i=0; i<n; i++)
  {
    float a = 2*PI * i / n;
    x[i] = x0 + cos(a)*r;
    y[i] = y0 + sin(a)*r;
  }
}

void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t colour)
{
  Serial.println(nex.drawLine(x1, y1, x2, y2, colour));
}

void DrawChord()
{
  for (int i=0;i<(n-1); i++)
  {
    for (int j= i+1; j<n; j++)
    {
      DrawLine(x[i], y[i], x[j],y[j], NEX_COL_YELLOW);
    }
  }
}

void setup()
{
  Serial.begin(9600);

  nextionSerial.begin(9600);
  nex.init();

    Serial.println(nex.clear(NEX_COL_BLACK));
  
  PreComputeNodes();
  DrawChord();
}

void loop()
{
  nex.poll();
}
