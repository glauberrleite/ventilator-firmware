#include <sensors.h>

float getFlowAWM720P(float v)
{
  v = v < 1.0 ? 1.0 : v;
  v = v > 5.0 ? 5.0 : v;

  float flow = 0;

  if (v >= 1 && v < 2.99) {
    // Polynomial coefficients in our interpolation method are local coefficients for each interval
    float rel = (v - 1);

    flow = 1.9385 * pow(rel, 3) - 3.0982 * pow(rel, 2) + 11.0514 * rel;
  }
  else if (v >= 2.99 && v < 3.82) {
    float rel = (v - 2.99);

    flow = 1.9385 * pow(rel, 3) + 8.4748 * pow(rel, 2) + 21.7509 * rel + 25;
  }
  else if (v >= 3.82 && v < 4.3) {
    float rel = (v - 3.82);

    flow = 25.4906 * pow(rel, 3) + 13.3017 * pow(rel, 2) + 39.8255 * rel + 50;
  }
  else if (v >= 4.3 && v < 4.58) {
    float rel = (v - 4.3);

    flow = 64.6584 * pow(rel, 3) + 50.0081 * pow(rel, 2) + 70.2142 * rel + 75;
  }
  else if (v >= 4.58 && v < 4.86) {
    float rel = (v - 4.58);

    flow = 458.3556 * pow(rel, 3) + 104.3212 * pow(rel, 2) + 113.4264 * rel + 100;
  }
  else {
    float rel = (v - 4.86);

    flow = 458.3556 * pow(rel, 3) + 489.3389 * pow(rel, 2) + 279.6515 * rel + 150;
  }

  return flow;
}

float getPressureASDX(float v, float p_min, float p_max)
{  
  v = v < 0.5 ? 0.5 : v;
  v = v > 4.5 ? 4.5 : v;

  return ((v - 0.5)*(p_max - p_min)/4) + p_min;
}

float getPressureASDX001PDAA5(float v)
{
  return getPressureASDX(v, -1, 1);
}

float getPressureASDX005NDAA5(float v)
{
  return getPressureASDX(v, -5, 5);
}