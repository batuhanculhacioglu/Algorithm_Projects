#include "BMP280.h"

  BMP280_config bmpC;
  bmpC.i2c = hi2c1;
  bmpC.id = 0x76<<1;
  bmpC.pM = normalMode;
  bmpC.pO = pressureOversampling_X16;
  bmpC.tO = temperatureOversampling_X16;
  bmpC.IIRF = filter_16;
  BMP280_setConfig(bmpC);
  BMP280_datas bmp;
  double temp = 0, press = 0;
  
  int main()
  {
  
   	while (1)
  	{
    	  BMP280_readAllDatas(&bmp);
	  temp = bmp.temperature;
	  press = bmp.pressure;
  	}
  }
