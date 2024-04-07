#include "BMP280.h"

  BMP280_config bmpCnfg;
  bmpCnfg.i2c = hi2c1;
  bmpCnfg.id = BMP280_ADDR_W;
  bmpCnfg.IIRF = filter_off;
  bmpCnfg.pM = normalMode;
  bmpCnfg.pO = pressureOversampling_X16;
  bmpCnfg.tO = temperatureOversampling_X16;
  BMP280_setConfig(bmpCnfg);

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
