/*
  xsns_92_max6675.ino - MAX6675 thermocouple sensor support for Tasmota

  Adapted from xsns_39_max31855.ino, written by Markus Past

  Copyright (C) 2020  Markus Past
  Copyright (C) 2020  Mirko Augsburger

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_MAX6675

#define XSNS_92				92

#define MAX6675_MAX_SENSORS	5		// Maximum of 5 per type. Don't increase, as pins are not selected correctly if you do

bool MAX6675_initialized = false;

struct MAX6675_ResultStruct{
    float ProbeTemperature;             // Measured temperature of the 'hot' TC junction (probe temp)
    uint32_t ErrorCode;                  // Error Codes: 0 = No Error / 1 = TC open circuit / 2 = TC short to GND / 4 = TC short to VCC
};

struct MAX6675_sensorStruct {
	MAX6675_ResultStruct lastResult;	// last measurements for this sensor
	uint32_t CSPin;
};

uint32_t MAX6675_sensors_num = 0;

MAX6675_sensorStruct MAX6675_sensors[MAX6675_MAX_SENSORS];

void MAX6675_init(void){

    // Set GPIO modes for SW-SPI
    pinMode(Pin(GPIO_MAX6675_CLK), OUTPUT);
    pinMode(Pin(GPIO_MAX6675_DO), INPUT);

    // Chip not selected / Clock low
    digitalWrite(Pin(GPIO_MAX6675_CLK), LOW);

	#warning debug code!
	pinMode(Pin(GPIO_MAX6675_CS_0, 0), OUTPUT);
	digitalWrite(Pin(GPIO_MAX6675_CS_0, 0), HIGH);
	MAX6675_sensors[0].CSPin = Pin(GPIO_MAX6675_CS_0, 0);
	MAX6675_sensors[0].lastResult.ErrorCode = 2;
	MAX6675_sensors[0].lastResult.ProbeTemperature = 99.f;
	
	// Run through all Pins
	/*
	for (uint32_t i = 0; i < MAX6675_MAX_SENSORS; i++) {
		if (! PinUsed(GPIO_MAX6675_CS_0, i)) {
			continue;
		}

		pinMode(Pin(GPIO_MAX6675_CS_0, i), OUTPUT);
		digitalWrite(Pin(GPIO_MAX6675_CS_0, i), HIGH);

		MAX6675_sensors[MAX6675_sensors_num].CSPin = Pin(GPIO_MAX6675_CS_0, i);
		MAX6675_sensors[MAX6675_sensors_num].lastResult.ErrorCode = 2;
		MAX6675_sensors[MAX6675_sensors_num].lastResult.ProbeTemperature = NAN;
		MAX6675_sensors_num++;
	}
	*/



	MAX6675_initialized = true;
}

/*
 *   MAX6675_GetResult(void)
 *   Acquires the raw data via SPI, checks for MAX31855 errors and fills result structure
 */
void MAX6675_getResult(struct MAX6675_sensorStruct* sensor){
    int32_t rawData = MAX6675_shiftIn(sensor->CSPin, 16);
    uint8_t probeerror = rawData & 0x4;

    sensor->lastResult.ErrorCode = probeerror;
    if(probeerror)
        sensor->lastResult.ProbeTemperature = NAN;     // Return NaN if MAX31855 reports an error
    else
        sensor->lastResult.ProbeTemperature = MAX6675_getProbeTemperature(rawData);
}


/*
*   MAX6675_GetProbeTemperature(int32_t RawData)
*   Decodes and returns the temperature of TCs 'hot' junction from RawData
*/
float MAX6675_getProbeTemperature(int32_t rawData) {
    rawData >>= 3;									// Drop lower 3 bits
    float result = (rawData * 0.25f);				// MAX31855 LSB resolution is 0.25°C for probe temperature
    return ConvertTemp(result);						// Check if we have to convert to Fahrenheit
}


/*
*   MAX31855_ShiftIn(uint8_t Length)
*   Communicates with MAX31855 via SW-SPI and returns the raw data read from the chip
*/
int32_t MAX6675_shiftIn(uint32_t csPin, uint8_t Length) {
    int32_t dataIn = 0;

    digitalWrite(csPin, LOW);					// CS = LOW -> Start SPI communication
    delayMicroseconds(1);                   // CS fall to output enable = max. 100ns

    for (uint32_t i = 0; i < Length; i++)
    {
        digitalWrite(Pin(GPIO_MAX6675_CLK), LOW);
        delayMicroseconds(1);                           // CLK pulse width low = min. 100ns / CLK fall to output valid = max. 40ns
        dataIn <<= 1;
        if(digitalRead(Pin(GPIO_MAX6675_DO)))
            dataIn |= 1;
        digitalWrite(Pin(GPIO_MAX6675_CLK), HIGH);
        delayMicroseconds(1);                           // CLK pulse width high = min. 100ns
    }

    digitalWrite(csPin, HIGH);				// CS = HIGH -> End SPI communication
    digitalWrite(Pin(GPIO_MAX6675_CLK), LOW);
    return dataIn;
}

void MAX6675_show(struct MAX6675_sensorStruct *sensor, bool Json){
    char probetemp[33];
    dtostrfd(sensor->lastResult.ProbeTemperature, Settings.flag2.temperature_resolution, probetemp);

    if(Json){
        ResponseAppend_P(
			PSTR(
				",\"MAX 6675 \":{\"" D_JSON_PROBETEMPERATURE "\":%s,\"" D_JSON_ERROR "\":%d}"
			), probetemp, sensor->lastResult.ErrorCode
		);
#ifdef USE_DOMOTICZ
        if (0 == tele_period) {
          DomoticzSensor(DZ_TEMP, probetemp);
        }
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
        if (0 == tele_period) {
          KnxSensor(KNX_TEMPERATURE, sensor->lastResult.ProbeTemperature);
        }
#endif  // USE_KNX
	} else {
#ifdef USE_WEBSERVER
		WSContentSend_PD(HTTP_SNS_TEMP, "MAX6675", probetemp, TempUnit());
#endif  // USE_WEBSERVER
	}
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns92(uint8_t function)
{
	bool result = false;
	if(PinUsed(GPIO_MAX6675_CLK) && PinUsed(GPIO_MAX6675_DO)){

		switch (function) {
			case FUNC_INIT:
				if(! MAX6675_initialized)
					MAX6675_init();
				break;
			case FUNC_EVERY_SECOND:
//				MAX6675_getResult(&MAX6675_sensors[0]);
				break;
			case FUNC_JSON_APPEND:
//				MAX6675_show(&MAX6675_sensors[0], true);
				break;
#ifdef USE_WEBSERVER
			case FUNC_WEB_SENSOR:
				MAX6675_show(&MAX6675_sensors[0], false);
				break;
#endif	// USE_WEBSERVER
		}
	}
	return result;
}

#endif  // USE_MAX31855
