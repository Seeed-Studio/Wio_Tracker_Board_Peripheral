#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_Storage.hpp>
#include <Tracker_Peripheral.hpp>

// Set a execution period
static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

// Instance
static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

// LIS3DHTR 3-axis data
float x; 
float y; 
float z;

// SHT4x Temperture & humidity
float temperature; 
float humidity;

// Packed data buffer
uint8_t buf[64];
// Packed data size
uint8_t size = 0;

// The time point of reading the sensor
uint32_t start_sensor_read_time = 0; 

// Period for reading sensor data
uint32_t sensor_read_priod = 0; 
void setup()
{
    // Initializes the debug output
    Serial.begin(115200);
    while (!Serial) delay(100);     // Wait for ready  

    printf("Wio Tracker 1110 Factory Datas Uplink Example\r\n");

    // Initializes the detected IIC peripheral sensors(include LIS3DHTR,SHT4x,Si1151,SGP40,DPS310)
    tracker_peripheral.begin();

    // Initializes the storage area    
    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters(); // Load all parameters (WM1110_Param_Var.h)

    // Set the location mode to GNSS and uplink the data to SenseCAP platform
    wm1110_geolocation.begin(Track_Scan_Gps,true);

    // Get period for reading sensor data
    sensor_read_priod = wm1110_geolocation.getSensorMeasurementPeriod();
    sensor_read_priod = sensor_read_priod*60*1000; // Convert to ms 

    // Start running
    wm1110_geolocation.run(); 
    
}

void loop()
{   
    uint32_t now_time = 0;
    uint32_t consume_time = 0;

    // Run process 
    // sleepTime is the desired sleep time for LoRaWAN's next task   
    uint32_t sleepTime = wm1110_geolocation.trackProcess(); 

    if(wm1110_geolocation.time_sync_flag) // The device has been synchronized time from the LNS
    {
        if(sleepTime > 500) // Free time
        { 
            now_time = smtc_modem_hal_get_time_in_ms();
            if(now_time - start_sensor_read_time > sensor_read_priod ||(start_sensor_read_time == 0)) // Periodic collection of custom sensor data
            {
                printf("Reading sensor data...\r\n");
                tracker_peripheral.measureLIS3DHTRDatas(&x,&y,&z); // Get the 3-axis data
                tracker_peripheral.measureSHT4xDatas(&temperature,&humidity); // Get the Temperture & humidity data

                // Pack datas      
                tracker_peripheral.packUplinkSensorDatas();

                // Display sensor raw datas  
                tracker_peripheral.displaySensorDatas();

                // Dispaly packed datas
                tracker_peripheral.displayUplinkSensorDatas();

                // Get packed datas
                tracker_peripheral.getUplinkSensorDatas( buf, &size );   

                // Insert all sensor data to lora tx buffer
                wm1110_geolocation.insertIntoTxQueue(buf,size);

                // Mark time point of reading the sensor
                start_sensor_read_time = smtc_modem_hal_get_time_in_ms( );
                consume_time = start_sensor_read_time - now_time; 
                sleepTime = sleepTime - consume_time;
            }
        }
    } 
       
    //delay
    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
