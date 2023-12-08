#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_Storage.hpp>
#include <Tracker_Peripheral.hpp>


// Set a execution period
static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

// Instance
static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

// Packed custom data buffer
uint8_t buf[64];

// Packed custom data size
uint8_t size = 0;

// Custom data buffer
uint8_t custom_data_buf[8];

// Custom data len
uint8_t custom_data_len = 0;

// The time point of reading the sensor
uint32_t start_sensor_read_time = 0; 

// adc pin
static constexpr int adc_pin1 = A1;

//digital pin
static constexpr int digital_pin0 = D0;

void setup()
{
    // Initializes the debug output
    Serial.begin(115200);
    while (!Serial) delay(100);     // Wait for ready 

    printf("Wio Tracker 1110 Custom Datas Uplink Example\r\n");

    // Initializes the detected IIC peripheral sensors(include LIS3DHTR,SHT4x,Si1151,SGP40,DPS310)
    tracker_peripheral.begin();

    // Initializes the storage area
    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters(); // Load all parameters (WM1110_Param_Var.h)

    // Set the location mode to GNSS and uplink the data to SenseCAP platform
    wm1110_geolocation.begin(Track_Scan_Gps,true);

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
            if(now_time - start_sensor_read_time > 120000 ||(start_sensor_read_time == 0)) // Periodic collection of custom sensor data
            {
                uint32_t adc_value1 = analogRead(adc_pin1); // Get the sensor adc value
                uint32_t pin_value1 = digitalRead( digital_pin0 ); // Get the sensor Hall status

                // Put in custom_data_buf(4 bytes of data in a group)
                memcpyr(custom_data_buf,(uint8_t*)&adc_value1,4);
                memcpyr(&custom_data_buf[4],(uint8_t*)&pin_value1,4);

                // Set custom_data_len
                custom_data_len = 8;     

                // Pack datas      
                tracker_peripheral.packUplinkCustomDatas(custom_data_buf,custom_data_len);

                // Dispaly packed datas
                tracker_peripheral.displayUplinkCustomDatas();

                // Get packed datas
                tracker_peripheral.getUplinkCustomDatas(buf, &size);

                // Insert packed datas to lora tx buffer
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
