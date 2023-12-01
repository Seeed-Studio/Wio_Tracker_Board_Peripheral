#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_Storage.hpp>
#include <Tracker_Peripheral.hpp>



static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

uint8_t buf[40];
uint8_t size = 0;

uint8_t custom_data_buf[8];
uint8_t custom_data_len = 0;

uint32_t start_sensor_read_time = 0; 

static constexpr int adc_pin1 = A1;
static constexpr int digital_pin0 = D0;

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(100); 

    printf("Wio Tracker 1110 Custom Datas Uplink Example\r\n");

    tracker_peripheral.begin();

    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters();

    wm1110_geolocation.begin(Track_Scan_Gps,true);

    wm1110_geolocation.run();
    
}

void loop()
{   
    uint32_t now_time = 0;
    uint32_t consume_time = 0;
    uint32_t sleepTime = wm1110_geolocation.trackProcess();

    if(wm1110_geolocation.time_sync_flag)
    {
        if(sleepTime > 500)
        { 
            now_time = smtc_modem_hal_get_time_in_ms();
            if(now_time - start_sensor_read_time > 120000 ||(start_sensor_read_time == 0))
            {
                uint32_t adc_value1 = analogRead(adc_pin1);
                uint32_t pin_value1 = digitalRead( digital_pin0 );
                memcpyr(custom_data_buf,(uint8_t*)&adc_value1,4);
                memcpyr(&custom_data_buf[4],(uint8_t*)&pin_value1,4);
                custom_data_len = 8;           
                tracker_peripheral.packUplinkCustomDatas(custom_data_buf,custom_data_len);
                tracker_peripheral.displayUplinkCustomDatas();
                tracker_peripheral.getUplinkCustomDatas(buf, &size);
                // Insert all sensor data to lora tx buffer
                wm1110_geolocation.insertIntoTxQueue(buf,size);
                start_sensor_read_time = smtc_modem_hal_get_time_in_ms( );
                consume_time = start_sensor_read_time - now_time; 
                sleepTime = sleepTime - consume_time;
            }
        }
    }    

    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
