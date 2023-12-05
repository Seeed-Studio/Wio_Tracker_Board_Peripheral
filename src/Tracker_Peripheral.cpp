#include "Tracker_Peripheral.hpp"

#include <LbmWm1110.hpp>
#include <Lbmx.hpp>

#include <Lbm_Modem_Common.hpp>
#include <WM1110_Geolocation.hpp>


#include <functional>
#include <tuple>
#include "internal/nrf_hal/gpio.hpp"
#include "internal/nrf_hal/gpiote.hpp"

static VOCGasIndexAlgorithm voc_algorithm(10);   // Used to calculate voc_index

//digital pin
static constexpr int ultrasonic_pin    = D0;
static Ultrasonic ultrasonic(ultrasonic_pin);

static SensirionI2CSht4x sht4x;
static LIS3DHTR<TwoWire> lis3dhtr; 
static SensirionI2CSgp41 sgp41;
static Si115X si1151;
Dps310 Dps310PressureSensor = Dps310();

static constexpr int lis3dhtr_irq_pin = PIN_LIS3DHTR_INT2;   

//analog pin
static constexpr int sound_adc_pin = A0;

bool Tracker_Peripheral::lis3dhtr_irq_flag = false;
bool Tracker_Peripheral::userbutton_irq_flag = false;


Tracker_Peripheral* Tracker_Peripheral::instance_ = nullptr;


#if 1 // public function
Tracker_Peripheral& Tracker_Peripheral::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new Tracker_Peripheral();
    }

    return *instance_;
}

Tracker_Peripheral::Tracker_Peripheral(void)
{

}

void Tracker_Peripheral::begin( void )
{
    // power on
    powerOn();

    //init-----------------
    // LIS3DHTR
    if(detectIICSlaveAvailable(LIS3DHTR_I2C_ADDRESS))
    {
        pinMode(lis3dhtr_irq_pin, INPUT);
        attachInterrupt(lis3dhtr_irq_pin, &Tracker_Peripheral::lis3dhtr_irq_callback, ISR_DEFERRED | FALLING); //RISING

        lis3dhtr.begin(Wire, LIS3DHTR_ADDRESS_UPDATED); 
        lis3dhtr.setInterrupt(); 
    } 
    //SHT4x
    if(detectIICSlaveAvailable(SHT4X_I2C_ADDRESS))
    {
        sht4x.begin(Wire);
    }
    // Si1151
    if(detectIICSlaveAvailable(SI1151_I2C_ADDRESS))  
    {
        si1151.Begin();
    }
    //SGP41
    if(detectIICSlaveAvailable(SGP41_I2C_ADDRESS))
    {
        sgp41.begin(Wire);
    }
    //DPS310
    if(detectIICSlaveAvailable(DPS310_IIC_ADDRESS))
    {
        Dps310PressureSensor.begin(Wire);
    }
    uplink_temperature = 0x8000;
    uplink_humidity = 0x80;
    uplink_x = 0x8000;
    uplink_y = 0x8000;
    uplink_z = 0x8000;
    uplink_uv = 0x8000;
    sensor_data.sgp41_voc_index = 0x8000;             // sgp41 voc index data
    sensor_data.sound_val = 0x8000;                   // sound sensor data
    sensor_data.ultrasonic_distance_cm = 0x8000;      // ultrasonic distance sensor data
    sensor_data.dps310_pressure_val = 0x800000;         // dps310 Atmospheric pressure data
    iic_mutex_status = false;
     
}

void Tracker_Peripheral::powerOn(void)
{
    digitalWrite(PIN_POWER_SUPPLY_GROVE, HIGH);   //grove power on
    pinMode(PIN_POWER_SUPPLY_GROVE, OUTPUT);   
    delay(100);    
}

bool Tracker_Peripheral::detectIICSlaveAvailable(uint8_t slave_addr)
{
    Wire.begin();
    if(slave_addr<1 || slave_addr >127)
    {
        return false;
    }
    Wire.beginTransmission(slave_addr);
    if ( 0 == Wire.endTransmission() )
    {
        return true;
    } 
    return false;  
}

bool Tracker_Peripheral::measureSHT4xDatas(float* temperature, float* humidity)
{
    uint16_t error; 
    if(iic_mutex_status == true)
        return false;
    iic_mutex_status = true;
    uplink_temperature = 0x8000;
    uplink_humidity = 0x80;

#if((SENSOR_HOT_SWAPPABLE_DETECT == 1)&&(SHT4x_HOT_SWAPPABLE_DETECT == 1))
    if(detectIICSlaveAvailable(SHT4X_I2C_ADDRESS))
    {
        sht4x.begin(Wire);
    } 
    else
    {
        iic_mutex_status = false;  
        return false;
    } 
#endif

    error = sht4x.measureHighPrecision(*temperature, *humidity);
    if (error == 0)
    {
        uint8_t humi = *humidity;
        if(humi >= 100) //Prevents the humidity calculation from being greater than 100%
        {
            *humidity = 100.f;
        }
        iic_mutex_status = false;
        sensor_data.sht4x_temperature = *temperature;
        sensor_data.sht4x_humidity = *humidity;
        uplink_temperature = sensor_data.sht4x_temperature*10;
        uplink_humidity = sensor_data.sht4x_humidity;
        return true;
    }
    iic_mutex_status = false;
    return false;
}

bool Tracker_Peripheral::measureLIS3DHTRDatas(float* x, float* y, float* z)
{
    uint8_t flag = 0;
    if(iic_mutex_status == true)
        return false;
    iic_mutex_status = true;

    uplink_x = 0x8000;
    uplink_y = 0x8000;
    uplink_z = 0x8000;
#if((SENSOR_HOT_SWAPPABLE_DETECT == 1)&&(LIS3DHTR_HOT_SWAPPABLE_DETECT == 1))
    if(detectIICSlaveAvailable(LIS3DHTR_I2C_ADDRESS))
    {
        pinMode(lis3dhtr_irq_pin, INPUT);
        attachInterrupt(lis3dhtr_irq_pin, &Tracker_Peripheral::lis3dhtr_irq_callback, ISR_DEFERRED | FALLING); //RISING

        lis3dhtr.begin(Wire, LIS3DHTR_ADDRESS_UPDATED); 
        lis3dhtr.setInterrupt(); 
    } 
    else
    {
        iic_mutex_status = false;
        return false;
    }
#endif

    lis3dhtr.getIntStatus(&flag);
    lis3dhtr.getAcceleration(x, y, z);
    sensor_data.lis3dhtr_x = *x;
    sensor_data.lis3dhtr_y = *y;
    sensor_data.lis3dhtr_z = *z;

    uplink_x = 100*sensor_data.lis3dhtr_x*acc_convert;
    uplink_y = 100*sensor_data.lis3dhtr_y*acc_convert;
    uplink_z = 100*sensor_data.lis3dhtr_z*acc_convert;


    iic_mutex_status = false;
    return true;   
}

bool Tracker_Peripheral::measureSGP41Datas(float temperature, float humidity, int32_t* voc_index)
{
    uint16_t compensationT;
    uint16_t compensationRh;
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    uint16_t error;

    if(iic_mutex_status == true)
        return false;
    iic_mutex_status = true;

    sensor_data.sgp41_voc_index = 0x8000;  

#if(SENSOR_HOT_SWAPPABLE_DETECT == 1)
    if(detectIICSlaveAvailable(SGP41_I2C_ADDRESS))
    {
        sgp41.begin(Wire);
    }
    else
    {
        iic_mutex_status = false;
        return false;
    }

#endif
    //temperature and humidity for SGP internal compensation

    // convert temperature and humidity to ticks as defined by sgp41
    // interface
    // NOTE: in case you read RH and T raw signals check out the
    // ticks specification in the datasheet, as they can be different for
    // different sensors
    compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);

    // Request a first measurement to heat up the plate (ignoring the result)
    error = sgp41.measureRawSignals(compensationRh, compensationT, srawVoc,srawNox);
    if (error) 
    {
        iic_mutex_status = false;
        return false;
    }
    delay(140);
    // Request the measurement values
    error = sgp41.measureRawSignals(compensationRh, compensationT, srawVoc,srawNox);
    if (error) 
    {
        iic_mutex_status = false;
        return false;
    }  
    error = sgp41.turnHeaterOff(); //for low power consumption
    if (error) 
    {
        iic_mutex_status = false;
        return false;
    }
    // Process raw signals by Gas Index Algorithm to get the VOC index values
    *voc_index = voc_algorithm.process(srawVoc); 
    sensor_data.sgp41_voc_index = *voc_index;
    iic_mutex_status = false; 
    return true;       
}

bool Tracker_Peripheral::measureSi1151Datas(float* UV_val)
{
    if(iic_mutex_status == true)
        return false;
    iic_mutex_status = true;
    uplink_uv = 0x8000;
#if(SENSOR_HOT_SWAPPABLE_DETECT == 1)
    if(detectIICSlaveAvailable(SI1151_I2C_ADDRESS))  
    {
        si1151.Begin();
    }
    else
    {
        return false;
    }
#endif
    *UV_val = si1151.ReadHalfWord_UV();
    sensor_data.UV_val = *UV_val;
    uplink_uv = sensor_data.UV_val*100;
    iic_mutex_status = false;
    return true;
}

bool Tracker_Peripheral::measureDPS310(uint32_t* pressure_val)
{
	float Detection_pressure;
	uint8_t oversampling = 7;

    if(iic_mutex_status == true)
        return false;
    iic_mutex_status = true;
    sensor_data.dps310_pressure_val = 0x800000;
#if(SENSOR_HOT_SWAPPABLE_DETECT == 1)
    if(detectIICSlaveAvailable(DPS310_IIC_ADDRESS))
    {
        Dps310PressureSensor.begin(Wire);
    }
    else
    {
        iic_mutex_status = false;
        return false;
    }
#endif

    Dps310PressureSensor.measurePressureOnce(Detection_pressure, oversampling);
    *pressure_val = Detection_pressure;
    sensor_data.dps310_pressure_val = *pressure_val;

    iic_mutex_status = false;
    return true;

}

bool Tracker_Peripheral::measureSoundAdc(uint16_t* sound_adc)
{
    uint32_t sum = 0;
    int  adc_val = 0;

    for(int i=0; i<16; i++)
    {
        sum += analogRead(sound_adc_pin);
        delay(2);
    }
    adc_val = sum >>4;

    *sound_adc = (uint16_t)((float)adc_val * mv_per_lsb);

    sensor_data.sound_val = *sound_adc;
    return true;

}

bool Tracker_Peripheral::measureUltrasonicDistance(uint16_t* distance_cm)
{
    *distance_cm = ultrasonic.MeasureInCentimeters(40000);     //40ms timeout 4m  => 4*2 /340 *1000 = 23.5ms   Velocity of sound:340m/s
    if(*distance_cm > 400)
    {
        *distance_cm = 0xFFFF;
    }
    sensor_data.ultrasonic_distance_cm = *distance_cm;
    return true;
}

void Tracker_Peripheral::clearShockFlag( void )
{
    lis3dhtr_irq_flag = false;
}

void Tracker_Peripheral::getLIS3DHTRIrqStatus( bool *irqstatus )
{
    *irqstatus = lis3dhtr_irq_flag;
}

void Tracker_Peripheral::setUserButton( void )
{
    pinMode(PIN_BUTTON1, INPUT);
    attachInterrupt(PIN_BUTTON1, &Tracker_Peripheral::user_button_irq_callback, ISR_DEFERRED | CHANGE);     
}

void Tracker_Peripheral::clearUserButtonFlag( void )
{   
    userbutton_irq_flag = false;
}

void Tracker_Peripheral::getUserButtonIrqStatus( bool *irqstatus )
{
    *irqstatus = userbutton_irq_flag;
}

void Tracker_Peripheral::displaySensorDatas( void )
{
    // temperture 
    if( uplink_temperature != 0x8000 )
    {
        printf("Temperature:%0.2f\r\n",sensor_data.sht4x_temperature);        
    }
    // humidity
    if( uplink_humidity != 0x80 )
    {
        printf("Humidity:%0.2f%%\r\n",sensor_data.sht4x_humidity);        
    }
    // pressure
    if( sensor_data.dps310_pressure_val != 0x800000 )
    {
        printf("Atmospheric pressure:%lu\r\n",sensor_data.dps310_pressure_val);
    }
    // distance
    if( sensor_data.ultrasonic_distance_cm != 0x8000 )
    {
        printf("Ultrasonic distance:%d cm\r\n",sensor_data.ultrasonic_distance_cm);
    }
    // uv
    if( uplink_uv != 0x8000 )
    {
        printf("Ultraviolet illumination:%0.2f\r\n",sensor_data.UV_val); 
    }
    //voc index
    if( sensor_data.sgp41_voc_index != 0x8000 )
    {
        printf("tVOC index:%ld\r\n",sensor_data.sgp41_voc_index); 
    }
    //sound
    if( sensor_data.sound_val != 0x8000 )
    {
        printf("Sound analog:%d mV\r\n",sensor_data.sound_val);       
    }
    //3-axis
    if( (uplink_x !=0x8000)&&(uplink_y !=0x8000)&&(uplink_z !=0x8000) )
    {
        printf("3-axis: x:%0.2f y:%0.2f z:%0.2f\r\n",sensor_data.lis3dhtr_x,sensor_data.lis3dhtr_y,sensor_data.lis3dhtr_z);
    }
}

void Tracker_Peripheral::packUplinkSensorDatas( void )
{
    //Packet ID
    sensor_data_buf[0] = DATA_ID_UP_PACKET_FACT_SENSOR;

    //event  status
    sensor_data_buf[1] = ( sensor_event_state >> 16 ) & 0xff;
    sensor_data_buf[2] = ( sensor_event_state >> 8 ) & 0xff;
    sensor_data_buf[3] = ( sensor_event_state ) & 0xff;

    //utc time
    uint32_t utc = apps_modem_common_get_utc_time( );
    memcpyr( &sensor_data_buf[4], ( uint8_t * )( &utc ), 4 );

    //temperture & humidity
    memcpyr( &sensor_data_buf[8], ( uint8_t * )( &uplink_temperature ), 2 );  //temperature
    sensor_data_buf[10] = uplink_humidity;                              //humidity

    //Atmospheric pressure
    sensor_data_buf[11] = (sensor_data.dps310_pressure_val>>16)&0xFF;    //pressure
    sensor_data_buf[12] = (sensor_data.dps310_pressure_val>>8)&0xFF;
    sensor_data_buf[13] = sensor_data.dps310_pressure_val&0xFF;        

    //ultrasonic
    memcpyr( &sensor_data_buf[14], ( uint8_t * )( &sensor_data.ultrasonic_distance_cm ), 2 );    //distance  

    //UV
    memcpyr( &sensor_data_buf[16], ( uint8_t * )( &uplink_uv ), 2 );  //UV

    //voc index
    sensor_data_buf[18] = (sensor_data.sgp41_voc_index>>8)&0xFF;        //voc_index
    sensor_data_buf[19] = sensor_data.sgp41_voc_index&0xFF;

    //sound
    memcpyr( &sensor_data_buf[20], ( uint8_t * )( &sensor_data.sound_val ), 2 );    //sound vcc

    //3-axis
    memcpyr( &sensor_data_buf[22], ( uint8_t * )( &uplink_x ), 2 );    //x-axis
    memcpyr( &sensor_data_buf[24], ( uint8_t * )( &uplink_y ), 2 );    //y-axis
    memcpyr( &sensor_data_buf[26], ( uint8_t * )( &uplink_z ), 2 );    //z-axis

    sensor_data_len = 28;

    sensor_event_state = 0;

}


void Tracker_Peripheral::getUplinkSensorDatas( uint8_t *buf,uint8_t *size )
{
    memcpy(buf,sensor_data_buf,sensor_data_len);
    *size = sensor_data_len;
}  

void Tracker_Peripheral::displayUplinkSensorDatas( void )
{
    printf("Sensor datas:");
    for(uint8_t u8i = 0; u8i < sensor_data_len; u8i++ )
    {
        printf("%02x ",sensor_data_buf[u8i]);
    }
    printf("\r\n");      
}

void Tracker_Peripheral::setSensorEventStatus( uint32_t state )
{
    sensor_event_state |= state;
}

bool Tracker_Peripheral::packUplinkCustomDatas( uint8_t *buf,uint8_t size )
{
    //it's must be 4bytes/group
    if((size > 40) || (size%4 != 0)||(size == 0))
    {
        return false;
    }
    else
    {
        //Packet ID
        custom_data_buf[0] = DATA_ID_UP_PACKET_USER_SENSOR;

        //event  status
        custom_data_buf[2] = ( costom_event_state >> 16 ) & 0xff;
        custom_data_buf[3] = ( costom_event_state >> 8 ) & 0xff;
        custom_data_buf[4] = ( costom_event_state ) & 0xff;

        uint32_t utc = apps_modem_common_get_utc_time( );
        
        //utc time
        memcpyr( custom_data_buf + 5, ( uint8_t * )( &utc ), 4 );

        //custom datas
        memcpy(&custom_data_buf[9],buf,size);
        custom_data_len = 9 + size;  

        //package len
        custom_data_buf[1] = custom_data_len;

        costom_event_state = 0;

        return true;   
    }

    return true;   

}

void Tracker_Peripheral::getUplinkCustomDatas( uint8_t *buf,uint8_t *size )
{
    memcpy(buf,custom_data_buf,custom_data_len);
    *size = custom_data_len;
}  

void Tracker_Peripheral::displayUplinkCustomDatas( void )
{
    printf("Custom datas:");
    for(uint8_t u8i = 0; u8i < custom_data_len; u8i++ )
    {
        printf("%02x ",custom_data_buf[u8i]);
    }
    printf("\r\n");      
}

void Tracker_Peripheral::setCustomEventStatus( uint32_t state )
{
    costom_event_state |= state;
}
#endif

#if 1 // private function

bool Tracker_Peripheral::detectSHT4xAvailable(void)
{
    if(detectIICSlaveAvailable(SHT4X_I2C_ADDRESS))
    {
        return true;
    } 
    return false;  
}

bool Tracker_Peripheral::detectLIS3DHTRAvailable(void)
{
    if(detectIICSlaveAvailable(LIS3DHTR_I2C_ADDRESS))
    {
        return true;
    } 
    return false;  
}

bool Tracker_Peripheral::detectSi1151Available(void)
{
    if(detectIICSlaveAvailable(SI1151_I2C_ADDRESS))
    {
        return true;
    } 
    return false;  
}
bool Tracker_Peripheral::detectSGP41Available(void)
{
    if(detectIICSlaveAvailable(SGP41_I2C_ADDRESS))
    {
        return true;
    } 
    return false;  
}
bool Tracker_Peripheral::detectDPS310Available(void)
{
    if(detectIICSlaveAvailable(DPS310_IIC_ADDRESS))
    {
        return true;
    } 
    return false; 
}

void Tracker_Peripheral::lis3dhtr_irq_callback(void)
{

    if(lis3dhtr_irq_flag == false)
    {
        lis3dhtr_irq_flag = true;  
    }
} 

void Tracker_Peripheral::user_button_irq_callback(void)
{
    static uint32_t key_press_cnt = 0;
    static uint32_t key_release_cnt = 0;    
    static uint8_t btn_state = 0; 

    if( btn_state == 0 )
    {
        if( 0 == digitalRead( PIN_BUTTON1 ))
        {
            btn_state = 1;
            key_press_cnt = smtc_modem_hal_get_time_in_ms( );
        }
    }
    else if( btn_state == 1 )
    {
        if( 1 == digitalRead( PIN_BUTTON1 ))
        {
            btn_state = 0;
            key_release_cnt = smtc_modem_hal_get_time_in_ms( );
            if(( key_release_cnt - key_press_cnt ) > 10 ) //press
            {
                if(userbutton_irq_flag == false)
                {
                    userbutton_irq_flag = true;
                }            
            }
        }
    }
}

#endif 


Tracker_Peripheral tracker_peripheral;

