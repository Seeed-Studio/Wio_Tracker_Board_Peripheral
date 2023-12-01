#ifndef _TRACKER_PERIPHERAL_H_
#define _TRACKER_PERIPHERAL_H_

#pragma once

#include <cstdio>
#include <cstdlib>

#include <Lbm_Modem_Common.hpp>

#include "LIS3DHTR.h"
#include "SensirionI2CSht4x.h"
#include "SensirionI2CSgp41.h"
#include "VOCGasIndexAlgorithm.h"
#include "Dps310.h"
#include "Si115X.h"
#include "Ultrasonic.h"


//If defined as 1:detect iic slave device hot swap function
#ifndef SENSOR_HOT_SWAPPABLE_DETECT
#define SENSOR_HOT_SWAPPABLE_DETECT     1
#endif

#ifndef LIS3DHTR_HOT_SWAPPABLE_DETECT
#define LIS3DHTR_HOT_SWAPPABLE_DETECT     0
#endif

#ifndef SHT4x_HOT_SWAPPABLE_DETECT
#define SHT4x_HOT_SWAPPABLE_DETECT     0
#endif

//IIC slave device addr
#define LIS3DHTR_I2C_ADDRESS                (LIS3DHTR_ADDRESS_UPDATED)
#define SHT4X_I2C_ADDRESS                   (0x44)
#define SI1151_I2C_ADDRESS                  (0x53)
#define SGP41_I2C_ADDRESS                   (0x59)
#define DPS310_IIC_ADDRESS                  (0x77)

typedef struct Sensor_Data
{
    float sht4x_temperature; // sht41 temperture data
    float sht4x_humidity;    // sht41 humidity data
    float lis3dhtr_x;    // lis3dhtr X-axis data
    float lis3dhtr_y;    // lis3dhtr Y-axis data
    float lis3dhtr_z;    // lis3dhtr Z-axis data
    int32_t sgp41_voc_index;// sgp41 voc index data
    float UV_val;    // si1151 uv data
    uint16_t sound_val;  // sound sensor data
    uint16_t ultrasonic_distance_cm; // ultrasonic distance sensor data
    uint32_t dps310_pressure_val;    // dps310 Atmospheric pressure data
}Sensor_Data_t;


class Tracker_Peripheral
{
    private:
        static Tracker_Peripheral* instance_;

        Sensor_Data_t sensor_data;

        static constexpr float acc_convert = 9.80F;
        static constexpr float mv_per_lsb = 3600.0F/1024.0F;             // 10-bit ADC with 3.6V input range 

        bool iic_mutex_status;         // Ensure that devices are mutually exclusive when using the iic bus   
        static bool lis3dhtr_irq_flag;

        static bool userbutton_irq_flag;

        uint32_t sensor_event_state = 0;
        uint32_t costom_event_state = 0;

        uint8_t sensor_data_buf[64];
        uint8_t sensor_data_len;
        int16_t uplink_temperature; 
        uint8_t uplink_humidity;    
        int16_t uplink_x; 
        int16_t uplink_y;   
        int16_t uplink_z;    
        uint16_t uplink_uv;    

        uint8_t custom_data_buf[64];
        uint8_t custom_data_len; 

    public:
        Tracker_Peripheral(void);  
        static Tracker_Peripheral& getInstance();

        void begin( void );

        /*!
        *  @brief  Start sensor power supply
        */   
        void powerOn(void);

        /*!
        *  @brief  Detect whether a slave IIC device exists
        *  @param slave_addr   
        *  @return bool
        */   
        bool detectIICSlaveAvailable(uint8_t slave_addr);

        /*!
        *  @brief  Measure the temperature and humidity data of the SHT4x sensor
        *  @param temperature   
        *  @param humidity   
        *  @return bool
        */           
        bool measureSHT4xDatas(float* temperature, float* humidity);

        /*!
        *  @brief  Measure the 3-Axis data of the LIS3DHTR sensor
        *  @param x   
        *  @param y   
        *  @param z  
        *  @return bool
        */   
        bool measureLIS3DHTRDatas(float* x, float* y, float* z);

        /*!
        *  @brief  Measure the tVOC data of the SGP41 sensor
        *  @param temperature   
        *  @param humidity  
        *  @param voc_index 
        *  @return bool
        */   
        bool measureSGP41Datas(float temperature, float humidity, int32_t* voc_index);

        /*!
        *  @brief  Measure the ambient light data of the Si1151 sensor
        *  @param UV_val   
        *  @return bool
        */  
        bool measureSi1151Datas(float* UV_val);

        /*!
        *  @brief  Measure the barometer data of the DPS310 sensor
        *  @param pressure_val   
        *  @return bool
        */  
        bool measureDPS310(uint32_t* pressure_val);

        /*!
        *  @brief  Measure the adc data of the Sound sensor
        *  @param sound_adc   
        *  @return bool
        */  
        bool measureSoundAdc(uint16_t* sound_adc);

        /*!
        *  @brief  Measure the Ultrasonic distance data of the Ultrasonic sensor
        *  @param distance_cm   
        *  @return bool
        */  
        bool measureUltrasonicDistance(uint16_t* distance_cm);

        /*!
        *  @brief  Clear the shock flag of the 3-axis sensor
        */  
        void clearShockFlag( void );

        /*!
        *  @brief  Get shock flag of the 3-axis sensor
        *  @param irqstatus         
        */          
        void getLIS3DHTRIrqStatus( bool *irqstatus );

        /*!
        *  @brief  Set user button
        */  
        void setUserButton( void );

        /*!
        *  @brief  Clear user button action flag
        */  
        void clearUserButtonFlag( void );

        /*!
        *  @brief  Get user button action flag
        *  @param irqstatus         
        */  
        void getUserButtonIrqStatus( bool *irqstatus );

        /*!
        *  @brief  Dispaly Sensor datas   
        */  
        void displaySensorDatas( void );  

        /*!
        *  @brief  Pack uplink sensor datas
        */  
        void packUplinkSensorDatas( void );  

        /*!
        *  @brief  Get uplink sensor datas
        *  @param buf 
        *  @param size         
        */  
        void getUplinkSensorDatas( uint8_t *buf,uint8_t *size );   

        /*!
        *  @brief  Display uplink sensor datas     
        */          
        void displayUplinkSensorDatas( void ); 

        /*!
        *  @brief  Set sensor event status  
        *  @param state      
        */ 
        void setSensorEventStatus( uint32_t state );

        /*!
        *  @brief  Pack uplink custom sensor datas     
        *  @param buf 
        *  @param size         
        */       
        bool packUplinkCustomDatas( uint8_t *buf,uint8_t size );   

        /*!
        *  @brief  Get uplink custom sensor datas     
        *  @param buf 
        *  @param size         
        */  
        void getUplinkCustomDatas( uint8_t *buf,uint8_t *size );   

        /*!
        *  @brief  Display uplink custom sensor datas     
        */ 
        void displayUplinkCustomDatas( void ); 

        /*!
        *  @brief  Set Custom sensor event status
        *  @param state           
        */ 
        void setCustomEventStatus( uint32_t state );

    private:
    
        bool detectSHT4xAvailable(void);
        
        bool detectLIS3DHTRAvailable(void);

        bool detectSi1151Available(void);

        bool detectSGP41Available(void);

        bool detectDPS310Available(void);

        static void lis3dhtr_irq_callback(void);

        static void user_button_irq_callback(void);
};

extern Tracker_Peripheral tracker_peripheral;

#endif /* _TRACKER_PERIPHERAL_H_ */



