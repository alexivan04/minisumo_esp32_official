#include "vl53l0x_esp32.h"

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

#define BYTES_PER_DWORD        4
#define BYTES_PER_WORD         2

void i2c_master_init() 
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 
        I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

VL53L0X_Error VL53L0X_device_initialise(VL53L0X_Dev_t *pDevice, uint8_t new_i2c_addr, uint32_t RangeProfile) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    Status = VL53L0X_DataInit(pDevice); 
    if(Status != VL53L0X_ERROR_NONE) return Status;

    //set address
    new_i2c_addr &= 0x7F;
    Status = VL53L0X_SetDeviceAddress(pDevice, new_i2c_addr * 2); // 7->8 bit
    if(Status != VL53L0X_ERROR_NONE) return Status;
    pDevice->i2c_address = new_i2c_addr;
    ESP_LOGI("VL53L0X", "New address: %d", pDevice->i2c_address);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    
    Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_PerformRefCalibration(pDevice,
        	&VhvSettings, &PhaseCal); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
    Status = VL53L0X_PerformRefSpadManagement(pDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
    Status = VL53L0X_SetLimitCheckEnable(pDevice,
        	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE) return Status;
   
    Status = VL53L0X_SetLimitCheckValue(pDevice,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            (FixPoint1616_t)(0.1*65536));
	if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetLimitCheckValue(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            (FixPoint1616_t)(60*65536));			
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		RangeProfile);
	if(Status != VL53L0X_ERROR_NONE) return Status;
	   
    Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
	        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    return Status;
}

VL53L0X_Error VL53L0X_SingleRanging(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *MeasuredData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    Status = VL53L0X_PerformSingleRangingMeasurement(pDevice, MeasuredData);

    /* for accuracy average several samples
    uint32_t ranging=0;
    uint32_t valid_count=0;
    int i;
    for(i=0; i<10; i++){
        Status = VL53L0X_PerformSingleRangingMeasurement(pDevice,
                &RangingMeasurementData);
        if (Status == VL53L0X_ERROR_NONE && RangingMeasurementData.RangeStatus == 0) {
            ranging += RangingMeasurementData.RangeMilliMeter;
            valid_count++;
        }
        
        if (Status != VL53L0X_ERROR_NONE) break;
    }
    
    if (valid_count == 0) {
        Status = VL53L0X_ERROR_RANGE_ERROR;
    } else {
        *MeasuredData = ranging/valid_count;
    }
    */ 
    return Status;
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t dataReady = 0;
    uint32_t start_time = xTaskGetTickCount();

    while (1) {
        Status = VL53L0X_GetMeasurementDataReady(pDevice, &dataReady);
        if ((dataReady == 0x01) || Status != VL53L0X_ERROR_NONE || xTaskGetTickCount() - start_time >= pdMS_TO_TICKS(10)) {
            break;
        }
    }
    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t stopCompleted = 1;
    uint32_t start_time = xTaskGetTickCount();

    // Wait until stop is complete or timeout occurs
    while (1) {
        Status = VL53L0X_GetStopCompletedStatus(pDevice, &stopCompleted);
        if (Status != VL53L0X_ERROR_NONE || stopCompleted == 0x00) {
            break; // Break if stop is completed or an error occurs
        }

        // Check for timeout (200 ms in this example)
        if (xTaskGetTickCount() - start_time >= pdMS_TO_TICKS(200)) {
            Status = VL53L0X_ERROR_TIME_OUT; // Set timeout error
            break;
        }
        
        // Optional: Add a small delay to avoid busy waiting
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust the delay as needed
    }

    return Status; // Return the final status
}



VL53L0X_Error VL53L0X_ContinuousRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasuredData, uint16_t RangeCount, uint16_t *validCount){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

    Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_StartMeasurement(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    *validCount=0;
    uint16_t vCount=0;
    for (int i=0; i < RangeCount; i++) {

        Status = WaitMeasurementDataReady(pDevice);
        if (Status != VL53L0X_ERROR_NONE) break;

        Status = VL53L0X_GetRangingMeasurementData(pDevice, &RangingMeasurementData);
        if (Status == VL53L0X_ERROR_NONE ) { 
            if (RangingMeasurementData.RangeStatus == 0) {
                MeasuredData[vCount++] = RangingMeasurementData.RangeMilliMeter;
                //printf("valid:%d, m:%d \n",vCount, RangingMeasurementData.RangeMilliMeter);
            // Clear the interrupt
            }
            VL53L0X_ClearInterruptMask(pDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            VL53L0X_PollingDelay(pDevice);
        }
    }
    *validCount = vCount;

    Status = VL53L0X_StopMeasurement(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = WaitStopCompleted(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_ClearInterruptMask(pDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return Status;
}

void init_xshuts(uint8_t *dist_sensors_xshuts, int cnt)
{
    for(int i = 0; i < cnt; i++)
    {
        gpio_reset_pin(dist_sensors_xshuts[i]);
        gpio_set_direction(dist_sensors_xshuts[i], GPIO_MODE_OUTPUT);
        gpio_set_level(dist_sensors_xshuts[i], false);
    }
}
 
void set_all_xshut_states(uint8_t *dist_sensors_xshuts, bool running, int cnt)
{
    for(int i = 0; i < cnt; i++)
        gpio_set_level(dist_sensors_xshuts[i], running);
}

void set_dist_sensors_consecutive_addrs(uint8_t *dist_sensors_addrs, uint8_t start_address, int cnt)
{
    for(int i = 0; i < cnt; i++)
        dist_sensors_addrs[i] = start_address + i;
}

VL53L0X_Error init_dist_sensor(VL53L0X_Dev_t *pDevice, uint8_t XSHUT, uint8_t address, uint32_t range_profile)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    pDevice->i2c_address      =  0x29; //default address
    pDevice->i2c_port_num     =  I2C_MASTER_NUM;

    gpio_set_level(XSHUT, true);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    //NOTE: for VL53L0X_SetDeviceAddress, it was dividing the given addres by 2. i removed it from the definition, now it works 
    Status = VL53L0X_device_initialise(pDevice, address, range_profile);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    // Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    return Status;
}