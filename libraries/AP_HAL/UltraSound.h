#pragma once
/*
 * struct related to adc
 * data to receive and process adc datas
 */
struct adcCapture_t {
    struct iio_device *device;
    struct iio_buffer *buffer;
    unsigned int buffer_size;
    struct iio_channel *channel;
    unsigned int freq;

     /* Used in order to match two echoes of two ADC acquisitions */
    unsigned short threshold_time_rejection;
};

class AP_HAL::UltraSound {
public:
    virtual void init(void) = 0;
    virtual int launch(void) = 0;
    virtual int capture(void) = 0;
    virtual int update_mode(float altitude) = 0;
    virtual unsigned int get_buffer_size() = 0;
    virtual struct adcCapture_t* get_capture() = 0;        
};
