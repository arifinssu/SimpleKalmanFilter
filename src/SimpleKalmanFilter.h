#ifndef SimpleKalmanFilter_H
#define SimpleKalmanFilter_H

#ifdef __cplusplus
extern "C"
{
#endif

    #include <math.h>
    #include <stdio.h>
    #include <stdlib.h>

   	typedef struct SimpleKalmanFilter SimpleKalmanFilter;

    struct SimpleKalmanFilter
    {
        float _err_measure;
        float _err_estimate;
        float _q;
        float _current_estimate;
        float _last_estimate;
        float _kalman_gain;

        float(*updateEstimate)(SimpleKalmanFilter *self, float mea);
        void(*setMeasurementError)(SimpleKalmanFilter *self, float mea_e);
        void(*setEstimateError)(SimpleKalmanFilter *self, float est_e);
        void(*setProcessNoise)(SimpleKalmanFilter *self, float q);
        float(*getKalmanGain)(SimpleKalmanFilter *self);
        float(*getEstimateError)(SimpleKalmanFilter *self);
        void(*deleteKalman)(SimpleKalmanFilter *self);
    };

    SimpleKalmanFilter* newSimpleKalmanFilter(float mea_e, float est_e, float q);

#ifdef __cplusplus
}
#endif

#endif //SimpleKalmanFilter_H