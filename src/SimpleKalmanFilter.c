#include "SimpleKalmanFilter.h"

float updateEstimate(SimpleKalmanFilter *self, float mea)
{
    self->_kalman_gain = self->_err_estimate / (self->_err_estimate + self->_err_measure);
    self->_current_estimate = self->_last_estimate + self->_kalman_gain *(mea - self->_last_estimate);
    self->_err_estimate = (1.0 - self->_kalman_gain) *self->_err_estimate + fabs(self->_last_estimate - self->_current_estimate) *self->_q;
    self->_last_estimate = self->_current_estimate;
    return self->_current_estimate;
}

void setMeasurementError(SimpleKalmanFilter *self, float mea_e)
{
    self->_err_measure = mea_e;
}

void setEstimateError(SimpleKalmanFilter *self, float est_e)
{
    self->_err_estimate = est_e;
}

void setProcessNoise(SimpleKalmanFilter *self, float q)
{
    self->_q = q;
}

float getKalmanGain(SimpleKalmanFilter *self)
{
    return self->_kalman_gain;
}

float getEstimateError(SimpleKalmanFilter *self)
{
    return self->_err_estimate;
}

void deleteKalman(SimpleKalmanFilter *self)
{
    return free(self);
}

Kalman* newSimpleKalmanFilter(float mea_e, float est_e, float q)
{
    SimpleKalmanFilter *self = (SimpleKalmanFilter*) malloc(sizeof(SimpleKalmanFilter));
    self->updateEstimate = &updateEstimate;
    self->setMeasurementError = &setMeasurementError;
    self->setEstimateError = &setEstimateError;
    self->setProcessNoise = &setProcessNoise;
    self->getKalmanGain = &getKalmanGain;
    self->getEstimateError = &getEstimateError;
    self->_err_measure = mea_e;
    self->_err_estimate = est_e;
    self->_q = q;
    self->_current_estimate = 0;
    self->_last_estimate = 0;
    self->_kalman_gain = 0;
    return self;
}