C-style Simple Kalman Filter Library for STM32 HAL
========================================

 ![KalmanFilter](images/kalman_filter_example_1.png)

This is a basic kalman filter library for unidimensional models that you can use with a stream of single values like barometric sensors, temperature sensors or even gyroscope and accelerometers.

* Take a look at this [youtube video](https://www.youtube.com/watch?v=4Q5kJ96YYZ4) to see the Kalman Filter working on a stream of values!

*Special thanks to Denys Sene for his amazing work in https://github.com/denyssene/SimpleKalmanFilter*

Repository Contents
-------------------
* **/src** - Source files for the library (.c, .h).

Basic Usage
-------------------
 * **e_mea: Measurement Uncertainty** - How much do we expect to our measurement vary 
 * **e_est: Estimation Uncertainty**  - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
 * **q: Process Variance** - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
 
```c
int main()
{
	SimpleKalmanFilter* kf = newSimpleKalmanFilter(e_mea, e_est, q);
	while (1)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		float x = HAL_ADC_GetValue(&hadc1);
		float estimated_x = kf->updateEstimate(kf, x);
	}
}
``` 
 

License Information
-------------------

This is an _**open source**_ project! 
Please review the LICENSE.md file for license information. 
