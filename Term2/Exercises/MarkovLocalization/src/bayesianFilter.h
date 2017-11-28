//============================================================================
// Name        : bayesianFilter.h
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#ifndef BAYESIANFILTER_H_
#define BAYESIANFILTER_H_

#include <vector>
#include <string>
#include <fstream>

#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"

class bayesianFilter {
public:
	//constructor:
	bayesianFilter();
	//deconstructor:
	virtual ~bayesianFilter();


	/*
	 *updates the belief of the car's position in response to the arrival of a measurement
	 */
	void process_measurement(const MeasurementPackage &measurements,
					                 const map &map_1d,
					                 help_functions &helpers);

	/*
	 * belief of state x: denotes the car's position belief at time t
	 * (it is a probability distribution over the space of positions)
	 */
	std::vector<float> bel_x ;

  
private:

	//flag, if filter is initialized
	bool is_initialized_;

	//precision of control information
	float control_std_;

	//precision of observations
	float observations_std_;

	//initial belief of state x
	std::vector<float> bel_x_init_;

};

#endif /* BAYESIANFILTER_H_ */
