//============================================================================
// Name        : bayesianFilter.cpp
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//map size
//1D resolution from 0 to 99m
const int MAP_SIZE = 100;

//constructor:
bayesianFilter::bayesianFilter() {

  //filter not initialized
	is_initialized_ = false;

	//initialize standard deviation of control
	//assume it is 1m!
	control_std_ = 1.0f;

	//initialize standard deviation of observations
	observations_std_ = 1.0f;

	//define size of believe of state vectors
	//fill in with zeros
	//same size as map (!!)
	bel_x.resize(MAP_SIZE, 0);
	bel_x_init_.resize(MAP_SIZE, 0);

}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
        						             				const map &map_1d,
                                        help_functions &helpers){

  /******************************************************************************
     * Set initial belief of state vector
     * Assumptions:
     *  Car is parked at a tree or street lamp +/- 1 meter
    ******************************************************************************/
  if (!is_initialized_) {

    //run over map
    for (unsigned int l=0; l<map_1d.landmark_list.size(); ++l) {

      //define landmark
      map::single_landmark_s landmark_temp;
      //get landmark from map
      landmark_temp = map_1d.landmark_list[l];

      //check if landmark position is in the range of state vector x
      if (landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init_.size()) {

        //cast float to int
        int position_x = int(landmark_temp.x_f);
        //set belief to 1 (initial values does not matter)
        bel_x_init_[position_x] = 1.0f;
        bel_x_init_[position_x - 1] = 1.0f;
        bel_x_init_[position_x + 1] = 1.0f;
      }
    }

    //normalize belief at time 0: sum of the beliefs must be one
    bel_x_init_ = helpers.normalize_vector(bel_x_init_);

    //set initialized flag to true
    is_initialized_ = true;

    /* ---- INITIAL BELIEF ---- */
    cout << "bel_x_init_: " << endl;
    for (int i=0; i<bel_x_init_.size(); ++i) {
      cout << bel_x_init_[i] << "\t";
    }
    cout << endl;

  }
	
  /******************************************************************************
   *  motion model and observation update
  ******************************************************************************/
  std::cout <<"-->motion model for state x ! \n" << std::endl;

  //get current observations and control information:
  MeasurementPackage::control_s controls = measurements.control_s_;
  MeasurementPackage::observation_s observations = measurements.observation_s_;

  //run over the whole state (index represents the pose in x)
  for (int i=0; i< bel_x.size(); ++i) {

    //index represents the pose in x
    float x_i = float(i);

    /**************************************************************************
     *  posterior for motion model
    **************************************************************************/

    // motion posterior (future belief)
    float posterior_motion = 0.0f;

    //loop over state space x_t-1 (convolution)
    for (unsigned int j=0; j< bel_x.size(); ++j) {
      float x_prev = float(j) ;

      float distance_ij =  x_i - x_prev;
//      float distance_ij =  pose_j - pose_i;

      /*transition model is modeled as a 1D normal distribution with
       *  mean: x_t - x_t-1
       *  std: control std (1.0f)
       * transition probabilities do not change over time */
      float transition_prob = helpers.normpdf(distance_ij,
                                    controls.delta_x_f,
                                    control_std_) ;


//      cout << "i=" << i << " j=" << j << " distance_ij=" << distance_ij << " transition_prob=" << transition_prob << endl;

      //motion model
      posterior_motion += transition_prob*bel_x_init_[j];
    }

    /**************************************************************************
     *  observation update:
    **************************************************************************/

    //define pseudo observation vector:
    //stores the distance between the landmarks and the current position (x_i)
    std::vector<float> pseudo_ranges;

    //define maximum distance
    float distance_max = 100;

    //loop over number of landmarks and estimate pseudo ranges
    for (unsigned int l=0; l< map_1d.landmark_list.size(); ++l) {

      //calculate pseudo_ranges
      float range_l = map_1d.landmark_list[l].x_f - x_i;

      //check, if distances are positive, and store positive range
      //car goes always forwards, negative distance are not possible!
      if(range_l > 0.0f) {
        pseudo_ranges.push_back(range_l);
      }
    }

    //sort pseudo range vector
    sort(pseudo_ranges.begin(), pseudo_ranges.end());

    //define observation posterior
    float posterior_obs = 1.0f;

    //run over current observations vector defined above:
    for (unsigned int z=0; z<observations.distance_f.size(); ++z) {

      //define min distance
      float pseudo_range_min;
      // set min distance either to the closet landmark
      if (pseudo_ranges.size() > 0) {
        pseudo_range_min = pseudo_ranges[0];
        //remove distance to closest landmark:
        //next observation must refer to the second closest landmark
        pseudo_ranges.erase(pseudo_ranges.begin());
      }
      // or if no landmarks exist to the maximum set distance
      // range spectrum of the sensor = [0, 100] meters
      else {
        pseudo_range_min = distance_max;
      }
      //estimate the posterior for observation model
      /* observation model is modeled as a 1D normal distribution with
       *  x: distance between landmark and current position
       *  mean: mesurement (distance to closest landmark)
       *  std: obseravtion_std (1.0f) */

//      posterior_obs *= helpers.normpdf(observations.distance_f[z],
//                                       pseudo_range_min,
//                                       observations_std_);

      posterior_obs *= helpers.normpdf(pseudo_range_min,
                                       observations.distance_f[z],
                                       observations_std_);

//      cout << "i=" << i << " z=" << z << " observation=" << observations.distance_f[z] << " pseudo_range_min=" << pseudo_range_min << "\t";
//      cout << "posterior_obs=" << posterior_obs << endl;
    }

    /**************************************************************************
     *  finalize bayesian localization filter:
     *************************************************************************/
    //update belief
    bel_x[i] = posterior_motion * posterior_obs;

  };

  //normalize:
  bel_x = helpers.normalize_vector(bel_x);

  cout << "bel_x: " << endl;
  for (int i=0; i<bel_x.size(); ++i) {
    cout << bel_x[i] << "\t";
  }
  cout << endl;

  //set bel_x to bel_init:
  bel_x_init_ = bel_x;
};
