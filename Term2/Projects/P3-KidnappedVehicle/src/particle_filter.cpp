/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  //set number of particles
  num_particles = 20;

  //creates normal distributions for x, y and theta
  std::normal_distribution<double> norm_dist_x(0, std[0]);
  std::normal_distribution<double> norm_dist_y(0, std[1]);
  std::normal_distribution<double> norm_dist_theta(0, std[2]);

  //random number engine class that generates pseudo-random numbers
  std::default_random_engine gen;

  //initializes particles based on first position (plus random Gaussian noise)
  for (int i=0; i<num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0; //initial weigh is set to 1.0

    //adds random Gaussian noise
    p.x += norm_dist_x(gen);
    p.y += norm_dist_y(gen);
    p.theta += norm_dist_theta(gen);

    particles.push_back(p);

  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  //creates normal distributions for Gaussian noise
  std::normal_distribution<double> noise_x(0, std_pos[0]);
  std::normal_distribution<double> noise_y(0, std_pos[1]);
  std::normal_distribution<double> noise_theta(0, std_pos[2]);

  //random number engine class that generates pseudo-random numbers
  std::default_random_engine gen;

  //estimate new position for each particle (
  for (Particle &particle : particles) {

    if (fabs(yaw_rate) < 0.00001) {
      //process model: constant yaw_rate
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);

    } else {
      //process model: bicycle motion mode
      particle.x += velocity/yaw_rate * (sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta));
      particle.y += velocity/yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t));
      particle.theta += yaw_rate * delta_t;

    }

    //adds random Gaussian noise
    particle.x += noise_x(gen);
    particle.y += noise_y(gen);
    particle.theta += noise_theta(gen);

  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


  //for each observation...
  for (LandmarkObs &obs : observations) {

    double min_distance = 1000000.0;

    //for each landmark...
    for (LandmarkObs landmark : predicted) {

      //calculate distance between them
      double distance = dist(obs.x, obs.y, landmark.x, landmark.y);

      if (distance < min_distance) {
        //associate closest landmark with current observation
        obs.id = landmark.id;
        min_distance = distance;

      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


  //for each particle..
  for (Particle &particle : particles) {

    //landmarks within sensor's range
    std::vector<LandmarkObs> predictions;

    //observations in MAP's coordinate system
    std::vector<LandmarkObs> transformation_obs;

    //variables needed for setAssociation
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    //only consider landmarks within sensor range
    for (Map::single_landmark_s map_landmark : map_landmarks.landmark_list) {
      double distance_to_landmark = dist(map_landmark.x_f, map_landmark.y_f, particle.x, particle.y);
      if (distance_to_landmark < sensor_range) {
        LandmarkObs landmark;
        landmark.id = map_landmark.id_i;
        landmark.x = map_landmark.x_f;
        landmark.y = map_landmark.y_f;
        predictions.push_back(landmark);
      }
    }

    //transforms observations from vehicle's to map's coordinate systems
    for (LandmarkObs observation : observations) {

      //observation in MAP's coordinate system
      LandmarkObs observation_map;

      observation_map.id = observation.id;

      //rotation of the map's frame according to the particle's theta angle
      observation_map.x = observation.x*cos(particle.theta) - observation.y*sin(particle.theta);
      observation_map.y = observation.x*sin(particle.theta) + observation.y*cos(particle.theta);

      //translation
      observation_map.x += particle.x;
      observation_map.y += particle.y;

      transformation_obs.push_back(observation_map);
    }

    //find closest landmark to each observation
    //id of transformation_obs is set to id of closest landmark inside predictions
    dataAssociation(predictions, transformation_obs);

    //calculate particle's final weight
    particle.weight = 1.0; //reinit

    //landmark measurement uncertainty std_x [m], std_y [m]
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];

    //for each observation...
    for (LandmarkObs observation : transformation_obs) {

      int closest_landmark_index;

      //find closest landmark
      for (unsigned int i=0; i<predictions.size(); ++i) {
        if (predictions[i].id == observation.id) closest_landmark_index = i;
      }

      //matching landmark in the map
      LandmarkObs association = predictions[closest_landmark_index];

      //push back association info to display it in the simulator
      associations.push_back(association.id);
      sense_x.push_back(observation.x);
      sense_y.push_back(observation.y);

      //calculate multivariate Gaussian probability
      double m1 = 1 / (2 * M_PI * std_x * std_y);
      double e1 = (pow((association.x - observation.x), 2)) / (2 * pow(std_x, 2));
      double e2 = (pow((association.y - observation.y), 2)) / (2 * pow(std_y, 2));

      //particle's final weight is a combination of all probabilities
      particle.weight *= m1 * (exp(-(e1 + e2)));

    }

    SetAssociations(particle, associations, sense_x, sense_y);

  } //end particles loop
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  //final vector with resampled particles
  std::vector<Particle> resampled_particles;

  std::vector<double> weights;
  //get weights from current particles
  for (Particle p : particles) {
    weights.push_back(p.weight);
  }

  double weight_max = *max_element(weights.begin(), weights.end());

  //initial index is initialized uniformly
  std::default_random_engine gen;
  std::uniform_int_distribution<int> index_distribution(0, num_particles-1);
  std::uniform_real_distribution<double> beta_distribution(0.0, 2.0*weight_max);

  int index = index_distribution(gen);
  double beta = 0.0;

  //resampling wheel
  for (int i=0; i<num_particles; ++i) {
    beta += beta_distribution(gen);

    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    particles[index].id = i;
    resampled_particles.push_back(particles[index]);
  }

  particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();


    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
