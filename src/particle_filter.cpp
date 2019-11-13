/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  bool verbose = false;

  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   */

  // TODO: Set the number of particles
  num_particles = 100;
  particles.resize(num_particles);
  weights.resize(num_particles);

  // Create default random generator
  std::default_random_engine gen;

  // Create normal distributions for x, y and theta
  const double std_x     = std[0];
  const double std_y     = std[1];
  const double std_theta = std[2];
  std::normal_distribution<double> dist_x(x,std_x);
  std::normal_distribution<double> dist_y(y,std_y);
  std::normal_distribution<double> dist_theta(theta,std_theta);

  /*
   * TODO: Add random Gaussian noise to each particle, and all weights to 1
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */

  for(int i = 0; i < num_particles; ++i){
    particles[i].id     = i;
    particles[i].x      = dist_x(gen); // Add noise to x with standard deviation
    particles[i].y      = dist_y(gen); // Add noise to y with standard deviation
    particles[i].theta  = dist_theta(gen); // Add noise to theta with standard deviation
    particles[i].weight = 1.0;

    weights[i] = 1.0;

    if(verbose) {
      // Print samples to the terminal.
      std::cout << "Sample " << i + 1 << " " << particles[i].x << " " << particles[i].y << " "
                << particles[i].theta << std::endl;
    }
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Create normal distributions for x, y, theta
  const double std_x     = std_pos[0];
  const double std_y     = std_pos[1];
  const double std_theta = std_pos[2];
  
  // Create default random generator
  std::default_random_engine gen;
  
  std::normal_distribution<double> dist_x(0,std_x);
  std::normal_distribution<double> dist_y(0,std_y);
  std::normal_distribution<double> dist_theta(0,std_theta);

  double x_0, y_0, theta_0;
  for(int i = 0; i < num_particles; ++i) {
      // Initial particle position
      x_0     = particles[i].x;
      y_0     = particles[i].y;
      theta_0 = particles[i].theta;

      // Avoid division by zero
      if(fabs(yaw_rate) > 0.001) {
          particles[i].x     = x_0 + velocity/yaw_rate * (sin(theta_0+yaw_rate*delta_t) - sin(theta_0));
          particles[i].y     = y_0 + velocity/yaw_rate * (cos(theta_0) - cos(theta_0+yaw_rate*delta_t));
          particles[i].theta = theta_0 + yaw_rate * delta_t;
      } else {
          // Predict next particle location by assuming constant yaw rate when the yaw rate is close to zero
          particles[i].x     = x_0 + velocity * delta_t * cos(theta_0);
          particles[i].y     = y_0 + velocity * delta_t * sin(theta_0);
      }

      // Add random gaussian noise with zero mean to particle measurements
      particles[i].x     += dist_x(gen);
      particles[i].y     += dist_y(gen);
      particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Landmark Uncertainities
  const double sig_x = std_landmark[0];
  const double sig_y = std_landmark[1];
  
  // Particle weight normalization term
  const double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
  
  // Particle weight multi-variate Gaussian exponent
  const double x_denom = 2 * pow(sig_x, 2);
  const double y_denom = 2 * pow(sig_y, 2);
  double exponent;
  
  const vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
  
  double landmark_particle_dist;
  vector<double> obs_landmark_dist (landmarks.size());
  
  int min_idx;
  double nearest_landmark_x, nearest_landmark_y;
  
  double x_p, y_p, theta_p, weight;
  double x_c, y_c, x_m, y_m;
  for(int i=0; i < num_particles; ++i) {
    // Initialize particle weight
    x_p     = particles[i].x;
    y_p     = particles[i].y;
    theta_p = particles[i].theta;
    weight  = 1.0;

    for(int j=0; j < observations.size(); ++j) {
      // Transform observed position in car coordinate to map coordinate
      x_c = observations[j].x;
      y_c = observations[j].y;
      x_m = cos(theta_p)*x_c - sin(theta_p)*y_c + x_p;
      y_m = sin(theta_p)*x_c + cos(theta_p)*y_c + y_p;

      // Find the nearest landmarks
      for(int k=0; k < landmarks.size(); ++k){
          // Calculate distance between particle and landmark
          landmark_particle_dist = dist(x_p, y_p, landmarks[k].x_f, landmarks[k].y_f);

          // Calculate distance between observations and landmark only when it is
          // within the sensor range
          if(landmark_particle_dist <= sensor_range){
            obs_landmark_dist[k] = dist(x_m, y_m, landmarks[k].x_f, landmarks[k].y_f);
          } else {
            obs_landmark_dist[k] = 999.0;
          }
      }
      
      // Find the nearest landmark
      min_idx = distance(obs_landmark_dist.begin(),min_element(obs_landmark_dist.begin(),obs_landmark_dist.end()));
      nearest_landmark_x = landmarks[min_idx].x_f;
      nearest_landmark_y = landmarks[min_idx].y_f;
      
      exponent = (pow(x_m - nearest_landmark_x, 2) / x_denom)
                   + (pow(y_m - nearest_landmark_y, 2) / y_denom);
      // Calculate weight using normalization terms and exponent
      weight *= gauss_norm * exp(-exponent);
    }
    
    // Update particle weights with combined multi-variate Gaussian distribution
    particles[i].weight = weight;
    weights[i] = weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> new_particles (num_particles);

  std::random_device rd;
  std::default_random_engine gen(rd());
  // Define discrete distribution of the weighted particles
  std::discrete_distribution<int> index(weights.begin(), weights.end());
  
  // Randomly sample the particle based on the weight discrete distribution
  for (int i = 0; i < num_particles; ++i) {
    new_particles[i] = particles[index(gen)];
  }

  // Replace old particles with the new resampled particles
  particles = new_particles;
  
  /*
   * Alternative method: Resample wheel method
  */
//  vector<Particle> new_particles (num_particles);
//
//  int index = rand() % num_particles;
//  double w_max = *max_element(weights.begin(), weights.end());
//
//  double beta = 0.0;
//  for(int i=0; i < num_particles; ++i){
//      beta += (rand() / (RAND_MAX + 1.0)) * (2*w_max);
//      while (beta > weights[index]){
//          beta -= weights[index];
//          index = (index+1) % num_particles;
//      }
//      new_particles[i] = particles[index];
//  }
//  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
