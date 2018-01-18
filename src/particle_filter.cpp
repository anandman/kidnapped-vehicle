/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

#define NUM_PARTICLES 100
#define INITIAL_WEIGHT 1.0
#define MIN_WEIGHT 0.000001
#define ZERO_YAW 0.001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  if (is_initialized) {
    return;
  } else {
    // set number of particles we will use
    num_particles = NUM_PARTICLES;
    particles.reserve(num_particles);
    weights.reserve(num_particles);

    // engine used for random Gaussian distributions
    random_device rd;
    default_random_engine rgen(rd());
    // initialize particles with standard Gaussian distribution around GPS position data
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // generate initial state of particles
    for (int i = 0; i < num_particles; i++) {
      Particle p = {};
      p.id = i;
      p.x = dist_x(rgen);
      p.y = dist_y(rgen);
      p.theta = dist_theta(rgen);
      p.weight = INITIAL_WEIGHT;
      particles.push_back(p);
      weights[i] = INITIAL_WEIGHT;
    }

    is_initialized = true;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // engine used for random Gaussian distributions
  random_device rd;
  default_random_engine rgen(rd());
  // standard Gaussian distribution around 0
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (auto &p: particles) {
    if (abs(yaw_rate) <= ZERO_YAW) {
      // predict location if moving straight
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    } else {
      // predict position if turning
      double new_theta = p.theta + yaw_rate * delta_t;
      p.x += (velocity / yaw_rate) * (sin(new_theta) - sin(p.theta));
      p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(new_theta));
      p.theta = new_theta;
    }

    // add Gaussian noise to particle
    p.x += dist_x(rgen);
    p.y += dist_y(rgen);
    p.theta += dist_theta(rgen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // iterate through observations
  for (auto &obs : observations) {
    double min_distance = numeric_limits<double>::max();
    // iterate through predicted landmarks
    for (int i=0; i < predicted.size(); i++) {
      // calculate the distance of predicted measurement and observation measurement
      double distance = dist(obs.x, obs.y, predicted[i].x, predicted[i].y);
      // find the nearest landmark and assign the id
      if(distance < min_distance){
        min_distance = distance;
        //obs.id = predicted[i].id;
        obs.id = i;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // calculate normalization term
  const double gauss_norm = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
  const double std_x_2 = 2.0 * std_landmark[0] * std_landmark[0];
  const double std_y_2 = 2.0 * std_landmark[1] * std_landmark[1];

  // iterate through particles
  for (auto &p : particles) {
    // transform observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> observations_in_map;
    for (const auto &obs : observations) {
      LandmarkObs mobs = {};
      mobs.id = obs.id;
      mobs.x = p.x + cos(p.theta) * obs.x - sin(p.theta) * obs.y;
      mobs.y = p.y + sin(p.theta) * obs.x + cos(p.theta) * obs.y;
      observations_in_map.push_back(mobs);
    }

    // find all landmarks within sensor range of particles
    vector<LandmarkObs> landmark_predictions;
    for (const auto &l : map_landmarks.landmark_list) {
      // if landmark is within sensor range of particle, calculate distance between observation and landmark.
      double p_to_l_dist = dist(p.x, p.y, l.x_f, l.y_f);
      if (p_to_l_dist <= sensor_range) {
        LandmarkObs obs = {};
        obs.id = l.id_i;
        obs.x = l.x_f;
        obs.y = l.y_f;
        landmark_predictions.push_back(obs);
      }
    }

    // associate the observation to its nearest neighbor landmark
    dataAssociation(landmark_predictions, observations_in_map);
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // calculate weights
    double gaussDist = 1.0;

    for (auto &mobs: observations_in_map) {
      // get associated landmark
      //Map::single_landmark_s mu = map_landmarks.landmark_list[mobs.id - 1];
      LandmarkObs mu = landmark_predictions[mobs.id];

      // calculate multi-variate Gaussian distribution
      double x_diff = mobs.x - mu.x;
      double y_diff = mobs.y - mu.y;
      double exponent = ((x_diff * x_diff) / std_x_2) +
                        ((y_diff * y_diff) / std_y_2);
      // multiply all the calculated measurement probabilities
      gaussDist *= gauss_norm * exp(-exponent);
      // record particle's associations
      associations.push_back(mu.id);
      sense_x.push_back(mu.x);
      sense_y.push_back(mu.y);
    }

    // update particle weights with multi-variate Gaussian distribution
    if (gaussDist == 0.0) {
      p.weight = MIN_WEIGHT;
    } else {
      p.weight = gaussDist;
    }
    weights.push_back(p.weight);
    // update particle's associations
    SetAssociations(p, associations, sense_x, sense_y);
  }
}

void ParticleFilter::resample() {
  vector<Particle> resampled_particles;
  resampled_particles.reserve(num_particles);

  // engine used for random Gaussian distributions
  random_device rd;
  default_random_engine rgen(rd());
  discrete_distribution<int> index(weights.begin(), weights.end());

  // use discrete distribution to return particles by weight
  weights.clear();
  for (auto &p: particles) {
    const int id = index(rgen);
    Particle resampled_p = particles[id];
    resampled_particles.push_back(resampled_p);
    //weights.push_back(resampled_p.weight);
  }

  // replace old particles with the resampled particles
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
