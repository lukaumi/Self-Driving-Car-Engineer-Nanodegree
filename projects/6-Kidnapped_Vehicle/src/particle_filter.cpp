/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *         Luka Umiljanovic
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

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   *  Set the number of particles.
   *  Initialize all particles to first position
   *  (based on estimates of x, y, theta and their uncertainties from GPS)
   *  and all weights to 1.
   *  Add random Gaussian noise to each particle.
   */

  // generate pseudo-random numbers
  std::default_random_engine gen;

  // create a normal (Gaussian) distribution for x, y and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // set the number of particles
  num_particles = 100;

  // initialize particles
  for (int i = 0; i < num_particles; ++i)
  {
    Particle particle;

    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
    weights.push_back(particle.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   *  Add measurements to each particle and add random Gaussian noise.
   *  NOTE: When adding noise you may find
   *  std::normal_distribution and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // generate pseudo-random numbers
  std::default_random_engine gen;

  // predict final x, y and theta after dt for each particle
  for (int i = 0; i < num_particles; ++i)
  {
    double x_0 = particles[i].x;         // initial x position
    double y_0 = particles[i].y;         // initial y position
    double theta_0 = particles[i].theta; // initial yaw

    double x_f;     // final x position
    double y_f;     // final y position
    double theta_f; // final yaw

    // equations when the yaw rate is equal to zero
    // consider very small values which approximate to zero
    if (abs(yaw_rate) < 0.0001)
    {
      x_f = x_0 + velocity * delta_t * cos(theta_0);
      y_f = y_0 + velocity * delta_t * sin(theta_0);
      theta_f = theta_0;
    }
    // equations when the yaw rate is not zero
    else
    {
      x_f = x_0 + velocity / yaw_rate * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
      y_f = y_0 + velocity / yaw_rate * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
      theta_f = theta_0 + yaw_rate * delta_t;
    }

    // create a normal (Gaussian) distribution for x, y and theta
    std::normal_distribution<double> dist_x(x_f, std_pos[0]);
    std::normal_distribution<double> dist_y(y_f, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_f, std_pos[2]);

    // generate final position and angle given the Gaussian distribution (noise)
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   *  Update the weights of each particle using a mult-variate Gaussian distribution.
   *  You can read more about this distribution here:
   *  https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   *  NOTE: The observations are given in the VEHICLE'S coordinate system.
   *  Your particles are located according to the MAP'S coordinate system.
   *  You will need to transform between the two systems.
   *  Keep in mind that this transformation requires both rotation AND translation (but no scaling).
   *  The following is a good resource for the theory:
   *  https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *  and the following is a good resource for the actual equation to implement
   *  (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // iterate over particles
  for (int i = 0; i < num_particles; ++i)
  {
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    // initialize particles' weight to 1
    double particle_weight = 1.0;

    // iterate over observations
    for (unsigned int j = 0; j < observations.size(); ++j)
    {
      // initialize shortest distance between observation and landmark with highest value of type double
      double shortest_distance = std::numeric_limits<double>::max();

      // transform observations from the vehicle frame of reference to the map frame of reference
      double x_map = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      double y_map = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);

      // keep landmark association for this observation
      int association = -1;

      // find the predicted measurement that is closest to each observed measurement
      // and assign the observed measurement to this particular landmark
      // i.e. find the nearest landmark to the current observation using nearest neighbor algorithm
      for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k)
      {
        // distance between current observation iteration and landmark iteration
        double current_distance = dist(map_landmarks.landmark_list[k].x_f,
                                       map_landmarks.landmark_list[k].y_f,
                                       x_map,
                                       y_map);

        if (current_distance < shortest_distance)
        {
          shortest_distance = current_distance;
          association = k;
        }
      } // landmarks

      // compute particles' weight using Multivariate-Gaussian
      particle_weight *= multiv_prob(std_landmark[0],
                                     std_landmark[1],
                                     x_map,
                                     y_map,
                                     map_landmarks.landmark_list[association].x_f,
                                     map_landmarks.landmark_list[association].y_f);

      // save association and association's (x,y) world coordinates mapping
      associations.push_back(association + 1);
      sense_x.push_back(x_map);
      sense_y.push_back(y_map);
    } // observations

    // set associations (blue lines in simulator)
    SetAssociations(particles[i], associations, sense_x, sense_y);

    // update particles' weights with combined Multivariate-Gaussian distribution
    particles[i].weight = particle_weight;
    weights[i] = particles[i].weight;
  } // particles
}

void ParticleFilter::resample()
{
  /**
   *  Resample particles with replacement with probability proportional to their weight.
   *  Uses Resampling Wheel algorithm.
   *  NOTE: You may find std::discrete_distribution helpful here.
   *  http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // resampled particles to replace set of current particles after resampling
  std::vector<Particle> resampled_particles;

  // generate pseudo-random numbers
  std::default_random_engine gen;

  // generate random starting particle index uniformly from the set of all indices
  std::uniform_int_distribution<int> uni_int_dist(0, num_particles - 1);
  int index = uni_int_dist(gen);

  // get largest weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniformly draw continuous value in range [0.0, 2 * max_weight)
  std::uniform_real_distribution<double> uni_real_dist(0.0, 2.0 * max_weight);

  // construct the function beta and initialize to 0
  double beta = 0.0;

  // Resampling Wheel
  for (int i = 0; i < num_particles; i++)
  {
    beta += uni_real_dist(gen);
    while (beta > weights[index])
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }

  // replace set of current particles
  particles = std::move(resampled_particles);
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const std::vector<int> &associations,
                                     const std::vector<double> &sense_x,
                                     const std::vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association, and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

std::string ParticleFilter::getAssociations(Particle best)
{
  std::vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

std::string ParticleFilter::getSenseCoord(Particle best, std::string coord)
{
  std::vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
