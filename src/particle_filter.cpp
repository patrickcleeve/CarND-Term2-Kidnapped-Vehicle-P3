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

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	// Reference: Project Q&A Video, Project Slack

	num_particles = 100;
	default_random_engine gen;

	// Initialise x, y, theta distributions
	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(theta, std[2]);


	// Generate each particle, with id, weight, x, y, theta
	for (int i=0; i < num_particles; i++) {

		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;

		//cout << "Particle: " << particle.id << " || X:" << particle.x << " | Y: " << particle.y << " | T: " << particle.theta << endl;

		// Add generated particle and weights to vectors
		particles.push_back(particle);
		weights.push_back(particle.weight);


	}

	// Set initialised status
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	//  Add measurements to each particle and add random Gaussian noise.
	
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	// Reference: Project Q&A Video, Project Slack

	default_random_engine gen;

	// Predict motion of each particle with noise
	for (int i=0; i < num_particles; i++) {
		
		double new_x;
		double new_y;
		double new_theta;

		// Predict new position and theta, based on yaw_rate
		if (yaw_rate == 0) {

			new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			new_theta = particles[i].theta;

		} else {

			new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;

		}

		// Assign updated particle position and theta with Gaussian noise.
		normal_distribution<double> N_x(new_x, std_pos[0]);
		normal_distribution<double> N_y(new_y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[2]);

		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//   Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//
	// Function Unused: data association done inside updateWeights()
	//

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	
	//  Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	// Reference: Project Q&A Video, Project Slack


	// Update weights of each particle, based on probability of observations
	// Each observation needs to be transformed from vehicle to map coordinates
	// Transformed observations need to be associated with landmarks in map coordinates
	// Update weights based on probability of observations associated with landmarks


	for (int i = 0; i < num_particles; i++) {
		
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;

		vector<LandmarkObs> trans_observations;
		LandmarkObs obs;

		// Transform observations from vehicle coordinates to map coordinates
		for (int j = 0; j < observations.size(); j++) {

			LandmarkObs trans_obs;
			obs = observations[j];

			// Transform x, y from vehicle coordinates to map coordinates using homogenous transformation
			// xm = xp + cos(theta) * xc - sin(theta) * yc
			// ym = yp + sin(theta) * xc + cos(theta) * yc
			trans_obs.x = particles[i].x + cos(particles[i].theta) * obs.x - sin(particles[i].theta) * obs.y;
			trans_obs.y = particles[i].y + sin(particles[i].theta) * obs.x + cos(particles[i].theta) * obs.y;
			trans_observations.push_back(trans_obs);

		}

		particles[i].weight = 1.0;

		for (int k = 0; k < trans_observations.size(); k++) {

			// For each transformed observation need to set closest landmark association

			// For each landmark, loop through, check distance
			// assoicate closest observation dist(), with landmark

			// NB: Function is really slow, have to loop through each landmark for each observation.
			// N(mn) m=n_observations, n=n_landmarks
			
			int association;

			// Extract transformed observation x and y values
			double meas_x = trans_observations[k].x;
			double meas_y = trans_observations[k].y;

			// Init closest dist to range of the sensor (50)
			double closest_dist = sensor_range;
			

			for (int m=0; m < map_landmarks.landmark_list.size(); m++) {

				// Extract landmark x and y values
				float landmark_x = map_landmarks.landmark_list[m].x_f;
				float landmark_y = map_landmarks.landmark_list[m].y_f;
				

				// Calculate euclidean distance between observation and landmark
				double calc_dist = dist(meas_x, meas_y, landmark_x, landmark_y);

				// If observation is closer to landmark, update distance and add association
				if (calc_dist < closest_dist) {

					closest_dist = calc_dist;
					association = m;

				}

				//cout << "Closest Dist: " << closest_dist << endl;
				//cout << calc_dist << closest_dist << endl;

			}
			
			// If particle has an associated landmark, update particle weight based on probability
			if (association != 0) {

				double meas_x = trans_observations[k].x;
				double meas_y = trans_observations[k].y;

				double mu_x = map_landmarks.landmark_list[association].x_f;
				double mu_y = map_landmarks.landmark_list[association].y_f;

				long double multipler = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]) * exp(- (pow(meas_x - mu_x, 2) / (2 * std_landmark[0] * std_landmark[0]) + pow(meas_y - mu_y, 2) / (2 * std_landmark[1] * std_landmark[1])));
				

				if (multipler > 0) {
					
					// Update particle weight
					particles[i].weight *= multipler;

				}  

				// From Project Slack Channel:
				// If particle weight is very small (approx zero), reset to prevent weights going to zero and disappearing.
				if (particles[i].weight < 0.0001) {
					
					particles[i].weight = 0.0001;

				}

			}

			associations.push_back(association+1);
			sense_x.push_back(trans_observations[k].x);
			sense_y.push_back(trans_observations[k].y);


		}

		// Update weights vector (used only for resampling)
		weights[i] = particles[i].weight;

		// Update Particle associations and sense
		particles[i].associations = associations;
		particles[i].sense_x = sense_x;
		particles[i].sense_y = sense_y;

	}



}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// Reference: Project Q&A Video, Project Slack

	// Use C++ discrete distribution built in to resample based on weights
	
	default_random_engine gen;
	
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;

	for (int i = 0; i < num_particles; i++) {

		resample_particles.push_back(particles[distribution(gen)]);
	
	}

	particles = resample_particles;
	

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
