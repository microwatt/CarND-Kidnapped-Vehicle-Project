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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	

	if (!is_initialized) {

		default_random_engine gen;
		num_particles = 1;
		
		

		// Create the Gaussian distributions for x, y, theta
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);

		for (int i = 0; i < num_particles; ++i) {

			Particle init_particle;
			init_particle.id = i;
		//	init_particle.x = dist_x(gen);
		//	init_particle.y = dist_y(gen);
		//	init_particle.theta = dist_theta(gen);
			init_particle.weight = 1.0;
			
			init_particle.x = x;
			init_particle.y = y;
			init_particle.theta = theta;

			// add init_particle to the vector class
			particles.push_back(init_particle);

		}
	}
	else
		is_initialized = true;

	//cout << " Finished INIT" << endl;
	//cout << "Weights size" << weights.size() << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;


	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);



	for (int i = 0; i < num_particles; i++) {

		// Calculate updated positions(x, y, theta) based on yaw rate

		if (yaw_rate < 0.00001) { 
			particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
			
		}
		else {
			particles[i].x = particles[i].x + (velocity / yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y = particles[i].y + (velocity / yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			particles[i].theta = particles[i].theta + yaw_rate*delta_t;
		}


		//particles[i].x += dist_x(gen);
		//particles[i].y += dist_y(gen);
		//particles[i].theta += dist_theta(gen);

		cout << "** PREDICTION (x,y,theta): (" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ")" << endl;
	}

	//cout << "Finished Prediction" << endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	/*
	
	A) For each particle
		1) Transform particle'sobservations into map coordinates 

			std::vecotr >LandmarkObs> obs_transformation
			for (landmarksObs observ: observations)
				LandmarkObs obst;
				obs_t.id = ;
				ob_transform.push<obs_t?
			
				for each transformed observations calculate distinace to all landmarks

				for map::single_landmarks landm: map_landkmarks_.list{
					dist(landm.x, landm.y, obs_t.ots_t.
					vector distance.push
				}

				find min_elemant and associate the id. (use min_element class)

				After finding this, calculate proability the particular particles observation saw this particular landmark.

				particle's total weight is product of probabilities of each observations.

	*/

	return;

	double gauss_norm = (1 / (2 * M_PI*std_landmark[0] * std_landmark[1]));
	double var_x = 2 * std_landmark[0] * std_landmark[0];
	double var_y = 2 * std_landmark[1] * std_landmark[1];

//	cout << "STD_LANDMARK (x,y): " << std_landmark[0] << ", " << std_landmark[1] << endl;


	for (int i = 0; i < particles.size(); i++) {
		
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;

		particles[i].weight = 1.0;

		// Transform each particle's observations into MAP coordinates
		LandmarkObs landmark_t;
		for (int j = 0; j < observations.size(); j++) {
			
			landmark_t = observations[j];
			LandmarkObs obs_t;
			
			obs_t.id = landmark_t.id;
			obs_t.x = particles[i].x + (cos(particles[i].theta) * landmark_t.x) - (sin(particles[i].theta) * landmark_t.y);
			obs_t.y = particles[i].y + (sin(particles[i].theta) * landmark_t.x) + (cos(particles[i].theta) * landmark_t.y);

			//  DEBUG
			//cout << "******  *********************" << endl;
	//		cout << "Observation(x,y): (" << landmark_t.x << ", " << landmark_t.y << ")";
	//		cout << "Transformed(x,y): (" << obs_t.x << ", " << obs_t.y << ")" << endl;


			double min_distance = sensor_range;
			int min_distance_idx = -1;

			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
				
				double dists = dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, obs_t.x, obs_t.y);

				if (dists < min_distance) {
					min_distance = dists;
					min_distance_idx = k;
				}
			} // for (k-loop)

			if (min_distance_idx != -1) {
				
		//		cout << "Min Distance IDX: " << min_distance_idx;
		//		cout << " Landmark X: " << map_landmarks.landmark_list[min_distance_idx].x_f;
		//		cout << " Landmark Y: " << map_landmarks.landmark_list[min_distance_idx].y_f << endl;

				double mu_x = map_landmarks.landmark_list[min_distance_idx].x_f;
				double mu_y = map_landmarks.landmark_list[min_distance_idx].y_f;

//				cout << "MU_X: " << mu_x << " MU_Y: " << mu_y << endl;
				
				double exponent_term = (obs_t.x - mu_x)*(obs_t.x - mu_x) / var_x + (obs_t.y - mu_y)*(obs_t.y - mu_y) / var_y;

				double particle_wgt = gauss_norm*exp(-exponent_term);
				
	//			cout << "Particle wgt: " << particle_wgt;

				particles[i].weight *= particle_wgt;

	//			cout << " Particle Wgt Prob (MULT): " << particles[i].weight << endl;

			}
			else
				particles[i].weight = 0;

			associations.push_back(min_distance_idx + 1);
			sense_x.push_back(obs_t.x);
			sense_y.push_back(obs_t.y);

		} // for (j-loop - observations)

		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);

		//cout << "Final Particle WGT: " << i << "  ***: " << particles[i].weight << endl;
	} // for (i-loop - particles)

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	return;

	default_random_engine gen;
	vector<double> weights;

//	cout << "***** Resample function" << endl;

	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);

		//cout << "****: " << particles[i].weight << endl;
	}

	discrete_distribution<int> distribution(weights.begin(), weights.end());

	std::vector<Particle> resample_particles;

	for (int i = 0; i < num_particles; i++) {
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
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
