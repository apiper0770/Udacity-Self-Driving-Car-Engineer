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
        //Random Number Engine Generator
        if(!is_initialized){
            std::default_random_engine gen;
            //declare number of particles
            num_particles = 10;
   
            //Create a particle to update with initialization data
            Particle part;

            //resize particles
            particles.resize(num_particles);

            //resize weights
            weights.resize(num_particles);
         
            //Create a normal (Gaussian) distribution for x.
            std::normal_distribution<double> dist_x(x, std[0]);
            //Create normal distribution for y.
            std::normal_distribution<double> dist_y(y, std[1]);
            //Create normal distribution for theta.
            std::normal_distribution<double> angle_theta(theta, std[2]);

            for (int i = 0; i < num_particles; i++){
            
                //Set particle weight and id
                part.id = i;
                part.weight = 1.0;            
	    
	        //Create a normal (Gaussian) distribution for x.
	       // std::normal_distribution<double> dist_x(x, std[0]);
	        //Create normal distribution for y.
	       // std::normal_distribution<double> dist_y(y, std[1]);
                //Create normal distribution for theta.
	        //std::normal_distribution<double> angle_theta(theta, std[2]);
		
	        // Sample  and from these normal distrubtions like this: 
	        // sample_x = dist_x(gen);
	       // where "gen" is the random engine initialized earlier.
                part.x = dist_x(gen);
                part.y = dist_y(gen);
                part.theta = angle_theta(gen);
                //put particle in particles matrix
                particles[i] = part;
	        
                // Print your samples to the terminal.
	        //cout << "Particle: " << i << " " << part.x << " " << part.y << " " << part.theta << endl;
	    }
            is_initialized = true;
        }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
        std::default_random_engine gen2;
        //initialize parameters
        double future_x, future_y, future_theta;
        
        //generate noise
        //Udacity Forum Post told me to leave normal_distribution function and default_random_engine outside FOR loops to eliminate bugs in the program
        //That is why I have placed this distribution here now, instead of inside the for loop.
        std::normal_distribution<double> pred_x(0.0, std_pos[0]);
        std::normal_distribution<double> pred_y(0.0, std_pos[1]);
        std::normal_distribution<double> pred_theta(0.0, std_pos[2]);
        for (int i = 0; i < num_particles; i++){
            Particle& part = particles[i];    
            if(fabs(yaw_rate) < 0.00001){
                future_x = part.x + (velocity*cos(part.theta)*delta_t);
                future_y = part.y + (velocity*sin(part.theta)*delta_t);
                future_theta = part.theta;
            }      
            //add measurement prediction for x, y, & theta
            //if yaw_rate greater than zero
            else{
                future_x = part.x + (velocity/yaw_rate)*(sin(part.theta + yaw_rate*delta_t) - sin(part.theta));
                future_y = part.y + (velocity/yaw_rate)*(cos(part.theta) - cos(part.theta + yaw_rate*delta_t));
                future_theta = part.theta + yaw_rate * delta_t;
            } 
            //generate noise
            //std::normal_distribution<double> pred_x(future_x, std_pos[0]);
            //std::normal_distribution<double> pred_y(future_y, std_pos[1]);
            //std::normal_distribution<double> pred_theta(future_theta, std_pos[2]);
            // Sample  and from these normal distrubtions like this:
            // sample_x = dist_x(gen);
            // where "gen" is the random engine initialized earlier.
            part.x = future_x + pred_x(gen2);
            part.y = future_y + pred_y(gen2);
            part.theta = future_theta +  pred_theta(gen2);

            // Print your samples to the terminal.
            //cout << "Prediction: "<< part.x << " " << part.y << " " << part.theta << endl; 
        }
}

//VectorXd ParticleFilter::HTransform(LandmarkObs LM){
        //The purpose of this method is to convert observed particle coordinates into map coordinates 
    
        //Create Vectors and Matrix for transformation
       // MatrixXd Transform = Matrix::Zero(3,3);
       // VectorXd obs_coord = VectorXd(3);
        //VectorXd map_coord = Vector(3);

        //angle of rotation
        //double angle = particle[i].theta;

        //Populate Homogenous Transform Matrix 
        //Transform[0,0] = cos(angle);
        //Transform[0,1] = -sin(angle);
        //Car x location
        //Transform[0,2] = particle[i].x;
        //Transform[1,0] = -Transform[0,1];
        //Transform[1,1] = Transform[0,0];
        //Car y location
        //Transform[1,2] = particle[i].y;
        //Transform[2,2] = 1;
        //Populate observation position vector
        //obs_coord[0] = LM.x;
        //obs_coord[1] = LM.y;
        //obs_coord[2] = 1;

        //Perform Homogenous Transform
        //map_coord = Transform * obs_coord;
        //return map_coord;


//}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
        /*
        int num_obs = len(predicted);
        int num_pred = len(observations);
        double goal;
        int marker;
        VectorXd obs_trans = vectorXd(3);
        for (i = 0; i < num_obs; i++){

            //Change observations coords into map coords
            obs_trans = HTransform(observations[0]);
            //check prediction against first observation and set the goal to beat
            goal = dist(obs_trans[0], obs_trans[1], predicted[i].x, predicted[i].y);
            marker = 0;
            for(j = 1; j < num_pred; j++){
                //check prediction against remaining observations and update the goal to beat if the euclidean distance is now closer than previous
                obs_trans = HTransform(observations[j]);
                goal_temp = dist(obs_trans[0], obs_trans[1], predicted[i].x, predicted[i].y);
                if(goal_temp < goal):
                    goal = goal_temp;
                    marker = j;
            }
            //set observation id to measurement to nearest neighbor landmark
            observations[i].id = marker;
        }*/
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
        int num_obs = observations.size();
        int num_lm = map_landmarks.landmark_list.size();
        double goal;
        double goal_temp = 0.0;
        int marker;
        double x_diff = 0.0;
        double y_diff = 0.0;
        double x_sqr_diff = 0.0;
        double y_sqr_diff = 0.0;
        double x_sqr_landm = std_landmark[0]*std_landmark[0];
        double y_sqr_landm = std_landmark[1]*std_landmark[1];
        const double gauss_norm = (1.0/(2.0*M_PI*std_landmark[0]*std_landmark[1]));
        double expo_term = 0.0;
        double exponential = 0.0;
        double update_weight;
        int index = 0;
        double test_dist = 0.0;
        double x_m = 0.0;
        double y_m = 0.0;
        double x_c = 0.0;
        double y_c = 0.0;
        double angle = 0.0;
        double map_landm_x = 0.0;
        double map_landm_y = 0.0;

        for(int n = 0; n < num_particles; n++){
            //initialize update_weight parameter
            update_weight = 1.0;
            //initialize pointer for particle
            Particle& pt = particles[n];
            for(int i = 0; i < num_obs; i++){
                //Transform observation from vehicle to global coordinates
                //Populate the observation coords into x_c & y_c for readability
                x_c = observations[i].x;
                y_c = observations[i].y;

                //angle of rotation for readability
                angle = pt.theta;

                //Perform Homogenous Transform
                x_m = pt.x + (cos(angle)*x_c) - (sin(angle)*y_c);
               // cout << "Transformed x : " << x_m <<endl;
                y_m = pt.y + (sin(angle)*x_c) + (cos(angle)*y_c);
               // cout << "Transformed y : " << y_m << endl;
                //Reset the goal to determine landmark closest to observation
                goal = 100;
                marker = -1;
                //Determine the landmark that is closest to the observation
                for(int j = 0; j < num_lm; j++){
                    //check observation against landmark & update goal 
                    //if the euclidean distance is now closer than previous
                    //Perform test to see if landmark is even within range
                    map_landm_x = map_landmarks.landmark_list[j].x_f;
                    // cout<<"LM_X: " << map_landm_x<<endl;
                    map_landm_y = map_landmarks.landmark_list[j].y_f; 
                    // cout<<"LM_Y: " << map_landm_y<<endl;
                    test_dist = dist(map_landm_x, map_landm_y, pt.x, pt.y);
                    //if landmark is in range then see that landmark is the closest
                    if(test_dist <= sensor_range){
                        goal_temp = dist(x_m, y_m, map_landm_x, map_landm_y);
                       // cout<<"goal_temp: " << goal_temp << endl;
                        if(goal_temp < goal){
                          //  cout<<"Set new goal"<<endl;
                            goal = goal_temp;
                            marker = map_landmarks.landmark_list[j].id_i;
                        }
                    }
                }
               //if(marker != -1){
                //cout marker that is closest landmark for observation
               // cout<< "ID: " << marker << endl;
                //update sqr_diff for x & y for readability
               // cout << "Near landm x: " << map_landmarks.landmark_list[marker-1].x_f << endl;
               // cout << "Near landm y: " << map_landmarks.landmark_list[marker-1].y_f << endl;
                x_diff = (x_m - map_landmarks.landmark_list[marker-1].x_f);
                y_diff = (y_m - map_landmarks.landmark_list[marker-1].y_f);
                //cout<< "x_diff: " << x_diff << endl;
                //cout<< "y_diff: " << y_diff << endl;
                x_sqr_diff = x_diff * x_diff;
                y_sqr_diff = y_diff * y_diff;
                //cout<< " x_sqr_diff: " << x_sqr_diff << endl;
                //cout<< " y_sqr_diff: " << x_sqr_diff << endl;
                //update weight, gaussian normal distribution
                //calculate expo term for readability 
                expo_term = (x_sqr_diff/(2.0*x_sqr_landm)) + (y_sqr_diff /(2.0*y_sqr_landm));
                //cout <<"Exp term: " << expo_term << endl;
                //update weight, exponential
                exponential =  exp(-expo_term);
 
                //Full update weight
                update_weight *= gauss_norm * exponential;
                
                //cout exponential
                //cout << "Exp: " << exponential << endl;
               // }else{
               // update_weight = 0.0;
                //}
            }
            //update the particle weight
            pt.weight = update_weight;
        
            //update weights vector
            weights[n] = update_weight;

            // Print your samples to the terminal.
           // cout << "Weight: " << update_weight << endl;
        }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
        // Utilized the following website to help me understand the resampling step of the particle filter
        // http://calebmadrigal.com/resampling-wheel-algorithm/, which talks about resample wheel in udacity robotics course
        //Also utilized several udacity Forum posts about the mt19937 random number generator and use of the discrete distribution function
        //cout<<"made it to resample"<<endl;
        std::random_device rd;
	std::mt19937 gen3(rd());
        //create new particle vector to store resample particles temporarily
        std::vector<Particle> updated_particles;
        updated_particles.resize(num_particles);
        //inspect part done for readability and rebugging
        Particle inspect_part;
        //Resample code utilized from udacity post "Resampling algorithm using resampling wheel"
        //Utilized Forum mentor driveWell suggest resample method as it made more sense than the resample wheel method
        std::discrete_distribution<int> DDistribution(weights.begin(), weights.end());
        //cout<<"made it to the loop in resample"<<endl;
        for(int n = 0; n < num_particles; ++n) {
            //done for readabiliity and debugging
            inspect_part = particles[DDistribution(gen3)];
            updated_particles[n] = inspect_part;
           // cout <<"Resamp Part x:"<<inspect_part.x <<" Resamp Part y:" << inspect_part.y << " Resamp Part theta:" <<inspect_part.theta << endl;
        } 
        particles = updated_particles;
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
