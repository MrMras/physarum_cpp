#include <iostream>
#include <cmath>
#include <tira/image.h>
#include <vector>
#include <filesystem>
#include <string>

// Global constants
const float PI = 3.14159265358f;
const float CONVERSION_DEG_TO_RAD = 2.f * PI / 360.f;

// Simulation parameters
const float TOTAL_UNITS = 1.0f;
const int NUM_STEPS = 10000;
const int NUM_AGENTS = 10000;
const bool PERIODIC_BOUNDARY = true;
const bool SAVE_TRAIL = false;
const int SAVE_STEP = 1;
int token; // Random token of the folder

// Agent parameters
int SENSOR_COUNT = 3;
float SENSOR_OFFSET = TOTAL_UNITS * 38 / 300; // in units
float SENSOR_ANGLE = 45.f * CONVERSION_DEG_TO_RAD;

float AGENT_STEP = TOTAL_UNITS / 300; // Move, step size
float ROTATION_ANGLE = 45.0f * CONVERSION_DEG_TO_RAD;
const float RANDOM_ANGLE_ROTATION = 0.0f * CONVERSION_DEG_TO_RAD;

float DELTA = 0.1f; // Deposit value
float DECAY = 0.1f;

// Image simulation
const int MATRIX_SIZE = 300; // Assume shape of square

tira::image<float> convolveMask(3, 3, 1); // 3x3 mask for convolving

tira::image<float> trailField(MATRIX_SIZE, MATRIX_SIZE, 1);

struct Agent {
	float pos[2];
	float theta; // in radians
};

bool outOfBounds(float _unit) {
	if (_unit < 0 || _unit >= TOTAL_UNITS) return true;
	
	return false;
}

float unit2pixel(float _unit) {
	return int(std::floor(_unit / TOTAL_UNITS * MATRIX_SIZE));
}

int main(int argc, char** argv) {
	// Setup the mask
	srand(time(NULL));
	/*
	convolveMask(1, 1, 0) = 1 / 9.f;

	convolveMask(0, 1, 0) = 1 / 9.f;
	convolveMask(1, 0, 0) = convolveMask(0, 1, 0);
	convolveMask(2, 1, 0) = convolveMask(0, 1, 0);
	convolveMask(1, 2, 0) = convolveMask(0, 1, 0);

	convolveMask(0, 0, 0) = 1 / 9.f;
	convolveMask(2, 0, 0) = convolveMask(0, 0, 0);
	convolveMask(0, 2, 0) = convolveMask(0, 0, 0);
	convolveMask(2, 2, 0) = convolveMask(0, 0, 0);
	*/
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			convolveMask(i, j, 0) = 1.0f / 9;
		}
	}
	
	// Create the folder if needed
	if (SAVE_TRAIL) {
		token = 52;
		std::filesystem::create_directories("./sim_" + std::to_string(token));
	}
	
	// Clean the folder
	if (SAVE_TRAIL) {
		for (const auto& entry : std::filesystem::directory_iterator("./sim_" + std::to_string(token))) {
			std::filesystem::remove(entry.path());
		}
	}

	std::vector<Agent> agents;
	// 1.1 Initialize agents
	float random_x, random_y, random_theta;
	for (int i = 0; i < NUM_AGENTS; i++) {
		random_x = (rand() % 1000) / 1000.f * TOTAL_UNITS;
		random_y = (rand() % 1000) / 1000.f * TOTAL_UNITS;
		random_theta = (rand() % 1000) / 1000.f * 2 * PI;
		agents.push_back(Agent({ {random_x, random_y}, random_theta }));
	}

	
	// 1.2 Setup sensors
	float sensors[3] = { -SENSOR_ANGLE, 0.f, SENSOR_ANGLE };
	float sensed[3];

	float sensed_x;
	float sensed_y;


	for (int STEP = 0; STEP < NUM_STEPS; STEP++) {
		// Iterate sense, move, rotate for each agent
		if (!(STEP % 100)) {
			std::cout << STEP << std::endl;
		}
		for (Agent& agent : agents) {
			// 2. Sense the trail field
			for (int i = 0; i < 3; i++) {
				sensed_x = agent.pos[0] + std::cos(agent.theta + sensors[i]) * SENSOR_OFFSET;
				sensed_y = agent.pos[1] + std::sin(agent.theta + sensors[i]) * SENSOR_OFFSET;
				if (PERIODIC_BOUNDARY) {
					sensed_x = fmod(sensed_x, TOTAL_UNITS);
					if (sensed_x < 0) {
						sensed_x += TOTAL_UNITS;
					}
					sensed_y = fmod(sensed_y, TOTAL_UNITS);
					if (sensed_y < 0) {
						sensed_y += TOTAL_UNITS;
					}
					sensed[i] = trailField(unit2pixel(sensed_x), unit2pixel(sensed_y), 0);
				}
				else {
					// std::cout << sensed_x << " " << sensed_y << std::endl;
					
					if (outOfBounds(sensed_x) || outOfBounds(sensed_y)) {
						sensed[i] = 0;
					}
					else {
						// std::cout << unit2pixel(sensed_x) << " " << unit2pixel(sensed_y) << std::endl;
						// std::cout << trailField(unit2pixel(sensed_x), unit2pixel(sensed_y), 0) << std::endl;
						sensed[i] = trailField(unit2pixel(sensed_x), unit2pixel(sensed_y), 0);
					}
				}	
			}
			// Direction choosing algorithm
			float index;
			if (sensed[1] > sensed[0] && sensed[1] > sensed[2]) {
				index = 0.f;
			}
			else if (sensed[1] < sensed[0] && sensed[1] < sensed[2]) {
				index = float(rand() % 2 * 2. - 1);
			}
			else if (sensed[0] > sensed[2]) {
				index = -1.f;
			}
			else if (sensed[0] < sensed[2]) {
				index = 1.f;
			}
			else {
				index = float(rand() % 3 - 1);
			}
			// 3. Rotate based on index
			agent.theta += index * ROTATION_ANGLE + RANDOM_ANGLE_ROTATION * (rand() % 2 - 1);
			// 4. Move
			agent.pos[0] += std::cos(agent.theta) * AGENT_STEP;
			agent.pos[1] += std::sin(agent.theta) * AGENT_STEP;
			if (PERIODIC_BOUNDARY) {
				// Check if out of bounds, trace back to the opposite field
				if (outOfBounds(agent.pos[0])) {
					agent.pos[0] = fmod(agent.pos[0] + TOTAL_UNITS, TOTAL_UNITS);
					if (outOfBounds(agent.pos[0])) agent.pos[2];
				}
				if (outOfBounds(agent.pos[1])) {
					agent.pos[1] = fmod(agent.pos[1] + TOTAL_UNITS, TOTAL_UNITS);
				}
			}
			else {
				// Check if out of bounds, turn around and move back into the field
				if (outOfBounds(agent.pos[0]) || outOfBounds(agent.pos[1])) {
					agent.theta += PI * ((rand() % 2) * 2 - 1);
					agent.pos[0] += std::cos(agent.theta) * AGENT_STEP;
					agent.pos[1] += std::sin(agent.theta) * AGENT_STEP;
				}
			}
		}
		// 5. Deposit
		for (Agent& agent : agents) {
			trailField(unit2pixel(agent.pos[0]), unit2pixel(agent.pos[1]), 0) += DELTA;
		}
		
		// 6. Update Trail field
		// 6.1. Diffuse
		// Uniform filter, possibly add to tiralib
		float new_val; // TODO!!!
		if (PERIODIC_BOUNDARY) {
			for (int x = 0; x < MATRIX_SIZE; x++) {
				for (int y = 0; y < MATRIX_SIZE; y++) {
					// Sum up all the values around the initial cell using modulo
					new_val = 0;
					for (int x_tmp = x - 1 + MATRIX_SIZE; x_tmp <= x + 1 + MATRIX_SIZE; x_tmp++) {
						for (int y_tmp = y - 1 + MATRIX_SIZE; y_tmp <= y + 1 + MATRIX_SIZE; y_tmp++) {
							new_val += trailField(x_tmp % MATRIX_SIZE, y_tmp % MATRIX_SIZE, 0) / 9.f;
						}
					}
					trailField(x, y, 0) = new_val;	
				}
			}
		}
		else {
			trailField = trailField.border(1).convolve2(convolveMask);
		}
		
		// 6.2. Decay

		// trailField *= DECAY; // Possibly add the operator to tiralib/image.h
		for (int x = 0; x < MATRIX_SIZE; x++) {
			for (int y = 0; y < MATRIX_SIZE; y++) {
				trailField(x, y, 0) = trailField(x, y, 0) * (1 - DECAY);
			}
		}

		if (SAVE_TRAIL && (STEP % SAVE_STEP == 0)) {
			trailField.cmap(ColorMap::Magma).save("./sim_" + std::to_string(token) + "/step_" + std::to_string(STEP) + ".bmp");
			// ./sim_token/step_50.bmp
		}
	}
	

}