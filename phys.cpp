#include <iostream>
#include <cmath>
#include <tira/image.h>
#include <vector>
#include <filesystem>
#include <string>

// Global constants
const float PI = 3.14159265358f;
const float CONVERSION_DEG_TO_RAD = 2.f * PI / 360.f;
const float multiplier = 4 * std::sqrt(2) + 13;

// Simulation parameters
const int SHAPE_WIDTH = 300;
const int SHAPE_HEIGHT = 300;
const int NUM_STEPS = 5000;
const float POPULATION_RATIO = 0.6f; // between 0.03 and 0.15
const int NUM_AGENTS = std::floor(SHAPE_HEIGHT * SHAPE_WIDTH * POPULATION_RATIO);
const bool PERIODIC_BOUNDARY = false;
const bool SAVE_TRAIL = true;
const int SAVE_STEP = 1;
int token; // Random token of the folder

// Agent parameters
int SENSOR_COUNT = 3;
float SENSOR_OFFSET = 15.f;
float SENSOR_ANGLE = 45.f * CONVERSION_DEG_TO_RAD;

float ROTATION_ANGLE = 45.0f * CONVERSION_DEG_TO_RAD;
float AGENT_STEP = 1.f; // Move, step size
float DELTA = 0.1f; // Deposit value
float DECAY = 0.1f;

tira::image<float> convolveMask(3, 3, 1); // 3x3 mask for convolving

tira::image<float> trailField(SHAPE_WIDTH, SHAPE_HEIGHT, 1);

struct Agent {
	float pos[2];
	float theta; // in radians
};

float customRound(float value) {
	return (value >= 0.0) ? std::floor(value + 0.5) : std::ceil(value - 0.49999);
}

int main(int argc, char** argv) {
	// Setup the mask
	srand(time(NULL));
	convolveMask(1, 1, 0) = 2 / 3.f;

	convolveMask(0, 1, 0) = 1 / 18.f;
	convolveMask(1, 0, 0) = convolveMask(0, 1, 0);
	convolveMask(2, 1, 0) = convolveMask(0, 1, 0);
	convolveMask(1, 2, 0) = convolveMask(0, 1, 0);

	convolveMask(0, 0, 0) = 1 / 36.f;
	convolveMask(2, 0, 0) = convolveMask(0, 0, 0);
	convolveMask(0, 2, 0) = convolveMask(0, 0, 0);
	convolveMask(2, 2, 0) = convolveMask(0, 0, 0);
	/*
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			convolveMask(i, j, 0) = 1.0f / 9;
		}
	}
	*/
	// Create the folder if needed
	if (SAVE_TRAIL) {
		token = 52;
		std::filesystem::create_directories("./sim_" + std::to_string(token));
	}

	std::vector<Agent> agents;
	// 1.1 Initialize agents
	float random_x, random_y, random_theta;
	for (int i = 0; i < NUM_AGENTS; i++) {
		random_x = float(rand() % 1000) / 1000.f * (SHAPE_WIDTH - 1);
		random_y = float(rand() % 1000) / 1000.f * (SHAPE_HEIGHT - 1);
		random_theta = float(rand() % 314) / 314.f * 2 * 3.14159265358f;
		agents.push_back(Agent({ {random_x, random_y}, random_theta }));
	}

	
	// 1.2 Setup sensors
	float sensors[3] = { -SENSOR_ANGLE, 0.f, SENSOR_ANGLE };
	int sensed_x;
	int sensed_y;
	float sensed[3];

	for (int STEP = 0; STEP < NUM_STEPS; STEP++) {
		// Iterate sense, move, rotate for each agent
		if (!(STEP % 100)) {
			std::cout << STEP << std::endl;
		}

		for (Agent& agent : agents) {
			// 2. Sense the trail field
			for (int i = 0; i < 3; i++) {
				sensed_x = customRound(agent.pos[0] + std::cos(agent.theta + sensors[i]) * SENSOR_OFFSET);
				sensed_y = customRound(agent.pos[1] + std::sin(agent.theta + sensors[i]) * SENSOR_OFFSET);
				if (PERIODIC_BOUNDARY) {
					sensed[i] = trailField((sensed_x + 2 * SHAPE_WIDTH) % SHAPE_WIDTH, (sensed_y + 2 * SHAPE_HEIGHT) % SHAPE_HEIGHT, 0);
				}
				else {
					// Check if out of bounds, return 0 otherwise
					if (sensed_x < 0 || sensed_x >= SHAPE_WIDTH || sensed_y < 0 || sensed_y >= SHAPE_HEIGHT) {
						sensed[i] = 0;
					}
					else {
						sensed[i] = trailField(sensed_x, sensed_y, 0);
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
			agent.theta += index * ROTATION_ANGLE;
			// 4. Move
			agent.pos[0] += std::cos(agent.theta) * AGENT_STEP;
			agent.pos[1] += std::sin(agent.theta) * AGENT_STEP;
			if (PERIODIC_BOUNDARY) {
				// Check if out of bounds, trace back to the opposite field
				if (customRound(agent.pos[0]) >= SHAPE_WIDTH) {
					agent.pos[0] -= SHAPE_WIDTH;
				}
				else if (customRound(agent.pos[0]) < 0) {
					agent.pos[0] += SHAPE_WIDTH;
				}
				if (customRound(agent.pos[1]) >= SHAPE_HEIGHT) {
					agent.pos[1] -= SHAPE_HEIGHT;
				}
				else if (customRound(agent.pos[1]) < 0) {
					agent.pos[1] += SHAPE_HEIGHT;
				}
			}
			else {
				// Check if out of bounds, turn around and move back into the field
				if (customRound(agent.pos[0]) >= SHAPE_WIDTH || customRound(agent.pos[0]) < 0 ||
					customRound(agent.pos[1]) >= SHAPE_HEIGHT || customRound(agent.pos[1]) < 0) {
					agent.theta += PI * ((rand() % 2) * 2 - 1);
					agent.pos[0] += std::cos(agent.theta) * AGENT_STEP;
					agent.pos[1] += std::sin(agent.theta) * AGENT_STEP;
				}
			}
		}
		
		// 5. Deposit
		for (Agent& agent : agents) {
			trailField(customRound(agent.pos[0]), customRound(agent.pos[1]), 0) += DELTA;
		}
		
		// 6. Update Trail field
		// 6.1. Diffuse
		// Uniform filter, possibly add to tiralib
		float new_val;
		if (PERIODIC_BOUNDARY) {
			for (int x = 0; x < SHAPE_WIDTH; x++) {
				for (int y = 0; y < SHAPE_HEIGHT; y++) {
					// Sum up all the values around the initial cell using modulo
					new_val = 0;
					for (int x_tmp = x - 1 + SHAPE_WIDTH; x_tmp <= x + 1 + SHAPE_WIDTH; x_tmp++) {
						for (int y_tmp = y - 1 + SHAPE_HEIGHT; y_tmp <= y + 1 + SHAPE_HEIGHT; y_tmp++) {
							new_val += trailField(x_tmp % SHAPE_WIDTH, y_tmp % SHAPE_HEIGHT, 0) / 9.f;
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
		for (int x = 0; x < SHAPE_WIDTH; x++) {
			for (int y = 0; y < SHAPE_HEIGHT; y++) {
				trailField(x, y, 0) = trailField(x, y, 0) * (1 - DECAY);
			}
		}

		if (SAVE_TRAIL && (STEP % SAVE_STEP == 0)) {
			trailField.cmap(0.f, 3.f, ColorMap::Magma).save("./sim_" + std::to_string(token) + "/step_" + std::to_string(STEP) + ".bmp");
			// ./sim_token/step_50.bmp
		}
	}
	

}