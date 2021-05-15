/*
 * Circle.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

/**
 * \file   Circle.cpp
 * \brief  Example with 250 agents navigating through a circular environment.
 */
#define _USE_MATH_DEFINES
#include <graphics_handler.hpp>
#include <cmath>
#include <HRVO.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
using namespace hrvo;

std::vector <cv::Scalar> random_colors_gen(size_t nBots);
std::vector <Vector2> gen_square_positions(size_t nBots, double side_lenght);
std::vector <Vector2> gen_L_letter_positions(size_t nBots, double side_lenght);

int main(int argc, char *argv[])
{
	// INITIALIZATIONS AND PARAMETERS
	srand(time(NULL));
	size_t nBots = 30;
	double step = 1/((double) nBots);
	size_t windowSize = 1000;
	double scale_factor = 8;
	size_t botSize = 5;
	double side_lenght = 60.0f;
	std::vector <cv::Scalar> colors = random_colors_gen(nBots);
	graphicsHandler display(windowSize, windowSize, scale_factor, botSize);

	std::vector <Vector2> square_positions = gen_square_positions(nBots, side_lenght);
	std::vector <Vector2> L_letter_positions = gen_L_letter_positions(nBots, side_lenght);
	// ENVIRONMENT DEFINITION

	Simulator simulator;
	simulator.setTimeStep(0.25f);
	simulator.setAgentDefaults(15.0f, 10, 1.f, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < nBots; ++i) {
		simulator.addAgent(square_positions[i], simulator.addGoal(L_letter_positions[i]));
	}

	double x, y;
	do {
		for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
			x = simulator.getAgentPosition(i).getX();
			y = simulator.getAgentPosition(i).getY();
			display.drawRobot(x,y,colors[i]);
		}
		display.showAndClearImage();
		simulator.doStep();
	}
	while (!simulator.haveReachedGoals());

	bool visible = true;
	while(visible)
	{
		cv::waitKey(0);
		visible = cv::getWindowProperty(display.get_window_name(),cv::WND_PROP_VISIBLE) != 0;
	}
	return 0;
}

//#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
// FUNCTIONS
//#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#

// Generator of random colors for the robots
std::vector <cv::Scalar> random_colors_gen(size_t nBots){
	std::vector <cv::Scalar> colors;
	double R, G, B;
	for (size_t i = 0; i<nBots; i ++)
	{
		R = rand()%225 + 30;
		G = rand()%225 + 30;
		B = rand()%225 + 30;
		colors.push_back(cv::Scalar(R,G,B));
	}
	return colors;
}

// Generator of the initial positions along a square centered in the 0,0
std::vector <Vector2> gen_square_positions(size_t nBots, double side_lenght){
	std::vector <Vector2> positions;
	Vector2 currPos = Vector2(side_lenght/2, side_lenght/2);
	positions.push_back(currPos);
	double step = side_lenght * 4 / ((float) nBots);
	
	std::vector <Vector2> directions;
	directions.push_back(Vector2( 0,-1)*step); // 1st go down
	directions.push_back(Vector2(-1, 0)*step); // 2nd go left
	directions.push_back(Vector2( 0, 1)*step); // 3rd go up
	directions.push_back(Vector2( 1, 0)*step); // 4rd go right

	std::vector <double> lenghts;
	lenghts.push_back(side_lenght); 			
	lenghts.push_back(side_lenght);
	lenghts.push_back(side_lenght);
	lenghts.push_back(side_lenght);
	uint j = 0; // keeps track of the direction
	double already_advanced = 0;

	for (size_t i; i < nBots-1; i++){
		currPos += directions[j]; // move in the current direction
		already_advanced += step;
		if (already_advanced > lenghts[j]){
			currPos -= directions[j]; // go back
			currPos += directions[j]/step * (step - (already_advanced-lenghts[j])); // advance untill the end of the side
			j += 1; // change directions
			currPos += directions[j]/step * (already_advanced-lenghts[j-1]); // advance the rest of the step in the new direction
			already_advanced -= lenghts[j-1]; // restart the counter of the advancement on the new side
		}
		positions.push_back(currPos);
	}
	return positions;
}


std::vector <Vector2> gen_L_letter_positions(size_t nBots, double side_lenght){
		std::vector <Vector2> positions;

	
	Vector2 currPos = Vector2(-side_lenght/4, side_lenght/2);
	positions.push_back(currPos);

	double step = (side_lenght + side_lenght*3/4) / ((float) nBots);
	
	std::vector <Vector2> directions;
	directions.push_back(Vector2( 0,-1)*step); // 1st go down
	directions.push_back(Vector2( 1, 0)*step); // 2nd go right
	std::vector <double> lenghts;
	lenghts.push_back(side_lenght); 			// vertical is as tall as square
	lenghts.push_back(side_lenght*3/4);			// horizontal is 3/4 the lenght of the vertical
	uint j = 0; // keeps track of the direction
	double already_advanced = 0;

	for (size_t i; i < nBots-1; i++){
		currPos += directions[j]; // move in the current direction
		already_advanced += step;
		if (already_advanced > lenghts[j]){
			currPos -= directions[j]; // go back
			currPos += directions[j]/step * (step - (already_advanced-lenghts[j])); // advance untill the end of the side
			j += 1; // change directions
			currPos += directions[j]/step * (already_advanced-lenghts[j-1]); // advance the rest of the step in the new direction
			already_advanced -= lenghts[j-1]; // restart the counter of the advancement on the new side
		}
		positions.push_back(currPos);
	}
	return positions;
}