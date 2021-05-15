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

int main(int argc, char *argv[])
{
	// INITIALIZATIONS AND PARAMETERS
	srand(time(NULL));
	size_t nBots = 50;
	double step = 1/((double) nBots);
	size_t windowSize = 1000;
	double scale_factor = 8;
	size_t botSize = 5;
	double circleRad = 40.0f;
	std::vector <cv::Scalar> colors = random_colors_gen(nBots);
	graphicsHandler display(windowSize, windowSize, scale_factor, botSize);

	// ENVIRONMENT DEFINITION

	Simulator simulator;
	simulator.setTimeStep(0.25f);
	simulator.setAgentDefaults(15.0f, 10, 1.f, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < nBots; ++i) {
		const Vector2 position = circleRad * Vector2(std::cos(step * i * HRVO_TWO_PI), std::sin(step * i * HRVO_TWO_PI));
		simulator.addAgent(position, simulator.addGoal(-position));
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
// FUNCIONES
//#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#

// Generador de colores aleatorios para los robots.
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