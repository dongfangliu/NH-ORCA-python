/*
 * Agent.h
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

/**
 * \file       Agent.h
 * \brief      Contains the Agent class.
 */

#include "Definitions.h"
#include "RVOSimulator.h"

namespace RVO {
	/**
	 * \brief      Defines an agent in the simulation.
	 */
	class Agent:public std::enable_shared_from_this<Agent> {
	public:
		/**
		 * \brief      ructs an agent instance.
		 * \param      sim             The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim);

		/**
		 * \brief      Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief      Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief      Inserts an agent neighbor into the set of neighbors of
		 *             this agent.
		 * \param      agent           A pointer to the agent to be inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertAgentNeighbor( std::shared_ptr<Agent>agent, float &rangeSq);

		/**
		 * \brief      Inserts a static obstacle neighbor into the set of neighbors
		 *             of this agent.
		 * \param      obstacle        The number of the static obstacle to be
		 *                             inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertObstacleNeighbor( std::shared_ptr<Obstacle>obstacle, float rangeSq);

		/**
		 * \brief      Updates the two-dimensional position and two-dimensional
		 *             velocity of this agent.
		 */
		void update();

		std::vector<std::pair<float,  std::shared_ptr<Agent>> > agentNeighbors_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		Eigen::Vector2f newVelocity_;
		std::vector<std::pair<float,  std::shared_ptr<Obstacle>> > obstacleNeighbors_;
		std::vector<Line> orcaLines_;
		Eigen::Vector2f position_;
		Eigen::Vector2f prefVelocity_;
		float radius_;
		RVOSimulator *sim_;
		float timeHorizon_;
		float timeHorizonObst_;
		Eigen::Vector2f velocity_;

		size_t id_;

		friend class KdTree;
		friend class RVOSimulator;
	};

	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear raints defined by lines and a circular
	 *             raint.
	 * \param      lines         Lines defining the linear raints.
	 * \param      lineNo        The specified line raint.
	 * \param      radius        The radius of the circular raint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1( std::vector<Line> &lines, size_t lineNo,
						float radius,  Eigen::Vector2f &optVelocity,
						bool directionOpt, Eigen::Vector2f &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             raints defined by lines and a circular raint.
	 * \param      lines         Lines defining the linear raints.
	 * \param      radius        The radius of the circular raint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	size_t linearProgram2( std::vector<Line> &lines, float radius,
						   Eigen::Vector2f &optVelocity, bool directionOpt,
						  Eigen::Vector2f &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             raints defined by lines and a circular raint.
	 * \param      lines         Lines defining the linear raints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular raint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3( std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
						float radius, Eigen::Vector2f &result);
}

#endif /* RVO_AGENT_H_ */
