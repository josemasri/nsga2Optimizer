
#ifndef ARLOTRAJECTORY_H
#define ARLOTRAJECTORY_H

#include "mop.h"
#include "../Individual.h"
#include <ros/ros.h>
#include "arlo_nn_controller/EvaluateDriver.h"
#include <vector>

using namespace std;

class ArloTrajectory : public ProblemFunction
{
public:
   //   ArloTrajectory(int nProcs=1, int rallySize=4, const int numObjs=3, const int numCons=1,
   //		         const int nInputs=60, const int nOutputs=5, const int nHidden=5);

   ArloTrajectory(int nProcs = 1, int teamSize = 3, const int numObjs = 2, const int numCons = 1,
                  const int nInputs = 96, const int nOutputs = 2, const int nHidden = 0);

   virtual ~ArloTrajectory();

   // This method come from the base class MOP.
   void evaluatePop(Individual **ind, int popSize) const;

   void evaluate(vector<double> const &x,
                         vector<int> const &gene,
                         vector<double> &fx,
                         vector<double> &gcons) const;

private:
   int nProcs;   // Number of simultaneous processes to evaluate the population.
   int teamSize; // Number of robots in the team

   int numInputsNN;  // Number of inputs of the neural network.
   int numOutputsNN; // Number of outputs of the neural network.
   int numHidden;    // Number of hidden nodes of the neural network.

   string popFilePattern; // Name of the file in which each individual will be written.

   string objFilePattern; // Name of the file in which the evaluation of each individual is written by the server.
   ofstream popStream;    // Stream to write each individual.

   ifstream objStream; // Stream to read the evaluation of each individual.

   void resetObjVectors(Individual **ind, int popSize);

   void writeWeightsFile(vector<double> const &weights, int idxIndividual = 1) const;

   void readSimEvaluation(int idxRally,
                          int idxInd,
                          int rallySize,
                          Individual **ind);

   void printVector(vector<double> &v);
   string getPopFileName(int i = 1) const;

   string getObjFileName(int i);

   void readObjectiveEvaluation(int idxIndividual,
                                vector<double> &objValues,
                                vector<double> &consValues);
};

#endif /* ARLOTRAJECTORY_H */
