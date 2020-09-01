#include <unistd.h>
#include <sys/wait.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <utility>
#include <iomanip>
#include "ArloTrajectory.h"

using namespace std;

/*********************************************************************
* Constructor of the ArloTrajectory Multi-Objective Problem.
*********************************************************************/

ArloTrajectory::ArloTrajectory(int nProcs, int teamSize, int numObjs, int numCons, int nInputs, int nOutputs, int nHidden)
    : ProblemFunction("ArloTrajectory", nInputs * nHidden + nHidden * nOutputs, numObjs, numCons)
{
   this->nProcs = nProcs;
   this->numInputsNN = nInputs;
   this->numOutputsNN = nOutputs;
   this->numHidden = nHidden;
   this->teamSize = teamSize;

   if (nHidden == 0)
      this->nVariables = nInputs * nOutputs;

   // Set the range for the NN's weight values.
   vector<pair<double, double>> ranges(nVariables, make_pair(-8000, 8000));
   setVarsRange(ranges);

   popFilePattern.assign("../arlo_nn_controller/popfile_%02d.csv");

   objFilePattern.assign("../arlo_nn_controller/objEval_%02d.csv");
}

ArloTrajectory::~ArloTrajectory() {}

// void ArloTrajectory::evaluate(vector<double> const &x1, vector<double> const &x2, vector<double> const &x3, vector<int> const &gene, vector<double> &f,  vector<double> &cons) const
// {
//    // Create a client object to remotely call the "evaluate_driver" service in a ROS node.
//    ros::NodeHandle nh;
//    ros::ServiceClient  client = nh.serviceClient<arlo_nn_controller::EvaluateDriver>("evaluate_driver");

//    /** 1. Write the weights for the NN encoded by the individual */
//    writeWeightsFile1(x1, 1);
//    writeWeightsFile2(x2, 2);
//    writeWeightsFile3(x3, 3);

//    /** 2.Solicitar el servicio de simulación de un driver de arlo al nodo de simulación de ROS */
//    arlo_nn_controller::EvaluateDriver server;
//    server.request.maxtime = 50; // 300 segundos.
//    server.request.weightsfile1 = getPopFileName1();
//    server.request.weightsfile2 = getPopFileName2();
//    server.request.weightsfile3 = getPopFileName3();

//    if (client.call(server))
//    {
//        /** 3. Take the output  **/
//        f[0] = server.response.time;
//        f[1] = server.response.energy;
//        cons[0] = -1*server.response.dist2go;

//        cout << "time: " << f[0] << ", energy: " << f[1] << ", dist2go: " << cons[0] << endl;
//    }
//    else
//    {
// 	   ROS_ERROR("Failed to call service evaluate_driver to evaluate Xolobot Driver.\n\n");
//    }
// }

/*
 */
//void ArloTrajectory::evaluatePop(Individual **ind, int popSize)
//{
//   pid_t pid; // to get result from calling fork.
//
//   int remainingInds;          // Number of individuals to be evaluated.
//   int indCounter;             // Number of individuals evaluated.
//   int nProcessors = nProcs;       // Number of parallel RALLIES.
//   vector<string> trackNames = {"g-track-2", "forza"};
//   //vector<string> trackNames = {"g-track-2"};
//
//   if (popSize % rallySize != 0) {
//      perror("\nThe population size must be multiple of the rally size.\n");
//      exit(1);
//   }
//
//   resetObjVectors(ind, popSize);
//
//   cerr << "\nNumber of processors: " << nProcessors << endl;
//   for (string tname: trackNames) {
//
//      // Repeat all the simulations for the each track.
//      indCounter = 0;
//      remainingInds = popSize;
//
//      while (remainingInds > 0) {
//         cerr << "\n\nBonche de individuos que comienza en " << indCounter << endl;
//
//         // Does the remaining individuals needs less than nProcessors?
//         if (remainingInds < nProcessors*rallySize)
//            nProcessors = remainingInds / rallySize;
//
//         /** 1. Write every individual of the nProcessors rallies to a different output file ending with "_i" **/
//         for (int i = 0, j=indCounter; i < nProcessors*rallySize; i++, j++)
//            writePopFile(i, ind[j]->xreal);
//
//         /** 2. Create a different child process for each nP TORCS server w rallySize clients (a RALLY) **/
//         for (int i = 0; i < nProcessors; i++) {
//            cerr << "\nEjecución para procesador " << i << endl;
//
//            // Create a child process.
//            pid = fork();
//            if (pid == 0) { // If you are the child, change your code image.
//
//
//               // Execute each car j of the rally for processor i
//               for (int r = 0, j=i*rallySize; r < rallySize; ++r, ++j) {
//                  string port = std::to_string(3001 + j);
//                  int serverID = (r==0) ? i : -1;
//                  stringstream clientCmd;
//                  clientCmd << "./launch_torcs_client.sh " << " "
//                        << port.c_str() << " "
//                        << getPopFileName(j) << " "
//                        << getObjFileName(i) << " "
//                        << std::to_string(serverID).c_str() << " "
//                        << std::to_string(rallySize).c_str() << endl;
//
//                  system(clientCmd.str().c_str());  // Con system porque no es necesario esperar nada.
//               }
//
//               stringstream command;
//               command << "./launch_torcs_server_rally_reuse_cars.sh" << " "
//                     << std::to_string(i).c_str() << " "
//                     << getObjFileName(i).c_str() << " "
//                     << std::to_string(rallySize).c_str() << " "
//                     << tname.c_str() << endl;
//
//               // Execute the server for the rally for processor i
//               int res = execlp("./launch_torcs_server_rally_reuse_cars.sh",
//                     "./launch_torcs_server_rally_reuse_cars.sh",
//                     std::to_string(i).c_str(),
//                     getObjFileName(i).c_str(),
//                     std::to_string(rallySize).c_str(),
//                     tname.c_str(), NULL);
//
//               if (res == -1) {
//                  perror("Error al ejecutar execlp");
//                  exit(1);
//               }
//            }
//         }
//
//         /** 3. Wait for every child process to finish the simulation ***/
//         cerr << "\nNSGA2 waiting for TORCS simulation results...";
//         while ( wait(NULL) > 0 ); // Esto espera a todos los hijos creados (simulaciones paralelas)
//         cerr << "\n...Resultados listos.\n";
//
//         /** 4. Read every output file for each rally **/
//         for (int i = 0, j=indCounter; i < nProcessors; i++, j+=rallySize)
//            readRallyEvaluation(i, j, rallySize, ind);
//
//         indCounter += nProcessors*rallySize;
//         remainingInds -= nProcessors*rallySize;
//      }
//   }
//}

void ArloTrajectory::evaluatePop(Individual **ind, int popSize) const
{
   // Create a client object to remotely call the "evaluate_driver" service in a ROS node.
   ros::NodeHandle nh;
   ros::ServiceClient client = nh.serviceClient<arlo_nn_controller::EvaluateDriver>("evaluate_driver");
   for (int i = 0; i < popSize; i += teamSize)
   {
      /** 1. Write the weights for the NN encoded by the individual */
      writeWeightsFile(ind[i]->xreal, 1);
      writeWeightsFile(ind[i + 1]->xreal, 2);
      writeWeightsFile(ind[i + 2]->xreal, 3);

      /** 2.Solicitar el servicio de simulación de un driver de arlo al nodo de simulación de ROS */
      arlo_nn_controller::EvaluateDriver server;
      server.request.maxtime = 30; 
      server.request.weightsfile1 = getPopFileName(1);
      server.request.weightsfile2 = getPopFileName(2);
      server.request.weightsfile3 = getPopFileName(3);

      if (client.call(server))
      {
         /** 3. Take the output  **/
         for (int i = 0; i < 3; i++)
         {
            ind[i]->obj[0] = server.response.time;
            ind[i]->obj[1] = -1 * server.response.boxDistance;
         }

         cout << "time: " << ind[0]->obj[0] << ", boxDistance: " << ind[0]->obj[1] << endl;
      }
      else
      {
         ROS_ERROR("Failed to call service evaluate_driver to evaluate Xolobot Driver.\n\n");
      }
      ind[i]->constr_violation = 0.0;
      ind[i + 1]->constr_violation = 0.0;
      ind[i + 2]->constr_violation = 0.0;
   }
}

void ArloTrajectory::writeWeightsFile(vector<double> const &weights, int idxIndividual) const
{
   ofstream popStream;
   popStream.exceptions(std::ifstream::failbit | std::ifstream::badbit);

   try
   {
      cerr << "Antes de abrirlo "
           << "\n";
      popStream.open(getPopFileName(idxIndividual), std::ofstream::out);
      popStream << fixed << setw(5 + 8) << setprecision(8);
      cerr << "Se creo el archivo "
           << "\n";
      //cout << "Escribiré el archivo de pesos: " << getPopFileName(idxIndividual) << endl;
      popStream << numInputsNN << "\n"
                << numOutputsNN << "\n"
                << numHidden << "\n";

      if (numHidden == 0)
      {
         for (int i = 0; i < numInputsNN; i++)
         {
            for (int j = 0; j < numOutputsNN; j++)
            {
               popStream << weights[i * numOutputsNN + j] << "\t";
            }
            popStream << "\t"; // Change \t for \n to write row in different lines.
         }

         //popStream.close();
      }
      else if (numHidden > 0)
      {
         for (int i = 0; i < numInputsNN; i++)
         {
            for (int j = 0; j < numHidden; j++)
            {
               popStream << weights[i * numHidden + j] << "\t";
            }
            //popStream << "\t"; // Change \t for \n to write row in different lines.
         }

         int numWeightsHid1 = numInputsNN * numHidden;

         for (int i = 0; i < numHidden; i++)
         {
            for (int j = 0; j < numOutputsNN; j++)
            {
               popStream << weights[numWeightsHid1 + i * numOutputsNN + j] << "\t";
            }
            //popStream << "\t"; // Change \t for \n to write row in different lines.
         }
      }

      popStream.close();
      cerr << "Todo bien "
           << "\n";
   }
   catch (ofstream::failure &e)
   {
      cerr << "No se pudo crear el achivo: " << getPopFileName(idxIndividual) << "\n";
      exit(0);
   }
}

void ArloTrajectory::resetObjVectors(Individual **ind, int popSize)
{
   for (int j = 0; j < popSize; j++)
   {
      ind[j]->constr_violation = 0;
      ind[j]->constr[0] = 0;
      for (int i = 0; i < nObjectives; i++)
      {
         ind[j]->obj[i] = 0;
      }
   }
}

//void ArloTrajectory::readRallyEvaluation(int idxRally, int idxInd, int rallySize, Individual **ind)
//{
//   ifstream objStream(getObjFileName(idxRally), std::ifstream::in);
//   //cout << "FINALMENTE ---> Leeré el archivo de evaluaciones: " << getObjFileName(idxIndividual) << endl;
//   float dummie;
//   for (int j = idxInd; j < idxInd+rallySize; ++j) {
//      objStream >> dummie; // LEER la posición y desecharla.
//      cerr << "j=" << j << "--> \t\t" << dummie << "\t\t";
//
//      double value;
//      for (int i = 0; i < nObjectives; i++) {
//         objStream >> value;
//         ind[j]->obj[i] += value;
//         cerr << ind[j]->obj[i] << "\t\t";
//      }
//
//      // The last value is Distance_to_finish
//      objStream >> value;
//      ind[j]->constr[0] += value; // Este será el acumulado.
//      ind[j]->constr_violation = -1*ind[j]->constr[0]; // OJO: Este no debe llevar +=.
//      cerr << ind[j]->constr_violation << endl;
//   }
//
//   objStream.close();
//}

string ArloTrajectory::getPopFileName(int i) const
{
   char buffer[200];
   sprintf(buffer, popFilePattern.c_str(), i);
   return string(buffer);
}

string ArloTrajectory::getObjFileName(int i)
{
   char buffer[200];
   sprintf(buffer, objFilePattern.c_str(), i);
   return string(buffer);
}

void ArloTrajectory::printVector(vector<double> &v)
{
   for (size_t i = 0; i < v.size(); i++)
   {
      std::cout << v[i] << "\t";
   }
}

void ArloTrajectory::evaluate(vector<double> const &x,
                              vector<int> const &gene,
                              vector<double> &fx,
                              vector<double> &gcons) const {};