/*
 * NSGA.h
 *
 *  Created on: 11/Junio/2018
 *      Author: Antonio López Jaimes
 *
 */

#ifndef NSGA_H_
#define NSGA_H_

#include "mops/mop.h"
#include "RandUtils.h"
#include "List.h"
#include "Population.h"
#include "Individual.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

using namespace std;


/* Contains all the possible parameters of
 * NSGA.
 */
struct NSGAParams {
   int popsize;        // Population size.
   int ngen;           // Number of generations.
   int nreal;          // Number of variables using a real encoding.
   double pcross_real; // Crossover probability for real variables.
   double pmut_real;   // Mutation probability for real variables.
   double eta_c;
   double eta_m;
   int nbin;           // Number of variables using binary encoding.
   vector<int> nbits;  // Bits for each binary encoded variable.
   double pcross_bin;  // Crossover probability for binary variables.
   double pmut_bin;    // Mutation probability for binary variables.
};

class NSGA {

public:
	bool verbose;
   ofstream file_firstPop;    //fpt1
   ofstream file_everyPop;    //fpt4
   ofstream file_finalPop;    //fpt2
   ofstream file_feasiblePop; //fpt3
   ofstream file_params;      //ftp5


   ofstream file_varSpace;
   ofstream file_objSpace;
   ofstream file_popHistory;

//   ofstream file_archive;

   string outputDir;
   string name_firstPop;
   string name_everyPop;
   string name_finalPop;
   string name_feasiblePop;
   string name_params;

   string name_varSpace;
   string name_objSpace;
   string name_popHistory;


   ProblemFunction *mop;
   Randomizer *r;

   /* Atributos del NSGA */
   Population *parent_pop;
   Population *child_pop;
   Population *mixed_pop;

   int ngen;
   int popsize;
   int nbinmut;
   int nrealmut;
   int nbincross;
   int nrealcross;
   vector<int> nbits;
   vector<pair<double,double> > range_realvar;
   vector<pair<double,double> > range_binvar;
   int bitlength;


   /* Atributos del MOP */
   int nreal;
   int nbin;
   int nobj;
   int ncon;

   /* Parámetros operadores */
   double pcross_real;
   double pcross_bin;
   double pmut_real;
   double pmut_bin;
   double eta_c;
   double eta_m;

   vector<int> entireObjSet;

   bool rndPopulation;
   vector<vector<double> > initialPop;


   NSGA(ProblemFunction *mop, Randomizer *r, string outputDir, NSGAParams &np);

   ~NSGA();

   void startWithThisPopulation(vector<vector<double> > const &initPop);
   void startWithRandomPopulation();

   void archivePopulation(Population *pop);
   string rangesToStr(int digitsPrec = 5, const char *delimiter = ", ");

   void createInitialPopulation();
   void optimize();

   void openInfoFiles();
   void closeInfoFiles();

   void nGenerations(int n);

   void allocate_memory();
   void allocate_memory_pop (Population *pop, int size);
   void allocate_memory_ind (Individual *ind);
   void deallocate_memory_pop (Population *pop, int size);
   void deallocate_memory_ind (Individual *ind);

   void crossover (Individual *parent1, Individual *parent2, Individual *child1, Individual *child2);
   void realcross (Individual *parent1, Individual *parent2, Individual *child1, Individual *child2);
   void bincross (Individual *parent1, Individual *parent2, Individual *child1, Individual *child2);

   void assign_crowding_distance_list(Population *pop, myList *lst, int front_size);
   void assign_crowding_distance_list(Population *pop, myList *lst, int front_size, vector<int> &objSet);
   void assign_crowding_distance_indices (Population *pop, int c1, int c2); // <------
   void assign_crowding_distance_indices (Population *pop, int c1, int c2, vector<int> &objSet);
   void assign_crowding_distance (Population *pop, vector<int> &dist, vector<vector<int> >  &obj_array,
                                  vector<int> &objSet, int front_size);
   void assign_crowding_distance (Population *pop, int *dist, int **obj_array, int front_size); // <----

   void decode_pop (Population *pop);
   void decode_ind (Individual *ind);

   int check_dominance (Individual *a, Individual *b);
   int check_dominance (Individual *a, Individual *b, vector<int> &objSet);

   void evaluate_pop(Population *pop);
//   void evaluate_ind(Individual *ind);
   void evaluateIndividual(int i);

   void fill_nondominated_sort (Population *mixed_pop, Population *new_pop, vector<int> &objSet);
   void fill_nondominated_sort (Population *mixed_pop, Population *new_pop);
   void crowding_fill(Population *mixed_pop, Population *new_pop, int count, int front_size,
                      myList *cur, vector<int> &objSet);
   void crowding_fill(Population *mixed_pop, Population *new_pop, int count, int front_size,
   		             myList *elite); //<----

   void initialize_pop (Population *pop);
   void initialize_pop_fix(Population *pop);
   void initialize_ind (Individual *ind);

   void insert(myList *node, int x);
   myList* del(myList *node);

   void merge(Population *pop1, Population *pop2, Population *pop3);
   void copy_ind (Individual *ind1, Individual *ind2);

   void mutation_pop (Population *pop);
   void mutation_ind (Individual *ind);
   void bin_mutate_ind (Individual *ind);
   void real_mutate_ind (Individual *ind);

   void test_problem (double *xreal, double *xbin, int **gene, double *obj, double *constr);
   void assign_rank_and_crowding_distance (Population *new_pop);

   void quicksort_front_obj(Population *pop, int objcount, int obj_array[], int obj_array_size);
   void q_sort_front_obj(Population *pop, int objcount, int obj_array[], int left, int right);
   void quicksort_front_obj(Population *pop, int objcount, vector<int> &obj_array, int obj_array_size);
   void q_sort_front_obj(Population *pop, int objcount, vector<int> &obj_array, int left, int right);

   void quicksort_dist(Population *pop, vector<int> &dist, int front_size);
   void q_sort_dist(Population *pop, vector<int> &dist, int left, int right);
   void quicksort_dist(Population *pop, int *dist, int front_size);
   void q_sort_dist(Population *pop, int *dist, int left, int right);

   void selection (Population *old_pop, Population *new_pop);
   Individual* tournament (Individual *ind1, Individual *ind2);

   void write_feasible(ostream &output) const;
   void write_whole_pop(ostream &output) const;

   void report_feasible(Population *pop, ostream &output) const;
   void report_objSpace(Population *pop, ostream &output) const;
   void report_varSpace(Population *pop, ostream &output) const;
   void report_pop_2files(Population *pop, int gen) const;
   void report_pop(Population *pop, ostream &output) const;
   void report_params(ostream &output) const;
   void report_pop_objs(Population *pop, ostream &output) const;
};

#endif /* NSGA_H_ */
