/*
 * Population.cpp
 *
 *  Created on: 11/11/2009
 *      Author: antonio
 */

#include <cassert>
#include "Population.h"
#include <omp.h>
#include <iostream>
#include <chrono>

using namespace std;

Population::Population(int size) {
   assert(size >= 4 && size % 4 == 0);

   this->size = size;
   ind = new Individual* [size];
   for (int i=0; i < size; ++i)
      ind[i] = new Individual();

   /*
   int i;
   pop->ind = (individual *) malloc(size*sizeof(individual));
   for (i=0; i<size; i++)
   {
       allocate_memory_ind (&(pop->ind[i]));
   }
   */
}

void Population::cloneIndInitialize(Individual *base) {
   nreal = base->xreal.size();
   nobj = base->obj.size();
   ncon = base->constr.size();
   nbin = base->xbin.size();
   nbits = base->nbits;
   range_realvar = base->range_realvar;
   range_binvar = base->range_binvar;

   for (int i = 0; i < size; ++i)
      ind[i]->cloneInitialize( *base );
}

void Population::cloneInitialize(Population &pop) {
   if (size != pop.size) {
      /* Delete current individuals. */
      for (int i = 0; i < size; ++i)
         ind[i]->~Individual();
      delete[] ind;

      size = pop.size;
      ind = new Individual* [size];
   }

   nreal = pop.nreal;
   nobj = pop.nobj;
   ncon = pop.ncon;
   nbin = pop.nbin;
   nbits = pop.nbits;
   range_realvar = pop.range_realvar;
   range_binvar = pop.range_binvar;

   for (int i = 0; i < size; ++i)
      ind[i]->cloneInitialize(*(pop.ind[i]));
}

void Population::copyPopopulation(Population &pop, int position) {
   /* is there enough space to copy all members of pop from the given position? */
   assert(size >= position + pop.size);

   /* Copy each individual. */
   for (int i = 0; i < pop.size; ++i)
      ind[i + position]->copyIndividual(*(pop.ind[i]));
}

void Population::rndInitialize(int nreal, int nobj, int ncon, int nbin, vector<int> &nbits,
      vector<pair<double,double> > &range_realvar,
      vector<pair<double,double> > &range_bivar, Randomizer *r)
{
   for (int i = 0; i < size; ++i)
      ind[i]->rndInitialize(nreal, nobj, ncon, nbin, nbits, range_realvar, range_bivar, r);
}

Population::~Population() {
   for (int i=0; i < size; ++i)
      delete ind[i];

   delete[] ind;
}

/** Return the size of the population */
int Population::getSize() {
   return size;
}

void Population::decode_pop() {
   if (ind[0]->xbin.size() != 0) /* If there are binary variables */
       for (int i=0; i < size; i++)
          ind[i]->decode();
}

void Population::evaluate_ind(ProblemFunction *mop, std::vector<int> index) {
   //  mop->evaluate(ind[index[0]]->xreal, ind[index[1]]->xreal, ind[index[2]]->xreal, ind[i]->gene[0], ind[i]->obj, ind[i]->constr);

   //  if (mop->getNumConstraints() == 0)
   //      ind[i]->constr_violation = 0.0;
   //  else
   //  {
   //      ind[i]->constr_violation = 0.0;
   //      for (int j=0; j < mop->getNumConstraints(); ++j)
   //          if ( ind[i]->constr[j] < 0.0 )
   //              ind[i]->constr_violation += ind[i]->constr[j];
   //  }
}

void Population::evaluate_pop(ProblemFunction *mop) {

//   chrono::time_point<chrono::system_clock> start, end;

   //start = std::chrono::system_clock::now();

//   #pragma omp parallel for num_threads(28)
      // mop->evaluate(ind[0]->xreal, ind[0]->gene[0], ind[0]->obj, ind[0]->constr);

//   end = std::chrono::system_clock::now();

//   chrono::duration<double> elapsed_seconds = end-start;
//   time_t end_time = chrono::system_clock::to_time_t(end);

//   cout << "finished computation at " << ctime(&end_time)
//             << "elapsed time: " << elapsed_seconds.count() << "s\n";

}
