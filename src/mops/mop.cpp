#include "mop.h"
#include <iostream>

using std::string;

ProblemFunction::ProblemFunction(const char *name, int numVariables, int numObjectives, int numConstraints)
{
   this->name.assign(name);
   this->nObjectives = numObjectives;
   this->nVariables = numVariables;
   this->nConstraints = numConstraints;
   isScalableObjSpc = false;
   isScalableVarSpc = false;

   // These are some default values, each concrete MOP should set appropriate values.
   xRanges.assign(numVariables, make_pair(0.0, 1.0));
}

ProblemFunction::~ProblemFunction() {}

//void MOP::evaluate(int const *gene, double *fx, double *gcons) const
//{
//   vector<int> geneCopy(gene, gene + nVariables);
//   vector<double> fxCopy(fx, fx + nObjectives);
//   vector<double> gconsCopy(gcons, gcons + nConstraints);
//
//   evaluate(geneCopy, fxCopy, gconsCopy);
//
//   copy(fxCopy.begin(), fxCopy.end(), fx);
//   copy(gconsCopy.begin(), gconsCopy.end(), gcons);
//}

void ProblemFunction::setNumObjectives(int numObjectives) {
   if (isScalableObjSpc)
      this->nObjectives = numObjectives;
}

void ProblemFunction::setNumVariables(int numVariables) {
   if (isScalableVarSpc)
      this->nVariables = numVariables;
}

void ProblemFunction::setVarsRange(vector<pair<double,double> > &xRanges) {
   this->xRanges = xRanges;
}

const bool ProblemFunction::isScalableObjSpace() const{
   return isScalableObjSpc;
}

const bool ProblemFunction::isScalableVarSpace() const{
   return isScalableVarSpc;
}

const int ProblemFunction::getNumVariables() const {
   return nVariables;
}

vector<pair<double,double> > ProblemFunction::getVarsRange() const {
   return xRanges;
}

const int ProblemFunction::getNumObjectives() const {
   return nObjectives;
}

const int ProblemFunction::getNumConstraints() const {
   return nConstraints;
}

string ProblemFunction::getName() const {
   return name;
}
