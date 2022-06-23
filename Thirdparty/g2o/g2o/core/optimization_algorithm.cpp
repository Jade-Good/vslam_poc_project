// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "optimization_algorithm.h"

using namespace std;

namespace g2o
{
OptimizationAlgorithm::OptimizationAlgorithm() : _optimizer(0) {}

OptimizationAlgorithm::~OptimizationAlgorithm() {}

void OptimizationAlgorithm::printProperties(std::ostream& os) const
{
  os << "------------- Algorithm Properties -------------" << endl;
  for (PropertyMap::const_iterator it = _properties.begin();
       it != _properties.end();
       ++it)
  {
    BaseProperty* p = it->second;
    os << it->first << "\t" << p->toString() << endl;
  }
  os << "------------------------------------------------" << endl;
}

bool OptimizationAlgorithm::updatePropertiesFromString(
  const std::string& propString)
{
  return _properties.updateMapFromString(propString);
}

void OptimizationAlgorithm::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

}  // namespace g2o
