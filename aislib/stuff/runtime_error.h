// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef RUNTIME_ERROR_H
#define RUNTIME_ERROR_H

#include <exception>
#include <string>

/**
 * \brief a run time error exception
 */
class RuntimeError : public std::exception
{
  public:
    /**
     * constructor which allows to give a error message
     * with printf like syntax
     */
    explicit RuntimeError(const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
    virtual ~RuntimeError() throw();
    virtual const char* what() const throw() {return _errorMsg.c_str();}

  protected:
    std::string _errorMsg;
};

#endif
