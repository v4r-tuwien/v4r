/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

#pragma once
#include <v4r/core/macros.h>
#include <string>

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

namespace v4r {

/**
 * @brief Stopwatch implementation (based on PCL implementation but using GLOG for output)
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date March 2018
 */
/** \brief Simple stopwatch.
 * \ingroup common
 */
class V4R_EXPORTS StopWatch {
 public:
  /** \brief Constructor. */
  StopWatch() : start_time_(boost::posix_time::microsec_clock::local_time()) {}

  /** \brief Destructor. */
  virtual ~StopWatch() {}

  /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
  inline double getTime() {
    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
    return (static_cast<double>(((end_time - start_time_).total_milliseconds())));
  }

  /** \brief Retrieve the time in seconds spent since the last call to \a reset(). */
  inline double getTimeSeconds() {
    return (getTime() * 0.001);
  }

  /** \brief Reset the stopwatch to 0. */
  inline void reset() {
    start_time_ = boost::posix_time::microsec_clock::local_time();
  }

 protected:
  boost::posix_time::ptime start_time_;
};

/** \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create an instance at the beginning of the function. Example:
 *
 * \code
 * {
 *   v4r::ScopeTime t1 ("calculation");
 *
 *   // ... perform calculation here
 * }
 * \endcode
 *
 * \ingroup common
 */
class V4R_EXPORTS ScopeTime : public StopWatch {
 private:
  std::string title_;

 public:
  ScopeTime(const std::string& title = "") : StopWatch(), title_(title) {}

  ~ScopeTime();
};

}  // namespace v4r