/****************************************************************************
**
** Copyright (C) 2018 TU Wien, ACIN, Vision 4 Robotics (V4R) group
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

#include <iostream>

#include <glog/logging.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

class ProgramOptionsBase {
 public:
  virtual ~ProgramOptionsBase() {}

  bool parse(int argc, const char** argv) {
    google::InitGoogleLogging(argv[0]);
    po::options_description general("Options");
    std::vector<po::options_description> other;
    po::options_description hidden;
    po::positional_options_description positional;
    general.add_options()("help,h", "Print this help message");
    addGeneral(general);
    addOther(other);
    addPositional(hidden, positional);
    po::options_description all;
    all.add(general).add(hidden);
    for (const auto& opt : other)
      all.add(opt);

    try {
      auto parser = po::command_line_parser(argc, argv).options(all).positional(positional);
      po::store(parser.run(), vm);
      if (vm.count("help")) {
        std::cout << "Usage: " << argv[0] << " [options] ";
        auto num_positional = static_cast<int>(positional.max_total_count());
        if (num_positional == -1)
          std::cout << "<" << positional.name_for_position(0) << "> ";
        else if (num_positional > 0)
          for (int i = 0; i < num_positional; ++i)
            std::cout << "<" << positional.name_for_position(i) << "> ";
        std::cout << "\n\n";
        printDescription();
        std::cout << "\n";
        std::cout << general << std::endl;
        for (const auto& opt : other)
          std::cout << opt << std::endl;
        return false;
      }
      po::notify(vm);
      validate();
    } catch (boost::program_options::error& e) {
      std::cerr << "Option parsing error: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  /// Check if a given option was parsed.
  bool operator()(const std::string& name) const {
    return vm.count(name) != 0;
  }

  /// Access parsed option by name.
  const po::variable_value& operator[](const std::string& name) const {
    return vm[name];
  }

 protected:
  virtual void addGeneral(po::options_description& /* desc */) {}

  virtual void addOther(std::vector<po::options_description>& /* descs */) {}

  virtual void addPositional(po::options_description& /* desc */, po::positional_options_description& /* pos */) {}

  /// Print program description.
  /// This function is called if the user requested --help.
  virtual void printDescription() {}

  /// Validate supplied options.
  /// This function is called directly after parsing command line.
  /// Should throw boost::program_options::error if validation fails.
  virtual void validate() {}

  po::variables_map vm;
};
