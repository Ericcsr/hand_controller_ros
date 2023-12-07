// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef PROGRESS_LOGGER_H_
#define PROGRESS_LOGGER_H_

#include <ros/ros.h>

using namespace std;

namespace kdl_control_tools{

/**
* Track progresses of multiple variables asynchronously, write to a csv file.
**/
class ProgressLogger{

  typedef map<string, vector<double> > ProgressMap;

protected:

  ProgressMap progress_map_;

  bool checkExists_(const string var_name);

public:

	ProgressLogger();
  ~ProgressLogger();

  void log(const string var_name, const double new_value);
  void logTimed(const string var_name, const double new_value, const double t);

  void writeVarToFile(const string var_name, const string filename_suffix);
  void writeAllToFile(const string filename);

  vector<double> getProgress(const string var_name);
  int getProgressLength(const string var_name);
  double getLog(const string var_name, const int index);
};
} // end namespace kdl_control_tools

#endif /* PROGRESS_LOGGER_H_*/
