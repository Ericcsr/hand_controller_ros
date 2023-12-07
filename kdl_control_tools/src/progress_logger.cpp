// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "kdl_control_tools/progress_logger.h"

#include <iostream>
#include <fstream>

using namespace kdl_control_tools;

/*********************************************************************
* Track progresses of multiple variables asynchronously, write as a csv file.
*********************************************************************/
ProgressLogger::ProgressLogger()
{

}
/*********************************************************************
* Destructor
*********************************************************************/
ProgressLogger::~ProgressLogger()
{

}
/*********************************************************************
* Comment
*********************************************************************/
void ProgressLogger::log(const string var_name, const double new_value)
{
  // Check existence
  if(!checkExists_(var_name)){
    // if not create empty
    progress_map_[var_name] = vector<double>();
  }

  // add the new_value
  progress_map_[var_name].push_back(new_value);

}
/*********************************************************************
* Comment
*********************************************************************/
void ProgressLogger::logTimed(const string var_name, const double new_value, const double t)
{
  // Check existence
  if(!checkExists_(var_name)){
    // if not create empty
    progress_map_[var_name] = vector<double>();
    progress_map_[var_name+"_time"] = vector<double>();
  }

  // add the new_value with time
  progress_map_[var_name].push_back(new_value);
  progress_map_[var_name+"_time"].push_back(t);

}
/*********************************************************************
* Comment
*********************************************************************/
void ProgressLogger::writeVarToFile(const string var_name, const string filename_suffix)
{
  if(!checkExists_(var_name)){
    ROS_WARN("ProgressLogger: No logs for %s.", var_name.c_str());
    return;
  }
  // is there a time record for this var?
  bool timed = checkExists_(var_name+"_time");

  string filename = var_name+filename_suffix;
  ofstream myfile (filename);
  if (myfile.is_open())
  {
    // labels row
    myfile << "time," << var_name << ",\n";
    // data rows
    int time_length = progress_map_[var_name].size();
    for(int ti=0; ti<time_length; ti++){
      // get time value, if timed, otherwise use index
      double time_val = ti;
      if(timed) time_val = progress_map_[var_name+"_time"][ti];

      myfile << time_val << ", " << progress_map_[var_name][ti] << ",\n";
    }
    myfile.close();

    ROS_INFO("ProgressLogger: Written %s logs to file.", var_name.c_str());
  }
  else ROS_WARN("ProgressLogger: Unable to open file");
}
/*********************************************************************
* Comment
*********************************************************************/
void ProgressLogger::writeAllToFile(const string filename)
{
  ofstream myfile (filename);

  if (!myfile.is_open()){
    ROS_WARN("ProgressLogger: Unable to open file");
    return;
  }

  // labels row
  myfile << "time,";
  for (auto const& p : progress_map_)
      myfile << p.first << ",";
  myfile << "\n";

  // data rows
  // find max time length
  int time_length = 0;
  for (auto const& p : progress_map_)
      if((int)p.second.size() > time_length)
        time_length = p.second.size();

  // loop for time
  for(int ti=0; ti<time_length; ti++){
    // time column
    myfile << ti << ",";
    // var columns
    for (auto const& p : progress_map_)
    {
      if((int)p.second.size() > ti)
        myfile << p.second[ti] << ",";
    }
    myfile << "\n";
  }
  myfile.close();

  ROS_INFO("ProgressLogger: Written all logs to file.");
}
/*********************************************************************
* Check if the var_name is already in map
*********************************************************************/
bool ProgressLogger::checkExists_(const string var_name)
{
  ProgressMap::iterator it;
  it = progress_map_.find(var_name);

  return (it != progress_map_.end());
}
/*********************************************************************
* Getters and setters
*********************************************************************/
vector<double> ProgressLogger::getProgress(const string var_name){
  if (!checkExists_(var_name)) {
    ROS_WARN("ProgressLogger: No logs for %s.", var_name.c_str());
    return vector<double>(); // return empty vector
  }
  return progress_map_[var_name];
}
int ProgressLogger::getProgressLength(const string var_name){
  if (!checkExists_(var_name)) {
    // ROS_WARN("ProgressLogger: No logs for %s.", var_name.c_str());
    return 0;
  }
  return progress_map_[var_name].size();
}
double ProgressLogger::getLog(const string var_name, const int index){
  // non existing index
  if(index < 0 || index >= progress_map_[var_name].size())
    return 0;

  return progress_map_[var_name][index];
}
