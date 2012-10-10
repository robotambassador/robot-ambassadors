#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "gmm.h"

void operator >> (const YAML::Node& node, Feature& feature)
{
	node["id"] >> feature.id;
	std::string norm;
	node["normalized"] >> norm;

	if( norm == std::string( "none" ) ) feature.normalized = TYPE_NONE;
	else if( norm == std::string( "last" ) ) feature.normalized = TYPE_LAST;
	else if( norm == std::string( "first" ) ) feature.normalized = TYPE_FIRST;
	else 
	{
		ROS_WARN( "%s: not recognized as a normalization type, defaulting to none", norm.c_str() );
		feature.normalized = TYPE_NONE;
	}

	ROS_INFO( "feature: %u:%u", feature.id, feature.normalized );
}

bool GMM::read_features_from_file( std::string filename )
{
	std::ifstream fin;
	fin.open(filename.c_str());
	if( fin.fail() ) 
	{
		ROS_WARN("couldn't find file: [%s]", filename.c_str());
		return false;
	}

	YAML::Node doc;
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc);

	std::string modelname;
	doc[0]["model"] >> modelname;
	ROS_INFO( "model: [%s]", modelname.c_str() );

	doc[0]["total_features"] >> num_dimensions_;

	const YAML::Node& y_features = doc[0]["features"];
	const YAML::Node& y_files = doc[0]["files"];

	features_.clear();
	files_.clear();

	for( unsigned i = 0; i < y_features.size(); i++ )
	{
		Feature feature;
		y_features[i] >> feature;
		features_.push_back(feature);
	}

	for( unsigned i = 0; i < y_files.size(); i++ )
	{
		std::string fname;
		y_files[i]["filename"] >> fname;
		files_.push_back(fname);
		ROS_INFO( "adding %s to model", fname.c_str() );
	}

	return true;;
}
