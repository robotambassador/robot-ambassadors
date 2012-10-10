#ifndef GMM_H_
#define GMM_H_

#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>


#define TYPE_NONE   0
#define TYPE_FIRST  1
#define TYPE_LAST		2
#define TYPE_AVG    3
#define TYPE_MAX    4
#define TYPE_MIN    5
#define TYPE_VALUE  6

struct Feature {
  unsigned int  id;
  unsigned int  normalized;
};


class DaveEM : public CvEM {
  public:
  double get_likelihood() { return log_likelihood; }

};


class GMM {
	public:
		GMM();
		~GMM();

	public:

		bool read_features_from_file( std::string filename );
		bool load_data_from_file( std::string filename, std::string prefix );
		bool train_model( int num_clusters, std::vector<int> *dimensions );

		double prob( std::vector<double> *vec );


	protected:
	  CvEMParams params_;
		CvMat* samples_;

	  DaveEM em_model_;

		int num_dimensions_;
		int num_trained_dims_;
		int nsamples_;

		std::vector<Feature> features_;
		std::vector<std::string> files_;
};

#endif
