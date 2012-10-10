#include "gmm.h"

#include <stdio.h>
#include <ros/ros.h>

GMM::GMM()
{
  params_.covs      = NULL;
  params_.means     = NULL;
  params_.weights   = NULL;
  params_.probs     = NULL;
  //params_.nclusters = N;
  params_.cov_mat_type       = CvEM::COV_MAT_GENERIC;
  params_.start_step         = CvEM::START_AUTO_STEP;
  params_.term_crit.max_iter = 10000;
  params_.term_crit.epsilon  = 0.01;
  params_.term_crit.type     = CV_TERMCRIT_ITER |CV_TERMCRIT_EPS;
}

GMM::~GMM()
{
}

bool GMM::load_data_from_file( std::string filename, std::string prefix )
{
	std::vector<int> sizes;

	/* read data from file */
	read_features_from_file( prefix + "/" + filename );
	for( unsigned i = 0; i < files_.size(); i++ )
	{
		// read size
		std::string fname = prefix + "/" + files_[i];
		ROS_INFO ("file: [%s]", fname.c_str() );
		std::string cmd = std::string("wc -l ") + fname;
		FILE* wcout = popen( cmd.c_str(), "r" );
		int num = 0;
		fscanf( wcout, "%d", &num );
		pclose( wcout );
		sizes.push_back( num );
	}

	nsamples_ = 0;
	for( unsigned i = 0; i < sizes.size(); i++ )
		nsamples_ += sizes[i];

	samples_ = cvCreateMat( nsamples_, num_dimensions_, CV_32FC1 );
	int pfx = 0;

	for( unsigned int f = 0; f < files_.size(); f++ )
	{
		FILE* train_file;
		std::string fname = prefix + "/" + files_[f];
		train_file = fopen( fname.c_str(), "r" );
		if( !train_file )
		{
			fprintf( stderr, "couldn't open file: [%s]\n", files_[f].c_str() );
			return false;
		}

		for( int i = 0; i < sizes[f]; i++ )
		{
			for( int j = 0; j < num_dimensions_; j++ )
			{
				float x = 0;
				fscanf( train_file, "%f ", &x );
				((float*)(samples_->data.ptr+samples_->step*(i+pfx)))[j] = x;
			}
			fscanf( train_file, "\n" );
		}

		fclose( train_file );
		pfx += sizes[f];
	}

	pfx = 0;

	// properly normalize data

	for( unsigned int f = 0; f < files_.size(); f++ )
	{
		std::vector<float> norms;
		for( unsigned i = 0; i < features_.size(); i++ )
		{
			if( features_[i].normalized == TYPE_LAST )
				norms.push_back(((float*)(samples_->data.ptr+samples_->step*(sizes[f]+pfx-1)))[features_[i].id]);
			else if( features_[i].normalized == TYPE_FIRST )
				norms.push_back(((float*)(samples_->data.ptr+samples_->step*(pfx)))[features_[i].id]);
			else norms.push_back(1.0);
		}

		for( int i = 0; i < sizes[f]; i++ )
		{
			for( unsigned j = 0; j < features_.size(); j++ )
			{
				float t = ((float*)(samples_->data.ptr+samples_->step*(i+pfx)))[features_[j].id];
				t /= norms[j];
				((float*)(samples_->data.ptr+samples_->step*(i+pfx)))[features_[j].id] = t;
				//printf( "%4.6f ", t );
			}
			//printf( "\n" );
		}
		pfx += sizes[f];
	}

	

	return true;
}

bool GMM::train_model( int num_clusters, std::vector<int> *dimensions )
{
	int N = nsamples_;
	int numD = dimensions->size();
	CvMat* trainData = cvCreateMat( N, numD, CV_32FC1 );
	CvMat* labels = cvCreateMat( N, 1, CV_32SC1 );
	for( int d = 0; d < numD; d++ )
	{
		int nd = (*dimensions)[d];
		for( int i = 0; i < N; i++ )
		{
			//printf( "i: %d d: %d ==> nd: %d\n", i, d, nd );
			float x = samples_->data.fl[i*num_dimensions_+nd]; 
			trainData->data.fl[i*numD+d] = x;
			//float x = ((float*)(samples_->data.ptr+samples_->step*i))[nd]; 
			//((float*)(trainData->data.ptr+trainData->step*i))[d] = x;
		}
	}
	
	params_.nclusters = num_clusters;
	num_trained_dims_ = numD;
	em_model_.train(trainData, 0, params_, labels );

	std::vector<CvScalar> colors;
	colors.push_back( CV_RGB( 0, 0, 255 ) );
	colors.push_back( CV_RGB( 0, 255, 0 ) );
	colors.push_back( CV_RGB( 255, 0, 0 ) );
	colors.push_back( CV_RGB( 255, 255, 0 ) );
	colors.push_back( CV_RGB( 255, 0, 255 ) );
	colors.push_back( CV_RGB( 0, 255, 255 ) );
	colors.push_back( CV_RGB( 255, 255, 255 ) );
	colors.push_back( CV_RGB( 128, 0, 0 ) );
	colors.push_back( CV_RGB( 0, 128, 0 ) );
	colors.push_back( CV_RGB( 0, 0, 128 ) );

	for( int d = 0; d < numD; d++ )
	{
		for( int dd = d+1; dd < numD; dd++ )
		{
			char windowName[256];
			windowName[0] = '\0';
			sprintf( windowName, "%d / %d", d, dd );

			//cvNamedWindow( windowName, 1 );
			IplImage * displayImage = cvCreateImage( cvSize(256, 256), 8, 3 );

			for( int i = 0; i < N; i++ )
			{
				float x, y;


				x = trainData->data.fl[i*numD+d]; 
				y = trainData->data.fl[i*numD+dd];
				//x = ((float*)(samples_->data.ptr+samples_->step*i))[d]; 
				//y = ((float*)(samples_->data.ptr+samples_->step*i))[dd];

				//printf( "%0.2f, %0.2f ", x, y );				

				x *= 256.;
				if( dd != 3 ) y *= 256.;
				else y *= 64.;

				//printf( "(%0.2f, %0.2f) (%d,%d)\n", x, y, (int)x-1, int(y)-1 );

				CvPoint p1 = cvPoint( (int) x -1, 256- (int) y -1 );
				CvPoint p2 = cvPoint( (int) x +1, 256- (int) y +1 );

				cvRectangle( displayImage, p1, p2, colors[labels->data.i[i]], 1 );
			}

			//cvShowImage( windowName, displayImage );
		}
	}

	return true;
}

double
GMM::prob( std::vector<double> *vec )
{
	std::vector<double> data = *vec;
	//printf("\n----------------------------------------\n\n" );

	for( unsigned i = 0; i < data.size(); i++ )
	{
		//printf( "%0.6f ", data[i] );
	}
	//printf("==> " );
	cv::Mat uu;
	em_model_.getMeans().copyTo(uu);
	cv::Mat w(1, num_trained_dims_, CV_64FC1);

	for( int i = 0; i < num_trained_dims_; i++ )
	{
		w.at<double>(0,i) = data[i];
	}

	std::vector<cv::Mat> cvcovs;
	em_model_.getCovs(cvcovs);
	std::vector<double> mdists;

	double min = DBL_MAX;
	double ret = 0.0;
	double norm = 0.0;

	for( int i = 0; i < params_.nclusters; i++ )
	{
		//printf( "\n\n======================================\n" );
		// isolate a vector for each cluster's mean
		cv::Mat u;//(1, num_trained_dims_, CV_64F );
		u = em_model_.getMeans().row(i);//.t();
		cv::Mat wu = w-u;
		//printf( "w(%d,%d) u(%d,%d) wu(%d,%d)\n", w.rows, w.cols, u.rows, u.cols, wu.rows, wu.cols );
		cv::Mat sigma = cvcovs[i];
		cv::Mat wut = wu.t();

		for( int j = 0; j < w.cols; j++ )
		{
			//printf( "%0.6f %0.6f %0.6f\n", w.at<double>(0,j), u.at<double>(0,j), wu.at<double>(0,j) );
		}

	  //printf( "=== %d %d (%d) | %d %d (%d) | %d %d (%d) ===\n", wu.rows, w.cols, wu.type(), wut.rows, wut.cols, wut.type(), sigma.rows, sigma.cols, sigma.type() );
		//printf( "===%d,%d %d,%d\n===", w.rows, w.cols, u.rows, u.cols );

		cv::Mat siginv = sigma.inv(cv::DECOMP_SVD);
		cv::Mat iterm =  wu * siginv * wut;
		double power = pow(2*M_PI,(num_trained_dims_/2.)) * sqrt(fabs(cv::determinant(sigma)));
		double leading_term = 1. / power;
		double prob = 1.0;
		double mahal = sqrt(iterm.at<double>(0,0));
		double dt = mahal / 20.;

/*
		for( double t = 0; t <= mahal; t += dt )
		{
			prob = prob - (leading_term * exp(-0.5*t*t) * dt);
			printf( "t: %0.4f mahal: %0.4f leading_term: %0.4f prob: %0.4f pow: %0.4f\n", t, mahal, leading_term, prob, power );
		}
*/
		prob = leading_term * exp( -0.5 * mahal );


		mdists.push_back( mahal );
		//printf( "%0.2f/%0.4f ", mahal, prob );
		if( mdists[i] < min )
		{
			min = mdists[i];
			ret = prob;
			norm = leading_term;
		}
	}

	//printf( "==> %0.2f\n", ret / norm );
	//printf("\n----------------------------------------\n\n" );
	return ret / norm;
}
