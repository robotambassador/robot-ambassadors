#include "gmm.h"
#include <stdio.h>

int main( int argc, char* argv[] )
{

	GMM g;
	g.load_data_from_file( argv[1], "/home/dfseifer/diamondback-usc/stacks/interaction-ros-pkg/sandbox/oit_models" );

	std::vector<int> used_dims;
	used_dims.push_back(0);
	used_dims.push_back(9);
	used_dims.push_back(10);
	used_dims.push_back(11);

	g.train_model( 3, &used_dims );

	std::vector<double> data1, data2, data3, data4;

	data1.push_back( -0.133169 );
	data1.push_back( 1.191623 );
	data1.push_back( 1.608428 );
	data1.push_back( 1.891003 );

	g.prob( &data1 );

	data2.push_back( 0.9 );
	data2.push_back( 0.9 );
	data2.push_back( 0.9 );
	data2.push_back( 4.0 );
	
	g.prob( &data2 );
	
	data3.push_back( 0.9 );
	data3.push_back( 0.2 );
	data3.push_back( 0.2 );
	data3.push_back( 4.0 );
	
	g.prob( &data3 );

	data4.push_back( 0.7 );
	data4.push_back( 0.5 );
	data4.push_back( 1.0 );
	data4.push_back( 3.0 );
	
	g.prob( &data4 );

  double p1 = -0.72641;
  double p2 =  0.97636;
  double p3 = -1.1394;
  double p4 =  1.0673;

  //double t = -0.86466*req.data[1] + 1.0661;
  double x = 0.2;
  double t = p1 * x*x*x + p2 * x*x + p3 * x + p4;
	printf( "x: %0.4f t: %0.4f\n", x, t );


	char c = 0;

	while( c != 'q' )
	{
		c = cvWaitKey(10);
	}

	return 0;
}
