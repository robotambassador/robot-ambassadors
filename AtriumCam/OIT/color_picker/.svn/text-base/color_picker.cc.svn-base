#include "ros/ros.h"
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/highgui.h"

#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"

#include <unistd.h>

IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
CvHistogram *hist = 0;
sensor_msgs::CvBridge img_bridge;
int backproject_mode = 0;
int select_object = 0;
int color_selected = 0;
int show_hist = 1;
CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 30;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 10, vmax = 256, smin = 30;

std::string prefix;
void image_cb( const sensor_msgs::ImageConstPtr& image_msg );


void process_key( int key )
{
  FILE* hist_out;
  char name[1024];
  name[0] = 0;
  //prefix[0] = 0;
  time_t t = ::time(NULL);
  struct tm *tms = localtime(&t);


  switch( (char) key )
  {
    case 'b':
      backproject_mode ^= 1;
      break;
    case 'c':
      color_selected = 0;
      cvZero( histimg );
      break;
    case 'h':
       show_hist ^= 1;
       if( !show_hist )
           cvDestroyWindow( "Histogram" );
       else
           cvNamedWindow( "Histogram", 1 );
       break;
    case 's':
      if( prefix != "" )
      {
        snprintf(name, sizeof(name), "%s/%d-%02d-%02d-%02d-%02d-%02d-dump.txt",
                 prefix.c_str(),
                 tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                 tms->tm_hour     , tms->tm_min  , tms->tm_sec);
      }
      else 
      {
        snprintf(name, sizeof(name), "%d-%02d-%02d-%02d-%02d-%02d-dump.txt",
                 tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                 tms->tm_hour     , tms->tm_min  , tms->tm_sec);
      }
      ROS_INFO( "saving color: [%s]", name );
      hist_out = fopen( name, "w" );
      fprintf( hist_out, "%d\n%d\n%d\n%d\n", hdims, smin, vmin, vmax );
      for( int i = 0; i < hdims; i++ )
        fprintf( hist_out, "%f\n", cvGetReal1D(hist->bins,i));
      fclose( hist_out );
      break;
    default:
       ;
  }
}

void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);
        
        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, image->width );
        selection.height = MIN( selection.height, image->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            color_selected = -1;
        break;
    }
}


CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

int main( int argc, char** argv )
{
	ros::init(argc,argv,"color_picker");
	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
  n_priv.param( "prefix", prefix, std::string(""));
	ros::Rate loop_rate(10);
	ros::Subscriber image_sub = n.subscribe("image",1,image_cb);
  printf( "Hot keys: \n"
      "\tESC - quit the program\n"
      "\tc - stop the tracking\n"
      "\tb - switch to/from backprojection view\n"
      "\th - show/hide object histogram\n"
      "To initialize tracking, select the object with mouse\n" );

	/* create windows */
  cvNamedWindow( "Histogram", 1 );
  cvNamedWindow( "CamShiftDemo", 1 );
  cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
  cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
  cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
  cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );
            

	while( ros::ok() )
	{

		if( image )
		{
			int c = cvWaitKey(10);
			process_key(c);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}	


void image_cb( const sensor_msgs::ImageConstPtr& img_msg )
{
  int i, bin_w;

	/* get frame */
	if( img_bridge.fromImage(*img_msg, "bgr8" ) )
	{
		IplImage* frame = img_bridge.toIpl();

		if( !image )
		{
			/* allocate all the buffers */
		  image = cvCreateImage( cvGetSize(frame), 8, 3 );
		  image->origin = frame->origin;
		  hsv	= cvCreateImage( cvGetSize(frame), 8, 3 );
		  hue = cvCreateImage( cvGetSize(frame), 8, 1 );
		  mask = cvCreateImage( cvGetSize(frame), 8, 1 );
		  backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		  hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
		  histimg = cvCreateImage( cvSize(320,200), 8, 3 );
		  cvZero( histimg );
		}

    cvCopy( frame, image, 0 );
    cvCvtColor( image, hsv, CV_BGR2HSV );

    if( color_selected )
    {
	    int _vmin = vmin, _vmax = vmax;
      cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
                  cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
      cvSplit( hsv, hue, 0, 0, 0 );
      if( color_selected < 0 )
      {    
				float max_val = 0.f;
        cvSetImageROI( hue, selection );
        cvSetImageROI( mask, selection );
        cvCalcHist( &hue, hist, 0, mask );
        cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
        cvConvertScale(hist->bins,hist->bins,max_val ? 255. / max_val : 0.,0);
        cvResetImageROI( hue );
        cvResetImageROI( mask );
        track_window = selection;
        color_selected = 1;

        cvZero( histimg );
        bin_w = histimg->width / hdims;
        for( i = 0; i < hdims; i++ )
        {
 	      	int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
          CvScalar color = hsv2rgb(i*180.f/hdims);
          cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                       cvPoint((i+1)*bin_w,histimg->height - val),
                       color, -1, 8, 0 );
        }
				printf( "hist dump\n" );
      }

      cvCalcBackProject( &hue, backproject, hist );
      cvAnd( backproject, mask, backproject, 0 );
      if( backproject_mode )
      	cvCvtColor( backproject, image, CV_GRAY2BGR );
      cvCamShift( backproject, track_window,
      						cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
                  &track_comp, &track_box );
      track_window = track_comp.rect;
      if( !image->origin )
        track_box.angle = -track_box.angle;
      //printf( "(%f,%f) %f:%f\n", track_box.center.x, track_box.center.y, track_box.size.width, track_box.size.height );
      cvEllipseBox( image, track_box, CV_RGB(255,0,0), 1, CV_AA, 0 );

		}
        
    if( select_object && selection.width > 0 && selection.height > 0 )
    {
    	cvSetImageROI( image, selection );
      cvXorS( image, cvScalarAll(255), image, 0 );
      cvResetImageROI( image );
    }
		cvShowImage( "CamShiftDemo", image );
  	cvShowImage( "Histogram", histimg );
	}	
}

