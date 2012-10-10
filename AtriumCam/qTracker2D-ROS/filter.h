#ifndef FILTER_H
#define FILTER_H


#include "opencv/highgui.h"

/**
 * Base class for filters
 */
class Filter {
public:
	Filter();
	void setListener(Filter* list);
        virtual void processPoint(IplImage* image) = 0;
protected:
	Filter* listener;
        void notifyListener(IplImage* image);
};

#endif
