#include "imagebuffer.h"
#include <QDebug>
#include <QTime>

ImageBuffer::ImageBuffer(int size) : bufferSize(size) {}

/* Add a frame to the buffer.  
 * The data in the frame is copied to the internal image buffer.  
 * If there are no images in the buffer, the clients of the wait condition are woken to notify them of a new frame.
 * If the buffer is full, the function blocks until it there is space
 * @param image The image to add
 */
void ImageBuffer::addFrame(const IplImage* image) {
	//QTime time;
	//time.start();
	if(!image) {
		qDebug() << "E: Imagebuffer received a null image";
		return;
	}
	//qDebug() << "Adding a frame";
	mutex.lock();
	if(imageQueue.size() == bufferSize) {
		bufferNotFull.wait(&mutex);
	}
	mutex.unlock();
	
	// copy the image
	IplImage* temp = cvCloneImage(image);
	imageQueue.enqueue(temp);
	//qDebug() << "AF" << time.elapsed();
	mutex.lock();
	bufferNotEmpty.wakeAll();
	mutex.unlock();
}

/* Return the oldest frame from the buffer.  
 * This is a blocking operation, so if there are no available images, this function will block until one is available or image acquisition is stopped.
  * @returns IplImage pointer to the next available frame.  Ownership of the data is passed to the callee, so the image must be released
 */
IplImage* ImageBuffer::getFrame() {
	mutex.lock();
	if(imageQueue.isEmpty()) {
	//	qDebug() << "Get frame waiting on frame";
		bufferNotEmpty.wait(&mutex);
	//	qDebug() << "Get frame has been received";
	}
	mutex.unlock();

	IplImage* temp = 0;

	mutex.lock();
	if(!imageQueue.isEmpty()) {
		temp = imageQueue.dequeue();
	}
	bufferNotFull.wakeAll();
	mutex.unlock();
	return temp;
}

/* Clear the buffer and release clients */
void ImageBuffer::clear() {
	imageQueue.clear();
	bufferNotEmpty.wakeAll();
}
