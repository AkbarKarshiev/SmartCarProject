#include "CollectionAggregator.h"
#include "Object.h"
#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

CollectionAggregator::CollectionAggregator()
	:newCycle(false)
{}

CollectionAggregator::~CollectionAggregator()
{};



void CollectionAggregator::append(std::list<Object> incoming) {

	if (newCycle) {
		newCycle = false;
		objects.clear();
	}

	objects.splice(objects.end(), incoming);
}

std::list<Object> CollectionAggregator::retrieve() {
	return objects;
}

void CollectionAggregator::setNewCycle(){
	newCycle = true;
}

bool CollectionAggregator::isDuplicate(cv::Mat img, std::list<Object> objects) {
	//allocate iterator
	list<Object>::iterator iter = objects.begin();
	while (iter != objects.end()) {
		Object& temp = *iter;
	}

	return false;
}