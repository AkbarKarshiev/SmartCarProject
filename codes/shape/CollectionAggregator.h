#pragma once
#ifndef COLLECTIONAGGREGATOR_H
#define COLLECTIONAGGREGATOR_H

#include "opencv2/opencv.hpp" 
#include "Object.h"
#include <list>

using namespace cv;
using namespace std;

class CollectionAggregator{

public:
	CollectionAggregator();
	~CollectionAggregator();
	void append(std::list<Object> objects);
	std::list<Object> retrieve();
	void setNewCycle();
private:
	std::list<Object> objects;
	bool newCycle;
	bool isDuplicate(Mat img, std::list<Object> objects);
};


#endif