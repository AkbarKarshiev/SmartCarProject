#include "Object.h"
#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

Object::Object(string name, string color, Point position){
	this->name = name;
	this->color = color;
	this->position = position;
}

string Object::getName(){
	return name;
}

string Object::getColor(){
	return color;
}

Point Object::getPosition(){
	return position;
}