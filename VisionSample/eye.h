#if defined(_MSC_VER)
#pragma once
#endif

#ifndef HUMAN_EYE_H
#define HUMAN_EYE_H

#include "pbrt.h"
#include "geometry.h"

class Lens {
public:
	Lens(double rad, double asph, double zpos, double refr, double aper);
	bool RefractRay(const Ray &inRay, Ray &outRay) const;
	Vector GetNormal(const Ray * ray, const Point & p) const;
	bool OnLens(Point p) const;
	double yToZ(double);
    
	double radius;
	double asphericity;
	double zPos;
	double refractionRatio;
	double aperture;
};

class HumanEye{
public:
   HumanEye();
   ~HumanEye();
   double GenerateRay(double focus, double r, double pupil, double theta, double phi, char *dirName) const;
   void ParseSpecfile(const string& specfile);
   bool TraceLenses(const Ray &inRay, Ray &outRay, int start, int end) const;
   bool TraceLenses(const Ray &inRay, Ray &outRay, int start, int end, vector<Point>& points) const;
   Point getRandomPointOnLens() const;
   bool SetFocalLength(double focus);
   void SetDP(double);
   void MoveUp()
   {
	   phi += 5;
	   testPoint.y = 500 * sin(phi * M_PI / 180);
	   testPoint.z = 500 * cos(phi * M_PI / 180);
   }
   void MoveDown()
   {
	   phi -= 5;
	   testPoint.y = 500 * sin(phi * M_PI / 180);
	   testPoint.z = 500 * cos(phi * M_PI / 180);
   }
   void MoveLeft() {testPoint.z += 10;}//printf("zpos: %f\n", testZPos + imagePlane);}
   void MoveRight() {testPoint.z -= 10;}//printf("zpos: %f\n", testZPos + imagePlane);}
   void MoreDP() {dp += 0.2; if (dp > 12) dp = 12; SetDP(dp); }
   void LessDP() {dp -= 0.2; if (dp < 0) dp = 0; SetDP(dp); }
   void SetPupilSize(double pupilSize)
   {
	   lenses[3].aperture = pupilSize;
   }

private:
   // film plane's z position
   double filmPlane;
   double filmDistance;
   // lens disc z position
   double discZ;
   double apertureDiameter;
   double filmSize;
   vector<Lens> lenses;
   double dp;
   Point testPoint;
   double phi;
};

#endif
