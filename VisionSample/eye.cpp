#include "eye.h"

#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

using namespace std;

HumanEye::HumanEye()
{
    ParseSpecfile("Navarro.dat");
    double dz = -2.674404474;
    discZ = lenses[lenses.size() - 1].zPos + dz;
    
    testPoint = Point(0, 100, 200);
    phi = 0;
}

bool HumanEye::TraceLenses(const Ray &inRay, Ray &outRay, int start, int end) const
{
	Ray tempRay = inRay;
	if (inRay.o.z > 10)
    {
		// move start point close enough
        double t = (Point(0, 0, 10) - inRay.o).z / inRay.d.z;
        tempRay.o = inRay(t);
    }

	int increment = (end >= start)? 1: -1;
	int i = start;
	
	for (; i != end; i += increment)
	{
		if (!lenses[i].RefractRay(tempRay, outRay))
		{
			break;
		}
		tempRay = outRay;
	}
	if (i == end)
		return true;
	else
		return false;
}

bool HumanEye::TraceLenses(const Ray &inRay, Ray &outRay, int start, int end, vector<Point>& points) const
{
	Ray tempRay = inRay;
	points.clear();
	points.push_back(inRay.o);
	if (inRay.o.z > 10)
    {
        double t = (Point(0, 0, 10) - inRay.o).z / inRay.d.z;
        tempRay.o = inRay(t);
    }

	int increment = (end >= start)? 1: -1;
	int i = start;
	
	for (; i != end; i += increment)
	{
		if (!lenses[i].RefractRay(tempRay, outRay))
		{
            points.push_back(outRay.o);
			break;
		}
		points.push_back(outRay.o);
		tempRay = outRay;
	}
	if (i == end)
	{
//		points.push_back(outRay(100));
		return true;
	}
	else
		return false;
}

void ConcentricSampleDisk(double u1, double u2, double *dx, double *dy) {
    double r, theta;
    // Map uniform random numbers to $[-1,1]^2$
    double sx = 2 * u1 - 1;
    double sy = 2 * u2 - 1;

    // Map square to $(r,\theta)$

    // Handle degeneracy at the origin
    if (sx == 0.0 && sy == 0.0) {
        *dx = 0.0;
        *dy = 0.0;
        return;
    }
    if (sx >= -sy) {
        if (sx > sy) {
            // Handle first region of disk
            r = sx;
            if (sy > 0.0) theta = sy / r;
            else          theta = 8.0f + sy / r;
        }
        else {
            // Handle second region of disk
            r = sy;
            theta = 2.0f - sx / r;
        }
    }
    else {
        if (sx <= sy) {
            // Handle third region of disk
            r = -sx;
            theta = 4.0f - sy / r;
        }
        else {
            // Handle fourth region of disk
            r = -sy;
            theta = 6.0f + sx / r;
        }
    }
    theta *= M_PI / 4.f;
    *dx = r * cosf(theta);
    *dy = r * sinf(theta);
}

Point HumanEye::getRandomPointOnLens() const
{
    Point p;
    double ux, uy, dx, dy;
    ux = ((double) rand() / (RAND_MAX));
    uy = ((double) rand() / (RAND_MAX));
    ConcentricSampleDisk(ux, uy, &dx, &dy);
	p.x = dx * lenses[3].aperture / 2;
    p.y = dy * lenses[3].aperture / 2;
    p.z = discZ;
    return p;
}

void HumanEye::ParseSpecfile(const string& specfile)
{
	ifstream infile(specfile.c_str());
	if (!infile) {
		fprintf(stderr, "Cannot open spec file %s\n", specfile.c_str());
		exit (-1);
	}

	char line[512];
	double rad, zpos = 0, asph, axpos, lastRefr = 1.0f, refr, aper;
	while (!infile.eof()) {
		infile.getline(line, 512);
		if (line[0] != '\0' && line[0] != '#' && line[0] != '\n')
		{
			sscanf(line, "%lf\t%lf\t%lf\t%lf\t%lf\n", &rad, &asph, &axpos, &refr, &aper);
			Lens lens(-rad, asph, zpos, refr / lastRefr, aper);
			lenses.insert(lenses.begin(), lens);
			zpos -= axpos;
			if (refr != 0)
				lastRefr = refr;
		}
	}
}

HumanEye::~HumanEye()
{
	lenses.clear();
}

double HumanEye::GenerateRay(double focus, double r, double pupil, double theta, double phi, char *dirName) const
{
	FILE* fp;
	int iter = 100, pass = 0;
    char filename[128];
    memset(filename, 0, sizeof(filename));
	sprintf(filename, "%s\\_%0.1f_%0.1f.txt", dirName, theta, phi);
	fp = fopen(filename, "w");

	double x = r * cos(theta / 180 * M_PI) * sin(phi / 180 * M_PI);
	double y = r * sin(theta / 180 * M_PI);
	double z = r * cos(theta / 180 * M_PI) * cos(phi / 180 * M_PI);
	Point objectSample = Point(x, y, z);
	vector<Point> points;

	double aperture = lenses[lenses.size() - 1].aperture / 2;

	for (double i = -aperture / 2; i <= aperture / 2; i += aperture / 25)
		for (double j = -aperture / 2; j <= aperture / 2; j += aperture / 25)
		{
			Point lensSample = Point(i, j, discZ);

			Ray filmLensRay(objectSample, Normalize(lensSample - objectSample), 0), primaryRay;
			if (TraceLenses(filmLensRay, primaryRay, (int)lenses.size() - 1, -1, points))
			{
				primaryRay.d = Normalize(primaryRay.o - Point(0, 0, lenses[0].zPos + 2 * lenses[0].radius));
				double t = (lenses[0].zPos - primaryRay.o.z) / primaryRay.d.z;
				Point hit = primaryRay(t);
				fprintf(fp, "%f %f\n", hit.x, hit.y);
				pass++;
			}
		}

	//printf("pass number: %d\n", pass);

	fclose(fp);
	return 0;
}

void HumanEye::SetDP(double A)
{
	lenses[2].radius = -(10.2 - 1.75 * log(A + 1));
	lenses[1].radius = -(-6 + 0.2294 * log(A + 1));
	lenses[3].zPos = lenses[4].zPos - (3.05 - 0.05 * log(A + 1));
    lenses[2].zPos = lenses[3].zPos;
	lenses[1].zPos = lenses[2].zPos - (4 + 0.1 * log(A + 1));
    lenses[0].zPos = lenses[1].zPos - 16.3203;
	lenses[2].refractionRatio = (1.42 + 9e-5 * (10 * A + A * A)) / 1.3374;
	lenses[2].asphericity = -3.1316 - 0.34 * log(A + 1);
	lenses[1].asphericity = -1 - 0.125 * log(A + 1);
}

bool HumanEye::SetFocalLength(double f)
{
	Ray inRay, outRay;
	double start = 0, end = 13, current = start;
	int iter = 500;
	inRay.o = Point(0, 0, 0);
	while (iter--)
	{
		SetDP(current);
		inRay.o.z = f;
		double passed = 0;
		double focusSum = 0;
		int iter = 50;
		for (int i = 0; i < iter; ++i)
		{
			// trace 50 rays through eye model
			Point p = getRandomPointOnLens();
			p.x = 0;
			//p.y = 0.4;
			//p.z = 0;
			inRay.d = Normalize(p - inRay.o);
			
			//inRay.o = Point(0,1,1);
			//inRay.d = Vector(0,0,-1);
			if (TraceLenses(inRay, outRay, lenses.size() - 1, 0))
			{
				double t = -outRay.o.y / outRay.d.y;
				if (t > 0)
					focusSum += outRay(t).z;
				passed++;
			}
		}
		double focus = focusSum / passed;
		if (fabsf((double)(focus - lenses[0].zPos)) < 0.002)
		{
			//fprintf(fp, "%f", log(current));
			printf("%f %f\n", focus, current);
			return true;
		}
		if (focus > lenses[0].zPos)
			end = current;
		else
			start = current;
		current = (start + end) / 2;
	}
	printf("failed %f\n", f);
	return false;
}

// Add code for lens
Lens::Lens(double rad, double asph, double zpos, double refr, double aper)
	: radius(rad), asphericity(asph), zPos(zpos), refractionRatio(refr), aperture(aper)
{
}

Vector Lens::GetNormal(const Ray * ray, const Point & p) const {
	Vector o = p - Point(0,0,zPos);
    Vector result = Normalize(Vector(o.x,
                                 o.y,
                                 (1 + asphericity) * o.z - radius));
	if (result.z < 0)
		result = -1 * result;
	return result;
}

bool Lens::RefractRay(const Ray &inRay, Ray &outRay) const
{
	Point phit;
	if (radius == 0)
	{
		// lens is aperture
		double thit = (zPos - inRay.o.z) / inRay.d.z;
		if (thit < 0 || thit > inRay.maxt)
			return false;
		phit = inRay(thit);
		if (phit.x * phit.x + phit.y * phit.y > aperture * aperture * 0.25)
			return false;
		outRay.o = phit;
		outRay.d = inRay.d;
		return true;
	}
	// Transfer inRay's origin relative to center point
	Vector tOrigin = inRay.o - Point(0,0,zPos);
	Vector tDir = inRay.d;//Normalize(inRay.d - Vector(0,0,zPos));
	// Compute quadratic sphere coefficients
	//double A = inRay.d.x * inRay.d.x + inRay.d.y * inRay.d.y + inRay.d.z * inRay.d.z;
	//double A = 1;
	double A = 1 + asphericity * tDir.z * tDir.z;
	double B = 2 * (tDir.x * tOrigin.x + tDir.y * tOrigin.y + (1 + asphericity) * tDir.z * tOrigin.z - radius * tDir.z);
	double C = tOrigin.x * tOrigin.x + tOrigin.y * tOrigin.y + (1 + asphericity) * tOrigin.z * tOrigin.z - 2 * radius * tOrigin.z;
	if (A == 0)
	{
		double t = -C / B + 1e-6;
		phit = inRay(t);
	}
	else
	{
		const double discriminant = B*B - 4*A*C;
		if (discriminant < 0)
			return false;

		const double t0 = (-B - sqrt(discriminant)) * 0.5f / A;
		const double t1 = (-B + sqrt(discriminant)) * 0.5f / A;

		//const double & tt = t0 > 0 ? t0 : t1;
		double tt;
		//if (asphericity <= -1) // first 2 lenses
		//	tt = t0 > t1? t0 : t1;
		//else
		//	tt = t0 > t1? t1 : t0;
		if (t0 > 0 && t1 > 0)
			tt = t0 > t1? t1: t0;
		else if (t0 > 0 && t1 < 0)
			tt = t0;
		else if (t0 < 0 && t1 > 0)
			tt = t1;
		else if (t0 == 0)
			tt = 0;
		else
			tt = -1;
		if (tt < 0 || tt != tt)
			return false;

		phit = inRay(tt);
	}
    if (refractionRatio == 0)
    {
        // hit retina
        outRay.o = phit;
        outRay.d = inRay.d;
        return true;
    }
    if (phit.HasNaNs() || !OnLens(phit))
        return false;
	//Vector n  = Normalize(phit - center);
	Vector n = GetNormal(&inRay, phit);
	Vector in = Normalize(tDir);

	double mu = refractionRatio;
	if (tDir.z < 0)
		mu = 1 / mu;
	double cosI = -Dot(n, in);
	double sinT2 = mu * mu * (1 - cosI * cosI);
	if (sinT2 > 1.0)
		return false;
	double cosT = sqrt(1 - sinT2);
	outRay.d = mu * in + (mu * cosI - cosT) * n;
  
	outRay.o = phit;
	return true;
}

bool Lens::OnLens(Point p) const
{
	Vector pp = p - Point(0,0,zPos);
	if (pp.x * pp.x + pp.y * pp.y > aperture * aperture * 0.25)
		return false;
	//if ((pp.z < -1e-6) != (radius < -1e-6))
	//	return false;
    return true;
}

double Lens::yToZ(double y)
{
	double posz1, posz2;
    if (refractionRatio == 0) {
        return zPos + radius - sqrt(radius * radius - y * y + 1e-6);
    }
	if (!Quadratic(1 + asphericity, -2 * radius, y * y, &posz1, &posz2))
        return 1;
	if (asphericity == -1)
		return zPos + posz1;
	if (asphericity < -1 && asphericity > -3)
		return zPos + posz2;
	if (asphericity < -3)
		return zPos + posz1;
	return zPos - ((radius > 0)? posz1: -posz2);
	//return zPos + posz1;
}

// Add code end