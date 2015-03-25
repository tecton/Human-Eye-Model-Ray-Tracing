#include "eye.h"
#include <direct.h>

int main()
{
	HumanEye camera;
	float focus, phi, theta, r, pupil;

	const int SAMPLE_DISTANCE_NUM = 10;
	const int OBJECT_DISTANCE_NUM = 8;
	const int SAMPLE_ANGLE_NUM = 8;
	const int PUPIL_SIZE_NUM = 5;

	// sample distances where eye model focuses at.
	float focus_distance[SAMPLE_DISTANCE_NUM] = {120, 200, 500, 800, 1000, 1600, 2000, 2500, 3000, 5000};
	// object distances away from eye model.
	float object_distance[SAMPLE_DISTANCE_NUM][OBJECT_DISTANCE_NUM] = {
		{50, 120, 150, 200, 300, 500, 1000, 5000},		// 120
		{50, 100, 150, 200, 300, 500, 1000, 5000},
		{100, 250, 400, 500, 600, 800, 1000, 5000},		// 500
		{100, 250, 400, 600, 800, 1000, 1500, 5000},
		{100, 300, 600, 800, 1000, 1200, 2000, 5000},	// 1000
		{200, 400, 800, 1200, 1600, 1800, 2000, 5000},
		{200, 500, 1000, 1600, 1800, 2000, 2500, 5000},	// 2000
		{200, 500, 800, 1200, 2000, 2500, 3000, 5000},
		{200, 500, 900, 1400, 2000, 2500, 3000, 5000},	// 3000
		{200, 500, 900, 1500, 2000, 3000, 4000, 5000}
	};
	float angle_sample[SAMPLE_ANGLE_NUM] = {0, 5, 10, 15, 20, 25, 30, 40};
	float pupil_size[PUPIL_SIZE_NUM] = {2, 3, 4, 6, 8};

	char dirName[100];

	// focus samples
	for (int i = 9; i < SAMPLE_DISTANCE_NUM; ++i)
	{
		focus = focus_distance[i];
		while(!camera.SetFocalLength(focus)) {}

		sprintf_s(dirName, "result\\focus-%0.0f", focus);
		_mkdir(dirName);

		printf("Focused on %fmm and computing...\n", focus);
		float z_step = focus * 0.2;

		// object samples
		for (int j = 0; j < OBJECT_DISTANCE_NUM; ++j)
		{
			r = object_distance[i][j];
			sprintf_s(dirName, "result\\focus-%0.0f\\r-%0.0f", focus, r);
			printf("Ray tracing object at distance %fmm\n", r);
			_mkdir(dirName);

			// pupil samples
			for (int k = 0; k < PUPIL_SIZE_NUM; ++k)
			{
				pupil = pupil_size[k];
				sprintf_s(dirName, "result\\focus-%0.0f\\r-%0.0f\\p-%0.0f", focus, r, pupil);
				printf("Set pupil size to %fmm\n", pupil);
				_mkdir(dirName);

				camera.SetPupilSize(pupil);

				// just sample 1/4 circle is enough (symmetry)
				for (int phi = 0; phi < SAMPLE_ANGLE_NUM; ++phi)
				{
					for (int theta = 0; theta < SAMPLE_ANGLE_NUM; ++theta)
					{
						camera.GenerateRay(focus, r, pupil, angle_sample[theta], angle_sample[phi], dirName);
					}
				}
			}
		}
	}

	system("pause");
	return 1;
}