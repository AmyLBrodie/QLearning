#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>
#include <vector>

typedef unsigned int uint;
class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;

	std::vector<std::vector<std::vector<std::vector<double>>>> Q_tables;

	double lambda = 0.2;
	double gamma = 0.5;

public:
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
};

