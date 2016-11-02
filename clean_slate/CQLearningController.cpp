/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"
#include <vector>



CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x(CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y(CParams::WindowHeight / CParams::iGridCellDim + 1)
{
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	//TODO
	
	for (int i = 0; i < m_NumSweepers; i++) {
		std::vector<std::vector<std::vector<double>>> actionStatesx;
		for (int j = 0; j < _grid_size_x; j++) {
			std::vector<std::vector<double>> actionStatesy;
			for (int k = 0; k < _grid_size_y; k++) {
				actionStatesy.push_back({ 0.0, 0.0, 0.0, 0.0 });
			}
			actionStatesx.push_back(actionStatesy);
		}
		Q_tables.push_back(actionStatesx);
	}
	
}


double maxActionValue(std::vector<double> actions) {
	double maxValue = 0.0;
	for (int i = 0; i < 4; i++) {
		if (actions[i] > maxValue) {
			maxValue = actions[i];
		}
	}
	return maxValue;
}


/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
	//TODO: roll your own here!

	double reward = 0.0;
	int objectHit = m_vecSweepers[sweeper_no]->CheckForObject(m_vecObjects, CParams::dMineScale);
	if (objectHit >= 0) {
		if (m_vecObjects[objectHit]->getType() == CDiscCollisionObject::Mine) {
			if (!m_vecObjects[objectHit]->isDead()) {
				reward = 100.0;
			}
		}
		else if (m_vecObjects[objectHit]->getType() == CDiscCollisionObject::SuperMine) {
			reward = -100.0f;
		}
	}

	return reward;
}



int MaxValuedAction(std::vector<double> actions) {
	int maxAction = 0;
	double maxValue = actions[maxAction];
	for (int i = 0; i < 4; i++) {
		if (actions[i] > maxValue) {
			maxAction = i;
			maxValue = actions[i];
		}
	}

	std::vector<int> allMaxValues;
	for (int i = 0; i < 4; i++) {
		if (actions[i] == maxValue) {
			allMaxValues.push_back(i);
		}
	}

	if (allMaxValues.size() > 1) {
		maxAction = RandInt(0, allMaxValues.size() - 1);
	}
	else {
		maxAction = 0;
	}

	return allMaxValues[maxAction];
}



/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update(void)
{
	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
	uint cDead = std::count_if(m_vecSweepers.begin(),
							   m_vecSweepers.end(),
						       [](CDiscMinesweeper * s)->bool{
								return s->isDead();
							   });
	if (cDead == CParams::iNumSweepers){
		printf("All dead ... skipping to next iteration\n");
		m_iTicks = CParams::iNumTicks;
	}

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/
		//1:::Observe the current state:
		//TODO
		SVector2D<int> position = m_vecSweepers[sw]->Position();
		position /= 10;

		//2:::Select action with highest historic return:
		//TODO
		int maxAction = MaxValuedAction(Q_tables[sw][position.x][position.y]);
		m_vecSweepers[sw]->setRotation(ROTATION_DIRECTION(maxAction));

		//now call the parents update, so all the sweepers fulfill their chosen action
	}
	
	CDiscController::Update(); //call the parent's class update. Do not delete this.
	
	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
		SVector2D<int> previousPosition = m_vecSweepers[sw]->PrevPosition();
		previousPosition /= 10;
		SVector2D<int> position = m_vecSweepers[sw]->Position();
		position /= 10;

		//3:::Observe new state:
		//TODO
		int action = (int)m_vecSweepers[sw]->getRotation();
		//4:::Update _Q_s_a accordingly:
		//TODO
		Q_tables[sw][previousPosition.x][previousPosition.y][action] += (lambda * (R(position.x, position.y, sw) + (gamma * maxActionValue(Q_tables[sw][position.x][position.y])) - Q_tables[sw][previousPosition.x][previousPosition.y][action]));

	}
	return true;
}

CQLearningController::~CQLearningController(void)
{
	//TODO: dealloc stuff here if you need to	
}
