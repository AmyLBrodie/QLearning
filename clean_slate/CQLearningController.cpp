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
	setLifeState(); // set all sweepers to alive
	
	// set all actions for each state to 0 in the tables for each sweeper
	for (int i = 0; i < m_NumSweepers; i++) {
		std::vector<std::vector<std::vector<double>>> actionStatesx;
		for (int x = 0; x < _grid_size_x; x++)
		{
			std::vector<std::vector<double > > actionStatesy;
			for (int y = 0; y < _grid_size_y; y++)
			{
				actionStatesy.push_back({ 0.0, 0.0, 0.0, 0.0 });
			}
			actionStatesx.push_back(actionStatesy);
		}
		Q_tables.push_back(actionStatesx);
	}
	
}



int DecideNextMove(vector<double> actions)
{
	int direction = 0; 
	double value = actions[0];

	// find direction with best reward
	for (int i = 0; i < 4; i++)
	{
		if (actions[i] > value) {
			direction = i;
			value = actions[i];
		}
	}

	// if multiple actions carry the same reward then randomly decide the action to be taken
	vector<int> maxActions;
	for (int b = 0; b < 4; b++)
	{
		if (actions[b] == value) {
			maxActions.push_back(b); 
		}
	}

	int chosenDirection = maxActions[RandInt(0, maxActions.size() - 1)];

	return chosenDirection;
}




/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
	
	double reward = 0.0;
	int hit = (m_vecSweepers[sweeper_no])->CheckForObject(m_vecObjects, CParams::dMineScale);

	if (hit >= 0)
	{
		switch (m_vecObjects[hit]->getType()) {
		case CDiscCollisionObject::Mine: // collides with a mine (collects it)
		{
			if (!m_vecObjects[hit]->isDead())
			{
				return 100.0;
			}
			break;
		}
		case CDiscCollisionObject::SuperMine: // collides with a suer mine (sweeper destroyed)
		{
			return -100.0;
			break;
		}
		}
	}

	return reward;
}


//returns the max value action of the state  
int MaxValuedAction(std::vector<double> actions) {
	double maxValue = actions[0];
	for (uint a = 1; a < 4; a++) {
		if (actions[a] > maxValue) {
			maxValue = actions[a];
		}
	}
	return maxValue;
}


// sets all sweepers to alive
void CQLearningController::setLifeState()
{
	isdead.clear();
	for (int i = 0; i < m_NumSweepers; i++)
	{
		isdead.push_back(false);
	}
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

		setLifeState();
	}


	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw) {
		if (m_vecSweepers[sw]->isDead()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/
		// get current state
		SVector2D<int> currentState = m_vecSweepers[sw]->Position();
		currentState /= 10; 

		// decide the next state
		int nextMove = DecideNextMove(Q_tables[sw][currentState.x][currentState.y]);
	
		// move towards the next state
		m_vecSweepers[sw]->setRotation((ROTATION_DIRECTION)nextMove);
	}

	CDiscController::Update(); //call the parent's class update. Do not delete this.

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw) {
		if (m_vecSweepers[sw]->isDead() && isdead[sw]) { 
			continue; 
		}
		else if (m_vecSweepers[sw]->isDead()) {
			isdead[sw] = true; // set sweeper to dead
		}
		
		// get information on current state and previous state
		SVector2D<int> previousPosition = m_vecSweepers[sw]->PrevPosition();
		SVector2D<int> position = m_vecSweepers[sw]->Position();
		previousPosition /= 10;
		position /= 10;

		//get the previous chosen action
		int action = (int)m_vecSweepers[sw]->getRotation();
		
		// update the q table
		Q_tables[sw][previousPosition.x][previousPosition.y][action] += (learn*(R(position.x, position.y, sw) + (discount * MaxValuedAction(Q_tables[sw][position.x][position.y])) - Q_tables[sw][previousPosition.x][previousPosition.y][action]));
	}

	//reset life state of sweepers at end of iteration 
	if (m_iTicks == CParams::iNumTicks)
	{
		setLifeState();
	}

	return true;
}

CQLearningController::~CQLearningController(void)
{
	//TODO: dealloc stuff here if you need to	
}



