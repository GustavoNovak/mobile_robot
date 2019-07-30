<?php

T ← InitializeStartTree(startPoint); 
T ← InitializeGoalTree(goalPoint);
T ← InsertNode(Ø, Zinit, T);

for i = 1:maxIteration {
	activeTree ← AlternateTree(activeTree); 
	
	Zrand ← Sample(i); 
	Znearest ← Nearest(T, activeTree, Zrand); 
	Znew ← generateNewNode(Znearest, Zrand); 
	Zparent ← Chooseparent (T, actualTree, Znew, searchRadius); 

	if Obstaclefree(Zparent, Znew) {
		T ← InsertNode(T, actualTree, Zparent, Znew);   
		T ← Rewire (T, actualTree, Znew, searchRadius); 
		ZnearestOtherTree ← getNearestNodeOtherTree (T, actualTree, Znew);  

		if Obstaclefree(ZnearestOtherTree, Znew)	 
			path ← PathOptimization(T, Znew, ZnearestOtherTree); 
			return path; 
		}
	}
}


