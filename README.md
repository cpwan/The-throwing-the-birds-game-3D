# Angerborbs

The framework splits the simulation into three sub-simulations,
one for the sling (SlingSubSim), one for the bird (BirdSubSim) and 
one for the obstacles and floor (ObstacleSubSim).

This way, all contributors can work on different simulations without touching
producing merge conflicts.

All meshes are loaded as rigid bodies, the collision system between differen 
sub-simulations should probably be implemented using a custom interface.

Some additional, custom mesh-files (sling.off, foot.off) can be found in the
/data folder along the default ones from the exercises.