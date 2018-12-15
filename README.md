# Angerborbs

The framework splits the simulation into three sub-simulations,
one for the sling (SlingSubSim), one for the bird (BirdSubSim) and 
one for the obstacles and floor (ObstacleSubSim).

This way, all contributors can work on different simulations without
producing merge conflicts.

All meshes are loaded as rigid bodies, the collision system between different 
sub-simulations should be implemented using a custom interface.

Some additional, custom mesh-files (sling.off, foot.off) can be found in the
/data folder along the default ones from the exercises.

## Bird simulation

Implements a soft body with a rigid core.
The soft body has a spherical topology. In each iteration, the mesh is treated 
as a soft body. Gravity, any external force, and the damping force of the mass 
spring system are applied to each vertex. Additionally, to enforce a smooth 
bouncing motion, instead of collapsing the sphere, a 'rigid core' is used.

### Rigid core
In each iteration, we compute the 'rigid core', the position of vertice as if it 
would get if it is treated as a rigid body. The 'rigid core' is invariant in its 
own coordinate while its position **q** in the world coordinate is updated every 
iteration by undergoing a rigid motion **A** that best conforms to the new position 
**p** of the soft body vertices. It involves solving a linear system **Aq**=**p**:

> (p', q' denotes the transpose of p, q)

>**Aq**=**p** 

>**Aqq'**=**pq'**

>**A**=**(qq')^-1pq'**


Since only the rigid motion is desired, polar decomposition is applied to extract the rotation **R** of **A**:
>**R=VSV'** where SVD(**A**)=**USV'**.

 and the translation is simply the displacement of the center of mass.


We try to pull each vertice of the soft 
body back to the 'rigid core' to avoid collapsing. We connect a 'sping' from 
each of the current soft body vertex to the corresponding vertex in the 'rigid core' 






Wang, Yanzhen, et al. "A mass-spring model for surface mesh deformation based on shape matching." GRAPHITE. Vol. 6. 2006.



## Obstacle simulation

### Obstacle class

Inherits from RigidObject and implements operations needed for movement/collision.
As Rigid bodies are stored in an array not as pointers but data structs they cannot add their own fields, the default RigidObject has been extended instead. 

#### Added fields

- extend: Extend of the class relative to its center
- active: Whether the body is put asleep by the simulation (non-moving)
- name: Identifier for the object for debugging purposes
- counter: Counter used to time when an object can be put to sleep
- time: Time left on the object to simulate (for speculative contacts)

#### Collision

There are multiple trace functions that test for edges and points on boxes. For custom collisions, trace can be used. All traces return a HitResult which contains:

- insider: Trace end-point
- location: Point on the surface
- normal: Surface normal
- depth: Depth of the insider point
- times: Trace distance multiple of input direction

Hit tests on whole objects return multiple hits. Helper functions do a sweep or resolve penetrations. External impulses can be resolved using the applyImpulse functions. For non-obstacle classes, the computeImpulse and applyImpulse functions can be used.


### Obstacle simulation class

The obstacle simulation class moves obstacles and resolves contacts. In the advance function, first all objects are moved (symplectic solved), then collisions are resolved, then unmoved objects put to sleep. External simulations can access the obstacles using getCollideables.

Multiple features can be enabled via the UI.

