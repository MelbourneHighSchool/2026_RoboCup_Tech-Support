# Localisation
In order to find its position in the field, the bot uses a custom Monte Carlo Localisation (MCL) algorithm.

## Particle Filter
Core to this algorithm is a particle filter. This is a model where initially 'particles' are placed randomly around the field. Each of these particles represents a 'pose'. A pose is a combination of position and yaw (rotation). Then, every time a new lidar scan arrives, each particle is scored with how well it matches the lidar scan. In order to calculate this, for each particle it is calculated, 'If the bot was really here, what would the lidar think the distance is for this angle?' Then, that predicted distance is compared with the actual measured distance. Close matches increase the likelihood of that particles, while matches that are further away decrease the likelihood. Then, less likely particles are given lower weights as opposed to more likely particles, which are given higher weights. To get the robot's pose from this particle filter, a weighted average is taken of all the particles.

### Resampling
After a few scans, the particles which are very close to the true position have an extremely high weight compared to all the other particles. This triggers a resample, where low weight particles are deleted and replaced with copies of the higher weighted ones. This process allows constantly refining the pose, getting more precise every resample.

In order to measure how concentrated the weight is, a measurement called the Effective Sample Size (ESS) is used.
For every particle $i$ with a weight $w_i$ normalised so that all weights sum to 1:
$$ESS = \frac{1}{\sum {w_i}^2}$$

If all 1000 particles have an equal weight, ESS = 1000. However, if only 10 have an equal weight and all the rest have 0, ESS = 10. If $\frac{\text{ESS}}{\text{PARTICLE\_COUNT}}$ drops below ESS_RESAMPLE_FRACTION (defined in `lib/localisation.cpp`), a resample will be triggered.

## Movement
Using the above method works well to find the pose of a stationary bot. However, during gameplay, this bot is always moving. Apart from the few seconds it gets while paused before kickoff, it needs to be able to localise while constantly moving. To do this, the particle filter is advanced between lidar scans through odometry.

### Odometry
Odometry is the process of using data from motion sensors to estimate change in position over time. In our robot, odometry refers to the Quick Data Readout (QDR) from the motors and the angular velocity measurement from the IMU. Another relevant term here is dead reckoning, which is the process of using only odometry to calculate the current position of an object using a previous position and an estimated change in position (such as odometry). In a case where the lidar disconnects during gameplay, dead reckoning is used to continue inferring its position. However, this is not a long term solution as small errors in odometry can accumultate over time, so dead reckoning can end up far off the true pose.

However, odometry still has a place in our localisation algorithm. Since our lidar only refreshes at 10Hz, there is a 100ms period in between scans where the particle filter has no new lidar data. If the bot was moving at 2000mm/s, in this time it would have already travelled 200mm. For a particle filter which has converged to a certain pose, there would likely be no particles in this new location, so it would take many new scans to redistribute particles and find the new pose, by which the robot would have already moved further away. To solve this, all particles are propagated through time based on the velocities from odometry. In order to account for the uncertainty in odometry measurements, each particle has a different amount of random noise added based on a normal distribution, so they naturally spread out. Then the lidar scan is used to reweight the particles.