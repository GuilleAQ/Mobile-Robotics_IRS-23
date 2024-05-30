import time
import numpy as np
from GUI import GUI
from HAL import HAL
import MAP

# Number of particles
N_PARTICLES = 20

# Constant robot velocities
LINEAR_VEL = 0.5
ANGULAR_VEL = 0.8

# Time of the last propagation of the particles
last_update_time = time.time()

def initialize_particles():
    """ Generate random particles in world coordinates (meters).
        X/Y values are constrained within the map limits.
        Yaw values are in the [0, 2*pi] range.
    """
    # Allocate space
    particles = np.zeros((N_PARTICLES, 4))
    # Get the limits from the MAP module
    x_low, y_low = MAP.WORLD_LIMITS_LOW
    x_high, y_high = MAP.WORLD_LIMITS_HIGH
    # Distribute randomly in the map
    particles = np.random.uniform(low=[x_low, y_low, 0.0],
                                  high=[x_high, y_high, 2*np.pi],
                                  size=(particles.shape[0], 3))
    return particles

def update_particle_pose(particle, dt):
    """ Update the pose of a particle in the dt period.
        Add a random Gaussian noise to the movement.
    """
    yaw = particle[2]
    # Estimate robot movement in dt according to the set velocities
    dx = dt * LINEAR_VEL * np.cos(yaw)
    dy = dt * LINEAR_VEL * np.sin(yaw)
    dyaw = dt * ANGULAR_VEL
    # Add this movement to the particle, with an extra Gaussian noise
    particle[0] += dx + np.random.normal(0.0, 0.02)
    particle[1] += dy + np.random.normal(0.0, 0.02)
    particle[2] += dyaw + np.random.normal(0.0, 0.01)

def propagate_particles(particles):
    """ Estimate the movement of the robot since the last update
        and propagate the pose of all particles according to this movement.
    """
    global last_update_time
    # Get the time diference since the last update
    current_time = time.time()
    dt = current_time - last_update_time
    # Update all particles according to dt
    for p in particles:
        update_particle_pose(p, dt)
    # Reset the update time
    last_update_time = current_time
    return particles

def main():
    # Create a HAL (robot) object
    robot = HAL()
    # Set a custom initial pose
    robot.pose[0] = 1.1
    # Create a GUI object and link it with the robot
    gui = GUI(robot=robot)
    print(F"Robot pose: {gui.getRobotPose()}")

    # Initialize some random particles
    particles = initialize_particles()
    print(F"Initial particles:\n{particles}")

    # Set a small velocity
    robot.setV(LINEAR_VEL)
    robot.setW(ANGULAR_VEL)

    # Store the time of the last pose update
    last_update_time = time.time()

    while True:
        # Propagation (prediction) step
        particles = propagate_particles(particles)

        # Show the particles in the GUI
        gui.showParticles(particles)
        # Robot pose is automatically updated inside the GUI "updateGUI"
        gui.updateGUI()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
