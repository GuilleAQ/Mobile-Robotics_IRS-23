import numpy as np
from GUI import GUI
import MAP

N_PARTICLES = 10

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


def compute_particle_weights(particles):
    """ Compute the weight of each particle.
        This function should generate a virtual laser measurement
        for each particle and compute the error (difference)
        between the virtual laser and the actual sensor measurement.

        This example function just generates silly weights.
    """
    weights = np.arange(particles.shape[0], dtype=np.float32)
    return weights


def resample_particles(old_particles, weights):
    """ Resample the set of particles given their weights. """
    # Allocate space for new particles
    particles = np.zeros((N_PARTICLES, 3))

    # Normalize the weights so the total sum is 1
    weights /= np.sum(weights)
    print(F"Normalized weights: {weights}")

    # Get random indices from the list of particles
    selected_idx = np.random.choice(N_PARTICLES, replace=True, size=N_PARTICLES, p=weights)
    print(F"Selected indices:\n{selected_idx}")
    print(F"Selected particles:\n{old_particles[selected_idx]}")
    particles = old_particles[selected_idx]
    return particles


def main():
    # Create the GUI object
    gui = GUI()

    # Initialize some random particles
    particles = initialize_particles()
    print(F"Initial particles:\n{particles}")

    # Show the particles in the GUI
    gui.showParticles(particles)

    # Update the GUI image and wait
    gui.updateGUI(block=True)

    # Compute the weights
    weights = compute_particle_weights(particles)
    print(F"Weights: {weights}")

    # Resample the particles
    particles = resample_particles(particles, weights)
    print(F"Resampled particles:\n{particles}")

    # Show the particles in the GUI
    gui.showParticles(particles)
    # Update the GUI image and wait
    gui.updateGUI(block=True)

if __name__ == '__main__':
    main()
