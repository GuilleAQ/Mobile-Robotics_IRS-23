import time
import numpy as np
from GUI import GUI
from HAL import HAL
import MAP
import multiprocessing as mp

# Maximum laser detection distance in meters
MAX_LASER_DISTANCE = 100

# Number of particles
N_PARTICLES = 120
N_GROUPS = 12
PARTICLES_PER_GROUP = N_PARTICLES // N_GROUPS

# Constant robot velocities
LINEAR_VEL = 0.5
ANGULAR_VEL = 0.8

# Time of the last propagation of the particles
last_update_time = time.time()

# Max laser distance (map scale)
laser_distance_cells = MAX_LASER_DISTANCE * MAP.MAP_SCALE

# highest probability particle
max_prob = 0
best_particle = np.array([0, 0, 0])

class Particle:
    def __init__(self, x_cor, y_cor, robot_yaw, prob):
        self.xcoord = x_cor
        self.ycoord = y_cor
        self.yaw = robot_yaw
        self.probability = prob

def initialize_particles():
    """ Generate random particles in world coordinates (meters).
        X/Y values are constrained within the map limits.
        Yaw values are in the [0, 2*pi] range.
    """
    particles = np.zeros((N_PARTICLES, 4))
    x_low, y_low = MAP.WORLD_LIMITS_LOW
    x_high, y_high = MAP.WORLD_LIMITS_HIGH
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

    x_low, y_low = MAP.WORLD_LIMITS_LOW
    x_high, y_high = MAP.WORLD_LIMITS_HIGH
    particle[0] = np.clip(particle[0], x_low, x_high)
    particle[1] = np.clip(particle[1], y_low, y_high)

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


def calculate_similarity(real_data, virtual_data):
    """ Calculate the similarity between the real and virtual laser data.
        Convert both data to NumPy arrays and ensure they are the same size.
    """
    # Convert to NumPy arrays and ensure they are the same size
    real_data = np.array(real_data)
    virtual_data = np.array(virtual_data)
    
    # Use the minimum size to avoid out-of-range indices
    min_len = min(len(real_data), len(virtual_data))
    real_data = real_data[:min_len]
    virtual_data = virtual_data[:min_len]
    
    # Sample every 15 data points
    real_sampled = real_data[::15, :2]
    virtual_sampled = virtual_data[::15, :2]
    
    # Calculate distances and similarity
    distances = np.linalg.norm(real_sampled - virtual_sampled, axis=1)
    similarity = np.sum(np.exp(-distances))
    
    return similarity


def process_group(group_particles, robot):
    """ Process a group of particles and calculate their similarity
        to the robot's laser data. Return the group probabilities, 
        the highest probability, and the best particle.
    """
    hal_object = HAL()
    local_max_prob = 0
    local_best_particle = None
    group_probabilities = []

    # Copy the latest robot laser data
    robot_laser_data = robot.getLaserData().copy()

    for particle in group_particles:
        # Set the particle's pose in the HAL object
        hal_object.pose = particle
        # Get the laser data for the particle's pose
        particle_world_laser_data = hal_object.getLaserData()
        
        # Calculate the similarity between the robot's and the particle's laser data
        similarity = calculate_similarity(robot_laser_data, particle_world_laser_data)
        group_probabilities.append(similarity)

        # Update the local best particle if this one has higher similarity
        if similarity > local_max_prob:
            local_max_prob = similarity
            local_best_particle = particle

    return group_probabilities, local_max_prob, local_best_particle


def generate_new_particles(best_particles, num_particles_to_generate):
    """ Generate new particles around the best particles using a 
        multivariate normal distribution with reduced covariance.
    """
    new_particles = []
    for best_particle in best_particles:
        # Use x, y coordinates of the best particle
        mean = best_particle[:2]
        # Reduced covariance for less dispersion
        covariance_matrix = np.array([[0.05, 0], [0, 0.05]])
        new_positions = np.random.multivariate_normal(mean, 
                            covariance_matrix, num_particles_to_generate)
        
        for position in new_positions:
            # Reduced angle noise
            new_angle = best_particle[2] + np.random.normal(0.0, 0.01)
            # Ensure the angle is in the range [0, 2*pi]
            new_angle = np.mod(new_angle, 2 * np.pi)
            new_particle = np.array([position[0], position[1], new_angle])
            new_particles.append(new_particle)
    
    return new_particles


# Create a HAL (robot) object
robot = HAL()
# Set a custom initial pose
robot.pose[0] = 1.1
# Create a GUI object and link it with the robot
gui = GUI(robot=robot)

# Initialize some random particles
particles = initialize_particles()
gui.showParticles(particles)
gui.updateGUI()

# Set a small velocity
robot.setV(LINEAR_VEL)
robot.setW(ANGULAR_VEL)

# Store the time of the last pose update
last_update_time = time.time()

# counter and min_prob to let loop pass 2 laps to see initial particles
counter = 0
min_prob = 0.000000001

while True:
    # counter and min_prob to let loop pass 2 laps to see initial particles
    if counter >= 3:
        min_prob = 1 / N_PARTICLES

    # Propagation (prediction) step
    particles = propagate_particles(particles)
    gui.showParticles(particles)

    # Get some laser data and show it in the GUI
    robot_laser_data = robot.getLaserData()
    gui.showLaser(robot_laser_data)

    # Split particles into groups (particles per thread)
    particle_groups = [particles[i:i + PARTICLES_PER_GROUP] for i in range(0, N_PARTICLES, PARTICLES_PER_GROUP)]

    # Create a pool of workers and process groups in parallel
    with mp.Pool(processes=N_GROUPS) as pool:
        results = pool.starmap(process_group, [(group, robot) for group in particle_groups])

    # Collect results from all groups
    particle_probabilities = []
    for group_probabilities, local_max_prob, local_best_particle in results:
        particle_probabilities.extend(group_probabilities)
        if local_max_prob > max_prob:
            max_prob = local_max_prob
            best_particle = local_best_particle

    # Normalize probabilities
    total_probability = sum(particle_probabilities)
    particle_probabilities = [p / total_probability for p in particle_probabilities]

    new_particles = []
    particles_to_delete = 0

    for i in range(len(particle_probabilities)):
        if particle_probabilities[i] < min_prob:
            particles_to_delete += 1
        else:
            new_particles.append(particles[i])

    # Consider the top N best particles for generating new particles 
    # (4.5% of total particles)
    top_n = int(N_PARTICLES * 0.045) 
    sorted_indices = np.argsort(-np.array(particle_probabilities))
    best_particles = particles[sorted_indices[:top_n]]

    # Generate new particles
    new_generated_particles = generate_new_particles(best_particles, particles_to_delete)
    new_particles.extend(new_generated_particles)

    particles = np.array(new_particles[:N_PARTICLES])

    # Show the particles in the GUI
    gui.showParticles(particles)
    gui.updateGUI()
    counter += 1
    # time.sleep(0.1)
