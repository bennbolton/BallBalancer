import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

class Body:
    def __init__(self, x, y, radius, mass):
        """Generic physics body with position, velocity, rotation, and forces."""
        self.x = x  # Position
        self.y = y
        self.vx = 0  # Velocity
        self.vy = 0
        self.ax = 0  # Acceleration
        self.ay = 0
        self.theta = 0  # Angular position
        self.omega = 0  # Angular velocity
        self.alpha = 0  # Angular acceleration
        self.radius = radius  # Radius of the object
        self.mass = mass  # Mass of the object
        self.forces = []  # List of forces applied

    def apply_force(self, force, angle):
        """Apply a force in a given direction (angle in radians)."""
        fx = force * np.cos(angle)
        fy = force * np.sin(angle)
        self.ax += fx / self.mass  # Newton's second law
        self.ay += fy / self.mass

    def apply_torque(self, torque):
        """Apply a rotational force (torque)."""
        self.alpha += torque / (self.mass * self.radius ** 2)  # Rotational inertia ~ m * r²

    def update(self, dt):
        """Update the body's motion using Newtonian physics."""
        # Linear motion
        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Angular motion
        self.omega += self.alpha * dt
        self.theta += self.omega * dt

        # Reset accelerations (forces are applied per frame)
        self.ax, self.ay, self.alpha = 0, 0, 0

    def check_collision(self, other):
        """Ensure the robot stays on the surface of the ball."""
        dx = self.x - other.x
        dy = self.y - other.y
        distance = np.sqrt(dx**2 + dy**2)

        required_distance = other.radius + self.radius  # Surface distance
        if distance < required_distance:
            # Normalize displacement vector
            dx /= distance
            dy /= distance

            # Move robot to the correct surface position
            self.x = other.x + dx * required_distance
            self.y = other.y + dy * required_distance

            # Adjust velocity to follow the surface
            tangent_angle = np.arctan2(dy, dx) + np.pi / 2  # Perpendicular to radius
            v_total = np.sqrt(self.vx**2 + self.vy**2)  # Speed magnitude
            self.vx = v_total * np.cos(tangent_angle)
            self.vy = v_total * np.sin(tangent_angle)


class BallBalancingRobot(Body):
    def __init__(self, x, y, radius, mass):
        """The small robot balancing on top of a ball."""
        super().__init__(x, y, radius, mass)

    def control_movement(self, omega):
        """Directly set rotational velocity."""
        self.omega = omega


class LargeBall(Body):
    def __init__(self, x, y, radius, mass):
        """The large ball that the robot balances on."""
        super().__init__(x, y, radius, mass)


class Simulation:
    def __init__(self):
        """Set up the simulation with the ball and the robot."""
        self.R = 5  # Large ball radius
        self.r = 1  # Small robot radius
        self.frames = 200
        self.interval = 50
        self.dt = 0.01  # Time step
        self.pad = 5

        # Create physics objects
        self.objects = []
        self.large_ball = LargeBall(0, 0, self.R, mass=10)
        self.robot = BallBalancingRobot(3, self.R + self.r, self.r, mass=1)
        self.objects.extend([self.large_ball, self.robot])

        # Set up matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-self.R - self.pad, self.R + self.pad)
        self.ax.set_ylim(-self.R - self.pad, self.R + self.pad)
        self.ax.set_aspect('equal')

        # Visual elements
        self.large_ball_patch = Circle((self.large_ball.x, self.large_ball.y), self.R, color='lightgrey', fill=False)
        self.robot_patch = Circle((self.robot.x, self.robot.y), self.r, color='blue', fill=True)

        self.ax.add_patch(self.large_ball_patch)
        self.ax.add_patch(self.robot_patch)

        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=200, interval=self.dt*1000, blit=True)

    def update(self, frame):
        """Update the physics and redraw objects."""
        self.robot.apply_force(-9.81 * self.robot.mass, np.pi / 2)  # Apply gravity
        self.robot.update(self.dt)  # Update physics

        # Check collision with large ball (should always be touching)
        # if self.robot.check_collision(self.large_ball):
        #     # Keep the robot on top of the ball
        #     self.robot.y = self.large_ball.y + self.R + self.r

        # Update visual positions
        self.robot_patch.set_center((self.robot.x, self.robot.y))
        return (self.robot_patch,)

    def run(self):
        """Start the simulation."""
        plt.show()


# Run the simulation
if __name__ == "__main__":
    sim = Simulation()
    sim.run()