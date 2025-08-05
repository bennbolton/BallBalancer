import Box2D
import math
import matplotlib.pyplot as plt

class PhysicsBody:
    def __init__(self, world: Box2D.b2World, position, radius, density=1.0, friction=0.3):
        """Initializes a dynamic body with a circular shape in Box2D."""
        self.world = world
        self.body = self.world.CreateDynamicBody(position=position)
        self.fixture = self.body.CreateCircleFixture(
            radius=radius,
            density=density,
            friction=friction
        )
        self.radius = radius
    
    def apply_torque(self, torque):
        """Applies torque to the body."""
        self.body.ApplyTorque(torque, wake=True)

    def get_position(self):
        """Returns the position of the body."""
        return self.body.position

    def get_angle(self):
        """Returns the angle (tilt) of the body in degrees."""
        return math.degrees(self.body.angle)

    def update(self):
        """Updates the body's physics simulation."""
        self.world.Step(1.0 / 60, 6, 2)  # Step the simulation (velocity, position iterations)

class RobotSimulator:
    def __init__(self):
        """Initializes the physics world and robot setup."""
        self.world = Box2D.b2World(gravity=(0, -9.81), doSleep=True)
        
        groundBody = self.world.CreateStaticBody(
                        position=(0,-10),
                        shapes=Box2D.b2PolygonShape(box=(50,10)),
                        )

        # Ball (larger circle)
        self.ball_radius = 1.0  # 1 meter in radius
        self.ball_position = (0, 10)
        self.ball = PhysicsBody(self.world, self.ball_position, self.ball_radius)

        # Robot (smaller circle on top of the ball)
        self.robot_radius = 0.5  # Half the size of the ball
        self.robot_position = (0, self.ball_position[1] + self.ball_radius + self.robot_radius)
        self.robot = PhysicsBody(self.world, self.robot_position, self.robot_radius)

        # Set up matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(0, 15)
        
        # Plot circles (ball and robot)
        self.ball_plot, = self.ax.plot([], [], 'bo', markersize=10)  # Blue circle for the ball
        self.robot_plot, = self.ax.plot([], [], 'ro', markersize=6)  # Red circle for the robot

    def update_plot(self):
        """Updates the plot to reflect the new positions of the ball and robot."""
        self.ball_plot.set_data(self.ball.get_position().x, self.ball.get_position().y)
        self.robot_plot.set_data(self.robot.get_position().x, self.robot.get_position().y)

    def simulate(self, steps=300):
        """Simulate the physics for a given number of steps and update the plot."""
        for _ in range(steps):  # Run for a specific number of steps
            self.ball.update()
            self.robot.update()
            
            # Optional: Apply torque to robot at specific frame
            if _ == 50:
                self.robot.apply_torque(10.0)
            
            # Update the plot
            self.update_plot()
            
            # Draw and pause to show the plot
            plt.draw()
            plt.pause(1.0 / 60)  # Wait for the time step

            # Output robot position and angle for debugging
            print(f"Ball Position: {self.ball.get_position()}, Robot Position: {self.robot.get_position()}")
            print(f"Robot Angle (degrees): {self.robot.get_angle()}")
        
        plt.show()

# Main execution
if __name__ == "__main__":
    simulator = RobotSimulator()
    simulator.simulate()