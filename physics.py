#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#


from pyfrc.physics import motor_cfgs, tankmodel, motion
from pyfrc.physics.units import units

class PhysicsEngine(object):
    """
        Simulates a motor moving something that strikes two limit switches,
        one on each end of the track. Obviously, this is not particularly
        realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller):
        """
            :param physics_controller: `pyfrc.physics.core.PhysicsInterface` object
                                       to communicate simulation effects to
        """

        self.physics_controller = physics_controller
        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        '''
        Model for the other arcade bot
        '''
        self.hughmongoustrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            120 * units.lbs,                    # robot mass
            40,                                 # West Coast gear thing
            2,                                  # motors per side
            20* units.inch,                    # robot wheelbase
            27 * units.inch + bumper_width * 2, # robot width
            26 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on
        self.motion = motion.LinearMotion('Motion', 2, 360, 20, -20)

    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrai
        l_motor = hal_data["CAN"][1]["value"]
        r_motor = hal_data["CAN"][2]["value"]
        f_motor = hal_data["CAN"][3]["value"]
        b_motor = hal_data["CAN"][4]["value"]

        x, y, angle = self.hughmongoustrain.get_distance(l_motor, r_motor, tm_diff)
        self.physics_controller.distance_drive(x, y, angle)

		# Linear motion for encoder
        hal_data['encoder'][0]['value'] = self.motion.compute(l_motor, tm_diff)