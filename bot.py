# SPDX-License-Identifier: BSD-3-Clause

# flake8: noqa F401
from collections.abc import Callable

import numpy as np

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    Vector,
    config,
)
from vendeeglobe.utils import distance_on_surface


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    def __init__(self):
        self.team = "M/S Klara"  # This is your team name
        # This is the course that the ship has to follow

        # ORIGINAL:
        # self.course = [
        #     Checkpoint(latitude=43.797109, longitude=-11.264905, radius=50),
        #     Checkpoint(longitude=-29.908577, latitude=17.999811, radius=50),
        #     Checkpoint(latitude=-11.441808, longitude=-29.660252, radius=50),
        #     Checkpoint(longitude=-63.240264, latitude=-61.025125, radius=50),
        #     Checkpoint(latitude=2.806318, longitude=-168.943864, radius=1990.0),
        #     Checkpoint(latitude=-62.052286, longitude=169.214572, radius=50.0),
        #     Checkpoint(latitude=-15.668984, longitude=77.674694, radius=1190.0),
        #     Checkpoint(latitude=-39.438937, longitude=19.836265, radius=50.0),
        #     Checkpoint(latitude=14.881699, longitude=-21.024326, radius=50.0),
        #     Checkpoint(latitude=44.076538, longitude=-18.292936, radius=50.0),
        #     Checkpoint(
        #         latitude=config.start.latitude,
        #         longitude=config.start.longitude,
        #         radius=5,
        #     ),
        # ]

        # Through Panama canal
        self.course = [
            Checkpoint(latitude=44.076538, longitude=-18.292936, radius=50.0),
            Checkpoint(latitude=19.026818, longitude=-67.377639, radius=50.0),
            Checkpoint(latitude=17.426834, longitude=-67.709029, radius=50.0),
            Checkpoint(latitude=9.739913, longitude=-80.019893, radius=50.0),
            Checkpoint(latitude=9.444044, longitude=-79.958481, radius=5.0),
            Checkpoint(latitude=9.337686, longitude=-79.920029, radius=5.0),
            Checkpoint(latitude=9.246205, longitude=-79.883637, radius=5.0),
            Checkpoint(latitude=9.162157, longitude=-79.812912, radius=5.0),
            Checkpoint(latitude=9.105210, longitude=-79.690689, radius=5.0),
            Checkpoint(latitude=8.977047, longitude=-79.577393, radius=5.0),
            Checkpoint(latitude=8.914644, longitude=-79.538254, radius=5.0),
            Checkpoint(latitude=8.868513, longitude=-79.524907, radius=5.0),
            Checkpoint(latitude=8.735515, longitude=-79.257502, radius=5.0),
            Checkpoint(latitude=7.136045, longitude=-79.841150, radius=5.0),
            Checkpoint(latitude=-17.38580236480157, longitude=-143.47527508361958, radius=5.0),
            Checkpoint(latitude=-44.64625713801945, longitude=148.09603766967672, radius=5.0),
            Checkpoint(latitude=-35.48682269284264, longitude=111.36010576306454, radius=5.0),
            Checkpoint(latitude=-19.94004549333223, longitude=76.25221356954064, radius=5.0),
            Checkpoint(latitude=-37.96823566578649, longitude=20.529558154205304, radius=5.0),
            Checkpoint(latitude=9.830589476816863, longitude=-23.064192072233695, radius=5.0),
            Checkpoint(latitude=31.233705069053237, longitude=-21.65794228024158, radius=5.0),
            Checkpoint(latitude=44.7406853160408, longitude=-13.220442406626134, radius=5.0),
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            ),
        ]

    def run(
        self,
        t: float,
        dt: float,
        longitude: float,
        latitude: float,
        heading: float,
        speed: float,
        vector: np.ndarray,
        forecast: Callable,
        world_map: Callable,
    ) -> Instructions:
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            Method to query the weather forecast for the next 5 days.
            Example:
            current_position_forecast = forecast(
                latitudes=latitude, longitudes=longitude, times=0
            )
        world_map:
            Method to query map of the world: 1 for sea, 0 for land.
            Example:
            current_position_terrain = world_map(
                latitudes=latitude, longitudes=longitude
            )

        Returns
        -------
        instructions:
            A set of instructions for the ship. This can be:
            - a Location to go to
            - a Heading to point to
            - a Vector to follow
            - a number of degrees to turn Left
            - a number of degrees to turn Right

            Optionally, a sail value between 0 and 1 can be set.
        """
        # Initialize the instructions
        instructions = Instructions()

        # TODO: Remove this, it's only for testing =================
        current_position_forecast = forecast(
            latitudes=latitude, longitudes=longitude, times=0
        )
        current_position_terrain = world_map(latitudes=latitude, longitudes=longitude)
        # ===========================================================

        # Go through all checkpoints and find the next one to reach
        for ch in self.course:
            # Compute the distance to the checkpoint
            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )
            # Consider slowing down if the checkpoint is close
            jump = dt * np.linalg.norm(speed)
            if dist < 2.0 * ch.radius + jump:
                instructions.sail = min(ch.radius / jump, 1)
            else:
                instructions.sail = 1.0
            # Check if the checkpoint has been reached
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                instructions.location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break

        return instructions
