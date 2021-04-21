import os
import numpy as np
import orhelper
from random import gauss
import math
import sys
import matplotlib.pyplot as plt


class LandingPoints(list):
    "A list of landing points with ability to run simulations and populate itself"

    def __init__(self):
        self.ranges = []
        self.bearings = []

    def add_simulations(self, num):
        with orhelper.OpenRocketInstance() as instance:

            # Load the document and get simulation
            orh = orhelper.Helper(instance)
            doc = orh.load_doc(os.path.join('examples', 'simple.ork'))
            sim = doc.getSimulation(0)

            # Randomize various parameters
            opts = sim.getOptions()
            rocket = opts.getRocket()

            # Run num simulations and add to self
            for p in range(num):
                print('Running simulation ', p+1)

                opts.setLaunchRodAngle(math.radians(gauss(45, 5)))  # 45 +- 5 deg in direction
                opts.setLaunchRodDirection(math.radians(gauss(0, 5)))  # 0 +- 5 deg in direction
                opts.setWindSpeedAverage(gauss(15, 5))  # 15 +- 5 m/s in wind
                for component_name in ('Nose cone', 'Body tube'):  # 5% in the mass of various components
                    component = orh.get_component_named(rocket, component_name)
                    mass = component.getMass()
                    component.setMassOverridden(True)
                    component.setOverrideMass(mass * gauss(1.0, 0.05))

                airstarter = AirStart(gauss(1000, 50))  # simulation listener to drop from 1000 m +- 50
                lp = LandingPoint(self.ranges, self.bearings)
                orh.run_simulation(sim, listeners=(airstarter, lp))
                # print("Iteration result: ",self.ranges[-1])
                self.append(lp)

    def print_stats(self):
        print(
            'Rocket landing zone %3.2f m +- %3.2f m bearing %3.2f deg +- %3.4f deg from launch site. Based on %i simulations.' % \
            (np.mean(self.ranges), np.std(self.ranges), np.degrees(np.mean(self.bearings)),
             np.degrees(np.std(self.bearings)), len(self)))

    def get_stats(self):
        r_mean,r_std = np.mean(self.ranges),np.std(self.ranges)
        b_mean,b_std = np.mean(self.bearings),np.std(self.bearings)

        return r_mean,r_std,b_mean,b_std

    def plot_stats(self):

        iterations = [i for i in range(len(self.ranges))]
        mean_ranges = [np.mean(self.ranges[:i]) for i in range(1,len(self.ranges)+1)]
        std_ranges = [np.std(self.ranges[:i]) for i in range(1,len(self.ranges)+1)]

        upper_bound = [mean_ranges[i]+std_ranges[i] for i in range(len(mean_ranges))]
        lower_bound = [mean_ranges[i]-std_ranges[i] for i in range(len(mean_ranges))]

        fig = plt.figure()
        plt.plot(iterations, mean_ranges)
        plt.fill_between(iterations, lower_bound, upper_bound, facecolor = 'yellow', alpha=0.25)
        fig.suptitle('Mean range by monte carlo')
        plt.xlabel('Iterations',fontsize=10)
        plt.ylabel('Landing range',fontsize=10)
        fig.savefig('monte-carlo.jpg')
        plt.show()

class LandingPoint(orhelper.AbstractSimulationListener):
    def __init__(self, ranges, bearings):
        self.ranges = ranges
        self.bearings = bearings

    def endSimulation(self, status, simulation_exception):
        worldpos = status.getRocketWorldPosition()
        conditions = status.getSimulationConditions()
        launchpos = conditions.getLaunchSite()
        geodetic_computation = conditions.getGeodeticComputation()

        if geodetic_computation != geodetic_computation.FLAT:
            raise Exception("GeodeticComputationStrategy type not supported")

        self.ranges.append(range_flat(launchpos, worldpos))
        self.bearings.append(bearing_flat(launchpos, worldpos))


class AirStart(orhelper.AbstractSimulationListener):

    def __init__(self, altitude):
        self.start_altitude = altitude

    def startSimulation(self, status):
        position = status.getRocketPosition()
        position = position.add(0.0, 0.0, self.start_altitude)
        status.setRocketPosition(position)


METERS_PER_DEGREE_LATITUDE = 111325
METERS_PER_DEGREE_LONGITUDE_EQUATOR = 111050


def range_flat(start, end):
    dy = (end.getLatitudeDeg() - start.getLatitudeDeg()) * METERS_PER_DEGREE_LATITUDE
    dx = (end.getLongitudeDeg() - start.getLongitudeDeg()) * METERS_PER_DEGREE_LONGITUDE_EQUATOR
    return math.sqrt(dy * dy + dx * dx)


def bearing_flat(start, end):
    dy = (end.getLatitudeDeg() - start.getLatitudeDeg()) * METERS_PER_DEGREE_LATITUDE
    dx = (end.getLongitudeDeg() - start.getLongitudeDeg()) * METERS_PER_DEGREE_LONGITUDE_EQUATOR
    return math.pi / 2 - math.atan(dy / dx)


if __name__ == '__main__':
    num = int(sys.argv[1])

    points = LandingPoints()
    points.add_simulations(num)
    points.print_stats()
    points.plot_stats()