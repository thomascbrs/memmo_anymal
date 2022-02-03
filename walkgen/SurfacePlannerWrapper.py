import multiprocessing
import walkgen.SurfacePlanner as SurfacePlanner
from walkgen.SurfacePlanner import Surface
import yaml
import copy
import numpy as np

from multiprocessing import Process
import rospy
from visualization_msgs.msg import MarkerArray

class SurfacePlannerWrapper():
    '''
    Wrapper for the class SurfacePlanner for the paralellisation
    '''

    def __init__(self, T_gait, n_gait, filename):
        """ Initialize the surface planner.

        Args:
            - filename (str): Path to the config file.
            - T_gait (float): The period of a gait.
            - n_gait (int): Number of phases in one gait.
        """

        self._config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        self.planeseg = self._config["planeseg"]
        self.multiprocessing = self._config["params"]["multiprocessing"]
        self._N_phase_return = self._config["params"]["N_phase_return"]
        self.T_gait = T_gait
        self.n_gait = n_gait
        self.filename = filename

        if self.multiprocessing:  # Setup variables in the shared memory
            self.manager = multiprocessing.Manager()
            self.ns = self.manager.Namespace()
            self.ns.array_markers = MarkerArray()
            self.ns.locker = self.manager.Lock()
            self.ns.q = np.array(19)
            self.ns.bvref = np.array(6)
            self.ns.gait = np.zeros((4,4))
            self.ns.target_foostep = np.zeros((3,4))
            self.ns.running = self.manager.Value('b', True)
            self.ns.newData = self.manager.Value('b', False)
            self.ns.newResult = self.manager.Value('b', False)
            self.ns.iteration = self.manager.Value('i', 0)
            self.ns.selected_surfaces = dict()
        else:
            self.surface_planner = SurfacePlanner(T_gait, n_gait, filename)

        # Initial selected surface
        A = [[-1., 0., 0.], [0., -1., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., 1.], [-0., -0., -1.]]
        b = [1., 1., 1., 1., 0., 0.]
        vertices = [[-1., 1., 0.], [-1., -1., 0.], [1., -1., 0.], [1., 1., 0.]]
        self._init_surface = Surface(np.array(A), np.array(b), np.array(vertices).T)

        # Add initial surface to the result structure.
        self._selected_surfaces = dict()
        # Dictionary type :
        # For each foot is associated a list of surfaces, one foot is moving one time for each gait/phase
        self._contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']  # Order of the feet in the surface planner.
        for foot in range(4):
            L = []
            for k in range(self._N_phase_return):
                L.append(copy.deepcopy(self._init_surface))
            self._selected_surfaces[self._contact_names[foot]] = L

    def run(self, q, gait_in, bvref, target_foostep, array_markers=None):
        if self.multiprocessing:
            pass
            # self.run_asynchronous(q, gait_in, bvref, target_foostep, array_markers)
        else:
            return self.run_synchronous(q, gait_in, bvref, target_foostep, array_markers)

    def run_synchronous(self, q, gait_in, bvref, target_foostep, array_markers = None):

        selected_surfaces = copy.deepcopy(self._selected_surfaces) # Mimic async behaviour
        self._selected_surfaces = self.surface_planner.run(q, gait_in, bvref, target_foostep)

        return selected_surfaces


    def run_asynchronous(self,q, gait_in, bvref, target_foostep, array_markers= None):
        # If this is the first iteration, creation of the parallel process
        with self.dataIn.get_lock():
            if self.dataIn.iteration == 0:
                p = Process(target=self.create_MIP_asynchronous,
                            args=(self.ns,))
                p.start()
        # Stacking data to send them to the parallel process
        self.compress_data(q, gait_in, bvref, target_foostep, array_markers)
        self.newData.value = True

    def create_MIP_asynchronous(self, ns):
        while ns.running.value:
            # Checking if new data is available to trigger the asynchronous MPC
            if ns.newData.value:
                # Decompress data in
                self.ns.locker.acquire()
                # Set the shared variable to false to avoid re-trigering the asynchronous MPC
                ns.newData.value = False
                if ns.iteration.value == 0:
                    loop_planner = SurfacePlanner(self.T_gait, self.n_gait, self.filename)

                q = ns.q
                gait_in = ns.gait
                bvref = ns.bvref
                target_foostep = ns.target_foostep
                array_markers = ns.array_markers
                ns.locker.release()

                selected_surfaces = loop_planner.run(q, gait_in, bvref, target_foostep, array_markers)

                self.ns.locker.acquire()
                ns.selected_surfaces = selected_surfaces
                ns.iteration.value += 1
                ns.newResult.value = True
                ns.locker.release()

                print("OKKK")


    def compress_data(self, q, gait_in, bvref, target_foostep, array_markers):
        self.ns.locker.acquire()
        self.ns.q = q
        self.ns.gait = gait_in
        self.ns.bvref = bvref
        self.ns.target_foostep = target_foostep
        self.ns.array_markers = array_markers
        self.ns.locker.release()

if __name__ == "__main__":
    """ Run a simple example of SurfacePlanner.
    """
    import pickle
    import os
    # Load Marker array class example from ROS simulation
    fileObject = os.getcwd() + "/data/example_marker_array.pickle"

    with open(fileObject, 'rb') as file2:
        array_markers = pickle.load(file2)

    filename = "/home/thomas_cbrs/Desktop/edin/memmo_anymal/walkgen/config/parameters.yaml"
    surface_planner = SurfacePlannerWrapper(0.6, 2, filename)

    q = np.array([0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
    q[0] = 0.
    bvref = np.zeros(6)
    bvref[0] = 0.3
    bvref[5] = 0.1
    # gait_in = np.array([[1, 0, 0, 1], [0, 1, 1, 0]])
    gait_in = np.array([[0, 1, 1, 1], [1, 0, 1, 1], [1,1,0,1], [1,1,1,0]])
    current_position = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
    selected_surfaces = surface_planner.run(q, gait_in, bvref, current_position, array_markers)

    # import matplotlib.pyplot as plt
    # import sl1m.tools.plot_tools as plot

    # if not surface_planner.planeseg :
    #     ax = plot.draw_whole_scene(surface_planner._all_surfaces)
    #     plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)

    # else:
    #     from walkgen.tools.plot_tools import plot_marker_surface
    #     # Plot initial surfaces from markerArray
    #     fig = plt.figure(figsize=(10, 6))
    #     ax = plt.axes(projection='3d')
    #     plt.title("Initial surfaces")
    #     plot_marker_surface(array_markers,ax)

    #     # Plot SL1M results
    #     fig = plt.figure(figsize=(10, 6))
    #     ax = plt.axes(projection='3d')
    #     plt.title("SL1M result")
    #     for sf in surface_planner.surfaces_processed :
    #         plot.plot_surface(sf,ax=ax)
    #     plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)


