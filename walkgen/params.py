import yaml
import os

class WalkgenParams:

    def __init__(self, filename=None):
        """ Parameters for the reactive footstep planning.

        Args:
            - filename (string): path to the config file.
        """
        self.planeseg= False # Use URDF of the environment or planeseg visualisation.

        # URDF and heightmap environment without planeseg.
        path = os.path.dirname(os.path.abspath(__file__))
        self.urdf= path + "/data/urdf/lab_scene.urdf" # Env URDF path
        self.heightmap= path + "/data/lab_scene.dat"  # Heightmap path

        # Planeseg parameters for postprocessing.
        self.n_points= 6 # Maximum number of vertices for the surfaces
        self.method_id= 3 # Method to remove overlapping
        self.poly_size= 10 # Number of maximum point for convex decomposition.
        self.min_area= 0.03 # Minimum area to keep the surfaces

        # SurfacePlanner parameters.
        self.N_phase= 2 # Number of step to proceed (--> N_phase * n_gait step in SL1M)
        self.N_phase_return= 1 # Number of step to return (N_phase_return surfaces for each foot)
        self.com= False # Optimisation of the CoM
        self.multiprocessing_mimic= True # Mimic the behaviour of multiprocessing by using the surfaces of the previous optimisation.

        # Margin in m inside the convex surfaces.
        self.margin= 0.06 # inner surface margin in m

        # Gait parameters
        self.typeGait= "Walk" # Only "Walk" or "Trot" working
        self.dt= 0.01
        self.N_ss= 20 # 30 for Trot
        self.N_ds= 5 # 0 for Trot

        # Degree of the polynomial curves
        self.nx= 5
        self.ny= 5
        self.nz= 6

        if filename is not None:
            self.parseFile(filename)

    def parseFile(self, filename):
        """ Parse yaml config file to setup the parameters.

        Args:
            - filename (string): path to the config file.
        """
        config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        path = os.path.dirname(os.path.abspath(__file__))
        self.planeseg= config["walkgen_params"]["planeseg"]
        self.urdf= path + config["walkgen_params"]["world"]["urdf"]
        self.heightmap= path + config["walkgen_params"]["world"]["heightmap"]
        self.n_points= config["walkgen_params"]["params"]["n_points"]
        self.method_id= config["walkgen_params"]["params"]["method_id"]
        self.poly_size= config["walkgen_params"]["params"]["poly_size"]
        self.min_area= config["walkgen_params"]["params"]["min_area"]
        self.N_phase= config["walkgen_params"]["params"]["N_phase"]
        self.N_phase_return= config["walkgen_params"]["params"]["N_phase_return"]
        self.com= config["walkgen_params"]["params"]["com"]
        self.multiprocessing_mimic= config["walkgen_params"]["params"]["multiprocessing_mimic"]
        self.margin= config["walkgen_params"]["params"]["margin"]
        self.typeGait= config["walkgen_params"]["gait"]["type"]
        self.dt= config["walkgen_params"]["gait"]["dt"]
        self.N_ss= config["walkgen_params"]["gait"]["N_ss"]
        self.N_ds= config["walkgen_params"]["gait"]["N_ds"]
        self.nx = config["walkgen_params"]["trajectory"]["nx"]
        self.ny = config["walkgen_params"]["trajectory"]["ny"]
        self.nz = config["walkgen_params"]["trajectory"]["nz"]