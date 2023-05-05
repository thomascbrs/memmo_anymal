import os
import pickle
import datetime
import numpy as np


class Logger():

    def __init__(self, folder_path=None):
        """Log timing and performances information inside a dictionnary.

        Args:
            folder_path (str, optional): folder path. Defaults to "".
        """
        # Save information in a dictionnary
        self._profiler = {
            "update_position": [],
            "foot_position": [],
            "foot_trajectory": [],
            "surface_binding": [],
            "total_time": 0,
            "gait_time": 0,
            "fstep_time": 0
        }
        if folder_path is None:
            folder_path = os.getcwd()
        # Remove any trailing slahses from the folder path
        folder_path = folder_path.rstrip(os.path.sep)

        # Create the file path
        file_name = os.path.join(folder_path,
                                 "fsteps_" + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%fZ") + ".bin")
        self._file = open(file_name, "ab")
        self._counter = 0
        self._flushFrequency = 2

    def update_fsteps_data(self, fstep_profiler):
        """Update the logger's profiler with the values from the planner.
        (Subset dict of the main logger)

        Args:
            planner_profiler (dict): Surface planner profiler.
        """
        self._profiler.update(fstep_profiler)

    def update_global_timings(self, t0, t1, t2):
        self._profiler["total_time"] = t0
        self._profiler["gait_time"] = t1
        self._profiler["fstep_time"] = t2

    def reset_data(self):
        """Reset the profiler.
        """
        self._profiler["update_position"] = []
        self._profiler["foot_position"] = []
        self._profiler["foot_trajectory"] = []
        self._profiler["surface_binding"] = []
        self._profiler["total_time"] = 0
        self._profiler["gait_time"] = 0
        self._profiler["fstep_time"] = 0

    def write_data(self):
        """ Push data inside the binary file.
        """
        # Point to end of the file
        self._file.seek(0, 2)
        # Seriallize data as binary file
        binary_data = pickle.dumps(self._profiler)
        # Write the binary data to file buffer
        self._file.write(binary_data)
        # Flush the buffer to disk every n iterations
        # if self._counter % self._flushFrequency == 0:
        self._file.flush()
        self._counter += 1

    def close_file(self):
        self._file.close()


def process_data(filename):
    """Create a large dictionnary to analyse the datas.

    Args:
        filename (str): filename of the binary file.
    """
    profiler = {
        "update_position": [],
        "foot_position": [],
        "foot_trajectory": [],
        "surface_binding": [],
        "total_time": [],
        "gait_time": [],
        "fstep_time": []
    }

    with open(filename, mode='rb') as file:  # b is important -> binary
        while True:
            try:
                profiler_tmp = pickle.load(file)
                for elt in profiler_tmp["surface_binding"]:
                    profiler["surface_binding"].append(elt)
                for elt in profiler_tmp["foot_position"]:
                    if elt > 0.001:
                        print("problem POSITION")
                    else:
                        profiler["foot_position"].append(elt)
                for elt in profiler_tmp["foot_trajectory"]:
                    if elt > 0.0004:
                        print("problem TRAJE")
                    else:
                        profiler["foot_trajectory"].append(elt)
                for elt in profiler_tmp["update_position"]:
                    profiler["update_position"].append(elt)
                profiler["total_time"].append(profiler_tmp["total_time"])
                profiler["gait_time"].append(profiler_tmp["gait_time"])
                profiler["fstep_time"].append(profiler_tmp["fstep_time"])
            except EOFError:
                # End of file reached
                break

    return profiler


def print_stats(filename):

    profiler = process_data(filename)
    np.set_printoptions(precision=3)

    print("\n")
    print("total_time : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["total_time"]))
    print("Min [ms]: ", 1000 * np.min(profiler["total_time"]))
    print("Max [ms]: ", 1000 * np.max(profiler["total_time"]))
    print("std [ms]: ", 1000 * np.std(profiler["total_time"]))

    print("\n")
    print("gait_time : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["gait_time"]))
    print("Min [ms]: ", 1000 * np.min(profiler["gait_time"]))
    print("Max [ms]: ", 1000 * np.max(profiler["gait_time"]))
    print("std [ms]: ", 1000 * np.std(profiler["gait_time"]))

    print("\n")
    print("fstep_time : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["fstep_time"]))
    print("Min [ms]: ", 1000 * np.min(profiler["fstep_time"]))
    print("Max [ms]: ", 1000 * np.max(profiler["fstep_time"]))
    print("std [ms]: ", 1000 * np.std(profiler["fstep_time"]))

    print("\n")
    print("Foot location : ")
    print("Mean : ", 1000 * np.mean(profiler["foot_position"]))
    print("Min : ", 1000 * np.min(profiler["foot_position"]))
    print("Max : ", 1000 * np.max(profiler["foot_position"]))
    print("std : ", 1000 * np.std(profiler["foot_position"]))

    print("\n")
    print("Trajectory : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["foot_trajectory"]))
    print("Min [ms]: ", 1000 * np.min(profiler["foot_trajectory"]))
    print("Max [ms]: ", 1000 * np.max(profiler["foot_trajectory"]))
    print("std [ms]: ", 1000 * np.std(profiler["foot_trajectory"]))

    print("\n")
    print("surface_binding : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["surface_binding"]))
    print("Min [ms]: ", 1000 * np.min(profiler["surface_binding"]))
    print("Max [ms]: ", 1000 * np.max(profiler["surface_binding"]))
    print("std [ms]: ", 1000 * np.std(profiler["surface_binding"]))

    print("\n")
    print("whole_loop : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["update_position"]))
    print("Min [ms]: ", 1000 * np.min(profiler["update_position"]))
    print("Max [ms]: ", 1000 * np.max(profiler["update_position"]))
    print("std [ms]: ", 1000 * np.std(profiler["update_position"]))


def plot_stats(filename):
    import matplotlib.pyplot as plt
    profiler = process_data(filename)
    plt.figure()
    keys = ["total_time", "gait_time", "fstep_time"]
    for key in keys:
        x = np.arange(0, len(profiler.get(key)))
        plt.plot(x, profiler.get(key), "x-", label=key)
    plt.legend()
    plt.title("Timing global Fstepmanager")

    plt.figure()
    keys = ["foot_trajectory", "update_position", "foot_position", "surface_binding"]
    for key in keys:
        x = np.arange(0, len(profiler.get(key)))
        plt.plot(x, profiler.get(key), label=key)
    plt.legend()
    plt.title("Timing inside fsteps")

    plt.show()
