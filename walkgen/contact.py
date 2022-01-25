########################
# EXPORTED FROM CARACAL
########################

import pinocchio
import copy
from enum import Enum

# active-inactive-active (number of nodes)
# schedule = [ContactPhase(10), ContactPhase(10, pinocchio.SE3.Random())]


class ContactType(Enum):
    POINT = 1
    FULL = 2


class ContactPhase:
    """ Describe a motion phase.

    :param T: number of nodes
    :param oMf: placement of the contact (optional for active phases)
    """
    def __init__(self, T, contactType=ContactType.POINT, trajectory=None):
        self.T = T
        self.type = contactType
        if trajectory != None:
            self.trajectory = trajectory

    def __add__(self, phase):
        if self.type != phase.type:
            raise ArithmeticError("Couldn't append the contact phase since it doesn't belong to the same type")
        self.T += phase.T
        return self


class ContactSchedule:
    def __init__(self, dt, T, S_total, contactNames):
        self.dt = copy.deepcopy(dt)  # time step
        self.T = copy.deepcopy(T)  # number of nodes
        self.S_total = copy.deepcopy(S_total)  # maximum number of contact phases
        self.C = len(contactNames)  # number of contacts
        self.contactNames = copy.deepcopy(contactNames)  # contact names
        self.contactNames.sort()
        self.phases = [None] * self.C  # phases per each contact
        self.switches = dict()

    def __add__(self, contact_schedule):
        if len(contact_schedule.phases) != self.C:
            raise ArithmeticError(
                "Couldn't append the contact schedule since the total number of phases is not compatible")
        self.updateSwitches()
        self.T += contact_schedule.T
        self.S_total += contact_schedule.S_total
        for c in range(self.C):
            phases = copy.deepcopy(contact_schedule.phases[c])
            N_phases = len(phases)
            is_terminal_phase_inactive = len(self.phases[c]) % 2
            for p in range(N_phases):
                if is_terminal_phase_inactive and p == 0:
                    self.phases[c][-1] += phases[p]
                else:
                    self.phases[c].append(phases[p])
            for k, c in contact_schedule.switches.items():
                self.switches[k] = c
        return self

    def addSchedule(self, name, schedule):
        """ Add the contact schedule along the total horizon T.

        The first phase of each contact is by default active.
        Note that we could defined inactive first phases by passing 0 ot the first element of the contact schedule.
        :param name: name of the contact
        :param schedule: list of phases for a given contact
        """
        N_switch = 0
        N_phase = len(schedule)
        for p in range(0, N_phase, 2):
            if p + 1 < N_phase:
                if schedule[p + 1].T > 0:
                    N_switch += 1
        if N_switch > self.S_total:
            raise ArithmeticError(
                "Couldn't add the contact schedule since the total number of phases is higher than allowed")
        T = sum([t.T for t in schedule])
        if T != self.T:
            raise ArithmeticError("Couldn't add the contact schedule since the total duration is wrong.")
        c = -1
        for i in range(self.C):
            if name == self.contactNames[i]:
                c = i
                continue
        if c == -1:
            raise ArithmeticError("Couldn't add the contact schedule since it isn't define it.")
        self.phases[c] = copy.deepcopy(schedule)

    def updateSwitches(self):
        self.switches = dict()
        for c in range(self.C):
            phases = self.phases[c]
            N_phase = len(phases)
            phase_index = 0
            for p in range(0, N_phase, 2):
                T_active = phases[p].T
                T_inactive = 0
                if p + 1 < N_phase:
                    T_inactive = phases[p + 1].T
                    switch = phase_index + T_active + T_inactive - 1
                    if switch in self.switches:
                        self.switches[switch] += [c]
                    else:
                        self.switches[switch] = [c]
                phase_index += T_active + T_inactive

    def checkSchedule(self):
        N_phase = len(self.phases)
        for p in range(N_phase):
            phase = self.phases[p]
            if phase == None:
                name = self.contactNames[p]
                raise ArithmeticError("The contact-phase for " + name + " hasn't been defined")
