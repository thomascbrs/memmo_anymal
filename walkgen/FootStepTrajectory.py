#!/usr/bin/env python3
#
# Copyright 2021 University of Edinburgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of  nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import pinocchio
import numpy as np

class FootStepTrajectory():

    def __init__(self, dt, N, stepHeight, M_current, M_next):
            # Stored swing-trajectory properties
            self._dt = copy.deepcopy(dt)
            self._N = copy.deepcopy(N)
            self._stepHeight = copy.deepcopy(stepHeight)
            self._M_current = copy.deepcopy(M_current)
            self._M_next = copy.deepcopy(M_next)

            # Polynomial coefficients
            self.Ax = [0.] * 6  # x-axis
            self.Ay = [0.] * 6  # y-axis
            self.Az = [0.] * 7  # z-axis

            # Create polynomial curve
            self._createPolynomialCurve()

    def position(self, k):
        if k >= self._N:
            raise ArithmeticError(
                "Invalid argument: k is bigger than the allocated number of nodes (it should be less than or equal to "
                + str(self._N) + ")")
        # TODO(cmastalli): swing interpolation in SE3
        return pinocchio.SE3(np.eye(3), self._evaluate(0, k * self._dt))

    def velocity(self, k):
        if k >= self._N:
            raise ArithmeticError(
                "Invalid argument: k is bigger than the allocated number of nodes (it should be less than or equal to "
                + str(self._N) + ")")
        # TODO(cmastalli): swing interpolation in motion
        return pinocchio.Motion(self._evaluate(1, k * self._dt), np.zeros(3))

    def _createPolynomialCurve(self):
        x0 = self._M_current.translation
        xf = self._M_next.translation
        self._updatePolyCoeff_XY(x0, np.zeros(3), np.zeros(3), xf, 0., self._N * self._dt)
        self._updatePolyCoeff_Z(x0, np.zeros(3), np.zeros(3), xf, 0., self._N * self._dt, self._stepHeight )

        return 0


    def update(self,x0,v0, xf, t0):

        self._updatePolyCoeff_XY(x0, self._evaluate(1,t0), self._evaluate(2,t0), xf, t0, self._N * self._dt)
        if t0 == 0.:
            if xf[2] > 0.05:
                h = xf[2] + 0.1
            else:
                h = self._stepHeight
            self._updatePolyCoeff_Z(x0, v0, self._evaluate(2,t0), xf, t0, self._N * self._dt,h)

        return 0

    def set_reference(self, stepHeight=None, M_current=None, M_next=None):
        """ Only used for old code, only updating with position
        """
        if stepHeight != None:
            self._stepHeight = stepHeight
        if M_current != None:
            self._M_current = M_current
        if M_next != None:
            self._M_next = M_next
        if stepHeight == None and M_current == None and M_next == None:
            print("Warning: we don't set any reference, all the terms are None")
            return
        self.update(self._M_current.translation, np.zeros(3), self._M_next.translation, 0.)



    def _updatePolyCoeff_XY(self,x_init, v_init, a_init, x_end,  t0, t1):
        ''' Update internal coefficients for 5D polynomial curve, X and Y trajectory. Vel, Acc final is nulle.

        Args:
        - x_init (array x3) : initial position [x0,y0,z0]
        - v_init (array x3) : initial velocity [dx0,dy0,dz0]
        - a_init (array x3) : initial acceleration [ddx0,ddy0,ddz0]
        - x_end  (array x3) : end position [x1,y1,z1]
        - t0, t1 (float): intial and final time
        - h (float): height for z curve
        '''
        x0, y0, z0 = x_init
        dx0, dy0, dz0 = v_init
        ddx0, ddy0, ddz0 = a_init
        x1, y1, z1 = x_end

        # compute polynoms coefficients for x and y
        self.Ax[5] = (ddx0*t0**2 - 2*ddx0*t0*t1 - 6*dx0*t0 + ddx0*t1**2 + 6*dx0*t1 + 12 *
                            x0 - 12*x1)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ax[4] = (30*t0*x1 - 30*t0*x0 - 30*t1*x0 + 30*t1*x1 - 2*t0**3*ddx0 - 3*t1**3*ddx0 + 14*t0**2*dx0 - 16*t1**2*dx0 +
                            2*t0*t1*dx0 + 4*t0*t1**2*ddx0 + t0**2*t1*ddx0)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ax[3] = (t0**4*ddx0 + 3*t1**4*ddx0 - 8*t0**3*dx0 + 12*t1**3*dx0 + 20*t0**2*x0 - 20*t0**2*x1 + 20*t1**2*x0 - 20*t1**2*x1 + 80*t0*t1*x0 - 80*t0 *
                            t1*x1 + 4*t0**3*t1*ddx0 + 28*t0*t1**2*dx0 - 32*t0**2*t1*dx0 - 8*t0**2*t1**2*ddx0)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ax[2] = -(t1**5*ddx0 + 4*t0*t1**4*ddx0 + 3*t0**4*t1*ddx0 + 36*t0*t1**3*dx0 - 24*t0**3*t1*dx0 + 60*t0*t1**2*x0 + 60*t0**2*t1*x0 - 60*t0*t1 **
                            2*x1 - 60*t0**2*t1*x1 - 8*t0**2*t1**3*ddx0 - 12*t0**2*t1**2*dx0)/(2*(t0**2 - 2*t0*t1 + t1**2)*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ax[1] = -(2*t1**5*dx0 - 2*t0*t1**5*ddx0 - 10*t0*t1**4*dx0 + t0**2*t1**4*ddx0 + 4*t0**3*t1**3*ddx0 - 3*t0**4*t1**2*ddx0 - 16*t0**2 *
                            t1**3*dx0 + 24*t0**3*t1**2*dx0 - 60*t0**2*t1**2*x0 + 60*t0**2*t1**2*x1)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ax[0] = (2*x1*t0**5 - ddx0*t0**4*t1**3 - 10*x1*t0**4*t1 + 2*ddx0*t0**3*t1**4 + 8*dx0*t0**3*t1**3 + 20*x1*t0**3*t1**2 - ddx0*t0**2*t1**5 - 10*dx0*t0 **
                            2*t1**4 - 20*x0*t0**2*t1**3 + 2*dx0*t0*t1**5 + 10*x0*t0*t1**4 - 2*x0*t1**5)/(2*(t0**2 - 2*t0*t1 + t1**2)*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))

        self.Ay[5] = (ddy0*t0**2 - 2*ddy0*t0*t1 - 6*dy0*t0 + ddy0*t1**2 + 6*dy0*t1 + 12 *
                            y0 - 12*y1)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ay[4] = (30*t0*y1 - 30*t0*y0 - 30*t1*y0 + 30*t1*y1 - 2*t0**3*ddy0 - 3*t1**3*ddy0 + 14*t0**2*dy0 - 16*t1**2*dy0 +
                            2*t0*t1*dy0 + 4*t0*t1**2*ddy0 + t0**2*t1*ddy0)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ay[3] = (t0**4*ddy0 + 3*t1**4*ddy0 - 8*t0**3*dy0 + 12*t1**3*dy0 + 20*t0**2*y0 - 20*t0**2*y1 + 20*t1**2*y0 - 20*t1**2*y1 + 80*t0*t1*y0 - 80*t0 *
                            t1*y1 + 4*t0**3*t1*ddy0 + 28*t0*t1**2*dy0 - 32*t0**2*t1*dy0 - 8*t0**2*t1**2*ddy0)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ay[2] = -(t1**5*ddy0 + 4*t0*t1**4*ddy0 + 3*t0**4*t1*ddy0 + 36*t0*t1**3*dy0 - 24*t0**3*t1*dy0 + 60*t0*t1**2*y0 + 60*t0**2*t1*y0 - 60*t0*t1 **
                            2*y1 - 60*t0**2*t1*y1 - 8*t0**2*t1**3*ddy0 - 12*t0**2*t1**2*dy0)/(2*(t0**2 - 2*t0*t1 + t1**2)*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ay[1] = -(2*t1**5*dy0 - 2*t0*t1**5*ddy0 - 10*t0*t1**4*dy0 + t0**2*t1**4*ddy0 + 4*t0**3*t1**3*ddy0 - 3*t0**4*t1**2*ddy0 - 16*t0**2 *
                            t1**3*dy0 + 24*t0**3*t1**2*dy0 - 60*t0**2*t1**2*y0 + 60*t0**2*t1**2*y1)/(2*(t0 - t1)**2*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))
        self.Ay[0] = (2*y1*t0**5 - ddy0*t0**4*t1**3 - 10*y1*t0**4*t1 + 2*ddy0*t0**3*t1**4 + 8*dy0*t0**3*t1**3 + 20*y1*t0**3*t1**2 - ddy0*t0**2*t1**5 - 10*dy0*t0 **
                            2*t1**4 - 20*y0*t0**2*t1**3 + 2*dy0*t0*t1**5 + 10*y0*t0*t1**4 - 2*y0*t1**5)/(2*(t0**2 - 2*t0*t1 + t1**2)*(t0**3 - 3*t0**2*t1 + 3*t0*t1**2 - t1**3))

        return 0

    def _updatePolyCoeff_Z(self, x_init, v_init, a_init, x_end,  t0, t1, h):
        ''' Update internal coefficients for 5D polynomial curve, Z trajectory. Vel, Acc final is nulle.

        Args:
        - x_init (np.array x3) : initial position [x0,y0,z0]
        - v_init (np.array x3) : initial velocity [dx0,dy0,dz0]
        - a_init (np.array x3) : initial acceleration [ddx0,ddy0,ddz0]
        - x_end  (np.array x3) : end position [x1,y1,z1]
        - t0, t1 (float): intial and final time
        - h (float): height for z curve
        '''
        x0, y0, z0 = x_init
        dx0, dy0, dz0 = v_init
        ddx0, ddy0, ddz0 = a_init
        x1, y1, z1 = x_end

        self.Az[6] = (32.*z0 + 32.*z1 - 64.*h)/(t1**6)
        self.Az[5] = - (102.*z0 + 90.*z1 - 192.*h)/(t1**5)
        self.Az[4] = (111.*z0 + 81.*z1 - 192.*h)/(t1**4)
        self.Az[3] = - (42.*z0 + 22.*z1 - 64.*h)/(t1**3)
        self.Az[2] = 0
        self.Az[1] = 0
        self.Az[0] = z0

        return 0

    def _evaluate(self, indice, t):
        '''Evaluate the polynomial curves at time t.

        Args:
            - indice (int): Derivative index (Ex: 0 --> poly(t) ; 1 --> poly'(t) ...)
            - t (float) : time

        Returns:
            - (Array x3): Evaluation of the polynomial curve on x,y and z axis.
        '''
        x, y, z = 0., 0., 0.
        for id, coeff in enumerate(self.Ax):
            if id >= indice:
                x += (np.math.factorial(id) / np.math.factorial(id - indice)) * coeff * (t**(id - indice))
        for id, coeff in enumerate(self.Ay):
            if id >= indice:
                y += (np.math.factorial(id) / np.math.factorial(id - indice)) * coeff * (t**(id - indice))
        for id, coeff in enumerate(self.Az):
            if id >= indice:
                z += (np.math.factorial(id) / np.math.factorial(id - indice)) * coeff * (t**(id - indice))

        return np.array([x, y, z])