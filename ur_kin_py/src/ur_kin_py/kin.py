
import numpy as np

from ur_kin_py.ur5_kin_py import forward as ur5_forward
from ur_kin_py.ur5_kin_py import inverse as ur5_inverse
from ur_kin_py.ur10_kin_py import forward as ur10_forward
from ur_kin_py.ur10_kin_py import inverse as ur10_inverse

class Kinematics(object):
    UR5_ID = 'ur5'
    UR10_ID = 'ur10'
    def __init__(self, model):
        if model == 'ur5':
            self._forward = ur5_forward
            self._inverse = ur5_inverse
        elif model == 'ur10':
            self._forward = ur10_forward
            self._inverse = ur10_inverse
        else:
            print 'Model %s unknown (use "ur5" or "ur10")'
        # self.kdl_kin = create_kdl_kin('/base_link', '/ee_link', robot_urdf)

    def forward(self, q):
        q_arr = np.array(q)*1.0
        if len(q_arr) != 6:
            print 'q should be array-like of length 6'
            return None
        return np.mat(self._forward(q_arr))

    def inverse_all(self, x, q_guess=None, q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi]):
        x_mat = np.array(x)
        if x_mat.shape != (4,4):
            print 'q should be array-like of length 6'
            return None

        if q_guess is None:
            q_guess = np.zeros(6)
        q_min = np.array(q_min)
        q_max = np.array(q_max)
        sols = self._inverse(x_mat, q_guess[5])
        closest_sols = []
        for sol in sols:
            test_sol = np.ones(6)*9999.
            for i in range(6):
                for add_ang in [-2.*np.pi, 0]:
                    test_ang = sol[i] + add_ang
                    if (test_ang >= q_min[i] and test_ang <= q_max[i] and
                        abs(test_ang) < 2.*np.pi and 
                        abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                        test_sol[i] = test_ang
            if np.all(test_sol != 9999.):
                # sanity check for inverse_kin stuff
                if np.allclose(np.linalg.inv(self.forward(test_sol)) * x, np.eye(4)):
                    closest_sols.append(test_sol)
        return closest_sols

    def inverse(self, x, q_guess=None, q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi], weights=6*[1.]):
        x_mat = np.array(x)
        if x_mat.shape != (4,4):
            print 'q should be array-like of length 6'
            return None

        if q_guess is None:
            q_guess = np.zeros(6)
        closest_sols = self.inverse_all(x_mat, q_guess=q_guess, q_min=q_min, q_max=q_max)
        if len(closest_sols) == 0:
            return None
        best_sol_ind = np.argmin(np.sum((weights*(closest_sols - np.array(q_guess)))**2,1))
        best_sol = closest_sols[best_sol_ind]
        if False:
            print 'q_guess', q_guess
            print 'q_resid', q_resid
            print 'sols', sols 
            print 'closest_sols', closest_sols 
            print 'best_sol', best_sol 
        return best_sol

    # def wrap_angles(self, q):
    #     q_resid = 2.0*np.pi*(np.array(q) > np.pi) + -2.0*np.pi*(np.array(q) < -np.pi)
    #     q_wrapped = q - q_resid
    #     return q_wrapped, q_resid
