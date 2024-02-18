import dataclasses
from typing import List
import casadi as cs
from dual_quaternion.type import Scalar, Vector
import numpy as np
from numbers import Number
from dual_quaternion import Quaternion
from dataclasses import field

@dataclasses.dataclass
class DualQuaternion():
    # These fields will not be automatically initialized by dataclasses
    Qr: Quaternion = field(init=False, default=None)
    Qd: Quaternion = field(init=False, default=None)
    
    def __init__(self, q_real = None, q_dual = None):
        if q_real is not None and q_dual is not None:
            if not all(isinstance(i, Quaternion) for i in [q_real, q_dual]):
                raise ValueError("Elements of the DualQuaternions should be Quaternions")
            elif (isinstance(q_real.get, np.ndarray) and isinstance(q_dual.get, np.ndarray)) or (isinstance(q_real.get, cs.MX) and isinstance(q_dual.get, cs.MX)) or (isinstance(q_real.get, cs.SX) and isinstance(q_dual.get, cs.SX)):
                self.Qr = q_real
                self.Qd = q_dual
            else:
                raise ValueError("Elements of the DualQuaternions should be Quaternions of the same type: np.array. cs.MX or c.SX")
        else:
            # Handle case where q_primal or q_dual are not provided
            raise ValueError("Both primal and dual quaternions must be provided")

    def __str__(self) -> str:
        """
        Custom __str__ method for the Quaternion class.
        This method returns a string representation of the Quaternion object.
        """
        return f'DualQuaternion(Real: {self.Qr}, Dual: {self.Qd})'

    def __getattr__(self, name):
        # Attempt to delegate attribute access to the primal or dual quaternion
        try:
            return getattr(self.Qr, name)
        except AttributeError:
            pass  # If attribute is not found in Qr, try Qd

        try:
            return getattr(self.Qd, name)
        except AttributeError:
            # If attribute is not found in both, raise an AttributeError
            raise AttributeError(f"'DualQuaternion' object has no attribute '{name}'")

    @staticmethod
    def from_pose(quat: Vector, trans: Vector) -> "DualQuaternion":
        # Get  dual Quaternion form a pose a of rigid body
        t = Quaternion(q = trans)
        q = Quaternion(q = quat)
        q_r = q
        # Dual Part (1/2) * t * q
        # Dual Part (1/2) * q * t
        q_d = (1/2)* (q*t)
        #q_d = (1/2)* (t*q)
        return DualQuaternion(q_real = q_r, q_dual = q_d)

    def __mul__(self, q2: "DualQuaternion") -> "DualQuaternion":
        if isinstance(q2, DualQuaternion):
            return DualQuaternion.product(self, q2)
        if (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q1r * q2
            qd_out =  q1d * q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q1r * q2
            qd_out =  q1d * q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q1r * q2
            qd_out =  q1d * q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Right Multiplication Dualquaternion is only defined for DualQuaternions and scalars of the same type")

    def __rmul__(self, q2) -> "DualQuaternion":
        if (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q2 * q1r
            qd_out =  q2 * q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q2 * q1r
            qd_out =  q2 * q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out =  q2 * q1r
            qd_out =  q2 * q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Left Multiplication Dualquaternion is only defined for scalars of the same type")

    @staticmethod
    def product(p: "DualQuaternion", q: "DualQuaternion") -> "DualQuaternion":
        # Get elements of the dual quaternions
        q1r = p.Qr
        q1d = p.Qd

        q2r = q.Qr
        q2d = q.Qd
        if isinstance(q1r.get, np.ndarray) and isinstance(q2r.get, np.ndarray):  # Use Vector directly without parentheses
            real = q1r * q2r
            dual = q1r * q2d + q1d * q2r

        elif isinstance(q1r.get, cs.MX) and isinstance(q2r.get, cs.MX):
            real = q1r * q2r
            dual = q1r * q2d + q1d * q2r

        elif isinstance(q1r.get, cs.SX) and isinstance(q2r.get, cs.SX):
            real = q1r * q2r
            dual = q1r * q2d + q1d * q2r
        else:
            raise TypeError("The elements of both Dualquaternions should be of the same type.")
        return DualQuaternion(q_real= real, q_dual= dual)

    @property
    def get(self) -> Vector:
        Qr_aux = self.Qr.get
        Qd_aux = self.Qd.get
        if isinstance(Qr_aux, np.ndarray) and (Qd_aux, np.array):  # Use Vector directly without parentheses
            # Funtion that defines the conjugate of a quaternion
            return np.vstack([self.Qr.get, self.Qd.get])
        elif isinstance(Qr_aux, cs.MX) and (Qd_aux, cs.MX):
            # Funtion that defines the conjugate of a quaternion
            return cs.vertcat(self.Qr.get, self.Qd.get)
        elif isinstance(Qr_aux, cs.SX) and (Qd_aux, cs.SX):
            # Funtion that defines the conjugate of a quaternion
            return cs.vertcat(self.Qr.get, self.Qd.get)
        else:
            raise TypeError("Internal problem with the definition of the DualQuaternion, it should be a np.array, cs.MX or cs.SX both elements of the same type")

    def conjugate(self) -> "DualQuaternion":
        qr = self.Qr
        qd = self.Qd
        qr_conjugate = qr.conjugate()
        qd_conjugate = qd.conjugate()
        return DualQuaternion(q_real= qr_conjugate, q_dual = qd_conjugate)

    @property
    def norm(self) -> "Scalar":
        qr = self.Qr
        qd = self.Qd
        real_norm = qr.norm
        dual_norm_aux = qr.T@qd
        dual_norm = dual_norm_aux[0, 0]/real_norm
        return real_norm, dual_norm
    
    @property
    def get_trans(self) -> "Quaternion":
        # Check this section this is related to the definition of a rigid body trasnformation
        # q = qr + e(1/2)*t*qr                   - t = 2 * qd * qr_c
        # q = qr + e(1/2)*qr*t                   - t = 2 * qr_c * qd
        qr = self.Qr
        qr_c = qr.conjugate()
        qd = self.Qd
        t = 2*qr_c * qd
        #t = 2* qd * qr_c
        t_data = t.get
        return Quaternion(q = t_data)

    @property
    def get_quat(self) -> "Quaternion":
        qr = self.Qr
        qr_data = qr.get
        return Quaternion(q = qr_data)

    @property
    def get_real(self) -> "Quaternion":
        qr = self.Qr
        qr_data = qr.get
        return Quaternion(q = qr_data)

    @property
    def get_dual(self) -> "Quaternion":
        qd = self.Qd
        qd_data = qd.get
        return Quaternion(q = qd_data)

    def __add__(self, q2: "DualQuaternion") -> "DualQuaternion":
        if isinstance(q2, DualQuaternion):
            return DualQuaternion.add(self, q2)
        elif (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r + q2
            qd_out = q1d + q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r + q2
            qd_out = q1d + q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r + q2
            qd_out = q1d + q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Right addition is only defined for DualQuaternions and scalars.")

    def __radd__(self, q2: "DualQuaternion") -> "DualQuaternion":
        if isinstance(q2, DualQuaternion):
            return DualQuaternion.add(q2, self)
        elif (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 + q1r
            qd_out = q2 + q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 + q1r
            qd_out = q2 + q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 + q1r
            qd_out = q2 + q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Left addtion is only defined for DualQuaternions and scalars")

    @staticmethod
    def add(p: "DualQuaternion", q: "DualQuaternion") -> "DualQuaternion":
        # Get elements of the dual quaternions
        q1r = p.Qr
        q1d = p.Qd

        q2r = q.Qr
        q2d = q.Qd

        if isinstance(q1r.get, np.ndarray) and isinstance(q2r.get, np.ndarray):  # Use Vector directly without parentheses
            real = q1r + q2r
            dual = q1d + q2d

        elif isinstance(q1r.get, cs.MX) and isinstance(q2r.get, cs.MX):
            real = q1r + q2r
            dual = q1d + q2d

        elif isinstance(q1r.get, cs.SX) and isinstance(q2r.get, cs.SX):
            real = q1r + q2r
            dual = q1d + q2d
        else:
            raise TypeError("The elements of both Dualquaternions should be of the same type.")
        return DualQuaternion(q_real= real, q_dual= dual)

    def __sub__(self, q2: "DualQuaternion") -> "DualQuaternion":
        if isinstance(q2, DualQuaternion):
            return  DualQuaternion.sub(self, q2)
        elif (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r - q2
            qd_out = q1d - q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r - q2
            qd_out = q1d - q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q1r - q2
            qd_out = q1d - q2
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Right substracion only defined for DualQuaternions and scalars")

    def __rsub__(self, q2: "DualQuaternion") -> "DualQuaternion":
        if isinstance(q2, Quaternion):
            return  DualQuaternion.sub(q2, self)
        elif (isinstance(q2, Number) and isinstance(self.Qr.get, np.ndarray)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 - q1r
            qd_out = q2 - q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.MX) and isinstance(self.Qr.get, cs.MX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.MX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 - q1r
            qd_out = q2 - q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        elif(isinstance(q2, cs.SX) and isinstance(self.Qr.get, cs.SX)) or (isinstance(q2, Number) and isinstance(self.Qr.get, cs.SX)):
            q1r = self.Qr
            q1d = self.Qd
            qr_out = q2 - q1r
            qd_out = q2 - q1d
            return DualQuaternion(q_real = qr_out, q_dual = qd_out)
        else:
            raise TypeError("Left Substraction is only defined for DualQuaternions and scalars")

    @staticmethod
    def sub(p: "DualQuaternion", q: "DualQuaternion") -> "DualQuaternion":
        # Get elements of the dual quaternions
        q1r = p.Qr
        q1d = p.Qd

        q2r = q.Qr
        q2d = q.Qd

        if isinstance(q1r.get, np.ndarray) and isinstance(q2r.get, np.ndarray):  # Use Vector directly without parentheses
            real = q1r - q2r
            dual = q1d - q2d

        elif isinstance(q1r.get, cs.MX) and isinstance(q2r.get, cs.MX):
            real = q1r - q2r
            dual = q1d - q2d

        elif isinstance(q1r.get, cs.SX) and isinstance(q2r.get, cs.SX):
            real = q1r - q2r
            dual = q1d - q2d
        else:
            raise TypeError("The elements of both Dualquaternions should be of the same type.")
        return DualQuaternion(q_real= real, q_dual= dual)

    def set(self, q_real = None, q_dual = None):
        # Set new elements of the Dualquaternion elements
        if q_real is not None and q_dual is not None:
            if not all(isinstance(i, Quaternion) for i in [q_real, q_dual]):
                raise ValueError("Elements of the DualQuaternions should be Quaternions")
            elif (isinstance(q_real.get, np.ndarray) and isinstance(q_dual.get, np.ndarray)) or (isinstance(q_real.get, cs.MX) and isinstance(q_dual.get, cs.MX)) or (isinstance(q_real.get, cs.SX) and isinstance(q_dual.get, cs.SX)):
                self.Qr = q_real
                self.Qd = q_dual
            else:
                raise ValueError("Elements of the DualQuaternions should be Quaternions of the same type: np.array. cs.MX or c.SX")
        else:
            # Handle case where q_primal or q_dual are not provided
            raise ValueError("Both primal and dual quaternions must be provided")

    def ln(self):
        # Log mapping of the dualQuaternion
        q1r = self.Qr
        # Get translation of the dual quaternion
        trans = self.get_trans
        trans_aux = (1/2) * trans
        # Get the log mapping of the quaterion inside the DualQuaternion
        q1r_ln = q1r.ln()

        Dual_ln = DualQuaternion(q_real=q1r_ln, q_dual= trans_aux)
        return Dual_ln

    @staticmethod
    def vector_dot(p: "DualQuaternion", q: "DualQuaternion") -> "DualQuaternion":
        # Get elements of the dual quaternions
        q1r = p.Qr
        q1d = p.Qd

        q2r = q.Qr
        q2d = q.Qd
        # Check for the type of variable of both elements
        if isinstance(q1r.get, np.ndarray) and isinstance(q2r.get, np.ndarray):  # Use Vector directly without parentheses
            real = q1r.get * q2r.get
            dual = q1d.get * q2d.get

        elif isinstance(q1r.get, cs.MX) and isinstance(q2r.get, cs.MX):
            real = q1r.get * q2r.get
            dual = q1d.get * q2d.get

        elif isinstance(q1r.get, cs.SX) and isinstance(q2r.get, cs.SX):
            real = q1r.get * q2r.get
            dual = q1d.get * q2d.get
        else:
            raise TypeError("The elements of both Dualquaternions should be of the same type.")

        # Creation of the quaternions
        real_quat = Quaternion(q = real)
        dual_quat = Quaternion(q = dual)

        return DualQuaternion(q_real= real_quat, q_dual= dual_quat)

    def vector_dot_product(self, q2):
        if isinstance(q2, DualQuaternion):
            product = DualQuaternion.vector_dot(self, q2)
            return product
        else:
            raise TypeError("Vector Dot Product only defined for DualQuaternions")