import dataclasses
from typing import List
import casadi as cs
from dual_quaternion.type import Scalar, Vector
import numpy as np
from numbers import Number

@dataclasses.dataclass
class Quaternion():
    # Properties of the class
    q: Vector
    def __init__(self, q = None):
        if q is not None:
            # Check if the vairbale is a np.array
            if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
                if q.shape[0] != 4:
                    raise ValueError("quaternion must have exactly 4 elements.")
                q_aux = q.reshape((4, 1))
                self.q = q_aux

            # Check for casadi variables
            elif isinstance(q, cs.MX) or isinstance(q, cs.SX):
                if q.shape[0] != 4:
                    raise ValueError("quaternion must have exactly 4 elements.")
                self.q = q
            else:
                raise TypeError("quaternion must be an ndarray or Casadi MX  SX")

    def __getattr__(self, attr):
        # Funcion that enables the access to the same atributes inside an np.array or numpy object
        return getattr(self.q, attr)

    def __repr__(self) -> str:
        return f"Quaternion: {self.q}"

    def __str__(self) -> str:
        return str(self.q)

    @property
    def get(self) -> Vector:
        return self.q

    def __mul__(self, q2: "Quaternion") -> "Quaternion":
        # Funtions that operates between quaternions
        if isinstance(q2, Quaternion):
            return Quaternion(q = Quaternion.product(self.q, q2.q))
        elif isinstance(q2, Number):
            q = self.q
            q_out = q * q2
            return Quaternion(q = q_out)
        else:
            raise TypeError("Right Multiplication only defined for Quaternions and scalars")

    def __rmul__(self, q2: Scalar) -> "Quaternion":
        if isinstance(q2, Number):
            return Quaternion(q=q2 * self.q)
        else:
            raise TypeError("Left Multiplication only defined for scalars")

    @staticmethod
    def product(p: Vector, q: Vector) -> Vector:
        if isinstance(p, np.ndarray) and isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            aux_1 = p[0, 0] * q[0, 0] - np.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ np.cross(p[1:4, 0], q[1:4, 0])
            q_product = np.vstack((aux_1, aux_2[0], aux_2[1], aux_2[2]))
            q_product = q_product.reshape((4, 1))
            return q_product

        elif isinstance(p, cs.MX) and isinstance(q, cs.MX):
            aux_1 = p[0, 0] * q[0, 0] - cs.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ cs.cross(p[1:4, 0], q[1:4, 0])
            q_product = cs.vertcat(aux_1, aux_2)
            return q_product

        elif isinstance(p, cs.SX) and isinstance(q, cs.SX):
            aux_1 = p[0, 0] * q[0, 0] - cs.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ cs.cross(p[1:4, 0], q[1:4, 0])
            q_product = cs.vertcat(aux_1, aux_2)
            return q_product
        else:
            raise TypeError("The elements of both quaternions should be of the same type.")

    def __add__(self, q2: "Quaternion") -> "Quaternion":
        if isinstance(q2, Quaternion):
            return Quaternion(q = Quaternion.add(self.q, q2.q))
        elif isinstance(q2, Number) or isinstance(q2, cs.MX) or isinstance(q2, cs.SX):
            q = self.q
            q_out = q + q2
            return Quaternion(q = q_out)
        else:
            raise TypeError("Right add only defined for Quaternions and scalars")

    def __radd__(self, q2: "Quaternion") -> "Quaternion":
        if isinstance(q2, Quaternion):
            return Quaternion(q = Quaternion.add(q2.q, self.q))
        elif isinstance(q2, Number) or isinstance(q2, cs.MX) or isinstance(q2, cs.SX):
            q = self.q
            q_out =  q2 + q
            return Quaternion(q = q_out)
        else:
            raise TypeError("Left add only defined for Quaternions and scalars")

    @staticmethod
    def add(p: Vector, q: Vector) -> Vector:
        # Funtion that defines the addition opperation
        if isinstance(p, np.ndarray) and isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            aux_1 = p + q
            q_product = aux_1
            return q_product

        elif isinstance(p, cs.MX) and isinstance(q, cs.MX):
            aux_1 = p + q
            q_product = aux_1
            return q_product

        elif isinstance(p, cs.SX) and isinstance(q, cs.SX):
            aux_1 = p + q
            q_product = aux_1
            return q_product
        else:
            raise TypeError("The elements of both quaternions should be of the same type.")
            
    def __sub__(self, q2: "Quaternion") -> "Quaternion":
        if isinstance(q2, Quaternion):
            return Quaternion(q = Quaternion.sub(self.q, q2.q))
        elif isinstance(q2, Number) or isinstance(q2, cs.MX) or isinstance(q2, cs.SX):
            q = self.q
            q_out = q - q2
            return Quaternion(q = q_out)
        else:
            raise TypeError("Right sub only defined for Quaternions and scalars")

    def __rsub__(self, q2: "Quaternion") -> "Quaternion":
        if isinstance(q2, Quaternion):
            return Quaternion(q = Quaternion.sub(q2.q, self.q))
        elif isinstance(q2, Number) or isinstance(q2, cs.MX) or isinstance(q2, cs.SX):
            q = self.q
            q_out =  q2 - q
            return Quaternion(q = q_out)
        else:
            raise TypeError("Left add only defined for Quaternions and scalars")

    @staticmethod
    def sub(p: Vector, q: Vector) -> Vector:
        # Function uses to define the substraction operation
        if isinstance(p, np.ndarray) and isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            aux_1 = p - q
            q_product = aux_1

        elif isinstance(p, cs.MX) and isinstance(q, cs.MX):
            aux_1 = p - q
            q_product = aux_1

        elif isinstance(p, cs.SX) and isinstance(q, cs.SX):
            aux_1 = p - q
            q_product = aux_1
        else:
            raise TypeError("The elements of both quaternions should be of the same type.")
        return q_product

    def conjugate(self) -> "Quaternion":
        q = self.q
        if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            qw = q[0, 0]
            qx = -q[1, 0]
            qy = -q[2, 0]
            qz = -q[3, 0]
            q_conjugate = np.vstack((qw, qx, qy, qz))
            # Funtion that defines the conjugate of a quaternion
            return Quaternion(q = q_conjugate)
        elif isinstance(q, cs.MX):
            qw = q[0, 0]
            qx = -q[1, 0]
            qy = -q[2, 0]
            qz = -q[3, 0]
            q_conjugate = cs.vertcat(qw, qx, qy, qz)
            # Funtion that defines the conjugate of a quaternion
            return Quaternion(q = q_conjugate)
        elif isinstance(q, cs.SX):
            qw = q[0, 0]
            qx = -q[1, 0]
            qy = -q[2, 0]
            qz = -q[3, 0]
            q_conjugate = cs.vertcat(qw, qx, qy, qz)
            # Funtion that defines the conjugate of a quaternion
            return Quaternion(q = q_conjugate)
        else:
            raise TypeError("Internal problem with the definition of the Quaternion, it should be a np.array, cs.MX or cs.SX.")
    @property
    def norm(self) -> "Scalar":
        q = self.q
        if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            norm = np.sqrt(np.dot(q.T, q))
            norm_value = norm[0,0]
            return norm_value
        elif isinstance(q, cs.MX):
            norm = cs.sqrt(cs.dot(q, q))
            norm_value = norm
            return norm_value
        elif isinstance(q, cs.SX):
            norm = cs.sqrt(cs.dot(q, q))
            norm_value = norm
            return norm_value
        else:
            raise TypeError("Internal problem with the definition of the Quaternion, it should be a np.array, cs.MX or cs.SX.")

    @property
    def square_norm(self) -> "Scalar":
        q = self.q
        if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            norm = np.dot(q.T, q)
            norm_value = norm[0,0]
            return norm_value
        elif isinstance(q, cs.MX):
            norm = cs.dot(q, q)
            norm_value = norm
            return norm_value
        elif isinstance(q, cs.SX):
            norm = cs.dot(q, q)
            norm_value = norm
            return norm_value
        else:
            raise TypeError("Internal problem with the definition of the Quaternion, it should be a np.array, cs.MX or cs.SX.")

    def inverse(self) -> "Quaternion":
        # Function that computes the inverse of a quaternion
        return self.conjugate() / self.square_norm()

    def set(self, q = None):
         if q is not None:
            # Check if the vairbale is a np.array
            if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
                if q.shape[0] != 4:
                    raise ValueError("quaternion must have exactly 4 elements.")
                q_aux = q.reshape((4, 1))
                self.q = q_aux

            # Check for casadi variables
            elif isinstance(q, cs.MX) or isinstance(q, cs.SX):
                if q.shape[0] != 4:
                    raise ValueError("quaternion must have exactly 4 elements.")
                self.q = q
            else:
                raise TypeError("quaternion must be an ndarray or Casadi MX  SX")
    @property
    def angle_axis(self):
        q = self.q
        norm = self.norm
        if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            qw = q[0, 0]
            angle = np.arctan2(norm, qw)
            if  np.abs(angle) > 2.22e-15:
                x = q[1, 0] / norm
                y = q[2, 0] / norm
                z = q[3, 0] / norm
            else:
                angle = 0.0
                x = 0.0
                y = 0.0
                z = 1.0
            result = np.vstack((angle, x, y, z))
            return result
        elif isinstance(q, cs.MX):
            qw = q[0, 0]
            angle = cs.atan2(norm, qw)
            if  cs.fabs(angle) > 2.22e-15:
                x = q[1, 0] / norm
                y = q[2, 0] / norm
                z = q[3, 0] / norm
            else:
                angle = 0.0
                x = 0.0
                y = 0.0
                z = 1.0
            result = cs.vertcat(angle, x, y, z)
            return result
        elif isinstance(q, cs.SX):
            qw = q[0, 0]
            angle = cs.atan2(norm, qw)
            if  cs.fabs(angle) > 2.22e-15:
                x = q[1, 0] / norm
                y = q[2, 0] / norm
                z = q[3, 0] / norm
            else:
                angle = 0.0
                x = 0.0
                y = 0.0
                z = 1.0
            result = cs.vertcat(angle, x, y, z)
            return result
        else:
            raise TypeError("Internal problem with the definition of the Quaternion, it should be a np.array, cs.MX or cs.SX.")

    def ln(self):
        # Log mapping
        q = self.q
        angle_axis_aux = self.angle_axis
        if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            angle = angle_axis_aux[0, 0]
            x = angle_axis_aux[1, 0]
            y = angle_axis_aux[2, 0]
            z = angle_axis_aux[3, 0]
            result = np.vstack((0.0, (1/2)*angle*x, (1/2)*angle*y, (1/2)*angle*z))
            return Quaternion(q = result)
        elif isinstance(q, cs.MX):
            angle = angle_axis_aux[0, 0]
            x = angle_axis_aux[1, 0]
            y = angle_axis_aux[2, 0]
            z = angle_axis_aux[3, 0]
            result = cs.vertcat(0.0, (1/2)*angle*x, (1/2)*angle*y, (1/2)*angle*z)
            return Quaternion(q = result)
        elif isinstance(q, cs.SX):
            angle = angle_axis_aux[0, 0]
            x = angle_axis_aux[1, 0]
            y = angle_axis_aux[2, 0]
            z = angle_axis_aux[3, 0]
            result = cs.vertcat(0.0, (1/2)*angle*x, (1/2)*angle*y, (1/2)*angle*z)
            return Quaternion(q = result)
        else:
            raise TypeError("Internal problem with the definition of the Quaternion, it should be a np.array, cs.MX or cs.SX.")

    def vector_dot_product(self, q2):
        if isinstance(q2, Quaternion):
            q_rot = Quaternion.vector_dot(self.q, q2.q)
            return Quaternion(q = q_rot)
        else:
            raise TypeError("Vector Dot Product only defined for Quaternions")

    @staticmethod
    def vector_dot(p: Vector, q: Vector) -> Vector:
        # Funtion that defines the addition opperation
        if isinstance(p, np.ndarray) and isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            aux_1 = p * q
            q_product = aux_1
            return q_product
        elif isinstance(p, cs.MX) and isinstance(q, cs.MX):
            aux_1 = p * q
            q_product = aux_1
            return q_product
        elif isinstance(p, cs.SX) and isinstance(q, cs.SX):
            aux_1 = p * q
            q_product = aux_1
            return q_product
        else:
            raise TypeError("The elements of both quaternions should be of the same type.")