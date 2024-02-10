import dataclasses
from typing import List
import casadi as cs
from dual_quaternion.type import Scalar, Vector
import numpy as np

@dataclasses.dataclass
class Quaternion():
    # Properties of the class
    q: Vector
    def __init__(self, q = None):
        if q is not None:
            # Check if the vairbale is a np.array
            if isinstance(q, np.ndarray):  # Use Vector directly without parentheses
                if q.shape[0] != 4:
                    raise ValueError("q must have exactly 4 elements.")
                q_aux = q.reshape((4, 1))
                self.q = q_aux

            # Check for casadi variables
            elif isinstance(q, cs.MX) or isinstance(q, cs.SX):
                if q.shape[0] != 4:
                    raise ValueError("q must have exactly 4 elements.")
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
        return Quaternion(q = Quaternion.product(self.q, q2.q))

    @staticmethod
    def product(p: Vector, q: Vector) -> Vector:
        if isinstance(p, np.ndarray) and isinstance(q, np.ndarray):  # Use Vector directly without parentheses
            aux_1 = p[0, 0] * q[0, 0] - np.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ np.cross(p[1:4, 0], q[1:4, 0])
            q_product = np.vstack((aux_1, aux_2[0], aux_2[1], aux_2[2]))
            q_product = q_product.reshape((4, 1))

        elif isinstance(p, cs.MX) and isinstance(q, cs.MX):
            aux_1 = p[0, 0] * q[0, 0] - np.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ np.cross(p[1:4, 0], q[1:4, 0])
            q_product = cs.vertcat(aux_1, aux_2)

        elif isinstance(p, cs.SX) and isinstance(q, cs.SX):
            aux_1 = p[0, 0] * q[0, 0] - cs.dot(p[1:4, 0], q[1:4, 0])
            aux_2 = p[0, 0] * q[1:4, 0] + q[0, 0]* p[1:4, 0]+ cs.cross(p[1:4, 0], q[1:4, 0])
            q_product = cs.vertcat(aux_1, aux_2)
        else:
            raise TypeError("The elements of both quaternions should be of the same type.")
            
        return q_product