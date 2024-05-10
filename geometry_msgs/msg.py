class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


# @property
# def x(self):
#     return self.x

# @property
# def y(self):
#     return self.y

# @property
# def z(self):
#     return self.z


class Twist:
    def __init__(self, linear=Vector3(), angular=Vector3()):
        self.linear = linear
        self.angular = angular


# @property
# def linear(self) -> Vector3:
#     return self.linear

# @property
# def angular(self) -> Vector3:
#     return self.angular


class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

# @property
# def x(self):
#     return self.x
