import math


#----------------------------------------------------------#

class roc_curve_data:
    def __init__(self):
        self.tp = 0     # True Positives
        self.fp = 0      # False Positives
        self.tn = 0     # True Negatives
        self.fn = 0     # False Negatives

    def add_tp(self):
        self.tp = self.tp + 1

    def add_fp(self):
        self.fp = self.fp + 1

    def add_tn(self):
        self.tn = self.tn + 1

    def add_fn(self):
        self.fn = self.fn + 1

#----------------------------------------------------------#

class trajectory_data:
    def __init__(self):
        self.x = []
        self.y = []
        self.theta = []
        self.time = []

    def add_new_position(self, x, y, theta, time):
        self.x.append(x)
        self.y.append(y)
        self.theta.append(theta)
        self.time.append(time)

#----------------------------------------------------------#

class position_data:
    def __init__(self):
        self.x = []
        self.y = []
        self.time = []
        self.velocity = velocity_data()

    def add_value(self, x, y, time):
        self.x.append(x)
        self.y.append(y)
        self.time.append(time)

#----------------------------------------------------------#

class rotation_data:
    def __init__(self):
        self.theta = []
        self.time = []
        self.velocity = velocity_data()

    def add_value(self, theta, time):
        self.theta.append(theta)
        self.time.append(time)

    def latest(self):
        if len(self.theta) > 0:
            return self.theta[-1]
        else:
            return 0

# ----------------------------------------------------------#

class distance_data:
    def __init__(self):
        self.dist = []
        self.time = []

    def add_value(self, theta, time):
        self.dist.append(theta)
        self.time.append(time)

    def latest(self):
        if len(self.dist) > 0:
            return self.dist[-1]
        else:
            return 0

# ----------------------------------------------------------#

class difference_data:
    def __init__(self):
        self.dif = []
        self.time = []

    def add_value(self, dif, time):
        self.dif.append(dif)
        self.time.append(time)

    def latest(self):
        if len(self.dif) > 0:
            return self.dif[-1]
        else:
            return 0


# ----------------------------------------------------------#

class velocity_data:
   def __init__(self):
       self.velocity = []
       self.time = []

   def add_value(self, velocity, time):
       self.velocity.append(velocity)
       self.time.append(time)

   def latest(self):
       if len(self.velocity) > 0:
           return self.velocity[-1]
       else:
           return 0

#----------------------------------------------------------#

class measurements:
    def __init__(self):
        self.position = position_data()
        self.distance = distance_data()
        self.rotation = rotation_data()
        self.trajectory = trajectory_data()

    def calculate_distance(self):
        dist = math.sqrt(self.position.x[-1] * self.position.x[-1] +
                         self.position.y[-1] * self.position.y[-1])
        return dist

#----------------------------------------------------------#

class wrong_measurements:
    def  __init__(self):
        self.values = []
        self.time = []

    def add_new(self, new_wrong, current_time):
        self.values.append(new_wrong)
        self.time.append(current_time)

#----------------------------------------------------------#

class true_slip:
    def __init__(self, true_position, true_orientation):
        self.true_position = true_position
        self.true_orientation = true_orientation
        self.wrong_position = wrong_measurements()
        self.wrong_orientation = wrong_measurements()

        self.true_graph_position =  wrong_measurements()
        self.true_graph_orientation = wrong_measurements()


#----------------------------------------------------------#

class time:
    def __init__(self):
        self.secs = 0
        self.nsecs = 0