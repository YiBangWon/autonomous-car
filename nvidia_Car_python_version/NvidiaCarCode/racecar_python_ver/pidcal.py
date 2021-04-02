import rospy

class PidCal:
    error_sum = 0
    error_old = 0
    p = [0.0035, 0.000005, 0.005] # optimized kp,ki,kd
    dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd

    # setpoint = 295

    def __init__(self):
        print "init PidCal"
        self.x = 0
    # def cal_error(self, setpoint=288):
    def cal_error(self, setpoint=295):
    # def cal_error(self):
        # global setpoint
        # return self.setpoint - self.x
        return setpoint - self.x

    # twiddle is for optimize the kp,ki,kd
    # def twiddle(self):
    def twiddle(self, setpoint=295):
    # def twiddle(self, setpoint=288):
    # def twiddle(self, setpoint=318):
        # global setpoint
        best_err = self.cal_error()
        # threshold = 0.001
        #threshold = 1e-09
        threshold = 0.0000000000000000000000000000001

        # searching by move 1.1x to the target and if go more through the target comeback to -2x
        while sum(self.dp) > threshold:
            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error()

                if err < best_err:  # There was some improvement
                    best_err = err
                    self.dp[i] *= 1.1
                else:  # There was no improvement
                    self.p[i] -= 2*self.dp[i]  # Go into the other direction
                    err = self.cal_error()

                    if err < best_err:  # There was an improvement
                        best_err = err
                        self.dp[i] *= 1.05
                    else:  # There was no improvement
                        self.p[i] += self.dp[i]
                        # As there was no improvement, the step size in either
                        # direction, the step size might simply be too big.
                        self.dp[i] *= 0.95

        print(self.p)

    # setpoint is the center and the x_current is where the car is
    # width = 640, so 320 is the center but 318 is more accurate in real
    # def pid_control(self, x_current):
    def pid_control(self, x_current, setpoint=295):
    # def pid_control(self, x_current, setpoint=288):   # if x_location is left side of camera cencer
    # def pid_control(self, x_current, setpoint=318):     # then error is more than 0 so car goes left
        print "HHHHHHHHHHHHHHH"                         # if x_locarion is right side of camera center
        print x_current                                 # then error is less than 0 so car goes right
        self.x = int(x_current)                         # camera center is 318.

        # global setpoint

        # error = self.setpoint - x_current
        error = setpoint - x_current
        #todo
        # if x_current > 280 and x_current < 296:
        #     self.setpoint = 288
        # else:
        #     self.setpoint = 295

        # if error > 40:
        #     setpoint = 330
        #     error = (setpoint - x_current) * 1.2
        # elif error > 30:
        #     setpoint = 330
        #     error = setpoint - x_current
        # elif error > 20:

        # if error > 15:
        #     setpoint = 318
        #     error = setpoint - x_current
        # elif error > 10:
        #     setpoint = 300
        #     error = setpoint - x_current

        if error > 17:
            # setpoint = 318
            # error = error * 2.2
            error = error * 2.4
        if error < -17:
            # setpoint = 318
            # error = error * 2.2
            error = error * 2.4

        # rospy.loginfo("setpoint %f xcurrent %f, error: %f",setpoint, x_current, error)
        # rospy.loginfo("setpoint : %f", setpoint)

        # if error < 10:                     #if error is less than 10 = linear line
        #     setpoint = 295                 #make setpoint 285 because we dont need to go more left and we need to go more right to fit center
        #     error = setpoint - x_current   #calculate error again
        #
        # elif error < 20:
        #     setpoint = 310
        #     error = setpoint - x_current

        self.twiddle()                  # then calcualate pid

        # error = setpoint - x_current
        print error

        p1 = round(self.p[0] * error, 9)   #p[0] = p if error is plus then go to left.
                                          #else if p error is minus, then go to right
        self.error_sum += error
        i1 = round(self.p[1] * self.error_sum, 9)  #p[1] = i
        d1 = round(self.p[2] * (error -  self.error_old), 9)  #p[2] = d
        self.error_old = error
        pid = p1 + i1 + d1
        #print("p : " ,p)
        #print("i : " ,i)
        #print("d : " ,d)
        # rospy.loginfo("cur: %.6f,  p: %f, i: %f, d: %f,  pid: %f", x_current, p1, i1, d1, pid)

        return pid
