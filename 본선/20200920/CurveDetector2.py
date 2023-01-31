import time


class CurveDetector:
    def __init__(self):
        self.curve_count = 0
        self.time_old = 0
        self.pid_list = [0.0 for i in range(30)]
	self.pid_list_sum = 0

    def check_time(self):
        if time.time() - self.time_old < 1.5:
            return False
        else:
            return True

    def list_update(self, pid):
        self.pid_list.pop(0)
        self.pid_list.append(pid)
	#self.pid_list_sum = sum(self.pid_list)

    def count_curve(self):
        self.pid_list_sum = sum(self.pid_list)
        if self.check_time(): 
            if self.pid_list_sum > 2:
                self.time_old = time.time()
                self.curve_count += 1
