import threading

from .tk_litter_world import TkLitterWorld

class TkLitterWorldThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        self.tk_litter_world = TkLitterWorld()
        self.tk_litter_world.mainloop()  

    def update_counter(self, num):
        self.tk_litter_world.upd_num(num)