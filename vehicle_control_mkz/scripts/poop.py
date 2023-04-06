#!/usr/bin/env python
from collect_waypoints import Collect_Waypoints
import numpy as np 
class PoopHandle():
    def plotter(self):
        self.pathArray = Collect_Waypoints.path_array_handler()
        print( self.pathArray)
        '''
        self.pathArray = np.asarray(self.pathArray[1:-1])	#Ignore the first one
        np.savetxt(self.filename,self.pathArray,delimiter=',') 
        plt.figure()
        plt.scatter(self.pathArray[:,0],self.pathArray[:,1])
        plt.title("waypoints")
        plt.show()
        '''

def main(args=None):
    cw= PoopHandle()
    cw.plotter()

    


	
if __name__ == '__main__':
    main()