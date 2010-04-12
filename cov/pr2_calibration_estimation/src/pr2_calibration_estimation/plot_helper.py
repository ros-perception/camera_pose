
import matplotlib.pyplot as plt
import threading
import gtk.gdk

class graphicsThread(threading.Thread):
    def run(self):
        print "PlotHelper: Calling show()"
        plt.show()
        print "PlotHelper: Thread exiting"

def start():
    gtk.gdk.threads_init()
    plt.figure(1)
    graphicsThread().start()



#print "About to start thread"
#graphicsThread().start()
#print "Done starting thread"
#
#
#plt.plot([1,2,3],[4,6,5],'ro:')
#plt.draw()
#
#time.sleep(3)
#
#plt.clf()
#
#print "Starting 2nd plot"
##plt.figure(1)
#plt.plot([1,2,3],[4,7,6],'bo:')
#print "done with 2nd plot"
#plt.draw()
#
#
#print "Sleeping"
#time.sleep(1)
#print "Done sleeping"
#
