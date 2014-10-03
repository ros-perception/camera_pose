#!/usr/bin/python
    
# yliu Jul 27, 2011
  
import roslib; roslib.load_manifest('camera_pose_toolkits')
import rospy
import rosservice
import socket
import wx
from camera_pose_toolkits.srv import *
from std_msgs.msg import String
from calibration_msgs.msg import *


class MainWindow(wx.Frame):
    def __init__(self, output_ns):
	
        # ..._dispatcher_gui on ... ?
    
   
        title = '[' + output_ns + ']' + ' dispatcher gui on %s' % socket.gethostname()
        self.output_ns = output_ns

        wx.Frame.__init__(self, None, wx.ID_ANY, title, pos=(200, 200), size=(400, 100))

        # Create menu
        self.menubar = wx.MenuBar()
        self.filemenu = wx.Menu()
        self.filemenu.Append(wx.ID_EXIT, 'E&xit', 'Exit the program')
        wx.EVT_MENU(self, wx.ID_EXIT, self.on_exit)
        self.menubar.Append(self.filemenu, '&File')
        self.SetMenuBar(self.menubar)

        camera_ns_list = ['zero', 'one', 'two', 'three', 'four', 'five','six', 'seven', 'eight']
        self.combobox = wx.ComboBox(self, wx.ID_ANY, choices=camera_ns_list, style=wx.CB_READONLY)

        self.statusbar = self.CreateStatusBar()
        self.statusbar.SetStatusText('[' + self.output_ns + ']' +' now relays:  ')
        #self.sub = None 
        self.sub = rospy.Subscriber(self.output_ns+"/selected", String, self.cb_func)
        self.features_sub = rospy.Subscriber(self.output_ns+'/features', CalibrationPattern, self.features_cb)
        self.cb_in_sight = 0

        self.Bind(wx.EVT_COMBOBOX, self.on_combobox)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.combobox, 0, wx.EXPAND)
        self.SetSizer(self.sizer)


        # Refresh camera_ns list periodically
        self.timer1 = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self.on_timer1, self.timer1)
        self.timer1.Start(500, False)
	# default is oneShot=False, timer keeps restarting

        # Check every 1s if node has shutdown (and close GUI)
        self.timer2 = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self.on_timer2, self.timer2)
        self.timer2.Start(1000, False)
	
        self.selected_cam_ns = ""



    def cb_func(self, msg):
        self.selected_cam_ns = msg.data

    def features_cb(self, msg):
        self.cb_in_sight = msg.success

    def on_timer2(self, event):
        #print 'time out 2'
        if rospy.is_shutdown():
            self.Close(True)
            self.Refresh()

    def on_timer1(self, event):
        #print 'time out 1'
        #self.camera_ns_list = [  s[:-len('/image_rect')]  for s in rosservice.get_service_list() if s.endswith('/image_rect')] 
        topic_list=[ tn_tt[0]  for tn_tt in rospy.client.get_published_topics()]  #  [topic name, topic type]
        self.camera_ns_list = [ tn[:-len('/image_rect')] for tn in topic_list if tn.endswith('/image_rect')]
        self.camera_ns_list = [ ns for ns in self.camera_ns_list if not ( ns.startswith(self.output_ns) or ns.startswith('/'+self.output_ns) ) ]
        self.combobox.SetItems(self.camera_ns_list)

        for cam_ns in self.camera_ns_list:
            if cam_ns == self.selected_cam_ns:
                #wx.CallAfter(self.statusbar.SetStatusText,  'current camera: ' + msg.data)
                self.statusbar.SetStatusText('[' + self.output_ns + ']' + ' now relays:  ' + self.selected_cam_ns)
                break
        else: 
            self.statusbar.SetStatusText('[' + self.output_ns + ']' + ' now relays:  ')        
            #wx.CallAfter(self.statusbar.SetStatusText, 'current camera: ')
       
        #print self.combobox.GetValue()
        print self.camera_ns_list
        print self.output_ns
        print self.selected_cam_ns
        

    def on_timer(self, event):
        if rospy.is_shutdown():
            self.Close(True)
            self.Refresh()

    def on_exit(self, e):
        self.Close(True)
        self.Refresh()

    def on_error(self):
        self.Raise()

    def on_combobox(self, event):
        print 'combobox event'

        if self.cb_in_sight == 0 :
            print str(self.combobox.GetValue())
            #print rosservice.get_service_list()
            for s in rosservice.get_service_list(): 
                if s.endswith(self.output_ns+ '/switch'): #'camera_dispatcher/switch'
                    switch_cam = rospy.ServiceProxy(self.output_ns+ '/switch', Switch) # 'camera_dispatcher/switch' | camera_pose_toolkits.srv.Switch
                    resp = switch_cam ( str(self.combobox.GetValue()))
                    break
            else:
                wx.MessageBox('Service not ready. Try again later.', 'Info!')
        else:
            wx.MessageBox('Remove checkerboard from '+ self.output_ns +'\' field of view before switching camera', 'Info!') 
	
        #rospy.wait_for_service('camera_dispatcher/switch')
        
        
        


if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) <2:
        print 'Usage:  '
        exit(1)
    
    print argv[0]
    print argv[1]



    app = wx.PySimpleApp(clearSigInt=False)
    rospy.init_node('dispatcher_gui', anonymous=True)

    frame = MainWindow(argv[1])
    frame.Show()

    print 'dispatcher_gui started'

    try:
        app.MainLoop()
    except KeyboardInterrupt, e:
        pass
    print 'exiting'
